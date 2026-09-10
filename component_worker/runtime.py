"""Fixed worker runtime. Copied alone with actuator.py into a private directory."""

import json
import math
import os
import resource
import sys
import types


MAX_BYTES = 65_536


def limits():
    # CPU is cumulative across this persistent worker, wall time is per request
    # in the host. macOS rejects RLIMIT_AS/DATA/RSS reductions, so the host also
    # runs a resident-memory watchdog; its sampling limitation is documented.
    resource.setrlimit(resource.RLIMIT_CPU, (120, 120))
    resource.setrlimit(resource.RLIMIT_FSIZE, (0, 0))
    resource.setrlimit(resource.RLIMIT_NOFILE, (32, 32))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def valid_state(state):
    budget = 2048

    def visit(value, depth=0):
        nonlocal budget
        budget -= 1
        if depth > 8 or budget < 0:
            raise ValueError("invalid state")
        if value is None or type(value) is bool:
            return
        if type(value) in (float, int):
            if not math.isfinite(value) or abs(value) > 1e15:
                raise ValueError("invalid state")
            return
        if type(value) is list:
            for entry in value:
                visit(entry, depth + 1)
            return
        if type(value) is dict:
            for key, entry in value.items():
                if type(key) is not str or not key.isidentifier() or len(key) > 64:
                    raise ValueError("invalid state")
                visit(entry, depth + 1)
            return
        raise ValueError("invalid state")

    if type(state) is not dict:
        raise ValueError("invalid state")
    visit(state)


def candidate_source_line(error):
    """Return only an own-source line number, never traceback text or paths."""
    line = None
    if isinstance(error, SyntaxError) and error.filename == "actuator.py":
        line = error.lineno
    traceback = error.__traceback__
    for _ in range(128):
        if traceback is None:
            break
        if traceback.tb_frame.f_code.co_filename == "actuator.py":
            line = traceback.tb_lineno
        traceback = traceback.tb_next
    return line if type(line) is int and 1 <= line <= MAX_BYTES else None


def valid_number(value, *, lower=None, upper=1e9):
    if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > upper:
        raise ValueError("invalid number")
    if lower is not None and value < lower:
        raise ValueError("invalid number")


def valid_vector(values, *, lower=None, upper=1e9):
    if type(values) is not list or len(values) != 4:
        raise ValueError("invalid vector")
    for value in values:
        valid_number(value, lower=lower, upper=upper)


def main():
    limits()
    # Candidate print/log calls cannot fill an unread pipe or corrupt messages.
    incoming = sys.stdin.buffer
    outgoing = os.fdopen(os.dup(sys.stdout.fileno()), "wb", buffering=0)
    null_fd = os.open(os.devnull, os.O_WRONLY)
    os.dup2(null_fd, 1)
    os.dup2(null_fd, 2)
    os.close(null_fd)
    module = types.ModuleType("actuator")
    module.__file__ = "actuator.py"
    state = None
    ready = False
    protocol = None
    while True:
        line = incoming.readline(MAX_BYTES + 1)
        if not line:
            return
        if len(line) > MAX_BYTES or not line.endswith(b"\n"):
            return
        identifier = None
        try:
            request = json.loads(line)
            identifier = request["id"]
            operation = request["operation"]
            if protocol is None:
                protocol = request.get("protocol")
                if protocol not in ("scalar_v1", "wheel_v2"):
                    raise ValueError("unknown component protocol")
            elif request.get("protocol") != protocol:
                raise ValueError("component protocol cannot change")
            wheel = protocol == "wheel_v2"
            if wheel and operation in ("torque_limits", "advance"):
                valid_number(request["brake"], lower=0, upper=1)
                valid_vector(request["omega"], upper=1e6)
                if operation == "advance":
                    valid_vector(request["applied_torque"])
                    valid_number(request["dt"], lower=1e-9, upper=60)
            if not ready:
                source = open("actuator.py", encoding="utf-8").read(MAX_BYTES + 1)
                exec(compile(source, "actuator.py", "exec"), module.__dict__)
                functions = ("init_state", "compute_brake_torque_limits", "advance_state", "on_trial_reset") if wheel else (
                    "init_state", "compute_force", "advance_state")
                for name in functions:
                    if not callable(getattr(module, name, None)):
                        raise TypeError("missing function")
                ready = True
            if operation == "reset":
                state = module.init_state()
                value = state
            elif (operation == "force" and not wheel) or (operation == "torque_limits" and wheel):
                # A force query cannot advance history. Give it a detached copy
                # and check both copies: candidate globals may retain a reference
                # to the original object returned by init_state/advance_state.
                snapshot = json.dumps(state, allow_nan=False, sort_keys=True)
                force_state = json.loads(snapshot)
                value = (module.compute_brake_torque_limits(force_state, request["brake"], request["omega"])
                         if wheel else module.compute_force(force_state, request["brake"], request["velocity"]))
                try:
                    valid_state(force_state)
                    valid_state(state)
                    unchanged = (json.dumps(force_state, allow_nan=False, sort_keys=True) == snapshot
                                 and json.dumps(state, allow_nan=False, sort_keys=True) == snapshot)
                except (ValueError, TypeError, OverflowError):
                    unchanged = False
                if not unchanged:
                    reason = "wheel_state_mutation" if wheel else "state_mutation"
                    outgoing.write(json.dumps({"id": identifier, "ok": False, "reason": reason}).encode() + b"\n")
                    return
                try:
                    if wheel:
                        valid_vector(value, lower=0)
                    else:
                        valid_number(value, lower=0)
                except (ValueError, TypeError, OverflowError):
                    reason = "invalid_torque" if wheel else "invalid_force"
                    outgoing.write(json.dumps({"id": identifier, "ok": False, "reason": reason}).encode() + b"\n")
                    return
            elif operation == "advance":
                state = (module.advance_state(state, request["brake"], request["omega"], request["applied_torque"], request["dt"])
                         if wheel else module.advance_state(state, request["brake"], request["velocity"], request["applied_force"], request["dt"]))
                value = state
            elif operation == "reposition" and wheel:
                state = module.on_trial_reset(state)
                value = state
            elif operation == "inspect":
                value = state
            else:
                raise ValueError("invalid operation")
            try:
                valid_state(state)
            except (ValueError, TypeError, OverflowError):
                outgoing.write(json.dumps({"id": identifier, "ok": False, "reason": "invalid_state"}).encode() + b"\n")
                return
            result = {"id": identifier, "ok": True, "value": value}
        except BaseException as error:
            # Errors may include host paths or deliberate disclosure attempts.
            # The host further constrains these fixed reason codes.
            result = {"id": identifier, "ok": False, "reason": "candidate_error"}
            try:
                source_line = candidate_source_line(error)
                if source_line is not None:
                    result["source_line"] = source_line
            except BaseException:
                pass
        try:
            encoded = json.dumps(result, allow_nan=False, separators=(",", ":")).encode() + b"\n"
            if len(encoded) > MAX_BYTES:
                return
            outgoing.write(encoded)
        except BaseException:
            return
        if not result["ok"]:
            return


if __name__ == "__main__":
    main()
