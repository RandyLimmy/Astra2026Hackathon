"""Fixed platform worker runtime, copied alone beside the editable actuator.py.

The host imports these pure validation functions and repeats output validation
outside the child. Importing this module does not load candidate code or set
process limits; those operations happen only in main().
"""

import json
import math
import os
import sys
import types


MAX_BYTES = 65_536
MAX_OBSERVATION_BYTES = 16_384
MAX_STRING_CHARS = 4000


def valid_number(value, *, lower=None, upper=1e15):
    if type(value) not in (int, float) or not math.isfinite(value) or abs(value) > upper:
        raise ValueError("invalid number")
    if lower is not None and value < lower:
        raise ValueError("invalid number")


def valid_json(value, *, strings=False):
    budget = 2048

    def visit(item, depth=0):
        nonlocal budget
        budget -= 1
        if depth > 8 or budget < 0:
            raise ValueError("invalid JSON size")
        if item is None or type(item) is bool:
            return
        if type(item) in (int, float):
            valid_number(item)
            return
        if strings and type(item) is str and len(item) <= MAX_STRING_CHARS:
            return
        if type(item) is list:
            for entry in item:
                visit(entry, depth + 1)
            return
        if type(item) is dict:
            for key, entry in item.items():
                if type(key) is not str or not key.isidentifier() or len(key) > 64:
                    raise ValueError("invalid JSON key")
                visit(entry, depth + 1)
            return
        raise ValueError("invalid JSON value")

    if type(value) is not dict:
        raise ValueError("expected dictionary")
    visit(value)


def valid_state(state):
    valid_json(state)


def valid_observation(observation):
    valid_json(observation, strings=True)
    if len(json.dumps(observation, allow_nan=False, separators=(",", ":")).encode()) > MAX_OBSERVATION_BYTES:
        raise ValueError("observation too large")


def valid_parameters(parameters):
    if type(parameters) is not dict or len(parameters) > 64:
        raise ValueError("invalid parameters")
    for key, value in parameters.items():
        if type(key) is not str or not key.isidentifier() or len(key) > 64:
            raise ValueError("invalid parameter key")
        valid_number(value, upper=1e9)


def candidate_source_line(error):
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


def main():
    import resource

    resource.setrlimit(resource.RLIMIT_CPU, (120, 120))
    resource.setrlimit(resource.RLIMIT_FSIZE, (0, 0))
    resource.setrlimit(resource.RLIMIT_NOFILE, (32, 32))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    incoming = sys.stdin.buffer
    outgoing = os.fdopen(os.dup(sys.stdout.fileno()), "wb", buffering=0)
    null_fd = os.open(os.devnull, os.O_WRONLY)
    os.dup2(null_fd, 1)
    os.dup2(null_fd, 2)
    os.close(null_fd)
    module = types.ModuleType("actuator")
    module.__file__ = "actuator.py"
    ready = False
    state = None

    while True:
        line = incoming.readline(MAX_BYTES + 1)
        if not line:
            return
        if len(line) > MAX_BYTES or not line.endswith(b"\n"):
            return
        identifier = None
        reason = "runtime_error"
        try:
            request = json.loads(line)
            identifier = request["id"]
            if type(identifier) is not int or request.get("protocol") != "platform_v1":
                raise ValueError("invalid protocol")
            operation = request["operation"]
            if operation not in ("reset", "inspect", "parameters", "advance"):
                raise ValueError("invalid operation")
            if operation == "advance":
                valid_observation(request["observation"])
                valid_number(request["dt"], lower=0, upper=60)
            reason = "candidate_error"
            if not ready:
                source = open("actuator.py", encoding="utf-8").read(MAX_BYTES + 1)
                exec(compile(source, "actuator.py", "exec"), module.__dict__)
                for name in ("init_state", "predict_parameters", "advance_state"):
                    if not callable(getattr(module, name, None)):
                        raise TypeError("missing function")
                ready = True
            if operation == "reset":
                state = module.init_state()
                value = state
            elif operation == "parameters":
                # Both the detached query input and retained original aliases
                # must remain unchanged; parameters() cannot advance history.
                snapshot = json.dumps(state, allow_nan=False, sort_keys=True)
                query_state = json.loads(snapshot)
                value = module.predict_parameters(query_state)
                reason = "parameter_state_mutation"
                valid_state(query_state)
                valid_state(state)
                if (json.dumps(query_state, allow_nan=False, sort_keys=True) != snapshot
                        or json.dumps(state, allow_nan=False, sort_keys=True) != snapshot):
                    raise ValueError("query changed state")
                reason = "invalid_parameters"
                valid_parameters(value)
            elif operation == "advance":
                state = module.advance_state(state, request["observation"], request["dt"])
                value = state
            else:
                value = state
            reason = "invalid_state"
            valid_state(state)
            result = {"id": identifier, "ok": True, "value": value}
        except BaseException as error:
            result = {"id": identifier, "ok": False, "reason": reason}
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
