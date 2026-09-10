"""Bounded stateful platform predictions behind the existing OS boundary."""

from typing import cast

from .client import ActuatorWorker, WorkerError, _finite
from .platform_runtime import valid_observation, valid_parameters, valid_state


class PlatformModelWorker(ActuatorWorker):
    """Persistent editable predictor with explicit public-observation updates.

    Parameter names and their physical ranges are checked by the host platform
    adapter. This boundary validates finite, bounded JSON and read-only queries.
    """

    _protocol = "platform_v1"
    _runtime_filename = "platform_runtime.py"
    _extra_safe_reasons = {
        "candidate_error": "Platform model computation failed.",
        "runtime_error": "Platform model runtime rejected the request.",
        "invalid_state": "Platform model returned invalid numeric state.",
        "invalid_parameters": "Platform model returned invalid parameters.",
        "parameter_state_mutation": "predict_parameters must not modify model state; use advance_state.",
    }

    def _validate_response(self, operation: str, value: object) -> object:
        if operation not in {"parameters", "reset", "inspect", "advance"}:
            raise WorkerError("Unknown platform model operation.")
        try:
            (valid_parameters if operation == "parameters" else valid_state)(value)
        except (ValueError, TypeError, OverflowError, RecursionError):
            label = "parameters" if operation == "parameters" else "numeric state"
            raise WorkerError(f"Platform model returned invalid {label}.") from None
        return value

    def parameters(self) -> dict[str, float]:
        return cast(dict[str, float], self._request("parameters"))

    def advance(self, observation: dict, dt: float) -> dict:  # type: ignore[override]
        """Advance using one public observation; dt=0 initializes trial context."""
        try:
            valid_observation(observation)
        except (ValueError, TypeError, OverflowError, RecursionError):
            raise WorkerError("Public observation must be bounded finite JSON.") from None
        return cast(dict, self._request("advance", observation=observation,
                                       dt=_finite(dt, "Timestep", lower=0, upper=60)))

    def force(self, brake: float, velocity: float) -> float:
        raise WorkerError("Platform models use parameters.")
