"""OpenAI Responses adapter with explicitly pinned investigation profiles."""

from dataclasses import dataclass, field
import os
from pathlib import Path

from dotenv import dotenv_values
from openai import OpenAI


MODEL = "gpt-6-astra"
REASONING_EFFORT = "medium"
API_BASE = "https://api.openai.com/v1"
DEFAULT_PROFILE = "astra-medium"


@dataclass(frozen=True)
class ModelProfile:
    name: str
    model: str
    reasoning_effort: str
    environment_prefix: str


PROFILES = {
    "astra-medium": ModelProfile("astra-medium", MODEL, REASONING_EFFORT, "ASTRA"),
    "sol-high": ModelProfile("sol-high", "gpt-5.6-sol", "high", "SOL"),
}


def get_profile(name=DEFAULT_PROFILE):
    if not isinstance(name, str) or name not in PROFILES:
        raise ValueError("Select the astra-medium or sol-high investigation profile")
    return PROFILES[name]


@dataclass(frozen=True)
class Settings:
    api_key: str = field(repr=False)
    model: str | None = None
    reasoning_effort: str | None = None
    profile: str = DEFAULT_PROFILE

    def __post_init__(self):
        if not self.api_key or not self.api_key.strip():
            raise ValueError("Configure OPENAI_API_KEY in .env or the backend environment")
        selected = get_profile(self.profile)
        if self.model is None:
            object.__setattr__(self, "model", selected.model)
        if self.reasoning_effort is None:
            object.__setattr__(self, "reasoning_effort", selected.reasoning_effort)
        if self.model != selected.model or self.reasoning_effort != selected.reasoning_effort:
            raise ValueError(f"The {selected.name} profile requires {selected.model} with "
                             f"{selected.reasoning_effort} reasoning; no fallback model is used")

    @classmethod
    def load(cls, env_path: Path = Path(".env"), *, profile=DEFAULT_PROFILE):
        selected = get_profile(profile)
        values = dotenv_values(env_path, interpolate=False) if env_path.is_file() else {}
        # The explicitly selected local file takes precedence over the shell.
        # Never export credentials into the candidate worker environment.
        def value(name, default=""):
            return values.get(name) or os.environ.get(name) or default
        return cls(api_key=value("OPENAI_API_KEY").strip(),
                   model=value(f"{selected.environment_prefix}_MODEL", selected.model).strip(),
                   reasoning_effort=value(f"{selected.environment_prefix}_REASONING_EFFORT", selected.reasoning_effort).strip(),
                   profile=selected.name)

    def client(self):
        return OpenAI(api_key=self.api_key, base_url=API_BASE, timeout=180, max_retries=0)


def request_response(client, conversation, tools, *, max_output_tokens=8192, final=False, profile=DEFAULT_PROFILE):
    """Every request explicitly sends the same model and reasoning effort."""
    selected = get_profile(profile)
    return client.responses.create(
        model=selected.model,
        reasoning={"effort": selected.reasoning_effort, "summary": "auto"},
        input=conversation,
        tools=tools,
        tool_choice="none" if final else "auto",
        parallel_tool_calls=False,
        max_output_tokens=max_output_tokens,
        store=False,
        include=["reasoning.encrypted_content"],
    )
