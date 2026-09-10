"""OpenAI Responses adapter fixed to the user's requested Astra/medium pilot."""

from dataclasses import dataclass, field
import os
from pathlib import Path

from dotenv import dotenv_values
from openai import OpenAI


MODEL = "gpt-6-astra"
REASONING_EFFORT = "medium"
API_BASE = "https://api.openai.com/v1"


@dataclass(frozen=True)
class Settings:
    api_key: str = field(repr=False)
    model: str = MODEL
    reasoning_effort: str = REASONING_EFFORT

    def __post_init__(self):
        if not self.api_key or not self.api_key.strip():
            raise ValueError("Configure OPENAI_API_KEY in .env or the backend environment")
        if self.model != MODEL or self.reasoning_effort != REASONING_EFFORT:
            raise ValueError("This pilot requires gpt-6-astra with medium reasoning; no fallback model is used")

    @classmethod
    def load(cls, env_path: Path = Path(".env")):
        values = dotenv_values(env_path, interpolate=False) if env_path.is_file() else {}
        # The explicitly selected local file takes precedence over the shell.
        # Never export credentials into the candidate worker environment.
        def value(name, default=""):
            return values.get(name) or os.environ.get(name) or default
        return cls(api_key=value("OPENAI_API_KEY").strip(),
                   model=value("ASTRA_MODEL", MODEL).strip(),
                   reasoning_effort=value("ASTRA_REASONING_EFFORT", REASONING_EFFORT).strip())

    def client(self):
        return OpenAI(api_key=self.api_key, base_url=API_BASE, timeout=180, max_retries=0)


def request_response(client, conversation, tools, *, max_output_tokens=8192, final=False):
    """Every request explicitly sends the same model and reasoning effort."""
    return client.responses.create(
        model=MODEL,
        reasoning={"effort": REASONING_EFFORT, "summary": "auto"},
        input=conversation,
        tools=tools,
        tool_choice="none" if final else "auto",
        parallel_tool_calls=False,
        max_output_tokens=max_output_tokens,
        store=False,
        include=["reasoning.encrypted_content"],
    )
