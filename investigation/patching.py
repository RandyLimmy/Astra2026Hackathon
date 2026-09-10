"""Apply a bounded, single-file unified diff without invoking external tools."""

import re


MAX_SOURCE_BYTES = 65_536
MAX_DIFF_BYTES = 48_000


class PatchError(ValueError):
    """A fixed, neutral patch error suitable for a tool response."""


def apply_actuator_diff(source: str, diff: str) -> str:
    if not isinstance(diff, str) or not diff or len(diff.encode("utf-8")) > MAX_DIFF_BYTES:
        raise PatchError("Patch must be a nonempty unified diff of at most 48000 bytes.")
    if "\r" in diff or "\x00" in diff:
        raise PatchError("Patch must use ordinary UTF-8 text with Unix newlines.")
    lines = diff.splitlines(keepends=True)
    index = 0
    if lines[0].startswith("diff --git"):
        if lines[0].rstrip("\n") != "diff --git a/actuator.py b/actuator.py":
            raise PatchError("Only actuator.py may be patched.")
        index += 1
        if index < len(lines) and lines[index].startswith("index "):
            if not re.fullmatch(r"index [0-9a-f]{1,64}\.\.[0-9a-f]{1,64}(?: 100644)?\n?", lines[index]):
                raise PatchError("File modes and non-text patch metadata are not supported.")
            index += 1
    if index + 1 >= len(lines):
        raise PatchError("Patch is missing its file headers.")
    if lines[index].rstrip("\n") not in ("--- actuator.py", "--- a/actuator.py") or lines[index + 1].rstrip("\n") not in ("+++ actuator.py", "+++ b/actuator.py"):
        raise PatchError("Only the existing actuator.py may be patched.")
    index += 2
    original = source.splitlines(keepends=True)
    output = []
    cursor = 0
    hunks = 0
    while index < len(lines):
        match = re.fullmatch(r"@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@(?: [^\n]*)?\n?", lines[index])
        if not match:
            raise PatchError("Malformed unified-diff hunk or an additional file.")
        old_start, old_count, new_start, new_count = (
            int(match[1]), int(match[2] or 1), int(match[3]), int(match[4] or 1))
        start = old_start - 1 if old_count else old_start
        if start < cursor or start > len(original) or (old_count and old_start == 0):
            raise PatchError("Patch hunks must address existing lines in increasing order.")
        output.extend(original[cursor:start])
        if new_start != len(output) + (1 if new_count else 0):
            raise PatchError("Patch new-line positions do not match its preceding hunks.")
        index += 1
        operations = []
        while index < len(lines) and not lines[index].startswith("@@ "):
            line = lines[index]
            if line.rstrip("\n") == "\\ No newline at end of file":
                if not operations or not operations[-1][1].endswith("\n"):
                    raise PatchError("Invalid no-newline marker.")
                operations[-1] = (operations[-1][0], operations[-1][1][:-1])
            elif line and line[0] in " +-" and not line.startswith(("--- ", "+++ ")):
                operations.append((line[0], line[1:]))
            else:
                raise PatchError("Patch contains an unsupported operation or another file.")
            index += 1
        if sum(kind != "+" for kind, _ in operations) != old_count or sum(kind != "-" for kind, _ in operations) != new_count:
            raise PatchError("Patch hunk line counts do not match its header.")
        cursor = start
        for kind, content in operations:
            if kind != "+":
                if cursor >= len(original) or original[cursor] != content:
                    raise PatchError("Patch context does not match the current actuator source.")
                cursor += 1
            if kind != "-":
                output.append(content)
        hunks += 1
    if not hunks:
        raise PatchError("Patch must contain at least one hunk.")
    output.extend(original[cursor:])
    updated = "".join(output)
    if updated == source:
        raise PatchError("Patch does not change the actuator source.")
    if len(updated.encode("utf-8")) > MAX_SOURCE_BYTES:
        raise PatchError("Patched source exceeds 65536 bytes.")
    return updated
