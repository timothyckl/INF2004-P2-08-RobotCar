#!/usr/bin/env python3
"""Check the automatically enforceable BARR-C rules for subsystem 5."""

from pathlib import Path
import re
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODULE_GROUPS = (
    (
        PROJECT_ROOT / "app_program/subsystem5/include",
        PROJECT_ROOT / "app_program/subsystem5/src",
    ),
    (
        PROJECT_ROOT / "testing/subsystem5/unit_tests/include",
        PROJECT_ROOT / "testing/subsystem5/unit_tests/src",
    ),
)
FORBIDDEN_TOKENS = {
    "abort",
    "auto",
    "continue",
    "exit",
    "goto",
    "long",
    "longjmp",
    "register",
    "setjmp",
    "short",
}
STANDARD_HEADER_NAMES = {
    "assert",
    "complex",
    "ctype",
    "errno",
    "fenv",
    "float",
    "inttypes",
    "iso646",
    "limits",
    "locale",
    "math",
    "setjmp",
    "signal",
    "stdalign",
    "stdarg",
    "stdatomic",
    "stdbool",
    "stddef",
    "stdint",
    "stdio",
    "stdlib",
    "stdnoreturn",
    "string",
    "tgmath",
    "threads",
    "time",
    "uchar",
    "wchar",
    "wctype",
}
END_MARKER = "/* End of file. */\n"


def strip_comments_and_literals(source: str) -> str:
    """Replace comments and literals while retaining line boundaries."""
    result = []
    index = 0
    state = "code"

    while index < len(source):
        char = source[index]
        next_char = source[index + 1] if index + 1 < len(source) else ""

        if state == "code":
            if (char == "/") and (next_char == "*"):
                result.extend("  ")
                index += 2
                state = "block_comment"
            elif (char == "/") and (next_char == "/"):
                result.extend("  ")
                index += 2
                state = "line_comment"
            elif char == '"':
                result.append(" ")
                index += 1
                state = "string"
            elif char == "'":
                result.append(" ")
                index += 1
                state = "character"
            else:
                result.append(char)
                index += 1
        elif state == "block_comment":
            if (char == "*") and (next_char == "/"):
                result.extend("  ")
                index += 2
                state = "code"
            else:
                result.append("\n" if char == "\n" else " ")
                index += 1
        elif state == "line_comment":
            result.append("\n" if char == "\n" else " ")
            index += 1
            if char == "\n":
                state = "code"
        else:
            if char == "\\":
                result.append(" ")
                index += 1
                if index < len(source):
                    escaped = source[index]
                    result.append("\n" if escaped == "\n" else " ")
                    index += 1
            elif ((state == "string") and (char == '"')) or (
                (state == "character") and (char == "'")
            ):
                result.append(" ")
                index += 1
                state = "code"
            else:
                result.append("\n" if char == "\n" else " ")
                index += 1

    return "".join(result)


def check_module_groups(errors: list[str]) -> list[Path]:
    files = []
    module_locations: dict[str, Path] = {}

    for include_dir, source_dir in MODULE_GROUPS:
        headers = sorted(include_dir.glob("*.h"))
        sources = sorted(source_dir.glob("*.c"))
        files.extend(headers)
        files.extend(sources)

        header_stems = {path.stem for path in headers}
        source_stems = {path.stem for path in sources}

        for stem in sorted(header_stems - source_stems):
            errors.append(f"{include_dir}: no source file for {stem}.h")

        for stem in sorted(source_stems - header_stems):
            errors.append(f"{source_dir}: no header file for {stem}.c")

        for path in headers + sources:
            stem = path.stem

            if not re.fullmatch(r"[a-z0-9_]+", stem):
                errors.append(f"{path}: module name is not lower snake case")

            if stem in STANDARD_HEADER_NAMES:
                errors.append(f"{path}: module name conflicts with a standard header")

            module_locations.setdefault(stem, path)

    prefixes: dict[str, str] = {}

    for stem, path in module_locations.items():
        prefix = stem[:8]
        previous = prefixes.get(prefix)

        if (previous is not None) and (previous != stem):
            errors.append(
                f"{path}: first eight characters collide with module {previous}"
            )
        else:
            prefixes[prefix] = stem

    return files


def check_file(path: Path, errors: list[str]) -> None:
    relative_path = path.relative_to(PROJECT_ROOT)
    data = path.read_bytes()

    if b"\t" in data:
        errors.append(f"{relative_path}: contains a tab character")

    if b"\r" in data:
        errors.append(f"{relative_path}: contains a carriage return")

    invalid_controls = {
        byte for byte in data if (byte < 0x20) and (byte not in {0x0A, 0x0C})
    }

    if invalid_controls:
        controls = ", ".join(
            f"0x{byte:02X}" for byte in sorted(invalid_controls)
        )
        errors.append(f"{relative_path}: contains control characters {controls}")

    try:
        source = data.decode("utf-8")
    except UnicodeDecodeError as error:
        errors.append(f"{relative_path}: is not valid UTF-8: {error}")
        return

    if not source.endswith(END_MARKER):
        errors.append(f"{relative_path}: missing the end-of-file marker or final LF")

    for line_number, line in enumerate(source.splitlines(), 1):
        if len(line) > 80:
            errors.append(
                f"{relative_path}:{line_number}: line has {len(line)} characters"
            )

    stripped = strip_comments_and_literals(source)

    for match in re.finditer(r"\b[A-Za-z_]\w*\b", stripped):
        identifier = match.group(0)
        line_number = stripped.count("\n", 0, match.start()) + 1

        if identifier in FORBIDDEN_TOKENS:
            errors.append(
                f"{relative_path}:{line_number}: forbidden token '{identifier}'"
            )

        if len(identifier) > 31:
            errors.append(
                f"{relative_path}:{line_number}: identifier exceeds 31 characters"
            )

    for line_number, line in enumerate(stripped.splitlines(), 1):
        match = re.match(r"^#\s*define\s+([A-Za-z_]\w*)", line)

        if (match is not None) and (match.group(1) != match.group(1).upper()):
            errors.append(
                f"{relative_path}:{line_number}: macro name contains lowercase"
            )


def main() -> int:
    """Run all checks and return a process status."""
    errors: list[str] = []
    files = check_module_groups(errors)

    for path in files:
        check_file(path, errors)

    if errors:
        for error in errors:
            print(f"BARR-C: {error}", file=sys.stderr)

        return 1

    print(f"BARR-C check passed: {len(files)} subsystem 5 files.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
