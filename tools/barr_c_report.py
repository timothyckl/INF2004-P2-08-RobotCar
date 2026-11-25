#!/usr/bin/env python3
"""Report the BARR-C:2018 rules that a script can check, for every file in
app_program/ (production and test code).

    python3 tools/barr_c_report.py            from the repository root

Report only: it always exits 0. Subsystem 5 has its own enforcing check
(tools/check_subsystem5_barr_c.py, run by the build). The other rules
(names, comments on casts, function design) need code review; see
docs/barr_c_compliance.md. Comments and string literals are removed before
the token checks so that words inside them are not counted.
"""
from collections import Counter, defaultdict
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "app_program"


def strip(source: str) -> str:
    """Blank out comments and string/char literals, keeping line breaks."""
    out, i, n = [], 0, len(source)
    while i < n:
        two = source[i:i + 2]
        if two == "/*":
            end = source.find("*/", i + 2)
            end = n if end < 0 else end + 2
            out.append(re.sub(r"[^\n]", " ", source[i:end]))
            i = end
        elif two == "//":
            end = source.find("\n", i)
            end = n if end < 0 else end
            out.append(" " * (end - i))
            i = end
        elif source[i] in "\"'":
            quote, j = source[i], i + 1
            while j < n and source[j] != quote:
                j += 2 if source[j] == "\\" else 1
            out.append(" " * (min(j + 1, n) - i))
            i = j + 1
        else:
            out.append(source[i])
            i += 1
    return "".join(out)


RANGE_WORDS = re.compile(
    r"fits?\b|at most|below|within|wraps?|saturat|clamp|mask|checked|"
    r"overflow|range|cannot|bounded|limit", re.I)
CONTROL_ON_ONE_LINE = re.compile(
    r"^\s*(?:if|while|for)\s*\(.*\)\s*[^\s{;].*;\s*$")
BRACE_SAME_LINE = re.compile(
    r"^\s*(?:if|else|while|for|switch|do)\b.*\{\s*$|^\s*else\s*\{")
CONST_ON_RIGHT = re.compile(
    r"\b[a-z_][A-Za-z0-9_]*(?:\[[^\]]*\]|->\w+|\.\w+)*\s*[!=]=\s*"
    r"(?:[A-Z][A-Z0-9_]+\b|\d|NULL\b)")
CAST = re.compile(
    r"\(\s*(?:const\s+)?(?:u?int(?:8|16|32|64)_t|bool|char|UB|UH|UW|H|W|"
    r"INT|UINT|ID|ER|FP|size_t)\s*(?:const\s*)?\*?\s*\)\s*[\w(&*-]")

TYPE = (r"(?:u?int(?:8|16|32|64)_t|bool|char|int|UB|UH|UW|H|W|INT|UINT|ID|"
        r"ER|size_t|[a-z][a-z0-9_]*_t)")
DECL_BOOL = re.compile(r"\bbool\s+(?:const\s+)?([a-z_][a-z0-9_]*)\s*[=;,)\[]")
DECL_POINTER = re.compile(
    TYPE + r"\s*(?:const\s*)?\*\s*(?:const\s+)?([a-z_][a-z0-9_]*)\s*[=;,)\[]")
DECL_ANY = re.compile(
    r"(?:^\s*|[(,;{]\s*)(?:static\s+|const\s+|volatile\s+)*" + TYPE +
    r"\s+(?:const\s+)?([a-z_][a-z0-9_]*)\s*[=;,)\[]")


def check(path: Path, counts: Counter, where: dict) -> None:
    raw = path.read_text(encoding="utf-8", errors="replace")
    code = strip(raw)
    rel = path.relative_to(ROOT).as_posix()
    is_test = ("test" in path.name)
    raw_lines, lines = raw.split("\n"), code.split("\n")

    def hit(rule: str, line_no: int) -> None:
        counts[rule] += 1
        where[rule].append(f"{rel}:{line_no}")

    for no, line in enumerate(raw_lines, 1):
        if len(line.rstrip("\r")) > 80:
            hit("1.2a line over 80 characters", no)
        if "\t" in line:
            hit("3.5a tab character", no)
        if line.rstrip("\r") != line.rstrip("\r").rstrip():
            hit("3.6 trailing white space", no)
    func_start = [None]
    depth_now = 0
    for no, line in enumerate(lines, 1):
        if (0 == depth_now) and ("{" in line):
            func_start[0] = max(0, no - 6)  # include the header comment
        depth_now += line.count("{") - line.count("}")
        for word, rule in (("auto", "1.7a auto"), ("register", "1.7b register"),
                           ("goto", "1.7c goto"),
                           ("continue", "1.7d continue (preferred practice)")):
            if re.search(rf"\b{word}\b", line):
                hit(rule, no)
        if re.search(r"\b(?:short|long)\b", line):
            hit("5.2a short or long keyword", no)
        if (CONTROL_ON_ONE_LINE.match(line)
                and line.count("(") == line.count(")")):
            hit("1.3a unbraced body", no)
        if BRACE_SAME_LINE.match(line):
            hit("1.3b brace on the same line", no)
        if CONST_ON_RIGHT.search(line):
            hit("8.6a constant on the right of == or !=", no)
        for match in DECL_BOOL.finditer(line):
            if not match.group(1).startswith(("b_", "gb_", "g_b_")):
                hit("7.1m bool variable not named b_... ("
                    + ("test" if is_test else "production") + ")", no)
        for match in DECL_POINTER.finditer(line):
            if not match.group(1).startswith(("p_", "pp_", "gp_")):
                hit("7.1k pointer variable not named p_...", no)
        for match in DECL_ANY.finditer(line):
            if len(match.group(1)) < 3:
                hit("7.1e variable name under 3 characters ("
                    + ("test" if is_test else "production") + ")", no)
        for match in CAST.finditer(line):
            text = match.group(0)
            if re.match(r"\(\s*(const\s+)?UB\s*(const\s*)?\*", text):
                # tm_printf() takes UB *; the cast only changes the pointer
                # type of a string literal (char to unsigned char).
                counts["1.6a cast to UB * for tm_printf (interface-required)"] += 1
                continue
            # Associated comment: on the line, or in the same paragraph
            # above it (back to the previous blank line, at most 25 lines).
            start = no - 1
            while (start > 0) and (no - start < 25) \
                    and raw_lines[start - 1].strip():
                start -= 1
            nearby = "\n".join(raw_lines[start:no])
            # Or a comment earlier in the same function that argues the
            # range (fits, at most, wraps, saturated, checked, ...).
            body = "\n".join(raw_lines[func_start[0]:no]) \
                if func_start[0] is not None else ""
            ranged = any(RANGE_WORDS.search(c)
                         for c in re.findall(r"/\*.*?\*/", body, re.S))
            kind = "test" if is_test else "production"
            if ("/*" in nearby) or ("*/" in nearby) or ranged:
                counts[f"1.6a cast with an associated comment ({kind})"] += 1
            else:
                counts[f"1.6a cast with no associated comment ({kind})"] += 1
                where[f"1.6a no comment ({kind})"].append(f"{rel}:{no}")
    if path.suffix == ".h":
        guard = re.search(r"#ifndef\s+(\w+)\s*\n\s*#define\s+\1", raw)
        if not guard:
            hit("4.2 header without an include guard", 1)
    if not re.search(r"end of file", raw[-200:], re.IGNORECASE):
        hit("4.4 no end-of-file comment", len(raw_lines))

    # 6.2a: functions longer than 100 lines. At file scope, remember the
    # last identifier followed by "(" (the function name, which BARR-C puts
    # on its own line after the return type); a "{" at file scope then
    # opens that function's body.
    depth, start, name, last = 0, None, "", ""
    for no, line in enumerate(lines, 1):
        if depth == 0:
            for match in re.finditer(r"\b([A-Za-z_]\w*)\s*\(", line):
                if match.group(1) not in ("if", "while", "for", "switch",
                                          "sizeof", "return"):
                    last = match.group(1)
                    break
        for char in line:
            if char == "{":
                if depth == 0:
                    start, name = no, last
                depth += 1
            elif char == "}":
                depth -= 1
                if depth == 0 and start is not None:
                    length = no - start + 1
                    if length > 100:
                        hit("6.2a function over 100 lines", start)
                        where["6.2a names"].append(
                            f"{name} ({length} lines, {rel})")
                    start = None


def main() -> None:
    files = sorted(p for p in APP.rglob("*") if p.suffix in (".c", ".h"))
    counts, where = Counter(), defaultdict(list)
    for path in files:
        check(path, counts, where)
    print(f"BARR-C report: {len(files)} files under app_program/")
    rules = sorted(set(counts) | {"1.7a auto", "1.7b register", "1.7c goto",
                                  "3.5a tab character",
                                  "4.2 header without an include guard"})
    for rule in rules:
        print(f"  {counts[rule]:5}  {rule}")
        if 0 < counts[rule] <= 12 and rule in where:
            for loc in where[rule]:
                print(f"           {loc}")
    if where["6.2a names"]:
        print("  functions over 100 lines:")
        for item in where["6.2a names"]:
            print(f"           {item}")


if __name__ == "__main__":
    main()
