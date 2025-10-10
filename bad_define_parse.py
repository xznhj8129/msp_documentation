#!/usr/bin/env python3
"""
Read all_defines.h, parse #define constants, and write defines.py.

- Keeps object-like macros with simple arithmetic (+ - * // % << >> & | ^ ~, parentheses) and identifiers.
- Empty defines become NAME = None.
- Skips function-like macros and unusable junk (sizeof/alignof/typeof/offsetof/ARRAYLEN, casts, ->, ., [], {}, and any IDENT(...)).
- Converts / to // and strips C integer suffixes (U, UL, LL).
- Preserves references and orders them topologically when possible.
- After writing defines.py, attempts to exec it; on NameError, prepends the missing name as NAME = None and retries until it runs.

Hardcoded I/O:
  input:  all_defines.h
  output: inav_defines.py
"""

import re
from collections import defaultdict, deque
from pathlib import Path
import sys

IN_FILE = Path("all_defines.h")
OUT_FILE = Path("inav_defines.py")

DEFINE_RE = re.compile(r'^\s*#\s*define\s+([A-Za-z_]\w*)(?:\s+(.*?))?\s*$')
FUNC_LIKE_RE = re.compile(r'^\s*#\s*define\s+([A-Za-z_]\w*)\s*\(')
IDENT_RE = re.compile(r'[A-Za-z_]\w*')
CALL_IN_RHS_RE = re.compile(r'\b[A-Za-z_]\w*\s*\(')  # foo( ... )

INT_WITH_SUFFIX_RE = re.compile(r'''
    (?P<num>
        0[xX][0-9A-Fa-f]+
        |0[bB][01]+
        |0[oO][0-7]+
        |\d+
    )
    (?P<suf>[uUlL]+)?
''', re.VERBOSE)

ALLOWED_TOKENS_RE = re.compile(r'^[\s0-9A-Fa-fxXbo()+\-*%<>&|^~/_]*$')

FORBIDDEN_SNIPPETS = [
    'sizeof', 'alignof', 'typeof', 'offsetof', 'ARRAYLEN',
    '->', '.', '{', '}', '[', ']'
]
CAST_RE = re.compile(r'\(\s*[A-Za-z_][\w\s\*]*\)\s*')

def strip_block_comments(text: str) -> str:
    return re.sub(r'/\*.*?\*/', '', text, flags=re.S)

def strip_line_comment(s: str) -> str:
    return s.split('//', 1)[0]

def join_backslash_lines(text: str) -> str:
    lines = text.splitlines()
    out = []
    buf = ''
    for ln in lines:
        if ln.rstrip().endswith('\\'):
            buf += ln.rstrip()[:-1]
        else:
            out.append(buf + ln)
            buf = ''
    if buf:
        out.append(buf)
    return '\n'.join(out)

def normalize_expr(expr: str) -> str:
    s = expr.strip()
    s = strip_line_comment(s)
    s = s.replace('/', '//')
    s = INT_WITH_SUFFIX_RE.sub(lambda m: m.group('num'), s)
    s = re.sub(r'\s+', ' ', s).strip()
    return s

def is_func_like_define(line: str) -> bool:
    return bool(FUNC_LIKE_RE.match(line))

def is_unusable(rhs: str) -> bool:
    if any(tok in rhs for tok in FORBIDDEN_SNIPPETS):
        return True
    if CAST_RE.search(rhs):
        return True
    if CALL_IN_RHS_RE.search(rhs):
        return True
    if '?' in rhs or ':' in rhs or ',' in rhs:
        return True
    return False

def extract_defines(text: str):
    macros = {}
    for line in text.splitlines():
        if is_func_like_define(line):
            continue
        m = DEFINE_RE.match(line)
        if not m:
            continue
        name, rhs = m.group(1), m.group(2)

        if rhs is None or not strip_line_comment(rhs).strip():
            macros[name] = None
            continue

        rhs = normalize_expr(rhs)
        if is_unusable(rhs):
            continue

        tmp = IDENT_RE.sub('', rhs)
        if not ALLOWED_TOKENS_RE.match(tmp):
            continue

        macros[name] = rhs
    return macros

def dependency_graph(macros: dict):
    names = set(macros.keys())
    deps = {}
    for k, v in macros.items():
        if v is None:
            deps[k] = set()
        else:
            deps[k] = set(IDENT_RE.findall(v)) & names
    return deps

def topo_sort(macros: dict, deps: dict):
    indeg = {k: len(deps[k]) for k in macros}
    rev = defaultdict(set)
    for k, ds in deps.items():
        for d in ds:
            rev[d].add(k)
    order = []
    from collections import deque as dq
    q = dq([k for k, d in indeg.items() if d == 0])
    while q:
        u = q.popleft()
        order.append(u)
        for v in rev[u]:
            indeg[v] -= 1
            if indeg[v] == 0:
                q.append(v)
    leftover = [k for k, d in indeg.items() if d > 0]
    return order, leftover

def write_python_base(out_path: Path, order, macros, leftover):
    with out_path.open('w', encoding='utf-8') as f:
        f.write('# Auto-generated from C #defines.\n')
        f.write('# flake8: noqa\n\n')
        f.write('\nclass InavDefines:\n')
        for name in order:
            val = macros[name]
            if val is None:
                f.write(f'    {name} = None\n')
            else:
                f.write(f'    {name} = {val}\n')
        if leftover:
            f.write('\n# Cyclic or unresolved references, left as comments:\n')
            for name in leftover:
                f.write(f'    # {name} = {macros[name] if macros[name] is not None else "None"}\n')

def try_exec_and_fix(out_path: Path, max_iters: int = 500):
    """
    Repeatedly exec the file. On NameError, prepend NAME = None and retry.
    Stops when exec succeeds or max_iters hit.
    """
    for _ in range(max_iters):
        code = out_path.read_text(encoding='utf-8')
        try:
            exec(compile(code, str(out_path), 'exec'), {})
            return True
        except NameError as e:
            # Try to pull missing name
            missing = getattr(e, 'name', None)
            if not missing:
                # Fallback parse from message
                # e.g. "name 'FOO' is not defined"
                m = re.search(r"name '([^']+)' is not defined", str(e))
                if m:
                    missing = m.group(1)
            if not missing:
                # Give up if we can't extract
                print(f"Could not extract missing name from NameError: {e}", file=sys.stderr)
                return False

            # If already defined, avoid infinite loop
            if re.search(rf'^\s*{re.escape(missing)}\s*=', code, flags=re.M):
                # Already present, but still NameError elsewhere; continue to next loop to catch next one
                # To avoid spinning, inject a dummy unique sentinel
                sentinel = f'__MISSING_SENTINEL_{missing}__'
                if sentinel in code:
                    return False
                patched = code.replace('# flake8: noqa', f'# flake8: noqa\n{sentinel} = None')
                out_path.write_text(patched, encoding='utf-8')
                continue

            # Prepend definition after the header
            lines = code.splitlines()
            insert_idx = 0
            # Keep the first two comment lines if present
            if lines and lines[0].startswith('# Auto-generated'):
                insert_idx = 2 if len(lines) > 1 and lines[1].startswith('# flake8') else 1
            new_line = f'{missing} = None'
            lines.insert(insert_idx, new_line)
            out_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
            continue
        except Exception as e:
            # Some other runtime error; bail with info
            print(f"Execution failed: {type(e).__name__}: {e}", file=sys.stderr)
            return False
    print("Max iterations reached while fixing missing names.", file=sys.stderr)
    return False

def main():
    if not IN_FILE.exists():
        print(f"Missing {IN_FILE}", file=sys.stderr)
        sys.exit(1)

    raw = IN_FILE.read_text(encoding='utf-8', errors='ignore')
    raw = strip_block_comments(raw)
    raw = join_backslash_lines(raw)

    macros = extract_defines(raw)
    if not macros:
        print("No usable #define lines found.", file=sys.stderr)
        sys.exit(2)

    deps = dependency_graph(macros)
    order, leftover = topo_sort(macros, deps)
    if not order and leftover:
        print("All macros appear cyclic or invalid; nothing written.", file=sys.stderr)
        sys.exit(3)

    write_python_base(OUT_FILE, order, macros, leftover)

    if try_exec_and_fix(OUT_FILE):
        print(f"Wrote {OUT_FILE} and resolved missing references with None where needed.")
    else:
        print(f"Wrote {OUT_FILE} but could not fully resolve all missing names.", file=sys.stderr)
        sys.exit(4)

if __name__ == '__main__':
    main()
