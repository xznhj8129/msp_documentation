#!/usr/bin/env python3
"""
Read all_defines.h, parse #define constants, and write defines.py.

- Keeps object-like macros with simple arithmetic (+ - * // % << >> & | ^ ~, parentheses) and identifiers.
- Empty defines become NAME = None.
- Skips function-like macros and unusable junk (sizeof/alignof/typeof/offsetof/ARRAYLEN, casts, ->, ., [], {}, and any IDENT(...)).
- Converts / to // and strips C integer suffixes (U, UL, LL).
- Preserves references where safe and orders them topologically when possible.
- Critically: never emits an arithmetic expression if any dependency is unknown or None. In that case it writes NAME = None.
- After writing defines.py, attempts to exec it; on NameError, prepends the missing name as NAME = None and retries until it runs.

Hardcoded I/O:
  input:  all_defines.h
  output: inav_defines.py
"""

import re
from collections import defaultdict
from pathlib import Path
import sys

IN_FILE = Path("lib/all_defines.h")
OUT_FILE = Path("lib/inav_defines.py")

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

# Force some known-problematic names to None unconditionally.
FORCE_NONE = {
    'FIXED_WING_LEVEL_TRIM_CONTROLLER_LIMIT',
}

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

def is_simple_ident(rhs: str) -> bool:
    s = rhs.strip()
    # Allow optional wrapping parens around a single identifier.
    while s.startswith('(') and s.endswith(')'):
        s = s[1:-1].strip()
    return bool(re.fullmatch(r'[A-Za-z_]\w*', s))

def is_numeric_literal(rhs: str) -> bool:
    return bool(re.fullmatch(r'(0[xX][0-9A-Fa-f]+|0[bB][01]+|0[oO][0-7]+|\d+)', rhs.strip()))

def is_operator_expr(rhs: str) -> bool:
    s = rhs.strip()
    # Strip outer parens
    while s.startswith('(') and s.endswith(')'):
        s = s[1:-1].strip()
    if is_simple_ident(s) or is_numeric_literal(s):
        return False
    # Any operator symbol means it's arithmetic/bitwise, which will evaluate in class body.
    return bool(re.search(r'[+\-*/%<>&|^~()]', s))

def extract_defines(text: str):
    macros = {}
    for line in text.splitlines():
        if is_func_like_define(line):
            continue
        m = DEFINE_RE.match(line)
        if not m:
            continue
        name, rhs = m.group(1), m.group(2)

        if name in FORCE_NONE:
            macros[name] = None
            continue

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

def resolve_safe_values(macros: dict, order: list):
    """
    Decide what to write for each macro so import never evaluates arithmetic on None.
    Rules:
      - If name in FORCE_NONE => None
      - If rhs is None => None
      - If rhs is a simple identifier:
            - write the reference as-is (safe even if it is None)
      - If rhs contains any operator:
            - if rhs references any identifier that is not a known macro => None
            - if rhs references any identifier already resolved to None => None
            - else keep the expression
    """
    names = set(macros.keys())
    resolved = {}
    for name in order:
        rhs = macros[name]
        if name in FORCE_NONE:
            resolved[name] = None
            continue
        if rhs is None:
            resolved[name] = None
            continue
        # Simple alias is always safe
        if is_simple_ident(rhs):
            ident = re.fullmatch(r'\(?\s*([A-Za-z_]\w*)\s*\)?', rhs).group(1)
            # If it aliases an unknown external, prefer None so we do not depend on exec-fixer.
            if ident not in names:
                resolved[name] = None
            else:
                resolved[name] = rhs
            continue
        if not is_operator_expr(rhs):
            # literal like (123) etc.
            resolved[name] = rhs
            continue
        # Operator expression: check deps
        deps = set(IDENT_RE.findall(rhs))
        externals = [d for d in deps if d not in names]
        if externals:
            resolved[name] = None
            continue
        any_none = any(resolved.get(d) is None for d in deps if d in resolved)
        if any_none:
            resolved[name] = None
        else:
            resolved[name] = rhs
    return resolved

def write_python_base(out_path: Path, order, macros, leftover, resolved):
    with out_path.open('w', encoding='utf-8') as f:
        f.write('# Auto-generated from C #defines.\n')
        f.write('# flake8: noqa\n\n')
        # Predeclare any external names that we chose to keep as alias (we currently set those to None instead)
        f.write('\nclass InavDefines:\n')
        for name in order:
            val = resolved[name]
            if val is None:
                f.write(f'    {name} = None\n')
            else:
                f.write(f'    {name} = {val}\n')
        if leftover:
            f.write('\n# Cyclic or unresolved references, left as comments:\n')
            for name in leftover:
                if name not in resolved:
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
            missing = getattr(e, 'name', None)
            if not missing:
                m = re.search(r"name '([^']+)' is not defined", str(e))
                if m:
                    missing = m.group(1)
            if not missing:
                print(f"Could not extract missing name from NameError: {e}", file=sys.stderr)
                return False
            if re.search(rf'^\s*{re.escape(missing)}\s*=', code, flags=re.M):
                sentinel = f'__MISSING_SENTINEL_{missing}__'
                if sentinel in code:
                    return False
                patched = code.replace('# flake8: noqa', f'# flake8: noqa\n{sentinel} = None')
                out_path.write_text(patched, encoding='utf-8')
                continue
            lines = code.splitlines()
            insert_idx = 0
            if lines and lines[0].startswith('# Auto-generated'):
                insert_idx = 2 if len(lines) > 1 and lines[1].startswith('# flake8') else 1
            lines.insert(insert_idx, f'{missing} = None')
            out_path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
            continue
        except Exception as e:
            # Any other runtime error here means something slipped through; surface it.
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

    resolved = resolve_safe_values(macros, order)
    resolved["DEBUG32_VALUE_COUNT"] = 8
    write_python_base(OUT_FILE, order, macros, leftover, resolved)

    if try_exec_and_fix(OUT_FILE):
        print(f"Wrote {OUT_FILE} and resolved missing references with None where needed.")
    else:
        print(f"Wrote {OUT_FILE} but could not fully resolve all missing names.", file=sys.stderr)
        sys.exit(4)

if __name__ == '__main__':
    main()
