#!/usr/bin/env python3
"""
gen_msp_headers.py
Turn an MSP JSON spec into a packed C header of message structs.

- Reads a JSON object: { "MSP_XYZ": { id, hex, mspv, direction, request, reply, notes, description } }
- Emits:
  - enum msp_command_id
  - #define MSP_<NAME>_ID
  - per-message packed structs: msp_<NAME>_request_t / msp_<NAME>_reply_t
  - payload size macros where fully known
  - comments with descriptions and notes

This does NOT implement encode/decode. It is a reference header generator.

Assumptions and behaviors:
- ctype "uint8_t", "uint16_t", "uint32_t", "int8_t", "int16_t", "int32_t", "float", "double" supported.
- ctype "char[<N>]" emits fixed arrays. "char[]" emits a flexible array member; must be last in struct or we fall back to char[1] with a comment.
- If field has "enum": "<TypeName>" and it is not "?", we use that typename instead of the numeric ctype.
- Fixed payload size is computed only if all fields are fixed width.
- Generates helpful comments so the header can be used as human-readable reference.

Author: you
License: do what you want.
"""

import argparse
import json
import os
import re
import sys
from typing import Any, Dict, List, Optional, Tuple

# -------------------- tiny helpers --------------------

INT_SIZES = {
    "uint8_t": 1, "int8_t": 1,
    "uint16_t": 2, "int16_t": 2,
    "uint32_t": 4, "int32_t": 4,
    "uint64_t": 8, "int64_t": 8,
    "float": 4, "double": 8,
    "bool": 1
}

CHAR_FIXED_RE = re.compile(r"^char\[(\d+)\]$")
CHAR_FLEX_RE = re.compile(r"^char\[\]$")

def to_c_ident(s: str) -> str:
    # Keep existing CAPS and underscores from MSP names.
    # Ensure it starts with a letter or underscore.
    s2 = re.sub(r"[^A-Za-z0-9_]", "_", s)
    if not re.match(r"[A-Za-z_]", s2):
        s2 = "_" + s2
    return s2

def to_type_name(msg_name: str, kind: str, prefix: str = "msp") -> str:
    # kind: "request" or "reply"
    return f"{prefix}_{msg_name}_{kind}_t"

def map_ctype(field: Dict[str, Any]) -> Tuple[str, Optional[int], bool, Optional[int]]:
    """
    Returns (ctype_str, fixed_size_bytes_or_None, is_char_array, array_len_or_None)
    - is_char_array is True for char[...] and char[].
    - array_len is None for flexible arrays or non-char types.
    """
    ctype = field.get("ctype", "").strip()

    # enum override
    enum_t = field.get("enum", None)
    if enum_t and enum_t != "?" and enum_t != "?_e":
        # trust user-provided enum. Size will follow underlying ctype if known; we cannot auto-size enums in C.
        # We will still compute size from underlying numeric ctype if possible.
        base = ctype if ctype in INT_SIZES else None
        size = INT_SIZES.get(base) if base else None
        return (enum_t, size, False, None)

    # char fixed
    m = CHAR_FIXED_RE.match(ctype)
    if m:
        n = int(m.group(1))
        return (f"char", n, True, n)

    # char flexible
    if CHAR_FLEX_RE.match(ctype):
        return ("char", None, True, None)

    # plain integer or float types
    if ctype in INT_SIZES:
        return (ctype, INT_SIZES[ctype], False, None)

    # Fallback: emit exactly what was given, unknown size
    return (ctype if ctype else "/* UNKNOWN_CTYPE */ uint8_t", None, False, None)

def compute_fixed_size(fields: List[Dict[str, Any]]) -> Optional[int]:
    total = 0
    for i, f in enumerate(fields):
        _, sz, is_char_arr, arr_len = map_ctype(f)
        if is_char_arr and arr_len is None:
            return None  # flexible array kills fixed size
        if sz is None:
            return None
        total += sz
    return total

def field_decl(f: Dict[str, Any], is_last: bool) -> Tuple[str, Optional[str]]:
    """
    Emit a single field declaration line and optional warning comment for illegal flexible placement.
    Returns (line, extra_comment_or_None).
    """
    name = to_c_ident(f.get("name", "field"))
    ctype, sz, is_char_arr, arr_len = map_ctype(f)
    desc = f.get("desc", "").strip()

    # choose the underlying numerical type for enums if enum name was used:
    # we already swapped the ctype to enum type above; keep it.
    # Nothing else to do.

    comment = f"// {desc}" if desc else ""

    if is_char_arr:
        if arr_len is not None:
            # fixed char[N]
            return (f"{ctype} {name}[{arr_len}]; {comment}", None)
        else:
            # flexible char[] must be last
            if is_last:
                return (f"{ctype} {name}[]; {comment}", None)
            else:
                warn = f"NOTE: JSON requested flexible array '{name}[]' not in final position. Using char[1] placeholder. Real payload is variable."
                line = f"{ctype} {name}[1]; // FLEX NOT LAST. Real field is variable length.\n    // {desc}" if desc else f"{ctype} {name}[1]; // FLEX NOT LAST. Real field is variable length."
                return (line, warn)
    else:
        return (f"{ctype} {name}; {comment}", None)

def emit_struct(msg_name: str, kind: str, payload: List[Dict[str, Any]], indent: str = "    ") -> Tuple[str, Optional[int], List[str]]:
    """
    Returns (struct_text, fixed_size_or_None, warnings[])
    """
    warnings = []
    tname = to_type_name(msg_name, kind)
    lines = [f"typedef struct MSP_PACKED {{"]
    for idx, f in enumerate(payload):
        last = (idx == len(payload) - 1)
        decl, warn = field_decl(f, last)
        lines.append(indent + decl)
        if warn:
            warnings.append(f"[{msg_name} {kind}] {warn}")
    lines.append(f"}} {tname};")
    lines.append("")  # blank line
    size = compute_fixed_size(payload)
    return ("\n".join(lines), size, warnings)

def emit_size_macro(msg_name: str, kind: str, size: Optional[int]) -> str:
    U = to_c_ident(msg_name)
    if size is None:
        return f"// #define {U}_{kind.upper()}_PAYLOAD_SIZE  /* variable or unknown size */"
    return f"#define {U}_{kind.upper()}_PAYLOAD_SIZE {size}"

def sort_by_id(items: List[Tuple[str, Dict[str, Any]]]) -> List[Tuple[str, Dict[str, Any]]]:
    return sorted(items, key=lambda kv: int(kv[1].get("id", 0)))

# -------------------- header generation --------------------

HEADER_PROLOGUE = """\
/*
 * Auto-generated by gen_msp_headers.py. Do not edit by hand.
 * This header mirrors the MSP message payloads defined in a JSON spec.
 *
 * It is a reference-only declaration set. No encode/decode here.
 */

#ifndef MSP_GENERATED_H
#define MSP_GENERATED_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "all_enums.h"
#include "all_defines.h"

#if defined(_MSC_VER)
  #define MSP_PACKED
  #pragma pack(push, 1)
#else
  #define MSP_PACKED __attribute__((packed))
#endif

#ifdef __cplusplus
extern "C" {
#endif

"""

HEADER_EPILOGUE = """
#ifdef __cplusplus
} // extern "C"
#endif

#if defined(_MSC_VER)
  #pragma pack(pop)
#endif

#endif // MSP_GENERATED_H
"""

def gen_header(spec: Dict[str, Any]) -> str:
    out: List[str] = []
    warnings: List[str] = []

    # sanity: normalize keys to stable order by id
    items = sort_by_id(list(spec.items()))

    # enum of command ids
    out.append("// ------- MSP command IDs -------")
    out.append("typedef enum {")

    for name, msg in items:
        U = to_c_ident(name)
        mid = int(msg.get("id"))
        out.append(f"    {U} = {mid},")
    out.append("} msp_command_id;")
    out.append("")

    # per-message ID and metadata macros
    out.append("// ------- MSP command metadata -------")
    for name, msg in items:
        U = to_c_ident(name)
        mid = int(msg.get("id"))
        hexv = msg.get("hex", None)
        mspv = msg.get("mspv", None)
        direction = msg.get("direction", None)
        if hexv:
            out.append(f"#define {U}_HEX {hexv}")
        out.append(f"#define {U}_ID {mid}")
        if mspv is not None:
            out.append(f"#define {U}_MSPV {int(mspv)}")
        if direction is not None:
            out.append(f"#define {U}_DIRECTION {int(direction)}  // 1=requests expecting reply, 0=write-only (as per JSON)")
        out.append("")
    out.append("")

    # per-message docs and structs
    out.append("// ------- MSP message structs -------")
    for name, msg in items:
        U = to_c_ident(name)
        desc = (msg.get("description") or "").strip()
        notes = (msg.get("notes") or "").strip()

        out.append(f"// {U}")
        if desc:
            out.append(f"// {desc}")
        if notes:
            out.append(f"// Notes: {notes}")

        # request
        req = msg.get("request", None)
        if req and isinstance(req, dict) and req.get("payload"):
            payload = req["payload"]
            text, sz, w = emit_struct(U, "request", payload)
            out.append(text)
            out.append(emit_size_macro(U, "request", sz))
            out.append("")
            warnings.extend(w)
        else:
            out.append(f"// No request payload for {U}.")
            out.append("")

        # reply
        rep = msg.get("reply", None)
        if rep and isinstance(rep, dict) and rep.get("payload"):
            payload = rep["payload"]
            text, sz, w = emit_struct(U, "reply", payload)
            out.append(text)
            out.append(emit_size_macro(U, "reply", sz))
            out.append("")
            warnings.extend(w)
        else:
            out.append(f"// No reply payload for {U}.")
            out.append("")

    # warnings as comments at the end
    if warnings:
        out.append("// ------- Generation warnings -------")
        for w in warnings:
            out.append(f"// {w}")
        out.append("")

    return HEADER_PROLOGUE + "\n".join(out) + HEADER_EPILOGUE

# -------------------- cli --------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="Generate packed C headers from an MSP JSON spec.")
    ap.add_argument("json", help="Input JSON file (object keyed by MSP message name).")
    ap.add_argument("-o", "--out", default="-", help="Output header path (default: stdout).")
    args = ap.parse_args()

    with open(args.json, "r", encoding="utf-8") as f:
        spec = json.load(f)

    if not isinstance(spec, dict):
        print("Top-level JSON must be an object mapping message names to specs.", file=sys.stderr)
        sys.exit(1)

    header = gen_header(spec)

    if args.out == "-" or args.out == "":
        sys.stdout.write(header)
    else:
        with open(args.out, "w", encoding="utf-8", newline="\n") as fo:
            fo.write(header)

if __name__ == "__main__":
    main()
