#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Generate Markdown documentation from an MSP message definitions JSON.

Strict + Index:
- Builds an Index grouped as MSPv1 (0–254) and MSPv2 (4096–20000) using lib.msp_enum.MSPCodes.
- STRICT: If a code exists in one (MSPCodes vs JSON) but not the other, crash with details.
- Index items link to headings via GitHub-style auto-anchors.
- Tight layout; identical Request/Reply tables; skip complex=true with a stub.
- Default input: lib/msp_messages.json ; default output: MSP_Doc.md
"""

import sys
import json
import re
import unicodedata
import manual_docs_fix
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from lib.msp_enum import MSPCodes  # required

# ---- C type size helpers ----------------------------------------------------

BASE_SIZES = {
    "uint8_t": 1, "int8_t": 1, "char": 1,
    "uint16_t": 2, "int16_t": 2,
    "uint32_t": 4, "int32_t": 4,
    "uint64_t": 8, "int64_t": 8,
    "float": 4, "double": 8,
}

array_brackets_re = re.compile(r"^(?P<base>[A-Za-z_0-9]+)\[(?P<size>.*)\]$")

def parse_ctype(ctype: str) -> Tuple[str, Optional[str]]:
    m = array_brackets_re.match(ctype.strip())
    if not m:
        return ctype.strip(), None
    return m.group("base").strip(), m.group("size").strip()

def sizeof_entry(field: Dict[str, Any]) -> str:
    ctype = field.get("ctype", "").strip()
    base, bracket = parse_ctype(ctype)

    is_array = bool(field.get("array", False))
    array_size_meta = field.get("array_size", None)
    array_ctype = field.get("array_ctype", base)

    if is_array or bracket is not None:
        base_for_size = array_ctype if is_array else base
        base_bytes = BASE_SIZES.get(base_for_size, None)

        if isinstance(array_size_meta, int):
            return str(array_size_meta * base_bytes) if base_bytes is not None else str(array_size_meta)

        if isinstance(array_size_meta, str) and array_size_meta:
            if base_bytes is None or base_for_size == "char":
                return array_size_meta
            return f"{array_size_meta} * {base_bytes}"

        if bracket is not None:
            if bracket == "":
                return "array"
            if bracket.isdigit():
                n = int(bracket)
                return str(n * base_bytes) if base_bytes is not None else str(n)
            if base_bytes is None or base == "char":
                return bracket
            return f"{bracket} * {base_bytes}"

        return "array"

    base_bytes = BASE_SIZES.get(base, None)
    return str(base_bytes) if base_bytes is not None else "-"


# ---- Markdown rendering -----------------------------------------------------

inav_wiki_url = "https://github.com/xznhj8129/msp_documentation/blob/master/docs/"
#inav_wiki_url = "https://github.com/iNavFlight/inav/wiki/"

def units_cell(field: Dict[str, Any]) -> str:
    if "enum" in field:
        if field["enum"]=="?_e":
            return "[ENUM_NAME](LINK_TO_ENUM)"
        else:
            return f"[{field['enum']}]({inav_wiki_url}inav_enums_ref.md#enum-{field['enum'].lower()})"
    u = (field.get("units") or "").strip()
    return u if u else "-"

def has_fields(section: Any) -> bool:
    if not isinstance(section, dict):
        return False
    payload = section.get("payload")
    return isinstance(payload, list) and len(payload) > 0

def get_fields(section: Any) -> List[Dict[str, Any]]:
    if not isinstance(section, dict):
        return []
    payload = section.get("payload")
    return payload if isinstance(payload, list) else []

def table_with_units(fields: List[Dict[str, Any]], label: str) -> str:
    header = (
        f"  \n**{label}:**\n"
        "| Field | C Type | Size (Bytes) | Units | Description |\n"
        "|---|---|---|---|---|\n"
    )
    rows = []
    for f in fields:
        name = f.get("name", "")
        ctype = f.get("ctype", "")
        size = sizeof_entry(f)
        if size == "0":
            size = "-"
        units = units_cell(f)
        desc = (f.get("desc") or "").strip()
        rows.append(f"| `{name}` | `{ctype}` | {size} | {units} | {desc} |")
    return header + "\n".join(rows) + "\n"

def render_message(name: str, msg: Dict[str, Any]) -> Tuple[str, str]:
    """
    Returns (section_markdown, heading_text_for_anchor)
    """
    code = msg.get("code", 0)
    hex_str = msg.get("hex", hex(code))
    description = (msg.get("description") or "").strip()
    notes = (msg.get("notes") or "").strip()
    complex_flag = bool(msg.get("complex", False))

    heading = f'## <a id="{name.lower()}"></a>`{name} ({code} / {hex_str})`'
    #heading = f"### `{name}` ({code} / {hex_str})"
    out = [heading + "\n"]

    #out.append("\n**Request Payload:** **None**  \n")
    if description:
        out.append(f"**Description:** {description}  \n")

    if complex_flag:
        out.append("**Special case, skipped for now**\n\n")
        return "".join(out), heading

    req = msg.get("request", None)
    rep = msg.get("reply", None)

    if has_fields(req):
        out.append(table_with_units(get_fields(req), "Request Payload"))
    else:
        out.append("\n**Request Payload:** **None**  \n")

    if has_fields(rep):
        out.append(table_with_units(get_fields(rep), "Reply Payload"))
    else:
        out.append("\n**Reply Payload:** **None**  \n")

    if notes:
        out.append(f"\n**Notes:** {notes}\n")

    out.append("\n")
    return "".join(out), heading

# ---- Index + strict consistency --------------------------------------------

def build_maps(defs: Dict[str, Any]) -> Tuple[Dict[int, str], Dict[int, str]]:
    """
    Returns:
      json_by_code: {code -> message_name_from_json}
      mw_by_code:   {code -> enum_name_from_MSPCodes}
    Only for codes in the enforced ranges (v1 and v2).
    """
    v1_range = range(0, 255)
    v2_range = range(4096, 20001)

    # JSON: build by code (restrict to ranges)
    json_by_code: Dict[int, str] = {}
    for name, body in defs.items():
        code = int(body.get("code", -1))
        if code in v1_range or code in v2_range:
            json_by_code[code] = name

    # MSPCodes: probe the same ranges
    mw_by_code: Dict[int, str] = {}
    def try_get(code: int) -> Optional[str]:
        try:
            e = MSPCodes(code)
            return e.name
        except Exception:
            return None

    for code in list(v1_range) + list(v2_range):
        ename = try_get(code)
        if ename is not None:
            mw_by_code[code] = ename

    return json_by_code, mw_by_code

def enforce_strict_match(json_by_code: Dict[int, str], mw_by_code: Dict[int, str]) -> None:
    json_codes = set(json_by_code.keys())
    mw_codes = set(mw_by_code.keys())

    only_in_json = sorted(json_codes - mw_codes)
    only_in_mw   = sorted(mw_codes - json_codes)

    if only_in_json or only_in_mw:
        lines = ["MSP code mismatch detected:"]
        if only_in_json:
            lines.append("  Present in JSON but missing in MSPCodes:")
            for c in only_in_json:
                lines.append(f"    {c}\t{json_by_code[c]}")
        if only_in_mw:
            lines.append("  Present in MSPCodes but missing in JSON:")
            for c in only_in_mw:
                lines.append(f"    {c}\t{mw_by_code[c]}")
        raise SystemExit("\n".join(lines))

def build_index(json_by_code: Dict[int, str]) -> str:
    """
    Build a compact index linking to each heading.
    """
    v1 = []
    v2 = []
    for code, name in sorted(json_by_code.items()):
        hex_str = hex(code)
        item = f"[{code} - {name}](#{name.lower()})  "
        if 0 <= code <= 255:
            v1.append(item)
        elif 4096 <= code <= 20000:
            v2.append(item)

    parts = ["## Index", "### MSPv1"]
    parts.extend(v1)
    parts.append("\n### MSPv2")
    parts.extend(v2)
    parts.append("")  # trailing newline
    return "\n".join(parts)

# ---- Orchestration ----------------------------------------------------------

def generate_markdown(defs: Dict[str, Any]) -> str:
    # Strict maps & check
    json_by_code, mw_by_code = build_maps(defs)
    enforce_strict_match(json_by_code, mw_by_code)

    # Build sections, remembering headings for slugging (already handled in index)
    items = sorted(((int(body.get("code", 0)), name, body) for name, body in defs.items()),
                   key=lambda t: t[0])

    sections = []
    for _, name, body in items:
        sec, _heading = render_message(name, body)

        if name == "MSP_SET_VTX_CONFIG":
            sections.append(sec.split('\n')[0]+'\n')
            sec = manual_docs_fix.MSP_SET_VTX_CONFIG + '\n\n'
        if name == "MSP2_COMMON_SET_SETTING":
            sections.append(sec.split('\n')[0]+'\n')
            sec = manual_docs_fix.MSP2_COMMON_SET_SETTING + '\n\n'
        if name == "MSP2_INAV_SET_GEOZONE_VERTEX":
            sections.append(sec.split('\n')[0]+'\n')
            sec = manual_docs_fix.MSP2_INAV_SET_GEOZONE_VERTEX + '\n\n'
        if name == "MSP2_SENSOR_HEADTRACKER": 
            sections.append(sec.split('\n')[0]+'\n')
            sec = manual_docs_fix.MSP2_SENSOR_HEADTRACKER + '\n\n'
        sections.append(sec)

    with open("docs_v2_header.md", "r", encoding="utf-8") as f:
        header = f.read()

    index_md = build_index(json_by_code)
    return header + "\n" + index_md + "\n" + "".join(sections)

def main():
    in_path = Path(sys.argv[1]) if len(sys.argv) >= 2 else Path("lib/msp_messages.json")
    out_path = Path(sys.argv[2]) if len(sys.argv) >= 3 else Path("docs/msp_ref_2.md")

    with in_path.open("r", encoding="utf-8") as f:
        defs = json.load(f)

    md = generate_markdown(defs)
    out_path.write_text(md, encoding="utf-8")
    print(f"Wrote {out_path}")

if __name__ == "__main__":
    main()
