#!/usr/bin/env python3
import re
import json
from typing import List, Tuple, Optional, Dict, Any
from msp_codes import MSPCodes

bin_type_map = {
    "enum": "B",
    "uint8_t": "B",
    "uint16_t": "H",
    "uint32_t": "I",
    "uint64_t": "Q",
    "int8_t": "b",
    "int16_t": "h",
    "int32_t": "i",
    "int64_t": "q",
    "float": "f",
    "double": "d",
    "char": "c",
    "bool": "?",
    "boolean": "?",  # normalize to bool
    "boxBitmask_t": "Q",
}

msg_fmt = {
    "hex": "",
    "id": 0,
    "mspv": None,
    "size": None,
    "struct": None,
    "direction": None,
    "variable_len": None,
    "payload": None,
    "deprecated": None
}

# Byte sizes for struct format chars
_type_sizes = {"B":1,"H":2,"I":4,"Q":8,"b":1,"h":2,"i":4,"q":8,"f":4,"d":8,"c":1,"?":1}

_array_re = re.compile(r'^\s*([A-Za-z_]\w*)\s*\[\s*(.*?)\s*\]\s*$')

def _parse_int(s: str) -> Optional[int]:
    s = (s or "").strip()
    return int(s) if s.isdigit() else None

def _map_base_type(t: str) -> Optional[str]:
    t = t.strip()
    if t.lower() == "boolean":
        t = "bool"
    return bin_type_map.get(t)

def _emit_fixed_array(fmt_char: str, count: int) -> str:
    # Prefer compact "8H" over repeating
    return f"{count}{fmt_char}"

def build_struct_or_none(payload_fields, repetition_factor_str=None):
    """
    Build a concrete Python 'struct' format string from payload_fields or return None.
    Accepts either:
      - list of dicts: {"name":..., "ctype":..., "size":...[, "units":..., "desc":...]}
      - list of tuples: (name, ctype, size, units, desc)
    Strict rules:
      - Scalars must map via bin_type_map.
      - Arrays must have a numeric count in brackets, or a purely numeric total byte Size we can divide by the element size.
      - Open arrays 'type[]' or symbolic counts/macros (e.g., type[MAX_FOO]) return None (unless count derivable from numeric Size).
      - Unknown/opaque types return None.
      - Any single unknown => None (fail fast).
    Dependencies expected in outer scope:
      - bin_type_map, _type_sizes, _array_re, _parse_int, _map_base_type, _emit_fixed_array
    """
    def _norm_item(item):
        if isinstance(item, dict):
            name = item.get("name", "")
            ctype = item.get("ctype", "")
            size = item.get("size", "")
            # normalize size to string for downstream routines
            if isinstance(size, int):
                size = str(size)
            return name, ctype, (size or ""), item.get("units", ""), item.get("desc", "")
        # tuple: allow (name,ctype,size) or (name,ctype,size,units,desc)
        if isinstance(item, (tuple, list)):
            if len(item) >= 3:
                name = item[0]
                ctype = item[1]
                size = item[2]
                if isinstance(size, int):
                    size = str(size)
                units = item[3] if len(item) >= 4 else ""
                desc  = item[4] if len(item) >= 5 else ""
                return name, ctype, (size or ""), units, desc
        # fallback
        return "", "", "", "", ""

    parts = []

    for raw in payload_fields or []:
        field_name, c_type, size_bytes_str, _units, _desc = _norm_item(raw)
        t = (c_type or "").strip()
        if not t:
            return None

        # Array?
        m = _array_re.match(t)
        if m:
            base = m.group(1)
            count_str = (m.group(2) or "").strip()

            # char[N] -> Ns ; char[] -> None (unknown)
            if base == "char":
                if count_str.isdigit():
                    parts.append(f"{count_str}s")
                    continue
                size_n = _parse_int(size_bytes_str)
                if size_n is not None:
                    parts.append(f"{size_n}s")
                    continue
                return None  # unknown char array length
                
                raise Exception("unknown char array length")

            fmt = _map_base_type(base)
            #if not fmt:
            #    print('base:',base)
            #    raise Exception("unknown base type")

            # Numeric count in brackets
            if count_str.isdigit():
                parts.append(_emit_fixed_array(fmt, int(count_str)))
                continue

            # Empty [] => variable/open ended -> None
            if count_str == "":
                return None

            # Symbolic like MAX_FOO -> try derive from purely numeric Size
            size_n = _parse_int(size_bytes_str)
            if size_n is None:
                return None
            bpt = _type_sizes.get(fmt)
            if not bpt or size_n % bpt != 0:
                return None
            parts.append(_emit_fixed_array(fmt, size_n // bpt))
            continue

        # Scalar
        fmt = _map_base_type(t)
        if fmt:
            parts.append(fmt)
            continue

        # Anything else (structs/opaque/custom) -> unknown
        return None

    if not parts:
        return None

    # Optional compaction of consecutive identical single-letter tokens (e.g., III -> 3I)
    compact = []
    i = 0
    while i < len(parts):
        tok = parts[i]
        # tokens like '8H' or '4s' already compact; leave as-is
        if len(tok) > 1 and tok[0].isdigit():
            compact.append(tok)
            i += 1
            continue
        # count run of same single-letter tokens
        j = i + 1
        while j < len(parts) and parts[j] == tok and len(parts[j]) == 1:
            j += 1
        run_len = j - i
        if run_len > 1:
            compact.append(f"{run_len}{tok}")
        else:
            compact.append(tok)
        i = j

    struct_str = "".join(compact)

    # Apply repetition strictly; symbolic repetition => unknown
    if repetition_factor_str:
        rep = str(repetition_factor_str).strip()
        if rep.isdigit():
            struct_str = struct_str * int(rep)
        else:
            return None

    return struct_str



# ===== Markdown parsing helpers (no regex soup) =====

MSG_HEADER_RE = re.compile(r'^###\s+`(MSP[A-Za-z0-9_]+(?:_EX|_EXT2)?)`\s*\(([^)]+)\)\s*$', re.MULTILINE)

def _strip_ticks(s: str) -> str:
    return s.strip().strip('`').strip()

def _find_after_label(body: str, label: str) -> Optional[str]:
    """
    Returns the remainder of the line after a markdown label like '**Direction:**'.
    Purely line oriented; not greedy into next sections.
    """
    needle = f"**{label}:**"
    i = body.find(needle)
    if i == -1:
        return None
    line = body[i + len(needle):].splitlines()[0]
    return line.strip()

def _slice_between(body: str, start_idx: int, end_markers: List[str]) -> str:
    """
    Slice body from start_idx to the earliest next end marker (or end of string).
    """
    j = len(body)
    for marker in end_markers:
        k = body.find(marker, start_idx)
        if k != -1 and k < j:
            j = k
    return body[start_idx:j]

def _find_payload_block(msg_content: str, direction: str) -> str:
    """
    Heuristic to return the payload subsection text (which may contain tables or 'Payload: None').
    Prefers Reply for In/Out, Request for In, plain Payload for Out.
    Falls back to first '**Payload' block if specific not found.
    """
    # Candidates in priority order
    candidates: List[str] = []

    if direction == "In/Out":
        candidates += [r'**Reply Payload', r'**Request Payload', r'**Payload']
    elif direction == "In":
        candidates += [r'**Request Payload', r'**Payload']
    else:
        candidates += [r'**Payload', r'**Reply Payload', r'**Request Payload']

    for label in candidates:
        pos = msg_content.find(label)
        if pos != -1:
            # Start after the closing '**' and optional colon
            line_start = pos
            # take from end of that line
            start = msg_content.find('\n', line_start)
            if start == -1:
                start = len(msg_content)
            else:
                start += 1
            block = _slice_between(
                msg_content,
                start,
                end_markers=[
                    "\n**Notes:**",
                    "\n**Field Tables for other formats:**",
                    "\n### `",
                    "\n**Request Payload",
                    "\n**Reply Payload",
                    "\n**Payload"
                ]
            )
            return block
    return ""  # not found

REPETITION_RE = re.compile(r'Repeated\s+(?:`([\w_]+)`|(\d+))\s+times:', re.IGNORECASE)

def _extract_repetition_factor(text: str) -> Tuple[Optional[str], str]:
    """
    Finds 'Repeated X times:' and returns (X_as_string_or_symbol, remaining_text_after_that_line).
    If not found, returns (None, original_text).
    """
    m = REPETITION_RE.search(text)
    if not m:
        return None, text
    factor = m.group(1) if m.group(1) else m.group(2)
    # Keep everything after the matched line for the actual table content
    after = text[m.end():]
    return factor, after

def _parse_table_from_block(block: str) -> List[Tuple[str,str,str,str,str]]:
    """
    Parses the first markdown table found in 'block'.
    Returns rows as (field, ctype, size, units, desc).
    Works with 4 or 5 column tables. If 'Units' column is absent, sets units="".
    Ignores header and separator rows.
    """
    lines = [ln.rstrip().lstrip() for ln in block.splitlines()]

    # Find the header row (starts with '|' and contains 'Field' somewhere) and the separator row
    header_idx = -1
    for i, ln in enumerate(lines):
        if ln.startswith('|') and 'Field' in ln:
            # next non-empty should be the --- separator
            if i + 1 < len(lines) and lines[i+1].startswith('|---'):
                header_idx = i
                break
    if header_idx == -1:
        # No table found; maybe "Payload: None" styled prose
        return False, []

    header_cells = [c.strip().strip('`') for c in lines[header_idx].split('|')[1:-1]]
    header_norm = [c.lower() for c in header_cells]

    # Determine column positions
    # Accept variants: "size (bytes)" or "size"
    col_field = _safe_index_name(header_norm, ["field"])
    col_ctype = _safe_index_name(header_norm, ["c type", "ctype", "type"])
    col_size  = _safe_index_name(header_norm, ["size (bytes)", "size"])
    col_units = _safe_index_name(header_norm, ["units"])
    col_desc  = _safe_index_name(header_norm, ["description", "desc"])

    rows: List[Tuple[str,str,str,str,str]] = []

    # Data rows start after the separator
    i = header_idx + 2
    varlen = False
    while i < len(lines):
        ln = lines[i]
        if not ln.startswith('|'):
            break
        if ln.startswith('|---'):
            i += 1
            continue

        cells = [c.strip().strip('`') for c in ln.split('|')[1:-1]]
        if len(cells) < 2:
            i += 1
            raise Exception('2')

        def get(col_idx: Optional[int]) -> str:
            if col_idx is None:
                return ""
            return cells[col_idx] if col_idx < len(cells) else ""

        field_name = get(col_field)
        if not field_name or field_name.lower() == "field":
            i += 1
            raise Exception('field name')

        # Skip subheaders some specs jam into tables
        fn_low = field_name.lower()
        if (fn_low.startswith("payload (format") # THIS IS BAD. THIS IS VERY BAD.
            or "**vehicle data (repeated" in fn_low
            or "**parts data (repeated" in fn_low):
            i += 1
            varlen = True
            continue

        ctype = get(col_ctype)
        size  = get(col_size)
        units = get(col_units)
        desc  = get(col_desc)
        
        if 'enum' in desc.lower():
            match = re.search(r'`([^`]*)`', desc)
            if match:
                print('ENUM DETECTED')
                print(match.group(1))
                isenum = match.group(1)
            else:
                isenum = False
        elif units=="Enum":
            isenum = "?_e"
        else:
            isenum = False

        # Normalize "Variable" forms (case differences)
        size = size.replace("variable", "Variable")
        try:
            size = int(size)
        except:
            varlen = True
            size = None
        val = {
            "name": field_name,
            "ctype":ctype,
            "size":size,
            "units":units,
            "desc":desc,
        }
        if isenum:
            val["enum"]=isenum
        rows.append(val)
        i += 1

    return varlen, rows

def _safe_index_name(header_norm: List[str], names: List[str]) -> Optional[int]:
    for name in names:
        for idx, hn in enumerate(header_norm):
            if hn == name:
                return idx
    return None


# ===== Main generator =====

def generate_msp_dict(markdown_content: str) -> Dict[str, Any]:
    msp_references: Dict[str, Any] = {}

    # Find all message headers and their body spans
    sections = []
    for m in MSG_HEADER_RE.finditer(markdown_content):
        msg_name = m.group(1).strip()
        id_info_str = m.group(2).strip()
        start = m.end()
        # body ends at next header or end of file
        next_m = MSG_HEADER_RE.search(markdown_content, pos=start)
        end = next_m.start() if next_m else len(markdown_content)
        body = markdown_content[start:end]
        sections.append((msg_name, id_info_str, body))

    for (msg_name, id_info_str, msg_content) in sections:
        # Parse message code from "(DEC / 0xHEX)" or "(0xHEX / DEC)"
        msg_code: Optional[int] = None
        match_dec_first = re.match(r"^\s*(\d+)\s*/\s*0x[\dA-Fa-f]+\s*$", id_info_str)
        if match_dec_first:
            msg_code = int(match_dec_first.group(1))
        else:
            match_hex_first = re.match(r"^\s*0x[\dA-Fa-f]+\s*/\s*(\d+)\s*$", id_info_str)
            if match_hex_first:
                msg_code = int(match_hex_first.group(1))
        if msg_code is None:
            parts = [p.strip() for p in id_info_str.split('/')]
            if parts and parts[0].isdigit():
                msg_code = int(parts[0])
            elif len(parts) > 1 and parts[1].isdigit():
                msg_code = int(parts[1])
        if msg_code is None:
            # Skip if we cannot determine code
            raise Exception("Cant find code")

        direction_str = _find_after_label(msg_content, "Direction")
        if not direction_str:
            # Skip messages without Direction
            print(msg_content)
            if "Not implemented" in msg_content:
                msp_references[msg_name] = {"implemented": False}
                continue
            else:
                raise Exception(f"Cant find direction for {msg_name}")
        direction_val = -1
        if direction_str.strip().startswith("Out"):
            direction_val = 1
        elif direction_str.strip().startswith("In/Out"):
            direction_val = 2
        elif direction_str.strip().startswith("In"):
            direction_val = 0
        elif direction_str.strip().startswith("N/A"):
            direction_val = -1

        # Description (single line)
        description_line = _find_after_label(msg_content, "Description")

        # Payload block
        payload_block = _find_payload_block(msg_content, "In/Out" if direction_val == 2 else ("In" if direction_val == 0 else "Out"))

        # "Payload: None" fast path
        payload_fields: List[Tuple[str,str,str,str,str]] = []
        repetition_factor_str: Optional[str] = None

        if "Payload: None" in payload_block:
            # No fields
            varlen = False
        else:
            # Extract repetition if present and trim block
            repetition_factor_str, payload_core = _extract_repetition_factor(payload_block)
            if repetition_factor_str is None:
                payload_core = payload_block

            # Parse first table found in payload_core
            varlen, payload_fields = _parse_table_from_block(payload_core)

        final_struct_str = build_struct_or_none(payload_fields, repetition_factor_str)
        compsize = 0
        for plf in payload_fields:
            try:
                compsize += plf['size']
            except:
                compsize = None
                break
        msg = msg_fmt.copy()

        msg["hex"] = hex(msg_code)
        msg["id"] = msg_code
        msg["mspv"] = 1 if msg_code<=255 else 2
        msg["size"] = compsize
        msg["struct"] = final_struct_str
        msg["direction"] = direction_val
        msg["payload"] = payload_fields
        msg["variable_len"] = varlen
        msp_references[msg_name] = msg
        if description_line:
            msp_references[msg_name]["description"] = description_line
        print(msg_name)
        for key, value in msp_references[msg_name].items():
            if key == 'payload':
                print('\t', key, ":")
                for p in value:  # value is the list of payload fields
                    #print("\t\t", p)   # each `p` is a tuple (field, ctype, size, units, desc)
                    sizs = p['size'] if p['size'] else '?'
                    print(f"\t\t{p['name']:20} {p['ctype']:12} {sizs:4} {p['units']:16} {p['desc']:16}")
            else:
                print('\t', key, ":", value)
    #print(msp_references[msg_name])

    return msp_references


# ===== CLI usage example =====

if __name__ == "__main__":
    with open("msp_messages_reference.md", "r", encoding="utf-8") as f:
        markdown = f.read()

    data = generate_msp_dict(markdown)
    for i,j in enumerate(MSPCodes):
        if j not in data:
            print(i,j)
            msg = msg_fmt.copy()
            msg['id'] = i
            msg['hex'] = hex(i)
            msg['name'] = j
            msg['missing'] = True
    text = json.dumps(data, indent=4, ensure_ascii=False)
    with open("msp_messages.json", "w", encoding="utf-8") as f:
        f.write(text)
