#!/usr/bin/env python3
import json
from typing import List, Tuple, Optional, Dict, Any
from lib.msp_enum import MultiWii

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
    "id": 0,
    "hex": "",
    "mspv": None,
    "direction": None,
    "request": None,
    #"reply": None,
    #"complex": False,
    #"deprecated": False
}

type_fmt = {
    "size": 0,
    "struct": "",
    "payload": None
    #"variants": 
}

val_fmt = {
    "name": None,
    "ctype": None,
    #"size": None,
    "units": None,
    "desc": None,
}

# Byte sizes for struct format chars
_type_sizes = {
    "B":1,
    "H":2,
    "I":4,
    "Q":8,
    "b":1,
    "h":2,
    "i":4,
    "q":8,
    "f":4,
    "d":8,
    "c":1,
    "?":1
}

def _parse_int(s: str) -> Optional[int]:
    s = (s or "").strip()
    if not s:
        return None
    if s.startswith(("0x", "0X")):
        try:
            return int(s, 16)
        except Exception:
            return None
    return int(s) if s.isdigit() else None

def _map_base_type(t: str):
    t = (t or "").strip()
    if t.lower() == "boolean":
        t = "bool"
    tt = bin_type_map.get(t)
    if not tt:
        if t.endswith(']'):
            tt = 'array'
        elif t.endswith('_t') and ' or ' not in t:
            tt = 'struct'
    
    return tt

def _emit_fixed_array(fmt_char: str, count: int) -> str:
    return f"{count}{fmt_char}"

def _parse_array_type(t: str) -> Optional[Tuple[str, str]]:
    """
    Parse "foo[bar]" without regex.
    Returns (base, count_str) or None if not an array type.
    """
    t = (t or "").strip()
    if not t:
        return None
    lb = t.find('[')
    rb = t.rfind(']')
    if lb == -1 or rb == -1 or rb < lb:
        return None
    base = t[:lb].strip()
    count_str = t[lb+1:rb].strip()
    return base, count_str

# ===== Markdown parsing helpers (no regex) =====

def _strip_ticks(s: str) -> str:
    return (s or "").strip().strip('`').strip()

def _find_after_label(body: str, label: str) -> Optional[str]:
    needle = f"**{label}:**"
    i = body.find(needle)
    if i == -1:
        return None
    line = body[i + len(needle):].splitlines()[0]
    return line.strip()

def _slice_between_ci(body: str, start_idx: int, end_markers: List[str]) -> str:
    """
    Case-insensitive earliest cut among markers. Returns slice body[start_idx:j].
    """
    if start_idx >= len(body):
        return ""
    low = body.casefold()
    j = len(body)
    for marker in end_markers:
        m = marker.casefold()
        k = low.find(m, start_idx)
        if k != -1 and k < j:
            j = k
    return body[start_idx:j]

def _normalize_bullet_prefix(line: str) -> str:
    s = line.lstrip()
    while s.startswith("* ") or s.startswith("- ") or s.startswith("+ "):
        s = s[2:].lstrip()
    # also tolerate "*   " etc.
    while s.startswith(("*", "-", "+")) and len(s) > 1 and s[1].isspace():
        s = s[2:].lstrip()
    return s

def _is_section_start(norm_line: str) -> bool:
    L = norm_line.strip()
    if L.startswith("### `"):
        return True
    if not L.startswith("**"):
        return False
    Llow = L.casefold()
    return (
        Llow.startswith("**request payload")
        or Llow.startswith("**reply payload")
        or Llow.startswith("**payload")
        or Llow.startswith("**notes:")
    )


def _find_named_payload_block(msg_content: str, label: str) -> str:
    """
    Find the block after '**Request Payload' or '**Reply Payload'.
    Handles list bullets before the '**' and same-line trailers (e.g. 'Repeated ...').
    """
    lines = msg_content.splitlines()
    target = f"**{label} payload".casefold()

    i = 0
    while i < len(lines):
        norm = _normalize_bullet_prefix(lines[i])
        nlow = norm.casefold()
        if nlow.startswith(target):
            # capture same-line trailer after the closing '**'
            tail = ""
            if "**" in norm[2:]:
                close_bold = norm.find("**", 2)
                if close_bold != -1:
                    tail = norm[close_bold + 2:].strip().lstrip(":").strip()

            # collect until next section start
            buf = []
            j = i + 1
            while j < len(lines):
                normj = _normalize_bullet_prefix(lines[j])
                if _is_section_start(normj):
                    break
                buf.append(lines[j])
                j += 1

            if tail:
                if tail.casefold() == "none":
                    return "Payload: None"
                return tail + "\n" + "\n".join(buf)
            return "\n".join(buf)
        i += 1
    return ""


def _find_generic_payload_block(msg_content: str) -> str:
    """
    Find a generic '**Payload**' header, optionally with '(Optional)', optionally with ':',
    and return any same-line note plus the following block until the next section marker.
    """
    low = msg_content.casefold()
    key = "**payload"
    idx = low.find(key)
    if idx == -1:
        return ""

    # Find the end of the bold header: the next '**' after idx
    close_bold = msg_content.find("**", idx + 2)
    if close_bold == -1:
        # give up; treat the rest of the line after key as same-line text
        line_end = msg_content.find("\n", idx)
        same_line = "" if line_end == -1 else msg_content[idx:line_end]
        return same_line

    # Same-line text after the bold header
    line_end = msg_content.find("\n", close_bold + 2)
    if line_end == -1:
        line_end = len(msg_content)
    same_line = msg_content[close_bold + 2:line_end].strip()
    if same_line:
        if same_line.strip().casefold() == "none":
            return "Payload: None"
        block_prefix = same_line + "\n"
    else:
        block_prefix = ""

    start = line_end + 1 if line_end < len(msg_content) else len(msg_content)
    block_body = _slice_between_ci(
        msg_content,
        start,
        end_markers=[
            "\n**Notes:**",
            "\n**Field Tables for other formats:**",
            "\n### `",
            "\n**Request Payload",
            "\n**Reply Payload",
            "\n**Payload",
        ],
    )
    return block_prefix + block_body

def _extract_repetition_factor(text: str) -> Tuple[Optional[str], str]:
    """
    Look for lines like:
      'Repeat 4 times:'
      'Repeated N times:'
      'Repeating `FOO_COUNT`:'
      'Repeating (barCount):'
      'Repeated for each thing:'
    Returns (factor_str_or_None, remainder_after_colon)
    """
    low = text.casefold()
    # find first 'repeat' variant
    idx = low.find("repeat")
    if idx == -1:
        return None, text
    #print(low)
    #input('stop')

    # ensure we actually have a colon in the same line
    nl = text.find("\n", idx)
    colon = text.find(":", idx, nl if nl != -1 else len(text))
    if colon == -1:
        #print('no colon')
        #input('stop')
        return None, text

    phrase = text[idx:colon]
    # 1) numeric
    digits = "".join(ch for ch in phrase if ch.isdigit())
    if digits:
        factor = digits
    else:
        # 2) backticked symbol
        bt1 = phrase.find("`")
        if bt1 != -1:
            bt2 = phrase.find("`", bt1 + 1)
            factor = phrase[bt1 + 1:bt2].strip() if bt2 != -1 else None
        else:
            # 3) parenthesized
            p1 = phrase.find("(")
            p2 = phrase.find(")", p1 + 1) if p1 != -1 else -1
            if p1 != -1 and p2 != -1 and p2 > p1:
                factor = phrase[p1 + 1:p2].strip()
            else:
                # 4) for each ...
                fe_idx = phrase.casefold().find("for each")
                if fe_idx != -1:
                    factor = phrase[fe_idx:].strip()
                else:
                    factor = phrase.strip()

    after = text[colon + 1:]
    #print(factor, after)
    #input('stop')
    return factor, after

def _first_backticked(s: str) -> Optional[str]:
    a = s.find("`")
    if a == -1:
        return None
    b = s.find("`", a + 1)
    if b == -1:
        return None
    return s[a + 1:b]

def _safe_index_name(header_norm: List[str], names: List[str]) -> Optional[int]:
    for name in names:
        for idx, hn in enumerate(header_norm):
            if hn == name:
                return idx
    return None

def _parse_table_from_block(block: str) -> List[Dict[str, Any]]:
    lines = [ln.strip() for ln in (block or "").splitlines()]

    # Find header row and separator
    header_idx = -1
    for i, ln in enumerate(lines):
        if ln.startswith('|') and 'Field' in ln:
            if i + 1 < len(lines):
                nxt = lines[i + 1].lstrip()
                if nxt.startswith('|---'):
                    header_idx = i
                    break
    if header_idx == -1:
        return []

    def _split_cells(row: str) -> List[str]:
        # remove leading and trailing pipe, then split
        parts = row.split('|')[1:-1]
        return [c.strip().strip('`') for c in parts]

    header_cells = _split_cells(lines[header_idx])
    header_norm = [c.lower() for c in header_cells]

    col_field = _safe_index_name(header_norm, ["field"])
    col_ctype = _safe_index_name(header_norm, ["c type", "ctype", "type"])
    col_size  = _safe_index_name(header_norm, ["size (bytes)", "size"])
    col_units = _safe_index_name(header_norm, ["units"])
    col_desc  = _safe_index_name(header_norm, ["description", "desc"])

    rows: List[Dict[str, Any]] = []
    i = header_idx + 2
    while i < len(lines):
        ln = lines[i]
        if not ln.startswith('|'):
            break
        if ln.lstrip().startswith('|---'):
            i += 1
            continue

        cells = _split_cells(ln)
        if len(cells) < 2:
            i += 1
            continue

        def get(col_idx: Optional[int]) -> str:
            if col_idx is None:
                return ""
            return cells[col_idx] if col_idx < len(cells) else ""

        field_name = get(col_field)
        if not field_name or field_name.lower() == "field":
            i += 1
            continue

        fn_low = field_name.lower()
        if (fn_low.startswith("payload (format")
            or "**vehicle data (repeated" in fn_low
            or "**parts data (repeated" in fn_low):
            i += 1
            continue

        ctype = get(col_ctype)
        #print(ctype)
        wtfisit = _map_base_type(ctype)
        #print(wtfisit)
        size_raw = get(col_size)
        units = get(col_units)
        desc  = get(col_desc)

        # enum detection: prefer backticked name in description; fallback to Units == Enum
        isenum = False
        if 'enum' in (desc or "").lower():
            bt = _first_backticked(desc or "")
            isenum = bt if bt else True
        elif (units or "") == "Enum":
            isenum = "?_e"

        # optional detection:
        isoptional = 'Optional' in desc


        # normalize size to int where possible; anything else -> None
        size: Optional[int] = None
        if size_raw:
            sr = size_raw.strip()
            if sr.isdigit():
                size = int(sr)
            else:
                # allow hex
                if sr.startswith(("0x","0X")):
                    try:
                        size = int(sr, 16)
                    except Exception:
                        size = None
                else:
                    size = None
        val = val_fmt.copy()
        val['name'] = field_name
        val['ctype'] = ctype
        if not wtfisit:
            val['polymorph'] = True
        #val['size'] = size
        val['units'] = units
        val['desc'] = desc
        if isoptional:
            val["optional"] = True 
        if isenum:
            # if True without a specific name, annotate opaque enum
            if isinstance(isenum, str):
                val["enum"] = isenum
            else:
                val["enum"] = "?_e"
        rows.append(val)
        i += 1

    return rows
def _iter_generic_payload_blocks(msg_content: str) -> list[str]:
    lines = msg_content.splitlines()
    blocks = []
    i = 0
    while i < len(lines):
        norm = _normalize_bullet_prefix(lines[i])
        nlow = norm.casefold()
        if nlow.startswith("**payload"):
            # tail after the closing '**'
            tail = ""
            close_bold = norm.find("**", 2)
            if close_bold != -1:
                tail = norm[close_bold + 2:].strip().lstrip(":").strip()
            # collect until next section
            buf = []
            j = i + 1
            while j < len(lines):
                normj = _normalize_bullet_prefix(lines[j])
                if _is_section_start(normj):
                    break
                buf.append(lines[j])
                j += 1
            if tail.casefold() == "none":
                blocks.append("Payload: None")
            elif tail:
                blocks.append((tail + "\n" + "\n".join(buf)).strip())
            else:
                blocks.append(("\n".join(buf)).strip())
            i = j
        else:
            i += 1
    return blocks

def _find_payload_blocks(msg_content: str, direction: int) -> tuple[str, str]:
    req = _find_named_payload_block(msg_content, "Request")
    rep = _find_named_payload_block(msg_content, "Reply")

    if not req or not rep:
        blocks = _iter_generic_payload_blocks(msg_content)
        if len(blocks) > 1:
            raise ValueError("Multiple generic '**Payload' sections found; unsupported multi-profile payload.")
        if blocks:
            g = blocks[0]  # string, not list
            if not req and direction == 0:
                req = g
            elif not rep:
                rep = g

    return req or "", rep or ""


# ===== Section scanner (no regex) =====

def _scan_message_sections(markdown_content: str) -> List[Tuple[str, str, str]]:
    """
    Return a list of (msg_name, id_info_str, body) for each '### `NAME` (ID / 0xHEX)' section.
    """
    sections: List[Tuple[str, str, str]] = []
    lines = markdown_content.splitlines(keepends=True)
    offsets = []
    total = 0
    for ln in lines:
        offsets.append(total)
        total += len(ln)

    header_indices: List[int] = []
    for idx, ln in enumerate(lines):
        s = ln.lstrip()
        if s.startswith("### "):
            # must contain a backticked name and parentheses
            if "`" in s and "(" in s and ")" in s:
                header_indices.append(idx)

    for h_idx_i, line_idx in enumerate(header_indices):
        header = lines[line_idx].lstrip()
        # extract name between backticks
        t1 = header.find("`")
        t2 = header.find("`", t1 + 1) if t1 != -1 else -1
        if t1 == -1 or t2 == -1:
            continue
        msg_name = header[t1 + 1:t2].strip()

        # extract id info inside parentheses after the name
        p1 = header.find("(", t2 + 1)
        p2 = header.find(")", p1 + 1) if p1 != -1 else -1
        if p1 == -1 or p2 == -1:
            id_info_str = ""
        else:
            id_info_str = header[p1 + 1:p2].strip()

        start_line = line_idx + 1
        end_line = header_indices[h_idx_i + 1] if (h_idx_i + 1) < len(header_indices) else len(lines)
        body = "".join(lines[start_line:end_line])
        sections.append((msg_name, id_info_str, body))
    return sections

def _parse_msg_code(id_info_str: str) -> Optional[int]:
    """
    Accepts forms like:
      '105 / 0x69'
      '0x69 / 105'
      '105/0x69'
      '0x69/105'
      '105'
    """
    s = (id_info_str or "").strip()
    if not s:
        return None
    parts = [p.strip() for p in s.split('/')]
    # try obvious digit first in order
    for p in parts:
        if p.isdigit():
            try:
                return int(p)
            except Exception:
                pass
    # try last resort: if a single token is present and numeric-like
    if len(parts) == 1 and parts[0]:
        p = parts[0]
        if p.isdigit():
            return int(p)
        if p.startswith(("0x","0X")):
            # hex-only header is rare, ignore
            return None
    # nothing usable
    return None

# ===== Main generator =====

def generate_msp_dict(markdown_content: str) -> Dict[str, Any]:
    msp_references: Dict[str, Any] = {}

    sections = _scan_message_sections(markdown_content)

    for (msg_name, id_info_str, msg_content) in sections:
        try:
            msg_code = int(id_info_str.split(' / ')[0])
        except:
            msg_code = int(id_info_str.split(' / ')[1])
        if msg_code is None:
            # If truly unknown, this is a spec error. Bail hard to catch it.
            raise Exception("Cant find code")

        direction_str = _find_after_label(msg_content, "Direction")
        if not direction_str:
            if "Not implemented" in msg_content:
                msp_references[msg_name] = {
                    "hex": hex(msg_code),
                    "id": msg_code,
                    "mspv": 1 if msg_code <= 255 else 2,
                    "implemented": False
                    }
                continue
            else:
                raise Exception(f"Cant find direction for {msg_name}")

        direction_val = -1
        d = direction_str.strip()
        if d.startswith("Out"):
            direction_val = 1
        elif d.startswith("In/Out"):
            direction_val = 2
        elif d.startswith("In"):
            direction_val = 0
        elif d.startswith("N/A"):
            direction_val = -1

        description_line = _find_after_label(msg_content, "Description")

        notes_str = _find_after_label(msg_content, "Notes")
        if not notes_str:
            notes_str = ""
        try:
            req_block, rep_block = _find_payload_blocks(msg_content, direction_val)
        except: #MSP2_COMMON_MOTOR_MIXER
            msg = msg_fmt.copy()
            msg["hex"] = hex(msg_code)
            msg["id"] = msg_code
            msg["mspv"] = 1 if msg_code <= 255 else 2
            msg["direction"] = direction_val
            msg["request"] = None
            msg["reply"] = None
            msg["complex"] = True
            msg["notes"] = notes_str
            if description_line:
                msg["description"] = description_line

            msp_references[msg_name] = msg
            continue

        def parse_block(block_text: str) -> tuple[list, Optional[int], Optional[str]]:
            if not block_text:
                return [], None, None
            if "Payload: None" in block_text or "**Payload:** None" in block_text:
                return [], None, None

            rep_factor, core = _extract_repetition_factor(block_text)
            if rep_factor is None:
                core = block_text

            fields = _parse_table_from_block(core)

            compsize: Optional[int] = 0
            for plf in fields:
                try:
                    sz = plf.get("size", None)
                    if sz is None:
                        compsize = None
                        break
                    compsize += sz
                except Exception:
                    compsize = None
                    break
            return fields, compsize, rep_factor
        
        req_fields, req_size, repeating_req = parse_block(req_block)
        rep_fields, rep_size, repeating_rep = parse_block(rep_block)
        #if msg_name == "MSP_OSD_CHAR_WRITE": exit(0)
        req_data = None
        if len(req_fields) > 0:
            req_data = type_fmt.copy()
            req_data["payload"] = req_fields
            #req_data["size"] = req_size
            if repeating_req: 
                if msg_name == "MSP2_INAV_SET_TEMP_SENSOR_CONFIG":
                    req_data["repeating"] = "MAX_TEMP_SENSORS"
                else:
                    req_data["repeating"] = repeating_req

        polymorph = False
        for f in req_fields:
            if 'polymorph' in f:
                polymorph = True
        for f in rep_fields:
            if 'polymorph' in f:
                polymorph = True
            

        rep_data = None
        if len(rep_fields) > 0:
            rep_data = type_fmt.copy()
            rep_data["payload"] = rep_fields
            #rep_data["size"] = rep_size
            if repeating_rep: rep_data["repeating"] = repeating_rep

        msg = msg_fmt.copy()
        msg["hex"] = hex(msg_code)
        msg["id"] = msg_code
        msg["mspv"] = 1 if msg_code <= 255 else 2
        #msg["size"] = rep_size if (rep_fields or rep_size is not None) else req_size
        msg["direction"] = direction_val
        msg["request"] = req_data
        if rep_data:
            msg["reply"] = rep_data
        msg["notes"] = notes_str
        if description_line:
            msg["description"] = description_line
        if polymorph:
            msg["complex"] = True

        msp_references[msg_name] = msg
        # Debug printout 
        print(msg_name) 
        for key, value in msp_references[msg_name].items(): 
            print("\t", key, ":", value) 

    return msp_references

# ===== CLI usage example =====

if __name__ == "__main__":
    with open("msp_messages_reference.md", "r", encoding="utf-8") as f:
        markdown = f.read()

    data = generate_msp_dict(markdown)
    for c in MultiWii:
        print(c.name, c.value)
        if c.name not in data:
            print(c.name, c.value)
            msg = msg_fmt.copy()
            msg['id'] = c.value
            msg['hex'] = hex(c.value)
            msg['name'] = c.name
            msg['missing'] = True
    text = json.dumps(data, indent=4, ensure_ascii=False)
    with open("lib/msp_messages.json", "w", encoding="utf-8") as f:
        f.write(text)
