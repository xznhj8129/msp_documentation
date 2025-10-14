#!/usr/bin/env python3
import datetime
import re
from pathlib import Path

BASE = Path('../inav/src/main')
SUBDIRS = [
    'msp',
]

enumfile = """
import enum

class MSPCodes(enum.IntEnum):
"""

def strip_comments(text: str) -> str:
    # Remove /* ... */ block comments and // line comments
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    text = re.sub(r'//.*', '', text)
    return text

def extract_defines(text: str):
    """
    Return a list of #define blocks.
    Handles:
      - Optional leading whitespace before '#'
      - Multi-line macros using trailing backslash
    """
    text = strip_comments(text)
    lines = text.splitlines()
    defines = []
    i = 0
    n = len(lines)

    define_re = re.compile(r'^\s*#\s*define\b')

    while i < n:
        line = lines[i]
        if define_re.match(line):
            block = [line.rstrip()]
            # Consume continuation lines ending with backslash (ignoring trailing spaces)
            while block[-1].rstrip().endswith('\\') and i + 1 < n:
                i += 1
                block.append(lines[i].rstrip())
            def1 = block[0]
            if def1.startswith ('#define MSP_') or def1.startswith ('#define MSP2_'):
                if "MSP_V2_" not in def1 and "MSP_VERSION_MAGIC_INITIALIZER" not in def1 and "MSP2_IS_SENSOR_MESSAGE" not in def1:
                    defines.append('\n'.join(block) + '\n')
        i += 1

    return defines, len(defines)


all_out_lines = []
total_defines = 0
file_hits = 0

for sd in SUBDIRS:
    root = BASE / sd
    if not root.is_dir():
        continue
    for fn in root.rglob('*'):
        if fn.suffix in ('.c', '.h'):
            if 'msp_serial' in str(fn) or 'sensor_msg' in str(fn):
                continue
            txt = fn.read_text(errors='ignore')
            extracted, count = extract_defines(txt)
            if count > 0:
                file_hits += 1
                all_out_lines.extend(extracted)
                total_defines += count

inav_msp_msgs = {}
asddsa = {}
for l in all_out_lines:
    parts = l.split()
    directive, name, value = parts[0], parts[1], parts[2]
    num = int(value, 0)
    asddsa[name] = num

# manual corrections
asddsa["MSP_IDENT"] = 100
asddsa["MSP_V2_FRAME"] = 255

inav_msp_msgs = dict(sorted(asddsa.items(), key=lambda x:x[1]))

del inav_msp_msgs["MSP_PROTOCOL_VERSION"]

for key in inav_msp_msgs:
    enumfile += f"    {key} = {inav_msp_msgs[key]}\n"
    print(key,":", inav_msp_msgs[key])

import json
with open('lib/msp_enum.py', 'w') as out:
    out.write(enumfile)

#print(f"Found {total_defines} #define entries across {file_hits} files")
