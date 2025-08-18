import json
import re
import os


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
    "boolean": "?"
}


with open("msp_messages.json","r") as file:
    f = file.read()
    msp = json.loads(f)

print(len(msp))

for msg_code in msp:
    print(msg_code)
    total_bytes = 0
    structstr = "<"
    for key, dtype in msp[msg_code]["payload"]:
        print('\t',key,':',dtype)
        if dtype.startswith("char["):
            m = re.search(r'\[(\d+)\]', dtype)
            if m:
                number = int(m.group(1))
                char_code = 'c' * number
            else:
                char_code = 's'
        else:
            char_code = bin_type_map[dtype] 

        structstr += char_code

        if char_code in ['x']: # padding byte, counts as 1
            total_bytes += 1
        elif char_code in ['c', 'b', 'B', '?']: # 1 byte
            total_bytes += 1
        elif char_code in ['h', 'H']: # 2 bytes
            total_bytes += 2
        elif char_code in ['i', 'I', 'f']: # 4 bytes
            total_bytes += 4
        elif char_code in ['q', 'Q', 'd']: # 8 bytes
            total_bytes += 8
        elif len(char_code)>1 and char_code.startswith('c'):
            total_bytes += len(char_code)
        elif char_code == "s": # at least 1 byte?
            total_bytes += 1
        # '<', '>', '=', '!' are for endianness/alignment, not data bytes
        # '(', ')' are for grouping with symbolic multipliers, handled above.
        # '*' is part of symbolic multipliers, handled above.
        # if char_code not in '<>=!@()*':
        #    print(f"Warning: Unknown char_code '{char_code}' in byte count for {msg_name}")
    print('Struct:',structstr, 'Bytes:',total_bytes)