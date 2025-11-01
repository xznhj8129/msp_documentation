import json
import re
import os
from lib.inav_defines import InavDefines  # the auto-generated module
from mspcommonlib import *

with open("lib/msp_messages.json","r") as file:
    f = file.read()
    msp = json.loads(f)

print(len(msp))

for msg_code in msp:
    print()
    print(msg_code, msp[msg_code]['code'])
    total_bytes = 0
    
    abort = False
    for direction in ["request", "reply"]:
        structstr = "<"
        if abort:
            break
        print('\t',direction)
        if direction in msp[msg_code] and msp[msg_code][direction] is not None:
            for fieldidx, dat in enumerate(msp[msg_code][direction]["payload"]):

                #print(dat)
                key = dat["name"]
                ctype = dat["ctype"]
                print(f'\t\t {key} : {ctype}')
                char_code = None
                if ctype.startswith("char["):
                    m = re.search(r'\[(\d+)\]', ctype)
                    if m:
                        number = int(m.group(1))
                        char_code = 'c' * number
                        print("num*char_code", char_code)
                    else:
                        char_code = 's'
                        print("char_code", char_code)
                    #msp[msg_code][direction]["payload"][fieldidx]["struct"]=char_code
                else:
                    try:
                        
                        char_code = bin_type_map[ctype] 
                    except KeyError:
                        print('ctype',ctype)
                        try:
                            dt = bin_type_map[ctype[:ctype.find('[')]]
                        except KeyError:
                            if "_t" in ctype:
                                dt = 'struct'
                                
                            elif ' or ' not in ctype and 'Varie' not in ctype:
                                print("NEVERMIND THIS ONE", ctype)
                                msp[msg_code]['complex'] = True
                                abort = True
                                break
                        print('dt',dt)
                        m = re.search(r'\[\s*([^\]]+)\s*\]', ctype)

                        if m:
                            idx = m.group(1)
                            print('idx',idx)
                            try:
                                defval = int(idx)
                                if defval==0:
                                    raise Exception
                            except:
                                defval = get_define(idx)
                            if not defval:
                                defval = 1
                                #raise Exception
                                #print("NEVERMIND THIS ONE")
                                #msp[msg_code]['complex'] = True
                                #abort = True
                                #break
                            char_code = dt * defval
                            print(defval, char_code)
                        else:
                            char_code = dt
                msp[msg_code][direction]["payload"][fieldidx]["struct"] = char_code
                print("char_code", char_code)
                if char_code!= "struct":
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
            print('\t\tStruct:',structstr)
            print('\t\tBytes:',total_bytes)
            if msp[msg_code]["variable_len"]:
                total_bytes = 0
            msp[msg_code][direction]["struct"] = structstr
            msp[msg_code][direction]["size"] = total_bytes
            #if msg_code=="MSP_OSD_CONFIG":
            #    raise Exception


with open("lib/msp_messages.json","w+") as file:
    file.write(json.dumps(msp,indent=4))