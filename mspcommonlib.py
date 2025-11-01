

try:
    from lib.inav_defines import InavDefines
except:
    pass
try:
    from lib.inav_enums import InavEnums
except:
    pass
try:
    from lib.msp_enum import MSPCodes
except:
    pass


def get_define(name: str):
    if not name.isidentifier() or not name.isupper():
        raise ValueError(f"Bad define name: {name!r}")
    # returns the value, or None if missing or explicitly set to None
    return getattr(InavDefines, name, None)


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

## Commented out = Optional
msg_fmt = {
    "code": 0,
    "hex": "",
    "mspv": None,
    "direction": None,
    "request": None,
    "reply": None,
    "variable_len": False,
    "complex": False,
    #"deprecated": False,
}

type_fmt = {
    "size": 0,
    #"struct": "",
    "payload": None
    #"variants": 
}

val_fmt = {
    "name": None,
    "ctype": None,
    #"size": None,
    #"units": None,
    "desc": None,
    #"enum": None,
    #"bitmask": None,
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
