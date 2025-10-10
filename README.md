# msp_documentation
mapping/documentation of INAV multiwii serial protocol

## Heavily WIP

This is super bad and convoluted and has real problems with variable length/format messages (which isn't a surprise) but is intended as a basis to start defining the protocol PROGRAMMATICALLY

What we do:
* Get all possible enums, structs and defines we can from the inav source
* Read the generated markdown file
* Generate a json dictionnary of MSP messages from the markdown file
* Go look in the source files for structure details

This fails on complex undefined and variable length messages, which were a TERRIBLE idea to begin with AAAH!!!

** just run.sh **\

get_msp_defs.py \
parsemd.py -> msp_messages.json\
get_inav_defines.py\
get_inav_enums.py\
h_to_enum.py\
manually fix all_defines.h -> inav_defines.h\
bad_define_parse.py (this, as named, is bad; as it skips any define that isn't simple arithmetic and ignores #ifdefs; complex cases shouldn't be handled here anyway but in the flight controller)\
mspref.py (updates msp_messages.json)

# Not meant to be run every time/often, run it *ONCE* and *THEN* fix mistakes in the JSON file, not the error-prone Markdown

## Problems:
* need to (manually for now) remove all ifdefs from defines!!!
* some acquired enums and defines are unusable and have to be manually removed
* doubtful source-of-truth before burning the markdown file and going solely from json
* How do we handle polymorphic messages? Dunno yet lol
* consolidate everything
* I should run it once, be content, then burn everything before and only use the json

## JSON format example:
```
    "MSP_API_VERSION": {
        "id": 1,
        "hex": "0x1",
        "mspv": 1,
        "direction": 1,
        "request": null,
        "reply": {
            "size": 3,
            "payload": [
                {
                    "name": "mspProtocolVersion",
                    "ctype": "uint8_t",
                    "units": "",
                    "desc": "MSP Protocol version (`MSP_PROTOCOL_VERSION`, typically 0)."
                },
                {
                    "name": "apiVersionMajor",
                    "ctype": "uint8_t",
                    "units": "",
                    "desc": "INAV API Major version (`API_VERSION_MAJOR`)."
                },
                {
                    "name": "apiVersionMinor",
                    "ctype": "uint8_t",
                    "units": "",
                    "desc": "INAV API Minor version (`API_VERSION_MINOR`)."
                }
            ],
            "struct": "<BBB"
        },
        "notes": "Used by configurators to check compatibility.",
        "description": "Provides the MSP protocol version and the INAV API version."
    },
```
### Keys: 
**id**: Message code\
**direction**: 0 = to FC, 1 = from FC\
**request**: Message payload to FC. If empty, no payload\
**reply**: Message payload from FC, If empty, no payload\
**complex**: Weird polymorphic message that is a problem for future me

