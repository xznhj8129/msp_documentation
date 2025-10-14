# msp_documentation
mapping/documentation/reverse engineering of INAV multiwii serial protocol
Creates one master JSON file from AI-gathered markdown, plugs holes, correct errors\
Also auto-generates markdown files\

## Heavily WIP

## Not meant to be run every time/often, run it *ONCE* and *THEN* fix mistakes in the JSON file, not the error-prone Markdown

This has real problems with variable length/format messages (which isn't a surprise) but is intended as a basis to start defining the protocol PROGRAMMATICALLY

What we do:
* Get all possible enums, structs and defines we can from the inav source
* Read the generated markdown file
* Generate a json dictionnary of MSP messages from the markdown file
* Go look in the source files for structure details
* Go look for the enums and structs

This fails on complex undefined and variable length messages, which were a TERRIBLE idea to begin with. Since there's only a dozen or so of them, they will just be added manually.\

** just run.sh **\

Once we're happy with the JSON, we use it as master for everything else going forward.





## Problems:
* wrote old get_enums, ifdef problems, wrote new get_enums, forgot about it, forgot how it worked, worked on old one, it had problems, remembered old one, forgot how it works, but just works. yeah.
* doubtful source-of-truth before burning the markdown file and going solely from json
* How do we handle polymorphic messages? Dunno yet lol
* consolidate everything
* I should run it once, be content, then burn everything before and only use the json

## JSON format example:
```
    "MSP_API_VERSION": {
        "code": 1,
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

