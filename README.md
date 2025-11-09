# msp_documentation
mapping/documentation/reverse engineering of INAV multiwii serial protocol\

## JSON Spec Format: 
see [format.md](format.md)

## Docgen:
The `gen_docs.sh` helper regenerates the MSP documentation in a fixed sequence:
- Runs `get_all_inav_enums_h.py` to pull the latest INAV enum definitions and store them in `all_enums.h` for downstream tooling.
- Computes an `md5sum` of `msp_messages.json` and compares it with `msp_messages.checksum`; if the API definition changed it increments the numeric revision in `rev` so versioned outputs track the update.
- Executes `gen_msp_md.py` to rebuild the MSP message reference markdown from the JSON specification.
- Executes `gen_enum_md.py` to render the enum reference markdown using the freshly generated `all_enums.h`, then deletes the temporary header.