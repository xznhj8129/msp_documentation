MSP_SET_VTX_CONFIG = """{
    "variants": {
        "dataSize >= 2": {
        "request": {
            "size": 0,
            "struct": "<",
            "payload": [
            {
                "name": "newFrequency",
                "ctype": "uint16_t",
                "desc": "If <= VTXCOMMON_MSP_BANDCHAN_CHKVAL, value encodes (band,channel) as ((band-1)*8 + (channel-1)). Otherwise raw frequency (MHz).",
                "struct": "H"
            },

            
            {
                "when": "bytesRemaining > 1",
                "payload": [
                {
                    "name": "newPower",
                    "ctype": "uint8_t",
                    "desc": "VTX power level index.",
                    "struct": "B"
                },
                {
                    "name": "newPitmode",
                    "ctype": "uint8_t",
                    "desc": "Pit mode (0/1).",
                    "struct": "B"
                },

                {
                    "when": "bytesRemaining > 0",
                    "payload": [
                    {
                        "name": "lowPowerDisarm",
                        "ctype": "uint8_t",
                        "desc": "Low power on disarm (0/1).",
                        "struct": "B"
                    }
                    ]
                },

                {
                    "when": "bytesRemaining >= 2",
                    "payload": [
                    {
                        "name": "pitModeFreq",
                        "ctype": "uint16_t",
                        "desc": "Pit mode frequency (skipped by firmware if present).",
                        "struct": "H"
                    }
                    ]
                },

                {
                    "when": "bytesRemaining >= 2",
                    "payload": [
                    {
                        "name": "bandOverride",
                        "ctype": "uint8_t",
                        "desc": "Explicit band (overrides encoded band if provided).",
                        "struct": "B"
                    },
                    {
                        "name": "channelOverride",
                        "ctype": "uint8_t",
                        "desc": "Explicit channel (overrides encoded channel if provided).",
                        "struct": "B"
                    }
                    ]
                },

                {
                    "when": "bytesRemaining >= 2",
                    "payload": [
                    {
                        "name": "freqOverride",
                        "ctype": "uint16_t",
                        "desc": "Explicit frequency (MHz).",
                        "struct": "H"
                    }
                    ]
                },

                {
                    "when": "bytesRemaining >= 3",
                    "payload": [
                    {
                        "name": "bandCount",
                        "ctype": "uint8_t",
                        "desc": "Reported band count.",
                        "struct": "B"
                    },
                    {
                        "name": "channelCount",
                        "ctype": "uint8_t",
                        "desc": "Reported channels per band.",
                        "struct": "B"
                    },
                    {
                        "name": "powerCount",
                        "ctype": "uint8_t",
                        "desc": "Reported power level count (may clamp device capability).",
                        "struct": "B"
                    }
                    ]
                }
                ]
            }
            ]
        },
        "reply": null
        }
    },
    "description": "Set VTX configuration. Starts with a 16-bit word that is either an encoded (band,channel) or a raw frequency. Additional fields are optional and parsed only if enough bytes remain, in the exact order used by firmware."
}
"""

fixes = """{
    "MSP_SET_OSD_CONFIG": {
    "complex": false,
    "variants": {
        "dataSize >= 10": {
            "size": 0,
            "struct": "",
            "payload": [
                    {
                        "name": "videoSystem",
                        "ctype": "uint8_t",
                        "desc": "Enum `videoSystem_e`: Video system (Auto/PAL/NTSC) (`osdConfig()->video_system`). Sent even if OSD disabled",
                        "units": "Enum",
                        "enum": "videoSystem_e",
                        "struct": "B"
                    },
                    {
                        "name": "units",
                        "ctype": "uint8_t",
                        "desc": "Enum `osd_unit_e` Measurement units (Metric/Imperial) (`osdConfig()->units`). Sent even if OSD disabled",
                        "units": "Enum",
                        "enum": "osd_unit_e",
                        "struct": "B"
                    },
                    {
                        "name": "rssiAlarm",
                        "ctype": "uint8_t",
                        "desc": "RSSI alarm threshold (`osdConfig()->rssi_alarm`). Sent even if OSD disabled",
                        "units": "%",
                        "struct": "B"
                    },
                    {
                        "name": "capAlarm",
                        "ctype": "uint16_t",
                        "desc": "Capacity alarm threshold (`currentBatteryProfile->capacity.warning`). Sent even if OSD disabled",
                        "units": "mAh/mWh",
                        "struct": "H"
                    },
                    {
                        "name": "timerAlarm",
                        "ctype": "uint16_t",
                        "desc": "Timer alarm threshold (`osdConfig()->time_alarm`). Sent even if OSD disabled",
                        "units": "seconds",
                        "struct": "H"
                    },
                    {
                        "name": "altAlarm",
                        "ctype": "uint16_t",
                        "desc": "Altitude alarm threshold (`osdConfig()->alt_alarm`). Sent even if OSD disabled",
                        "units": "meters",
                        "struct": "H"
                    },
                    {
                        "name": "distAlarm",
                        "ctype": "uint16_t",
                        "desc": "Distance alarm threshold (`osdConfig()->dist_alarm`). Sent even if OSD disabled",
                        "units": "meters",
                        "struct": "H"
                    },
                    {
                        "name": "negAltAlarm",
                        "ctype": "uint16_t",
                        "desc": "Negative altitude alarm threshold (`osdConfig()->neg_alt_alarm`). Sent even if OSD disabled",
                        "units": "meters",
                        "struct": "H"
                    }
                ],
                "reply": null
        },
        "dataSize == 3": {
            "request": {     
                "size": 0,
                "struct": "",
                    "payload": [               
                    {
                        "name": "itemPositions",
                        "ctype": "uint16_t[OSD_ITEM_COUNT]",
                        "desc": "Packed X/Y position for each OSD item on screen 0 (`osdLayoutsConfig()->item_pos[0][i]`). Sent even if OSD disabled",
                        "array": true,
                        "array_ctype": "uint16_t",
                        "array_size": "OSD_ITEM_COUNT",
                        "units": "Coordinates",
                        "struct": "H"
                    }
                ]   

                },
                "reply": null
            }
        }
    },
    "MSP_OSD_CHAR_WRITE": {
        "complex": false,
        "variants": {
        "dataSize >= OSD_CHAR_BYTES + 2": {
            "request": {
            "size": 0,
            "struct": "<",
            "payload": [
                {
                "name": "address",
                "ctype": "uint16_t",
                "desc": "16-bit character address",
                "struct": "H"
                },
                {
                "name": "charData",
                "ctype": "uint8_t[OSD_CHAR_BYTES]",
                "desc": "Full character bytes (with metadata)",
                "array": true,
                "array_ctype": "uint8_t",
                "array_size": "OSD_CHAR_BYTES",
                "struct": "B"
                }
            ]
            },
            "reply": null
        },
        "dataSize >= OSD_CHAR_BYTES + 1": {
            "request": {
            "size": 0,
            "struct": "<",
            "payload": [
                {
                "name": "address",
                "ctype": "uint8_t",
                "desc": "8-bit character address",
                "struct": "B"
                },
                {
                "name": "charData",
                "ctype": "uint8_t[OSD_CHAR_BYTES]",
                "desc": "Full character bytes (with metadata)",
                "array": true,
                "array_ctype": "uint8_t",
                "array_size": "OSD_CHAR_BYTES",
                "struct": "B"
                }
            ]
            },
            "reply": null
        },
        "dataSize >= OSD_CHAR_VISIBLE_BYTES + 2": {
            "request": {
            "size": 0,
            "struct": "<",
            "payload": [
                {
                "name": "address",
                "ctype": "uint16_t",
                "desc": "16-bit character address",
                "struct": "H"
                },
                {
                "name": "charData",
                "ctype": "uint8_t[OSD_CHAR_VISIBLE_BYTES]",
                "desc": "Only visible character bytes (no metadata)",
                "array": true,
                "array_ctype": "uint8_t",
                "array_size": "OSD_CHAR_VISIBLE_BYTES",
                "struct": "B"
                }
            ]
            },
            "reply": null
        },
        "dataSize >= OSD_CHAR_VISIBLE_BYTES + 1": {
            "request": {
            "size": 0,
            "struct": "<",
            "payload": [
                {
                "name": "address",
                "ctype": "uint8_t",
                "desc": "8-bit character address",
                "struct": "B"
                },
                {
                "name": "charData",
                "ctype": "uint8_t[OSD_CHAR_VISIBLE_BYTES]",
                "desc": "Only visible character bytes (no metadata)",
                "array": true,
                "array_ctype": "uint8_t",
                "array_size": "OSD_CHAR_VISIBLE_BYTES",
                "struct": "B"
                }
            ]
            },
            "reply": null
        }
        }
    },

    "MSP2_COMMON_SET_TZ": {
        "complex": false,
        "variants": {
        "dataSize == 2": {
            "request": {
            "size": 0,
            "struct": "<",
            "payload": [
                {
                "name": "tz_offset",
                "ctype": "int16_t",
                "desc": "Timezone offset from UTC.",
                "units": "minutes",
                "struct": "h"
                }
            ]
            },
            "reply": null
        },
        "dataSize == 3": {
            "request": {
            "size": 0,
            "struct": "<",
            "payload": [
                {
                "name": "tz_offset",
                "ctype": "int16_t",
                "desc": "Timezone offset from UTC.",
                "units": "minutes",
                "struct": "h"
                },
                {
                "name": "tz_automatic_dst",
                "ctype": "uint8_t",
                "desc": "Automatic DST enable (0/1).",
                "units": "bool",
                "struct": "B"
                }
            ]
            },
            "reply": null
        }
        }
    }
}"""




# MANUAL CORRECTONS
def correct_enum(msgcode, direction, field, e_type):
    idx = next(i for i, f in enumerate(msp[msgcode][direction]["payload"]) if f.get("name","") == field)
    msp[msgcode][direction]["payload"][idx]["enum"] = e_type


import json
from lib.inav_defines import InavDefines  # the auto-generated module
from mspcommonlib import *
a = json.loads(fixes)

#a = json.loads(MSP_SET_VTX_CONFIG)

## manual message fix
with open("lib/msp_messages.json","r") as file:
    f = file.read()
    msp = json.loads(f)

for i in a:
    print(i)
    for j in a[i]:
        print('\t',j)
        msp[i][j] = a[i][j]

#bin_type_map[ctype] 

## manual struct fix:
structs = {
    "escSensorData_t": [
        { "name": "dataAge",     "ctype": "uint8_t"  },
        { "name": "temperature", "ctype": "int16_t"  },
        { "name": "voltage",     "ctype": "int16_t"  },
        { "name": "current",     "ctype": "int32_t"  },
        { "name": "rpm",         "ctype": "uint32_t" }
    ]
}
fmt = "".join(bin_type_map[f["ctype"]] for f in structs["escSensorData_t"])
msp["MSP2_INAV_ESC_TELEM"]["reply"]["payload"][1]["struct"] = fmt

with open("lib/msp_messages.json","w+") as file:
    file.write(json.dumps(msp,indent=4))