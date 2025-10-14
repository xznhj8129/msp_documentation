
# INAV MSP Messages reference

**Warning: Work in progress**\
**Auto-generated with AI and source reverse-engineering until master reference file established**\
**Verification needed, exercise caution until completely verified for accuracy and cleared**\
**Refer to source for absolute certainty**

For details on the structure of MSP, see [The wiki page](https://github.com/iNavFlight/inav/wiki/MSP-V2)

For generation code, see [documentation project](https://github.com/xznhj8129/msp_documentation)


**Basic Concepts:**

*   **MSP Versions:**
    *   **MSPv1:** The original protocol. Uses command IDs from 0 to 254.
    *   **MSPv2:** An extended version. Uses command IDs from 0x1000 onwards. Can be encapsulated within an MSPv1 frame (`MSP_V2_FRAME` ID 255) or used natively.
*   **Direction:**
    *   **Out:** Message sent *from* the Flight Controller (FC) *to* the Ground Control Station (GCS), OSD, or other peripheral. Usually a request for data or status.
    *   **In:** Message sent *from* the GCS/OSD *to* the FC. Usually a command to set a parameter, perform an action, or provide data to the FC.
    *   **In/Out:** Can function in both directions, often used for getting/setting related data where the request might specify a subset (e.g., get specific waypoint, get specific setting info).
*   **Payload:** The data carried by the message, following the command ID. The structure (order, type, size of fields) is critical.
*   **Data Types:** Common C data types are used (`uint8_t`, `int16_t`, `uint32_t`, `float`, etc.). Pay close attention to signed vs. unsigned types and sizes.
*   **Packing:** Data fields are packed sequentially in the order listed. `__attribute__((packed))` is often used in struct definitions to prevent compiler padding.

---