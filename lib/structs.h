// TODO: get these automatically

typedef struct {
    uint8_t dataAge;
    int16_t temperature;
    int16_t voltage;
    int32_t current;
    uint32_t rpm;
} escSensorData_t;

typedef struct escSensorConfig_s {
    uint16_t currentOffset;             // offset consumed by the flight controller / VTX / cam / ... in mA
    uint8_t  listenOnly;
} escSensorConfig_t;