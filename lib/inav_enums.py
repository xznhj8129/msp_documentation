import enum
from typing import Union, Dict, Any

# --- Enums ---

class InavEnums:
    class DjiCraftNameElements_t(enum.IntEnum):
        DJI_OSD_CN_MESSAGES = 0
        DJI_OSD_CN_THROTTLE = 1
        DJI_OSD_CN_THROTTLE_AUTO_THR = 2
        DJI_OSD_CN_AIR_SPEED = 3
        DJI_OSD_CN_EFFICIENCY = 4
        DJI_OSD_CN_DISTANCE = 5
        DJI_OSD_CN_ADJUSTEMNTS = 6
        DJI_OSD_CN_MAX_ELEMENTS = 7

    class RCDEVICE_5key_connection_event_e(enum.IntEnum):
        RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN = 0x01
        RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE = 0x02

    class Srxl2BindRequest(enum.IntEnum):
        EnterBindMode = 0xEB
        RequestBindStatus = 0xB5
        BoundDataReport = 0xDB
        SetBindInfo = 0x5B

    class Srxl2BindType(enum.IntEnum):
        NotBound = 0x0
        DSM2_1024_22ms = 0x01
        DSM2_1024_MC24 = 0x02
        DMS2_2048_11ms = 0x12
        DMSX_22ms = 0xA2
        DMSX_11ms = 0xB2
        Surface_DSM2_16_5ms = 0x63
        DSMR_11ms_22ms = 0xE2
        DSMR_5_5ms = 0xE4

    class Srxl2ControlDataCommand(enum.IntEnum):
        ChannelData = 0x00
        FailsafeChannelData = 0x01
        VTXData = 0x02

    class Srxl2DeviceId(enum.IntEnum):
        FlightControllerDefault = 0x30
        FlightControllerMax = 0x3
        Broadcast = 0xFF

    class Srxl2DeviceType(enum.IntEnum):
        NoDevice = 0
        RemoteReceiver = 1
        Receiver = 2
        FlightController = 3
        ESC = 4
        Reserved = 5
        SRXLServo = 6
        SRXLServo_2 = 7
        VTX = 8

    class Srxl2PacketType(enum.IntEnum):
        Handshake = 0x21
        BindInfo = 0x41
        ParameterConfiguration = 0x50
        SignalQuality = 0x55
        TelemetrySensorData = 0x80
        ControlData = 0xCD

    class Srxl2State(enum.IntEnum):
        Disabled = 0
        ListenForActivity = 1
        SendHandshake = 2
        ListenForHandshake = 3
        Running = 4

    class accEvent_t(enum.IntEnum):
        ACC_EVENT_NONE = 0
        ACC_EVENT_HIGH = 1
        ACC_EVENT_LOW = 2
        ACC_EVENT_NEG_X = 3

    class accelerationSensor_e(enum.IntEnum):
        ACC_NONE = 0
        ACC_AUTODETECT = 1
        ACC_MPU6000 = 2
        ACC_MPU6500 = 3
        ACC_MPU9250 = 4
        ACC_BMI160 = 5
        ACC_ICM20689 = 6
        ACC_BMI088 = 7
        ACC_ICM42605 = 8
        ACC_BMI270 = 9
        ACC_LSM6DXX = 10
        ACC_FAKE = 11
        ACC_MAX = ACC_FAKE

    class adjustmentFunction_e(enum.IntEnum):
        ADJUSTMENT_NONE = 0
        ADJUSTMENT_RC_RATE = 1
        ADJUSTMENT_RC_EXPO = 2
        ADJUSTMENT_THROTTLE_EXPO = 3
        ADJUSTMENT_PITCH_ROLL_RATE = 4
        ADJUSTMENT_YAW_RATE = 5
        ADJUSTMENT_PITCH_ROLL_P = 6
        ADJUSTMENT_PITCH_ROLL_I = 7
        ADJUSTMENT_PITCH_ROLL_D = 8
        ADJUSTMENT_PITCH_ROLL_FF = 9
        ADJUSTMENT_PITCH_P = 10
        ADJUSTMENT_PITCH_I = 11
        ADJUSTMENT_PITCH_D = 12
        ADJUSTMENT_PITCH_FF = 13
        ADJUSTMENT_ROLL_P = 14
        ADJUSTMENT_ROLL_I = 15
        ADJUSTMENT_ROLL_D = 16
        ADJUSTMENT_ROLL_FF = 17
        ADJUSTMENT_YAW_P = 18
        ADJUSTMENT_YAW_I = 19
        ADJUSTMENT_YAW_D = 20
        ADJUSTMENT_YAW_FF = 21
        ADJUSTMENT_RATE_PROFILE = 22
        ADJUSTMENT_PITCH_RATE = 23
        ADJUSTMENT_ROLL_RATE = 24
        ADJUSTMENT_RC_YAW_EXPO = 25
        ADJUSTMENT_MANUAL_RC_EXPO = 26
        ADJUSTMENT_MANUAL_RC_YAW_EXPO = 27
        ADJUSTMENT_MANUAL_PITCH_ROLL_RATE = 28
        ADJUSTMENT_MANUAL_ROLL_RATE = 29
        ADJUSTMENT_MANUAL_PITCH_RATE = 30
        ADJUSTMENT_MANUAL_YAW_RATE = 31
        ADJUSTMENT_NAV_FW_CRUISE_THR = 32
        ADJUSTMENT_NAV_FW_PITCH2THR = 33
        ADJUSTMENT_ROLL_BOARD_ALIGNMENT = 34
        ADJUSTMENT_PITCH_BOARD_ALIGNMENT = 35
        ADJUSTMENT_LEVEL_P = 36
        ADJUSTMENT_LEVEL_I = 37
        ADJUSTMENT_LEVEL_D = 38
        ADJUSTMENT_POS_XY_P = 39
        ADJUSTMENT_POS_XY_I = 40
        ADJUSTMENT_POS_XY_D = 41
        ADJUSTMENT_POS_Z_P = 42
        ADJUSTMENT_POS_Z_I = 43
        ADJUSTMENT_POS_Z_D = 44
        ADJUSTMENT_HEADING_P = 45
        ADJUSTMENT_VEL_XY_P = 46
        ADJUSTMENT_VEL_XY_I = 47
        ADJUSTMENT_VEL_XY_D = 48
        ADJUSTMENT_VEL_Z_P = 49
        ADJUSTMENT_VEL_Z_I = 50
        ADJUSTMENT_VEL_Z_D = 51
        ADJUSTMENT_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE = 52
        ADJUSTMENT_VTX_POWER_LEVEL = 53
        ADJUSTMENT_TPA = 54
        ADJUSTMENT_TPA_BREAKPOINT = 55
        ADJUSTMENT_NAV_FW_CONTROL_SMOOTHNESS = 56
        ADJUSTMENT_FW_TPA_TIME_CONSTANT = 57
        ADJUSTMENT_FW_LEVEL_TRIM = 58
        ADJUSTMENT_NAV_WP_MULTI_MISSION_INDEX = 59
        ADJUSTMENT_NAV_FW_ALT_CONTROL_RESPONSE = 60
        ADJUSTMENT_FUNCTION_COUNT = 61

    class adjustmentMode_e(enum.IntEnum):
        ADJUSTMENT_MODE_STEP = 0
        ADJUSTMENT_MODE_SELECT = 1

    class afatfsAppendFreeClusterPhase_e(enum.IntEnum):
        AFATFS_APPEND_FREE_CLUSTER_PHASE_INITIAL = 0
        AFATFS_APPEND_FREE_CLUSTER_PHASE_FIND_FREESPACE = 0
        AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT1 = 1
        AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FAT2 = 2
        AFATFS_APPEND_FREE_CLUSTER_PHASE_UPDATE_FILE_DIRECTORY = 3
        AFATFS_APPEND_FREE_CLUSTER_PHASE_COMPLETE = 4
        AFATFS_APPEND_FREE_CLUSTER_PHASE_FAILURE = 5

    class afatfsAppendSuperclusterPhase_e(enum.IntEnum):
        AFATFS_APPEND_SUPERCLUSTER_PHASE_INIT = 0
        AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FREEFILE_DIRECTORY = 1
        AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FAT = 2
        AFATFS_APPEND_SUPERCLUSTER_PHASE_UPDATE_FILE_DIRECTORY = 3

    class afatfsCacheBlockState_e(enum.IntEnum):
        AFATFS_CACHE_STATE_EMPTY = 0
        AFATFS_CACHE_STATE_IN_SYNC = 1
        AFATFS_CACHE_STATE_READING = 2
        AFATFS_CACHE_STATE_WRITING = 3
        AFATFS_CACHE_STATE_DIRTY = 4

    class afatfsClusterSearchCondition_e(enum.IntEnum):
        CLUSTER_SEARCH_FREE_AT_BEGINNING_OF_FAT_SECTOR = 0
        CLUSTER_SEARCH_FREE = 1
        CLUSTER_SEARCH_OCCUPIED = 2

    class afatfsDeleteFilePhase_e(enum.IntEnum):
        AFATFS_DELETE_FILE_DELETE_DIRECTORY_ENTRY = 0
        AFATFS_DELETE_FILE_DEALLOCATE_CLUSTERS = 1

    class afatfsError_e(enum.IntEnum):
        AFATFS_ERROR_NONE = 0
        AFATFS_ERROR_GENERIC = 1
        AFATFS_ERROR_BAD_MBR = 2
        AFATFS_ERROR_BAD_FILESYSTEM_HEADER = 3

    class afatfsExtendSubdirectoryPhase_e(enum.IntEnum):
        AFATFS_EXTEND_SUBDIRECTORY_PHASE_INITIAL = 0
        AFATFS_EXTEND_SUBDIRECTORY_PHASE_ADD_FREE_CLUSTER = 0
        AFATFS_EXTEND_SUBDIRECTORY_PHASE_WRITE_SECTORS = 1
        AFATFS_EXTEND_SUBDIRECTORY_PHASE_SUCCESS = 2
        AFATFS_EXTEND_SUBDIRECTORY_PHASE_FAILURE = 3

    class afatfsFATPattern_e(enum.IntEnum):
        AFATFS_FAT_PATTERN_UNTERMINATED_CHAIN = 0
        AFATFS_FAT_PATTERN_TERMINATED_CHAIN = 1
        AFATFS_FAT_PATTERN_FREE = 2

    class afatfsFileOperation_e(enum.IntEnum):
        AFATFS_FILE_OPERATION_NONE = 0
        AFATFS_FILE_OPERATION_CREATE_FILE = 1
        AFATFS_FILE_OPERATION_SEEK = 2
        AFATFS_FILE_OPERATION_CLOSE = 3
        AFATFS_FILE_OPERATION_TRUNCATE = 4
        AFATFS_FILE_OPERATION_UNLINK = 5
        AFATFS_FILE_OPERATION_APPEND_SUPERCLUSTER = 6
        AFATFS_FILE_OPERATION_LOCKED = 7
        AFATFS_FILE_OPERATION_APPEND_FREE_CLUSTER = 8
        AFATFS_FILE_OPERATION_EXTEND_SUBDIRECTORY = 9

    class afatfsFileType_e(enum.IntEnum):
        AFATFS_FILE_TYPE_NONE = 0
        AFATFS_FILE_TYPE_NORMAL = 1
        AFATFS_FILE_TYPE_FAT16_ROOT_DIRECTORY = 2
        AFATFS_FILE_TYPE_DIRECTORY = 3

    class afatfsFilesystemState_e(enum.IntEnum):
        AFATFS_FILESYSTEM_STATE_UNKNOWN = 0
        AFATFS_FILESYSTEM_STATE_FATAL = 1
        AFATFS_FILESYSTEM_STATE_INITIALIZATION = 2
        AFATFS_FILESYSTEM_STATE_READY = 3

    class afatfsFindClusterStatus_e(enum.IntEnum):
        AFATFS_FIND_CLUSTER_IN_PROGRESS = 0
        AFATFS_FIND_CLUSTER_FOUND = 1
        AFATFS_FIND_CLUSTER_FATAL = 2
        AFATFS_FIND_CLUSTER_NOT_FOUND = 3

    class afatfsFreeSpaceSearchPhase_e(enum.IntEnum):
        AFATFS_FREE_SPACE_SEARCH_PHASE_FIND_HOLE = 0
        AFATFS_FREE_SPACE_SEARCH_PHASE_GROW_HOLE = 1

    class afatfsInitializationPhase_e(enum.IntEnum):
        AFATFS_INITIALIZATION_READ_MBR = 0
        AFATFS_INITIALIZATION_READ_VOLUME_ID = 1
        AFATFS_INITIALIZATION_FREEFILE_CREATE = 2
        AFATFS_INITIALIZATION_FREEFILE_CREATING = 3
        AFATFS_INITIALIZATION_FREEFILE_FAT_SEARCH = 4
        AFATFS_INITIALIZATION_FREEFILE_UPDATE_FAT = 5
        AFATFS_INITIALIZATION_FREEFILE_SAVE_DIR_ENTRY = 6
        AFATFS_INITIALIZATION_FREEFILE_LAST = AFATFS_INITIALIZATION_FREEFILE_SAVE_DIR_ENTRY

    class afatfsOperationStatus_e(enum.IntEnum):
        AFATFS_OPERATION_IN_PROGRESS = 0
        AFATFS_OPERATION_SUCCESS = 1
        AFATFS_OPERATION_FAILURE = 2

    class afatfsSaveDirectoryEntryMode_e(enum.IntEnum):
        AFATFS_SAVE_DIRECTORY_NORMAL = 0
        AFATFS_SAVE_DIRECTORY_FOR_CLOSE = 1
        AFATFS_SAVE_DIRECTORY_DELETED = 2

    class afatfsSeek_e(enum.IntEnum):
        AFATFS_SEEK_SET = 0
        AFATFS_SEEK_CUR = 1
        AFATFS_SEEK_END = 2

    class afatfsTruncateFilePhase_e(enum.IntEnum):
        AFATFS_TRUNCATE_FILE_INITIAL = 0
        AFATFS_TRUNCATE_FILE_UPDATE_DIRECTORY = 0
        AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_NORMAL = 1
        AFATFS_TRUNCATE_FILE_ERASE_FAT_CHAIN_CONTIGUOUS = 2
        AFATFS_TRUNCATE_FILE_PREPEND_TO_FREEFILE = 3
        AFATFS_TRUNCATE_FILE_SUCCESS = 4

    class airmodeHandlingType_e(enum.IntEnum):
        STICK_CENTER = 0
        THROTTLE_THRESHOLD = 1
        STICK_CENTER_ONCE = 2

    class armingFlag_e(enum.IntEnum):
        ARMED = (1 << 2)
        WAS_EVER_ARMED = (1 << 3)
        SIMULATOR_MODE_HITL = (1 << 4)
        SIMULATOR_MODE_SITL = (1 << 5)
        ARMING_DISABLED_GEOZONE = (1 << 6)
        ARMING_DISABLED_FAILSAFE_SYSTEM = (1 << 7)
        ARMING_DISABLED_NOT_LEVEL = (1 << 8)
        ARMING_DISABLED_SENSORS_CALIBRATING = (1 << 9)
        ARMING_DISABLED_SYSTEM_OVERLOADED = (1 << 10)
        ARMING_DISABLED_NAVIGATION_UNSAFE = (1 << 11)
        ARMING_DISABLED_COMPASS_NOT_CALIBRATED = (1 << 12)
        ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED = (1 << 13)
        ARMING_DISABLED_ARM_SWITCH = (1 << 14)
        ARMING_DISABLED_HARDWARE_FAILURE = (1 << 15)
        ARMING_DISABLED_BOXFAILSAFE = (1 << 16)
        ARMING_DISABLED_RC_LINK = (1 << 18)
        ARMING_DISABLED_THROTTLE = (1 << 19)
        ARMING_DISABLED_CLI = (1 << 20)
        ARMING_DISABLED_CMS_MENU = (1 << 21)
        ARMING_DISABLED_OSD_MENU = (1 << 22)
        ARMING_DISABLED_ROLLPITCH_NOT_CENTERED = (1 << 23)
        ARMING_DISABLED_SERVO_AUTOTRIM = (1 << 24)
        ARMING_DISABLED_OOM = (1 << 25)
        ARMING_DISABLED_INVALID_SETTING = (1 << 26)
        ARMING_DISABLED_PWM_OUTPUT_ERROR = (1 << 27)
        ARMING_DISABLED_NO_PREARM = (1 << 28)
        ARMING_DISABLED_DSHOT_BEEPER = (1 << 29)
        ARMING_DISABLED_LANDING_DETECTED = (1 << 30)
        ARMING_DISABLED_ALL_FLAGS = (ARMING_DISABLED_GEOZONE | ARMING_DISABLED_FAILSAFE_SYSTEM | ARMING_DISABLED_NOT_LEVEL | 
                                                       ARMING_DISABLED_SENSORS_CALIBRATING | ARMING_DISABLED_SYSTEM_OVERLOADED | ARMING_DISABLED_NAVIGATION_UNSAFE |
                                                       ARMING_DISABLED_COMPASS_NOT_CALIBRATED | ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED |
                                                       ARMING_DISABLED_ARM_SWITCH | ARMING_DISABLED_HARDWARE_FAILURE | ARMING_DISABLED_BOXFAILSAFE |
                                                       ARMING_DISABLED_RC_LINK | ARMING_DISABLED_THROTTLE | ARMING_DISABLED_CLI |
                                                       ARMING_DISABLED_CMS_MENU | ARMING_DISABLED_OSD_MENU | ARMING_DISABLED_ROLLPITCH_NOT_CENTERED |
                                                       ARMING_DISABLED_SERVO_AUTOTRIM | ARMING_DISABLED_OOM | ARMING_DISABLED_INVALID_SETTING |
                                                       ARMING_DISABLED_PWM_OUTPUT_ERROR | ARMING_DISABLED_NO_PREARM | ARMING_DISABLED_DSHOT_BEEPER |
                                                       ARMING_DISABLED_LANDING_DETECTED)

    class baroSensor_e(enum.IntEnum):
        BARO_NONE = 0
        BARO_AUTODETECT = 1
        BARO_BMP085 = 2
        BARO_MS5611 = 3
        BARO_BMP280 = 4
        BARO_MS5607 = 5
        BARO_LPS25H = 6
        BARO_SPL06 = 7
        BARO_BMP388 = 8
        BARO_DPS310 = 9
        BARO_B2SMPB = 10
        BARO_MSP = 11
        BARO_FAKE = 12
        BARO_MAX = BARO_FAKE

    class barometerState_e(enum.IntEnum):
        BAROMETER_NEEDS_SAMPLES = 0
        BAROMETER_NEEDS_CALCULATION = 1

    class batCapacityUnit_e(enum.IntEnum):
        BAT_CAPACITY_UNIT_MAH = 0
        BAT_CAPACITY_UNIT_MWH = 1

    class batVoltageSource_e(enum.IntEnum):
        BAT_VOLTAGE_RAW = 0
        BAT_VOLTAGE_SAG_COMP = 1

    class batteryState_e(enum.IntEnum):
        BATTERY_OK = 0
        BATTERY_WARNING = 1
        BATTERY_CRITICAL = 2
        BATTERY_NOT_PRESENT = 3

    class baudRate_e(enum.IntEnum):
        BAUD_AUTO = 0
        BAUD_1200 = 1
        BAUD_2400 = 2
        BAUD_4800 = 3
        BAUD_9600 = 4
        BAUD_19200 = 5
        BAUD_38400 = 6
        BAUD_57600 = 7
        BAUD_115200 = 8
        BAUD_230400 = 9
        BAUD_250000 = 10
        BAUD_460800 = 11
        BAUD_921600 = 12
        BAUD_1000000 = 13
        BAUD_1500000 = 14
        BAUD_2000000 = 15
        BAUD_2470000 = 16
        BAUD_MIN = BAUD_AUTO
        BAUD_MAX = BAUD_2470000

    class beeperMode_e(enum.IntEnum):
        BEEPER_SILENCE = 0
        BEEPER_RUNTIME_CALIBRATION_DONE = 1
        BEEPER_HARDWARE_FAILURE = 2
        BEEPER_RX_LOST = 3
        BEEPER_RX_LOST_LANDING = 4
        BEEPER_DISARMING = 5
        BEEPER_ARMING = 6
        BEEPER_ARMING_GPS_FIX = 7
        BEEPER_BAT_CRIT_LOW = 8
        BEEPER_BAT_LOW = 9
        BEEPER_GPS_STATUS = 10
        BEEPER_RX_SET = 11
        BEEPER_ACTION_SUCCESS = 12
        BEEPER_ACTION_FAIL = 13
        BEEPER_READY_BEEP = 14
        BEEPER_MULTI_BEEPS = 15
        BEEPER_DISARM_REPEAT = 16
        BEEPER_ARMED = 17
        BEEPER_SYSTEM_INIT = 18
        BEEPER_USB = 19
        BEEPER_LAUNCH_MODE_ENABLED = 20
        BEEPER_LAUNCH_MODE_LOW_THROTTLE = 21
        BEEPER_LAUNCH_MODE_IDLE_START = 22
        BEEPER_CAM_CONNECTION_OPEN = 23
        BEEPER_CAM_CONNECTION_CLOSE = 24
        BEEPER_ALL = 25
        BEEPER_PREFERENCE = 26

    class boxId_e(enum.IntEnum):
        BOXARM = 0
        BOXANGLE = 1
        BOXHORIZON = 2
        BOXNAVALTHOLD = 3
        BOXHEADINGHOLD = 4
        BOXHEADFREE = 5
        BOXHEADADJ = 6
        BOXCAMSTAB = 7
        BOXNAVRTH = 8
        BOXNAVPOSHOLD = 9
        BOXMANUAL = 10
        BOXBEEPERON = 11
        BOXLEDLOW = 12
        BOXLIGHTS = 13
        BOXNAVLAUNCH = 14
        BOXOSD = 15
        BOXTELEMETRY = 16
        BOXBLACKBOX = 17
        BOXFAILSAFE = 18
        BOXNAVWP = 19
        BOXAIRMODE = 20
        BOXHOMERESET = 21
        BOXGCSNAV = 22
        BOXSURFACE = 24
        BOXFLAPERON = 25
        BOXTURNASSIST = 26
        BOXAUTOTRIM = 27
        BOXAUTOTUNE = 28
        BOXCAMERA1 = 29
        BOXCAMERA2 = 30
        BOXCAMERA3 = 31
        BOXOSDALT1 = 32
        BOXOSDALT2 = 33
        BOXOSDALT3 = 34
        BOXNAVCOURSEHOLD = 35
        BOXBRAKING = 36
        BOXUSER1 = 37
        BOXUSER2 = 38
        BOXFPVANGLEMIX = 39
        BOXLOITERDIRCHN = 40
        BOXMSPRCOVERRIDE = 41
        BOXPREARM = 42
        BOXTURTLE = 43
        BOXNAVCRUISE = 44
        BOXAUTOLEVEL = 45
        BOXPLANWPMISSION = 46
        BOXSOARING = 47
        BOXUSER3 = 48
        BOXUSER4 = 49
        BOXCHANGEMISSION = 50
        BOXBEEPERMUTE = 51
        BOXMULTIFUNCTION = 52
        BOXMIXERPROFILE = 53
        BOXMIXERTRANSITION = 54
        BOXANGLEHOLD = 55
        BOXGIMBALTLOCK = 56
        BOXGIMBALRLOCK = 57
        BOXGIMBALCENTER = 58
        BOXGIMBALHTRK = 59
        CHECKBOX_ITEM_COUNT = 60

    class colorId_e(enum.IntEnum):
        COLOR_BLACK = 0
        COLOR_WHITE = 1
        COLOR_RED = 2
        COLOR_ORANGE = 3
        COLOR_YELLOW = 4
        COLOR_LIME_GREEN = 5
        COLOR_GREEN = 6
        COLOR_MINT_GREEN = 7
        COLOR_CYAN = 8
        COLOR_LIGHT_BLUE = 9
        COLOR_BLUE = 10
        COLOR_DARK_VIOLET = 11
        COLOR_MAGENTA = 12
        COLOR_DEEP_PINK = 13

    class crsfActiveAntenna_e(enum.IntEnum):
        CRSF_ACTIVE_ANTENNA1 = 0
        CRSF_ACTIVE_ANTENNA2 = 1

    class crsfAddress_e(enum.IntEnum):
        CRSF_ADDRESS_BROADCAST = 0x00
        CRSF_ADDRESS_USB = 0x10
        CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80
        CRSF_ADDRESS_RESERVED1 = 0x8A
        CRSF_ADDRESS_CURRENT_SENSOR = 0xC0
        CRSF_ADDRESS_GPS = 0xC2
        CRSF_ADDRESS_TBS_BLACKBOX = 0xC4
        CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
        CRSF_ADDRESS_RESERVED2 = 0xCA
        CRSF_ADDRESS_RACE_TAG = 0xCC
        CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA
        CRSF_ADDRESS_CRSF_RECEIVER = 0xEC
        CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE

    class crsfFrameTypeIndex_e(enum.IntEnum):
        CRSF_FRAME_START_INDEX = 0
        CRSF_FRAME_ATTITUDE_INDEX = CRSF_FRAME_START_INDEX

    class crsfFrameType_e(enum.IntEnum):
        CRSF_FRAMETYPE_GPS = 0x02
        CRSF_FRAMETYPE_VARIO_SENSOR = 0x07
        CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08
        CRSF_FRAMETYPE_LINK_STATISTICS = 0x14
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
        CRSF_FRAMETYPE_ATTITUDE = 0x1E
        CRSF_FRAMETYPE_FLIGHT_MODE = 0x21
        CRSF_FRAMETYPE_DEVICE_PING = 0x28
        CRSF_FRAMETYPE_DEVICE_INFO = 0x29
        CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B
        CRSF_FRAMETYPE_PARAMETER_READ = 0x2C
        CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D
        CRSF_FRAMETYPE_COMMAND = 0x32
        CRSF_FRAMETYPE_MSP_REQ = 0x7A
        CRSF_FRAMETYPE_MSP_RESP = 0x7B
        CRSF_FRAMETYPE_MSP_WRITE = 0x7C
        CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D

    class crsrRfMode_e(enum.IntEnum):
        CRSF_RF_MODE_4_HZ = 0
        CRSF_RF_MODE_50_HZ = 1
        CRSF_RF_MODE_150_HZ = 2

    class crsrRfPower_e(enum.IntEnum):
        CRSF_RF_POWER_0_mW = 0
        CRSF_RF_POWER_10_mW = 1
        CRSF_RF_POWER_25_mW = 2
        CRSF_RF_POWER_100_mW = 3
        CRSF_RF_POWER_500_mW = 4
        CRSF_RF_POWER_1000_mW = 5
        CRSF_RF_POWER_2000_mW = 6
        CRSF_RF_POWER_250_mW = 7

    class currentSensor_e(enum.IntEnum):
        CURRENT_SENSOR_NONE = 0
        CURRENT_SENSOR_ADC = 1
        CURRENT_SENSOR_VIRTUAL = 2
        CURRENT_SENSOR_FAKE = 3
        CURRENT_SENSOR_ESC = 4
        CURRENT_SENSOR_MAX = CURRENT_SENSOR_FAKE

    class displayportMspCommand_e(enum.IntEnum):
        MSP_DP_HEARTBEAT = 0
        MSP_DP_RELEASE = 1
        MSP_DP_CLEAR_SCREEN = 2
        MSP_DP_WRITE_STRING = 3
        MSP_DP_DRAW_SCREEN = 4
        MSP_DP_OPTIONS = 5
        MSP_DP_SYS = 6
        MSP_DP_COUNT = 7

    class dumpFlags_e(enum.IntEnum):
        DUMP_MASTER = (1 << 0)
        DUMP_CONTROL_PROFILE = (1 << 1)
        DUMP_BATTERY_PROFILE = (1 << 2)
        DUMP_MIXER_PROFILE = (1 << 3)
        DUMP_ALL = (1 << 4)
        DO_DIFF = (1 << 5)
        SHOW_DEFAULTS = (1 << 6)
        HIDE_UNUSED = (1 << 7)

    class dynamicGyroNotchMode_e(enum.IntEnum):
        DYNAMIC_NOTCH_MODE_2D = 0
        DYNAMIC_NOTCH_MODE_3D = 1

    class emergLandState_e(enum.IntEnum):
        EMERG_LAND_IDLE = 0
        EMERG_LAND_IN_PROGRESS = 1
        EMERG_LAND_HAS_LANDED = 2

    class escSensorFrameStatus_t(enum.IntEnum):
        ESC_SENSOR_FRAME_PENDING = 0
        ESC_SENSOR_FRAME_COMPLETE = 1
        ESC_SENSOR_FRAME_FAILED = 2

    class escSensorState_t(enum.IntEnum):
        ESC_SENSOR_WAIT_STARTUP = 0
        ESC_SENSOR_READY = 1
        ESC_SENSOR_WAITING = 2

    class failsafeChannelBehavior_e(enum.IntEnum):
        FAILSAFE_CHANNEL_HOLD = 0
        FAILSAFE_CHANNEL_NEUTRAL = 1

    class failsafePhase_e(enum.IntEnum):
        FAILSAFE_IDLE = 0
        FAILSAFE_RX_LOSS_DETECTED = 1
        FAILSAFE_RX_LOSS_IDLE = 2
        FAILSAFE_RETURN_TO_HOME = 3
        FAILSAFE_LANDING = 4
        FAILSAFE_LANDED = 5
        FAILSAFE_RX_LOSS_MONITORING = 6
        FAILSAFE_RX_LOSS_RECOVERED = 7

    class failsafeProcedure_e(enum.IntEnum):
        FAILSAFE_PROCEDURE_AUTO_LANDING = 0
        FAILSAFE_PROCEDURE_DROP_IT = 1
        FAILSAFE_PROCEDURE_RTH = 2
        FAILSAFE_PROCEDURE_NONE = 3

    class failsafeRxLinkState_e(enum.IntEnum):
        FAILSAFE_RXLINK_DOWN = 0
        FAILSAFE_RXLINK_UP = 1

    class fatFilesystemType_e(enum.IntEnum):
        FAT_FILESYSTEM_TYPE_INVALID = 0
        FAT_FILESYSTEM_TYPE_FAT12 = 1
        FAT_FILESYSTEM_TYPE_FAT16 = 2
        FAT_FILESYSTEM_TYPE_FAT32 = 3

    class features_e(enum.IntEnum):
        FEATURE_THR_VBAT_COMP = 1 << 0
        FEATURE_VBAT = 1 << 1
        FEATURE_TX_PROF_SEL = 1 << 2
        FEATURE_BAT_PROFILE_AUTOSWITCH = 1 << 3
        FEATURE_GEOZONE = 1 << 4
        FEATURE_UNUSED_1 = 1 << 5
        FEATURE_SOFTSERIAL = 1 << 6
        FEATURE_GPS = 1 << 7
        FEATURE_UNUSED_3 = 1 << 8
        FEATURE_UNUSED_4 = 1 << 9
        FEATURE_TELEMETRY = 1 << 10
        FEATURE_CURRENT_METER = 1 << 11
        FEATURE_REVERSIBLE_MOTORS = 1 << 12
        FEATURE_UNUSED_5 = 1 << 13
        FEATURE_UNUSED_6 = 1 << 14
        FEATURE_RSSI_ADC = 1 << 15
        FEATURE_LED_STRIP = 1 << 16
        FEATURE_DASHBOARD = 1 << 17
        FEATURE_UNUSED_7 = 1 << 18
        FEATURE_BLACKBOX = 1 << 19
        FEATURE_UNUSED_10 = 1 << 20
        FEATURE_TRANSPONDER = 1 << 21
        FEATURE_AIRMODE = 1 << 22
        FEATURE_SUPEREXPO_RATES = 1 << 23
        FEATURE_VTX = 1 << 24
        FEATURE_UNUSED_8 = 1 << 25
        FEATURE_UNUSED_9 = 1 << 26
        FEATURE_UNUSED_11 = 1 << 27
        FEATURE_PWM_OUTPUT_ENABLE = 1 << 28
        FEATURE_OSD = 1 << 29
        FEATURE_FW_LAUNCH = 1 << 30
        FEATURE_FW_AUTOTRIM = 1 << 31

    class flightModeFlags_e(enum.IntEnum):
        ANGLE_MODE = (1 << 0)
        HORIZON_MODE = (1 << 1)
        HEADING_MODE = (1 << 2)
        NAV_ALTHOLD_MODE = (1 << 3)
        NAV_RTH_MODE = (1 << 4)
        NAV_POSHOLD_MODE = (1 << 5)
        HEADFREE_MODE = (1 << 6)
        NAV_LAUNCH_MODE = (1 << 7)
        MANUAL_MODE = (1 << 8)
        FAILSAFE_MODE = (1 << 9)
        AUTO_TUNE = (1 << 10)
        NAV_WP_MODE = (1 << 11)
        NAV_COURSE_HOLD_MODE = (1 << 12)
        FLAPERON = (1 << 13)
        TURN_ASSISTANT = (1 << 14)
        TURTLE_MODE = (1 << 15)
        SOARING_MODE = (1 << 16)
        ANGLEHOLD_MODE = (1 << 17)
        NAV_FW_AUTOLAND = (1 << 18)
        NAV_SEND_TO = (1 << 19)

    class flightModeForTelemetry_e(enum.IntEnum):
        FLM_MANUAL = 0
        FLM_ACRO = 1
        FLM_ACRO_AIR = 2
        FLM_ANGLE = 3
        FLM_HORIZON = 4
        FLM_ALTITUDE_HOLD = 5
        FLM_POSITION_HOLD = 6
        FLM_RTH = 7
        FLM_MISSION = 8
        FLM_COURSE_HOLD = 9
        FLM_CRUISE = 10
        FLM_LAUNCH = 11
        FLM_FAILSAFE = 12
        FLM_ANGLEHOLD = 13
        FLM_COUNT = 14

    class flyingPlatformType_e(enum.IntEnum):
        PLATFORM_MULTIROTOR = 0
        PLATFORM_AIRPLANE = 1
        PLATFORM_HELICOPTER = 2
        PLATFORM_TRICOPTER = 3
        PLATFORM_ROVER = 4
        PLATFORM_BOAT = 5

    class fport2_control_frame_type_e(enum.IntEnum):
        CFT_RC = 0xFF
        CFT_OTA_START = 0xF0
        CFT_OTA_DATA = 0xF1
        CFT_OTA_STOP = 0xF2

    class frame_state_e(enum.IntEnum):
        FS_CONTROL_FRAME_START = 0
        FS_CONTROL_FRAME_TYPE = 1
        FS_CONTROL_FRAME_DATA = 2
        FS_DOWNLINK_FRAME_START = 3
        FS_DOWNLINK_FRAME_DATA = 4

    class frame_type_e(enum.IntEnum):
        FT_CONTROL = 0
        FT_DOWNLINK = 1

    class frskyOSDColor_e(enum.IntEnum):
        FRSKY_OSD_COLOR_BLACK = 0
        FRSKY_OSD_COLOR_TRANSPARENT = 1
        FRSKY_OSD_COLOR_WHITE = 2
        FRSKY_OSD_COLOR_GRAY = 3

    class frskyOSDLineOutlineType_e(enum.IntEnum):
        FRSKY_OSD_OUTLINE_TYPE_NONE = 0
        FRSKY_OSD_OUTLINE_TYPE_TOP = 1 << 0
        FRSKY_OSD_OUTLINE_TYPE_RIGHT = 1 << 1
        FRSKY_OSD_OUTLINE_TYPE_BOTTOM = 1 << 2
        FRSKY_OSD_OUTLINE_TYPE_LEFT = 1 << 3

    class frskyOSDRecvState_e(enum.IntEnum):
        RECV_STATE_NONE = 0
        RECV_STATE_SYNC = 1
        RECV_STATE_LENGTH = 2
        RECV_STATE_DATA = 3
        RECV_STATE_CHECKSUM = 4
        RECV_STATE_DONE = 5

    class frskyOSDTransactionOptions_e(enum.IntEnum):
        FRSKY_OSD_TRANSACTION_OPT_PROFILED = 1 << 0
        FRSKY_OSD_TRANSACTION_OPT_RESET_DRAWING = 1 << 1

    class frskyOSDWidgetID_e(enum.IntEnum):
        FRSKY_OSD_WIDGET_ID_AHI = 0
        FRSKY_OSD_WIDGET_ID_SIDEBAR_0 = 1
        FRSKY_OSD_WIDGET_ID_SIDEBAR_1 = 2
        FRSKY_OSD_WIDGET_ID_GRAPH_0 = 3
        FRSKY_OSD_WIDGET_ID_GRAPH_1 = 4
        FRSKY_OSD_WIDGET_ID_GRAPH_2 = 5
        FRSKY_OSD_WIDGET_ID_GRAPH_3 = 6
        FRSKY_OSD_WIDGET_ID_CHARGAUGE_0 = 7
        FRSKY_OSD_WIDGET_ID_CHARGAUGE_1 = 8
        FRSKY_OSD_WIDGET_ID_CHARGAUGE_2 = 9
        FRSKY_OSD_WIDGET_ID_CHARGAUGE_3 = 10
        FRSKY_OSD_WIDGET_ID_SIDEBAR_FIRST = FRSKY_OSD_WIDGET_ID_SIDEBAR_0
        FRSKY_OSD_WIDGET_ID_SIDEBAR_LAST = FRSKY_OSD_WIDGET_ID_SIDEBAR_1
        FRSKY_OSD_WIDGET_ID_GRAPH_FIRST = FRSKY_OSD_WIDGET_ID_GRAPH_0
        FRSKY_OSD_WIDGET_ID_GRAPH_LAST = FRSKY_OSD_WIDGET_ID_GRAPH_3
        FRSKY_OSD_WIDGET_ID_CHARGAUGE_FIRST = FRSKY_OSD_WIDGET_ID_CHARGAUGE_0
        FRSKY_OSD_WIDGET_ID_CHARGAUGE_LAST = FRSKY_OSD_WIDGET_ID_CHARGAUGE_3

    class fw_autotune_rate_adjustment_e(enum.IntEnum):
        FIXED = 0
        LIMIT = 1
        AUTO = 2

    class ghstAddr_e(enum.IntEnum):
        GHST_ADDR_RADIO = 0x80
        GHST_ADDR_TX_MODULE_SYM = 0x81
        GHST_ADDR_TX_MODULE_ASYM = 0x88
        GHST_ADDR_FC = 0x82
        GHST_ADDR_GOGGLES = 0x83
        GHST_ADDR_QUANTUM_TEE1 = 0x84
        GHST_ADDR_QUANTUM_TEE2 = 0x85
        GHST_ADDR_QUANTUM_GW1 = 0x86
        GHST_ADDR_5G_CLK = 0x87
        GHST_ADDR_RX = 0x89

    class ghstDl_e(enum.IntEnum):
        GHST_DL_OPENTX_SYNC = 0x20
        GHST_DL_LINK_STAT = 0x21
        GHST_DL_VTX_STAT = 0x22
        GHST_DL_PACK_STAT = 0x23
        GHST_DL_GPS_PRIMARY = 0x25
        GHST_DL_GPS_SECONDARY = 0x26

    class ghstFrameTypeIndex_e(enum.IntEnum):
        GHST_FRAME_START_INDEX = 0
        GHST_FRAME_PACK_INDEX = GHST_FRAME_START_INDEX

    class ghstUl_e(enum.IntEnum):
        GHST_UL_RC_CHANS_HS4_FIRST = 0x10
        GHST_UL_RC_CHANS_HS4_5TO8 = 0x10
        GHST_UL_RC_CHANS_HS4_9TO12 = 0x11
        GHST_UL_RC_CHANS_HS4_13TO16 = 0x12
        GHST_UL_RC_CHANS_HS4_RSSI = 0x13
        GHST_UL_RC_CHANS_HS4_LAST = 0x1

    class gimbalHeadtrackerState_e(enum.IntEnum):
        WAITING_HDR1 = 0
        WAITING_HDR2 = 1
        WAITING_PAYLOAD = 2
        WAITING_CRCH = 3
        WAITING_CRCL = 4

    class gpsAutoBaud_e(enum.IntEnum):
        GPS_AUTOBAUD_OFF = 0
        GPS_AUTOBAUD_ON = 1

    class gpsAutoConfig_e(enum.IntEnum):
        GPS_AUTOCONFIG_OFF = 0
        GPS_AUTOCONFIG_ON = 1

    class gpsBaudRate_e(enum.IntEnum):
        GPS_BAUDRATE_115200 = 0
        GPS_BAUDRATE_57600 = 1
        GPS_BAUDRATE_38400 = 2
        GPS_BAUDRATE_19200 = 3
        GPS_BAUDRATE_9600 = 4
        GPS_BAUDRATE_230400 = 5
        GPS_BAUDRATE_460800 = 6
        GPS_BAUDRATE_921600 = 7
        GPS_BAUDRATE_COUNT = 8

    class gpsDynModel_e(enum.IntEnum):
        GPS_DYNMODEL_PEDESTRIAN = 0
        GPS_DYNMODEL_AUTOMOTIVE = 1
        GPS_DYNMODEL_AIR_1G = 2
        GPS_DYNMODEL_AIR_2G = 3
        GPS_DYNMODEL_AIR_4G = 4
        GPS_DYNMODEL_SEA = 5
        GPS_DYNMODEL_MOWER = 6

    class gpsFixChar_e(enum.IntEnum):
        GPS_FIX_CHAR_NONE = "-"
        GPS_FIX_CHAR_2D = "2"
        GPS_FIX_CHAR_3D = "3"
        GPS_FIX_CHAR_DGPS = "D"

    class gpsFixType_e(enum.IntEnum):
        GPS_NO_FIX = 0
        GPS_FIX_2D = 1
        GPS_FIX_3D = 2

    class gpsProvider_e(enum.IntEnum):
        GPS_UBLOX = 0
        GPS_MSP = 1
        GPS_FAKE = 2
        GPS_PROVIDER_COUNT = 3

    class gpsState_e(enum.IntEnum):
        GPS_UNKNOWN = 0
        GPS_INITIALIZING = 1
        GPS_RUNNING = 2
        GPS_LOST_COMMUNICATION = 3

    class gyroFilterMode_e(enum.IntEnum):
        GYRO_FILTER_MODE_OFF = 0
        GYRO_FILTER_MODE_STATIC = 1
        GYRO_FILTER_MODE_DYNAMIC = 2
        GYRO_FILTER_MODE_ADAPTIVE = 3

    class gyroSensor_e(enum.IntEnum):
        GYRO_NONE = 0
        GYRO_AUTODETECT = 1
        GYRO_MPU6000 = 2
        GYRO_MPU6500 = 3
        GYRO_MPU9250 = 4
        GYRO_BMI160 = 5
        GYRO_ICM20689 = 6
        GYRO_BMI088 = 7
        GYRO_ICM42605 = 8
        GYRO_BMI270 = 9
        GYRO_LSM6DXX = 10
        GYRO_FAKE = 11

    class hardwareSensorStatus_e(enum.IntEnum):
        HW_SENSOR_NONE = 0
        HW_SENSOR_OK = 1
        HW_SENSOR_UNAVAILABLE = 2
        HW_SENSOR_UNHEALTHY = 3

    class hottEamAlarm1Flag_e(enum.IntEnum):
        HOTT_EAM_ALARM1_FLAG_NONE = 0
        HOTT_EAM_ALARM1_FLAG_MAH = (1 << 0)
        HOTT_EAM_ALARM1_FLAG_BATTERY_1 = (1 << 1)
        HOTT_EAM_ALARM1_FLAG_BATTERY_2 = (1 << 2)
        HOTT_EAM_ALARM1_FLAG_TEMPERATURE_1 = (1 << 3)
        HOTT_EAM_ALARM1_FLAG_TEMPERATURE_2 = (1 << 4)
        HOTT_EAM_ALARM1_FLAG_ALTITUDE = (1 << 5)
        HOTT_EAM_ALARM1_FLAG_CURRENT = (1 << 6)
        HOTT_EAM_ALARM1_FLAG_MAIN_VOLTAGE = (1 << 7)

    class hottEamAlarm2Flag_e(enum.IntEnum):
        HOTT_EAM_ALARM2_FLAG_NONE = 0
        HOTT_EAM_ALARM2_FLAG_MS = (1 << 0)
        HOTT_EAM_ALARM2_FLAG_M3S = (1 << 1)
        HOTT_EAM_ALARM2_FLAG_ALTITUDE_DUPLICATE = (1 << 2)
        HOTT_EAM_ALARM2_FLAG_MS_DUPLICATE = (1 << 3)
        HOTT_EAM_ALARM2_FLAG_M3S_DUPLICATE = (1 << 4)
        HOTT_EAM_ALARM2_FLAG_UNKNOWN_1 = (1 << 5)
        HOTT_EAM_ALARM2_FLAG_UNKNOWN_2 = (1 << 6)
        HOTT_EAM_ALARM2_FLAG_ON_SIGN_OR_TEXT_ACTIVE = (1 << 7)

    class hottState_e(enum.IntEnum):
        HOTT_WAITING_FOR_REQUEST = 0
        HOTT_RECEIVING_REQUEST = 1
        HOTT_WAITING_FOR_TX_WINDOW = 2
        HOTT_TRANSMITTING = 3
        HOTT_ENDING_TRANSMISSION = 4

    class ibusCommand_e(enum.IntEnum):
        IBUS_COMMAND_DISCOVER_SENSOR = 0x80
        IBUS_COMMAND_SENSOR_TYPE = 0x90
        IBUS_COMMAND_MEASUREMENT = 0xA0

    class ibusSensorType1_e(enum.IntEnum):
        IBUS_MEAS_TYPE1_INTV = 0x00
        IBUS_MEAS_TYPE1_TEM = 0x01
        IBUS_MEAS_TYPE1_MOT = 0x02
        IBUS_MEAS_TYPE1_EXTV = 0x03
        IBUS_MEAS_TYPE1_CELL = 0x04
        IBUS_MEAS_TYPE1_BAT_CURR = 0x05
        IBUS_MEAS_TYPE1_FUEL = 0x06
        IBUS_MEAS_TYPE1_RPM = 0x07
        IBUS_MEAS_TYPE1_CMP_HEAD = 0x08
        IBUS_MEAS_TYPE1_CLIMB_RATE = 0x09
        IBUS_MEAS_TYPE1_COG = 0x0a
        IBUS_MEAS_TYPE1_GPS_STATUS = 0x0b
        IBUS_MEAS_TYPE1_ACC_X = 0x0c
        IBUS_MEAS_TYPE1_ACC_Y = 0x0d
        IBUS_MEAS_TYPE1_ACC_Z = 0x0e
        IBUS_MEAS_TYPE1_ROLL = 0x0
        IBUS_MEAS_TYPE1_PITCH = 0x10
        IBUS_MEAS_TYPE1_YAW = 0x11
        IBUS_MEAS_TYPE1_VERTICAL_SPEED = 0x12
        IBUS_MEAS_TYPE1_GROUND_SPEED = 0x13
        IBUS_MEAS_TYPE1_GPS_DIST = 0x14
        IBUS_MEAS_TYPE1_ARMED = 0x15
        IBUS_MEAS_TYPE1_FLIGHT_MODE = 0x16
        IBUS_MEAS_TYPE1_PRES = 0x41
        IBUS_MEAS_TYPE1_SPE = 0x7e
        IBUS_MEAS_TYPE1_GPS_LAT = 0x80
        IBUS_MEAS_TYPE1_GPS_LON = 0x81
        IBUS_MEAS_TYPE1_GPS_ALT = 0x82
        IBUS_MEAS_TYPE1_ALT = 0x83
        IBUS_MEAS_TYPE1_S84 = 0x84
        IBUS_MEAS_TYPE1_S85 = 0x85
        IBUS_MEAS_TYPE1_S86 = 0x86
        IBUS_MEAS_TYPE1_S87 = 0x87
        IBUS_MEAS_TYPE1_S88 = 0x88
        IBUS_MEAS_TYPE1_S89 = 0x89
        IBUS_MEAS_TYPE1_S8a = 0x8a

    class ibusSensorType_e(enum.IntEnum):
        IBUS_MEAS_TYPE_INTERNAL_VOLTAGE = 0x00
        IBUS_MEAS_TYPE_TEMPERATURE = 0x01
        IBUS_MEAS_TYPE_RPM = 0x02
        IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE = 0x03
        IBUS_MEAS_TYPE_HEADING = 0x04
        IBUS_MEAS_TYPE_CURRENT = 0x05
        IBUS_MEAS_TYPE_CLIMB = 0x06
        IBUS_MEAS_TYPE_ACC_Z = 0x07
        IBUS_MEAS_TYPE_ACC_Y = 0x08
        IBUS_MEAS_TYPE_ACC_X = 0x09
        IBUS_MEAS_TYPE_VSPEED = 0x0a
        IBUS_MEAS_TYPE_SPEED = 0x0b
        IBUS_MEAS_TYPE_DIST = 0x0c
        IBUS_MEAS_TYPE_ARMED = 0x0d
        IBUS_MEAS_TYPE_MODE = 0x0e
        IBUS_MEAS_TYPE_PRES = 0x41
        IBUS_MEAS_TYPE_SPE = 0x7e
        IBUS_MEAS_TYPE_COG = 0x80
        IBUS_MEAS_TYPE_GPS_STATUS = 0x81
        IBUS_MEAS_TYPE_GPS_LON = 0x82
        IBUS_MEAS_TYPE_GPS_LAT = 0x83
        IBUS_MEAS_TYPE_ALT = 0x84
        IBUS_MEAS_TYPE_S85 = 0x85
        IBUS_MEAS_TYPE_S86 = 0x86
        IBUS_MEAS_TYPE_S87 = 0x87
        IBUS_MEAS_TYPE_S88 = 0x88
        IBUS_MEAS_TYPE_S89 = 0x89
        IBUS_MEAS_TYPE_S8A = 0x8A
        IBUS_MEAS_TYPE_GALT = 0xf9
        IBUS_MEAS_TYPE_GPS = 0xfd

    class ibusSensorValue_e(enum.IntEnum):
        IBUS_MEAS_VALUE_NONE = 0x00
        IBUS_MEAS_VALUE_TEMPERATURE = 0x01
        IBUS_MEAS_VALUE_MOT = 0x02
        IBUS_MEAS_VALUE_EXTERNAL_VOLTAGE = 0x03
        IBUS_MEAS_VALUE_CELL = 0x04
        IBUS_MEAS_VALUE_CURRENT = 0x05
        IBUS_MEAS_VALUE_FUEL = 0x06
        IBUS_MEAS_VALUE_RPM = 0x07
        IBUS_MEAS_VALUE_HEADING = 0x08
        IBUS_MEAS_VALUE_CLIMB = 0x09
        IBUS_MEAS_VALUE_COG = 0x0a
        IBUS_MEAS_VALUE_GPS_STATUS = 0x0b
        IBUS_MEAS_VALUE_ACC_X = 0x0c
        IBUS_MEAS_VALUE_ACC_Y = 0x0d
        IBUS_MEAS_VALUE_ACC_Z = 0x0e
        IBUS_MEAS_VALUE_ROLL = 0x0
        IBUS_MEAS_VALUE_PITCH = 0x10
        IBUS_MEAS_VALUE_YAW = 0x11
        IBUS_MEAS_VALUE_VSPEED = 0x12
        IBUS_MEAS_VALUE_SPEED = 0x13
        IBUS_MEAS_VALUE_DIST = 0x14
        IBUS_MEAS_VALUE_ARMED = 0x15
        IBUS_MEAS_VALUE_MODE = 0x16
        IBUS_MEAS_VALUE_PRES = 0x41
        IBUS_MEAS_VALUE_SPE = 0x7e
        IBUS_MEAS_VALUE_GPS_LAT = 0x80
        IBUS_MEAS_VALUE_GPS_LON = 0x81
        IBUS_MEAS_VALUE_GALT4 = 0x82
        IBUS_MEAS_VALUE_ALT4 = 0x83
        IBUS_MEAS_VALUE_GALT = 0x84
        IBUS_MEAS_VALUE_ALT = 0x85
        IBUS_MEAS_VALUE_STATUS = 0x87
        IBUS_MEAS_VALUE_GPS_LAT1 = 0x88
        IBUS_MEAS_VALUE_GPS_LON1 = 0x89
        IBUS_MEAS_VALUE_GPS_LAT2 = 0x90
        IBUS_MEAS_VALUE_GPS_LON2 = 0x91
        IBUS_MEAS_VALUE_GPS = 0xfd

    class imu_inertia_comp_method_e(enum.IntEnum):
        COMPMETHOD_VELNED = 0
        COMPMETHOD_TURNRATE = 1
        COMPMETHOD_ADAPTIVE = 2

    class inputSource_e(enum.IntEnum):
        INPUT_STABILIZED_ROLL = 0
        INPUT_STABILIZED_PITCH = 1
        INPUT_STABILIZED_YAW = 2
        INPUT_STABILIZED_THROTTLE = 3
        INPUT_RC_ROLL = 4
        INPUT_RC_PITCH = 5
        INPUT_RC_YAW = 6
        INPUT_RC_THROTTLE = 7
        INPUT_RC_CH5 = 8
        INPUT_RC_CH6 = 9
        INPUT_RC_CH7 = 10
        INPUT_RC_CH8 = 11
        INPUT_GIMBAL_PITCH = 12
        INPUT_GIMBAL_ROLL = 13
        INPUT_FEATURE_FLAPS = 14
        INPUT_RC_CH9 = 15
        INPUT_RC_CH10 = 16
        INPUT_RC_CH11 = 17
        INPUT_RC_CH12 = 18
        INPUT_RC_CH13 = 19
        INPUT_RC_CH14 = 20
        INPUT_RC_CH15 = 21
        INPUT_RC_CH16 = 22
        INPUT_STABILIZED_ROLL_PLUS = 23
        INPUT_STABILIZED_ROLL_MINUS = 24
        INPUT_STABILIZED_PITCH_PLUS = 25
        INPUT_STABILIZED_PITCH_MINUS = 26
        INPUT_STABILIZED_YAW_PLUS = 27
        INPUT_STABILIZED_YAW_MINUS = 28
        INPUT_MAX = 29
        INPUT_GVAR_0 = 30
        INPUT_GVAR_1 = 31
        INPUT_GVAR_2 = 32
        INPUT_GVAR_3 = 33
        INPUT_GVAR_4 = 34
        INPUT_GVAR_5 = 35
        INPUT_GVAR_6 = 36
        INPUT_GVAR_7 = 37
        INPUT_MIXER_TRANSITION = 38
        INPUT_HEADTRACKER_PAN = 39
        INPUT_HEADTRACKER_TILT = 40
        INPUT_HEADTRACKER_ROLL = 41
        INPUT_RC_CH17 = 42
        INPUT_RC_CH18 = 43
        INPUT_RC_CH19 = 44
        INPUT_RC_CH20 = 45
        INPUT_RC_CH21 = 46
        INPUT_RC_CH22 = 47
        INPUT_RC_CH23 = 48
        INPUT_RC_CH24 = 49
        INPUT_RC_CH25 = 50
        INPUT_RC_CH26 = 51
        INPUT_RC_CH27 = 52
        INPUT_RC_CH28 = 53
        INPUT_RC_CH29 = 54
        INPUT_RC_CH30 = 55
        INPUT_RC_CH31 = 56
        INPUT_RC_CH32 = 57
        INPUT_RC_CH33 = 58
        INPUT_RC_CH34 = 59
        INPUT_SOURCE_COUNT = 60

    class itermRelax_e(enum.IntEnum):
        ITERM_RELAX_OFF = 0
        ITERM_RELAX_RP = 1
        ITERM_RELAX_RPY = 2

    class ledBaseFunctionId_e(enum.IntEnum):
        LED_FUNCTION_COLOR = 0
        LED_FUNCTION_FLIGHT_MODE = 1
        LED_FUNCTION_ARM_STATE = 2
        LED_FUNCTION_BATTERY = 3
        LED_FUNCTION_RSSI = 4
        LED_FUNCTION_GPS = 5
        LED_FUNCTION_THRUST_RING = 6
        LED_FUNCTION_CHANNEL = 7

    class ledDirectionId_e(enum.IntEnum):
        LED_DIRECTION_NORTH = 0
        LED_DIRECTION_EAST = 1
        LED_DIRECTION_SOUTH = 2
        LED_DIRECTION_WEST = 3
        LED_DIRECTION_UP = 4
        LED_DIRECTION_DOWN = 5

    class ledModeIndex_e(enum.IntEnum):
        LED_MODE_ORIENTATION = 0
        LED_MODE_HEADFREE = 1
        LED_MODE_HORIZON = 2
        LED_MODE_ANGLE = 3
        LED_MODE_MAG = 4
        LED_MODE_BARO = 5
        LED_SPECIAL = 6

    class ledOverlayId_e(enum.IntEnum):
        LED_OVERLAY_THROTTLE = 0
        LED_OVERLAY_LARSON_SCANNER = 1
        LED_OVERLAY_BLINK = 2
        LED_OVERLAY_LANDING_FLASH = 3
        LED_OVERLAY_INDICATOR = 4
        LED_OVERLAY_WARNING = 5
        LED_OVERLAY_STROBE = 6

    class ledSpecialColorIds_e(enum.IntEnum):
        LED_SCOLOR_DISARMED = 0
        LED_SCOLOR_ARMED = 1
        LED_SCOLOR_ANIMATION = 2
        LED_SCOLOR_BACKGROUND = 3
        LED_SCOLOR_BLINKBACKGROUND = 4
        LED_SCOLOR_GPSNOSATS = 5
        LED_SCOLOR_GPSNOLOCK = 6
        LED_SCOLOR_GPSLOCKED = 7
        LED_SCOLOR_STROBE = 8

    class logicConditionFlags_e(enum.IntEnum):
        LOGIC_CONDITION_FLAG_LATCH = 1 << 0
        LOGIC_CONDITION_FLAG_TIMEOUT_SATISFIED = 1 << 1

    class logicConditionsGlobalFlags_t(enum.IntEnum):
        LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_ARMING_SAFETY = (1 << 0)
        LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE_SCALE = (1 << 1)
        LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_SWAP_ROLL_YAW = (1 << 2)
        LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_ROLL = (1 << 3)
        LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_PITCH = (1 << 4)
        LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_YAW = (1 << 5)
        LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE = (1 << 6)
        LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_OSD_LAYOUT = (1 << 7)
        LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_RC_CHANNEL = (1 << 8)
        LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_LOITER_RADIUS = (1 << 9)
        LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_FLIGHT_AXIS = (1 << 10)
        LOGIC_CONDITION_GLOBAL_FLAG_DISABLE_GPS_FIX = (1 << 11)

    class logicFlightModeOperands_e(enum.IntEnum):
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_FAILSAFE = 0
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_MANUAL = 1
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_RTH = 2
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_POSHOLD = 3
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_CRUISE = 4
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ALTHOLD = 5
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ANGLE = 6
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_HORIZON = 7
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_AIR = 8
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER1 = 9
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER2 = 10
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_COURSE_HOLD = 11
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER3 = 12
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER4 = 13
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ACRO = 14
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_WAYPOINT_MISSION = 15
        LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ANGLEHOLD = 16

    class logicFlightOperands_e(enum.IntEnum):
        LOGIC_CONDITION_OPERAND_FLIGHT_ARM_TIMER = 0
        LOGIC_CONDITION_OPERAND_FLIGHT_HOME_DISTANCE = 1
        LOGIC_CONDITION_OPERAND_FLIGHT_TRIP_DISTANCE = 2
        LOGIC_CONDITION_OPERAND_FLIGHT_RSSI = 3
        LOGIC_CONDITION_OPERAND_FLIGHT_VBAT = 4
        LOGIC_CONDITION_OPERAND_FLIGHT_CELL_VOLTAGE = 5
        LOGIC_CONDITION_OPERAND_FLIGHT_CURRENT = 6
        LOGIC_CONDITION_OPERAND_FLIGHT_MAH_DRAWN = 7
        LOGIC_CONDITION_OPERAND_FLIGHT_GPS_SATS = 8
        LOGIC_CONDITION_OPERAND_FLIGHT_GROUD_SPEED = 9
        LOGIC_CONDITION_OPERAND_FLIGHT_3D_SPEED = 10
        LOGIC_CONDITION_OPERAND_FLIGHT_AIR_SPEED = 11
        LOGIC_CONDITION_OPERAND_FLIGHT_ALTITUDE = 12
        LOGIC_CONDITION_OPERAND_FLIGHT_VERTICAL_SPEED = 13
        LOGIC_CONDITION_OPERAND_FLIGHT_TROTTLE_POS = 14
        LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_ROLL = 15
        LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_PITCH = 16
        LOGIC_CONDITION_OPERAND_FLIGHT_IS_ARMED = 17
        LOGIC_CONDITION_OPERAND_FLIGHT_IS_AUTOLAUNCH = 18
        LOGIC_CONDITION_OPERAND_FLIGHT_IS_ALTITUDE_CONTROL = 19
        LOGIC_CONDITION_OPERAND_FLIGHT_IS_POSITION_CONTROL = 20
        LOGIC_CONDITION_OPERAND_FLIGHT_IS_EMERGENCY_LANDING = 21
        LOGIC_CONDITION_OPERAND_FLIGHT_IS_RTH = 22
        LOGIC_CONDITION_OPERAND_FLIGHT_IS_LANDING = 23
        LOGIC_CONDITION_OPERAND_FLIGHT_IS_FAILSAFE = 24
        LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_ROLL = 25
        LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_PITCH = 26
        LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_YAW = 27
        LOGIC_CONDITION_OPERAND_FLIGHT_3D_HOME_DISTANCE = 28
        LOGIC_CONDITION_OPERAND_FLIGHT_LQ_UPLINK = 29
        LOGIC_CONDITION_OPERAND_FLIGHT_SNR = 30
        LOGIC_CONDITION_OPERAND_FLIGHT_GPS_VALID = 31
        LOGIC_CONDITION_OPERAND_FLIGHT_LOITER_RADIUS = 32
        LOGIC_CONDITION_OPERAND_FLIGHT_ACTIVE_PROFILE = 33
        LOGIC_CONDITION_OPERAND_FLIGHT_BATT_CELLS = 34
        LOGIC_CONDITION_OPERAND_FLIGHT_AGL_STATUS = 35
        LOGIC_CONDITION_OPERAND_FLIGHT_AGL = 36
        LOGIC_CONDITION_OPERAND_FLIGHT_RANGEFINDER_RAW = 37
        LOGIC_CONDITION_OPERAND_FLIGHT_ACTIVE_MIXER_PROFILE = 38
        LOGIC_CONDITION_OPERAND_FLIGHT_MIXER_TRANSITION_ACTIVE = 39
        LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_YAW = 40
        LOGIC_CONDITION_OPERAND_FLIGHT_FW_LAND_STATE = 41
        LOGIC_CONDITION_OPERAND_FLIGHT_BATT_PROFILE = 42
        LOGIC_CONDITION_OPERAND_FLIGHT_FLOWN_LOITER_RADIUS = 43
        LOGIC_CONDITION_OPERAND_FLIGHT_LQ_DOWNLINK = 44
        LOGIC_CONDITION_OPERAND_FLIGHT_UPLINK_RSSI_DBM = 45

    class logicOperation_e(enum.IntEnum):
        LOGIC_CONDITION_TRUE = 0
        LOGIC_CONDITION_EQUAL = 1
        LOGIC_CONDITION_GREATER_THAN = 2
        LOGIC_CONDITION_LOWER_THAN = 3
        LOGIC_CONDITION_LOW = 4
        LOGIC_CONDITION_MID = 5
        LOGIC_CONDITION_HIGH = 6
        LOGIC_CONDITION_AND = 7
        LOGIC_CONDITION_OR = 8
        LOGIC_CONDITION_XOR = 9
        LOGIC_CONDITION_NAND = 10
        LOGIC_CONDITION_NOR = 11
        LOGIC_CONDITION_NOT = 12
        LOGIC_CONDITION_STICKY = 13
        LOGIC_CONDITION_ADD = 14
        LOGIC_CONDITION_SUB = 15
        LOGIC_CONDITION_MUL = 16
        LOGIC_CONDITION_DIV = 17
        LOGIC_CONDITION_GVAR_SET = 18
        LOGIC_CONDITION_GVAR_INC = 19
        LOGIC_CONDITION_GVAR_DEC = 20
        LOGIC_CONDITION_PORT_SET = 21
        LOGIC_CONDITION_OVERRIDE_ARMING_SAFETY = 22
        LOGIC_CONDITION_OVERRIDE_THROTTLE_SCALE = 23
        LOGIC_CONDITION_SWAP_ROLL_YAW = 24
        LOGIC_CONDITION_SET_VTX_POWER_LEVEL = 25
        LOGIC_CONDITION_INVERT_ROLL = 26
        LOGIC_CONDITION_INVERT_PITCH = 27
        LOGIC_CONDITION_INVERT_YAW = 28
        LOGIC_CONDITION_OVERRIDE_THROTTLE = 29
        LOGIC_CONDITION_SET_VTX_BAND = 30
        LOGIC_CONDITION_SET_VTX_CHANNEL = 31
        LOGIC_CONDITION_SET_OSD_LAYOUT = 32
        LOGIC_CONDITION_SIN = 33
        LOGIC_CONDITION_COS = 34
        LOGIC_CONDITION_TAN = 35
        LOGIC_CONDITION_MAP_INPUT = 36
        LOGIC_CONDITION_MAP_OUTPUT = 37
        LOGIC_CONDITION_RC_CHANNEL_OVERRIDE = 38
        LOGIC_CONDITION_SET_HEADING_TARGET = 39
        LOGIC_CONDITION_MODULUS = 40
        LOGIC_CONDITION_LOITER_OVERRIDE = 41
        LOGIC_CONDITION_SET_PROFILE = 42
        LOGIC_CONDITION_MIN = 43
        LOGIC_CONDITION_MAX = 44
        LOGIC_CONDITION_FLIGHT_AXIS_ANGLE_OVERRIDE = 45
        LOGIC_CONDITION_FLIGHT_AXIS_RATE_OVERRIDE = 46
        LOGIC_CONDITION_EDGE = 47
        LOGIC_CONDITION_DELAY = 48
        LOGIC_CONDITION_TIMER = 49
        LOGIC_CONDITION_DELTA = 50
        LOGIC_CONDITION_APPROX_EQUAL = 51
        LOGIC_CONDITION_LED_PIN_PWM = 52
        LOGIC_CONDITION_DISABLE_GPS_FIX = 53
        LOGIC_CONDITION_RESET_MAG_CALIBRATION = 54
        LOGIC_CONDITION_LAST = 55

    class logicWaypointOperands_e(enum.IntEnum):
        LOGIC_CONDITION_OPERAND_WAYPOINTS_IS_WP = 0
        LOGIC_CONDITION_OPERAND_WAYPOINTS_WAYPOINT_INDEX = 1
        LOGIC_CONDITION_OPERAND_WAYPOINTS_WAYPOINT_ACTION = 2
        LOGIC_CONDITION_OPERAND_WAYPOINTS_NEXT_WAYPOINT_ACTION = 3
        LOGIC_CONDITION_OPERAND_WAYPOINTS_WAYPOINT_DISTANCE = 4
        LOGIC_CONDTIION_OPERAND_WAYPOINTS_DISTANCE_FROM_WAYPOINT = 5
        LOGIC_CONDITION_OPERAND_WAYPOINTS_USER1_ACTION = 6
        LOGIC_CONDITION_OPERAND_WAYPOINTS_USER2_ACTION = 7
        LOGIC_CONDITION_OPERAND_WAYPOINTS_USER3_ACTION = 8
        LOGIC_CONDITION_OPERAND_WAYPOINTS_USER4_ACTION = 9
        LOGIC_CONDITION_OPERAND_WAYPOINTS_USER1_ACTION_NEXT_WP = 10
        LOGIC_CONDITION_OPERAND_WAYPOINTS_USER2_ACTION_NEXT_WP = 11
        LOGIC_CONDITION_OPERAND_WAYPOINTS_USER3_ACTION_NEXT_WP = 12
        LOGIC_CONDITION_OPERAND_WAYPOINTS_USER4_ACTION_NEXT_WP = 13

    class ltmUpdateRate_e(enum.IntEnum):
        LTM_RATE_NORMAL = 0
        LTM_RATE_MEDIUM = 1
        LTM_RATE_SLOW = 2

    class ltm_frame_e(enum.IntEnum):
        LTM_FRAME_START = 0
        LTM_AFRAME = LTM_FRAME_START

    class ltm_modes_e(enum.IntEnum):
        LTM_MODE_MANUAL = 0
        LTM_MODE_RATE = 1
        LTM_MODE_ANGLE = 2
        LTM_MODE_HORIZON = 3
        LTM_MODE_ACRO = 4
        LTM_MODE_STABALIZED1 = 5
        LTM_MODE_STABALIZED2 = 6
        LTM_MODE_STABILIZED3 = 7
        LTM_MODE_ALTHOLD = 8
        LTM_MODE_GPSHOLD = 9
        LTM_MODE_WAYPOINTS = 10
        LTM_MODE_HEADHOLD = 11
        LTM_MODE_CIRCLE = 12
        LTM_MODE_RTH = 13
        LTM_MODE_FOLLOWWME = 14
        LTM_MODE_LAND = 15
        LTM_MODE_FLYBYWIRE1 = 16
        LTM_MODE_FLYBYWIRE2 = 17
        LTM_MODE_CRUISE = 18
        LTM_MODE_UNKNOWN = 19
        LTM_MODE_LAUNCH = 20
        LTM_MODE_AUTOTUNE = 21

    class magSensor_e(enum.IntEnum):
        MAG_NONE = 0
        MAG_AUTODETECT = 1
        MAG_HMC5883 = 2
        MAG_AK8975 = 3
        MAG_MAG3110 = 4
        MAG_AK8963 = 5
        MAG_IST8310 = 6
        MAG_QMC5883 = 7
        MAG_MPU9250 = 8
        MAG_IST8308 = 9
        MAG_LIS3MDL = 10
        MAG_MSP = 11
        MAG_RM3100 = 12
        MAG_VCM5883 = 13
        MAG_MLX90393 = 14
        MAG_FAKE = 15
        MAG_MAX = MAG_FAKE

    class mavlinkRadio_e(enum.IntEnum):
        MAVLINK_RADIO_GENERIC = 0
        MAVLINK_RADIO_ELRS = 1
        MAVLINK_RADIO_SIK = 2

    class mixerProfileATRequest_e(enum.IntEnum):
        MIXERAT_REQUEST_NONE = 0
        MIXERAT_REQUEST_RTH = 1
        MIXERAT_REQUEST_LAND = 2
        MIXERAT_REQUEST_ABORT = 3

    class mixerProfileATState_e(enum.IntEnum):
        MIXERAT_PHASE_IDLE = 0
        MIXERAT_PHASE_TRANSITION_INITIALIZE = 1
        MIXERAT_PHASE_TRANSITIONING = 2
        MIXERAT_PHASE_DONE = 3

    class modeActivationOperator_e(enum.IntEnum):
        MODE_OPERATOR_OR = 0
        MODE_OPERATOR_AND = 1

    class motorStatus_e(enum.IntEnum):
        MOTOR_STOPPED_USER = 0
        MOTOR_STOPPED_AUTO = 1
        MOTOR_RUNNING = 2

    class mspFlashfsFlags_e(enum.IntEnum):
        MSP_FLASHFS_BIT_READY = 1
        MSP_FLASHFS_BIT_SUPPORTED = 2

    class mspPassthroughType_e(enum.IntEnum):
        MSP_PASSTHROUGH_SERIAL_ID = 0xFD
        MSP_PASSTHROUGH_SERIAL_FUNCTION_ID = 0xFE
        MSP_PASSTHROUGH_ESC_4WAY = 0xFF

    class mspSDCardFlags_e(enum.IntEnum):
        MSP_SDCARD_FLAG_SUPPORTTED = 1

    class mspSDCardState_e(enum.IntEnum):
        MSP_SDCARD_STATE_NOT_PRESENT = 0
        MSP_SDCARD_STATE_FATAL = 1
        MSP_SDCARD_STATE_CARD_INIT = 2
        MSP_SDCARD_STATE_FS_INIT = 3
        MSP_SDCARD_STATE_READY = 4

    class multiFunctionFlags_e(enum.IntEnum):
        MF_SUSPEND_SAFEHOMES = (1 << 0)
        MF_SUSPEND_TRACKBACK = (1 << 1)
        MF_TURTLE_MODE = (1 << 2)

    class multi_function_e(enum.IntEnum):
        MULTI_FUNC_NONE = 0
        MULTI_FUNC_1 = 1
        MULTI_FUNC_2 = 2
        MULTI_FUNC_3 = 3
        MULTI_FUNC_4 = 4
        MULTI_FUNC_5 = 5
        MULTI_FUNC_6 = 6
        MULTI_FUNC_END = 7

    class opflowQuality_e(enum.IntEnum):
        OPFLOW_QUALITY_INVALID = 0
        OPFLOW_QUALITY_VALID = 1

    class opticalFlowSensor_e(enum.IntEnum):
        OPFLOW_NONE = 0
        OPFLOW_CXOF = 1
        OPFLOW_MSP = 2
        OPFLOW_FAKE = 3

    class osdCommand_e(enum.IntEnum):
        OSD_CMD_RESPONSE_ERROR = 0
        OSD_CMD_INFO = 1
        OSD_CMD_READ_FONT = 2
        OSD_CMD_WRITE_FONT = 3
        OSD_CMD_GET_CAMERA = 4
        OSD_CMD_SET_CAMERA = 5
        OSD_CMD_GET_ACTIVE_CAMERA = 6
        OSD_CMD_GET_OSD_ENABLED = 7
        OSD_CMD_SET_OSD_ENABLED = 8
        OSD_CMD_TRANSACTION_BEGIN = 16
        OSD_CMD_TRANSACTION_COMMIT = 17
        OSD_CMD_TRANSACTION_BEGIN_PROFILED = 18
        OSD_CMD_TRANSACTION_BEGIN_RESET_DRAWING = 19
        OSD_CMD_DRAWING_SET_STROKE_COLOR = 22
        OSD_CMD_DRAWING_SET_FILL_COLOR = 23
        OSD_CMD_DRAWING_SET_STROKE_AND_FILL_COLOR = 24
        OSD_CMD_DRAWING_SET_COLOR_INVERSION = 25
        OSD_CMD_DRAWING_SET_PIXEL = 26
        OSD_CMD_DRAWING_SET_PIXEL_TO_STROKE_COLOR = 27
        OSD_CMD_DRAWING_SET_PIXEL_TO_FILL_COLOR = 28
        OSD_CMD_DRAWING_SET_STROKE_WIDTH = 29
        OSD_CMD_DRAWING_SET_LINE_OUTLINE_TYPE = 30
        OSD_CMD_DRAWING_SET_LINE_OUTLINE_COLOR = 31
        OSD_CMD_DRAWING_CLIP_TO_RECT = 40
        OSD_CMD_DRAWING_CLEAR_SCREEN = 41
        OSD_CMD_DRAWING_CLEAR_RECT = 42
        OSD_CMD_DRAWING_RESET = 43
        OSD_CMD_DRAWING_DRAW_BITMAP = 44
        OSD_CMD_DRAWING_DRAW_BITMAP_MASK = 45
        OSD_CMD_DRAWING_DRAW_CHAR = 46
        OSD_CMD_DRAWING_DRAW_CHAR_MASK = 47
        OSD_CMD_DRAWING_DRAW_STRING = 48
        OSD_CMD_DRAWING_DRAW_STRING_MASK = 49
        OSD_CMD_DRAWING_MOVE_TO_POINT = 50
        OSD_CMD_DRAWING_STROKE_LINE_TO_POINT = 51
        OSD_CMD_DRAWING_STROKE_TRIANGLE = 52
        OSD_CMD_DRAWING_FILL_TRIANGLE = 53
        OSD_CMD_DRAWING_FILL_STROKE_TRIANGLE = 54
        OSD_CMD_DRAWING_STROKE_RECT = 55
        OSD_CMD_DRAWING_FILL_RECT = 56
        OSD_CMD_DRAWING_FILL_STROKE_RECT = 57
        OSD_CMD_DRAWING_STROKE_ELLIPSE_IN_RECT = 58
        OSD_CMD_DRAWING_FILL_ELLIPSE_IN_RECT = 59
        OSD_CMD_DRAWING_FILL_STROKE_ELLIPSE_IN_RECT = 60
        OSD_CMD_CTM_RESET = 80
        OSD_CMD_CTM_SET = 81
        OSD_CMD_CTM_TRANSLATE = 82
        OSD_CMD_CTM_SCALE = 83
        OSD_CMD_CTM_ROTATE = 84
        OSD_CMD_CTM_ROTATE_ABOUT = 85
        OSD_CMD_CTM_SHEAR = 86
        OSD_CMD_CTM_SHEAR_ABOUT = 87
        OSD_CMD_CTM_MULTIPLY = 88
        OSD_CMD_CONTEXT_PUSH = 100
        OSD_CMD_CONTEXT_POP = 101
        OSD_CMD_DRAW_GRID_CHR = 110
        OSD_CMD_DRAW_GRID_STR = 111
        OSD_CMD_DRAW_GRID_CHR_2 = 112
        OSD_CMD_DRAW_GRID_STR_2 = 113
        OSD_CMD_WIDGET_SET_CONFIG = 115
        OSD_CMD_WIDGET_DRAW = 116
        OSD_CMD_WIDGET_ERASE = 117
        OSD_CMD_SET_DATA_RATE = 122

    class osdCustomElementTypeVisibility_e(enum.IntEnum):
        CUSTOM_ELEMENT_VISIBILITY_ALWAYS = 0
        CUSTOM_ELEMENT_VISIBILITY_GV = 1
        CUSTOM_ELEMENT_VISIBILITY_LOGIC_CON = 2

    class osdCustomElementType_e(enum.IntEnum):
        CUSTOM_ELEMENT_TYPE_NONE = 0
        CUSTOM_ELEMENT_TYPE_TEXT = 1
        CUSTOM_ELEMENT_TYPE_ICON_STATIC = 2
        CUSTOM_ELEMENT_TYPE_ICON_GV = 3
        CUSTOM_ELEMENT_TYPE_ICON_LC = 4
        CUSTOM_ELEMENT_TYPE_GV_1 = 5
        CUSTOM_ELEMENT_TYPE_GV_2 = 6
        CUSTOM_ELEMENT_TYPE_GV_3 = 7
        CUSTOM_ELEMENT_TYPE_GV_4 = 8
        CUSTOM_ELEMENT_TYPE_GV_5 = 9
        CUSTOM_ELEMENT_TYPE_GV_FLOAT_1_1 = 10
        CUSTOM_ELEMENT_TYPE_GV_FLOAT_1_2 = 11
        CUSTOM_ELEMENT_TYPE_GV_FLOAT_2_1 = 12
        CUSTOM_ELEMENT_TYPE_GV_FLOAT_2_2 = 13
        CUSTOM_ELEMENT_TYPE_GV_FLOAT_3_1 = 14
        CUSTOM_ELEMENT_TYPE_GV_FLOAT_3_2 = 15
        CUSTOM_ELEMENT_TYPE_GV_FLOAT_4_1 = 16
        CUSTOM_ELEMENT_TYPE_LC_1 = 17
        CUSTOM_ELEMENT_TYPE_LC_2 = 18
        CUSTOM_ELEMENT_TYPE_LC_3 = 19
        CUSTOM_ELEMENT_TYPE_LC_4 = 20
        CUSTOM_ELEMENT_TYPE_LC_5 = 21
        CUSTOM_ELEMENT_TYPE_LC_FLOAT_1_1 = 22
        CUSTOM_ELEMENT_TYPE_LC_FLOAT_1_2 = 23
        CUSTOM_ELEMENT_TYPE_LC_FLOAT_2_1 = 24
        CUSTOM_ELEMENT_TYPE_LC_FLOAT_2_2 = 25
        CUSTOM_ELEMENT_TYPE_LC_FLOAT_3_1 = 26
        CUSTOM_ELEMENT_TYPE_LC_FLOAT_3_2 = 27
        CUSTOM_ELEMENT_TYPE_LC_FLOAT_4_1 = 28
        CUSTOM_ELEMENT_TYPE_END = 29

    class osdDrawPointType_e(enum.IntEnum):
        OSD_DRAW_POINT_TYPE_GRID = 0
        OSD_DRAW_POINT_TYPE_PIXEL = 1

    class osdSpeedSource_e(enum.IntEnum):
        OSD_SPEED_SOURCE_GROUND = 0
        OSD_SPEED_SOURCE_3D = 1
        OSD_SPEED_SOURCE_AIR = 2

    class osd_ahi_style_e(enum.IntEnum):
        OSD_AHI_STYLE_DEFAULT = 0
        OSD_AHI_STYLE_LINE = 1

    class osd_alignment_e(enum.IntEnum):
        OSD_ALIGN_LEFT = 0
        OSD_ALIGN_RIGHT = 1

    class osd_crosshairs_style_e(enum.IntEnum):
        OSD_CROSSHAIRS_STYLE_DEFAULT = 0
        OSD_CROSSHAIRS_STYLE_AIRCRAFT = 1
        OSD_CROSSHAIRS_STYLE_TYPE3 = 2
        OSD_CROSSHAIRS_STYLE_TYPE4 = 3
        OSD_CROSSHAIRS_STYLE_TYPE5 = 4
        OSD_CROSSHAIRS_STYLE_TYPE6 = 5
        OSD_CROSSHAIRS_STYLE_TYPE7 = 6

    class osd_crsf_lq_format_e(enum.IntEnum):
        OSD_CRSF_LQ_TYPE1 = 0
        OSD_CRSF_LQ_TYPE2 = 1
        OSD_CRSF_LQ_TYPE3 = 2

    class osd_items_e(enum.IntEnum):
        OSD_RSSI_VALUE = 0
        OSD_MAIN_BATT_VOLTAGE = 1
        OSD_CROSSHAIRS = 2
        OSD_ARTIFICIAL_HORIZON = 3
        OSD_HORIZON_SIDEBARS = 4
        OSD_ONTIME = 5
        OSD_FLYTIME = 6
        OSD_FLYMODE = 7
        OSD_CRAFT_NAME = 8
        OSD_THROTTLE_POS = 9
        OSD_VTX_CHANNEL = 10
        OSD_CURRENT_DRAW = 11
        OSD_MAH_DRAWN = 12
        OSD_GPS_SPEED = 13
        OSD_GPS_SATS = 14
        OSD_ALTITUDE = 15
        OSD_ROLL_PIDS = 16
        OSD_PITCH_PIDS = 17
        OSD_YAW_PIDS = 18
        OSD_POWER = 19
        OSD_GPS_LON = 20
        OSD_GPS_LAT = 21
        OSD_HOME_DIR = 22
        OSD_HOME_DIST = 23
        OSD_HEADING = 24
        OSD_VARIO = 25
        OSD_VARIO_NUM = 26
        OSD_AIR_SPEED = 27
        OSD_ONTIME_FLYTIME = 28
        OSD_RTC_TIME = 29
        OSD_MESSAGES = 30
        OSD_GPS_HDOP = 31
        OSD_MAIN_BATT_CELL_VOLTAGE = 32
        OSD_SCALED_THROTTLE_POS = 33
        OSD_HEADING_GRAPH = 34
        OSD_EFFICIENCY_MAH_PER_KM = 35
        OSD_WH_DRAWN = 36
        OSD_BATTERY_REMAINING_CAPACITY = 37
        OSD_BATTERY_REMAINING_PERCENT = 38
        OSD_EFFICIENCY_WH_PER_KM = 39
        OSD_TRIP_DIST = 40
        OSD_ATTITUDE_PITCH = 41
        OSD_ATTITUDE_ROLL = 42
        OSD_MAP_NORTH = 43
        OSD_MAP_TAKEOFF = 44
        OSD_RADAR = 45
        OSD_WIND_SPEED_HORIZONTAL = 46
        OSD_WIND_SPEED_VERTICAL = 47
        OSD_REMAINING_FLIGHT_TIME_BEFORE_RTH = 48
        OSD_REMAINING_DISTANCE_BEFORE_RTH = 49
        OSD_HOME_HEADING_ERROR = 50
        OSD_COURSE_HOLD_ERROR = 51
        OSD_COURSE_HOLD_ADJUSTMENT = 52
        OSD_SAG_COMPENSATED_MAIN_BATT_VOLTAGE = 53
        OSD_MAIN_BATT_SAG_COMPENSATED_CELL_VOLTAGE = 54
        OSD_POWER_SUPPLY_IMPEDANCE = 55
        OSD_LEVEL_PIDS = 56
        OSD_POS_XY_PIDS = 57
        OSD_POS_Z_PIDS = 58
        OSD_VEL_XY_PIDS = 59
        OSD_VEL_Z_PIDS = 60
        OSD_HEADING_P = 61
        OSD_BOARD_ALIGN_ROLL = 62
        OSD_BOARD_ALIGN_PITCH = 63
        OSD_RC_EXPO = 64
        OSD_RC_YAW_EXPO = 65
        OSD_THROTTLE_EXPO = 66
        OSD_PITCH_RATE = 67
        OSD_ROLL_RATE = 68
        OSD_YAW_RATE = 69
        OSD_MANUAL_RC_EXPO = 70
        OSD_MANUAL_RC_YAW_EXPO = 71
        OSD_MANUAL_PITCH_RATE = 72
        OSD_MANUAL_ROLL_RATE = 73
        OSD_MANUAL_YAW_RATE = 74
        OSD_NAV_FW_CRUISE_THR = 75
        OSD_NAV_FW_PITCH2THR = 76
        OSD_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE = 77
        OSD_DEBUG = 78
        OSD_FW_ALT_PID_OUTPUTS = 79
        OSD_FW_POS_PID_OUTPUTS = 80
        OSD_MC_VEL_X_PID_OUTPUTS = 81
        OSD_MC_VEL_Y_PID_OUTPUTS = 82
        OSD_MC_VEL_Z_PID_OUTPUTS = 83
        OSD_MC_POS_XYZ_P_OUTPUTS = 84
        OSD_3D_SPEED = 85
        OSD_IMU_TEMPERATURE = 86
        OSD_BARO_TEMPERATURE = 87
        OSD_TEMP_SENSOR_0_TEMPERATURE = 88
        OSD_TEMP_SENSOR_1_TEMPERATURE = 89
        OSD_TEMP_SENSOR_2_TEMPERATURE = 90
        OSD_TEMP_SENSOR_3_TEMPERATURE = 91
        OSD_TEMP_SENSOR_4_TEMPERATURE = 92
        OSD_TEMP_SENSOR_5_TEMPERATURE = 93
        OSD_TEMP_SENSOR_6_TEMPERATURE = 94
        OSD_TEMP_SENSOR_7_TEMPERATURE = 95
        OSD_ALTITUDE_MSL = 96
        OSD_PLUS_CODE = 97
        OSD_MAP_SCALE = 98
        OSD_MAP_REFERENCE = 99
        OSD_GFORCE = 100
        OSD_GFORCE_X = 101
        OSD_GFORCE_Y = 102
        OSD_GFORCE_Z = 103
        OSD_RC_SOURCE = 104
        OSD_VTX_POWER = 105
        OSD_ESC_RPM = 106
        OSD_ESC_TEMPERATURE = 107
        OSD_AZIMUTH = 108
        OSD_RSSI_DBM = 109
        OSD_LQ_UPLINK = 110
        OSD_SNR_DB = 111
        OSD_TX_POWER_UPLINK = 112
        OSD_GVAR_0 = 113
        OSD_GVAR_1 = 114
        OSD_GVAR_2 = 115
        OSD_GVAR_3 = 116
        OSD_TPA = 117
        OSD_NAV_FW_CONTROL_SMOOTHNESS = 118
        OSD_VERSION = 119
        OSD_RANGEFINDER = 120
        OSD_PLIMIT_REMAINING_BURST_TIME = 121
        OSD_PLIMIT_ACTIVE_CURRENT_LIMIT = 122
        OSD_PLIMIT_ACTIVE_POWER_LIMIT = 123
        OSD_GLIDESLOPE = 124
        OSD_GPS_MAX_SPEED = 125
        OSD_3D_MAX_SPEED = 126
        OSD_AIR_MAX_SPEED = 127
        OSD_ACTIVE_PROFILE = 128
        OSD_MISSION = 129
        OSD_SWITCH_INDICATOR_0 = 130
        OSD_SWITCH_INDICATOR_1 = 131
        OSD_SWITCH_INDICATOR_2 = 132
        OSD_SWITCH_INDICATOR_3 = 133
        OSD_TPA_TIME_CONSTANT = 134
        OSD_FW_LEVEL_TRIM = 135
        OSD_GLIDE_TIME_REMAINING = 136
        OSD_GLIDE_RANGE = 137
        OSD_CLIMB_EFFICIENCY = 138
        OSD_NAV_WP_MULTI_MISSION_INDEX = 139
        OSD_GROUND_COURSE = 140
        OSD_CROSS_TRACK_ERROR = 141
        OSD_PILOT_NAME = 142
        OSD_PAN_SERVO_CENTRED = 143
        OSD_MULTI_FUNCTION = 144
        OSD_ODOMETER = 145
        OSD_PILOT_LOGO = 146
        OSD_CUSTOM_ELEMENT_1 = 147
        OSD_CUSTOM_ELEMENT_2 = 148
        OSD_CUSTOM_ELEMENT_3 = 149
        OSD_ADSB_WARNING = 150
        OSD_ADSB_INFO = 151
        OSD_BLACKBOX = 152
        OSD_FORMATION_FLIGHT = 153
        OSD_CUSTOM_ELEMENT_4 = 154
        OSD_CUSTOM_ELEMENT_5 = 155
        OSD_CUSTOM_ELEMENT_6 = 156
        OSD_CUSTOM_ELEMENT_7 = 157
        OSD_CUSTOM_ELEMENT_8 = 158
        OSD_LQ_DOWNLINK = 159
        OSD_RX_POWER_DOWNLINK = 160
        OSD_RX_BAND = 161
        OSD_RX_MODE = 162
        OSD_COURSE_TO_FENCE = 163
        OSD_H_DIST_TO_FENCE = 164
        OSD_V_DIST_TO_FENCE = 165
        OSD_NAV_FW_ALT_CONTROL_RESPONSE = 166
        OSD_ITEM_COUNT = 167

    class osd_sidebar_arrow_e(enum.IntEnum):
        OSD_SIDEBAR_ARROW_NONE = 0
        OSD_SIDEBAR_ARROW_UP = 1
        OSD_SIDEBAR_ARROW_DOWN = 2

    class osd_sidebar_scroll_e(enum.IntEnum):
        OSD_SIDEBAR_SCROLL_NONE = 0
        OSD_SIDEBAR_SCROLL_ALTITUDE = 1
        OSD_SIDEBAR_SCROLL_SPEED = 2
        OSD_SIDEBAR_SCROLL_HOME_DISTANCE = 3
        OSD_SIDEBAR_SCROLL_MAX = OSD_SIDEBAR_SCROLL_HOME_DISTANCE

    class osd_stats_energy_unit_e(enum.IntEnum):
        OSD_STATS_ENERGY_UNIT_MAH = 0
        OSD_STATS_ENERGY_UNIT_WH = 1

    class osd_unit_e(enum.IntEnum):
        OSD_UNIT_IMPERIAL = 0
        OSD_UNIT_METRIC = 1
        OSD_UNIT_METRIC_MPH = 2
        OSD_UNIT_UK = 3
        OSD_UNIT_GA = 4
        OSD_UNIT_MAX = OSD_UNIT_GA

    class outputMode_e(enum.IntEnum):
        OUTPUT_MODE_AUTO = 0
        OUTPUT_MODE_MOTORS = 1
        OUTPUT_MODE_SERVOS = 2
        OUTPUT_MODE_LED = 3

    class pageId_e(enum.IntEnum):
        PAGE_WELCOME = 0
        PAGE_ARMED = 1
        PAGE_STATUS = 2

    class pidAutotuneState_e(enum.IntEnum):
        DEMAND_TOO_LOW = 0
        DEMAND_UNDERSHOOT = 1
        DEMAND_OVERSHOOT = 2
        TUNE_UPDATED = 3

    class pidIndex_e(enum.IntEnum):
        PID_ROLL = 0
        PID_PITCH = 1
        PID_YAW = 2
        PID_POS_Z = 3
        PID_POS_XY = 4
        PID_VEL_XY = 5
        PID_SURFACE = 6
        PID_LEVEL = 7
        PID_HEADING = 8
        PID_VEL_Z = 9
        PID_POS_HEADING = 10
        PID_ITEM_COUNT = 11

    class pidType_e(enum.IntEnum):
        PID_TYPE_NONE = 0
        PID_TYPE_PID = 1
        PID_TYPE_PIFF = 2
        PID_TYPE_AUTO = 3

    class pitotSensor_e(enum.IntEnum):
        PITOT_NONE = 0
        PITOT_AUTODETECT = 1
        PITOT_MS4525 = 2
        PITOT_ADC = 3
        PITOT_VIRTUAL = 4
        PITOT_FAKE = 5
        PITOT_MSP = 6
        PITOT_DLVR = 7

    class pollType_e(enum.IntEnum):
        PT_ACTIVE_ID = 0
        PT_INACTIVE_ID = 1

    class portSharing_e(enum.IntEnum):
        PORTSHARING_UNUSED = 0
        PORTSHARING_NOT_SHARED = 1
        PORTSHARING_SHARED = 2

    class quadrant_e(enum.IntEnum):
        QUADRANT_NORTH = 1 << 0
        QUADRANT_SOUTH = 1 << 1
        QUADRANT_EAST = 1 << 2
        QUADRANT_WEST = 1 << 3
        QUADRANT_NORTH_EAST = 1 << 4
        QUADRANT_SOUTH_EAST = 1 << 5
        QUADRANT_NORTH_WEST = 1 << 6
        QUADRANT_SOUTH_WEST = 1 << 7
        QUADRANT_NONE = 1 << 8
        QUADRANT_NOTDIAG = 1 << 9
        QUADRANT_ANY = QUADRANT_NORTH | QUADRANT_SOUTH | QUADRANT_EAST | QUADRANT_WEST | QUADRANT_NONE

    class rangefinderType_e(enum.IntEnum):
        RANGEFINDER_NONE = 0
        RANGEFINDER_SRF10 = 1
        RANGEFINDER_VL53L0X = 2
        RANGEFINDER_MSP = 3
        RANGEFINDER_BENEWAKE = 4
        RANGEFINDER_VL53L1X = 5
        RANGEFINDER_US42 = 6
        RANGEFINDER_TOF10102I2C = 7
        RANGEFINDER_FAKE = 8
        RANGEFINDER_TERARANGER_EVO = 9
        RANGEFINDER_USD1_V0 = 10
        RANGEFINDER_NANORADAR = 11

    class rcdeviceCamSimulationKeyEvent_e(enum.IntEnum):
        RCDEVICE_CAM_KEY_NONE = 0
        RCDEVICE_CAM_KEY_ENTER = 1
        RCDEVICE_CAM_KEY_LEFT = 2
        RCDEVICE_CAM_KEY_UP = 3
        RCDEVICE_CAM_KEY_RIGHT = 4
        RCDEVICE_CAM_KEY_DOWN = 5
        RCDEVICE_CAM_KEY_CONNECTION_CLOSE = 6
        RCDEVICE_CAM_KEY_CONNECTION_OPEN = 7
        RCDEVICE_CAM_KEY_RELEASE = 8

    class rcdeviceResponseStatus_e(enum.IntEnum):
        RCDEVICE_RESP_SUCCESS = 0
        RCDEVICE_RESP_INCORRECT_CRC = 1
        RCDEVICE_RESP_TIMEOUT = 2

    class rcdevice_5key_simulation_operation_e(enum.IntEnum):
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE = 0x00
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET = 0x01
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT = 0x02
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT = 0x03
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP = 0x04
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN = 0x05

    class rcdevice_camera_control_opeation_e(enum.IntEnum):
        RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_WIFI_BTN = 0x00
        RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_POWER_BTN = 0x01
        RCDEVICE_PROTOCOL_CAM_CTRL_CHANGE_MODE = 0x02
        RCDEVICE_PROTOCOL_CAM_CTRL_START_RECORDING = 0x03
        RCDEVICE_PROTOCOL_CAM_CTRL_STOP_RECORDING = 0x04
        RCDEVICE_PROTOCOL_CAM_CTRL_UNKNOWN_CAMERA_OPERATION = 0xFF

    class rcdevice_features_e(enum.IntEnum):
        RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON = (1 << 0)
        RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON = (1 << 1)
        RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE = (1 << 2)
        RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE = (1 << 3)
        RCDEVICE_PROTOCOL_FEATURE_START_RECORDING = (1 << 6)
        RCDEVICE_PROTOCOL_FEATURE_STOP_RECORDING = (1 << 7)
        RCDEVICE_PROTOCOL_FEATURE_CMS_MENU = (1 << 8)

    class rcdevice_protocol_version_e(enum.IntEnum):
        RCDEVICE_PROTOCOL_RCSPLIT_VERSION = 0x00
        RCDEVICE_PROTOCOL_VERSION_1_0 = 0x01
        RCDEVICE_PROTOCOL_UNKNOWN = 2

    class resolutionType_e(enum.IntEnum):
        SD_3016 = 0
        HD_5018 = 1
        HD_3016 = 2
        HD_6022 = 3
        HD_5320 = 4

    class reversibleMotorsThrottleState_e(enum.IntEnum):
        MOTOR_DIRECTION_FORWARD = 0
        MOTOR_DIRECTION_BACKWARD = 1
        MOTOR_DIRECTION_DEADBAND = 2

    class rollPitchStatus_e(enum.IntEnum):
        NOT_CENTERED = 0
        CENTERED = 1

    class rssiSource_e(enum.IntEnum):
        RSSI_SOURCE_NONE = 0
        RSSI_SOURCE_AUTO = 1
        RSSI_SOURCE_ADC = 2
        RSSI_SOURCE_RX_CHANNEL = 3
        RSSI_SOURCE_RX_PROTOCOL = 4
        RSSI_SOURCE_MSP = 5

    class rthState_e(enum.IntEnum):
        RTH_IDLE = 0
        RTH_IN_PROGRESS = 1
        RTH_HAS_LANDED = 2

    class rxFrameState_e(enum.IntEnum):
        RX_FRAME_PENDING = 0
        RX_FRAME_COMPLETE = (1 << 0)
        RX_FRAME_FAILSAFE = (1 << 1)
        RX_FRAME_PROCESSING_REQUIRED = (1 << 2)
        RX_FRAME_DROPPED = (1 << 3)

    class rxReceiverType_e(enum.IntEnum):
        RX_TYPE_NONE = 0
        RX_TYPE_SERIAL = 1
        RX_TYPE_MSP = 2
        RX_TYPE_SIM = 3

    class rxSerialReceiverType_e(enum.IntEnum):
        SERIALRX_SPEKTRUM1024 = 0
        SERIALRX_SPEKTRUM2048 = 1
        SERIALRX_SBUS = 2
        SERIALRX_SUMD = 3
        SERIALRX_IBUS = 4
        SERIALRX_JETIEXBUS = 5
        SERIALRX_CRSF = 6
        SERIALRX_FPORT = 7
        SERIALRX_SBUS_FAST = 8
        SERIALRX_FPORT2 = 9
        SERIALRX_SRXL2 = 10
        SERIALRX_GHST = 11
        SERIALRX_MAVLINK = 12
        SERIALRX_FBUS = 13
        SERIALRX_SBUS2 = 14

    class sbasMode_e(enum.IntEnum):
        SBAS_AUTO = 0
        SBAS_EGNOS = 1
        SBAS_WAAS = 2
        SBAS_MSAS = 3
        SBAS_GAGAN = 4
        SBAS_SPAN = 5
        SBAS_NONE = 6

    class sbusDecoderState_e(enum.IntEnum):
        STATE_SBUS_SYNC = 0
        STATE_SBUS_PAYLOAD = 1
        STATE_SBUS26_PAYLOAD = 2
        STATE_SBUS_WAIT_SYNC = 3

    class sensorIndex_e(enum.IntEnum):
        SENSOR_INDEX_GYRO = 0
        SENSOR_INDEX_ACC = 1
        SENSOR_INDEX_BARO = 2
        SENSOR_INDEX_MAG = 3
        SENSOR_INDEX_RANGEFINDER = 4
        SENSOR_INDEX_PITOT = 5
        SENSOR_INDEX_OPFLOW = 6
        SENSOR_INDEX_COUNT = 7

    class sensors_e(enum.IntEnum):
        SENSOR_GYRO = 1 << 0
        SENSOR_ACC = 1 << 1
        SENSOR_BARO = 1 << 2
        SENSOR_MAG = 1 << 3
        SENSOR_RANGEFINDER = 1 << 4
        SENSOR_PITOT = 1 << 5
        SENSOR_OPFLOW = 1 << 6
        SENSOR_GPS = 1 << 7
        SENSOR_GPSMAG = 1 << 8
        SENSOR_TEMP = 1 << 9

    class serialPortFunction_e(enum.IntEnum):
        FUNCTION_NONE = 0
        FUNCTION_MSP = (1 << 0)
        FUNCTION_GPS = (1 << 1)
        FUNCTION_UNUSED_3 = (1 << 2)
        FUNCTION_TELEMETRY_HOTT = (1 << 3)
        FUNCTION_TELEMETRY_LTM = (1 << 4)
        FUNCTION_TELEMETRY_SMARTPORT = (1 << 5)
        FUNCTION_RX_SERIAL = (1 << 6)
        FUNCTION_BLACKBOX = (1 << 7)
        FUNCTION_TELEMETRY_MAVLINK = (1 << 8)
        FUNCTION_TELEMETRY_IBUS = (1 << 9)
        FUNCTION_RCDEVICE = (1 << 10)
        FUNCTION_VTX_SMARTAUDIO = (1 << 11)
        FUNCTION_VTX_TRAMP = (1 << 12)
        FUNCTION_UNUSED_1 = (1 << 13)
        FUNCTION_OPTICAL_FLOW = (1 << 14)
        FUNCTION_LOG = (1 << 15)
        FUNCTION_RANGEFINDER = (1 << 16)
        FUNCTION_VTX_FFPV = (1 << 17)
        FUNCTION_ESCSERIAL = (1 << 18)
        FUNCTION_TELEMETRY_SIM = (1 << 19)
        FUNCTION_FRSKY_OSD = (1 << 20)
        FUNCTION_DJI_HD_OSD = (1 << 21)
        FUNCTION_SERVO_SERIAL = (1 << 22)
        FUNCTION_TELEMETRY_SMARTPORT_MASTER = (1 << 23)
        FUNCTION_UNUSED_2 = (1 << 24)
        FUNCTION_MSP_OSD = (1 << 25)
        FUNCTION_GIMBAL = (1 << 26)
        FUNCTION_GIMBAL_HEADTRACKER = (1 << 27)

    class serialPortIdentifier_e(enum.IntEnum):
        SERIAL_PORT_NONE = -1
        SERIAL_PORT_USART1 = 0
        SERIAL_PORT_USART2 = 1
        SERIAL_PORT_USART3 = 2
        SERIAL_PORT_USART4 = 3
        SERIAL_PORT_USART5 = 4
        SERIAL_PORT_USART6 = 5
        SERIAL_PORT_USART7 = 6
        SERIAL_PORT_USART8 = 7
        SERIAL_PORT_USB_VCP = 20
        SERIAL_PORT_SOFTSERIAL1 = 30
        SERIAL_PORT_SOFTSERIAL2 = 31
        SERIAL_PORT_IDENTIFIER_MAX = SERIAL_PORT_SOFTSERIAL2

    class servoAutotrimState_e(enum.IntEnum):
        AUTOTRIM_IDLE = 0
        AUTOTRIM_COLLECTING = 1
        AUTOTRIM_SAVE_PENDING = 2
        AUTOTRIM_DONE = 3

    class servoIndex_e(enum.IntEnum):
        SERVO_GIMBAL_PITCH = 0
        SERVO_GIMBAL_ROLL = 1
        SERVO_ELEVATOR = 2
        SERVO_FLAPPERON_1 = 3
        SERVO_FLAPPERON_2 = 4
        SERVO_RUDDER = 5
        SERVO_BICOPTER_LEFT = 4
        SERVO_BICOPTER_RIGHT = 5
        SERVO_DUALCOPTER_LEFT = 4
        SERVO_DUALCOPTER_RIGHT = 5
        SERVO_SINGLECOPTER_1 = 3
        SERVO_SINGLECOPTER_2 = 4
        SERVO_SINGLECOPTER_3 = 5
        SERVO_SINGLECOPTER_4 = 6

    class setting_mode_e(enum.IntEnum):
        MODE_DIRECT = (0 << SETTING_MODE_OFFSET)
        MODE_LOOKUP = (1 << SETTING_MODE_OFFSET)

    class setting_section_e(enum.IntEnum):
        MASTER_VALUE = (0 << SETTING_SECTION_OFFSET)
        PROFILE_VALUE = (1 << SETTING_SECTION_OFFSET)
        CONTROL_RATE_VALUE = (2 << SETTING_SECTION_OFFSET)
        BATTERY_CONFIG_VALUE = (3 << SETTING_SECTION_OFFSET)
        MIXER_CONFIG_VALUE = (4 << SETTING_SECTION_OFFSET)
        EZ_TUNE_VALUE = (5 << SETTING_SECTION_OFFSET)

    class setting_type_e(enum.IntEnum):
        VAR_UINT8 = (0 << SETTING_TYPE_OFFSET)
        VAR_INT8 = (1 << SETTING_TYPE_OFFSET)
        VAR_UINT16 = (2 << SETTING_TYPE_OFFSET)
        VAR_INT16 = (3 << SETTING_TYPE_OFFSET)
        VAR_UINT32 = (4 << SETTING_TYPE_OFFSET)
        VAR_FLOAT = (5 << SETTING_TYPE_OFFSET)
        VAR_STRING = (6 << SETTING_TYPE_OFFSET)

    class simATCommandState_e(enum.IntEnum):
        SIM_AT_OK = 0
        SIM_AT_ERROR = 1
        SIM_AT_WAITING_FOR_RESPONSE = 2

    class simModuleState_e(enum.IntEnum):
        SIM_MODULE_NOT_DETECTED = 0
        SIM_MODULE_NOT_REGISTERED = 1
        SIM_MODULE_REGISTERED = 2

    class simReadState_e(enum.IntEnum):
        SIM_READSTATE_RESPONSE = 0
        SIM_READSTATE_SMS = 1
        SIM_READSTATE_SKIP = 2

    class simTelemetryState_e(enum.IntEnum):
        SIM_STATE_INIT = 0
        SIM_STATE_INIT2 = 1
        SIM_STATE_INIT_ENTER_PIN = 2
        SIM_STATE_SET_MODES = 3
        SIM_STATE_SEND_SMS = 4
        SIM_STATE_SEND_SMS_ENTER_MESSAGE = 5

    class simTransmissionState_e(enum.IntEnum):
        SIM_TX_NO = 0
        SIM_TX_FS = 1
        SIM_TX = 2

    class simTxFlags_e(enum.IntEnum):
        SIM_TX_FLAG = (1 << 0)
        SIM_TX_FLAG_FAILSAFE = (1 << 1)
        SIM_TX_FLAG_GPS = (1 << 2)
        SIM_TX_FLAG_ACC = (1 << 3)
        SIM_TX_FLAG_LOW_ALT = (1 << 4)
        SIM_TX_FLAG_RESPONSE = (1 << 5)

    class simulatorFlags_t(enum.IntEnum):
        HITL_RESET_FLAGS = (0 << 0)
        HITL_ENABLE = (1 << 0)
        HITL_SIMULATE_BATTERY = (1 << 1)
        HITL_MUTE_BEEPER = (1 << 2)
        HITL_USE_IMU = (1 << 3)
        HITL_HAS_NEW_GPS_DATA = (1 << 4)
        HITL_EXT_BATTERY_VOLTAGE = (1 << 5)
        HITL_AIRSPEED = (1 << 6)
        HITL_EXTENDED_FLAGS = (1 << 7)
        HITL_GPS_TIMEOUT = (1 << 8)
        HITL_PITOT_FAILURE = (1 << 9)

    class smartAudioVersion_e(enum.IntEnum):
        SA_UNKNOWN = 0
        SA_1_0 = 1
        SA_2_0 = 2
        SA_2_1 = 3

    class smartportFuelUnit_e(enum.IntEnum):
        SMARTPORT_FUEL_UNIT_PERCENT = 0
        SMARTPORT_FUEL_UNIT_MAH = 1
        SMARTPORT_FUEL_UNIT_MWH = 2

    class stateFlags_t(enum.IntEnum):
        GPS_FIX_HOME = (1 << 0)
        GPS_FIX = (1 << 1)
        CALIBRATE_MAG = (1 << 2)
        SMALL_ANGLE = (1 << 3)
        FIXED_WING_LEGACY = (1 << 4)
        ANTI_WINDUP = (1 << 5)
        FLAPERON_AVAILABLE = (1 << 6)
        NAV_MOTOR_STOP_OR_IDLE = (1 << 7)
        COMPASS_CALIBRATED = (1 << 8)
        ACCELEROMETER_CALIBRATED = (1 << 9)
        GPS_ESTIMATED_FIX = (1 << 10)
        NAV_CRUISE_BRAKING = (1 << 11)
        NAV_CRUISE_BRAKING_BOOST = (1 << 12)
        NAV_CRUISE_BRAKING_LOCKED = (1 << 13)
        NAV_EXTRA_ARMING_SAFETY_BYPASSED = (1 << 14)
        AIRMODE_ACTIVE = (1 << 15)
        ESC_SENSOR_ENABLED = (1 << 16)
        AIRPLANE = (1 << 17)
        MULTIROTOR = (1 << 18)
        ROVER = (1 << 19)
        BOAT = (1 << 20)
        ALTITUDE_CONTROL = (1 << 21)
        MOVE_FORWARD_ONLY = (1 << 22)
        SET_REVERSIBLE_MOTORS_FORWARD = (1 << 23)
        FW_HEADING_USE_YAW = (1 << 24)
        ANTI_WINDUP_DEACTIVATED = (1 << 25)
        LANDING_DETECTED = (1 << 26)
        IN_FLIGHT_EMERG_REARM = (1 << 27)
        TAILSITTER = (1 << 28)

    class stickPositions_e(enum.IntEnum):
        pass # No valid members generated or all skipped

    class systemState_e(enum.IntEnum):
        SYSTEM_STATE_INITIALISING = 0
        SYSTEM_STATE_CONFIG_LOADED = (1 << 0)
        SYSTEM_STATE_SENSORS_READY = (1 << 1)
        SYSTEM_STATE_MOTORS_READY = (1 << 2)
        SYSTEM_STATE_TRANSPONDER_ENABLED = (1 << 3)
        SYSTEM_STATE_READY = (1 << 7)

    class tempSensorType_e(enum.IntEnum):
        TEMP_SENSOR_NONE = 0
        TEMP_SENSOR_LM75 = 1
        TEMP_SENSOR_DS18B20 = 2

    class throttleStatusType_e(enum.IntEnum):
        THROTTLE_STATUS_TYPE_RC = 0
        THROTTLE_STATUS_TYPE_COMMAND = 1

    class throttleStatus_e(enum.IntEnum):
        THROTTLE_LOW = 0
        THROTTLE_HIGH = 1

    class timId_e(enum.IntEnum):
        timBlink = 0
        timLarson = 1
        timBattery = 2
        timRssi = 3
        timGps = 4
        timWarning = 5
        timIndicator = 6
        timAnimation = 7
        timRing = 8
        timTimerCount = 9

    class ublox_nav_sig_health_e(enum.IntEnum):
        UBLOX_SIG_HEALTH_UNKNOWN = 0
        UBLOX_SIG_HEALTH_HEALTHY = 1
        UBLOX_SIG_HEALTH_UNHEALTHY = 2

    class ublox_nav_sig_quality(enum.IntEnum):
        UBLOX_SIG_QUALITY_NOSIGNAL = 0
        UBLOX_SIG_QUALITY_SEARCHING = 1
        UBLOX_SIG_QUALITY_ACQUIRED = 2
        UBLOX_SIG_QUALITY_UNUSABLE = 3
        UBLOX_SIG_QUALITY_CODE_LOCK_TIME_SYNC = 4
        UBLOX_SIG_QUALITY_CODE_CARRIER_LOCK_TIME_SYNC = 5
        UBLOX_SIG_QUALITY_CODE_CARRIER_LOCK_TIME_SYNC2 = 6
        UBLOX_SIG_QUALITY_CODE_CARRIER_LOCK_TIME_SYNC3 = 7

    class ubs_nav_fix_type_t(enum.IntEnum):
        FIX_NONE = 0
        FIX_DEAD_RECKONING = 1
        FIX_2D = 2
        FIX_3D = 3
        FIX_GPS_DEAD_RECKONING = 4
        FIX_TIME = 5

    class ubx_ack_state_t(enum.IntEnum):
        UBX_ACK_WAITING = 0
        UBX_ACK_GOT_ACK = 1
        UBX_ACK_GOT_NAK = 2

    class ubx_nav_status_bits_t(enum.IntEnum):
        NAV_STATUS_FIX_VALID = 1

    class ubx_protocol_bytes_t(enum.IntEnum):
        PREAMBLE1 = 0xB5
        PREAMBLE2 = 0x62
        CLASS_NAV = 0x01
        CLASS_ACK = 0x05
        CLASS_CFG = 0x06
        CLASS_MON = 0x0A
        MSG_CLASS_UBX = 0x01
        MSG_CLASS_NMEA = 0xF0
        MSG_VER = 0x04
        MSG_ACK_NACK = 0x00
        MSG_ACK_ACK = 0x01
        MSG_NMEA_GGA = 0x0
        MSG_NMEA_GLL = 0x1
        MSG_NMEA_GSA = 0x2
        MSG_NMEA_GSV = 0x3
        MSG_NMEA_RMC = 0x4
        MSG_NMEA_VGS = 0x5
        MSG_POSLLH = 0x2
        MSG_STATUS = 0x3
        MSG_SOL = 0x6
        MSG_PVT = 0x7
        MSG_VELNED = 0x12
        MSG_TIMEUTC = 0x21
        MSG_SVINFO = 0x30
        MSG_NAV_SAT = 0x35
        MSG_CFG_PRT = 0x00
        MSG_CFG_RATE = 0x08
        MSG_CFG_SET_RATE = 0x01
        MSG_CFG_NAV_SETTINGS = 0x24
        MSG_CFG_SBAS = 0x16
        MSG_CFG_GNSS = 0x3e
        MSG_MON_GNSS = 0x28
        MSG_NAV_SIG = 0x43

    class voltageSensor_e(enum.IntEnum):
        VOLTAGE_SENSOR_NONE = 0
        VOLTAGE_SENSOR_ADC = 1
        VOLTAGE_SENSOR_ESC = 2
        VOLTAGE_SENSOR_FAKE = 3

    class vs600Band_e(enum.IntEnum):
        VS600_BAND_A = 0
        VS600_BAND_B = 1
        VS600_BAND_C = 2
        VS600_BAND_D = 3
        VS600_BAND_E = 4
        VS600_BAND_F = 5

    class vs600Power_e(enum.IntEnum):
        VS600_POWER_PIT = 0
        VS600_POWER_25MW = 1
        VS600_POWER_200MW = 2
        VS600_POWER_600MW = 3

    class vtxLowerPowerDisarm_e(enum.IntEnum):
        VTX_LOW_POWER_DISARM_OFF = 0
        VTX_LOW_POWER_DISARM_ALWAYS = 1
        VTX_LOW_POWER_DISARM_UNTIL_FIRST_ARM = 2

    class vtxProtoResponseType_e(enum.IntEnum):
        VTX_RESPONSE_TYPE_NONE = 0
        VTX_RESPONSE_TYPE_CAPABILITIES = 1
        VTX_RESPONSE_TYPE_STATUS = 2

    class vtxProtoState_e(enum.IntEnum):
        VTX_STATE_RESET = 0
        VTX_STATE_OFFILE = 1
        VTX_STATE_DETECTING = 2
        VTX_STATE_IDLE = 3
        VTX_STATE_QUERY_DELAY = 4
        VTX_STATE_QUERY_STATUS = 5
        VTX_STATE_WAIT_STATUS = 6

    class vtxScheduleParams_e(enum.IntEnum):
        VTX_PARAM_POWER = 0
        VTX_PARAM_BANDCHAN = 1
        VTX_PARAM_PITMODE = 2
        VTX_PARAM_COUNT = 3

    class warningFlags_e(enum.IntEnum):
        WARNING_ARMING_DISABLED = 0
        WARNING_LOW_BATTERY = 1
        WARNING_FAILSAFE = 2
        WARNING_HW_ERROR = 3

    class warningLedState_e(enum.IntEnum):
        WARNING_LED_OFF = 0
        WARNING_LED_ON = 1
        WARNING_LED_FLASH = 2
