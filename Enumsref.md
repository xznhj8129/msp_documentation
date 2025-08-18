
# Enum Reference

**Warning: Work in progress, verification needed**

This document lists various enumerations used within the INAV firmware as of 8.0.1. These are useful for interpreting status flags, configuration values, and internal states.

---

## Arming Disable Flags (Betaflight Compatibility) (`armingDisableFlagNames_BF`)

Flags indicating why arming might be disabled (primarily for Betaflight compatibility references).

| Name        | Value |
|-------------|-------|
| NOGYRO      | 0     |
| FAILSAFE    | 1     |
| RXLOSS      | 2     |
| BADRX       | 3     |
| BOXFAILSAFE | 4     |
| RUNAWAY     | 5     |
| CRASH       | 6     |
| THROTTLE    | 7     |
| ANGLE       | 8     |
| BOOTGRACE   | 9     |
| NOPREARM    | 10    |
| LOAD        | 11    |
| CALIB       | 12    |
| CLI         | 13    |
| CMS         | 14    |
| BST         | 15    |
| MSP         | 16    |
| PARALYZE    | 17    |
| GPS         | 18    |
| RESCUE SW   | 19    |
| RPMFILTER   | 20    |
| ARMSWITCH   | 21    |

---

## Arming Flags / Disable Reasons (INAV) (`armingDisableFlagNames_INAV`)

Flags representing the arming state and reasons why arming might be blocked in INAV. Note: Values are bit positions.

| Name                                   | Bit Position |
|----------------------------------------|--------------|
| OK_TO_ARM                              | 0            |
| PREVENT_ARMING                         | 1            |
| ARMED                                  | 2            |
| WAS_EVER_ARMED                         | 3            |
| SIMULATOR_MODE                         | 4            |
| BLOCKED_FAILSAFE_SYSTEM                | 7            |
| BLOCKED_UAV_NOT_LEVEL                  | 8            |
| BLOCKED_SENSORS_CALIBRATING            | 9            |
| BLOCKED_SYSTEM_OVERLOADED              | 10           |
| BLOCKED_NAVIGATION_SAFETY              | 11           |
| BLOCKED_COMPASS_NOT_CALIBRATED         | 12           |
| BLOCKED_ACCELEROMETER_NOT_CALIBRATED   | 13           |
| BLOCKED_ARMING_DISABLED_ARM_SWITCH     | 14           |
| BLOCKED_HARDWARE_FAILURE               | 15           |
| BLOCKED_ARMING_DISABLED_BOXFAILSAFE    | 16           |
| BLOCKED_ARMING_DISABLED_BOXKILLSWITCH  | 17           |
| BLOCKED_ARMING_DISABLED_RC_LINK        | 18           |
| BLOCKED_ARMING_DISABLED_THROTTLE       | 19           |
| BLOCKED_ARMING_DISABLED_CLI            | 20           |
| BLOCKED_ARMING_DISABLED_CMS_MENU       | 21           |
| BLOCKED_ARMING_DISABLED_OSD_MENU       | 22           |
| BLOCKED_ARMING_DISABLED_ROLLPITCH_NOT_CENTERED | 23           |
| BLOCKED_ARMING_DISABLED_SERVO_AUTOTRIM | 24           |
| BLOCKED_ARMING_DISABLED_OOM            | 25           |
| BLOCKED_INVALID_SETTING                | 26           |
| BLOCKED_ARMING_DISABLED_PWM_OUTPUT_ERROR | 27           |
| BLOCKED_ARMING_DISABLED_NO_PREARM      | 28           |
| BLOCKED_ARMING_DISABLED_DSHOT_BEEPER   | 29           |
| BLOCKED_ARMING_DISABLED_LANDING_DETECTED | 30           |

---

## Flight Modes (Boxes) (`modesID_INAV`)

IDs corresponding to flight modes assignable to AUX channels (Boxes).

| Name               | ID |
|--------------------|----|
| ARM                | 0  |
| ANGLE              | 1  |
| HORIZON            | 2  |
| NAV ALTHOLD        | 3  |
| HEADING HOLD       | 5  |
| HEADFREE           | 6  |
| HEADADJ            | 7  |
| CAMSTAB            | 8  |
| NAV RTH            | 10 |
| NAV POSHOLD        | 11 |
| MANUAL             | 12 |
| BEEPER             | 13 |
| LEDS OFF           | 15 |
| LIGHTS             | 16 |
| OSD OFF            | 19 |
| TELEMETRY          | 20 |
| AUTO TUNE          | 21 |
| BLACKBOX           | 26 |
| FAILSAFE           | 27 |
| NAV WP             | 28 |
| AIR MODE           | 29 |
| HOME RESET         | 30 |
| GCS NAV            | 31 |
| FPV ANGLE MIX      | 32 |
| SURFACE            | 33 |
| FLAPERON           | 34 |
| TURN ASSIST        | 35 |
| NAV LAUNCH         | 36 |
| SERVO AUTOTRIM     | 37 |
| KILLSWITCH         | 38 |
| CAMERA CONTROL 1   | 39 |
| CAMERA CONTROL 2   | 40 |
| CAMERA CONTROL 3   | 41 |
| OSD ALT 1          | 42 |
| OSD ALT 2          | 43 |
| OSD ALT 3          | 44 |
| NAV COURSE HOLD    | 45 |
| MC BRAKING         | 46 |
| USER1              | 47 |
| USER2              | 48 |
| LOITER CHANGE      | 49 |
| MSP RC OVERRIDE    | 50 |
| PREARM             | 51 |
| TURTLE             | 52 |
| NAV CRUISE         | 53 |
| AUTO LEVEL         | 54 |
| WP PLANNER         | 55 |
| MISSION CHANGE     | 59 |
| BEEPER MUTE        | 60 |
| MULTI FUNCTION     | 61 |
| MIXER PROFILE 2    | 62 |
| MIXER TRANSITION   | 63 |
| ANG HOLD           | 64 |

---

## Platform Type (`platformType`)

Identifies the type of vehicle platform.

| Name                | Value |
|---------------------|-------|
| PLATFORM_MULTIROTOR | 0     |
| PLATFORM_AIRPLANE   | 1     |
| PLATFORM_HELICOPTER | 2     |
| PLATFORM_TRICOPTER  | 3     |
| PLATFORM_ROVER      | 4     |
| PLATFORM_BOAT       | 5     |
| PLATFORM_OTHER      | 6     |

---

## Navigation Waypoint Flags (`navSetWaypointFlags`)

Flags used in waypoint definitions (`p3` field).

| Name                          | Value      |
|-------------------------------|------------|
| NAV_POS_UPDATE_NONE           | 0          |
| NAV_POS_UPDATE_Z              | `1 << 1`   |
| NAV_POS_UPDATE_XY             | `1 << 0`   |
| NAV_POS_UPDATE_HEADING        | `1 << 2`   |
| NAV_POS_UPDATE_BEARING        | `1 << 3`   |
| NAV_POS_UPDATE_BEARING_TAIL_FIRST | `1 << 4`   |

---

## Climb Rate to Altitude Controller Mode (`climbRateToAltitudeControllerMode`)

Defines how the altitude controller manages climb rate.

| Name                 | Value |
|----------------------|-------|
| ROC_TO_ALT_CURRENT   | 0     |
| ROC_TO_ALT_CONSTANT  | 1     |
| ROC_TO_ALT_TARGET    | 2     |

---

## Navigation Estimate Status (`navigationEstimateStatus`)

Indicates the quality/trustworthiness of navigation estimates.

| Name         | Value |
|--------------|-------|
| EST_NONE     | 0     |
| EST_USABLE   | 1     |
| EST_TRUSTED  | 2     |

---

## Navigation Home Flags (`navigationHomeFlags`)

Flags indicating the validity of home position components.

| Name                   | Value                                                        |
|------------------------|--------------------------------------------------------------|
| NAV_HOME_INVALID       | 0                                                            |
| NAV_HOME_VALID_XY      | `1 << 0`                                                     |
| NAV_HOME_VALID_Z       | `1 << 1`                                                     |
| NAV_HOME_VALID_HEADING | `1 << 2`                                                     |
| NAV_HOME_VALID_ALL     | `NAV_HOME_VALID_XY | NAV_HOME_VALID_Z | NAV_HOME_VALID_HEADING` |

---

## Navigation FSM Events (`navigationFSMEvent`)

Events triggering transitions in the Navigation Finite State Machine.

| Name                                       | Value |
|--------------------------------------------|-------|
| NAV_FSM_EVENT_NONE                         | 0     |
| NAV_FSM_EVENT_TIMEOUT                      | 1     |
| NAV_FSM_EVENT_SUCCESS                      | 2     |
| NAV_FSM_EVENT_ERROR                        | 3     |
| NAV_FSM_EVENT_SWITCH_TO_IDLE               | 4     |
| NAV_FSM_EVENT_SWITCH_TO_ALTHOLD            | 5     |
| NAV_FSM_EVENT_SWITCH_TO_POSHOLD_3D         | 6     |
| NAV_FSM_EVENT_SWITCH_TO_RTH                | 7     |
| NAV_FSM_EVENT_SWITCH_TO_WAYPOINT           | 8     |
| NAV_FSM_EVENT_SWITCH_TO_EMERGENCY_LANDING  | 9     |
| NAV_FSM_EVENT_SWITCH_TO_LAUNCH             | 10    |
| NAV_FSM_EVENT_SWITCH_TO_COURSE_HOLD        | 11    |
| NAV_FSM_EVENT_SWITCH_TO_CRUISE             | 12    |
| NAV_FSM_EVENT_SWITCH_TO_COURSE_ADJ         | 13    |
| NAV_FSM_EVENT_SWITCH_TO_MIXERAT            | 14    |
| NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_FW_LANDING | 15    |
| NAV_FSM_EVENT_SWITCH_TO_SEND_TO            | 16    |
| NAV_FSM_EVENT_STATE_SPECIFIC_1             | 17    |
| NAV_FSM_EVENT_STATE_SPECIFIC_2             | 18    |
| NAV_FSM_EVENT_STATE_SPECIFIC_3             | 19    |
| NAV_FSM_EVENT_STATE_SPECIFIC_4             | 20    |
| NAV_FSM_EVENT_STATE_SPECIFIC_5             | 21    |
| NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_FW_LANDING_ABORT | 17    |
| NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_FW_LANDING_FINISHED | 18    |
| NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_HOLD_TIME | 17    |
| NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_RTH_LAND  | 18    |
| NAV_FSM_EVENT_SWITCH_TO_WAYPOINT_FINISHED  | 19    |
| NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_RTH_INITIALIZE | 17    |
| NAV_FSM_EVENT_SWITCH_TO_NAV_STATE_RTH_TRACKBACK | 18    |
| NAV_FSM_EVENT_SWITCH_TO_RTH_HEAD_HOME      | 19    |
| NAV_FSM_EVENT_SWITCH_TO_RTH_LOITER_ABOVE_HOME | 20    |
| NAV_FSM_EVENT_SWITCH_TO_RTH_LANDING        | 21    |
| NAV_FSM_EVENT_COUNT                        | 32    |

---

## Navigation Persistent IDs (`navigationPersistentId`)

Persistent identifiers for Navigation FSM states (used for saving/resuming).

| Name                                      | Value |
|-------------------------------------------|-------|
| NAV_PERSISTENT_ID_UNDEFINED               | 0     |
| NAV_PERSISTENT_ID_IDLE                    | 1     |
| NAV_PERSISTENT_ID_ALTHOLD_INITIALIZE      | 2     |
| NAV_PERSISTENT_ID_ALTHOLD_IN_PROGRESS     | 3     |
| NAV_PERSISTENT_ID_UNUSED_1                | 4     |
| NAV_PERSISTENT_ID_UNUSED_2                | 5     |
| NAV_PERSISTENT_ID_POSHOLD_3D_INITIALIZE   | 6     |
| NAV_PERSISTENT_ID_POSHOLD_3D_IN_PROGRESS  | 7     |
| NAV_PERSISTENT_ID_RTH_INITIALIZE          | 8     |
| NAV_PERSISTENT_ID_RTH_CLIMB_TO_SAFE_ALT   | 9     |
| NAV_PERSISTENT_ID_RTH_HEAD_HOME           | 10    |
| NAV_PERSISTENT_ID_RTH_LOITER_PRIOR_TO_LANDING | 11    |
| NAV_PERSISTENT_ID_RTH_LANDING             | 12    |
| NAV_PERSISTENT_ID_RTH_FINISHING           | 13    |
| NAV_PERSISTENT_ID_RTH_FINISHED            | 14    |
| NAV_PERSISTENT_ID_WAYPOINT_INITIALIZE     | 15    |
| NAV_PERSISTENT_ID_WAYPOINT_PRE_ACTION     | 16    |
| NAV_PERSISTENT_ID_WAYPOINT_IN_PROGRESS    | 17    |
| NAV_PERSISTENT_ID_WAYPOINT_REACHED        | 18    |
| NAV_PERSISTENT_ID_WAYPOINT_NEXT           | 19    |
| NAV_PERSISTENT_ID_WAYPOINT_FINISHED       | 20    |
| NAV_PERSISTENT_ID_WAYPOINT_RTH_LAND       | 21    |
| NAV_PERSISTENT_ID_EMERGENCY_LANDING_INITIALIZE | 22    |
| NAV_PERSISTENT_ID_EMERGENCY_LANDING_IN_PROGRESS | 23    |
| NAV_PERSISTENT_ID_EMERGENCY_LANDING_FINISHED | 24    |
| NAV_PERSISTENT_ID_LAUNCH_INITIALIZE       | 25    |
| NAV_PERSISTENT_ID_LAUNCH_WAIT             | 26    |
| NAV_PERSISTENT_ID_UNUSED_3                | 27    |
| NAV_PERSISTENT_ID_LAUNCH_IN_PROGRESS      | 28    |
| NAV_PERSISTENT_ID_COURSE_HOLD_INITIALIZE  | 29    |
| NAV_PERSISTENT_ID_COURSE_HOLD_IN_PROGRESS | 30    |
| NAV_PERSISTENT_ID_COURSE_HOLD_ADJUSTING   | 31    |
| NAV_PERSISTENT_ID_CRUISE_INITIALIZE       | 32    |
| NAV_PERSISTENT_ID_CRUISE_IN_PROGRESS      | 33    |
| NAV_PERSISTENT_ID_CRUISE_ADJUSTING        | 34    |
| NAV_PERSISTENT_ID_WAYPOINT_HOLD_TIME      | 35    |
| NAV_PERSISTENT_ID_RTH_LOITER_ABOVE_HOME   | 36    |
| NAV_PERSISTENT_ID_UNUSED_4                | 37    |
| NAV_PERSISTENT_ID_RTH_TRACKBACK           | 38    |
| NAV_PERSISTENT_ID_MIXERAT_INITIALIZE      | 39    |
| NAV_PERSISTENT_ID_MIXERAT_IN_PROGRESS     | 40    |
| NAV_PERSISTENT_ID_MIXERAT_ABORT           | 41    |
| NAV_PERSISTENT_ID_FW_LANDING_CLIMB_TO_LOITER | 42    |
| NAV_PERSISTENT_ID_FW_LANDING_LOITER       | 43    |
| NAV_PERSISTENT_ID_FW_LANDING_APPROACH     | 44    |
| NAV_PERSISTENT_ID_FW_LANDING_GLIDE        | 45    |
| NAV_PERSISTENT_ID_FW_LANDING_FLARE        | 46    |
| NAV_PERSISTENT_ID_FW_LANDING_ABORT        | 47    |
| NAV_PERSISTENT_ID_FW_LANDING_FINISHED     | 48    |
| NAV_PERSISTENT_ID_SEND_TO_INITALIZE       | 49    |
| NAV_PERSISTENT_ID_SEND_TO_IN_PROGRES      | 50    |
| NAV_PERSISTENT_ID_SEND_TO_FINISHED        | 51    |

---

## Navigation FSM States (`navigationFSMState`)

States within the Navigation Finite State Machine.

| Name                                    | Value |
|-----------------------------------------|-------|
| NAV_STATE_UNDEFINED                     | 0     |
| NAV_STATE_IDLE                          | 1     |
| NAV_STATE_ALTHOLD_INITIALIZE            | 2     |
| NAV_STATE_ALTHOLD_IN_PROGRESS           | 3     |
| NAV_STATE_POSHOLD_3D_INITIALIZE         | 4     |
| NAV_STATE_POSHOLD_3D_IN_PROGRESS        | 5     |
| NAV_STATE_RTH_INITIALIZE                | 6     |
| NAV_STATE_RTH_CLIMB_TO_SAFE_ALT         | 7     |
| NAV_STATE_RTH_TRACKBACK                 | 8     |
| NAV_STATE_RTH_HEAD_HOME                 | 9     |
| NAV_STATE_RTH_LOITER_PRIOR_TO_LANDING   | 10    |
| NAV_STATE_RTH_LOITER_ABOVE_HOME         | 11    |
| NAV_STATE_RTH_LANDING                   | 12    |
| NAV_STATE_RTH_FINISHING                 | 13    |
| NAV_STATE_RTH_FINISHED                  | 14    |
| NAV_STATE_WAYPOINT_INITIALIZE           | 15    |
| NAV_STATE_WAYPOINT_PRE_ACTION           | 16    |
| NAV_STATE_WAYPOINT_IN_PROGRESS          | 17    |
| NAV_STATE_WAYPOINT_REACHED              | 18    |
| NAV_STATE_WAYPOINT_HOLD_TIME            | 19    |
| NAV_STATE_WAYPOINT_NEXT                 | 20    |
| NAV_STATE_WAYPOINT_FINISHED             | 21    |
| NAV_STATE_WAYPOINT_RTH_LAND             | 22    |
| NAV_STATE_EMERGENCY_LANDING_INITIALIZE  | 23    |
| NAV_STATE_EMERGENCY_LANDING_IN_PROGRESS | 24    |
| NAV_STATE_EMERGENCY_LANDING_FINISHED    | 25    |
| NAV_STATE_LAUNCH_INITIALIZE             | 26    |
| NAV_STATE_LAUNCH_WAIT                   | 27    |
| NAV_STATE_LAUNCH_IN_PROGRESS            | 28    |
| NAV_STATE_COURSE_HOLD_INITIALIZE        | 29    |
| NAV_STATE_COURSE_HOLD_IN_PROGRESS       | 30    |
| NAV_STATE_COURSE_HOLD_ADJUSTING         | 31    |
| NAV_STATE_CRUISE_INITIALIZE             | 32    |
| NAV_STATE_CRUISE_IN_PROGRESS            | 33    |
| NAV_STATE_CRUISE_ADJUSTING              | 34    |
| NAV_STATE_FW_LANDING_CLIMB_TO_LOITER    | 35    |
| NAV_STATE_FW_LANDING_LOITER             | 36    |
| NAV_STATE_FW_LANDING_APPROACH           | 37    |
| NAV_STATE_FW_LANDING_GLIDE              | 38    |
| NAV_STATE_FW_LANDING_FLARE              | 39    |
| NAV_STATE_FW_LANDING_FINISHED           | 40    |
| NAV_STATE_FW_LANDING_ABORT              | 41    |
| NAV_STATE_MIXERAT_INITIALIZE            | 42    |
| NAV_STATE_MIXERAT_IN_PROGRESS           | 43    |
| NAV_STATE_MIXERAT_ABORT                 | 44    |
| NAV_STATE_SEND_TO_INITALIZE             | 45    |
| NAV_STATE_SEND_TO_IN_PROGESS            | 46    |
| NAV_STATE_SEND_TO_FINISHED              | 47    |
| NAV_STATE_COUNT                         | 48    |

---

## Navigation FSM State Flags (`navigationFSMStateFlags`)

Flags describing the capabilities or requirements of Navigation FSM states.

| Name                 | Value     |
|----------------------|-----------|
| NAV_CTL_ALT          | `(1 << 0)`|
| NAV_CTL_POS          | `(1 << 1)`|
| NAV_CTL_YAW          | `(1 << 2)`|
| NAV_CTL_EMERG        | `(1 << 3)`|
| NAV_CTL_LAUNCH       | `(1 << 4)`|
| NAV_REQUIRE_ANGLE    | `(1 << 5)`|
| NAV_REQUIRE_ANGLE_FW | `(1 << 6)`|
| NAV_REQUIRE_MAGHOLD  | `(1 << 7)`|
| NAV_REQUIRE_THRTILT  | `(1 << 8)`|
| NAV_AUTO_RTH         | `(1 << 9)`|
| NAV_AUTO_WP          | `(1 << 10)`|
| NAV_RC_ALT           | `(1 << 11)`|
| NAV_RC_POS           | `(1 << 12)`|
| NAV_RC_YAW           | `(1 << 13)`|
| NAV_CTL_LAND         | `(1 << 14)`|
| NAV_AUTO_WP_DONE     | `(1 << 15)`|
| NAV_MIXERAT          | `(1 << 16)`|
| NAV_CTL_HOLD         | `(1 << 17)`|

---

## Fixed Wing Autoland Waypoint Indices (`fwAutolandWaypoint`)

Indices for internally generated waypoints during fixed wing autoland.

| Name                       | Value |
|----------------------------|-------|
| FW_AUTOLAND_WP_TURN        | 0     |
| FW_AUTOLAND_WP_FINAL_APPROACH | 1     |
| FW_AUTOLAND_WP_LAND        | 2     |
| FW_AUTOLAND_WP_COUNT       | 3     |

---

## RTH Target Mode (`rthTargetMode`)

Internal state for RTH targeting logic.

| Name                          | Value |
|-------------------------------|-------|
| RTH_HOME_ENROUTE_INITIAL      | 0     |
| RTH_HOME_ENROUTE_PROPORTIONAL | 1     |
| RTH_HOME_ENROUTE_FINAL        | 2     |
| RTH_HOME_FINAL_LOITER         | 3     |
| RTH_HOME_FINAL_LAND           | 4     |

---

## AGL Estimate Quality (`navAGLEstimateQuality`)

Quality indicator for Altitude Ground Level estimates.

| Name                | Value |
|---------------------|-------|
| SURFACE_QUAL_LOW    | 0     |
| SURFACE_QUAL_MID    | 1     |
| SURFACE_QUAL_HIGH   | 2     |

---

## Position Estimation Flags (`navPositionEstimationFlags`)

Flags indicating the validity of different position/altitude sources.

| Name                  | Value     |
|-----------------------|-----------|
| EST_GPS_XY_VALID      | `(1 << 0)`|
| EST_GPS_Z_VALID       | `(1 << 1)`|
| EST_BARO_VALID        | `(1 << 2)`|
| EST_SURFACE_VALID     | `(1 << 3)`|
| EST_FLOW_VALID        | `(1 << 4)`|
| EST_XY_VALID          | `(1 << 5)`|
| EST_Z_VALID           | `(1 << 6)`|

---

## Default Altitude Sensor (`navDefaultAltitudeSensor`)

Defines the primary source used for altitude estimation.

| Name                     | Value |
|--------------------------|-------|
| ALTITUDE_SOURCE_GPS      | 0     |
| ALTITUDE_SOURCE_BARO     | 1     |
| ALTITUDE_SOURCE_GPS_ONLY | 2     |
| ALTITUDE_SOURCE_BARO_ONLY| 3     |

---

## Safe Home Usage Mode (`safehomeUsageMode`)

Configures when Safe Home locations are used.

| Name                | Value |
|---------------------|-------|
| SAFEHOME_USAGE_OFF  | 0     |
| SAFEHOME_USAGE_RTH  | 1     |
| SAFEHOME_USAGE_RTH_FS | 2     |

---

## Fixed Wing Autoland State (`fwAutolandState`)

Internal states for the fixed wing autoland procedure.

| Name                         | Value |
|------------------------------|-------|
| FW_AUTOLAND_STATE_IDLE       | 0     |
| FW_AUTOLAND_STATE_LOITER     | 1     |
| FW_AUTOLAND_STATE_DOWNWIND   | 2     |
| FW_AUTOLAND_STATE_BASE_LEG   | 3     |
| FW_AUTOLAND_STATE_FINAL_APPROACH | 4     |
| FW_AUTOLAND_STATE_GLIDE      | 5     |
| FW_AUTOLAND_STATE_FLARE      | 6     |

---

## Geozone Message State (`geozoneMessageState`)

Indicates the current status relative to defined geofences.

| Name                                        | Value |
|---------------------------------------------|-------|
| GEOZONE_MESSAGE_STATE_NONE                  | 0     |
| GEOZONE_MESSAGE_STATE_NFZ                   | 1     |
| GEOZONE_MESSAGE_STATE_LEAVING_FZ            | 2     |
| GEOZONE_MESSAGE_STATE_OUTSIDE_FZ            | 3     |
| GEOZONE_MESSAGE_STATE_ENTERING_NFZ          | 4     |
| GEOZONE_MESSAGE_STATE_AVOIDING_FB           | 5     |
| GEOZONE_MESSAGE_STATE_RETURN_TO_ZONE        | 6     |
| GEOZONE_MESSAGE_STATE_FLYOUT_NFZ            | 7     |
| GEOZONE_MESSAGE_STATE_AVOIDING_ALTITUDE_BREACH | 8     |
| GEOZONE_MESSAGE_STATE_POS_HOLD              | 9     |

---

## Navigation Home Reset Type (`nav_resetype`)

Configures when the home position is automatically reset.

| Name                     | Value |
|--------------------------|-------|
| NAV_RESET_NEVER          | 0     |
| NAV_RESET_ON_FIRST_ARM | 1     |
| NAV_RESET_ON_EACH_ARM    | 2     |

---

## RTH Allow Landing (`navRTHAllowLanding`)

Configures when RTH procedure is allowed to proceed to landing.

| Name                          | Value |
|-------------------------------|-------|
| NAV_RTH_ALLOW_LANDING_NEVER   | 0     |
| NAV_RTH_ALLOW_LANDING_ALWAYS  | 1     |
| NAV_RTH_ALLOW_LANDING_FS_ONLY | 2     |

---

## Extra Arming Safety (`navExtraArmingSafety`)

Configures additional arming checks related to navigation safety.

| Name                               | Value |
|------------------------------------|-------|
| NAV_EXTRA_ARMING_SAFETY_ON         | 0     |
| NAV_EXTRA_ARMING_SAFETY_ALLOW_BYPASS | 1     |

---

## Navigation Arming Blockers (`navArmingBlocker`)

Specific reasons related to navigation that can prevent arming.

| Name                                   | Value |
|----------------------------------------|-------|
| NAV_ARMING_BLOCKER_NONE                | 0     |
| NAV_ARMING_BLOCKER_MISSING_GPS_FIX     | 1     |
| NAV_ARMING_BLOCKER_NAV_IS_ALREADY_ACTIVE | 2     |
| NAV_ARMING_BLOCKER_FIRST_WAYPOINT_TOO_FAR | 3     |
| NAV_ARMING_BLOCKER_JUMP_WAYPOINT_ERROR | 4     |

---

## Navigation Overrides Motor Stop (`navOverridesMotorStop`)

Configures if navigation modes override the `motor_stop` setting.

| Name             | Value |
|------------------|-------|
| NOMS_OFF_ALWAYS  | 0     |
| NOMS_OFF         | 1     |
| NOMS_AUTO_ONLY   | 2     |
| NOMS_ALL_NAV     | 3     |

---

## RTH Climb First Mode (`navRTHClimbFirst`)

Configures how the initial climb during RTH behaves.

| Name                     | Value |
|--------------------------|-------|
| RTH_CLIMB_OFF            | 0     |
| RTH_CLIMB_ON             | 1     |
| RTH_CLIMB_ON_FW_SPIRAL | 2     |

---

## Fixed Wing Launch Status (`navFwLaunchStatus`)

Status codes for the fixed wing launch procedure.

| Name                   | Value |
|------------------------|-------|
| FW_LAUNCH_DETECTED     | 5     |
| FW_LAUNCH_ABORTED      | 10    |
| FW_LAUNCH_FLYING       | 11    |

---

## Waypoint Mission Planner Status (`wpMissionPlannerStatus`)

Status codes for the onboard waypoint planner mode.

| Name           | Value |
|----------------|-------|
| WP_PLAN_WAIT   | 0     |
| WP_PLAN_SAVE   | 1     |
| WP_PLAN_OK     | 2     |
| WP_PLAN_FULL   | 3     |

---

## Navigation Mission Restart Mode (`navMissionRestart`)

Configures how waypoint missions are started or resumed.

| Name                | Value |
|---------------------|-------|
| WP_MISSION_START    | 0     |
| WP_MISSION_RESUME   | 1     |
| WP_MISSION_SWITCH   | 2     |

---

## RTH Trackback Mode (`rthTrackbackMode`)

Configures if RTH follows the outbound track.

| Name               | Value |
|--------------------|-------|
| RTH_TRACKBACK_OFF  | 0     |
| RTH_TRACKBACK_ON   | 1     |
| RTH_TRACKBACK_FS   | 2     |

---

## Fixed Wing Waypoint Turn Smoothing (`wpFwTurnSmoothing`)

Configures turn behavior between waypoints for fixed wing.

| Name                  | Value |
|-----------------------|-------|
| WP_TURN_SMOOTHING_OFF | 0     |
| WP_TURN_SMOOTHING_ON  | 1     |
| WP_TURN_SMOOTHING_CUT | 2     |

---

## Multicopter Altitude Hold Throttle (`navMcAltHoldThrottle`)

Configures throttle behavior during Altitude Hold for multicopters.

| Name                  | Value |
|-----------------------|-------|
| MC_ALT_HOLD_STICK     | 0     |
| MC_ALT_HOLD_MID       | 1     |
| MC_ALT_HOLD_HOVER     | 2     |

---

## Navigation Waypoint Actions (`navWaypointActions`)

Actions performed when reaching a waypoint.

| Name                   | Value |
|------------------------|-------|
| NAV_WP_ACTION_WAYPOINT | 1     |
| NAV_WP_ACTION_HOLD_TIME| 3     |
| NAV_WP_ACTION_RTH      | 4     |
| NAV_WP_ACTION_SET_POI  | 5     |
| NAV_WP_ACTION_JUMP     | 6     |
| NAV_WP_ACTION_SET_HEAD | 7     |
| NAV_WP_ACTION_LAND     | 8     |

---

## Navigation Waypoint Headings (`navWaypointHeadings`)

Configures heading behavior during waypoint navigation.

| Name                   | Value |
|------------------------|-------|
| NAV_WP_HEAD_MODE_NONE  | 0     |
| NAV_WP_HEAD_MODE_POI   | 1     |
| NAV_WP_HEAD_MODE_FIXED | 2     |

---

## Navigation Waypoint Flags (`navWaypointFlags`)

Special flags for waypoints.

| Name             | Value | Description                   |
|------------------|-------|-------------------------------|
| NAV_WP_FLAG_HOME | 72    | Represents the home location |
| NAV_WP_FLAG_LAST | 165   | Marks the last waypoint       |

---

## Navigation Waypoint P3 Flags (`navWaypointP3Flags`)

Flags packed into the `p3` parameter of a waypoint.

| Name           | Value     |
|----------------|-----------|
| NAV_WP_ALTMODE | `(1<<0)`  |
| NAV_WP_USER1   | `(1<<1)`  |
| NAV_WP_USER2   | `(1<<2)`  |
| NAV_WP_USER3   | `(1<<3)`  |
| NAV_WP_USER4   | `(1<<4)`  |

---

## Navigation System Status Mode (`navSystemStatus_Mode`)

Overall mode reported by `MSP_NAV_STATUS`.

| Name             | Value |
|------------------|-------|
| MW_GPS_MODE_NONE | 0     |
| MW_GPS_MODE_HOLD | 1     |
| MW_GPS_MODE_RTH  | 2     |
| MW_GPS_MODE_NAV  | 3     |
| MW_GPS_MODE_EMERG| 15    |

---

## Navigation System Status State (`navSystemStatus_State`)

Detailed state reported by `MSP_NAV_STATUS` (partially legacy/MultiWii names).

| Name                          | Value |
|-------------------------------|-------|
| MW_NAV_STATE_NONE             | 0     |
| MW_NAV_STATE_RTH_START        | 1     |
| MW_NAV_STATE_RTH_ENROUTE      | 2     |
| MW_NAV_STATE_HOLD_INFINIT     | 3     |
| MW_NAV_STATE_HOLD_TIMED       | 4     |
| MW_NAV_STATE_WP_ENROUTE       | 5     |
| MW_NAV_STATE_PROCESS_NEXT     | 6     |
| MW_NAV_STATE_DO_JUMP          | 7     |
| MW_NAV_STATE_LAND_START       | 8     |
| MW_NAV_STATE_LAND_IN_PROGRESS | 9     |
| MW_NAV_STATE_LANDED           | 10    |
| MW_NAV_STATE_LAND_SETTLE      | 11    |
| MW_NAV_STATE_LAND_START_DESCENT | 12    |
| MW_NAV_STATE_HOVER_ABOVE_HOME | 13    |
| MW_NAV_STATE_EMERGENCY_LANDING| 14    |
| MW_NAV_STATE_RTH_CLIMB        | 15    |

---

## Navigation System Status Error (`navSystemStatus_Error`)

Error codes reported by `MSP_NAV_STATUS`.

| Name                      | Value |
|---------------------------|-------|
| MW_NAV_ERROR_NONE         | 0     |
| MW_NAV_ERROR_TOOFAR       | 1     |
| MW_NAV_ERROR_SPOILED_GPS  | 2     |
| MW_NAV_ERROR_WP_CRC       | 3     |
| MW_NAV_ERROR_FINISH       | 4     |
| MW_NAV_ERROR_TIMEWAIT     | 5     |
| MW_NAV_ERROR_INVALID_JUMP | 6     |
| MW_NAV_ERROR_INVALID_DATA | 7     |
| MW_NAV_ERROR_WAIT_FOR_RTH_ALT | 8     |
| MW_NAV_ERROR_GPS_FIX_LOST | 9     |
| MW_NAV_ERROR_DISARMED     | 10    |
| MW_NAV_ERROR_LANDING      | 11    |

---

## Navigation System Status Flags (`navSystemStatus_Flags`)

Additional status flags (not reported via MSP_NAV_STATUS).

| Name                           | Value     |
|--------------------------------|-----------|
| MW_NAV_FLAG_ADJUSTING_POSITION | `1 << 0`  |
| MW_NAV_FLAG_ADJUSTING_ALTITUDE | `1 << 1`  |

---

## Geoid Altitude Conversion Mode (`geoAltitudeConversionMode`)

Defines how altitude is converted relative to geoid/MSL.

| Name                   | Value |
|------------------------|-------|
| GEO_ALT_ABSOLUTE       | 0     |
| GEO_ALT_RELATIVE       | 1     |

---

## Geoid Origin Reset Mode (`geoOriginResetMode`)

Defines behavior when resetting the geoid origin.

| Name                        | Value |
|-----------------------------|-------|
| GEO_ORIGIN_SET              | 0     |
| GEO_ORIGIN_RESET_ALTITUDE | 1     |

---

## Geoid Altitude Datum Flag (`geoAltitudeDatumFlag`)

Indicates the altitude reference datum for waypoints.

| Name                   | Value |
|------------------------|-------|
| NAV_WP_TAKEOFF_DATUM   | 0     |
| NAV_WP_MSL_DATUM       | 1     |

---

## Current Sensor Type (`currentSensor`)

Defines the type of current sensor configured.

| Name                   | Value |
|------------------------|-------|
| CURRENT_SENSOR_NONE    | 0     |
| CURRENT_SENSOR_ADC     | 1     |
| CURRENT_SENSOR_VIRTUAL | 2     |
| CURRENT_SENSOR_FAKE    | 3     |
| CURRENT_SENSOR_ESC     | 4     |
| CURRENT_SENSOR_MAX     | 3     | *Note: MAX value seems inconsistent*

---

## Voltage Sensor Type (`voltageSensor`)

Defines the type of voltage sensor configured.

| Name                   | Value |
|------------------------|-------|
| VOLTAGE_SENSOR_NONE    | 0     |
| VOLTAGE_SENSOR_ADC     | 1     |
| VOLTAGE_SENSOR_ESC     | 2     |
| VOLTAGE_SENSOR_FAKE    | 3     |

---

## Battery Capacity Unit (`batCapacityUnit`)

Unit used for battery capacity measurement and reporting.

| Name                      | Value |
|---------------------------|-------|
| BAT_CAPACITY_UNIT_MAH     | 0     |
| BAT_CAPACITY_UNIT_MWH     | 1     |

---

## Battery Voltage Source (`batVoltageSource`)

Source used for battery voltage measurement.

| Name                   | Value |
|------------------------|-------|
| BAT_VOLTAGE_RAW        | 0     |
| BAT_VOLTAGE_SAG_COMP | 1     |

---

## Rangefinder Type (`rangefinderType`)

Type of rangefinder sensor configured.

| Name                       | Value |
|----------------------------|-------|
| RANGEFINDER_NONE           | 0     |
| RANGEFINDER_SRF10          | 1     |
| RANGEFINDER_VL53L0X        | 2     |
| RANGEFINDER_MSP            | 3     |
| RANGEFINDER_BENEWAKE       | 4     |
| RANGEFINDER_VL53L1X        | 5     |
| RANGEFINDER_US42           | 6     |
| RANGEFINDER_TOF10102I2C    | 7     |
| RANGEFINDER_FAKE           | 8     |
| RANGEFINDER_TERARANGER_EVO | 9     |
| RANGEFINDER_USD1_V0        | 10    |
| RANGEFINDER_NANORADAR      | 11    |

---

## Gyro Sensor Type (`gyroSensor`)

Type of gyroscope sensor configured.

| Name            | Value |
|-----------------|-------|
| GYRO_NONE       | 0     |
| GYRO_AUTODETECT | 1     |
| GYRO_MPU6000    | 2     |
| GYRO_MPU6500    | 3     |
| GYRO_MPU9250    | 4     |
| GYRO_BMI160     | 5     |
| GYRO_ICM20689   | 6     |
| GYRO_BMI088     | 7     |
| GYRO_ICM42605   | 8     |
| GYRO_BMI270     | 9     |
| GYRO_LSM6DXX    | 10    |
| GYRO_FAKE       | 11    |

---

## Dynamic Gyro Notch Mode (`dynamicGyroNotchMode`)

Mode for the dynamic gyro notch filter.

| Name                    | Value |
|-------------------------|-------|
| DYNAMIC_NOTCH_MODE_2D | 0     |
| DYNAMIC_NOTCH_MODE_3D | 1     |

---

## Gyro Filter Mode (`gyroFilterMode`)

Mode for the main gyro filter (LPF).

| Name                     | Value |
|--------------------------|-------|
| GYRO_FILTER_MODE_OFF     | 0     |
| GYRO_FILTER_MODE_STATIC  | 1     |
| GYRO_FILTER_MODE_DYNAMIC | 2     |
| GYRO_FILTER_MODE_ADAPTIVE| 3     |

---

## Optical Flow Sensor Type (`opticalFlowSensor`)

Type of optical flow sensor configured.

| Name          | Value |
|---------------|-------|
| OPFLOW_NONE   | 0     |
| OPFLOW_CXOF   | 1     |
| OPFLOW_MSP    | 2     |
| OPFLOW_FAKE   | 3     |

---

## Optical Flow Quality (`opflowQuality`)

Indicates the validity of the optical flow measurement.

| Name                   | Value |
|------------------------|-------|
| OPFLOW_QUALITY_INVALID | 0     |
| OPFLOW_QUALITY_VALID   | 1     |

---

## Battery State (`batteryState`)

Overall state of the battery based on voltage/capacity.

| Name                  | Value |
|-----------------------|-------|
| BATTERY_OK            | 0     |
| BATTERY_WARNING       | 1     |
| BATTERY_CRITICAL      | 2     |
| BATTERY_NOT_PRESENT   | 3     |

---

## Temperature Sensor Type (`tempSensorType`)

Type of external temperature sensor configured.

| Name                | Value |
|---------------------|-------|
| TEMP_SENSOR_NONE    | 0     |
| TEMP_SENSOR_LM75    | 1     |
| TEMP_SENSOR_DS18B20 | 2     |

---

## Pitot Sensor Type (`pitotSensor`)

Type of pitot (airspeed) sensor configured.

| Name             | Value |
|------------------|-------|
| PITOT_NONE       | 0     |
| PITOT_AUTODETECT | 1     |
| PITOT_MS4525     | 2     |
| PITOT_ADC        | 3     |
| PITOT_VIRTUAL    | 4     |
| PITOT_FAKE       | 5     |
| PITOT_MSP        | 6     |
| PITOT_DLVR       | 7     |

---

## Sensor Index (`sensorIndex`)

Internal indices used for accessing sensor data arrays.

| Name                   | Value |
|------------------------|-------|
| SENSOR_INDEX_GYRO      | 0     |
| SENSOR_INDEX_ACC       | 1     |
| SENSOR_INDEX_BARO      | 2     |
| SENSOR_INDEX_MAG       | 3     |
| SENSOR_INDEX_RANGEFINDER | 4     |
| SENSOR_INDEX_PITOT     | 5     |
| SENSOR_INDEX_OPFLOW    | 6     |
| SENSOR_INDEX_COUNT     | 7     |

---

## Sensors Bitmask (`sensors`)

Bitmask used to represent enabled/detected sensors.

| Name               | Value     |
|--------------------|-----------|
| SENSOR_GYRO        | `1 << 0`  |
| SENSOR_ACC         | `1 << 1`  |
| SENSOR_BARO        | `1 << 2`  |
| SENSOR_MAG         | `1 << 3`  |
| SENSOR_RANGEFINDER | `1 << 4`  |
| SENSOR_PITOT       | `1 << 5`  |
| SENSOR_OPFLOW      | `1 << 6`  |
| SENSOR_GPS         | `1 << 7`  |
| SENSOR_GPSMAG      | `1 << 8`  |
| SENSOR_TEMP        | `1 << 9`  |

---

## Accelerometer Sensor Type (`accelerationSensor`)

Type of accelerometer sensor configured.

| Name             | Value |
|------------------|-------|
| ACC_NONE         | 0     |
| ACC_AUTODETECT   | 1     |
| ACC_MPU6000      | 2     |
| ACC_MPU6500      | 3     |
| ACC_MPU9250      | 4     |
| ACC_BMI160       | 5     |
| ACC_ICM20689     | 6     |
| ACC_BMI088       | 7     |
| ACC_ICM42605     | 8     |
| ACC_BMI270       | 9     |
| ACC_LSM6DXX      | 10    |
| ACC_FAKE         | 11    |
| ACC_MAX          | 11    |

---

## Barometer Sensor Type (`baroSensor`)

Type of barometer sensor configured.

| Name            | Value |
|-----------------|-------|
| BARO_NONE       | 0     |
| BARO_AUTODETECT | 1     |
| BARO_BMP085     | 2     |
| BARO_MS5611     | 3     |
| BARO_BMP280     | 4     |
| BARO_MS5607     | 5     |
| BARO_LPS25H     | 6     |
| BARO_SPL06      | 7     |
| BARO_BMP388     | 8     |
| BARO_DPS310     | 9     |
| BARO_B2SMPB     | 10    |
| BARO_MSP        | 11    |
| BARO_FAKE       | 12    |
| BARO_MAX        | 12    |

---

## Hardware Sensor Status (`hardwareSensorStatus`)

Status codes indicating the health of individual hardware sensors.

| Name                  | Value |
|-----------------------|-------|
| HW_SENSOR_NONE        | 0     |
| HW_SENSOR_OK          | 1     |
| HW_SENSOR_UNAVAILABLE | 2     |
| HW_SENSOR_UNHEALTHY   | 3     |

---

## Magnetometer Sensor Type (`magSensor`)

Type of magnetometer sensor configured.

| Name          | Value |
|---------------|-------|
| MAG_NONE      | 0     |
| MAG_AUTODETECT| 1     |
| MAG_HMC5883   | 2     |
| MAG_AK8975    | 3     |
| MAG_MAG3110   | 4     |
| MAG_AK8963    | 5     |
| MAG_IST8310   | 6     |
| MAG_QMC5883   | 7     |
| MAG_MPU9250   | 8     |
| MAG_IST8308   | 9     |
| MAG_LIS3MDL   | 10    |
| MAG_MSP       | 11    |
| MAG_RM3100    | 12    |
| MAG_VCM5883   | 13    |
| MAG_MLX90393  | 14    |
| MAG_FAKE      | 15    |
| MAG_MAX       | 15    |

---

## Logic Condition Operations (`logicOperation`)

Operations available within the Programming Framework's Logic Conditions.

| Name                                  | Value |
|---------------------------------------|-------|
| LOGIC_CONDITION_TRUE                  | 0     |
| LOGIC_CONDITION_EQUAL                 | 1     |
| LOGIC_CONDITION_GREATER_THAN          | 2     |
| LOGIC_CONDITION_LOWER_THAN            | 3     |
| LOGIC_CONDITION_LOW                   | 4     |
| LOGIC_CONDITION_MID                   | 5     |
| LOGIC_CONDITION_HIGH                  | 6     |
| LOGIC_CONDITION_AND                   | 7     |
| LOGIC_CONDITION_OR                    | 8     |
| LOGIC_CONDITION_XOR                   | 9     |
| LOGIC_CONDITION_NAND                  | 10    |
| LOGIC_CONDITION_NOR                   | 11    |
| LOGIC_CONDITION_NOT                   | 12    |
| LOGIC_CONDITION_STICKY                | 13    |
| LOGIC_CONDITION_ADD                   | 14    |
| LOGIC_CONDITION_SUB                   | 15    |
| LOGIC_CONDITION_MUL                   | 16    |
| LOGIC_CONDITION_DIV                   | 17    |
| LOGIC_CONDITION_GVAR_SET              | 18    |
| LOGIC_CONDITION_GVAR_INC              | 19    |
| LOGIC_CONDITION_GVAR_DEC              | 20    |
| LOGIC_CONDITION_PORT_SET              | 21    |
| LOGIC_CONDITION_OVERRIDE_ARMING_SAFETY| 22    |
| LOGIC_CONDITION_OVERRIDE_THROTTLE_SCALE| 23    |
| LOGIC_CONDITION_SWAP_ROLL_YAW         | 24    |
| LOGIC_CONDITION_SET_VTX_POWER_LEVEL   | 25    |
| LOGIC_CONDITION_INVERT_ROLL           | 26    |
| LOGIC_CONDITION_INVERT_PITCH          | 27    |
| LOGIC_CONDITION_INVERT_YAW            | 28    |
| LOGIC_CONDITION_OVERRIDE_THROTTLE     | 29    |
| LOGIC_CONDITION_SET_VTX_BAND          | 30    |
| LOGIC_CONDITION_SET_VTX_CHANNEL       | 31    |
| LOGIC_CONDITION_SET_OSD_LAYOUT        | 32    |
| LOGIC_CONDITION_SIN                   | 33    |
| LOGIC_CONDITION_COS                   | 34    |
| LOGIC_CONDITION_TAN                   | 35    |
| LOGIC_CONDITION_MAP_INPUT             | 36    |
| LOGIC_CONDITION_MAP_OUTPUT            | 37    |
| LOGIC_CONDITION_RC_CHANNEL_OVERRIDE   | 38    |
| LOGIC_CONDITION_SET_HEADING_TARGET    | 39    |
| LOGIC_CONDITION_MODULUS               | 40    |
| LOGIC_CONDITION_LOITER_OVERRIDE       | 41    |
| LOGIC_CONDITION_SET_PROFILE           | 42    |
| LOGIC_CONDITION_MIN                   | 43    |
| LOGIC_CONDITION_MAX                   | 44    |
| LOGIC_CONDITION_FLIGHT_AXIS_ANGLE_OVERRIDE | 45    |
| LOGIC_CONDITION_FLIGHT_AXIS_RATE_OVERRIDE | 46    |
| LOGIC_CONDITION_EDGE                  | 47    |
| LOGIC_CONDITION_DELAY                 | 48    |
| LOGIC_CONDITION_TIMER                 | 49    |
| LOGIC_CONDITION_DELTA                 | 50    |
| LOGIC_CONDITION_APPROX_EQUAL          | 51    |
| LOGIC_CONDITION_LED_PIN_PWM           | 52    |
| LOGIC_CONDITION_DISABLE_GPS_FIX       | 53    |
| LOGIC_CONDITION_RESET_MAG_CALIBRATION | 54    |
| LOGIC_CONDITION_SET_GIMBAL_SENSITIVITY| 55    |
| LOGIC_CONDITION_LAST                  | 56    |

---

## Logic Condition Flight Operands (`logicFlightOperands`)

Operands related to flight status available for Logic Conditions.

| Name                                          | Value |
|-----------------------------------------------|-------|
| LOGIC_CONDITION_OPERAND_FLIGHT_ARM_TIMER      | 0     |
| LOGIC_CONDITION_OPERAND_FLIGHT_HOME_DISTANCE  | 1     |
| LOGIC_CONDITION_OPERAND_FLIGHT_TRIP_DISTANCE  | 2     |
| LOGIC_CONDITION_OPERAND_FLIGHT_RSSI           | 3     |
| LOGIC_CONDITION_OPERAND_FLIGHT_VBAT           | 4     |
| LOGIC_CONDITION_OPERAND_FLIGHT_CELL_VOLTAGE   | 5     |
| LOGIC_CONDITION_OPERAND_FLIGHT_CURRENT        | 6     |
| LOGIC_CONDITION_OPERAND_FLIGHT_MAH_DRAWN      | 7     |
| LOGIC_CONDITION_OPERAND_FLIGHT_GPS_SATS       | 8     |
| LOGIC_CONDITION_OPERAND_FLIGHT_GROUD_SPEED    | 9     |
| LOGIC_CONDITION_OPERAND_FLIGHT_3D_SPEED       | 10    |
| LOGIC_CONDITION_OPERAND_FLIGHT_AIR_SPEED      | 11    |
| LOGIC_CONDITION_OPERAND_FLIGHT_ALTITUDE       | 12    |
| LOGIC_CONDITION_OPERAND_FLIGHT_VERTICAL_SPEED | 13    |
| LOGIC_CONDITION_OPERAND_FLIGHT_TROTTLE_POS    | 14    |
| LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_ROLL  | 15    |
| LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_PITCH | 16    |
| LOGIC_CONDITION_OPERAND_FLIGHT_IS_ARMED       | 17    |
| LOGIC_CONDITION_OPERAND_FLIGHT_IS_AUTOLAUNCH  | 18    |
| LOGIC_CONDITION_OPERAND_FLIGHT_IS_ALTITUDE_CONTROL | 19    |
| LOGIC_CONDITION_OPERAND_FLIGHT_IS_POSITION_CONTROL | 20    |
| LOGIC_CONDITION_OPERAND_FLIGHT_IS_EMERGENCY_LANDING | 21    |
| LOGIC_CONDITION_OPERAND_FLIGHT_IS_RTH         | 22    |
| LOGIC_CONDITION_OPERAND_FLIGHT_IS_LANDING     | 23    |
| LOGIC_CONDITION_OPERAND_FLIGHT_IS_FAILSAFE    | 24    |
| LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_ROLL| 25    |
| LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_PITCH| 26    |
| LOGIC_CONDITION_OPERAND_FLIGHT_STABILIZED_YAW | 27    |
| LOGIC_CONDITION_OPERAND_FLIGHT_3D_HOME_DISTANCE | 28    |
| LOGIC_CONDITION_OPERAND_FLIGHT_LQ_UPLINK      | 29    |
| LOGIC_CONDITION_OPERAND_FLIGHT_SNR            | 30    |
| LOGIC_CONDITION_OPERAND_FLIGHT_GPS_VALID      | 31    |
| LOGIC_CONDITION_OPERAND_FLIGHT_LOITER_RADIUS  | 32    |
| LOGIC_CONDITION_OPERAND_FLIGHT_ACTIVE_PROFILE | 33    |
| LOGIC_CONDITION_OPERAND_FLIGHT_BATT_CELLS     | 34    |
| LOGIC_CONDITION_OPERAND_FLIGHT_AGL_STATUS     | 35    |
| LOGIC_CONDITION_OPERAND_FLIGHT_AGL            | 36    |
| LOGIC_CONDITION_OPERAND_FLIGHT_RANGEFINDER_RAW| 37    |
| LOGIC_CONDITION_OPERAND_FLIGHT_ACTIVE_MIXER_PROFILE | 38    |
| LOGIC_CONDITION_OPERAND_FLIGHT_MIXER_TRANSITION_ACTIVE | 39    |
| LOGIC_CONDITION_OPERAND_FLIGHT_ATTITUDE_YAW   | 40    |
| LOGIC_CONDITION_OPERAND_FLIGHT_FW_LAND_STATE  | 41    |
| LOGIC_CONDITION_OPERAND_FLIGHT_BATT_PROFILE   | 42    |
| LOGIC_CONDITION_OPERAND_FLIGHT_FLOWN_LOITER_RADIUS | 43    |
| LOGIC_CONDITION_OPERAND_FLIGHT_LQ_DOWNLINK    | 44    |
| LOGIC_CONDITION_OPERAND_FLIGHT_UPLINK_RSSI_DBM| 45    |

---

## Logic Condition Flight Mode Operands (`logicFlightModeOperands`)

Operands related to active flight modes available for Logic Conditions.

| Name                                          | Value |
|-----------------------------------------------|-------|
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_FAILSAFE  | 0     |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_MANUAL    | 1     |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_RTH       | 2     |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_POSHOLD   | 3     |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_CRUISE    | 4     |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ALTHOLD   | 5     |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ANGLE     | 6     |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_HORIZON   | 7     |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_AIR       | 8     |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER1     | 9     |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER2     | 10    |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_COURSE_HOLD | 11    |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER3     | 12    |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_USER4     | 13    |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ACRO      | 14    |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_WAYPOINT_MISSION | 15    |
| LOGIC_CONDITION_OPERAND_FLIGHT_MODE_ANGLEHOLD | 16    |

---

## Logic Condition Waypoint Operands (`logicWaypointOperands`)

Operands related to waypoint mission status available for Logic Conditions.

| Name                                                  | Value |
|-------------------------------------------------------|-------|
| LOGIC_CONDITION_OPERAND_WAYPOINTS_IS_WP               | 0     |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_WAYPOINT_INDEX      | 1     |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_WAYPOINT_ACTION     | 2     |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_NEXT_WAYPOINT_ACTION| 3     |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_WAYPOINT_DISTANCE   | 4     |
| LOGIC_CONDTIION_OPERAND_WAYPOINTS_DISTANCE_FROM_WAYPOINT | 5     |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_USER1_ACTION        | 6     |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_USER2_ACTION        | 7     |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_USER3_ACTION        | 8     |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_USER4_ACTION        | 9     |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_USER1_ACTION_NEXT_WP| 10    |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_USER2_ACTION_NEXT_WP| 11    |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_USER3_ACTION_NEXT_WP| 12    |
| LOGIC_CONDITION_OPERAND_WAYPOINTS_USER4_ACTION_NEXT_WP| 13    |

---

## Logic Condition Global Flags (`logicConditionsGlobalFlags`)

Flags indicating which global settings can be overridden by Logic Conditions.

| Name                                                | Value      |
|-----------------------------------------------------|------------|
| LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_ARMING_SAFETY  | `(1 << 0)` |
| LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE_SCALE | `(1 << 1)` |
| LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_SWAP_ROLL_YAW  | `(1 << 2)` |
| LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_ROLL    | `(1 << 3)` |
| LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_PITCH   | `(1 << 4)` |
| LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_INVERT_YAW     | `(1 << 5)` |
| LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_THROTTLE       | `(1 << 6)` |
| LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_OSD_LAYOUT     | `(1 << 7)` |
| LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_RC_CHANNEL     | `(1 << 8)` |
| LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_LOITER_RADIUS  | `(1 << 9)` |
| LOGIC_CONDITION_GLOBAL_FLAG_OVERRIDE_FLIGHT_AXIS    | `(1 << 10)`|
| LOGIC_CONDITION_GLOBAL_FLAG_DISABLE_GPS_FIX         | `(1 << 11)`|

---

## Logic Condition Flags (`logicConditionFlags`)

Flags applied to individual Logic Conditions.

| Name                             | Value     |
|----------------------------------|-----------|
| LOGIC_CONDITION_FLAG_LATCH       | `1 << 0`  |
| LOGIC_CONDITION_FLAG_TIMEOUT_SATISFIED | `1 << 1`  |

---

## RX Frame State (`rxFrameState`)

State flags for processing received RC frames.

| Name                           | Value     |
|--------------------------------|-----------|
| RX_FRAME_PENDING               | 0         |
| RX_FRAME_COMPLETE              | `(1 << 0)`|
| RX_FRAME_FAILSAFE              | `(1 << 1)`|
| RX_FRAME_PROCESSING_REQUIRED   | `(1 << 2)`|
| RX_FRAME_DROPPED               | `(1 << 3)`|

---

## RX Receiver Type (`rxReceiverType`)

General type of receiver configured.

| Name             | Value |
|------------------|-------|
| RX_TYPE_NONE     | 0     |
| RX_TYPE_SERIAL   | 1     |
| RX_TYPE_MSP      | 2     |
| RX_TYPE_SIM      | 3     |

---

## RX Serial Receiver Type (`rxSerialReceiverType`)

Specific protocol used for serial receivers.

| Name                   | Value |
|------------------------|-------|
| SERIALRX_SPEKTRUM1024  | 0     |
| SERIALRX_SPEKTRUM2048  | 1     |
| SERIALRX_SBUS          | 2     |
| SERIALRX_SUMD          | 3     |
| SERIALRX_IBUS          | 4     |
| SERIALRX_JETIEXBUS     | 5     |
| SERIALRX_CRSF          | 6     |
| SERIALRX_FPORT         | 7     |
| SERIALRX_SBUS_FAST     | 8     |
| SERIALRX_FPORT2        | 9     |
| SERIALRX_SRXL2         | 10    |
| SERIALRX_GHST          | 11    |
| SERIALRX_MAVLINK       | 12    |
| SERIALRX_FBUS          | 13    |
| SERIALRX_SBUS2         | 14    |

---

## RSSI Source (`rssiSource`)

Source used for determining the RSSI value.

| Name                   | Value |
|------------------------|-------|
| RSSI_SOURCE_NONE       | 0     |
| RSSI_SOURCE_AUTO       | 1     |
| RSSI_SOURCE_ADC        | 2     |
| RSSI_SOURCE_RX_CHANNEL | 3     |
| RSSI_SOURCE_RX_PROTOCOL| 4     |
| RSSI_SOURCE_MSP        | 5     |

---

## CRSF Addresses (`crsfAddress`)

Device addresses used in the Crossfire (CRSF) protocol.

| Name                        | Value |
|-----------------------------|-------|
| CRSF_ADDRESS_BROADCAST      | 0     |
| CRSF_ADDRESS_USB            | 16    |
| CRSF_ADDRESS_TBS_CORE_PNP_PRO | 128   |
| CRSF_ADDRESS_RESERVED1      | 138   |
| CRSF_ADDRESS_CURRENT_SENSOR | 192   |
| CRSF_ADDRESS_GPS            | 194   |
| CRSF_ADDRESS_TBS_BLACKBOX   | 196   |
| CRSF_ADDRESS_FLIGHT_CONTROLLER | 200   |
| CRSF_ADDRESS_RESERVED2      | 202   |
| CRSF_ADDRESS_RACE_TAG       | 204   |
| CRSF_ADDRESS_RADIO_TRANSMITTER | 234   |
| CRSF_ADDRESS_CRSF_RECEIVER  | 236   |
| CRSF_ADDRESS_CRSF_TRANSMITTER | 238   |

---

## CRSF Frame Types (`crsfFrameType`)

Frame type identifiers used in the Crossfire (CRSF) protocol.

| Name                              | Value |
|-----------------------------------|-------|
| CRSF_FRAMETYPE_GPS                | 2     |
| CRSF_FRAMETYPE_VARIO_SENSOR       | 7     |
| CRSF_FRAMETYPE_BATTERY_SENSOR     | 8     |
| CRSF_FRAMETYPE_LINK_STATISTICS    | 20    |
| CRSF_FRAMETYPE_RC_CHANNELS_PACKED | 22    |
| CRSF_FRAMETYPE_ATTITUDE           | 30    |
| CRSF_FRAMETYPE_FLIGHT_MODE        | 33    |
| CRSF_FRAMETYPE_DEVICE_PING        | 40    |
| CRSF_FRAMETYPE_DEVICE_INFO        | 41    |
| CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY | 43    |
| CRSF_FRAMETYPE_PARAMETER_READ     | 44    |
| CRSF_FRAMETYPE_PARAMETER_WRITE    | 45    |
| CRSF_FRAMETYPE_COMMAND            | 50    |
| CRSF_FRAMETYPE_MSP_REQ            | 122   |
| CRSF_FRAMETYPE_MSP_RESP           | 123   |
| CRSF_FRAMETYPE_MSP_WRITE          | 124   |
| CRSF_FRAMETYPE_DISPLAYPORT_CMD    | 125   |

---

## GHST Addresses (`ghstAddr`)

Device addresses used in the Ghost (GHST) protocol.

| Name                     | Value |
|--------------------------|-------|
| GHST_ADDR_RADIO          | 128   |
| GHST_ADDR_TX_MODULE_SYM  | 129   |
| GHST_ADDR_TX_MODULE_ASYM | 136   |
| GHST_ADDR_FC             | 130   |
| GHST_ADDR_GOGGLES        | 131   |
| GHST_ADDR_QUANTUM_TEE1   | 132   |
| GHST_ADDR_QUANTUM_TEE2   | 133   |
| GHST_ADDR_QUANTUM_GW1    | 134   |
| GHST_ADDR_5G_CLK         | 135   |
| GHST_ADDR_RX             | 137   |

---

## GHST Uplink Frame Types (`ghstUl`)

Uplink (RX to FC) frame types in the Ghost (GHST) protocol.

| Name                         | Value |
|------------------------------|-------|
| GHST_UL_RC_CHANS_HS4_FIRST   | 16    |
| GHST_UL_RC_CHANS_HS4_5TO8    | 16    |
| GHST_UL_RC_CHANS_HS4_9TO12   | 17    |
| GHST_UL_RC_CHANS_HS4_13TO16  | 18    |
| GHST_UL_RC_CHANS_HS4_RSSI    | 19    |
| GHST_UL_RC_CHANS_HS4_LAST    | 31    |

---

## GHST Downlink Frame Types (`ghstDl`)

Downlink (FC to RX/TX) frame types in the Ghost (GHST) protocol.

| Name                  | Value |
|-----------------------|-------|
| GHST_DL_OPENTX_SYNC   | 32    |
| GHST_DL_LINK_STAT     | 33    |
| GHST_DL_VTX_STAT      | 34    |
| GHST_DL_PACK_STAT     | 35    |
| GHST_DL_GPS_PRIMARY   | 37    |
| GHST_DL_GPS_SECONDARY | 38    |

---

## SRXL2 State (`Srxl2State`)

States for the SRXL2 protocol handler.

| Name                 | Value |
|----------------------|-------|
| Disabled             | 0     |
| ListenForActivity    | 1     |
| SendHandshake        | 2     |
| ListenForHandshake   | 3     |
| Running              | 4     |

---

## SRXL2 Packet Type (`Srxl2PacketType`)

Packet type identifiers in the SRXL2 protocol.

| Name                       | Value |
|----------------------------|-------|
| Handshake                  | 33    |
| BindInfo                   | 65    |
| ParameterConfiguration     | 80    |
| SignalQuality              | 85    |
| TelemetrySensorData        | 128   |
| ControlData                | 205   |

---

## SRXL2 Control Data Command (`Srxl2ControlDataCommand`)

Commands within an SRXL2 ControlData packet.

| Name                  | Value |
|-----------------------|-------|
| ChannelData           | 0     |
| FailsafeChannelData   | 1     |
| VTXData               | 2     |

---

## SRXL2 Device Type (`Srxl2DeviceType`)

Device type identifiers in the SRXL2 protocol.

| Name             | Value |
|------------------|-------|
| NoDevice         | 0     |
| RemoteReceiver   | 1     |
| Receiver         | 2     |
| FlightController | 3     |
| ESC              | 4     |
| Reserved         | 5     |
| SRXLServo        | 6     |
| SRXLServo_2      | 7     |
| VTX              | 8     |

---

## SRXL2 Device ID (`Srxl2DeviceId`)

Device ID assignments in the SRXL2 protocol.

| Name                      | Value |
|---------------------------|-------|
| FlightControllerDefault   | 48    |
| FlightControllerMax       | 63    |
| Broadcast                 | 255   |

---

## SRXL2 Bind Request (`Srxl2BindRequest`)

Commands related to SRXL2 binding procedure.

| Name                | Value |
|---------------------|-------|
| EnterBindMode       | 235   |
| RequestBindStatus   | 181   |
| BoundDataReport     | 219   |
| SetBindInfo         | 91    |

---

## SRXL2 Bind Type (`Srxl2BindType`)

Type of Spektrum bind protocol identified.

| Name               | Value |
|--------------------|-------|
| NotBound           | 0     |
| DSM2_1024_22ms     | 1     |
| DSM2_1024_MC24     | 2     |
| DMS2_2048_11ms     | 18    |
| DMSX_22ms          | 162   |
| DMSX_11ms          | 178   |
| Surface_DSM2_16_5ms| 99    |
| DSMR_11ms_22ms     | 226   |
| DSMR_5_5ms         | 228   |

---

## LTM Update Rate (`ltmUpdateRate`)

Rate selection for Light Telemetry (LTM) updates.

| Name          | Value |
|---------------|-------|
| LTM_RATE_NORMAL | 0     |
| LTM_RATE_MEDIUM | 1     |
| LTM_RATE_SLOW   | 2     |

---

## MAVLink Radio Type (`mavlinkRadio`)

Identifies the type of radio used for MAVLink telemetry.

| Name                  | Value |
|-----------------------|-------|
| MAVLINK_RADIO_GENERIC | 0     |
| MAVLINK_RADIO_ELRS    | 1     |
| MAVLINK_RADIO_SIK     | 2     |

---

## SmartPort Fuel Unit (`smartportFuelUnit`)

Unit used for reporting fuel/capacity via FrSky SmartPort.

| Name                      | Value |
|---------------------------|-------|
| SMARTPORT_FUEL_UNIT_PERCENT | 0     |
| SMARTPORT_FUEL_UNIT_MAH   | 1     |
| SMARTPORT_FUEL_UNIT_MWH   | 2     |

---

## HoTT EAM Alarm 1 Flags (`hottEamAlarm1Flag`)

Alarm flags in the HoTT Electric Air Module telemetry frame (Byte 1).

| Name                              | Value     |
|-----------------------------------|-----------|
| HOTT_EAM_ALARM1_FLAG_NONE         | 0         |
| HOTT_EAM_ALARM1_FLAG_MAH          | `(1 << 0)`|
| HOTT_EAM_ALARM1_FLAG_BATTERY_1    | `(1 << 1)`|
| HOTT_EAM_ALARM1_FLAG_BATTERY_2    | `(1 << 2)`|
| HOTT_EAM_ALARM1_FLAG_TEMPERATURE_1| `(1 << 3)`|
| HOTT_EAM_ALARM1_FLAG_TEMPERATURE_2| `(1 << 4)`|
| HOTT_EAM_ALARM1_FLAG_ALTITUDE     | `(1 << 5)`|
| HOTT_EAM_ALARM1_FLAG_CURRENT      | `(1 << 6)`|
| HOTT_EAM_ALARM1_FLAG_MAIN_VOLTAGE | `(1 << 7)`|

---

## HoTT EAM Alarm 2 Flags (`hottEamAlarm2Flag`)

Alarm flags in the HoTT Electric Air Module telemetry frame (Byte 2).

| Name                                  | Value     |
|---------------------------------------|-----------|
| HOTT_EAM_ALARM2_FLAG_NONE             | 0         |
| HOTT_EAM_ALARM2_FLAG_MS               | `(1 << 0)`|
| HOTT_EAM_ALARM2_FLAG_M3S              | `(1 << 1)`|
| HOTT_EAM_ALARM2_FLAG_ALTITUDE_DUPLICATE| `(1 << 2)`|
| HOTT_EAM_ALARM2_FLAG_MS_DUPLICATE     | `(1 << 3)`|
| HOTT_EAM_ALARM2_FLAG_M3S_DUPLICATE    | `(1 << 4)`|
| HOTT_EAM_ALARM2_FLAG_UNKNOWN_1        | `(1 << 5)`|
| HOTT_EAM_ALARM2_FLAG_UNKNOWN_2        | `(1 << 6)`|
| HOTT_EAM_ALARM2_FLAG_ON_SIGN_OR_TEXT_ACTIVE | `(1 << 7)`|

---

## LTM Frame Types (`ltm_frame`)

Frame identifiers used in Light Telemetry (LTM).

| Name            | Value |
|-----------------|-------|
| LTM_FRAME_START | 0     |
| LTM_AFRAME      | 0     | *Altitude* |
| LTM_SFRAME      | 2     | *Status* |
| LTM_GFRAME      | 3     | *GPS* |
| LTM_OFRAME      | 4     | *Origin* |
| LTM_XFRAME      | 5     | *Extended GPS* |
| LTM_NFRAME      | 6     | *Navigation* |
| LTM_FRAME_COUNT | 7     |

---

## LTM Modes (`ltm_modes`)

Flight mode identifiers used in Light Telemetry (LTM).

| Name             | Value |
|------------------|-------|
| LTM_MODE_MANUAL    | 0     |
| LTM_MODE_RATE      | 1     |
| LTM_MODE_ANGLE     | 2     |
| LTM_MODE_HORIZON   | 3     |
| LTM_MODE_ACRO      | 4     |
| LTM_MODE_STABALIZED1| 5     |
| LTM_MODE_STABALIZED2| 6     |
| LTM_MODE_STABILIZED3| 7     |
| LTM_MODE_ALTHOLD   | 8     |
| LTM_MODE_GPSHOLD   | 9     |
| LTM_MODE_WAYPOINTS | 10    |
| LTM_MODE_HEADHOLD  | 11    |
| LTM_MODE_CIRCLE    | 12    |
| LTM_MODE_RTH       | 13    |
| LTM_MODE_FOLLOWWME | 14    |
| LTM_MODE_LAND      | 15    |
| LTM_MODE_FLYBYWIRE1| 16    |
| LTM_MODE_FLYBYWIRE2| 17    |
| LTM_MODE_CRUISE    | 18    |
| LTM_MODE_UNKNOWN   | 19    |
| LTM_MODE_LAUNCH    | 20    |
| LTM_MODE_AUTOTUNE  | 21    |

---

## iBUS Sensor Type (Legacy) (`ibusSensorType`)

Legacy sensor type identifiers for FlySky iBUS telemetry.

| Name                           | Value |
|--------------------------------|-------|
| IBUS_MEAS_TYPE_INTERNAL_VOLTAGE| 0     |
| IBUS_MEAS_TYPE_TEMPERATURE     | 1     |
| IBUS_MEAS_TYPE_RPM             | 2     |
| IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE| 3     |
| IBUS_MEAS_TYPE_HEADING         | 4     |
| IBUS_MEAS_TYPE_CURRENT         | 5     |
| IBUS_MEAS_TYPE_CLIMB           | 6     |
| IBUS_MEAS_TYPE_ACC_Z           | 7     |
| IBUS_MEAS_TYPE_ACC_Y           | 8     |
| IBUS_MEAS_TYPE_ACC_X           | 9     |
| IBUS_MEAS_TYPE_VSPEED          | 10    |
| IBUS_MEAS_TYPE_SPEED           | 11    |
| IBUS_MEAS_TYPE_DIST            | 12    |
| IBUS_MEAS_TYPE_ARMED           | 13    |
| IBUS_MEAS_TYPE_MODE            | 14    |
| IBUS_MEAS_TYPE_PRES            | 65    |
| IBUS_MEAS_TYPE_SPE             | 126   |
| IBUS_MEAS_TYPE_COG             | 128   |
| IBUS_MEAS_TYPE_GPS_STATUS      | 129   |
| IBUS_MEAS_TYPE_GPS_LON         | 130   |
| IBUS_MEAS_TYPE_GPS_LAT         | 131   |
| IBUS_MEAS_TYPE_ALT             | 132   |
| IBUS_MEAS_TYPE_S85             | 133   |
| IBUS_MEAS_TYPE_S86             | 134   |
| IBUS_MEAS_TYPE_S87             | 135   |
| IBUS_MEAS_TYPE_S88             | 136   |
| IBUS_MEAS_TYPE_S89             | 137   |
| IBUS_MEAS_TYPE_S8A             | 138   |
| IBUS_MEAS_TYPE_GALT            | 249   |
| IBUS_MEAS_TYPE_GPS             | 253   |

---

## iBUS Sensor Type (Type 1) (`ibusSensorType1`)

Type 1 sensor identifiers for FlySky iBUS telemetry.

| Name                         | Value |
|------------------------------|-------|
| IBUS_MEAS_TYPE1_INTV         | 0     |
| IBUS_MEAS_TYPE1_TEM          | 1     |
| IBUS_MEAS_TYPE1_MOT          | 2     |
| IBUS_MEAS_TYPE1_EXTV         | 3     |
| IBUS_MEAS_TYPE1_CELL         | 4     |
| IBUS_MEAS_TYPE1_BAT_CURR     | 5     |
| IBUS_MEAS_TYPE1_FUEL         | 6     |
| IBUS_MEAS_TYPE1_RPM          | 7     |
| IBUS_MEAS_TYPE1_CMP_HEAD     | 8     |
| IBUS_MEAS_TYPE1_CLIMB_RATE   | 9     |
| IBUS_MEAS_TYPE1_COG          | 10    |
| IBUS_MEAS_TYPE1_GPS_STATUS   | 11    |
| IBUS_MEAS_TYPE1_ACC_X        | 12    |
| IBUS_MEAS_TYPE1_ACC_Y        | 13    |
| IBUS_MEAS_TYPE1_ACC_Z        | 14    |
| IBUS_MEAS_TYPE1_ROLL         | 15    |
| IBUS_MEAS_TYPE1_PITCH        | 16    |
| IBUS_MEAS_TYPE1_YAW          | 17    |
| IBUS_MEAS_TYPE1_VERTICAL_SPEED | 18    |
| IBUS_MEAS_TYPE1_GROUND_SPEED | 19    |
| IBUS_MEAS_TYPE1_GPS_DIST     | 20    |
| IBUS_MEAS_TYPE1_ARMED        | 21    |
| IBUS_MEAS_TYPE1_FLIGHT_MODE  | 22    |
| IBUS_MEAS_TYPE1_PRES         | 65    |
| IBUS_MEAS_TYPE1_SPE          | 126   |
| IBUS_MEAS_TYPE1_GPS_LAT      | 128   |
| IBUS_MEAS_TYPE1_GPS_LON      | 129   |
| IBUS_MEAS_TYPE1_GPS_ALT      | 130   |
| IBUS_MEAS_TYPE1_ALT          | 131   |
| IBUS_MEAS_TYPE1_S84          | 132   |
| IBUS_MEAS_TYPE1_S85          | 133   |
| IBUS_MEAS_TYPE1_S86          | 134   |
| IBUS_MEAS_TYPE1_S87          | 135   |
| IBUS_MEAS_TYPE1_S88          | 136   |
| IBUS_MEAS_TYPE1_S89          | 137   |
| IBUS_MEAS_TYPE1_S8a          | 138   |

---

## iBUS Sensor Value (`ibusSensorValue`)

Value identifiers for FlySky iBUS telemetry.

| Name                       | Value |
|----------------------------|-------|
| IBUS_MEAS_VALUE_NONE       | 0     |
| IBUS_MEAS_VALUE_TEMPERATURE| 1     |
| IBUS_MEAS_VALUE_MOT        | 2     |
| IBUS_MEAS_VALUE_EXTERNAL_VOLTAGE| 3     |
| IBUS_MEAS_VALUE_CELL       | 4     |
| IBUS_MEAS_VALUE_CURRENT    | 5     |
| IBUS_MEAS_VALUE_FUEL       | 6     |
| IBUS_MEAS_VALUE_RPM        | 7     |
| IBUS_MEAS_VALUE_HEADING    | 8     |
| IBUS_MEAS_VALUE_CLIMB      | 9     |
| IBUS_MEAS_VALUE_COG        | 10    |
| IBUS_MEAS_VALUE_GPS_STATUS | 11    |
| IBUS_MEAS_VALUE_ACC_X      | 12    |
| IBUS_MEAS_VALUE_ACC_Y      | 13    |
| IBUS_MEAS_VALUE_ACC_Z      | 14    |
| IBUS_MEAS_VALUE_ROLL       | 15    |
| IBUS_MEAS_VALUE_PITCH      | 16    |
| IBUS_MEAS_VALUE_YAW        | 17    |
| IBUS_MEAS_VALUE_VSPEED     | 18    |
| IBUS_MEAS_VALUE_SPEED      | 19    |
| IBUS_MEAS_VALUE_DIST       | 20    |
| IBUS_MEAS_VALUE_ARMED      | 21    |
| IBUS_MEAS_VALUE_MODE       | 22    |
| IBUS_MEAS_VALUE_PRES       | 65    |
| IBUS_MEAS_VALUE_SPE        | 126   |
| IBUS_MEAS_VALUE_GPS_LAT    | 128   |
| IBUS_MEAS_VALUE_GPS_LON    | 129   |
| IBUS_MEAS_VALUE_GALT4      | 130   |
| IBUS_MEAS_VALUE_ALT4       | 131   |
| IBUS_MEAS_VALUE_GALT       | 132   |
| IBUS_MEAS_VALUE_ALT        | 133   |
| IBUS_MEAS_VALUE_STATUS     | 135   |
| IBUS_MEAS_VALUE_GPS_LAT1   | 136   |
| IBUS_MEAS_VALUE_GPS_LON1   | 137   |
| IBUS_MEAS_VALUE_GPS_LAT2   | 144   |
| IBUS_MEAS_VALUE_GPS_LON2   | 145   |
| IBUS_MEAS_VALUE_GPS        | 253   |

---

## GPS Provider (`gpsProvider`)

Type of GPS module driver being used.

| Name               | Value |
|--------------------|-------|
| GPS_UBLOX          | 0     |
| GPS_MSP            | 1     |
| GPS_FAKE           | 2     |
| GPS_PROVIDER_COUNT | 3     |

---

## SBAS Mode (`sbasMode`)

Satellite-Based Augmentation System mode for GPS.

| Name         | Value |
|--------------|-------|
| SBAS_AUTO    | 0     |
| SBAS_EGNOS   | 1     |
| SBAS_WAAS    | 2     |
| SBAS_MSAS    | 3     |
| SBAS_GAGAN   | 4     |
| SBAS_SPAN    | 5     |
| SBAS_NONE    | 6     |

---

## GPS Baud Rate (`gpsBaudRate`)

Baud rate index used for communicating with the GPS module.

| Name                 | Value | Corresponding Baud Rate |
|----------------------|-------|-------------------------|
| GPS_BAUDRATE_115200  | 0     | 115200                  |
| GPS_BAUDRATE_57600   | 1     | 57600                   |
| GPS_BAUDRATE_38400   | 2     | 38400                   |
| GPS_BAUDRATE_19200   | 3     | 19200                   |
| GPS_BAUDRATE_9600    | 4     | 9600                    |
| GPS_BAUDRATE_230400  | 5     | 230400                  |
| GPS_BAUDRATE_460800  | 6     | 460800                  |
| GPS_BAUDRATE_921600  | 7     | 921600                  |
| GPS_BAUDRATE_COUNT   | 8     |                         |

---

## GPS Auto Config (`gpsAutoConfig`)

Enable/disable automatic configuration of the GPS module by INAV.

| Name                 | Value |
|----------------------|-------|
| GPS_AUTOCONFIG_OFF   | 0     |
| GPS_AUTOCONFIG_ON    | 1     |

---

## GPS Auto Baud (`gpsAutoBaud`)

Enable/disable automatic baud rate detection for the GPS module.

| Name               | Value |
|--------------------|-------|
| GPS_AUTOBAUD_OFF   | 0     |
| GPS_AUTOBAUD_ON    | 1     |

---

## GPS Dynamic Model (`gpsDynModel`)

Dynamic platform model used by the GPS module for better filtering.

| Name                  | Value |
|-----------------------|-------|
| GPS_DYNMODEL_PEDESTRIAN | 0     |
| GPS_DYNMODEL_AUTOMOTIVE | 1     |
| GPS_DYNMODEL_AIR_1G   | 2     |
| GPS_DYNMODEL_AIR_2G   | 3     |
| GPS_DYNMODEL_AIR_4G   | 4     |
| GPS_DYNMODEL_SEA      | 5     |
| GPS_DYNMODEL_MOWER    | 6     |

---

## GPS Fix Type (`gpsFixType`)

Type of position fix acquired by the GPS module.

| Name               | Value |
|--------------------|-------|
| GPS_NO_FIX         | 0     |
| GPS_FIX_2D         | 1     |
| GPS_FIX_3D         | 2     |

---

## DisplayPort MSP Commands (`displayportMspCommand`)

Commands used internally for MSP-based DisplayPort (OSD) communication.

| Name                  | Value |
|-----------------------|-------|
| MSP_DP_HEARTBEAT      | 0     |
| MSP_DP_RELEASE        | 1     |
| MSP_DP_CLEAR_SCREEN   | 2     |
| MSP_DP_WRITE_STRING   | 3     |
| MSP_DP_DRAW_SCREEN    | 4     |
| MSP_DP_OPTIONS        | 5     |
| MSP_DP_SYS            | 6     |
| MSP_DP_COUNT          | 7     |

---

## Gimbal Headtracker Serial State (`gimbalHeadtrackerState`)

Internal states for parsing serial headtracker data.

| Name              | Value |
|-------------------|-------|
| WAITING_HDR1      | 0     |
| WAITING_HDR2      | 1     |
| WAITING_PAYLOAD   | 2     |
| WAITING_CRCH      | 3     |
| WAITING_CRCL      | 4     |

---

## VTX Lower Power Disarm (`vtxLowerPowerDisarm`)

Configures when the VTX should use low power mode based on arm state.

| Name                               | Value |
|------------------------------------|-------|
| VTX_LOW_POWER_DISARM_OFF           | 0     |
| VTX_LOW_POWER_DISARM_ALWAYS        | 1     |
| VTX_LOW_POWER_DISARM_UNTIL_FIRST_ARM | 2     |

---

## Port Sharing (`portSharing`)

Indicates if a serial port resource is shared or exclusive.

| Name                   | Value |
|------------------------|-------|
| PORTSHARING_UNUSED     | 0     |
| PORTSHARING_NOT_SHARED | 1     |
| PORTSHARING_SHARED     | 2     |

---

## Serial Port Function (`serialPortFunction`)

Bitmask defining the functions enabled on a serial port.

| Name                             | Value      |
|----------------------------------|------------|
| FUNCTION_NONE                    | 0          |
| FUNCTION_MSP                     | `(1 << 0)` |
| FUNCTION_GPS                     | `(1 << 1)` |
| FUNCTION_UNUSED_3                | `(1 << 2)` |
| FUNCTION_TELEMETRY_HOTT          | `(1 << 3)` |
| FUNCTION_TELEMETRY_LTM           | `(1 << 4)` |
| FUNCTION_TELEMETRY_SMARTPORT     | `(1 << 5)` |
| FUNCTION_RX_SERIAL               | `(1 << 6)` |
| FUNCTION_BLACKBOX                | `(1 << 7)` |
| FUNCTION_TELEMETRY_MAVLINK       | `(1 << 8)` |
| FUNCTION_TELEMETRY_IBUS          | `(1 << 9)` |
| FUNCTION_RCDEVICE                | `(1 << 10)`|
| FUNCTION_VTX_SMARTAUDIO          | `(1 << 11)`|
| FUNCTION_VTX_TRAMP               | `(1 << 12)`|
| FUNCTION_UNUSED_1                | `(1 << 13)`|
| FUNCTION_OPTICAL_FLOW            | `(1 << 14)`|
| FUNCTION_LOG                     | `(1 << 15)`|
| FUNCTION_RANGEFINDER             | `(1 << 16)`|
| FUNCTION_VTX_FFPV                | `(1 << 17)`|
| FUNCTION_ESCSERIAL               | `(1 << 18)`|
| FUNCTION_TELEMETRY_SIM           | `(1 << 19)`|
| FUNCTION_FRSKY_OSD               | `(1 << 20)`|
| FUNCTION_DJI_HD_OSD              | `(1 << 21)`|
| FUNCTION_SERVO_SERIAL            | `(1 << 22)`|
| FUNCTION_TELEMETRY_SMARTPORT_MASTER | `(1 << 23)`|
| FUNCTION_UNUSED_2                | `(1 << 24)`|
| FUNCTION_MSP_OSD                 | `(1 << 25)`|
| FUNCTION_GIMBAL                  | `(1 << 26)`|
| FUNCTION_GIMBAL_HEADTRACKER      | `(1 << 27)`|

---

## Baud Rate Index (`baudRate`)

Indices mapping to standard serial baud rates.

| Name         | Value | Corresponding Baud Rate |
|--------------|-------|-------------------------|
| BAUD_AUTO    | 0     | Auto                    |
| BAUD_1200    | 1     | 1200                    |
| BAUD_2400    | 2     | 2400                    |
| BAUD_4800    | 3     | 4800                    |
| BAUD_9600    | 4     | 9600                    |
| BAUD_19200   | 5     | 19200                   |
| BAUD_38400   | 6     | 38400                   |
| BAUD_57600   | 7     | 57600                   |
| BAUD_115200  | 8     | 115200                  |
| BAUD_230400  | 9     | 230400                  |
| BAUD_250000  | 10    | 250000                  |
| BAUD_460800  | 11    | 460800                  |
| BAUD_921600  | 12    | 921600                  |
| BAUD_1000000 | 13    | 1000000                 |
| BAUD_1500000 | 14    | 1500000                 |
| BAUD_2000000 | 15    | 2000000                 |
| BAUD_2470000 | 16    | 2470000                 |
| BAUD_MIN     | 0     |                         |
| BAUD_MAX     | 16    |                         |

---

## Serial Port Identifier (`serialPortIdentifier`)

Identifiers for hardware and software serial ports.

| Name                     | Value |
|--------------------------|-------|
| SERIAL_PORT_NONE         | -1    |
| SERIAL_PORT_USART1       | 0     |
| SERIAL_PORT_USART2       | 2     |
| SERIAL_PORT_USART3       | 3     |
| SERIAL_PORT_USART4       | 4     |
| SERIAL_PORT_USART5       | 5     |
| SERIAL_PORT_USART6       | 6     |
| SERIAL_PORT_USART7       | 7     |
| SERIAL_PORT_USART8       | 8     |
| SERIAL_PORT_USB_VCP      | 20    |
| SERIAL_PORT_SOFTSERIAL1  | 30    |
| SERIAL_PORT_SOFTSERIAL2  | 11    |
| SERIAL_PORT_IDENTIFIER_MAX | 11    | *Note: MAX seems low* |

---

## RCDevice Features (`rcdevice_features`)

Features supported by the RCDevice protocol (e.g., RunCam).

| Name                                             | Value      |
|--------------------------------------------------|------------|
| RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON  | `(1 << 0)` |
| RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON   | `(1 << 1)` |
| RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE            | `(1 << 2)` |
| RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE | `(1 << 3)` |
| RCDEVICE_PROTOCOL_FEATURE_START_RECORDING        | `(1 << 6)` |
| RCDEVICE_PROTOCOL_FEATURE_STOP_RECORDING         | `(1 << 7)` |
| RCDEVICE_PROTOCOL_FEATURE_CMS_MENU               | `(1 << 8)` |

---

## RCDevice Camera Control Operation (`rcdevice_camera_control_opeation`)

Operations for controlling a camera via RCDevice protocol.

| Name                                             | Value |
|--------------------------------------------------|-------|
| RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_WIFI_BTN     | 0     |
| RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_POWER_BTN    | 1     |
| RCDEVICE_PROTOCOL_CAM_CTRL_CHANGE_MODE           | 2     |
| RCDEVICE_PROTOCOL_CAM_CTRL_START_RECORDING       | 3     |
| RCDEVICE_PROTOCOL_CAM_CTRL_STOP_RECORDING        | 4     |
| RCDEVICE_PROTOCOL_CAM_CTRL_UNKNOWN_CAMERA_OPERATION | 255   |

---

## RCDevice 5-Key Simulation Operation (`rcdevice_5key_simulation_operation`)

Operations simulating a 5-key OSD cable via RCDevice protocol.

| Name                                           | Value |
|------------------------------------------------|-------|
| RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE         | 0     |
| RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET          | 1     |
| RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT         | 2     |
| RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT        | 3     |
| RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP           | 4     |
| RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN         | 5     |

---

## RCDevice 5-Key Connection Event (`RCDEVICE_5key_connectionvent`)

Events related to the simulated 5-key OSD cable connection.

| Name                                     | Value |
|------------------------------------------|-------|
| RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN   | 1     |
| RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE  | 2     |

---

## RCDevice Camera Simulation Key Event (`rcdeviceCamSimulationKeyEvent`)

Key press events simulated for camera control via RCDevice.

| Name                           | Value |
|--------------------------------|-------|
| RCDEVICE_CAM_KEY_NONE          | 0     |
| RCDEVICE_CAM_KEY_ENTER         | 1     |
| RCDEVICE_CAM_KEY_LEFT          | 2     |
| RCDEVICE_CAM_KEY_UP            | 3     |
| RCDEVICE_CAM_KEY_RIGHT         | 4     |
| RCDEVICE_CAM_KEY_DOWN          | 5     |
| RCDEVICE_CAM_KEY_CONNECTION_CLOSE| 6     |
| RCDEVICE_CAM_KEY_CONNECTION_OPEN | 7     |
| RCDEVICE_CAM_KEY_RELEASE       | 8     |

---

## RCDevice Protocol Version (`rcdevice_protocol_version`)

Versions of the RCDevice protocol.

| Name                            | Value |
|---------------------------------|-------|
| RCDEVICE_PROTOCOL_RCSPLIT_VERSION | 0     |
| RCDEVICE_PROTOCOL_VERSION_1_0   | 1     |
| RCDEVICE_PROTOCOL_UNKNOWN       | 2     |

---

## RCDevice Response Status (`rcdeviceResponseStatus`)

Status codes for responses in the RCDevice protocol.

| Name                        | Value |
|-----------------------------|-------|
| RCDEVICE_RESP_SUCCESS       | 0     |
| RCDEVICE_RESP_INCORRECT_CRC | 1     |
| RCDEVICE_RESP_TIMEOUT       | 2     |

---

## Beeper Mode (`beeperMode`)

Conditions that trigger the onboard beeper.

| Name                                | Value |
|-------------------------------------|-------|
| BEEPER_SILENCE                      | 0     |
| BEEPER_RUNTIME_CALIBRATION_DONE     | 1     |
| BEEPER_HARDWARE_FAILURE             | 2     |
| BEEPER_RX_LOST                      | 3     |
| BEEPER_RX_LOST_LANDING              | 4     |
| BEEPER_DISARMING                    | 5     |
| BEEPER_ARMING                       | 6     |
| BEEPER_ARMING_GPS_FIX               | 7     |
| BEEPER_BAT_CRIT_LOW                 | 8     |
| BEEPER_BAT_LOW                      | 9     |
| BEEPER_GPS_STATUS                   | 10    |
| BEEPER_RX_SET                       | 11    |
| BEEPER_ACTION_SUCCESS               | 12    |
| BEEPER_ACTION_FAIL                  | 13    |
| BEEPER_READY_BEEP                   | 14    |
| BEEPER_MULTI_BEEPS                  | 15    |
| BEEPER_DISARM_REPEAT                | 16    |
| BEEPER_ARMED                        | 17    |
| BEEPER_SYSTEM_INIT                  | 18    |
| BEEPER_USB                          | 19    |
| BEEPER_LAUNCH_MODE_ENABLED          | 20    |
| BEEPER_LAUNCH_MODE_LOW_THROTTLE     | 21    |
| BEEPER_LAUNCH_MODE_IDLE_START       | 22    |
| BEEPER_CAM_CONNECTION_OPEN          | 23    |
| BEEPER_CAM_CONNECTION_CLOSE         | 24    |
| BEEPER_ALL                          | 25    |
| BEEPER_PREFERENCE                   | 26    |

---

## U-Blox Signal Health (`ublox_nav_sig_health`)

Health status reported for individual GPS satellite signals (U-Blox specific).

| Name                         | Value |
|------------------------------|-------|
| UBLOX_SIG_HEALTH_UNKNOWN     | 0     |
| UBLOX_SIG_HEALTH_HEALTHY     | 1     |
| UBLOX_SIG_HEALTH_UNHEALTHY   | 2     |

---

## U-Blox Signal Quality (`ublox_nav_sig_quality`)

Quality indicator reported for individual GPS satellite signals (U-Blox specific).

| Name                                         | Value |
|----------------------------------------------|-------|
| UBLOX_SIG_QUALITY_NOSIGNAL                   | 0     |
| UBLOX_SIG_QUALITY_SEARCHING                  | 1     |
| UBLOX_SIG_QUALITY_ACQUIRED                   | 2     |
| UBLOX_SIG_QUALITY_UNUSABLE                   | 3     |
| UBLOX_SIG_QUALITY_CODE_LOCK_TIME_SYNC        | 4     |
| UBLOX_SIG_QUALITY_CODE_CARRIER_LOCK_TIME_SYNC| 5     |
| UBLOX_SIG_QUALITY_CODE_CARRIER_LOCK_TIME_SYNC2| 6     |
| UBLOX_SIG_QUALITY_CODE_CARRIER_LOCK_TIME_SYNC3| 7     |

---

## U-Blox ACK State (`ubx_ack_state`)

State tracking for U-Blox command acknowledgements.

| Name              | Value |
|-------------------|-------|
| UBX_ACK_WAITING   | 0     |
| UBX_ACK_GOT_ACK   | 1     |
| UBX_ACK_GOT_NAK   | 2     |

---

## U-Blox Protocol Bytes (`ubx_protocol_bytes`)

Constants defining structure and IDs within the U-Blox UBX protocol.

| Name                 | Value | Description                       |
|----------------------|-------|-----------------------------------|
| PREAMBLE1            | 181   | UBX Sync Char 1 (0xB5)            |
| PREAMBLE2            | 98    | UBX Sync Char 2 (0x62)            |
| CLASS_NAV            | 1     | Navigation Results Class          |
| CLASS_ACK            | 5     | Acknowledgement Class             |
| CLASS_CFG            | 6     | Configuration Class               |
| CLASS_MON            | 10    | Monitoring Class                  |
| MSG_CLASS_UBX        | 1     | UBX Protocol                      |
| MSG_CLASS_NMEA       | 240   | NMEA Protocol                     |
| MSG_VER              | 4     | MON-VER Message ID                |
| MSG_ACK_NACK         | 0     | ACK-NAK Message ID                |
| MSG_ACK_ACK          | 1     | ACK-ACK Message ID                |
| MSG_NMEA_GGA         | 0     | NMEA GGA Message ID               |
| MSG_NMEA_GLL         | 1     | NMEA GLL Message ID               |
| MSG_NMEA_GSA         | 2     | NMEA GSA Message ID               |
| MSG_NMEA_GSV         | 3     | NMEA GSV Message ID               |
| MSG_NMEA_RMC         | 4     | NMEA RMC Message ID               |
| MSG_NMEA_VGS         | 5     | NMEA VTG Message ID (typo in code?) |
| MSG_POSLLH           | 2     | NAV-POSLLH Message ID             |
| MSG_STATUS           | 3     | NAV-STATUS Message ID             |
| MSG_SOL              | 6     | NAV-SOL Message ID                |
| MSG_PVT              | 7     | NAV-PVT Message ID                |
| MSG_VELNED           | 18    | NAV-VELNED Message ID             |
| MSG_TIMEUTC          | 33    | NAV-TIMEUTC Message ID            |
| MSG_SVINFO           | 48    | NAV-SVINFO Message ID             |
| MSG_NAV_SAT          | 53    | NAV-SAT Message ID                |
| MSG_CFG_PRT          | 0     | CFG-PRT Message ID                |
| MSG_CFG_RATE         | 8     | CFG-RATE Message ID               |
| MSG_CFG_SET_RATE     | 1     | Likely CFG-MSG Message ID         |
| MSG_CFG_NAV_SETTINGS | 36    | CFG-NAV5/NAVX5 Message ID         |
| MSG_CFG_SBAS         | 22    | CFG-SBAS Message ID               |
| MSG_CFG_GNSS         | 62    | CFG-GNSS Message ID               |
| MSG_MON_GNSS         | 40    | MON-GNSS Message ID               |
| MSG_NAV_SIG          | 67    | NAV-SIG Message ID                |

---

## U-Blox Navigation Fix Type (`ubs_nav_fixype`)

GPS fix type reported by U-Blox NAV messages.

| Name                   | Value |
|------------------------|-------|
| FIX_NONE               | 0     |
| FIX_DEAD_RECKONING     | 1     |
| FIX_2D                 | 2     |
| FIX_3D                 | 3     |
| FIX_GPS_DEAD_RECKONING | 4     |
| FIX_TIME               | 5     |

---

## U-Blox Navigation Status Bits (`ubx_nav_status_bits`)

Flags within the U-Blox NAV-STATUS message.

| Name                 | Value |
|----------------------|-------|
| NAV_STATUS_FIX_VALID | 1     |

---

## Dashboard Page ID (`pageId`)

Internal identifiers for OSD dashboard pages.

| Name         | Value |
|--------------|-------|
| PAGE_WELCOME | 0     |
| PAGE_ARMED   | 1     |
| PAGE_STATUS  | 2     |

---

## OSD Speed Source (`osdSpeedSource`)

Source selection for the speed value displayed on the OSD.

| Name                      | Value |
|---------------------------|-------|
| OSD_SPEED_SOURCE_GROUND   | 0     |
| OSD_SPEED_SOURCE_3D       | 1     |
| OSD_SPEED_SOURCE_AIR      | 2     |

---

## OSD Draw Point Type (`osdDrawPointType`)

Coordinate system used for drawing custom points on the OSD.

| Name                       | Value |
|----------------------------|-------|
| OSD_DRAW_POINT_TYPE_GRID   | 0     | *Character Grid* |
| OSD_DRAW_POINT_TYPE_PIXEL  | 1     | *Pixel Coordinates* |

---

## LED Strip Color ID (`colorId`)

Predefined color indices for LED strip configuration.

| Name             | Value |
|------------------|-------|
| COLOR_BLACK      | 0     |
| COLOR_WHITE      | 1     |
| COLOR_RED        | 2     |
| COLOR_ORANGE     | 3     |
| COLOR_YELLOW     | 4     |
| COLOR_LIME_GREEN | 5     |
| COLOR_GREEN      | 6     |
| COLOR_MINT_GREEN | 7     |
| COLOR_CYAN       | 8     |
| COLOR_LIGHT_BLUE | 9     |
| COLOR_BLUE       | 10    |
| COLOR_DARK_VIOLET| 11    |
| COLOR_MAGENTA    | 12    |
| COLOR_DEEP_PINK  | 13    |

---

## LED Mode Index (`ledModeIndex`)

Indices representing different modes for LED strip color assignments.

| Name               | Value |
|--------------------|-------|
| LED_MODE_ORIENTATION| 0     |
| LED_MODE_HEADFREE  | 1     |
| LED_MODE_HORIZON   | 2     |
| LED_MODE_ANGLE     | 3     |
| LED_MODE_MAG       | 4     |
| LED_MODE_BARO      | 5     |
| LED_SPECIAL        | 6     |

---

## LED Special Color IDs (`ledSpecialColorIds`)

Indices for special conditions mapped to LED colors.

| Name                       | Value |
|----------------------------|-------|
| LED_SCOLOR_DISARMED        | 0     |
| LED_SCOLOR_ARMED           | 1     |
| LED_SCOLOR_ANIMATION       | 2     |
| LED_SCOLOR_BACKGROUND      | 3     |
| LED_SCOLOR_BLINKBACKGROUND | 4     |
| LED_SCOLOR_GPSNOSATS       | 5     |
| LED_SCOLOR_GPSNOLOCK       | 6     |
| LED_SCOLOR_GPSLOCKED       | 7     |
| LED_SCOLOR_STROBE          | 8     |

---

## LED Direction ID (`ledDirectionId`)

Indices representing physical directions for LED color assignments.

| Name               | Value |
|--------------------|-------|
| LED_DIRECTION_NORTH| 0     |
| LED_DIRECTION_EAST | 1     |
| LED_DIRECTION_SOUTH| 2     |
| LED_DIRECTION_WEST | 3     |
| LED_DIRECTION_UP   | 4     |
| LED_DIRECTION_DOWN | 5     |

---

## LED Base Function ID (`ledBaseFunctionId`)

Base function assigned to an individual LED in the strip configuration.

| Name                   | Value |
|------------------------|-------|
| LED_FUNCTION_COLOR     | 0     |
| LED_FUNCTION_FLIGHT_MODE| 1     |
| LED_FUNCTION_ARM_STATE | 2     |
| LED_FUNCTION_BATTERY   | 3     |
| LED_FUNCTION_RSSI      | 4     |
| LED_FUNCTION_GPS       | 5     |
| LED_FUNCTION_THRUST_RING| 6     |
| LED_FUNCTION_CHANNEL   | 7     |

---

## LED Overlay ID (`ledOverlayId`)

Overlay effect applied to an individual LED, modifying its base function.

| Name                    | Value |
|-------------------------|-------|
| LED_OVERLAY_THROTTLE    | 0     |
| LED_OVERLAY_LARSON_SCANNER| 1     |
| LED_OVERLAY_BLINK       | 2     |
| LED_OVERLAY_LANDING_FLASH| 3     |
| LED_OVERLAY_INDICATOR   | 4     |
| LED_OVERLAY_WARNING     | 5     |
| LED_OVERLAY_STROBE      | 6     |

---

## FrSky OSD Transaction Options (`frskyOSDTransactionOptions`)

Options for FrSky OSD protocol transactions.

| Name                              | Value     |
|-----------------------------------|-----------|
| FRSKY_OSD_TRANSACTION_OPT_PROFILED | `1 << 0`  |
| FRSKY_OSD_TRANSACTION_OPT_RESET_DRAWING | `1 << 1`  |

---

## FrSky OSD Color (`frskyOSDColor`)

Color options for the FrSky OSD protocol.

| Name                      | Value |
|---------------------------|-------|
| FRSKY_OSD_COLOR_BLACK     | 0     |
| FRSKY_OSD_COLOR_TRANSPARENT| 1     |
| FRSKY_OSD_COLOR_WHITE     | 2     |
| FRSKY_OSD_COLOR_GRAY      | 3     |

---

## FrSky OSD Line Outline Type (`frskyOSDLineOutlineType`)

Outline options for lines drawn via the FrSky OSD protocol.

| Name                         | Value     |
|------------------------------|-----------|
| FRSKY_OSD_OUTLINE_TYPE_NONE  | 0         |
| FRSKY_OSD_OUTLINE_TYPE_TOP   | `1 << 0`  |
| FRSKY_OSD_OUTLINE_TYPE_RIGHT | `1 << 1`  |
| FRSKY_OSD_OUTLINE_TYPE_BOTTOM| `1 << 2`  |
| FRSKY_OSD_OUTLINE_TYPE_LEFT  | `1 << 3`  |

---

## OSD Items (`osd_items`)

Identifiers for all displayable items on the On-Screen Display.

*(Note: This table is very long, only showing the start and end)*

| Name                          | Value |
|-------------------------------|-------|
| OSD_RSSI_VALUE                | 0     |
| OSD_MAIN_BATT_VOLTAGE         | 1     |
| OSD_CROSSHAIRS                | 2     |
| ...                           | ...   |
| OSD_RX_BAND                   | 161   |
| OSD_RX_MODE                   | 162   |
| OSD_COURSE_TO_FENCE           | 163   |
| OSD_H_DIST_TO_FENCE           | 164   |
| OSD_V_DIST_TO_FENCE           | 165   |
| OSD_NAV_FW_ALT_CONTROL_RESPONSE | 166   |
| OSD_ITEM_COUNT                | 167   |

---

## OSD Units (`osd_unit`)

Measurement unit system selection for OSD display.

| Name                  | Value |
|-----------------------|-------|
| OSD_UNIT_IMPERIAL     | 0     |
| OSD_UNIT_METRIC       | 1     |
| OSD_UNIT_METRIC_MPH   | 2     |
| OSD_UNIT_UK           | 3     |
| OSD_UNIT_GA           | 4     |
| OSD_UNIT_MAX          | 4     |

---

## OSD Stats Energy Unit (`osd_statsnergy_unit`)

Unit selection for energy display in post-flight OSD statistics.

| Name                       | Value |
|----------------------------|-------|
| OSD_STATS_ENERGY_UNIT_MAH  | 0     |
| OSD_STATS_ENERGY_UNIT_WH   | 1     |

---

## OSD Crosshairs Style (`osd_crosshairs_style`)

Style selection for the center OSD crosshairs.

| Name                         | Value |
|------------------------------|-------|
| OSD_CROSSHAIRS_STYLE_DEFAULT | 0     |
| OSD_CROSSHAIRS_STYLE_AIRCRAFT| 1     |
| OSD_CROSSHAIRS_STYLE_TYPE3   | 2     |
| OSD_CROSSHAIRS_STYLE_TYPE4   | 3     |
| OSD_CROSSHAIRS_STYLE_TYPE5   | 4     |
| OSD_CROSSHAIRS_STYLE_TYPE6   | 5     |
| OSD_CROSSHAIRS_STYLE_TYPE7   | 6     |

---

## OSD Sidebar Scroll (`osd_sidebar_scroll`)

Configuration for the OSD sidebar scrolling behavior.

| Name                             | Value |
|----------------------------------|-------|
| OSD_SIDEBAR_SCROLL_NONE          | 0     |
| OSD_SIDEBAR_SCROLL_ALTITUDE      | 1     |
| OSD_SIDEBAR_SCROLL_SPEED         | 2     |
| OSD_SIDEBAR_SCROLL_HOME_DISTANCE | 3     |
| OSD_SIDEBAR_SCROLL_MAX           | 3     |

---

## OSD Alignment (`osd_alignment`)

Text alignment options for OSD elements (likely internal/unused).

| Name            | Value |
|-----------------|-------|
| OSD_ALIGN_LEFT  | 0     |
| OSD_ALIGN_RIGHT | 1     |

---

## OSD AHI Style (`osd_ahi_style`)

Style selection for the Artificial Horizon Indicator (AHI).

| Name                   | Value |
|------------------------|-------|
| OSD_AHI_STYLE_DEFAULT  | 0     |
| OSD_AHI_STYLE_LINE     | 1     |

---

## OSD CRSF LQ Format (`osd_crsf_lq_format`)

Format selection for displaying Crossfire Link Quality (LQ) on the OSD.

| Name                 | Value |
|----------------------|-------|
| OSD_CRSF_LQ_TYPE1    | 0     |
| OSD_CRSF_LQ_TYPE2    | 1     |
| OSD_CRSF_LQ_TYPE3    | 2     |

---

## VTX SmartAudio Version (`smartAudioVersion`)

Detected version of the TBS SmartAudio protocol.

| Name     | Value |
|----------|-------|
| SA_UNKNOWN | 0     |
| SA_1_0   | 1     |
| SA_2_0   | 2     |
| SA_2_1   | 3     |

---

## GPS State (`gpsState`)

Internal state of the GPS driver.

| Name                   | Value |
|------------------------|-------|
| GPS_UNKNOWN            | 0     |
| GPS_INITIALIZING       | 1     |
| GPS_RUNNING            | 2     |
| GPS_LOST_COMMUNICATION | 3     |

---

## SmartPort Master VS600 Band (`vs600Band`)

Band selection for the VS600 VTX via SmartPort Master.

| Name         | Value |
|--------------|-------|
| VS600_BAND_A | 0     |
| VS600_BAND_B | 1     |
| VS600_BAND_C | 2     |
| VS600_BAND_D | 3     |
| VS600_BAND_E | 4     |
| VS600_BAND_F | 5     |

---

## SmartPort Master VS600 Power (`vs600Power`)

Power level selection for the VS600 VTX via SmartPort Master.

| Name           | Value |
|----------------|-------|
| VS600_POWER_PIT| 0     |
| VS600_POWER_25MW| 1     |
| VS600_POWER_200MW| 2     |
| VS600_POWER_600MW| 3     |

---

## Servo Input Source (`inputSource`)

Sources available for driving servo outputs via the mixer.

| Name                          | Value |
|-------------------------------|-------|
| INPUT_STABILIZED_ROLL         | 0     |
| INPUT_STABILIZED_PITCH        | 1     |
| INPUT_STABILIZED_YAW          | 2     |
| INPUT_STABILIZED_THROTTLE     | 3     |
| INPUT_RC_ROLL                 | 4     |
| INPUT_RC_PITCH                | 5     |
| INPUT_RC_YAW                  | 6     |
| INPUT_RC_THROTTLE             | 7     |
| INPUT_RC_CH5                  | 8     |
| INPUT_RC_CH6                  | 9     |
| INPUT_RC_CH7                  | 10    |
| INPUT_RC_CH8                  | 11    |
| INPUT_GIMBAL_PITCH            | 12    |
| INPUT_GIMBAL_ROLL             | 13    |
| INPUT_FEATURE_FLAPS           | 14    |
| INPUT_RC_CH9                  | 15    |
| INPUT_RC_CH10                 | 16    |
| INPUT_RC_CH11                 | 17    |
| INPUT_RC_CH12                 | 18    |
| INPUT_RC_CH13                 | 19    |
| INPUT_RC_CH14                 | 20    |
| INPUT_RC_CH15                 | 21    |
| INPUT_RC_CH16                 | 22    |
| INPUT_STABILIZED_ROLL_PLUS    | 23    |
| INPUT_STABILIZED_ROLL_MINUS   | 24    |
| INPUT_STABILIZED_PITCH_PLUS   | 25    |
| INPUT_STABILIZED_PITCH_MINUS  | 26    |
| INPUT_STABILIZED_YAW_PLUS     | 27    |
| INPUT_STABILIZED_YAW_MINUS    | 28    |
| INPUT_MAX                     | 29    |
| INPUT_GVAR_0                  | 30    |
| INPUT_GVAR_1                  | 31    |
| INPUT_GVAR_2                  | 32    |
| INPUT_GVAR_3                  | 33    |
| INPUT_GVAR_4                  | 34    |
| INPUT_GVAR_5                  | 35    |
| INPUT_GVAR_6                  | 36    |
| INPUT_GVAR_7                  | 37    |
| INPUT_MIXER_TRANSITION        | 38    |
| INPUT_HEADTRACKER_PAN         | 39    |
| INPUT_HEADTRACKER_TILT        | 40    |
| INPUT_HEADTRACKER_ROLL        | 41    |
| INPUT_RC_CH17                 | 42    |
| INPUT_RC_CH18                 | 43    |
| INPUT_RC_CH19                 | 44    |
| INPUT_RC_CH20                 | 45    |
| INPUT_RC_CH21                 | 46    |
| INPUT_RC_CH22                 | 47    |
| INPUT_RC_CH23                 | 48    |
| INPUT_RC_CH24                 | 49    |
| INPUT_RC_CH25                 | 50    |
| INPUT_RC_CH26                 | 51    |
| INPUT_RC_CH27                 | 52    |
| INPUT_RC_CH28                 | 53    |
| INPUT_RC_CH29                 | 54    |
| INPUT_RC_CH30                 | 55    |
| INPUT_RC_CH31                 | 56    |
| INPUT_RC_CH32                 | 57    |
| INPUT_RC_CH33                 | 58    |
| INPUT_RC_CH34                 | 59    |
| INPUT_SOURCE_COUNT            | 60    |

---

## Servo Index (`servoIndex`)

Predefined indices for commonly used servo functions in mixer presets.

| Name                  | Value |
|-----------------------|-------|
| SERVO_GIMBAL_PITCH    | 0     |
| SERVO_GIMBAL_ROLL     | 1     |
| SERVO_ELEVATOR        | 2     |
| SERVO_FLAPPERON_1     | 3     |
| SERVO_FLAPPERON_2     | 4     |
| SERVO_RUDDER          | 5     |
| SERVO_BICOPTER_LEFT   | 4     |
| SERVO_BICOPTER_RIGHT  | 5     |
| SERVO_DUALCOPTER_LEFT | 4     |
| SERVO_DUALCOPTER_RIGHT| 5     |
| SERVO_SINGLECOPTER_1  | 3     |
| SERVO_SINGLECOPTER_2  | 4     |
| SERVO_SINGLECOPTER_3  | 5     |
| SERVO_SINGLECOPTER_4  | 6     |

---

## Mixer Profile Autotune Request (`mixerProfileATRequest`)

Requests related to Mixer Autotune (MixerAT) state transitions.

| Name                    | Value |
|-------------------------|-------|
| MIXERAT_REQUEST_NONE    | 0     |
| MIXERAT_REQUEST_RTH     | 1     |
| MIXERAT_REQUEST_LAND    | 2     |
| MIXERAT_REQUEST_ABORT   | 3     |

---

## Mixer Profile Autotune State (`mixerProfileATState`)

Internal states for the Mixer Autotune (MixerAT) process.

| Name                                  | Value |
|---------------------------------------|-------|
| MIXERAT_PHASE_IDLE                    | 0     |
| MIXERAT_PHASE_TRANSITION_INITIALIZE   | 1     |
| MIXERAT_PHASE_TRANSITIONING           | 2     |
| MIXERAT_PHASE_DONE                    | 3     |

---

## PID Controller Index (`pidIndex`)

Indices identifying the standard PID controllers.

| Name            | Value |
|-----------------|-------|
| PID_ROLL        | 0     |
| PID_PITCH       | 1     |
| PID_YAW         | 2     |
| PID_POS_Z       | 3     | *Altitude* |
| PID_POS_XY      | 4     | *Position Rate* |
| PID_VEL_XY      | 5     | *Position Hold* |
| PID_SURFACE     | 6     | *Surface Tracking* |
| PID_LEVEL       | 7     | *Angle Level* |
| PID_HEADING     | 8     | *Heading Hold* |
| PID_VEL_Z       | 9     | *Velocity Z* |
| PID_POS_HEADING | 10    | *Waypoint Heading* |
| PID_ITEM_COUNT  | 11    |

---

## PID Controller Type (`pidType`)

Type of PID controller implementation used (relevant for future extensions).

| Name         | Value |
|--------------|-------|
| PID_TYPE_NONE| 0     |
| PID_TYPE_PID | 1     |
| PID_TYPE_PIFF| 2     |
| PID_TYPE_AUTO| 3     |

---

## I-Term Relax (`itermRelax`)

Mode for relaxing the I-term accumulation based on stick input or setpoint changes.

| Name           | Value |
|----------------|-------|
| ITERM_RELAX_OFF| 0     |
| ITERM_RELAX_RP | 1     |
| ITERM_RELAX_RPY| 2     |

---

## Fixed Wing Autotune Rate Adjustment (`fw_autotune_rate_adjustment`)

Mode for adjusting rates during fixed wing autotune.

| Name    | Value |
|---------|-------|
| FIXED   | 0     |
| LIMIT   | 1     |
| AUTO    | 2     |

---

## Mixer Platform Type (`flyingPlatformType`)

Duplicate of `platformType` used specifically in mixer code.

| Name                | Value |
|---------------------|-------|
| PLATFORM_MULTIROTOR | 0     |
| PLATFORM_AIRPLANE   | 1     |
| PLATFORM_HELICOPTER | 2     |
| PLATFORM_TRICOPTER  | 3     |
| PLATFORM_ROVER      | 4     |
| PLATFORM_BOAT       | 5     |

---

## Output Mode (`outputMode`)

Mode configuration for timer outputs (Motor, Servo, LED).

| Name             | Value |
|------------------|-------|
| OUTPUT_MODE_AUTO | 0     |
| OUTPUT_MODE_MOTORS| 1     |
| OUTPUT_MODE_SERVOS| 2     |
| OUTPUT_MODE_LED  | 3     |

---

## Motor Status (`motorStatus`)

Runtime status of individual motors.

| Name               | Value |
|--------------------|-------|
| MOTOR_STOPPED_USER | 0     |
| MOTOR_STOPPED_AUTO | 1     |
| MOTOR_RUNNING      | 2     |

---

## Reversible Motors Throttle State (`reversibleMotorsThrottleState`)

State indication for 3D/reversible motor operation based on throttle input.

| Name                     | Value |
|--------------------------|-------|
| MOTOR_DIRECTION_FORWARD  | 0     |
| MOTOR_DIRECTION_BACKWARD | 1     |
| MOTOR_DIRECTION_DEADBAND | 2     |

---

## Failsafe Phase (`failsafePhase`)

Internal states of the failsafe handling system.

| Name                      | Value |
|---------------------------|-------|
| FAILSAFE_IDLE             | 0     |
| FAILSAFE_RX_LOSS_DETECTED | 1     |
| FAILSAFE_RX_LOSS_IDLE     | 2     |
| FAILSAFE_RETURN_TO_HOME   | 3     |
| FAILSAFE_LANDING          | 4     |
| FAILSAFE_LANDED           | 5     |
| FAILSAFE_RX_LOSS_MONITORING| 6     |
| FAILSAFE_RX_LOSS_RECOVERED| 7     |

---

## Failsafe RX Link State (`failsafeRxLinkState`)

Indicates if the primary RC link is considered up or down.

| Name                 | Value |
|----------------------|-------|
| FAILSAFE_RXLINK_DOWN | 0     |
| FAILSAFE_RXLINK_UP   | 1     |

---

## Failsafe Procedure (`failsafeProcedure`)

Configured action to take upon entering failsafe stage 2.

| Name                            | Value |
|---------------------------------|-------|
| FAILSAFE_PROCEDURE_AUTO_LANDING | 0     |
| FAILSAFE_PROCEDURE_DROP_IT      | 1     |
| FAILSAFE_PROCEDURE_RTH          | 2     |
| FAILSAFE_PROCEDURE_NONE         | 3     |

---

## RTH State (`rthState`)

Simplified state tracking for RTH failsafe procedure.

| Name             | Value |
|------------------|-------|
| RTH_IDLE         | 0     |
| RTH_IN_PROGRESS  | 1     |
| RTH_HAS_LANDED   | 2     |

---

## Emergency Land State (`emergLandState`)

Simplified state tracking for emergency landing failsafe procedure.

| Name                   | Value |
|------------------------|-------|
| EMERG_LAND_IDLE        | 0     |
| EMERG_LAND_IN_PROGRESS | 1     |
| EMERG_LAND_HAS_LANDED  | 2     |

---

## Blackbox Feature Mask (`blackboxFeatureMask`)

Flags to include/exclude specific data fields from Blackbox logs.

| Name                         | Value      |
|------------------------------|------------|
| BLACKBOX_FEATURE_NAV_ACC     | `1 << 0`   |
| BLACKBOX_FEATURE_NAV_POS     | `1 << 1`   |
| BLACKBOX_FEATURE_NAV_PID     | `1 << 2`   |
| BLACKBOX_FEATURE_MAG         | `1 << 3`   |
| BLACKBOX_FEATURE_ACC         | `1 << 4`   |
| BLACKBOX_FEATURE_ATTITUDE    | `1 << 5`   |
| BLACKBOX_FEATURE_RC_DATA     | `1 << 6`   |
| BLACKBOX_FEATURE_RC_COMMAND  | `1 << 7`   |
| BLACKBOX_FEATURE_MOTORS      | `1 << 8`   |
| BLACKBOX_FEATURE_GYRO_RAW    | `1 << 9`   |
| BLACKBOX_FEATURE_GYRO_PEAKS_ROLL | `1 << 10`  |
| BLACKBOX_FEATURE_GYRO_PEAKS_PITCH| `1 << 11`  |
| BLACKBOX_FEATURE_GYRO_PEAKS_YAW | `1 << 12`  |
| BLACKBOX_FEATURE_SERVOS      | `1 << 13`  |

---

## Blackbox Buffer Reserve Status (`blackboxBufferReserveStatus`)

Status codes indicating success or failure when reserving buffer space for Blackbox logging.

| Name                              | Value |
|-----------------------------------|-------|
| BLACKBOX_RESERVE_SUCCESS          | 0     |
| BLACKBOX_RESERVE_TEMPORARY_FAILURE| 1     |
| BLACKBOX_RESERVE_PERMANENT_FAILURE| 2     |
