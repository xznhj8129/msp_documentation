# MSP Payload Structures

_Auto-generated payload layout reference derived from MSP encode/decode logic in `src/main/fc/fc_msp.c`. For each command the fields are listed in the order they are sent. Field types default to the stream width, but explicit casts in the source expressions override the type. Comments reference the expression that provides or consumes the value so you can locate the true destination symbol._

## MSP2_ADSB_VEHICLE_LIST
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t max_adsb_vehicles; // MAX_ADSB_VEHICLES
    uint8_t adsb_call_sign_max_length; // ADSB_CALL_SIGN_MAX_LENGTH
    uint32_t getadsbstatus___vehiclesmessagestotal; // getAdsbStatus()->vehiclesMessagesTotal
    uint32_t getadsbstatus___heartbeatmessagestotal; // getAdsbStatus()->heartbeatMessagesTotal
    uint8_t adsbvehicle_vehiclevalues_callsign_ii; // adsbVehicle->vehicleValues.callsign[ii]
    uint32_t adsbvehicle_vehiclevalues_icao; // adsbVehicle->vehicleValues.icao
    uint32_t adsbvehicle_vehiclevalues_lat; // adsbVehicle->vehicleValues.lat
    uint32_t adsbvehicle_vehiclevalues_lon; // adsbVehicle->vehicleValues.lon
    uint32_t adsbvehicle_vehiclevalues_alt; // adsbVehicle->vehicleValues.alt
    uint16_t uint16_t_centidegrees_to_degrees_adsbvehicle_vehiclevalues_heading; // (uint16_t)CENTIDEGREES_TO_DEGREES(adsbVehicle->vehicleValues.heading)
    uint8_t adsbvehicle_vehiclevalues_tslc; // adsbVehicle->vehicleValues.tslc
    uint8_t adsbvehicle_vehiclevalues_emittertype; // adsbVehicle->vehicleValues.emitterType
    uint8_t adsbvehicle_ttl; // adsbVehicle->ttl
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint32_t f_0; // 0
    uint32_t f_0; // 0
} msp_msp2_adsb_vehicle_list_reply_t;
```

## MSP2_BETAFLIGHT_BIND
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t ptr_size; // ptr, size
    uint8_t name_buf_strlen_name_buf__1; // name_buf, strlen(name_buf) + 1
    uint16_t settinggetpgn_setting; // settingGetPgn(setting)
    uint8_t setting_type_setting; // SETTING_TYPE(setting)
    uint8_t setting_section_setting; // SETTING_SECTION(setting)
    uint8_t setting_mode_setting; // SETTING_MODE(setting)
    uint32_t uint32_t_min; // (uint32_t)min
    uint32_t max; // max
    uint16_t settinggetindex_setting; // settingGetIndex(setting)
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t getconfigprofile; // getConfigProfile()
    uint8_t max_profile_count; // MAX_PROFILE_COUNT
    uint8_t getconfigbatteryprofile; // getConfigBatteryProfile()
    uint8_t max_battery_profile_count; // MAX_BATTERY_PROFILE_COUNT
    uint8_t getconfigmixerprofile; // getConfigMixerProfile()
    uint8_t max_mixer_profile_count; // MAX_MIXER_PROFILE_COUNT
    uint8_t name_strlen_name__1; // name, strlen(name) + 1
    uint8_t ptr_size; // ptr, size
    uint16_t ii; // ii
    uint16_t start; // start
    uint16_t end; // end
    uint8_t f_255; // 255
    uint8_t osddisplayport_rows; // osdDisplayPort->rows
    uint8_t osddisplayport_cols; // osdDisplayPort->cols
    uint8_t osdpos_y; // osdPos_y
    uint8_t osdpos_x; // osdPos_x
    uint8_t f_255; // 255
    uint8_t lastchar___0xff; // lastChar & 0xff
    uint8_t f_0; // 0
    uint8_t cmd; // cmd
    uint8_t lastcharlow; // lastCharLow
    uint8_t lastcharlow; // lastCharLow
    uint8_t lastcharlow; // lastCharLow
    uint8_t lastcharlow; // lastCharLow
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
} msp_msp2_betaflight_bind_reply_t;
```

## MSP2_BLACKBOX_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t f_1; // 1
    uint8_t blackboxconfig___device; // blackboxConfig()->device
    uint16_t blackboxconfig___rate_num; // blackboxConfig()->rate_num
    uint16_t blackboxconfig___rate_denom; // blackboxConfig()->rate_denom
    uint32_t blackboxconfig___includeflags; // blackboxConfig()->includeFlags
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
} msp_msp2_blackbox_config_reply_t;
```

## MSP2_COMMON_GET_RADAR_GPS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t radar_pois_i__gps_sizeof_gpslocation_t; // &radar_pois[i].gps, sizeof(gpsLocation_t)
    uint8_t safe_home_no; // safe_home_no
    uint8_t safehomeconfig_safe_home_no__enabled; // safeHomeConfig(safe_home_no)->enabled
    uint32_t safehomeconfig_safe_home_no__lat; // safeHomeConfig(safe_home_no)->lat
    uint32_t safehomeconfig_safe_home_no__lon; // safeHomeConfig(safe_home_no)->lon
    uint8_t idx; // idx
    uint32_t fwautolandapproachconfig_idx__approachalt; // fwAutolandApproachConfig(idx)->approachAlt
    uint32_t fwautolandapproachconfig_idx__landalt; // fwAutolandApproachConfig(idx)->landAlt
    uint8_t fwautolandapproachconfig_idx__approachdirection; // fwAutolandApproachConfig(idx)->approachDirection
    uint16_t fwautolandapproachconfig_idx__landapproachheading1; // fwAutolandApproachConfig(idx)->landApproachHeading1
    uint16_t fwautolandapproachconfig_idx__landapproachheading2; // fwAutolandApproachConfig(idx)->landApproachHeading2
    uint8_t fwautolandapproachconfig_idx__issealevelref; // fwAutolandApproachConfig(idx)->isSeaLevelRef
    uint8_t idx; // idx
    uint8_t geozonesconfig_idx__type; // geoZonesConfig(idx)->type
    uint8_t geozonesconfig_idx__shape; // geoZonesConfig(idx)->shape
    uint32_t geozonesconfig_idx__minaltitude; // geoZonesConfig(idx)->minAltitude
    uint32_t geozonesconfig_idx__maxaltitude; // geoZonesConfig(idx)->maxAltitude
    uint8_t geozonesconfig_idx__issealevelref; // geoZonesConfig(idx)->isSealevelRef
    uint8_t geozonesconfig_idx__fenceaction; // geoZonesConfig(idx)->fenceAction
    uint8_t geozonesconfig_idx__vertexcount; // geoZonesConfig(idx)->vertexCount
    uint8_t geozonevertices_vertexidx__zoneid; // geoZoneVertices(vertexIdx)->zoneId
    uint8_t geozonevertices_vertexidx__idx; // geoZoneVertices(vertexIdx)->idx
    uint32_t geozonevertices_vertexidx__lat; // geoZoneVertices(vertexIdx)->lat
    uint32_t geozonevertices_vertexidx__lon; // geoZoneVertices(vertexIdx)->lon
    uint32_t geozonevertices_vertexradiusidx__lat; // geoZoneVertices(vertexRadiusIdx)->lat
    uint8_t logicconditions_idx__enabled; // logicConditions(idx)->enabled
    uint8_t logicconditions_idx__activatorid; // logicConditions(idx)->activatorId
    uint8_t logicconditions_idx__operation; // logicConditions(idx)->operation
    uint8_t logicconditions_idx__operanda_type; // logicConditions(idx)->operandA.type
    uint32_t logicconditions_idx__operanda_value; // logicConditions(idx)->operandA.value
    uint8_t logicconditions_idx__operandb_type; // logicConditions(idx)->operandB.type
    uint32_t logicconditions_idx__operandb_value; // logicConditions(idx)->operandB.value
    uint8_t logicconditions_idx__flags; // logicConditions(idx)->flags
    uint8_t msp_wp_no; // msp_wp_no
    uint8_t msp_wp_action; // msp_wp.action
    uint32_t msp_wp_lat; // msp_wp.lat
    uint32_t msp_wp_lon; // msp_wp.lon
    uint32_t msp_wp_alt; // msp_wp.alt
    uint16_t msp_wp_p1; // msp_wp.p1
    uint16_t msp_wp_p2; // msp_wp.p2
    uint16_t msp_wp_p3; // msp_wp.p3
    uint8_t msp_wp_flag; // msp_wp.flag
} msp_msp2_common_get_radar_gps_reply_t;
```

## MSP2_COMMON_MOTOR_MIXER
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t constrainf_primarymotormixer_i__throttle_2_0f_0_0f_4_0f____1000; // constrainf(primaryMotorMixer(i)->throttle + 2.0f, 0.0f, 4.0f) * 1000
    uint16_t constrainf_primarymotormixer_i__roll_2_0f_0_0f_4_0f____1000; // constrainf(primaryMotorMixer(i)->roll + 2.0f, 0.0f, 4.0f) * 1000
    uint16_t constrainf_primarymotormixer_i__pitch_2_0f_0_0f_4_0f____1000; // constrainf(primaryMotorMixer(i)->pitch + 2.0f, 0.0f, 4.0f) * 1000
    uint16_t constrainf_primarymotormixer_i__yaw_2_0f_0_0f_4_0f____1000; // constrainf(primaryMotorMixer(i)->yaw + 2.0f, 0.0f, 4.0f) * 1000
    uint16_t constrainf_mixermotormixersbyindex_nextmixerprofileindex__i__throttle_2_0f_0_0f_4_0f____1000; // constrainf(mixerMotorMixersByIndex(nextMixerProfileIndex)[i].throttle + 2.0f, 0.0f, 4.0f) * 1000
    uint16_t constrainf_mixermotormixersbyindex_nextmixerprofileindex__i__roll_2_0f_0_0f_4_0f____1000; // constrainf(mixerMotorMixersByIndex(nextMixerProfileIndex)[i].roll + 2.0f, 0.0f, 4.0f) * 1000
    uint16_t constrainf_mixermotormixersbyindex_nextmixerprofileindex__i__pitch_2_0f_0_0f_4_0f____1000; // constrainf(mixerMotorMixersByIndex(nextMixerProfileIndex)[i].pitch + 2.0f, 0.0f, 4.0f) * 1000
    uint16_t constrainf_mixermotormixersbyindex_nextmixerprofileindex__i__yaw_2_0f_0_0f_4_0f____1000; // constrainf(mixerMotorMixersByIndex(nextMixerProfileIndex)[i].yaw + 2.0f, 0.0f, 4.0f) * 1000
} msp_msp2_common_motor_mixer_reply_t;
```

## MSP2_COMMON_SERIAL_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t serialconfig___portconfigs_i__identifier; // serialConfig()->portConfigs[i].identifier
    uint32_t serialconfig___portconfigs_i__functionmask; // serialConfig()->portConfigs[i].functionMask
    uint8_t serialconfig___portconfigs_i__msp_baudrateindex; // serialConfig()->portConfigs[i].msp_baudrateIndex
    uint8_t serialconfig___portconfigs_i__gps_baudrateindex; // serialConfig()->portConfigs[i].gps_baudrateIndex
    uint8_t serialconfig___portconfigs_i__telemetry_baudrateindex; // serialConfig()->portConfigs[i].telemetry_baudrateIndex
    uint8_t serialconfig___portconfigs_i__peripheral_baudrateindex; // serialConfig()->portConfigs[i].peripheral_baudrateIndex
} msp_msp2_common_serial_config_reply_t;
```

## MSP2_COMMON_TZ
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t uint16_t_timeconfig___tz_offset; // (uint16_t)timeConfig()->tz_offset
    uint8_t uint8_t_timeconfig___tz_automatic_dst; // (uint8_t)timeConfig()->tz_automatic_dst
} msp_msp2_common_tz_reply_t;
```

## MSP2_INAV_AIR_SPEED
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t getairspeedestimate; // getAirspeedEstimate()
    uint32_t f_0; // 0
} msp_msp2_inav_air_speed_reply_t;
```

## MSP2_INAV_ANALOG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t batterywasfullwhenpluggedin____batteryusescapacitythresholds___1___getbatterystate___2___getbatterycellcount___4; // batteryWasFullWhenPluggedIn() | (batteryUsesCapacityThresholds() << 1) | (getBatteryState() << 2) | (getBatteryCellCount() << 4)
    uint16_t getbatteryvoltage; // getBatteryVoltage()
    uint16_t getamperage; // getAmperage()
    uint32_t getpower; // getPower()
    uint32_t getmahdrawn; // getMAhDrawn()
    uint32_t getmwhdrawn; // getMWhDrawn()
    uint32_t getbatteryremainingcapacity; // getBatteryRemainingCapacity()
    uint8_t calculatebatterypercentage; // calculateBatteryPercentage()
    uint16_t getrssi; // getRSSI()
} msp_msp2_inav_analog_reply_t;
```

## MSP2_INAV_BATTERY_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t batterymetersconfig___voltage_scale; // batteryMetersConfig()->voltage.scale
    uint8_t batterymetersconfig___voltagesource; // batteryMetersConfig()->voltageSource
    uint8_t currentbatteryprofile_cells; // currentBatteryProfile->cells
    uint16_t currentbatteryprofile_voltage_celldetect; // currentBatteryProfile->voltage.cellDetect
    uint16_t currentbatteryprofile_voltage_cellmin; // currentBatteryProfile->voltage.cellMin
    uint16_t currentbatteryprofile_voltage_cellmax; // currentBatteryProfile->voltage.cellMax
    uint16_t currentbatteryprofile_voltage_cellwarning; // currentBatteryProfile->voltage.cellWarning
    uint16_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t batterymetersconfig___current_offset; // batteryMetersConfig()->current.offset
    uint16_t batterymetersconfig___current_scale; // batteryMetersConfig()->current.scale
    uint32_t currentbatteryprofile_capacity_value; // currentBatteryProfile->capacity.value
    uint32_t currentbatteryprofile_capacity_warning; // currentBatteryProfile->capacity.warning
    uint32_t currentbatteryprofile_capacity_critical; // currentBatteryProfile->capacity.critical
    uint8_t batterymetersconfig___capacity_unit; // batteryMetersConfig()->capacity_unit
} msp_msp2_inav_battery_config_reply_t;
```

## MSP2_INAV_CUSTOM_OSD_ELEMENT
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t customelement_part_ii__type; // customElement->part[ii].type
    uint16_t customelement_part_ii__value; // customElement->part[ii].value
    uint8_t customelement_visibility_type; // customElement->visibility.type
    uint16_t customelement_visibility_value; // customElement->visibility.value
    uint8_t customelement_osdcustomelementtext_ii; // customElement->osdCustomElementText[ii]
} msp_msp2_inav_custom_osd_element_reply_t;
```

## MSP2_INAV_CUSTOM_OSD_ELEMENTS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t max_custom_elements; // MAX_CUSTOM_ELEMENTS
    uint8_t osd_custom_element_text_size_1; // OSD_CUSTOM_ELEMENT_TEXT_SIZE - 1
    uint8_t custom_elements_parts; // CUSTOM_ELEMENTS_PARTS
} msp_msp2_inav_custom_osd_elements_reply_t;
```

## MSP2_INAV_DEBUG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t debug_i; // debug[i]
} msp_msp2_inav_debug_reply_t;
```

## MSP2_INAV_ESC_RPM
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t escstate_rpm; // escState->rpm
} msp_msp2_inav_esc_rpm_reply_t;
```

## MSP2_INAV_ESC_TELEM
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t motorcount; // motorCount
    uint8_t escstate_sizeof_escsensordata_t; // escState, sizeof(escSensorData_t)
} msp_msp2_inav_esc_telem_reply_t;
```

## MSP2_INAV_EZ_TUNE
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t eztune___enabled; // ezTune()->enabled
    uint16_t eztune___filterhz; // ezTune()->filterHz
    uint8_t eztune___axisratio; // ezTune()->axisRatio
    uint8_t eztune___response; // ezTune()->response
    uint8_t eztune___damping; // ezTune()->damping
    uint8_t eztune___stability; // ezTune()->stability
    uint8_t eztune___aggressiveness; // ezTune()->aggressiveness
    uint8_t eztune___rate; // ezTune()->rate
    uint8_t eztune___expo; // ezTune()->expo
    uint8_t eztune___snappiness; // ezTune()->snappiness
} msp_msp2_inav_ez_tune_reply_t;
```

## MSP2_INAV_GVAR_STATUS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t gvget_i; // gvGet(i)
} msp_msp2_inav_gvar_status_reply_t;
```

## MSP2_INAV_LED_STRIP_CONFIG_EX
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t ledconfig_sizeof_ledconfig_t; // ledConfig, sizeof(ledConfig_t)
} msp_msp2_inav_led_strip_config_ex_reply_t;
```

## MSP2_INAV_LOGIC_CONDITIONS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t logicconditions_i__enabled; // logicConditions(i)->enabled
    uint8_t logicconditions_i__activatorid; // logicConditions(i)->activatorId
    uint8_t logicconditions_i__operation; // logicConditions(i)->operation
    uint8_t logicconditions_i__operanda_type; // logicConditions(i)->operandA.type
    uint32_t logicconditions_i__operanda_value; // logicConditions(i)->operandA.value
    uint8_t logicconditions_i__operandb_type; // logicConditions(i)->operandB.type
    uint32_t logicconditions_i__operandb_value; // logicConditions(i)->operandB.value
    uint8_t logicconditions_i__flags; // logicConditions(i)->flags
} msp_msp2_inav_logic_conditions_reply_t;
```

## MSP2_INAV_LOGIC_CONDITIONS_STATUS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t logicconditiongetvalue_i; // logicConditionGetValue(i)
} msp_msp2_inav_logic_conditions_status_reply_t;
```

## MSP2_INAV_MC_BRAKING
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t navconfig___mc_braking_speed_threshold; // navConfig()->mc.braking_speed_threshold
    uint16_t navconfig___mc_braking_disengage_speed; // navConfig()->mc.braking_disengage_speed
    uint16_t navconfig___mc_braking_timeout; // navConfig()->mc.braking_timeout
    uint8_t navconfig___mc_braking_boost_factor; // navConfig()->mc.braking_boost_factor
    uint16_t navconfig___mc_braking_boost_timeout; // navConfig()->mc.braking_boost_timeout
    uint16_t navconfig___mc_braking_boost_speed_threshold; // navConfig()->mc.braking_boost_speed_threshold
    uint16_t navconfig___mc_braking_boost_disengage_speed; // navConfig()->mc.braking_boost_disengage_speed
    uint8_t navconfig___mc_braking_bank_angle; // navConfig()->mc.braking_bank_angle
} msp_msp2_inav_mc_braking_reply_t;
```

## MSP2_INAV_MISC
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t pwm_range_middle; // PWM_RANGE_MIDDLE
    uint16_t f_0; // 0
    uint16_t getmaxthrottle; // getMaxThrottle()
    uint16_t motorconfig___mincommand; // motorConfig()->mincommand
    uint16_t currentbatteryprofile_failsafe_throttle; // currentBatteryProfile->failsafe_throttle
    uint8_t gpsconfig___provider; // gpsConfig()->provider
    uint8_t f_0; // 0
    uint8_t gpsconfig___sbasmode; // gpsConfig()->sbasMode
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t rxconfig___rssi_channel; // rxConfig()->rssi_channel
    uint16_t compassconfig___mag_declination_10; // compassConfig()->mag_declination / 10
    uint16_t f_0; // 0
    uint16_t batterymetersconfig___voltage_scale; // batteryMetersConfig()->voltage.scale
    uint8_t batterymetersconfig___voltagesource; // batteryMetersConfig()->voltageSource
    uint8_t currentbatteryprofile_cells; // currentBatteryProfile->cells
    uint16_t currentbatteryprofile_voltage_celldetect; // currentBatteryProfile->voltage.cellDetect
    uint16_t currentbatteryprofile_voltage_cellmin; // currentBatteryProfile->voltage.cellMin
    uint16_t currentbatteryprofile_voltage_cellmax; // currentBatteryProfile->voltage.cellMax
    uint16_t currentbatteryprofile_voltage_cellwarning; // currentBatteryProfile->voltage.cellWarning
    uint16_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint32_t currentbatteryprofile_capacity_value; // currentBatteryProfile->capacity.value
    uint32_t currentbatteryprofile_capacity_warning; // currentBatteryProfile->capacity.warning
    uint32_t currentbatteryprofile_capacity_critical; // currentBatteryProfile->capacity.critical
    uint8_t batterymetersconfig___capacity_unit; // batteryMetersConfig()->capacity_unit
} msp_msp2_inav_misc_reply_t;
```

## MSP2_INAV_MISC2
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t micros___1000000; // micros() / 1000000
    uint32_t getflighttime; // getFlightTime()
    uint8_t getthrottlepercent_true; // getThrottlePercent(true)
    uint8_t navigationiscontrollingthrottle___1_0; // navigationIsControllingThrottle() ? 1 : 0
} msp_msp2_inav_misc2_reply_t;
```

## MSP2_INAV_MIXER
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t mixerconfig___motordirectioninverted; // mixerConfig()->motorDirectionInverted
    uint8_t f_0; // 0
    uint8_t mixerconfig___motorstoponlow; // mixerConfig()->motorstopOnLow
    uint8_t mixerconfig___platformtype; // mixerConfig()->platformType
    uint8_t mixerconfig___hasflaps; // mixerConfig()->hasFlaps
    uint16_t mixerconfig___appliedmixerpreset; // mixerConfig()->appliedMixerPreset
    uint8_t max_supported_motors; // MAX_SUPPORTED_MOTORS
    uint8_t max_supported_servos; // MAX_SUPPORTED_SERVOS
} msp_msp2_inav_mixer_reply_t;
```

## MSP2_INAV_OPTICAL_FLOW
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t opflow_rawquality; // opflow.rawQuality
    uint16_t radians_to_degrees_opflow_flowrate_x; // RADIANS_TO_DEGREES(opflow.flowRate[X])
    uint16_t radians_to_degrees_opflow_flowrate_y; // RADIANS_TO_DEGREES(opflow.flowRate[Y])
    uint16_t radians_to_degrees_opflow_bodyrate_x; // RADIANS_TO_DEGREES(opflow.bodyRate[X])
    uint16_t radians_to_degrees_opflow_bodyrate_y; // RADIANS_TO_DEGREES(opflow.bodyRate[Y])
    uint8_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
} msp_msp2_inav_optical_flow_reply_t;
```

## MSP2_INAV_OSD_ALARMS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t osdconfig___rssi_alarm; // osdConfig()->rssi_alarm
    uint16_t osdconfig___time_alarm; // osdConfig()->time_alarm
    uint16_t osdconfig___alt_alarm; // osdConfig()->alt_alarm
    uint16_t osdconfig___dist_alarm; // osdConfig()->dist_alarm
    uint16_t osdconfig___neg_alt_alarm; // osdConfig()->neg_alt_alarm
    uint16_t osdconfig___gforce_alarm___1000; // osdConfig()->gforce_alarm * 1000
    int16_t int16_t__osdconfig___gforce_axis_alarm_min___1000; // (int16_t)(osdConfig()->gforce_axis_alarm_min * 1000)
    int16_t int16_t__osdconfig___gforce_axis_alarm_max___1000; // (int16_t)(osdConfig()->gforce_axis_alarm_max * 1000)
    uint8_t osdconfig___current_alarm; // osdConfig()->current_alarm
    uint16_t osdconfig___imu_temp_alarm_min; // osdConfig()->imu_temp_alarm_min
    uint16_t osdconfig___imu_temp_alarm_max; // osdConfig()->imu_temp_alarm_max
    uint16_t osdconfig___baro_temp_alarm_min; // osdConfig()->baro_temp_alarm_min
    uint16_t osdconfig___baro_temp_alarm_max; // osdConfig()->baro_temp_alarm_max
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t osdconfig___adsb_distance_warning; // osdConfig()->adsb_distance_warning
    uint16_t osdconfig___adsb_distance_alert; // osdConfig()->adsb_distance_alert
    uint16_t f_0; // 0
    uint16_t f_0; // 0
} msp_msp2_inav_osd_alarms_reply_t;
```

## MSP2_INAV_OSD_LAYOUTS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t osdlayoutsconfig___item_pos_layout__item; // osdLayoutsConfig()->item_pos[layout][item]
    uint16_t osdlayoutsconfig___item_pos_layout__ii; // osdLayoutsConfig()->item_pos[layout][ii]
    uint8_t osd_layout_count; // OSD_LAYOUT_COUNT
    uint8_t osd_item_count; // OSD_ITEM_COUNT
} msp_msp2_inav_osd_layouts_reply_t;
```

## MSP2_INAV_OSD_PREFERENCES
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t osdconfig___video_system; // osdConfig()->video_system
    uint8_t osdconfig___main_voltage_decimals; // osdConfig()->main_voltage_decimals
    uint8_t osdconfig___ahi_reverse_roll; // osdConfig()->ahi_reverse_roll
    uint8_t osdconfig___crosshairs_style; // osdConfig()->crosshairs_style
    uint8_t osdconfig___left_sidebar_scroll; // osdConfig()->left_sidebar_scroll
    uint8_t osdconfig___right_sidebar_scroll; // osdConfig()->right_sidebar_scroll
    uint8_t osdconfig___sidebar_scroll_arrows; // osdConfig()->sidebar_scroll_arrows
    uint8_t osdconfig___units; // osdConfig()->units
    uint8_t osdconfig___stats_energy_unit; // osdConfig()->stats_energy_unit
} msp_msp2_inav_osd_preferences_reply_t;
```

## MSP2_INAV_OUTPUT_MAPPING
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t timerhardware_i__usageflags; // timerHardware[i].usageFlags
} msp_msp2_inav_output_mapping_reply_t;
```

## MSP2_INAV_OUTPUT_MAPPING_EXT
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t i; // i
    uint8_t timer2id_timerhardware_i__tim; // timer2id(timerHardware[i].tim)
    uint8_t timerhardware_i__usageflags; // timerHardware[i].usageFlags
} msp_msp2_inav_output_mapping_ext_reply_t;
```

## MSP2_INAV_OUTPUT_MAPPING_EXT2
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t i; // i
    uint8_t timer2id_timerhardware_i__tim; // timer2id(timerHardware[i].tim)
    uint32_t timerhardware_i__usageflags; // timerHardware[i].usageFlags
    uint8_t f_0; // 0
    uint8_t timerhardware_i__tag_led_tag_pin_label_led_pin_label_none; // timerHardware[i].tag == led_tag ? PIN_LABEL_LED : PIN_LABEL_NONE
} msp_msp2_inav_output_mapping_ext2_reply_t;
```

## MSP2_INAV_PROGRAMMING_PID
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t programmingpids_i__enabled; // programmingPids(i)->enabled
    uint8_t programmingpids_i__setpoint_type; // programmingPids(i)->setpoint.type
    uint32_t programmingpids_i__setpoint_value; // programmingPids(i)->setpoint.value
    uint8_t programmingpids_i__measurement_type; // programmingPids(i)->measurement.type
    uint32_t programmingpids_i__measurement_value; // programmingPids(i)->measurement.value
    uint16_t programmingpids_i__gains_p; // programmingPids(i)->gains.P
    uint16_t programmingpids_i__gains_i; // programmingPids(i)->gains.I
    uint16_t programmingpids_i__gains_d; // programmingPids(i)->gains.D
    uint16_t programmingpids_i__gains_ff; // programmingPids(i)->gains.FF
} msp_msp2_inav_programming_pid_reply_t;
```

## MSP2_INAV_PROGRAMMING_PID_STATUS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t programmingpidgetoutput_i; // programmingPidGetOutput(i)
} msp_msp2_inav_programming_pid_status_reply_t;
```

## MSP2_INAV_RATE_DYNAMICS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t currentcontrolrateprofile_ratedynamics_sensitivitycenter; // currentControlRateProfile->rateDynamics.sensitivityCenter
    uint8_t currentcontrolrateprofile_ratedynamics_sensitivityend; // currentControlRateProfile->rateDynamics.sensitivityEnd
    uint8_t currentcontrolrateprofile_ratedynamics_correctioncenter; // currentControlRateProfile->rateDynamics.correctionCenter
    uint8_t currentcontrolrateprofile_ratedynamics_correctionend; // currentControlRateProfile->rateDynamics.correctionEnd
    uint8_t currentcontrolrateprofile_ratedynamics_weightcenter; // currentControlRateProfile->rateDynamics.weightCenter
    uint8_t currentcontrolrateprofile_ratedynamics_weightend; // currentControlRateProfile->rateDynamics.weightEnd
} msp_msp2_inav_rate_dynamics_reply_t;
```

## MSP2_INAV_RATE_PROFILE
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t currentcontrolrateprofile_throttle_rcmid8; // currentControlRateProfile->throttle.rcMid8
    uint8_t currentcontrolrateprofile_throttle_rcexpo8; // currentControlRateProfile->throttle.rcExpo8
    uint8_t currentcontrolrateprofile_throttle_dynpid; // currentControlRateProfile->throttle.dynPID
    uint16_t currentcontrolrateprofile_throttle_pa_breakpoint; // currentControlRateProfile->throttle.pa_breakpoint
    uint8_t currentcontrolrateprofile_stabilized_rcexpo8; // currentControlRateProfile->stabilized.rcExpo8
    uint8_t currentcontrolrateprofile_stabilized_rcyawexpo8; // currentControlRateProfile->stabilized.rcYawExpo8
    uint8_t currentcontrolrateprofile_stabilized_rates_i; // currentControlRateProfile->stabilized.rates[i]
    uint8_t currentcontrolrateprofile_manual_rcexpo8; // currentControlRateProfile->manual.rcExpo8
    uint8_t currentcontrolrateprofile_manual_rcyawexpo8; // currentControlRateProfile->manual.rcYawExpo8
    uint8_t currentcontrolrateprofile_manual_rates_i; // currentControlRateProfile->manual.rates[i]
} msp_msp2_inav_rate_profile_reply_t;
```

## MSP2_INAV_SERVO_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t servoparams_i__min; // servoParams(i)->min
    uint16_t servoparams_i__max; // servoParams(i)->max
    uint16_t servoparams_i__middle; // servoParams(i)->middle
    uint8_t servoparams_i__rate; // servoParams(i)->rate
} msp_msp2_inav_servo_config_reply_t;
```

## MSP2_INAV_SERVO_MIXER
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t customservomixers_i__targetchannel; // customServoMixers(i)->targetChannel
    uint8_t customservomixers_i__inputsource; // customServoMixers(i)->inputSource
    uint16_t customservomixers_i__rate; // customServoMixers(i)->rate
    uint8_t customservomixers_i__speed; // customServoMixers(i)->speed
    uint8_t customservomixers_i__conditionid; // customServoMixers(i)->conditionId
    uint8_t f_1; // -1
    uint8_t mixerservomixersbyindex_nextmixerprofileindex__i__targetchannel; // mixerServoMixersByIndex(nextMixerProfileIndex)[i].targetChannel
    uint8_t mixerservomixersbyindex_nextmixerprofileindex__i__inputsource; // mixerServoMixersByIndex(nextMixerProfileIndex)[i].inputSource
    uint16_t mixerservomixersbyindex_nextmixerprofileindex__i__rate; // mixerServoMixersByIndex(nextMixerProfileIndex)[i].rate
    uint8_t mixerservomixersbyindex_nextmixerprofileindex__i__speed; // mixerServoMixersByIndex(nextMixerProfileIndex)[i].speed
    uint8_t mixerservomixersbyindex_nextmixerprofileindex__i__conditionid; // mixerServoMixersByIndex(nextMixerProfileIndex)[i].conditionId
    uint8_t f_1; // -1
} msp_msp2_inav_servo_mixer_reply_t;
```

## MSP2_INAV_STATUS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t uint16_t_cycletime; // (uint16_t)cycleTime
    uint16_t i2cgeterrorcounter; // i2cGetErrorCounter()
    uint16_t f_0; // 0
    uint16_t packsensorstatus; // packSensorStatus()
    uint16_t averagesystemloadpercent; // averageSystemLoadPercent
    uint8_t getconfigbatteryprofile___4__getconfigprofile; // (getConfigBatteryProfile() << 4) | getConfigProfile()
    uint32_t armingflags; // armingFlags
    uint8_t mspboxmodeflags[sizeof(mspBoxModeFlags)]; // &mspBoxModeFlags, sizeof(mspBoxModeFlags)
    uint8_t getconfigmixerprofile; // getConfigMixerProfile()
} msp_msp2_inav_status_reply_t;
```

## MSP2_INAV_TEMPERATURES
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t valid_temperature_1000; // valid ? temperature : -1000
} msp_msp2_inav_temperatures_reply_t;
```

## MSP2_INAV_TEMP_SENSOR_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t sensorconfig_type; // sensorConfig->type
    uint8_t uint8_t____sensorconfig_address__addrindex; // ((uint8_t *)&sensorConfig->address)[addrIndex]
    uint16_t sensorconfig_alarm_min; // sensorConfig->alarm_min
    uint16_t sensorconfig_alarm_max; // sensorConfig->alarm_max
    uint8_t sensorconfig_osdsymbol; // sensorConfig->osdSymbol
    uint8_t sensorconfig_label_labelindex; // sensorConfig->label[labelIndex]
} msp_msp2_inav_temp_sensor_config_reply_t;
```

## MSP2_INAV_TIMER_OUTPUT_MODE
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t i; // i
    uint8_t timeroverrides_i__outputmode; // timerOverrides(i)->outputMode
    uint8_t timer; // timer
    uint8_t timeroverrides_timer__outputmode; // timerOverrides(timer)->outputMode
} msp_msp2_inav_timer_output_mode_reply_t;
```

## MSP2_PID
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t constrain_pidbank___pid_i__p_0_255; // constrain(pidBank()->pid[i].P, 0, 255)
    uint8_t constrain_pidbank___pid_i__i_0_255; // constrain(pidBank()->pid[i].I, 0, 255)
    uint8_t constrain_pidbank___pid_i__d_0_255; // constrain(pidBank()->pid[i].D, 0, 255)
    uint8_t constrain_pidbank___pid_i__ff_0_255; // constrain(pidBank()->pid[i].FF, 0, 255)
} msp_msp2_pid_reply_t;
```

## MSP_3D
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t reversiblemotorsconfig___deadband_low; // reversibleMotorsConfig()->deadband_low
    uint16_t reversiblemotorsconfig___deadband_high; // reversibleMotorsConfig()->deadband_high
    uint16_t reversiblemotorsconfig___neutral; // reversibleMotorsConfig()->neutral
} msp_msp_3d_reply_t;
```

## MSP_ACTIVEBOXES
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t mspboxmodeflags[sizeof(mspBoxModeFlags)]; // &mspBoxModeFlags, sizeof(mspBoxModeFlags)
} msp_msp_activeboxes_reply_t;
```

## MSP_ADJUSTMENT_RANGES
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t adjrange_adjustmentindex; // adjRange->adjustmentIndex
    uint8_t adjrange_auxchannelindex; // adjRange->auxChannelIndex
    uint8_t adjrange_range_startstep; // adjRange->range.startStep
    uint8_t adjrange_range_endstep; // adjRange->range.endStep
    uint8_t adjrange_adjustmentfunction; // adjRange->adjustmentFunction
    uint8_t adjrange_auxswitchchannelindex; // adjRange->auxSwitchChannelIndex
} msp_msp_adjustment_ranges_reply_t;
```

## MSP_ADVANCED_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t f_1; // 1
    uint8_t f_1; // 1
    uint8_t f_1; // 1
    uint8_t motorconfig___motorpwmprotocol; // motorConfig()->motorPwmProtocol
    uint16_t motorconfig___motorpwmrate; // motorConfig()->motorPwmRate
    uint16_t servoconfig___servopwmrate; // servoConfig()->servoPwmRate
    uint8_t f_0; // 0
} msp_msp_advanced_config_reply_t;
```

## MSP_ALTITUDE
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t lrintf_getestimatedactualposition_z; // lrintf(getEstimatedActualPosition(Z))
    uint16_t lrintf_getestimatedactualvelocity_z; // lrintf(getEstimatedActualVelocity(Z))
    uint32_t barogetlatestaltitude; // baroGetLatestAltitude()
    uint32_t f_0; // 0
} msp_msp_altitude_reply_t;
```

## MSP_ANALOG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t uint8_t_constrain_getbatteryvoltage___10_0_255; // (uint8_t)constrain(getBatteryVoltage() / 10, 0, 255)
    uint16_t uint16_t_constrain_getmahdrawn___0_0xffff; // (uint16_t)constrain(getMAhDrawn(), 0, 0xFFFF)
    uint16_t getrssi; // getRSSI()
    int16_t int16_t_constrain_getamperage___0x8000_0x7fff; // (int16_t)constrain(getAmperage(), -0x8000, 0x7FFF)
} msp_msp_analog_reply_t;
```

## MSP_API_VERSION
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t msp_protocol_version; // MSP_PROTOCOL_VERSION
    uint8_t api_version_major; // API_VERSION_MAJOR
    uint8_t api_version_minor; // API_VERSION_MINOR
} msp_msp_api_version_reply_t;
```

## MSP_ATTITUDE
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t attitude_values_roll; // attitude.values.roll
    uint16_t attitude_values_pitch; // attitude.values.pitch
    uint16_t decidegrees_to_degrees_attitude_values_yaw; // DECIDEGREES_TO_DEGREES(attitude.values.yaw)
} msp_msp_attitude_reply_t;
```

## MSP_BATTERY_STATE
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t constrain_getbatterycellcount___0_255; // constrain(getBatteryCellCount(), 0, 255)
    uint16_t currentbatteryprofile_capacity_value; // currentBatteryProfile->capacity.value
    uint8_t constrain_getbatteryvoltage___10_0_255; // constrain(getBatteryVoltage() / 10, 0, 255)
    uint16_t constrain_getmahdrawn___0_0xffff; // constrain(getMAhDrawn(), 0, 0xFFFF)
    uint16_t constrain_getamperage___0x8000_0x7fff; // constrain(getAmperage(), -0x8000, 0x7FFF)
    uint8_t getbatterystate; // getBatteryState()
    uint16_t getbatteryvoltage; // getBatteryVoltage()
} msp_msp_battery_state_reply_t;
```

## MSP_BLACKBOX_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
} msp_msp_blackbox_config_reply_t;
```

## MSP_BOARD_ALIGNMENT
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t boardalignment___rolldecidegrees; // boardAlignment()->rollDeciDegrees
    uint16_t boardalignment___pitchdecidegrees; // boardAlignment()->pitchDeciDegrees
    uint16_t boardalignment___yawdecidegrees; // boardAlignment()->yawDeciDegrees
} msp_msp_board_alignment_reply_t;
```

## MSP_BOARD_INFO
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t boardidentifier[BOARD_IDENTIFIER_LENGTH]; // boardIdentifier, BOARD_IDENTIFIER_LENGTH
    uint16_t hardwarerevision; // hardwareRevision
    uint16_t f_0; // 0
    uint8_t f_2; // 2
    uint8_t f_0; // 0
    uint8_t commcapabilities; // commCapabilities
    uint8_t strlen_targetname; // strlen(targetName)
    uint8_t targetname[strlen(targetName)]; // targetName, strlen(targetName)
} msp_msp_board_info_reply_t;
```

## MSP_BUILD_INFO
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t builddate[BUILD_DATE_LENGTH]; // buildDate, BUILD_DATE_LENGTH
    uint8_t buildtime[BUILD_TIME_LENGTH]; // buildTime, BUILD_TIME_LENGTH
    uint8_t shortgitrevision[GIT_SHORT_REVISION_LENGTH]; // shortGitRevision, GIT_SHORT_REVISION_LENGTH
} msp_msp_build_info_reply_t;
```

## MSP_CALIBRATION_DATA
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t accgetcalibrationaxisflags; // accGetCalibrationAxisFlags()
    uint16_t accelerometerconfig___acczero_raw_x; // accelerometerConfig()->accZero.raw[X]
    uint16_t accelerometerconfig___acczero_raw_y; // accelerometerConfig()->accZero.raw[Y]
    uint16_t accelerometerconfig___acczero_raw_z; // accelerometerConfig()->accZero.raw[Z]
    uint16_t accelerometerconfig___accgain_raw_x; // accelerometerConfig()->accGain.raw[X]
    uint16_t accelerometerconfig___accgain_raw_y; // accelerometerConfig()->accGain.raw[Y]
    uint16_t accelerometerconfig___accgain_raw_z; // accelerometerConfig()->accGain.raw[Z]
    uint16_t compassconfig___magzero_raw_x; // compassConfig()->magZero.raw[X]
    uint16_t compassconfig___magzero_raw_y; // compassConfig()->magZero.raw[Y]
    uint16_t compassconfig___magzero_raw_z; // compassConfig()->magZero.raw[Z]
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t opticalflowconfig___opflow_scale___256; // opticalFlowConfig()->opflow_scale * 256
    uint16_t f_0; // 0
    uint16_t compassconfig___maggain_x; // compassConfig()->magGain[X]
    uint16_t compassconfig___maggain_y; // compassConfig()->magGain[Y]
    uint16_t compassconfig___maggain_z; // compassConfig()->magGain[Z]
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
} msp_msp_calibration_data_reply_t;
```

## MSP_COMP_GPS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t gps_distancetohome; // GPS_distanceToHome
    uint16_t gps_directiontohome; // GPS_directionToHome
    uint8_t gpssol_flags_gpsheartbeat_1_0; // gpsSol.flags.gpsHeartbeat ? 1 : 0
} msp_msp_comp_gps_reply_t;
```

## MSP_CURRENT_METER_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t batterymetersconfig___current_scale; // batteryMetersConfig()->current.scale
    uint16_t batterymetersconfig___current_offset; // batteryMetersConfig()->current.offset
    uint8_t batterymetersconfig___current_type; // batteryMetersConfig()->current.type
    uint16_t constrain_currentbatteryprofile_capacity_value_0_0xffff; // constrain(currentBatteryProfile->capacity.value, 0, 0xFFFF)
} msp_msp_current_meter_config_reply_t;
```

## MSP_DEBUG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t debug_i; // debug[i]
} msp_msp_debug_reply_t;
```

## MSP_FAILSAFE_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t failsafeconfig___failsafe_delay; // failsafeConfig()->failsafe_delay
    uint8_t failsafeconfig___failsafe_off_delay; // failsafeConfig()->failsafe_off_delay
    uint16_t currentbatteryprofile_failsafe_throttle; // currentBatteryProfile->failsafe_throttle
    uint8_t f_0; // 0
    uint16_t failsafeconfig___failsafe_throttle_low_delay; // failsafeConfig()->failsafe_throttle_low_delay
    uint8_t failsafeconfig___failsafe_procedure; // failsafeConfig()->failsafe_procedure
    uint8_t failsafeconfig___failsafe_recovery_delay; // failsafeConfig()->failsafe_recovery_delay
    uint16_t failsafeconfig___failsafe_fw_roll_angle; // failsafeConfig()->failsafe_fw_roll_angle
    uint16_t failsafeconfig___failsafe_fw_pitch_angle; // failsafeConfig()->failsafe_fw_pitch_angle
    uint16_t failsafeconfig___failsafe_fw_yaw_rate; // failsafeConfig()->failsafe_fw_yaw_rate
    uint16_t failsafeconfig___failsafe_stick_motion_threshold; // failsafeConfig()->failsafe_stick_motion_threshold
    uint16_t failsafeconfig___failsafe_min_distance; // failsafeConfig()->failsafe_min_distance
    uint8_t failsafeconfig___failsafe_min_distance_procedure; // failsafeConfig()->failsafe_min_distance_procedure
} msp_msp_failsafe_config_reply_t;
```

## MSP_FC_VARIANT
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t flightcontrolleridentifier[FLIGHT_CONTROLLER_IDENTIFIER_LENGTH]; // flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH
} msp_msp_fc_variant_reply_t;
```

## MSP_FC_VERSION
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t fc_version_major; // FC_VERSION_MAJOR
    uint8_t fc_version_minor; // FC_VERSION_MINOR
    uint8_t fc_version_patch_level; // FC_VERSION_PATCH_LEVEL
} msp_msp_fc_version_reply_t;
```

## MSP_FEATURE
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t featuremask; // featureMask()
} msp_msp_feature_reply_t;
```

## MSP_FILTER_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t gyroconfig___gyro_main_lpf_hz; // gyroConfig()->gyro_main_lpf_hz
    uint16_t pidprofile___dterm_lpf_hz; // pidProfile()->dterm_lpf_hz
    uint16_t pidprofile___yaw_lpf_hz; // pidProfile()->yaw_lpf_hz
    uint16_t f_0; // 0
    uint16_t f_1; // 1
    uint16_t f_0; // 0
    uint16_t f_1; // 1
    uint16_t f_0; // 0
    uint16_t f_1; // 1
    uint16_t accelerometerconfig___acc_notch_hz; // accelerometerConfig()->acc_notch_hz
    uint16_t accelerometerconfig___acc_notch_cutoff; // accelerometerConfig()->acc_notch_cutoff
    uint16_t f_0; // 0
} msp_msp_filter_config_reply_t;
```

## MSP_FW_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t currentbatteryprofile_nav_fw_cruise_throttle; // currentBatteryProfile->nav.fw.cruise_throttle
    uint16_t currentbatteryprofile_nav_fw_min_throttle; // currentBatteryProfile->nav.fw.min_throttle
    uint16_t currentbatteryprofile_nav_fw_max_throttle; // currentBatteryProfile->nav.fw.max_throttle
    uint8_t navconfig___fw_max_bank_angle; // navConfig()->fw.max_bank_angle
    uint8_t navconfig___fw_max_climb_angle; // navConfig()->fw.max_climb_angle
    uint8_t navconfig___fw_max_dive_angle; // navConfig()->fw.max_dive_angle
    uint8_t currentbatteryprofile_nav_fw_pitch_to_throttle; // currentBatteryProfile->nav.fw.pitch_to_throttle
    uint16_t navconfig___fw_loiter_radius; // navConfig()->fw.loiter_radius
} msp_msp_fw_config_reply_t;
```

## MSP_GPSSTATISTICS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t gpsstats_lastmessagedt; // gpsStats.lastMessageDt
    uint32_t gpsstats_errors; // gpsStats.errors
    uint32_t gpsstats_timeouts; // gpsStats.timeouts
    uint32_t gpsstats_packetcount; // gpsStats.packetCount
    uint16_t gpssol_hdop; // gpsSol.hdop
    uint16_t gpssol_eph; // gpsSol.eph
    uint16_t gpssol_epv; // gpsSol.epv
} msp_msp_gpsstatistics_reply_t;
```

## MSP_GPSSVINFO
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t f_1; // 1
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t gpssol_hdop_100; // gpsSol.hdop / 100
    uint8_t gpssol_hdop_100; // gpsSol.hdop / 100
} msp_msp_gpssvinfo_reply_t;
```

## MSP_INAV_PID
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint8_t pidprofile___heading_hold_rate_limit; // pidProfile()->heading_hold_rate_limit
    uint8_t heading_hold_error_lpf_freq; // HEADING_HOLD_ERROR_LPF_FREQ
    uint16_t f_0; // 0
    uint8_t gyro_lpf_256hz; // GYRO_LPF_256HZ
    uint8_t accelerometerconfig___acc_lpf_hz; // accelerometerConfig()->acc_lpf_hz
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
} msp_msp_inav_pid_reply_t;
```

## MSP_LED_COLORS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t color_h; // color->h
    uint8_t color_s; // color->s
    uint8_t color_v; // color->v
} msp_msp_led_colors_reply_t;
```

## MSP_LED_STRIP_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t legacyledconfig; // legacyLedConfig
} msp_msp_led_strip_config_reply_t;
```

## MSP_LED_STRIP_MODECOLOR
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t i; // i
    uint8_t j; // j
    uint8_t ledstripconfig___modecolors_i__color_j; // ledStripConfig()->modeColors[i].color[j]
    uint8_t led_mode_count; // LED_MODE_COUNT
    uint8_t j; // j
    uint8_t ledstripconfig___specialcolors_color_j; // ledStripConfig()->specialColors.color[j]
} msp_msp_led_strip_modecolor_reply_t;
```

## MSP_LOOP_TIME
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t gyroconfig___looptime; // gyroConfig()->looptime
} msp_msp_loop_time_reply_t;
```

## MSP_MISC
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t pwm_range_middle; // PWM_RANGE_MIDDLE
    uint16_t f_0; // 0
    uint16_t getmaxthrottle; // getMaxThrottle()
    uint16_t motorconfig___mincommand; // motorConfig()->mincommand
    uint16_t currentbatteryprofile_failsafe_throttle; // currentBatteryProfile->failsafe_throttle
    uint8_t gpsconfig___provider; // gpsConfig()->provider
    uint8_t f_0; // 0
    uint8_t gpsconfig___sbasmode; // gpsConfig()->sbasMode
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t rxconfig___rssi_channel; // rxConfig()->rssi_channel
    uint8_t f_0; // 0
    uint16_t compassconfig___mag_declination_10; // compassConfig()->mag_declination / 10
    uint16_t f_0; // 0
    uint8_t batterymetersconfig___voltage_scale_10; // batteryMetersConfig()->voltage.scale / 10
    uint8_t currentbatteryprofile_voltage_cellmin_10; // currentBatteryProfile->voltage.cellMin / 10
    uint8_t currentbatteryprofile_voltage_cellmax_10; // currentBatteryProfile->voltage.cellMax / 10
    uint8_t currentbatteryprofile_voltage_cellwarning_10; // currentBatteryProfile->voltage.cellWarning / 10
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
} msp_msp_misc_reply_t;
```

## MSP_MIXER
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t f_3; // 3
} msp_msp_mixer_reply_t;
```

## MSP_MODE_RANGES
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t box_box_permanentid_0; // box ? box->permanentId : 0
    uint8_t mac_auxchannelindex; // mac->auxChannelIndex
    uint8_t mac_range_startstep; // mac->range.startStep
    uint8_t mac_range_endstep; // mac->range.endStep
} msp_msp_mode_ranges_reply_t;
```

## MSP_MOTOR
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t i_max_supported_motors_motor_i__0; // i < MAX_SUPPORTED_MOTORS ? motor[i] : 0
} msp_msp_motor_reply_t;
```

## MSP_NAME
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t name; // *name++
} msp_msp_name_reply_t;
```

## MSP_NAV_POSHOLD
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t navconfig___general_flags_user_control_mode; // navConfig()->general.flags.user_control_mode
    uint16_t navconfig___general_max_auto_speed; // navConfig()->general.max_auto_speed
    uint16_t mixerconfig___platformtype_platform_airplane_navconfig___fw_max_auto_climb_rate_navconfig___mc_max_auto_climb_rate; // mixerConfig()->platformType == PLATFORM_AIRPLANE ? navConfig()->fw.max_auto_climb_rate : navConfig()->mc.max_auto_climb_rate
    uint16_t navconfig___general_max_manual_speed; // navConfig()->general.max_manual_speed
    uint16_t mixerconfig___platformtype_platform_airplane_navconfig___fw_max_manual_climb_rate_navconfig___mc_max_manual_climb_rate; // mixerConfig()->platformType == PLATFORM_AIRPLANE ? navConfig()->fw.max_manual_climb_rate : navConfig()->mc.max_manual_climb_rate
    uint8_t navconfig___mc_max_bank_angle; // navConfig()->mc.max_bank_angle
    uint8_t navconfig___mc_althold_throttle_type; // navConfig()->mc.althold_throttle_type
    uint16_t currentbatteryprofile_nav_mc_hover_throttle; // currentBatteryProfile->nav.mc.hover_throttle
} msp_msp_nav_poshold_reply_t;
```

## MSP_NAV_STATUS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t nav_status_mode; // NAV_Status.mode
    uint8_t nav_status_state; // NAV_Status.state
    uint8_t nav_status_activewpaction; // NAV_Status.activeWpAction
    uint8_t nav_status_activewpnumber; // NAV_Status.activeWpNumber
    uint8_t nav_status_error; // NAV_Status.error
    int16_t int16_t__target_bearing_100; // (int16_t)(target_bearing/100)
    uint16_t getheadingholdtarget; // getHeadingHoldTarget()
} msp_msp_nav_status_reply_t;
```

## MSP_OSD_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t osd_driver_max7456; // OSD_DRIVER_MAX7456
    uint8_t osdconfig___video_system; // osdConfig()->video_system
    uint8_t osdconfig___units; // osdConfig()->units
    uint8_t osdconfig___rssi_alarm; // osdConfig()->rssi_alarm
    uint16_t currentbatteryprofile_capacity_warning; // currentBatteryProfile->capacity.warning
    uint16_t osdconfig___time_alarm; // osdConfig()->time_alarm
    uint16_t osdconfig___alt_alarm; // osdConfig()->alt_alarm
    uint16_t osdconfig___dist_alarm; // osdConfig()->dist_alarm
    uint16_t osdconfig___neg_alt_alarm; // osdConfig()->neg_alt_alarm
    uint16_t osdlayoutsconfig___item_pos_0__i; // osdLayoutsConfig()->item_pos[0][i]
    uint8_t osd_driver_none; // OSD_DRIVER_NONE
} msp_msp_osd_config_reply_t;
```

## MSP_PASSTHROUGH_ESC_4WAY
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t esc4wayinit; // esc4wayInit()
    uint8_t f_0; // 0
    uint8_t flags; // flags
    uint8_t state; // state
    uint8_t afatfs_getlasterror; // afatfs_getLastError()
    uint32_t afatfs_getcontiguousfreespace___1024; // afatfs_getContiguousFreeSpace() / 1024
    uint32_t sdcard_getmetadata___numblocks_2; // sdcard_getMetadata()->numBlocks / 2
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint32_t f_0; // 0
    uint32_t f_0; // 0
    uint8_t flashisready___1_0; // flashIsReady() ? 1 : 0
    uint32_t geometry_sectors; // geometry->sectors
    uint32_t geometry_totalsize; // geometry->totalSize
    uint32_t flashfsgetoffset; // flashfsGetOffset()
    uint8_t f_0; // 0
    uint32_t f_0; // 0
    uint32_t f_0; // 0
    uint32_t f_0; // 0
    uint32_t address; // address
} msp_msp_passthrough_esc_4way_reply_t;
```

## MSP_PASSTHROUGH_SERIAL_FUNCTION_ID
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t f_1; // 1
    uint8_t f_0; // 0
} msp_msp_passthrough_serial_function_id_reply_t;
```

## MSP_PIDNAMES
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t c; // *c
} msp_msp_pidnames_reply_t;
```

## MSP_PID_ADVANCED
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint16_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint16_t f_0; // 0
    uint8_t f_0; // 0
    uint16_t constrain_pidprofile___axisaccelerationlimitrollpitch_10_0_65535; // constrain(pidProfile()->axisAccelerationLimitRollPitch / 10, 0, 65535)
    uint16_t constrain_pidprofile___axisaccelerationlimityaw_10_0_65535; // constrain(pidProfile()->axisAccelerationLimitYaw / 10, 0, 65535)
} msp_msp_pid_advanced_reply_t;
```

## MSP_POSITION_ESTIMATION_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t positionestimationconfig___w_z_baro_p___100; // positionEstimationConfig()->w_z_baro_p * 100
    uint16_t positionestimationconfig___w_z_gps_p___100; // positionEstimationConfig()->w_z_gps_p * 100
    uint16_t positionestimationconfig___w_z_gps_v___100; // positionEstimationConfig()->w_z_gps_v * 100
    uint16_t positionestimationconfig___w_xy_gps_p___100; // positionEstimationConfig()->w_xy_gps_p * 100
    uint16_t positionestimationconfig___w_xy_gps_v___100; // positionEstimationConfig()->w_xy_gps_v * 100
    uint8_t gpsconfigmutable___gpsminsats; // gpsConfigMutable()->gpsMinSats
    uint8_t f_1; // 1
} msp_msp_position_estimation_config_reply_t;
```

## MSP_RAW_GPS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t gpssol_fixtype; // gpsSol.fixType
    uint8_t gpssol_numsat; // gpsSol.numSat
    uint32_t gpssol_llh_lat; // gpsSol.llh.lat
    uint32_t gpssol_llh_lon; // gpsSol.llh.lon
    uint16_t gpssol_llh_alt_100; // gpsSol.llh.alt/100
    uint16_t gpssol_groundspeed; // gpsSol.groundSpeed
    uint16_t gpssol_groundcourse; // gpsSol.groundCourse
    uint16_t gpssol_hdop; // gpsSol.hdop
} msp_msp_raw_gps_reply_t;
```

## MSP_RAW_IMU
### Reply payload
```c
typedef struct __attribute__((packed)) {
    int16_t int16_t_lrintf_acc_accadcf_i____512; // (int16_t)lrintf(acc.accADCf[i] * 512)
    uint16_t gyroratedps_i; // gyroRateDps(i)
    uint16_t lrintf_mag_magadc_i; // lrintf(mag.magADC[i])
    uint16_t f_0; // 0
} msp_msp_raw_imu_reply_t;
```

## MSP_RC
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t rxgetchannelvalue_i; // rxGetChannelValue(i)
} msp_msp_rc_reply_t;
```

## MSP_RC_DEADBAND
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t rccontrolsconfig___deadband; // rcControlsConfig()->deadband
    uint8_t rccontrolsconfig___yaw_deadband; // rcControlsConfig()->yaw_deadband
    uint8_t rccontrolsconfig___alt_hold_deadband; // rcControlsConfig()->alt_hold_deadband
    uint16_t rccontrolsconfig___mid_throttle_deadband; // rcControlsConfig()->mid_throttle_deadband
} msp_msp_rc_deadband_reply_t;
```

## MSP_RC_TUNING
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t f_100; // 100
    uint8_t currentcontrolrateprofile_stabilized_rcexpo8; // currentControlRateProfile->stabilized.rcExpo8
    uint8_t currentcontrolrateprofile_stabilized_rates_i; // currentControlRateProfile->stabilized.rates[i]
    uint8_t currentcontrolrateprofile_throttle_dynpid; // currentControlRateProfile->throttle.dynPID
    uint8_t currentcontrolrateprofile_throttle_rcmid8; // currentControlRateProfile->throttle.rcMid8
    uint8_t currentcontrolrateprofile_throttle_rcexpo8; // currentControlRateProfile->throttle.rcExpo8
    uint16_t currentcontrolrateprofile_throttle_pa_breakpoint; // currentControlRateProfile->throttle.pa_breakpoint
    uint8_t currentcontrolrateprofile_stabilized_rcyawexpo8; // currentControlRateProfile->stabilized.rcYawExpo8
} msp_msp_rc_tuning_reply_t;
```

## MSP_RSSI_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t rxconfig___rssi_channel; // rxConfig()->rssi_channel
} msp_msp_rssi_config_reply_t;
```

## MSP_RTC
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t uint32_t_seconds; // (uint32_t)seconds
    uint16_t millis; // millis
} msp_msp_rtc_reply_t;
```

## MSP_RTH_AND_LAND_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t navconfig___general_min_rth_distance; // navConfig()->general.min_rth_distance
    uint8_t navconfig___general_flags_rth_climb_first; // navConfig()->general.flags.rth_climb_first
    uint8_t navconfig___general_flags_rth_climb_ignore_emerg; // navConfig()->general.flags.rth_climb_ignore_emerg
    uint8_t navconfig___general_flags_rth_tail_first; // navConfig()->general.flags.rth_tail_first
    uint8_t navconfig___general_flags_rth_allow_landing; // navConfig()->general.flags.rth_allow_landing
    uint8_t navconfig___general_flags_rth_alt_control_mode; // navConfig()->general.flags.rth_alt_control_mode
    uint16_t navconfig___general_rth_abort_threshold; // navConfig()->general.rth_abort_threshold
    uint16_t navconfig___general_rth_altitude; // navConfig()->general.rth_altitude
    uint16_t navconfig___general_land_minalt_vspd; // navConfig()->general.land_minalt_vspd
    uint16_t navconfig___general_land_maxalt_vspd; // navConfig()->general.land_maxalt_vspd
    uint16_t navconfig___general_land_slowdown_minalt; // navConfig()->general.land_slowdown_minalt
    uint16_t navconfig___general_land_slowdown_maxalt; // navConfig()->general.land_slowdown_maxalt
    uint16_t navconfig___general_emerg_descent_rate; // navConfig()->general.emerg_descent_rate
} msp_msp_rth_and_land_config_reply_t;
```

## MSP_RX_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t rxconfig___serialrx_provider; // rxConfig()->serialrx_provider
    uint16_t rxconfig___maxcheck; // rxConfig()->maxcheck
    uint16_t pwm_range_middle; // PWM_RANGE_MIDDLE
    uint16_t rxconfig___mincheck; // rxConfig()->mincheck
    uint8_t rxconfig___spektrum_sat_bind; // rxConfig()->spektrum_sat_bind
    uint8_t f_0; // 0
    uint16_t rxconfig___rx_min_usec; // rxConfig()->rx_min_usec
    uint16_t rxconfig___rx_max_usec; // rxConfig()->rx_max_usec
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint16_t f_0; // 0
    uint8_t f_0; // 0
    uint32_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t rxconfig___receivertype; // rxConfig()->receiverType
} msp_msp_rx_config_reply_t;
```

## MSP_RX_MAP
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t rxconfig___rcmap[MAX_MAPPABLE_RX_INPUTS]; // rxConfig()->rcmap, MAX_MAPPABLE_RX_INPUTS
} msp_msp_rx_map_reply_t;
```

## MSP_SENSOR_ALIGNMENT
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t compassconfig___mag_align; // compassConfig()->mag_align
    uint8_t f_0; // 0
    uint8_t opticalflowconfig___opflow_align; // opticalFlowConfig()->opflow_align
    uint8_t f_0; // 0
} msp_msp_sensor_alignment_reply_t;
```

## MSP_SENSOR_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t accelerometerconfig___acc_hardware; // accelerometerConfig()->acc_hardware
    uint8_t barometerconfig___baro_hardware; // barometerConfig()->baro_hardware
    uint8_t f_0; // 0
    uint8_t compassconfig___mag_hardware; // compassConfig()->mag_hardware
    uint8_t f_0; // 0
    uint8_t pitotmeterconfig___pitot_hardware; // pitotmeterConfig()->pitot_hardware
    uint8_t f_0; // 0
    uint8_t rangefinderconfig___rangefinder_hardware; // rangefinderConfig()->rangefinder_hardware
    uint8_t f_0; // 0
    uint8_t opticalflowconfig___opflow_hardware; // opticalFlowConfig()->opflow_hardware
    uint8_t f_0; // 0
} msp_msp_sensor_config_reply_t;
```

## MSP_SENSOR_STATUS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t ishardwarehealthy___1_0; // isHardwareHealthy() ? 1 : 0
    uint8_t gethwgyrostatus; // getHwGyroStatus()
    uint8_t gethwaccelerometerstatus; // getHwAccelerometerStatus()
    uint8_t gethwcompassstatus; // getHwCompassStatus()
    uint8_t gethwbarometerstatus; // getHwBarometerStatus()
    uint8_t gethwgpsstatus; // getHwGPSStatus()
    uint8_t gethwrangefinderstatus; // getHwRangefinderStatus()
    uint8_t gethwpitotmeterstatus; // getHwPitotmeterStatus()
    uint8_t gethwopticalflowstatus; // getHwOpticalFlowStatus()
} msp_msp_sensor_status_reply_t;
```

## MSP_SERVO
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t servo[MAX_SUPPORTED_SERVOS * 2]; // &servo, MAX_SUPPORTED_SERVOS * 2
} msp_msp_servo_reply_t;
```

## MSP_SERVO_CONFIGURATIONS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t servoparams_i__min; // servoParams(i)->min
    uint16_t servoparams_i__max; // servoParams(i)->max
    uint16_t servoparams_i__middle; // servoParams(i)->middle
    uint8_t servoparams_i__rate; // servoParams(i)->rate
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_255; // 255
    uint32_t f_0; // 0
} msp_msp_servo_configurations_reply_t;
```

## MSP_SERVO_MIX_RULES
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t customservomixers_i__targetchannel; // customServoMixers(i)->targetChannel
    uint8_t customservomixers_i__inputsource; // customServoMixers(i)->inputSource
    uint16_t customservomixers_i__rate; // customServoMixers(i)->rate
    uint8_t customservomixers_i__speed; // customServoMixers(i)->speed
    uint8_t f_0; // 0
    uint8_t f_100; // 100
    uint8_t f_0; // 0
} msp_msp_servo_mix_rules_reply_t;
```

## MSP_SIMULATOR
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t uint16_t_simulatordata_input_input_stabilized_roll; // (uint16_t)simulatorData.input[INPUT_STABILIZED_ROLL]
    uint16_t uint16_t_simulatordata_input_input_stabilized_pitch; // (uint16_t)simulatorData.input[INPUT_STABILIZED_PITCH]
    uint16_t uint16_t_simulatordata_input_input_stabilized_yaw; // (uint16_t)simulatorData.input[INPUT_STABILIZED_YAW]
    uint16_t uint16_t__arming_flag_armed__simulatordata_input_input_stabilized_throttle__500; // (uint16_t)(ARMING_FLAG(ARMED) ? simulatorData.input[INPUT_STABILIZED_THROTTLE] : -500)
    uint8_t tmp_u8; // tmp_u8
    uint32_t debug_simulatordata_debugindex; // debug[simulatorData.debugIndex]
    uint16_t attitude_values_roll; // attitude.values.roll
    uint16_t attitude_values_pitch; // attitude.values.pitch
    uint16_t attitude_values_yaw; // attitude.values.yaw
} msp_msp_simulator_reply_t;
```

## MSP_SONAR_ALTITUDE
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t rangefindergetlatestaltitude; // rangefinderGetLatestAltitude()
    uint32_t f_0; // 0
} msp_msp_sonar_altitude_reply_t;
```

## MSP_STATUS
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint16_t uint16_t_cycletime; // (uint16_t)cycleTime
    uint16_t i2cgeterrorcounter; // i2cGetErrorCounter()
    uint16_t f_0; // 0
    uint16_t packsensorstatus; // packSensorStatus()
    uint8_t mspboxmodeflags[4]; // &mspBoxModeFlags, 4
    uint8_t getconfigprofile; // getConfigProfile()
    uint16_t averagesystemloadpercent; // averageSystemLoadPercent
    uint16_t armingflags; // armingFlags
    uint8_t accgetcalibrationaxisflags; // accGetCalibrationAxisFlags()
} msp_msp_status_reply_t;
```

## MSP_TX_INFO
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t getrssisource; // getRSSISource()
    uint8_t rtcdatetimeisset; // rtcDateTimeIsSet
} msp_msp_tx_info_reply_t;
```

## MSP_UID
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint32_t u_id_0; // U_ID_0
    uint32_t u_id_1; // U_ID_1
    uint32_t u_id_2; // U_ID_2
} msp_msp_uid_reply_t;
```

## MSP_VOLTAGE_METER_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t batterymetersconfig___voltage_scale_10; // batteryMetersConfig()->voltage.scale / 10
    uint8_t currentbatteryprofile_voltage_cellmin_10; // currentBatteryProfile->voltage.cellMin / 10
    uint8_t currentbatteryprofile_voltage_cellmax_10; // currentBatteryProfile->voltage.cellMax / 10
    uint8_t currentbatteryprofile_voltage_cellwarning_10; // currentBatteryProfile->voltage.cellWarning / 10
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
    uint8_t f_0; // 0
} msp_msp_voltage_meter_config_reply_t;
```

## MSP_VTXTABLE_POWERLEVEL
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t powerlevel; // powerLevel
    uint16_t f_0; // 0
    uint8_t str_len; // str_len
    uint8_t str_i; // str[i]
} msp_msp_vtxtable_powerlevel_reply_t;
```

## MSP_VTX_CONFIG
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t devicetype; // deviceType
    uint8_t vtxsettingsconfig___band; // vtxSettingsConfig()->band
    uint8_t vtxsettingsconfig___channel; // vtxSettingsConfig()->channel
    uint8_t vtxsettingsconfig___power; // vtxSettingsConfig()->power
    uint8_t pitmode; // pitmode
    uint16_t vtxsettingsconfig___freq; // vtxSettingsConfig()->freq
    uint8_t vtxcommondeviceisready_vtxdevice__1_0; // vtxCommonDeviceIsReady(vtxDevice) ? 1 : 0
    uint8_t vtxsettingsconfig___lowpowerdisarm; // vtxSettingsConfig()->lowPowerDisarm
    uint8_t f_1; // 1
    uint8_t vtxdevice_capability_bandcount; // vtxDevice->capability.bandCount
    uint8_t vtxdevice_capability_channelcount; // vtxDevice->capability.channelCount
    uint8_t vtxdevice_capability_powercount; // vtxDevice->capability.powerCount
    uint8_t vtxdev_unknown; // VTXDEV_UNKNOWN
    uint8_t vtxdev_unknown; // VTXDEV_UNKNOWN
} msp_msp_vtx_config_reply_t;
```

## MSP_WP_GETINFO
### Reply payload
```c
typedef struct __attribute__((packed)) {
    uint8_t f_0; // 0
    uint8_t nav_max_waypoints; // NAV_MAX_WAYPOINTS
    uint8_t iswaypointlistvalid; // isWaypointListValid()
    uint8_t getwaypointcount; // getWaypointCount()
} msp_msp_wp_getinfo_reply_t;
``` 
