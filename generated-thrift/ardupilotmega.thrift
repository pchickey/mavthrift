/* This thrift file has been autogenerated */

include "common.thrift"
namespace cpp mavlink.thrift

/* enums: */
enum ArdupilotmegaMessageTypes {
  HEARTBEAT,
  SYS_STATUS,
  SYSTEM_TIME,
  PING,
  CHANGE_OPERATOR_CONTROL,
  CHANGE_OPERATOR_CONTROL_ACK,
  AUTH_KEY,
  SET_MODE,
  PARAM_REQUEST_READ,
  PARAM_REQUEST_LIST,
  PARAM_VALUE,
  PARAM_SET,
  GPS_RAW_INT,
  GPS_STATUS,
  SCALED_IMU,
  RAW_IMU,
  RAW_PRESSURE,
  SCALED_PRESSURE,
  ATTITUDE,
  ATTITUDE_QUATERNION,
  LOCAL_POSITION_NED,
  GLOBAL_POSITION_INT,
  RC_CHANNELS_SCALED,
  RC_CHANNELS_RAW,
  SERVO_OUTPUT_RAW,
  MISSION_REQUEST_PARTIAL_LIST,
  MISSION_WRITE_PARTIAL_LIST,
  MISSION_ITEM,
  MISSION_REQUEST,
  MISSION_SET_CURRENT,
  MISSION_CURRENT,
  MISSION_REQUEST_LIST,
  MISSION_COUNT,
  MISSION_CLEAR_ALL,
  MISSION_ITEM_REACHED,
  MISSION_ACK,
  SET_GPS_GLOBAL_ORIGIN,
  GPS_GLOBAL_ORIGIN,
  SET_LOCAL_POSITION_SETPOINT,
  LOCAL_POSITION_SETPOINT,
  GLOBAL_POSITION_SETPOINT_INT,
  SET_GLOBAL_POSITION_SETPOINT_INT,
  SAFETY_SET_ALLOWED_AREA,
  SAFETY_ALLOWED_AREA,
  SET_ROLL_PITCH_YAW_THRUST,
  SET_ROLL_PITCH_YAW_SPEED_THRUST,
  ROLL_PITCH_YAW_THRUST_SETPOINT,
  ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT,
  SET_QUAD_MOTORS_SETPOINT,
  SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST,
  NAV_CONTROLLER_OUTPUT,
  SET_QUAD_SWARM_LED_ROLL_PITCH_YAW_THRUST,
  STATE_CORRECTION,
  REQUEST_DATA_STREAM,
  DATA_STREAM,
  MANUAL_CONTROL,
  RC_CHANNELS_OVERRIDE,
  VFR_HUD,
  COMMAND_LONG,
  COMMAND_ACK,
  ROLL_PITCH_YAW_RATES_THRUST_SETPOINT,
  MANUAL_SETPOINT,
  LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET,
  HIL_STATE,
  HIL_CONTROLS,
  HIL_RC_INPUTS_RAW,
  OPTICAL_FLOW,
  GLOBAL_VISION_POSITION_ESTIMATE,
  VISION_POSITION_ESTIMATE,
  VISION_SPEED_ESTIMATE,
  VICON_POSITION_ESTIMATE,
  HIGHRES_IMU,
  OMNIDIRECTIONAL_FLOW,
  FILE_TRANSFER_START,
  FILE_TRANSFER_DIR_LIST,
  FILE_TRANSFER_RES,
  BATTERY_STATUS,
  SETPOINT_8DOF,
  SETPOINT_6DOF,
  MEMORY_VECT,
  DEBUG_VECT,
  NAMED_VALUE_FLOAT,
  NAMED_VALUE_INT,
  STATUSTEXT,
  DEBUG,
  SENSOR_OFFSETS,
  SET_MAG_OFFSETS,
  MEMINFO,
  AP_ADC,
  DIGICAM_CONFIGURE,
  DIGICAM_CONTROL,
  MOUNT_CONFIGURE,
  MOUNT_CONTROL,
  MOUNT_STATUS,
  FENCE_POINT,
  FENCE_FETCH_POINT,
  FENCE_STATUS,
  AHRS,
  SIMSTATE,
  HWSTATUS,
  RADIO,
  LIMITS_STATUS,
  WIND,
  DATA16,
  DATA32,
  DATA64,
  DATA96,
  RANGEFINDER
}

enum Mount_mode {
  RETRACT = 0,
  NEUTRAL = 1,
  MAVLINK_TARGETING = 2,
  RC_TARGETING = 3,
  GPS_POINT = 4
}

enum Cmd {
  NAV_WAYPOINT = 16,
  NAV_LOITER_UNLIM = 17,
  NAV_LOITER_TURNS = 18,
  NAV_LOITER_TIME = 19,
  NAV_RETURN_TO_LAUNCH = 20,
  NAV_LAND = 21,
  NAV_TAKEOFF = 22,
  NAV_ROI = 80,
  NAV_PATHPLANNING = 81,
  NAV_LAST = 95,
  CONDITION_DELAY = 112,
  CONDITION_CHANGE_ALT = 113,
  CONDITION_DISTANCE = 114,
  CONDITION_YAW = 115,
  CONDITION_LAST = 159,
  DO_SET_MODE = 176,
  DO_JUMP = 177,
  DO_CHANGE_SPEED = 178,
  DO_SET_HOME = 179,
  DO_SET_PARAMETER = 180,
  DO_SET_RELAY = 181,
  DO_REPEAT_RELAY = 182,
  DO_SET_SERVO = 183,
  DO_REPEAT_SERVO = 184,
  DO_CONTROL_VIDEO = 200,
  DO_DIGICAM_CONFIGURE = 202,
  DO_DIGICAM_CONTROL = 203,
  DO_MOUNT_CONFIGURE = 204,
  DO_MOUNT_CONTROL = 205,
  DO_LAST = 240,
  PREFLIGHT_CALIBRATION = 241,
  PREFLIGHT_SET_SENSOR_OFFSETS = 242,
  PREFLIGHT_STORAGE = 245,
  PREFLIGHT_REBOOT_SHUTDOWN = 246,
  OVERRIDE_GOTO = 252,
  MISSION_START = 300,
  COMPONENT_ARM_DISARM = 400
}

enum Fence_action {
  NONE = 0,
  GUIDED = 1,
  REPORT = 2
}

enum Fence_breach {
  NONE = 0,
  MINALT = 1,
  MAXALT = 2,
  BOUNDARY = 3
}

enum Limits_state {
  INIT = 0,
  DISABLED = 1,
  ENABLED = 2,
  TRIGGERED = 3,
  RECOVERING = 4,
  RECOVERED = 5,
  STATE_ENUM_END = 6
}

enum Limit_module {
  GPSLOCK = 1,
  GEOFENCE = 2,
  ALTITUDE = 4,
  MODULE_ENUM_END = 5
}


/* structs: */
struct SensorOffsets {
  1: i16 mag_ofs_x,
  2: i16 mag_ofs_y,
  3: i16 mag_ofs_z,
  4: double mag_declination,
  5: i32 raw_press,
  6: i32 raw_temp,
  7: double gyro_cal_x,
  8: double gyro_cal_y,
  9: double gyro_cal_z,
  10: double accel_cal_x,
  11: double accel_cal_y,
  12: double accel_cal_z
}

struct SetMagOffsets {
  1: byte target_system,
  2: byte target_component,
  3: i16 mag_ofs_x,
  4: i16 mag_ofs_y,
  5: i16 mag_ofs_z
}

struct Meminfo {
  1: i16 brkval,
  2: i16 freemem
}

struct ApAdc {
  1: i16 adc1,
  2: i16 adc2,
  3: i16 adc3,
  4: i16 adc4,
  5: i16 adc5,
  6: i16 adc6
}

struct DigicamConfigure {
  1: byte target_system,
  2: byte target_component,
  3: byte mode,
  4: i16 shutter_speed,
  5: byte aperture,
  6: byte iso,
  7: byte exposure_type,
  8: byte command_id,
  9: byte engine_cut_off,
  10: byte extra_param,
  11: double extra_value
}

struct DigicamControl {
  1: byte target_system,
  2: byte target_component,
  3: byte session,
  4: byte zoom_pos,
  5: byte zoom_step,
  6: byte focus_lock,
  7: byte shot,
  8: byte command_id,
  9: byte extra_param,
  10: double extra_value
}

struct MountConfigure {
  1: byte target_system,
  2: byte target_component,
  3: byte mount_mode,
  4: byte stab_roll,
  5: byte stab_pitch,
  6: byte stab_yaw
}

struct MountControl {
  1: byte target_system,
  2: byte target_component,
  3: i32 input_a,
  4: i32 input_b,
  5: i32 input_c,
  6: byte save_position
}

struct MountStatus {
  1: byte target_system,
  2: byte target_component,
  3: i32 pointing_a,
  4: i32 pointing_b,
  5: i32 pointing_c
}

struct FencePoint {
  1: byte target_system,
  2: byte target_component,
  3: byte idx,
  4: byte count,
  5: double lat,
  6: double lng
}

struct FenceFetchPoint {
  1: byte target_system,
  2: byte target_component,
  3: byte idx
}

struct FenceStatus {
  1: byte breach_status,
  2: i16 breach_count,
  3: byte breach_type,
  4: i32 breach_time
}

struct Ahrs {
  1: double omegaIx,
  2: double omegaIy,
  3: double omegaIz,
  4: double accel_weight,
  5: double renorm_val,
  6: double error_rp,
  7: double error_yaw
}

struct Simstate {
  1: double roll,
  2: double pitch,
  3: double yaw,
  4: double xacc,
  5: double yacc,
  6: double zacc,
  7: double xgyro,
  8: double ygyro,
  9: double zgyro,
  10: double lat,
  11: double lng
}

struct Hwstatus {
  1: i16 Vcc,
  2: byte I2Cerr
}

struct Radio {
  1: byte rssi,
  2: byte remrssi,
  3: byte txbuf,
  4: byte noise,
  5: byte remnoise,
  6: i16 rxerrors,
  7: i16 fixed
}

struct LimitsStatus {
  1: byte limits_state,
  2: i32 last_trigger,
  3: i32 last_action,
  4: i32 last_recovery,
  5: i32 last_clear,
  6: i16 breach_count,
  7: byte mods_enabled,
  8: byte mods_required,
  9: byte mods_triggered
}

struct Wind {
  1: double direction,
  2: double speed,
  3: double speed_z
}

struct Data16 {
  1: byte data16_type,
  2: byte len,
  3: list<byte> data
}

struct Data32 {
  1: byte data32_type,
  2: byte len,
  3: list<byte> data
}

struct Data64 {
  1: byte data64_type,
  2: byte len,
  3: list<byte> data
}

struct Data96 {
  1: byte data96_type,
  2: byte len,
  3: list<byte> data
}

struct Rangefinder {
  1: double distance,
  2: double voltage
}


/* exceptions : */


/* services: */
service ArdupilotmegaMessagePostService  extends common.CommonMessagePostService {
  void postSensorOffsets(1: SensorOffsets msg) throws (1: common.InvalidMavlinkMessage err),
  void postSetMagOffsets(1: SetMagOffsets msg) throws (1: common.InvalidMavlinkMessage err),
  void postMeminfo(1: Meminfo msg) throws (1: common.InvalidMavlinkMessage err),
  void postApAdc(1: ApAdc msg) throws (1: common.InvalidMavlinkMessage err),
  void postDigicamConfigure(1: DigicamConfigure msg) throws (1: common.InvalidMavlinkMessage err),
  void postDigicamControl(1: DigicamControl msg) throws (1: common.InvalidMavlinkMessage err),
  void postMountConfigure(1: MountConfigure msg) throws (1: common.InvalidMavlinkMessage err),
  void postMountControl(1: MountControl msg) throws (1: common.InvalidMavlinkMessage err),
  void postMountStatus(1: MountStatus msg) throws (1: common.InvalidMavlinkMessage err),
  void postFencePoint(1: FencePoint msg) throws (1: common.InvalidMavlinkMessage err),
  void postFenceFetchPoint(1: FenceFetchPoint msg) throws (1: common.InvalidMavlinkMessage err),
  void postFenceStatus(1: FenceStatus msg) throws (1: common.InvalidMavlinkMessage err),
  void postAhrs(1: Ahrs msg) throws (1: common.InvalidMavlinkMessage err),
  void postSimstate(1: Simstate msg) throws (1: common.InvalidMavlinkMessage err),
  void postHwstatus(1: Hwstatus msg) throws (1: common.InvalidMavlinkMessage err),
  void postRadio(1: Radio msg) throws (1: common.InvalidMavlinkMessage err),
  void postLimitsStatus(1: LimitsStatus msg) throws (1: common.InvalidMavlinkMessage err),
  void postWind(1: Wind msg) throws (1: common.InvalidMavlinkMessage err),
  void postData16(1: Data16 msg) throws (1: common.InvalidMavlinkMessage err),
  void postData32(1: Data32 msg) throws (1: common.InvalidMavlinkMessage err),
  void postData64(1: Data64 msg) throws (1: common.InvalidMavlinkMessage err),
  void postData96(1: Data96 msg) throws (1: common.InvalidMavlinkMessage err),
  void postRangefinder(1: Rangefinder msg) throws (1: common.InvalidMavlinkMessage err)
}

service ArdupilotmegaMessageFetchService  extends common.CommonMessageFetchService {
  map<ArdupilotmegaMessageTypes,i32> availableMessages(),
  void fetchSensorOffsets(1: list<SensorOffsets> msg),
  void fetchSetMagOffsets(1: list<SetMagOffsets> msg),
  void fetchMeminfo(1: list<Meminfo> msg),
  void fetchApAdc(1: list<ApAdc> msg),
  void fetchDigicamConfigure(1: list<DigicamConfigure> msg),
  void fetchDigicamControl(1: list<DigicamControl> msg),
  void fetchMountConfigure(1: list<MountConfigure> msg),
  void fetchMountControl(1: list<MountControl> msg),
  void fetchMountStatus(1: list<MountStatus> msg),
  void fetchFencePoint(1: list<FencePoint> msg),
  void fetchFenceFetchPoint(1: list<FenceFetchPoint> msg),
  void fetchFenceStatus(1: list<FenceStatus> msg),
  void fetchAhrs(1: list<Ahrs> msg),
  void fetchSimstate(1: list<Simstate> msg),
  void fetchHwstatus(1: list<Hwstatus> msg),
  void fetchRadio(1: list<Radio> msg),
  void fetchLimitsStatus(1: list<LimitsStatus> msg),
  void fetchWind(1: list<Wind> msg),
  void fetchData16(1: list<Data16> msg),
  void fetchData32(1: list<Data32> msg),
  void fetchData64(1: list<Data64> msg),
  void fetchData96(1: list<Data96> msg),
  void fetchRangefinder(1: list<Rangefinder> msg)
}