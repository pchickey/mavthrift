
#include "SimpleFetchServer.h"

using namespace ::apache::thrift;
using namespace ::mavlink::thrift;

SimpleFetchServerHandler::SimpleFetchServerHandler() {}

void SimpleFetchServerHandler::availableMessages(std::map<CommonMessageTypes::type, int32_t> & _return) {
    _return[CommonMessageTypes::HEARTBEAT] = (int32_t) _Heartbeat_msg.size();
    _return[CommonMessageTypes::SYS_STATUS] = (int32_t)  _SysStatus_msg.size();
    _return[CommonMessageTypes::SYSTEM_TIME] = (int32_t)  _SystemTime_msg.size();
    _return[CommonMessageTypes::PING] = (int32_t)  _Ping_msg.size();
    _return[CommonMessageTypes::CHANGE_OPERATOR_CONTROL] = (int32_t)  _ChangeOperatorControl_msg.size();
    _return[CommonMessageTypes::CHANGE_OPERATOR_CONTROL_ACK] = (int32_t)  _ChangeOperatorControlAck_msg.size();
    _return[CommonMessageTypes::AUTH_KEY] = (int32_t)  _AuthKey_msg.size();
    _return[CommonMessageTypes::SET_MODE] = (int32_t)  _SetMode_msg.size();
    _return[CommonMessageTypes::PARAM_REQUEST_READ] = (int32_t)  _ParamRequestRead_msg.size();
    _return[CommonMessageTypes::PARAM_REQUEST_LIST] = (int32_t)  _ParamRequestList_msg.size();
    _return[CommonMessageTypes::PARAM_VALUE] = (int32_t)  _ParamValue_msg.size();
    _return[CommonMessageTypes::PARAM_SET] = (int32_t)  _ParamSet_msg.size();
    _return[CommonMessageTypes::GPS_RAW_INT] = (int32_t)  _GpsRawInt_msg.size();
    _return[CommonMessageTypes::GPS_STATUS] = (int32_t)  _GpsStatus_msg.size();
    _return[CommonMessageTypes::SCALED_IMU] = (int32_t)  _ScaledImu_msg.size();
    _return[CommonMessageTypes::RAW_IMU] = (int32_t)  _RawImu_msg.size();
    _return[CommonMessageTypes::RAW_PRESSURE] = (int32_t)  _RawPressure_msg.size();
    _return[CommonMessageTypes::SCALED_PRESSURE] = (int32_t)  _ScaledPressure_msg.size();
    _return[CommonMessageTypes::ATTITUDE] = (int32_t)  _Attitude_msg.size();
    _return[CommonMessageTypes::ATTITUDE_QUATERNION] = (int32_t)  _AttitudeQuaternion_msg.size();
    _return[CommonMessageTypes::LOCAL_POSITION_NED] = (int32_t)  _LocalPositionNed_msg.size();
    _return[CommonMessageTypes::GLOBAL_POSITION_INT] = (int32_t)  _GlobalPositionInt_msg.size();
    _return[CommonMessageTypes::RC_CHANNELS_SCALED] = (int32_t)  _RcChannelsScaled_msg.size();
    _return[CommonMessageTypes::RC_CHANNELS_RAW] = (int32_t)  _RcChannelsRaw_msg.size();
    _return[CommonMessageTypes::SERVO_OUTPUT_RAW] = (int32_t)  _ServoOutputRaw_msg.size();
    _return[CommonMessageTypes::MISSION_REQUEST_PARTIAL_LIST] = (int32_t)  _MissionRequestPartialList_msg.size();
    _return[CommonMessageTypes::MISSION_WRITE_PARTIAL_LIST] = (int32_t)  _MissionWritePartialList_msg.size();
    _return[CommonMessageTypes::MISSION_ITEM] = (int32_t)  _MissionItem_msg.size();
    _return[CommonMessageTypes::MISSION_REQUEST] = (int32_t)  _MissionRequest_msg.size();
    _return[CommonMessageTypes::MISSION_SET_CURRENT] = (int32_t)  _MissionSetCurrent_msg.size();
    _return[CommonMessageTypes::MISSION_CURRENT] = (int32_t)  _MissionCurrent_msg.size();
    _return[CommonMessageTypes::MISSION_REQUEST_LIST] = (int32_t)  _MissionRequestList_msg.size();
    _return[CommonMessageTypes::MISSION_COUNT] = (int32_t)  _MissionCount_msg.size();
    _return[CommonMessageTypes::MISSION_CLEAR_ALL] = (int32_t)  _MissionClearAll_msg.size();
    _return[CommonMessageTypes::MISSION_ITEM_REACHED] = (int32_t)  _MissionItemReached_msg.size();
    _return[CommonMessageTypes::MISSION_ACK] = (int32_t)  _MissionAck_msg.size();
    _return[CommonMessageTypes::SET_GPS_GLOBAL_ORIGIN] = (int32_t)  _SetGpsGlobalOrigin_msg.size();
    _return[CommonMessageTypes::GPS_GLOBAL_ORIGIN] = (int32_t)  _GpsGlobalOrigin_msg.size();
    _return[CommonMessageTypes::SET_LOCAL_POSITION_SETPOINT] = (int32_t)  _SetLocalPositionSetpoint_msg.size();
    _return[CommonMessageTypes::LOCAL_POSITION_SETPOINT] = (int32_t)  _LocalPositionSetpoint_msg.size();
    _return[CommonMessageTypes::GLOBAL_POSITION_INT] = (int32_t)  _GlobalPositionSetpointInt_msg.size();
    _return[CommonMessageTypes::SET_GLOBAL_POSITION_SETPOINT_INT] = (int32_t)  _SetGlobalPositionSetpointInt_msg.size();
    _return[CommonMessageTypes::SAFETY_SET_ALLOWED_AREA] = (int32_t)  _SafetySetAllowedArea_msg.size();
    _return[CommonMessageTypes::SAFETY_ALLOWED_AREA] = (int32_t)  _SafetyAllowedArea_msg.size();
    _return[CommonMessageTypes::SET_ROLL_PITCH_YAW_THRUST] = (int32_t)  _SetRollPitchYawThrust_msg.size();
    _return[CommonMessageTypes::SET_ROLL_PITCH_YAW_SPEED_THRUST] = (int32_t)  _SetRollPitchYawSpeedThrust_msg.size();
    _return[CommonMessageTypes::ROLL_PITCH_YAW_THRUST_SETPOINT] = (int32_t)  _RollPitchYawThrustSetpoint_msg.size();
    _return[CommonMessageTypes::ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT] = (int32_t)  _RollPitchYawSpeedThrustSetpoint_msg.size();
    _return[CommonMessageTypes::SET_QUAD_MOTORS_SETPOINT] = (int32_t)  _SetQuadMotorsSetpoint_msg.size();
    _return[CommonMessageTypes::SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST] = (int32_t)  _SetQuadSwarmRollPitchYawThrust_msg.size();
    _return[CommonMessageTypes::NAV_CONTROLLER_OUTPUT] = (int32_t)  _NavControllerOutput_msg.size();
    _return[CommonMessageTypes::SET_QUAD_SWARM_LED_ROLL_PITCH_YAW_THRUST] = (int32_t)  _SetQuadSwarmLedRollPitchYawThrust_msg.size();
    _return[CommonMessageTypes::STATE_CORRECTION] = (int32_t)  _StateCorrection_msg.size();
    _return[CommonMessageTypes::REQUEST_DATA_STREAM] = (int32_t)  _RequestDataStream_msg.size();
    _return[CommonMessageTypes::DATA_STREAM] = (int32_t)  _DataStream_msg.size();
    _return[CommonMessageTypes::MANUAL_CONTROL] = (int32_t)  _ManualControl_msg.size();
    _return[CommonMessageTypes::RC_CHANNELS_OVERRIDE] = (int32_t)  _RcChannelsOverride_msg.size();
    _return[CommonMessageTypes::VFR_HUD] = (int32_t)  _VfrHud_msg.size();
    _return[CommonMessageTypes::COMMAND_LONG] = (int32_t)  _CommandLong_msg.size();
    _return[CommonMessageTypes::COMMAND_ACK] = (int32_t)  _CommandAck_msg.size();
    _return[CommonMessageTypes::ROLL_PITCH_YAW_RATES_THRUST_SETPOINT] = (int32_t)  _RollPitchYawRatesThrustSetpoint_msg.size();
    _return[CommonMessageTypes::MANUAL_SETPOINT] = (int32_t)  _ManualSetpoint_msg.size();
    _return[CommonMessageTypes::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET] = (int32_t)  _LocalPositionNedSystemGlobalOffset_msg.size();
    _return[CommonMessageTypes::HIL_STATE] = (int32_t)  _HilState_msg.size();
    _return[CommonMessageTypes::HIL_CONTROLS] = (int32_t)  _HilControls_msg.size();
    _return[CommonMessageTypes::HIL_RC_INPUTS_RAW] = (int32_t)  _HilRcInputsRaw_msg.size();
    _return[CommonMessageTypes::OPTICAL_FLOW] = (int32_t)  _OpticalFlow_msg.size();
    _return[CommonMessageTypes::GLOBAL_VISION_POSITION_ESTIMATE] = (int32_t)  _GlobalVisionPositionEstimate_msg.size();
    _return[CommonMessageTypes::VISION_POSITION_ESTIMATE] = (int32_t)  _VisionPositionEstimate_msg.size();
    _return[CommonMessageTypes::VISION_SPEED_ESTIMATE] = (int32_t)  _VisionSpeedEstimate_msg.size();
    _return[CommonMessageTypes::VICON_POSITION_ESTIMATE] = (int32_t)  _ViconPositionEstimate_msg.size();
    _return[CommonMessageTypes::HIGHRES_IMU] = (int32_t)  _HighresImu_msg.size();
    _return[CommonMessageTypes::OMNIDIRECTIONAL_FLOW] = (int32_t)  _OmnidirectionalFlow_msg.size();
    _return[CommonMessageTypes::FILE_TRANSFER_START] = (int32_t)  _FileTransferStart_msg.size();
    _return[CommonMessageTypes::FILE_TRANSFER_DIR_LIST] = (int32_t)  _FileTransferDirList_msg.size();
    _return[CommonMessageTypes::FILE_TRANSFER_RES] = (int32_t)  _FileTransferRes_msg.size();
    _return[CommonMessageTypes::BATTERY_STATUS] = (int32_t)  _BatteryStatus_msg.size();
    _return[CommonMessageTypes::SETPOINT_8DOF] = (int32_t)  _Setpoint8dof_msg.size();
    _return[CommonMessageTypes::SETPOINT_6DOF] = (int32_t)  _Setpoint6dof_msg.size();
    _return[CommonMessageTypes::MEMORY_VECT] = (int32_t)  _MemoryVect_msg.size();
    _return[CommonMessageTypes::DEBUG_VECT] = (int32_t)  _DebugVect_msg.size();
    _return[CommonMessageTypes::NAMED_VALUE_FLOAT] = (int32_t)  _NamedValueFloat_msg.size();
    _return[CommonMessageTypes::NAMED_VALUE_INT] = (int32_t)  _NamedValueInt_msg.size();
    _return[CommonMessageTypes::STATUSTEXT] = (int32_t)  _Statustext_msg.size();
    _return[CommonMessageTypes::DEBUG] = (int32_t)  _Debug_msg.size();
}



void SimpleFetchServerHandler::availableMessages(std::map<ArdupilotmegaMessageTypes::type, int32_t> & _return) {
    _return[ArdupilotmegaMessageTypes::HEARTBEAT] = (int32_t) _Heartbeat_msg.size();
    _return[ArdupilotmegaMessageTypes::SYS_STATUS] = (int32_t)  _SysStatus_msg.size();
    _return[ArdupilotmegaMessageTypes::SYSTEM_TIME] = (int32_t)  _SystemTime_msg.size();
    _return[ArdupilotmegaMessageTypes::PING] = (int32_t)  _Ping_msg.size();
    _return[ArdupilotmegaMessageTypes::CHANGE_OPERATOR_CONTROL] = (int32_t)  _ChangeOperatorControl_msg.size();
    _return[ArdupilotmegaMessageTypes::CHANGE_OPERATOR_CONTROL_ACK] = (int32_t)  _ChangeOperatorControlAck_msg.size();
    _return[ArdupilotmegaMessageTypes::AUTH_KEY] = (int32_t)  _AuthKey_msg.size();
    _return[ArdupilotmegaMessageTypes::SET_MODE] = (int32_t)  _SetMode_msg.size();
    _return[ArdupilotmegaMessageTypes::PARAM_REQUEST_READ] = (int32_t)  _ParamRequestRead_msg.size();
    _return[ArdupilotmegaMessageTypes::PARAM_REQUEST_LIST] = (int32_t)  _ParamRequestList_msg.size();
    _return[ArdupilotmegaMessageTypes::PARAM_VALUE] = (int32_t)  _ParamValue_msg.size();
    _return[ArdupilotmegaMessageTypes::PARAM_SET] = (int32_t)  _ParamSet_msg.size();
    _return[ArdupilotmegaMessageTypes::GPS_RAW_INT] = (int32_t)  _GpsRawInt_msg.size();
    _return[ArdupilotmegaMessageTypes::GPS_STATUS] = (int32_t)  _GpsStatus_msg.size();
    _return[ArdupilotmegaMessageTypes::SCALED_IMU] = (int32_t)  _ScaledImu_msg.size();
    _return[ArdupilotmegaMessageTypes::RAW_IMU] = (int32_t)  _RawImu_msg.size();
    _return[ArdupilotmegaMessageTypes::RAW_PRESSURE] = (int32_t)  _RawPressure_msg.size();
    _return[ArdupilotmegaMessageTypes::SCALED_PRESSURE] = (int32_t)  _ScaledPressure_msg.size();
    _return[ArdupilotmegaMessageTypes::ATTITUDE] = (int32_t)  _Attitude_msg.size();
    _return[ArdupilotmegaMessageTypes::ATTITUDE_QUATERNION] = (int32_t)  _AttitudeQuaternion_msg.size();
    _return[ArdupilotmegaMessageTypes::LOCAL_POSITION_NED] = (int32_t)  _LocalPositionNed_msg.size();
    _return[ArdupilotmegaMessageTypes::GLOBAL_POSITION_INT] = (int32_t)  _GlobalPositionInt_msg.size();
    _return[ArdupilotmegaMessageTypes::RC_CHANNELS_SCALED] = (int32_t)  _RcChannelsScaled_msg.size();
    _return[ArdupilotmegaMessageTypes::RC_CHANNELS_RAW] = (int32_t)  _RcChannelsRaw_msg.size();
    _return[ArdupilotmegaMessageTypes::SERVO_OUTPUT_RAW] = (int32_t)  _ServoOutputRaw_msg.size();
    _return[ArdupilotmegaMessageTypes::MISSION_REQUEST_PARTIAL_LIST] = (int32_t)  _MissionRequestPartialList_msg.size();
    _return[ArdupilotmegaMessageTypes::MISSION_WRITE_PARTIAL_LIST] = (int32_t)  _MissionWritePartialList_msg.size();
    _return[ArdupilotmegaMessageTypes::MISSION_ITEM] = (int32_t)  _MissionItem_msg.size();
    _return[ArdupilotmegaMessageTypes::MISSION_REQUEST] = (int32_t)  _MissionRequest_msg.size();
    _return[ArdupilotmegaMessageTypes::MISSION_SET_CURRENT] = (int32_t)  _MissionSetCurrent_msg.size();
    _return[ArdupilotmegaMessageTypes::MISSION_CURRENT] = (int32_t)  _MissionCurrent_msg.size();
    _return[ArdupilotmegaMessageTypes::MISSION_REQUEST_LIST] = (int32_t)  _MissionRequestList_msg.size();
    _return[ArdupilotmegaMessageTypes::MISSION_COUNT] = (int32_t)  _MissionCount_msg.size();
    _return[ArdupilotmegaMessageTypes::MISSION_CLEAR_ALL] = (int32_t)  _MissionClearAll_msg.size();
    _return[ArdupilotmegaMessageTypes::MISSION_ITEM_REACHED] = (int32_t)  _MissionItemReached_msg.size();
    _return[ArdupilotmegaMessageTypes::MISSION_ACK] = (int32_t)  _MissionAck_msg.size();
    _return[ArdupilotmegaMessageTypes::SET_GPS_GLOBAL_ORIGIN] = (int32_t)  _SetGpsGlobalOrigin_msg.size();
    _return[ArdupilotmegaMessageTypes::GPS_GLOBAL_ORIGIN] = (int32_t)  _GpsGlobalOrigin_msg.size();
    _return[ArdupilotmegaMessageTypes::SET_LOCAL_POSITION_SETPOINT] = (int32_t)  _SetLocalPositionSetpoint_msg.size();
    _return[ArdupilotmegaMessageTypes::LOCAL_POSITION_SETPOINT] = (int32_t)  _LocalPositionSetpoint_msg.size();
    _return[ArdupilotmegaMessageTypes::GLOBAL_POSITION_INT] = (int32_t)  _GlobalPositionSetpointInt_msg.size();
    _return[ArdupilotmegaMessageTypes::SET_GLOBAL_POSITION_SETPOINT_INT] = (int32_t)  _SetGlobalPositionSetpointInt_msg.size();
    _return[ArdupilotmegaMessageTypes::SAFETY_SET_ALLOWED_AREA] = (int32_t)  _SafetySetAllowedArea_msg.size();
    _return[ArdupilotmegaMessageTypes::SAFETY_ALLOWED_AREA] = (int32_t)  _SafetyAllowedArea_msg.size();
    _return[ArdupilotmegaMessageTypes::SET_ROLL_PITCH_YAW_THRUST] = (int32_t)  _SetRollPitchYawThrust_msg.size();
    _return[ArdupilotmegaMessageTypes::SET_ROLL_PITCH_YAW_SPEED_THRUST] = (int32_t)  _SetRollPitchYawSpeedThrust_msg.size();
    _return[ArdupilotmegaMessageTypes::ROLL_PITCH_YAW_THRUST_SETPOINT] = (int32_t)  _RollPitchYawThrustSetpoint_msg.size();
    _return[ArdupilotmegaMessageTypes::ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT] = (int32_t)  _RollPitchYawSpeedThrustSetpoint_msg.size();
    _return[ArdupilotmegaMessageTypes::SET_QUAD_MOTORS_SETPOINT] = (int32_t)  _SetQuadMotorsSetpoint_msg.size();
    _return[ArdupilotmegaMessageTypes::SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST] = (int32_t)  _SetQuadSwarmRollPitchYawThrust_msg.size();
    _return[ArdupilotmegaMessageTypes::NAV_CONTROLLER_OUTPUT] = (int32_t)  _NavControllerOutput_msg.size();
    _return[ArdupilotmegaMessageTypes::SET_QUAD_SWARM_LED_ROLL_PITCH_YAW_THRUST] = (int32_t)  _SetQuadSwarmLedRollPitchYawThrust_msg.size();
    _return[ArdupilotmegaMessageTypes::STATE_CORRECTION] = (int32_t)  _StateCorrection_msg.size();
    _return[ArdupilotmegaMessageTypes::REQUEST_DATA_STREAM] = (int32_t)  _RequestDataStream_msg.size();
    _return[ArdupilotmegaMessageTypes::DATA_STREAM] = (int32_t)  _DataStream_msg.size();
    _return[ArdupilotmegaMessageTypes::MANUAL_CONTROL] = (int32_t)  _ManualControl_msg.size();
    _return[ArdupilotmegaMessageTypes::RC_CHANNELS_OVERRIDE] = (int32_t)  _RcChannelsOverride_msg.size();
    _return[ArdupilotmegaMessageTypes::VFR_HUD] = (int32_t)  _VfrHud_msg.size();
    _return[ArdupilotmegaMessageTypes::COMMAND_LONG] = (int32_t)  _CommandLong_msg.size();
    _return[ArdupilotmegaMessageTypes::COMMAND_ACK] = (int32_t)  _CommandAck_msg.size();
    _return[ArdupilotmegaMessageTypes::ROLL_PITCH_YAW_RATES_THRUST_SETPOINT] = (int32_t)  _RollPitchYawRatesThrustSetpoint_msg.size();
    _return[ArdupilotmegaMessageTypes::MANUAL_SETPOINT] = (int32_t)  _ManualSetpoint_msg.size();
    _return[ArdupilotmegaMessageTypes::LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET] = (int32_t)  _LocalPositionNedSystemGlobalOffset_msg.size();
    _return[ArdupilotmegaMessageTypes::HIL_STATE] = (int32_t)  _HilState_msg.size();
    _return[ArdupilotmegaMessageTypes::HIL_CONTROLS] = (int32_t)  _HilControls_msg.size();
    _return[ArdupilotmegaMessageTypes::HIL_RC_INPUTS_RAW] = (int32_t)  _HilRcInputsRaw_msg.size();
    _return[ArdupilotmegaMessageTypes::OPTICAL_FLOW] = (int32_t)  _OpticalFlow_msg.size();
    _return[ArdupilotmegaMessageTypes::GLOBAL_VISION_POSITION_ESTIMATE] = (int32_t)  _GlobalVisionPositionEstimate_msg.size();
    _return[ArdupilotmegaMessageTypes::VISION_POSITION_ESTIMATE] = (int32_t)  _VisionPositionEstimate_msg.size();
    _return[ArdupilotmegaMessageTypes::VISION_SPEED_ESTIMATE] = (int32_t)  _VisionSpeedEstimate_msg.size();
    _return[ArdupilotmegaMessageTypes::VICON_POSITION_ESTIMATE] = (int32_t)  _ViconPositionEstimate_msg.size();
    _return[ArdupilotmegaMessageTypes::HIGHRES_IMU] = (int32_t)  _HighresImu_msg.size();
    _return[ArdupilotmegaMessageTypes::OMNIDIRECTIONAL_FLOW] = (int32_t)  _OmnidirectionalFlow_msg.size();
    _return[ArdupilotmegaMessageTypes::FILE_TRANSFER_START] = (int32_t)  _FileTransferStart_msg.size();
    _return[ArdupilotmegaMessageTypes::FILE_TRANSFER_DIR_LIST] = (int32_t)  _FileTransferDirList_msg.size();
    _return[ArdupilotmegaMessageTypes::FILE_TRANSFER_RES] = (int32_t)  _FileTransferRes_msg.size();
    _return[ArdupilotmegaMessageTypes::BATTERY_STATUS] = (int32_t)  _BatteryStatus_msg.size();
    _return[ArdupilotmegaMessageTypes::SETPOINT_8DOF] = (int32_t)  _Setpoint8dof_msg.size();
    _return[ArdupilotmegaMessageTypes::SETPOINT_6DOF] = (int32_t)  _Setpoint6dof_msg.size();
    _return[ArdupilotmegaMessageTypes::MEMORY_VECT] = (int32_t)  _MemoryVect_msg.size();
    _return[ArdupilotmegaMessageTypes::DEBUG_VECT] = (int32_t)  _DebugVect_msg.size();
    _return[ArdupilotmegaMessageTypes::NAMED_VALUE_FLOAT] = (int32_t)  _NamedValueFloat_msg.size();
    _return[ArdupilotmegaMessageTypes::NAMED_VALUE_INT] = (int32_t)  _NamedValueInt_msg.size();
    _return[ArdupilotmegaMessageTypes::STATUSTEXT] = (int32_t)  _Statustext_msg.size();
    _return[ArdupilotmegaMessageTypes::DEBUG] = (int32_t)  _Debug_msg.size();

    _return[ArdupilotmegaMessageTypes::SENSOR_OFFSETS] = (int32_t)  _SensorOffsets_msg.size();
    _return[ArdupilotmegaMessageTypes::SET_MAG_OFFSETS] = (int32_t)  _SetMagOffsets_msg.size();
    _return[ArdupilotmegaMessageTypes::MEMINFO] = (int32_t)  _Meminfo_msg.size();
    _return[ArdupilotmegaMessageTypes::AP_ADC] = (int32_t)  _ApAdc_msg.size();
    _return[ArdupilotmegaMessageTypes::DIGICAM_CONFIGURE] = (int32_t)  _DigicamConfigure_msg.size();
    _return[ArdupilotmegaMessageTypes::DIGICAM_CONTROL] = (int32_t)  _DigicamControl_msg.size();
    _return[ArdupilotmegaMessageTypes::MOUNT_CONFIGURE] = (int32_t)  _MountConfigure_msg.size();
    _return[ArdupilotmegaMessageTypes::MOUNT_CONTROL] = (int32_t)  _MountControl_msg.size();
    _return[ArdupilotmegaMessageTypes::MOUNT_STATUS] = (int32_t)  _MountStatus_msg.size();
    _return[ArdupilotmegaMessageTypes::FENCE_POINT] = (int32_t)  _FencePoint_msg.size();
    _return[ArdupilotmegaMessageTypes::FENCE_FETCH_POINT] = (int32_t)  _FenceFetchPoint_msg.size();
    _return[ArdupilotmegaMessageTypes::FENCE_STATUS] = (int32_t)  _FenceStatus_msg.size();
    _return[ArdupilotmegaMessageTypes::AHRS] = (int32_t)  _Ahrs_msg.size();
    _return[ArdupilotmegaMessageTypes::SIMSTATE] = (int32_t)  _Simstate_msg.size();
    _return[ArdupilotmegaMessageTypes::HWSTATUS] = (int32_t)  _Hwstatus_msg.size();
    _return[ArdupilotmegaMessageTypes::RADIO] = (int32_t)  _Radio_msg.size();
    _return[ArdupilotmegaMessageTypes::LIMITS_STATUS] = (int32_t)  _LimitsStatus_msg.size();
    _return[ArdupilotmegaMessageTypes::WIND] = (int32_t)  _Wind_msg.size();
    _return[ArdupilotmegaMessageTypes::DATA16] = (int32_t)  _Data16_msg.size();
    _return[ArdupilotmegaMessageTypes::DATA32] = (int32_t)  _Data32_msg.size();
    _return[ArdupilotmegaMessageTypes::DATA64] = (int32_t)  _Data64_msg.size();
    _return[ArdupilotmegaMessageTypes::DATA96] = (int32_t)  _Data96_msg.size();
    _return[ArdupilotmegaMessageTypes::RANGEFINDER] = (int32_t)  _Rangefinder_msg.size();
}

/* fetch */

void SimpleFetchServerHandler::fetchHeartbeat(std::vector<Heartbeat> & _return) {
    /* prelude */
    _return = _Heartbeat_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSysStatus(std::vector<SysStatus> & _return) {
    /* prelude */
    _return = _SysStatus_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSystemTime(std::vector<SystemTime> & _return) {
    /* prelude */
    _return = _SystemTime_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchPing(std::vector<Ping> & _return) {
    /* prelude */
    _return = _Ping_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchChangeOperatorControl(std::vector<ChangeOperatorControl> & _return) {
    /* prelude */
    _return = _ChangeOperatorControl_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchChangeOperatorControlAck(std::vector<ChangeOperatorControlAck> & _return) {
    /* prelude */
    _return = _ChangeOperatorControlAck_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchAuthKey(std::vector<AuthKey> & _return) {
    /* prelude */
    _return = _AuthKey_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetMode(std::vector<SetMode> & _return) {
    /* prelude */
    _return = _SetMode_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchParamRequestRead(std::vector<ParamRequestRead> & _return) {
    /* prelude */
    _return = _ParamRequestRead_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchParamRequestList(std::vector<ParamRequestList> & _return) {
    /* prelude */
    _return = _ParamRequestList_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchParamValue(std::vector<ParamValue> & _return) {
    /* prelude */
    _return = _ParamValue_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchParamSet(std::vector<ParamSet> & _return) {
    /* prelude */
    _return = _ParamSet_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchGpsRawInt(std::vector<GpsRawInt> & _return) {
    /* prelude */
    _return = _GpsRawInt_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchGpsStatus(std::vector<GpsStatus> & _return) {
    /* prelude */
    _return = _GpsStatus_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchScaledImu(std::vector<ScaledImu> & _return) {
    /* prelude */
    _return = _ScaledImu_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchRawImu(std::vector<RawImu> & _return) {
    /* prelude */
    _return = _RawImu_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchRawPressure(std::vector<RawPressure> & _return) {
    /* prelude */
    _return = _RawPressure_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchScaledPressure(std::vector<ScaledPressure> & _return) {
    /* prelude */
    _return = _ScaledPressure_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchAttitude(std::vector<Attitude> & _return) {
    /* prelude */
    _return = _Attitude_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchAttitudeQuaternion(std::vector<AttitudeQuaternion> & _return) {
    /* prelude */
    _return = _AttitudeQuaternion_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchLocalPositionNed(std::vector<LocalPositionNed> & _return) {
    /* prelude */
    _return = _LocalPositionNed_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchGlobalPositionInt(std::vector<GlobalPositionInt> & _return) {
    /* prelude */
    _return = _GlobalPositionInt_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchRcChannelsScaled(std::vector<RcChannelsScaled> & _return) {
    /* prelude */
    _return = _RcChannelsScaled_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchRcChannelsRaw(std::vector<RcChannelsRaw> & _return) {
    /* prelude */
    _return = _RcChannelsRaw_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchServoOutputRaw(std::vector<ServoOutputRaw> & _return) {
    /* prelude */
    _return = _ServoOutputRaw_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMissionRequestPartialList(std::vector<MissionRequestPartialList> & _return) {
    /* prelude */
    _return = _MissionRequestPartialList_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMissionWritePartialList(std::vector<MissionWritePartialList> & _return) {
    /* prelude */
    _return = _MissionWritePartialList_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMissionItem(std::vector<MissionItem> & _return) {
    /* prelude */
    _return = _MissionItem_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMissionRequest(std::vector<MissionRequest> & _return) {
    /* prelude */
    _return = _MissionRequest_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMissionSetCurrent(std::vector<MissionSetCurrent> & _return) {
    /* prelude */
    _return = _MissionSetCurrent_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMissionCurrent(std::vector<MissionCurrent> & _return) {
    /* prelude */
    _return = _MissionCurrent_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMissionRequestList(std::vector<MissionRequestList> & _return) {
    /* prelude */
    _return = _MissionRequestList_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMissionCount(std::vector<MissionCount> & _return) {
    /* prelude */
    _return = _MissionCount_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMissionClearAll(std::vector<MissionClearAll> & _return) {
    /* prelude */
    _return = _MissionClearAll_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMissionItemReached(std::vector<MissionItemReached> & _return) {
    /* prelude */
    _return = _MissionItemReached_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMissionAck(std::vector<MissionAck> & _return) {
    /* prelude */
    _return = _MissionAck_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetGpsGlobalOrigin(std::vector<SetGpsGlobalOrigin> & _return) {
    /* prelude */
    _return = _SetGpsGlobalOrigin_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchGpsGlobalOrigin(std::vector<GpsGlobalOrigin> & _return) {
    /* prelude */
    _return = _GpsGlobalOrigin_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetLocalPositionSetpoint(std::vector<SetLocalPositionSetpoint> & _return) {
    /* prelude */
    _return = _SetLocalPositionSetpoint_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchLocalPositionSetpoint(std::vector<LocalPositionSetpoint> & _return) {
    /* prelude */
    _return = _LocalPositionSetpoint_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchGlobalPositionSetpointInt(std::vector<GlobalPositionSetpointInt> & _return) {
    /* prelude */
    _return = _GlobalPositionSetpointInt_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetGlobalPositionSetpointInt(std::vector<SetGlobalPositionSetpointInt> & _return) {
    /* prelude */
    _return = _SetGlobalPositionSetpointInt_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSafetySetAllowedArea(std::vector<SafetySetAllowedArea> & _return) {
    /* prelude */
    _return = _SafetySetAllowedArea_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSafetyAllowedArea(std::vector<SafetyAllowedArea> & _return) {
    /* prelude */
    _return = _SafetyAllowedArea_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetRollPitchYawThrust(std::vector<SetRollPitchYawThrust> & _return) {
    /* prelude */
    _return = _SetRollPitchYawThrust_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetRollPitchYawSpeedThrust(std::vector<SetRollPitchYawSpeedThrust> & _return) {
    /* prelude */
    _return = _SetRollPitchYawSpeedThrust_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchRollPitchYawThrustSetpoint(std::vector<RollPitchYawThrustSetpoint> & _return) {
    /* prelude */
    _return = _RollPitchYawThrustSetpoint_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchRollPitchYawSpeedThrustSetpoint(std::vector<RollPitchYawSpeedThrustSetpoint> & _return) {
    /* prelude */
    _return = _RollPitchYawSpeedThrustSetpoint_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetQuadMotorsSetpoint(std::vector<SetQuadMotorsSetpoint> & _return) {
    /* prelude */
    _return = _SetQuadMotorsSetpoint_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetQuadSwarmRollPitchYawThrust(std::vector<SetQuadSwarmRollPitchYawThrust> & _return) {
    /* prelude */
    _return = _SetQuadSwarmRollPitchYawThrust_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchNavControllerOutput(std::vector<NavControllerOutput> & _return) {
    /* prelude */
    _return = _NavControllerOutput_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetQuadSwarmLedRollPitchYawThrust(std::vector<SetQuadSwarmLedRollPitchYawThrust> & _return) {
    /* prelude */
    _return = _SetQuadSwarmLedRollPitchYawThrust_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchStateCorrection(std::vector<StateCorrection> & _return) {
    /* prelude */
    _return = _StateCorrection_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchRequestDataStream(std::vector<RequestDataStream> & _return) {
    /* prelude */
    _return = _RequestDataStream_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchDataStream(std::vector<DataStream> & _return) {
    /* prelude */
    _return = _DataStream_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchManualControl(std::vector<ManualControl> & _return) {
    /* prelude */
    _return = _ManualControl_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchRcChannelsOverride(std::vector<RcChannelsOverride> & _return) {
    /* prelude */
    _return = _RcChannelsOverride_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchVfrHud(std::vector<VfrHud> & _return) {
    /* prelude */
    _return = _VfrHud_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchCommandLong(std::vector<CommandLong> & _return) {
    /* prelude */
    _return = _CommandLong_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchCommandAck(std::vector<CommandAck> & _return) {
    /* prelude */
    _return = _CommandAck_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchRollPitchYawRatesThrustSetpoint(std::vector<RollPitchYawRatesThrustSetpoint> & _return) {
    /* prelude */
    _return = _RollPitchYawRatesThrustSetpoint_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchManualSetpoint(std::vector<ManualSetpoint> & _return) {
    /* prelude */
    _return = _ManualSetpoint_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchLocalPositionNedSystemGlobalOffset(std::vector<LocalPositionNedSystemGlobalOffset> & _return) {
    /* prelude */
    _return = _LocalPositionNedSystemGlobalOffset_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchHilState(std::vector<HilState> & _return) {
    /* prelude */
    _return = _HilState_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchHilControls(std::vector<HilControls> & _return) {
    /* prelude */
    _return = _HilControls_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchHilRcInputsRaw(std::vector<HilRcInputsRaw> & _return) {
    /* prelude */
    _return = _HilRcInputsRaw_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchOpticalFlow(std::vector<OpticalFlow> & _return) {
    /* prelude */
    _return = _OpticalFlow_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchGlobalVisionPositionEstimate(std::vector<GlobalVisionPositionEstimate> & _return) {
    /* prelude */
    _return = _GlobalVisionPositionEstimate_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchVisionPositionEstimate(std::vector<VisionPositionEstimate> & _return) {
    /* prelude */
    _return = _VisionPositionEstimate_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchVisionSpeedEstimate(std::vector<VisionSpeedEstimate> & _return) {
    /* prelude */
    _return = _VisionSpeedEstimate_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchViconPositionEstimate(std::vector<ViconPositionEstimate> & _return) {
    /* prelude */
    _return = _ViconPositionEstimate_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchHighresImu(std::vector<HighresImu> & _return) {
    /* prelude */
    _return = _HighresImu_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchOmnidirectionalFlow(std::vector<OmnidirectionalFlow> & _return) {
    /* prelude */
    _return = _OmnidirectionalFlow_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchFileTransferStart(std::vector<FileTransferStart> & _return) {
    /* prelude */
    _return = _FileTransferStart_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchFileTransferDirList(std::vector<FileTransferDirList> & _return) {
    /* prelude */
    _return = _FileTransferDirList_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchFileTransferRes(std::vector<FileTransferRes> & _return) {
    /* prelude */
    _return = _FileTransferRes_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchBatteryStatus(std::vector<BatteryStatus> & _return) {
    /* prelude */
    _return = _BatteryStatus_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetpoint8dof(std::vector<Setpoint8dof> & _return) {
    /* prelude */
    _return = _Setpoint8dof_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetpoint6dof(std::vector<Setpoint6dof> & _return) {
    /* prelude */
    _return = _Setpoint6dof_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMemoryVect(std::vector<MemoryVect> & _return) {
    /* prelude */
    _return = _MemoryVect_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchDebugVect(std::vector<DebugVect> & _return) {
    /* prelude */
    _return = _DebugVect_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchNamedValueFloat(std::vector<NamedValueFloat> & _return) {
    /* prelude */
    _return = _NamedValueFloat_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchNamedValueInt(std::vector<NamedValueInt> & _return) {
    /* prelude */
    _return = _NamedValueInt_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchStatustext(std::vector<Statustext> & _return) {
    /* prelude */
    _return = _Statustext_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchDebug(std::vector<Debug> & _return) {
    /* prelude */
    _return = _Debug_msg;
    /* postlude */
}

void SimpleFetchServerHandler::fetchSensorOffsets(std::vector<SensorOffsets> & _return) {
    /* prelude */
    _return = _SensorOffsets_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSetMagOffsets(std::vector<SetMagOffsets> & _return) {
    /* prelude */
    _return = _SetMagOffsets_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMeminfo(std::vector<Meminfo> & _return) {
    /* prelude */
    _return = _Meminfo_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchApAdc(std::vector<ApAdc> & _return) {
    /* prelude */
    _return = _ApAdc_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchDigicamConfigure(std::vector<DigicamConfigure> & _return) {
    /* prelude */
    _return = _DigicamConfigure_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchDigicamControl(std::vector<DigicamControl> & _return) {
    /* prelude */
    _return = _DigicamControl_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMountConfigure(std::vector<MountConfigure> & _return) {
    /* prelude */
    _return = _MountConfigure_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMountControl(std::vector<MountControl> & _return) {
    /* prelude */
    _return = _MountControl_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchMountStatus(std::vector<MountStatus> & _return) {
    /* prelude */
    _return = _MountStatus_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchFencePoint(std::vector<FencePoint> & _return) {
    /* prelude */
    _return = _FencePoint_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchFenceFetchPoint(std::vector<FenceFetchPoint> & _return) {
    /* prelude */
    _return = _FenceFetchPoint_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchFenceStatus(std::vector<FenceStatus> & _return) {
    /* prelude */
    _return = _FenceStatus_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchAhrs(std::vector<Ahrs> & _return) {
    /* prelude */
    _return = _Ahrs_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchSimstate(std::vector<Simstate> & _return) {
    /* prelude */
    _return = _Simstate_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchHwstatus(std::vector<Hwstatus> & _return) {
    /* prelude */
    _return = _Hwstatus_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchRadio(std::vector<Radio> & _return) {
    /* prelude */
    _return = _Radio_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchLimitsStatus(std::vector<LimitsStatus> & _return) {
    /* prelude */
    _return = _LimitsStatus_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchWind(std::vector<Wind> & _return) {
    /* prelude */
    _return = _Wind_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchData16(std::vector<Data16> & _return) {
    /* prelude */
    _return = _Data16_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchData32(std::vector<Data32> & _return) {
    /* prelude */
    _return = _Data32_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchData64(std::vector<Data64> & _return) {
    /* prelude */
    _return = _Data64_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchData96(std::vector<Data96> & _return) {
    /* prelude */
    _return = _Data96_msg;
    /* postlude */
}
void SimpleFetchServerHandler::fetchRangefinder(std::vector<Rangefinder> & _return) {
    /* prelude */
    _return = _Rangefinder_msg;
    /* postlude */
}

/* Post*/

void SimpleFetchServerHandler::postHeartbeat(const Heartbeat & msg) {
    /* prelude */
    _Heartbeat_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSysStatus(const SysStatus & msg) {
    /* prelude */
    _SysStatus_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSystemTime(const SystemTime & msg) {
    /* prelude */
    _SystemTime_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postPing(const Ping & msg) {
    /* prelude */
    _Ping_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postChangeOperatorControl(const ChangeOperatorControl & msg) {
    /* prelude */
    _ChangeOperatorControl_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postChangeOperatorControlAck(const ChangeOperatorControlAck & msg) {
    /* prelude */
    _ChangeOperatorControlAck_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postAuthKey(const AuthKey & msg) {
    /* prelude */
    _AuthKey_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetMode(const SetMode & msg) {
    /* prelude */
    _SetMode_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postParamRequestRead(const ParamRequestRead & msg) {
    /* prelude */
    _ParamRequestRead_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postParamRequestList(const ParamRequestList & msg) {
    /* prelude */
    _ParamRequestList_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postParamValue(const ParamValue & msg) {
    /* prelude */
    _ParamValue_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postParamSet(const ParamSet & msg) {
    /* prelude */
    _ParamSet_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postGpsRawInt(const GpsRawInt & msg) {
    /* prelude */
    _GpsRawInt_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postGpsStatus(const GpsStatus & msg) {
    /* prelude */
    _GpsStatus_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postScaledImu(const ScaledImu & msg) {
    /* prelude */
    _ScaledImu_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postRawImu(const RawImu & msg) {
    /* prelude */
    _RawImu_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postRawPressure(const RawPressure & msg) {
    /* prelude */
    _RawPressure_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postScaledPressure(const ScaledPressure & msg) {
    /* prelude */
    _ScaledPressure_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postAttitude(const Attitude & msg) {
    /* prelude */
    _Attitude_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postAttitudeQuaternion(const AttitudeQuaternion & msg) {
    /* prelude */
    _AttitudeQuaternion_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postLocalPositionNed(const LocalPositionNed & msg) {
    /* prelude */
    _LocalPositionNed_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postGlobalPositionInt(const GlobalPositionInt & msg) {
    /* prelude */
    _GlobalPositionInt_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postRcChannelsScaled(const RcChannelsScaled & msg) {
    /* prelude */
    _RcChannelsScaled_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postRcChannelsRaw(const RcChannelsRaw & msg) {
    /* prelude */
    _RcChannelsRaw_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postServoOutputRaw(const ServoOutputRaw & msg) {
    /* prelude */
    _ServoOutputRaw_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMissionRequestPartialList(const MissionRequestPartialList & msg) {
    /* prelude */
    _MissionRequestPartialList_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMissionWritePartialList(const MissionWritePartialList & msg) {
    /* prelude */
    _MissionWritePartialList_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMissionItem(const MissionItem & msg) {
    /* prelude */
    _MissionItem_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMissionRequest(const MissionRequest & msg) {
    /* prelude */
    _MissionRequest_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMissionSetCurrent(const MissionSetCurrent & msg) {
    /* prelude */
    _MissionSetCurrent_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMissionCurrent(const MissionCurrent & msg) {
    /* prelude */
    _MissionCurrent_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMissionRequestList(const MissionRequestList & msg) {
    /* prelude */
    _MissionRequestList_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMissionCount(const MissionCount & msg) {
    /* prelude */
    _MissionCount_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMissionClearAll(const MissionClearAll & msg) {
    /* prelude */
    _MissionClearAll_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMissionItemReached(const MissionItemReached & msg) {
    /* prelude */
    _MissionItemReached_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMissionAck(const MissionAck & msg) {
    /* prelude */
    _MissionAck_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetGpsGlobalOrigin(const SetGpsGlobalOrigin & msg) {
    /* prelude */
    _SetGpsGlobalOrigin_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postGpsGlobalOrigin(const GpsGlobalOrigin & msg) {
    /* prelude */
    _GpsGlobalOrigin_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetLocalPositionSetpoint(const SetLocalPositionSetpoint & msg) {
    /* prelude */
    _SetLocalPositionSetpoint_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postLocalPositionSetpoint(const LocalPositionSetpoint & msg) {
    /* prelude */
    _LocalPositionSetpoint_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postGlobalPositionSetpointInt(const GlobalPositionSetpointInt & msg) {
    /* prelude */
    _GlobalPositionSetpointInt_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetGlobalPositionSetpointInt(const SetGlobalPositionSetpointInt & msg) {
    /* prelude */
    _SetGlobalPositionSetpointInt_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSafetySetAllowedArea(const SafetySetAllowedArea & msg) {
    /* prelude */
    _SafetySetAllowedArea_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSafetyAllowedArea(const SafetyAllowedArea & msg) {
    /* prelude */
    _SafetyAllowedArea_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetRollPitchYawThrust(const SetRollPitchYawThrust & msg) {
    /* prelude */
    _SetRollPitchYawThrust_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetRollPitchYawSpeedThrust(const SetRollPitchYawSpeedThrust & msg) {
    /* prelude */
    _SetRollPitchYawSpeedThrust_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postRollPitchYawThrustSetpoint(const RollPitchYawThrustSetpoint & msg) {
    /* prelude */
    _RollPitchYawThrustSetpoint_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postRollPitchYawSpeedThrustSetpoint(const RollPitchYawSpeedThrustSetpoint & msg) {
    /* prelude */
    _RollPitchYawSpeedThrustSetpoint_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetQuadMotorsSetpoint(const SetQuadMotorsSetpoint & msg) {
    /* prelude */
    _SetQuadMotorsSetpoint_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetQuadSwarmRollPitchYawThrust(const SetQuadSwarmRollPitchYawThrust & msg) {
    /* prelude */
    _SetQuadSwarmRollPitchYawThrust_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postNavControllerOutput(const NavControllerOutput & msg) {
    /* prelude */
    _NavControllerOutput_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetQuadSwarmLedRollPitchYawThrust(const SetQuadSwarmLedRollPitchYawThrust & msg) {
    /* prelude */
    _SetQuadSwarmLedRollPitchYawThrust_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postStateCorrection(const StateCorrection & msg) {
    /* prelude */
    _StateCorrection_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postRequestDataStream(const RequestDataStream & msg) {
    /* prelude */
    _RequestDataStream_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postDataStream(const DataStream & msg) {
    /* prelude */
    _DataStream_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postManualControl(const ManualControl & msg) {
    /* prelude */
    _ManualControl_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postRcChannelsOverride(const RcChannelsOverride & msg) {
    /* prelude */
    _RcChannelsOverride_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postVfrHud(const VfrHud & msg) {
    /* prelude */
    _VfrHud_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postCommandLong(const CommandLong & msg) {
    /* prelude */
    _CommandLong_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postCommandAck(const CommandAck & msg) {
    /* prelude */
    _CommandAck_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postRollPitchYawRatesThrustSetpoint(const RollPitchYawRatesThrustSetpoint & msg) {
    /* prelude */
    _RollPitchYawRatesThrustSetpoint_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postManualSetpoint(const ManualSetpoint & msg) {
    /* prelude */
    _ManualSetpoint_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postLocalPositionNedSystemGlobalOffset(const LocalPositionNedSystemGlobalOffset & msg) {
    /* prelude */
    _LocalPositionNedSystemGlobalOffset_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postHilState(const HilState & msg) {
    /* prelude */
    _HilState_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postHilControls(const HilControls & msg) {
    /* prelude */
    _HilControls_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postHilRcInputsRaw(const HilRcInputsRaw & msg) {
    /* prelude */
    _HilRcInputsRaw_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postOpticalFlow(const OpticalFlow & msg) {
    /* prelude */
    _OpticalFlow_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postGlobalVisionPositionEstimate(const GlobalVisionPositionEstimate & msg) {
    /* prelude */
    _GlobalVisionPositionEstimate_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postVisionPositionEstimate(const VisionPositionEstimate & msg) {
    /* prelude */
    _VisionPositionEstimate_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postVisionSpeedEstimate(const VisionSpeedEstimate & msg) {
    /* prelude */
    _VisionSpeedEstimate_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postViconPositionEstimate(const ViconPositionEstimate & msg) {
    /* prelude */
    _ViconPositionEstimate_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postHighresImu(const HighresImu & msg) {
    /* prelude */
    _HighresImu_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postOmnidirectionalFlow(const OmnidirectionalFlow & msg) {
    /* prelude */
    _OmnidirectionalFlow_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postFileTransferStart(const FileTransferStart & msg) {
    /* prelude */
    _FileTransferStart_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postFileTransferDirList(const FileTransferDirList & msg) {
    /* prelude */
    _FileTransferDirList_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postFileTransferRes(const FileTransferRes & msg) {
    /* prelude */
    _FileTransferRes_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postBatteryStatus(const BatteryStatus & msg) {
    /* prelude */
    _BatteryStatus_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetpoint8dof(const Setpoint8dof & msg) {
    /* prelude */
    _Setpoint8dof_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetpoint6dof(const Setpoint6dof & msg) {
    /* prelude */
    _Setpoint6dof_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMemoryVect(const MemoryVect & msg) {
    /* prelude */
    _MemoryVect_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postDebugVect(const DebugVect & msg) {
    /* prelude */
    _DebugVect_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postNamedValueFloat(const NamedValueFloat & msg) {
    /* prelude */
    _NamedValueFloat_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postNamedValueInt(const NamedValueInt & msg) {
    /* prelude */
    _NamedValueInt_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postStatustext(const Statustext & msg) {
    /* prelude */
    _Statustext_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postDebug(const Debug & msg) {
    /* prelude */
    _Debug_msg.push_back(msg);
    /* postlude */
}

void SimpleFetchServerHandler::postSensorOffsets(const SensorOffsets & msg) {
    /* prelude */
    _SensorOffsets_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSetMagOffsets(const SetMagOffsets & msg) {
    /* prelude */
    _SetMagOffsets_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMeminfo(const Meminfo & msg) {
    /* prelude */
    _Meminfo_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postApAdc(const ApAdc & msg) {
    /* prelude */
    _ApAdc_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postDigicamConfigure(const DigicamConfigure & msg) {
    /* prelude */
    _DigicamConfigure_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postDigicamControl(const DigicamControl & msg) {
    /* prelude */
    _DigicamControl_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMountConfigure(const MountConfigure & msg) {
    /* prelude */
    _MountConfigure_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMountControl(const MountControl & msg) {
    /* prelude */
    _MountControl_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postMountStatus(const MountStatus & msg) {
    /* prelude */
    _MountStatus_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postFencePoint(const FencePoint & msg) {
    /* prelude */
    _FencePoint_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postFenceFetchPoint(const FenceFetchPoint & msg) {
    /* prelude */
    _FenceFetchPoint_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postFenceStatus(const FenceStatus & msg) {
    /* prelude */
    _FenceStatus_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postAhrs(const Ahrs & msg) {
    /* prelude */
    _Ahrs_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postSimstate(const Simstate & msg) {
    /* prelude */
    _Simstate_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postHwstatus(const Hwstatus & msg) {
    /* prelude */
    _Hwstatus_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postRadio(const Radio & msg) {
    /* prelude */
    _Radio_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postLimitsStatus(const LimitsStatus & msg) {
    /* prelude */
    _LimitsStatus_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postWind(const Wind & msg) {
    /* prelude */
    _Wind_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postData16(const Data16 & msg) {
    /* prelude */
    _Data16_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postData32(const Data32 & msg) {
    /* prelude */
    _Data32_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postData64(const Data64 & msg) {
    /* prelude */
    _Data64_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postData96(const Data96 & msg) {
    /* prelude */
    _Data96_msg.push_back(msg);
    /* postlude */
}
void SimpleFetchServerHandler::postRangefinder(const Rangefinder & msg) {
    /* prelude */
    _Rangefinder_msg.push_back(msg);
    /* postlude */
}

