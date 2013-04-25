
#ifndef SimpleFetchServer_H
#define SimpleFetchServer_H

#include "ArdupilotmegaMessageFetch.h"

namespace mavlink { namespace thrift {

class SimpleFetchServerHandler : virtual public ArdupilotmegaMessageFetchIf {
  public:
    SimpleFetchServerHandler();
    void availableMessages(std::map<CommonMessageTypes::type, int32_t> & _return);
    void availableMessages(std::map<ArdupilotmegaMessageTypes::type, int32_t> & _return);

    void fetchHeartbeat(std::vector<Heartbeat> & _return);
    void fetchSysStatus(std::vector<SysStatus> & _return);
    void fetchSystemTime(std::vector<SystemTime> & _return);
    void fetchPing(std::vector<Ping> & _return);
    void fetchChangeOperatorControl(std::vector<ChangeOperatorControl> & _return);
    void fetchChangeOperatorControlAck(std::vector<ChangeOperatorControlAck> & _return);
    void fetchAuthKey(std::vector<AuthKey> & _return);
    void fetchSetMode(std::vector<SetMode> & _return);
    void fetchParamRequestRead(std::vector<ParamRequestRead> & _return);
    void fetchParamRequestList(std::vector<ParamRequestList> & _return);
    void fetchParamValue(std::vector<ParamValue> & _return);
    void fetchParamSet(std::vector<ParamSet> & _return);
    void fetchGpsRawInt(std::vector<GpsRawInt> & _return);
    void fetchGpsStatus(std::vector<GpsStatus> & _return);
    void fetchScaledImu(std::vector<ScaledImu> & _return);
    void fetchRawImu(std::vector<RawImu> & _return);
    void fetchRawPressure(std::vector<RawPressure> & _return);
    void fetchScaledPressure(std::vector<ScaledPressure> & _return);
    void fetchAttitude(std::vector<Attitude> & _return);
    void fetchAttitudeQuaternion(std::vector<AttitudeQuaternion> & _return);
    void fetchLocalPositionNed(std::vector<LocalPositionNed> & _return);
    void fetchGlobalPositionInt(std::vector<GlobalPositionInt> & _return);
    void fetchRcChannelsScaled(std::vector<RcChannelsScaled> & _return);
    void fetchRcChannelsRaw(std::vector<RcChannelsRaw> & _return);
    void fetchServoOutputRaw(std::vector<ServoOutputRaw> & _return);
    void fetchMissionRequestPartialList(std::vector<MissionRequestPartialList> & _return);
    void fetchMissionWritePartialList(std::vector<MissionWritePartialList> & _return);
    void fetchMissionItem(std::vector<MissionItem> & _return);
    void fetchMissionRequest(std::vector<MissionRequest> & _return);
    void fetchMissionSetCurrent(std::vector<MissionSetCurrent> & _return);
    void fetchMissionCurrent(std::vector<MissionCurrent> & _return);
    void fetchMissionRequestList(std::vector<MissionRequestList> & _return);
    void fetchMissionCount(std::vector<MissionCount> & _return);
    void fetchMissionClearAll(std::vector<MissionClearAll> & _return);
    void fetchMissionItemReached(std::vector<MissionItemReached> & _return);
    void fetchMissionAck(std::vector<MissionAck> & _return);
    void fetchSetGpsGlobalOrigin(std::vector<SetGpsGlobalOrigin> & _return);
    void fetchGpsGlobalOrigin(std::vector<GpsGlobalOrigin> & _return);
    void fetchSetLocalPositionSetpoint(std::vector<SetLocalPositionSetpoint> & _return);
    void fetchLocalPositionSetpoint(std::vector<LocalPositionSetpoint> & _return);
    void fetchGlobalPositionSetpointInt(std::vector<GlobalPositionSetpointInt> & _return);
    void fetchSetGlobalPositionSetpointInt(std::vector<SetGlobalPositionSetpointInt> & _return);
    void fetchSafetySetAllowedArea(std::vector<SafetySetAllowedArea> & _return);
    void fetchSafetyAllowedArea(std::vector<SafetyAllowedArea> & _return);
    void fetchSetRollPitchYawThrust(std::vector<SetRollPitchYawThrust> & _return);
    void fetchSetRollPitchYawSpeedThrust(std::vector<SetRollPitchYawSpeedThrust> & _return);
    void fetchRollPitchYawThrustSetpoint(std::vector<RollPitchYawThrustSetpoint> & _return);
    void fetchRollPitchYawSpeedThrustSetpoint(std::vector<RollPitchYawSpeedThrustSetpoint> & _return);
    void fetchSetQuadMotorsSetpoint(std::vector<SetQuadMotorsSetpoint> & _return);
    void fetchSetQuadSwarmRollPitchYawThrust(std::vector<SetQuadSwarmRollPitchYawThrust> & _return);
    void fetchNavControllerOutput(std::vector<NavControllerOutput> & _return);
    void fetchSetQuadSwarmLedRollPitchYawThrust(std::vector<SetQuadSwarmLedRollPitchYawThrust> & _return);
    void fetchStateCorrection(std::vector<StateCorrection> & _return);
    void fetchRequestDataStream(std::vector<RequestDataStream> & _return);
    void fetchDataStream(std::vector<DataStream> & _return);
    void fetchManualControl(std::vector<ManualControl> & _return);
    void fetchRcChannelsOverride(std::vector<RcChannelsOverride> & _return);
    void fetchVfrHud(std::vector<VfrHud> & _return);
    void fetchCommandLong(std::vector<CommandLong> & _return);
    void fetchCommandAck(std::vector<CommandAck> & _return);
    void fetchRollPitchYawRatesThrustSetpoint(std::vector<RollPitchYawRatesThrustSetpoint> & _return);
    void fetchManualSetpoint(std::vector<ManualSetpoint> & _return);
    void fetchLocalPositionNedSystemGlobalOffset(std::vector<LocalPositionNedSystemGlobalOffset> & _return);
    void fetchHilState(std::vector<HilState> & _return);
    void fetchHilControls(std::vector<HilControls> & _return);
    void fetchHilRcInputsRaw(std::vector<HilRcInputsRaw> & _return);
    void fetchOpticalFlow(std::vector<OpticalFlow> & _return);
    void fetchGlobalVisionPositionEstimate(std::vector<GlobalVisionPositionEstimate> & _return);
    void fetchVisionPositionEstimate(std::vector<VisionPositionEstimate> & _return);
    void fetchVisionSpeedEstimate(std::vector<VisionSpeedEstimate> & _return);
    void fetchViconPositionEstimate(std::vector<ViconPositionEstimate> & _return);
    void fetchHighresImu(std::vector<HighresImu> & _return);
    void fetchOmnidirectionalFlow(std::vector<OmnidirectionalFlow> & _return);
    void fetchFileTransferStart(std::vector<FileTransferStart> & _return);
    void fetchFileTransferDirList(std::vector<FileTransferDirList> & _return);
    void fetchFileTransferRes(std::vector<FileTransferRes> & _return);
    void fetchBatteryStatus(std::vector<BatteryStatus> & _return);
    void fetchSetpoint8dof(std::vector<Setpoint8dof> & _return);
    void fetchSetpoint6dof(std::vector<Setpoint6dof> & _return);
    void fetchMemoryVect(std::vector<MemoryVect> & _return);
    void fetchDebugVect(std::vector<DebugVect> & _return);
    void fetchNamedValueFloat(std::vector<NamedValueFloat> & _return);
    void fetchNamedValueInt(std::vector<NamedValueInt> & _return);
    void fetchStatustext(std::vector<Statustext> & _return);
    void fetchDebug(std::vector<Debug> & _return);

    void fetchSensorOffsets(std::vector<SensorOffsets> & _return);
    void fetchSetMagOffsets(std::vector<SetMagOffsets> & _return);
    void fetchMeminfo(std::vector<Meminfo> & _return);
    void fetchApAdc(std::vector<ApAdc> & _return);
    void fetchDigicamConfigure(std::vector<DigicamConfigure> & _return);
    void fetchDigicamControl(std::vector<DigicamControl> & _return);
    void fetchMountConfigure(std::vector<MountConfigure> & _return);
    void fetchMountControl(std::vector<MountControl> & _return);
    void fetchMountStatus(std::vector<MountStatus> & _return);
    void fetchFencePoint(std::vector<FencePoint> & _return);
    void fetchFenceFetchPoint(std::vector<FenceFetchPoint> & _return);
    void fetchFenceStatus(std::vector<FenceStatus> & _return);
    void fetchAhrs(std::vector<Ahrs> & _return);
    void fetchSimstate(std::vector<Simstate> & _return);
    void fetchHwstatus(std::vector<Hwstatus> & _return);
    void fetchRadio(std::vector<Radio> & _return);
    void fetchLimitsStatus(std::vector<LimitsStatus> & _return);
    void fetchWind(std::vector<Wind> & _return);
    void fetchData16(std::vector<Data16> & _return);
    void fetchData32(std::vector<Data32> & _return);
    void fetchData64(std::vector<Data64> & _return);
    void fetchData96(std::vector<Data96> & _return);
    void fetchRangefinder(std::vector<Rangefinder> & _return);

    /* Post: */

    void postHeartbeat(const Heartbeat & msg);
    void postSysStatus(const SysStatus & msg);
    void postSystemTime(const SystemTime & msg);
    void postPing(const Ping & msg);
    void postChangeOperatorControl(const ChangeOperatorControl & msg);
    void postChangeOperatorControlAck(const ChangeOperatorControlAck & msg);
    void postAuthKey(const AuthKey & msg);
    void postSetMode(const SetMode & msg);
    void postParamRequestRead(const ParamRequestRead & msg);
    void postParamRequestList(const ParamRequestList & msg);
    void postParamValue(const ParamValue & msg);
    void postParamSet(const ParamSet & msg);
    void postGpsRawInt(const GpsRawInt & msg);
    void postGpsStatus(const GpsStatus & msg);
    void postScaledImu(const ScaledImu & msg);
    void postRawImu(const RawImu & msg);
    void postRawPressure(const RawPressure & msg);
    void postScaledPressure(const ScaledPressure & msg);
    void postAttitude(const Attitude & msg);
    void postAttitudeQuaternion(const AttitudeQuaternion & msg);
    void postLocalPositionNed(const LocalPositionNed & msg);
    void postGlobalPositionInt(const GlobalPositionInt & msg);
    void postRcChannelsScaled(const RcChannelsScaled & msg);
    void postRcChannelsRaw(const RcChannelsRaw & msg);
    void postServoOutputRaw(const ServoOutputRaw & msg);
    void postMissionRequestPartialList(const MissionRequestPartialList & msg);
    void postMissionWritePartialList(const MissionWritePartialList & msg);
    void postMissionItem(const MissionItem & msg);
    void postMissionRequest(const MissionRequest & msg);
    void postMissionSetCurrent(const MissionSetCurrent & msg);
    void postMissionCurrent(const MissionCurrent & msg);
    void postMissionRequestList(const MissionRequestList & msg);
    void postMissionCount(const MissionCount & msg);
    void postMissionClearAll(const MissionClearAll & msg);
    void postMissionItemReached(const MissionItemReached & msg);
    void postMissionAck(const MissionAck & msg);
    void postSetGpsGlobalOrigin(const SetGpsGlobalOrigin & msg);
    void postGpsGlobalOrigin(const GpsGlobalOrigin & msg);
    void postSetLocalPositionSetpoint(const SetLocalPositionSetpoint & msg);
    void postLocalPositionSetpoint(const LocalPositionSetpoint & msg);
    void postGlobalPositionSetpointInt(const GlobalPositionSetpointInt & msg);
    void postSetGlobalPositionSetpointInt(const SetGlobalPositionSetpointInt & msg);
    void postSafetySetAllowedArea(const SafetySetAllowedArea & msg);
    void postSafetyAllowedArea(const SafetyAllowedArea & msg);
    void postSetRollPitchYawThrust(const SetRollPitchYawThrust & msg);
    void postSetRollPitchYawSpeedThrust(const SetRollPitchYawSpeedThrust & msg);
    void postRollPitchYawThrustSetpoint(const RollPitchYawThrustSetpoint & msg);
    void postRollPitchYawSpeedThrustSetpoint(const RollPitchYawSpeedThrustSetpoint & msg);
    void postSetQuadMotorsSetpoint(const SetQuadMotorsSetpoint & msg);
    void postSetQuadSwarmRollPitchYawThrust(const SetQuadSwarmRollPitchYawThrust & msg);
    void postNavControllerOutput(const NavControllerOutput & msg);
    void postSetQuadSwarmLedRollPitchYawThrust(const SetQuadSwarmLedRollPitchYawThrust & msg);
    void postStateCorrection(const StateCorrection & msg);
    void postRequestDataStream(const RequestDataStream & msg);
    void postDataStream(const DataStream & msg);
    void postManualControl(const ManualControl & msg);
    void postRcChannelsOverride(const RcChannelsOverride & msg);
    void postVfrHud(const VfrHud & msg);
    void postCommandLong(const CommandLong & msg);
    void postCommandAck(const CommandAck & msg);
    void postRollPitchYawRatesThrustSetpoint(const RollPitchYawRatesThrustSetpoint & msg);
    void postManualSetpoint(const ManualSetpoint & msg);
    void postLocalPositionNedSystemGlobalOffset(const LocalPositionNedSystemGlobalOffset & msg);
    void postHilState(const HilState & msg);
    void postHilControls(const HilControls & msg);
    void postHilRcInputsRaw(const HilRcInputsRaw & msg);
    void postOpticalFlow(const OpticalFlow & msg);
    void postGlobalVisionPositionEstimate(const GlobalVisionPositionEstimate & msg);
    void postVisionPositionEstimate(const VisionPositionEstimate & msg);
    void postVisionSpeedEstimate(const VisionSpeedEstimate & msg);
    void postViconPositionEstimate(const ViconPositionEstimate & msg);
    void postHighresImu(const HighresImu & msg);
    void postOmnidirectionalFlow(const OmnidirectionalFlow & msg);
    void postFileTransferStart(const FileTransferStart & msg);
    void postFileTransferDirList(const FileTransferDirList & msg);
    void postFileTransferRes(const FileTransferRes & msg);
    void postBatteryStatus(const BatteryStatus & msg);
    void postSetpoint8dof(const Setpoint8dof & msg);
    void postSetpoint6dof(const Setpoint6dof & msg);
    void postMemoryVect(const MemoryVect & msg);
    void postDebugVect(const DebugVect & msg);
    void postNamedValueFloat(const NamedValueFloat & msg);
    void postNamedValueInt(const NamedValueInt & msg);
    void postStatustext(const Statustext & msg);
    void postDebug(const Debug & msg);

    void postSensorOffsets(const SensorOffsets & msg);
    void postSetMagOffsets(const SetMagOffsets & msg);
    void postMeminfo(const Meminfo & msg);
    void postApAdc(const ApAdc & msg);
    void postDigicamConfigure(const DigicamConfigure & msg);
    void postDigicamControl(const DigicamControl & msg);
    void postMountConfigure(const MountConfigure & msg);
    void postMountControl(const MountControl & msg);
    void postMountStatus(const MountStatus & msg);
    void postFencePoint(const FencePoint & msg);
    void postFenceFetchPoint(const FenceFetchPoint & msg);
    void postFenceStatus(const FenceStatus & msg);
    void postAhrs(const Ahrs & msg);
    void postSimstate(const Simstate & msg);
    void postHwstatus(const Hwstatus & msg);
    void postRadio(const Radio & msg);
    void postLimitsStatus(const LimitsStatus & msg);
    void postWind(const Wind & msg);
    void postData16(const Data16 & msg);
    void postData32(const Data32 & msg);
    void postData64(const Data64 & msg);
    void postData96(const Data96 & msg);
    void postRangefinder(const Rangefinder & msg);

  private:
    std::vector<Heartbeat> _Heartbeat_msg;
    std::vector<SysStatus> _SysStatus_msg;
    std::vector<SystemTime> _SystemTime_msg;
    std::vector<Ping> _Ping_msg;
    std::vector<ChangeOperatorControl> _ChangeOperatorControl_msg;
    std::vector<ChangeOperatorControlAck> _ChangeOperatorControlAck_msg;
    std::vector<AuthKey> _AuthKey_msg;
    std::vector<SetMode> _SetMode_msg;
    std::vector<ParamRequestRead> _ParamRequestRead_msg;
    std::vector<ParamRequestList> _ParamRequestList_msg;
    std::vector<ParamValue> _ParamValue_msg;
    std::vector<ParamSet> _ParamSet_msg;
    std::vector<GpsRawInt> _GpsRawInt_msg;
    std::vector<GpsStatus> _GpsStatus_msg;
    std::vector<ScaledImu> _ScaledImu_msg;
    std::vector<RawImu> _RawImu_msg;
    std::vector<RawPressure> _RawPressure_msg;
    std::vector<ScaledPressure> _ScaledPressure_msg;
    std::vector<Attitude> _Attitude_msg;
    std::vector<AttitudeQuaternion> _AttitudeQuaternion_msg;
    std::vector<LocalPositionNed> _LocalPositionNed_msg;
    std::vector<GlobalPositionInt> _GlobalPositionInt_msg;
    std::vector<RcChannelsScaled> _RcChannelsScaled_msg;
    std::vector<RcChannelsRaw> _RcChannelsRaw_msg;
    std::vector<ServoOutputRaw> _ServoOutputRaw_msg;
    std::vector<MissionRequestPartialList> _MissionRequestPartialList_msg;
    std::vector<MissionWritePartialList> _MissionWritePartialList_msg;
    std::vector<MissionItem> _MissionItem_msg;
    std::vector<MissionRequest> _MissionRequest_msg;
    std::vector<MissionSetCurrent> _MissionSetCurrent_msg;
    std::vector<MissionCurrent> _MissionCurrent_msg;
    std::vector<MissionRequestList> _MissionRequestList_msg;
    std::vector<MissionCount> _MissionCount_msg;
    std::vector<MissionClearAll> _MissionClearAll_msg;
    std::vector<MissionItemReached> _MissionItemReached_msg;
    std::vector<MissionAck> _MissionAck_msg;
    std::vector<SetGpsGlobalOrigin> _SetGpsGlobalOrigin_msg;
    std::vector<GpsGlobalOrigin> _GpsGlobalOrigin_msg;
    std::vector<SetLocalPositionSetpoint> _SetLocalPositionSetpoint_msg;
    std::vector<LocalPositionSetpoint> _LocalPositionSetpoint_msg;
    std::vector<GlobalPositionSetpointInt> _GlobalPositionSetpointInt_msg;
    std::vector<SetGlobalPositionSetpointInt> _SetGlobalPositionSetpointInt_msg;
    std::vector<SafetySetAllowedArea> _SafetySetAllowedArea_msg;
    std::vector<SafetyAllowedArea> _SafetyAllowedArea_msg;
    std::vector<SetRollPitchYawThrust> _SetRollPitchYawThrust_msg;
    std::vector<SetRollPitchYawSpeedThrust> _SetRollPitchYawSpeedThrust_msg;
    std::vector<RollPitchYawThrustSetpoint> _RollPitchYawThrustSetpoint_msg;
    std::vector<RollPitchYawSpeedThrustSetpoint> _RollPitchYawSpeedThrustSetpoint_msg;
    std::vector<SetQuadMotorsSetpoint> _SetQuadMotorsSetpoint_msg;
    std::vector<SetQuadSwarmRollPitchYawThrust> _SetQuadSwarmRollPitchYawThrust_msg;
    std::vector<NavControllerOutput> _NavControllerOutput_msg;
    std::vector<SetQuadSwarmLedRollPitchYawThrust> _SetQuadSwarmLedRollPitchYawThrust_msg;
    std::vector<StateCorrection> _StateCorrection_msg;
    std::vector<RequestDataStream> _RequestDataStream_msg;
    std::vector<DataStream> _DataStream_msg;
    std::vector<ManualControl> _ManualControl_msg;
    std::vector<RcChannelsOverride> _RcChannelsOverride_msg;
    std::vector<VfrHud> _VfrHud_msg;
    std::vector<CommandLong> _CommandLong_msg;
    std::vector<CommandAck> _CommandAck_msg;
    std::vector<RollPitchYawRatesThrustSetpoint> _RollPitchYawRatesThrustSetpoint_msg;
    std::vector<ManualSetpoint> _ManualSetpoint_msg;
    std::vector<LocalPositionNedSystemGlobalOffset> _LocalPositionNedSystemGlobalOffset_msg;
    std::vector<HilState> _HilState_msg;
    std::vector<HilControls> _HilControls_msg;
    std::vector<HilRcInputsRaw> _HilRcInputsRaw_msg;
    std::vector<OpticalFlow> _OpticalFlow_msg;
    std::vector<GlobalVisionPositionEstimate> _GlobalVisionPositionEstimate_msg;
    std::vector<VisionPositionEstimate> _VisionPositionEstimate_msg;
    std::vector<VisionSpeedEstimate> _VisionSpeedEstimate_msg;
    std::vector<ViconPositionEstimate> _ViconPositionEstimate_msg;
    std::vector<HighresImu> _HighresImu_msg;
    std::vector<OmnidirectionalFlow> _OmnidirectionalFlow_msg;
    std::vector<FileTransferStart> _FileTransferStart_msg;
    std::vector<FileTransferDirList> _FileTransferDirList_msg;
    std::vector<FileTransferRes> _FileTransferRes_msg;
    std::vector<BatteryStatus> _BatteryStatus_msg;
    std::vector<Setpoint8dof> _Setpoint8dof_msg;
    std::vector<Setpoint6dof> _Setpoint6dof_msg;
    std::vector<MemoryVect> _MemoryVect_msg;
    std::vector<DebugVect> _DebugVect_msg;
    std::vector<NamedValueFloat> _NamedValueFloat_msg;
    std::vector<NamedValueInt> _NamedValueInt_msg;
    std::vector<Statustext> _Statustext_msg;
    std::vector<Debug> _Debug_msg;

    std::vector<SensorOffsets> _SensorOffsets_msg;
    std::vector<SetMagOffsets> _SetMagOffsets_msg;
    std::vector<Meminfo> _Meminfo_msg;
    std::vector<ApAdc> _ApAdc_msg;
    std::vector<DigicamConfigure> _DigicamConfigure_msg;
    std::vector<DigicamControl> _DigicamControl_msg;
    std::vector<MountConfigure> _MountConfigure_msg;
    std::vector<MountControl> _MountControl_msg;
    std::vector<MountStatus> _MountStatus_msg;
    std::vector<FencePoint> _FencePoint_msg;
    std::vector<FenceFetchPoint> _FenceFetchPoint_msg;
    std::vector<FenceStatus> _FenceStatus_msg;
    std::vector<Ahrs> _Ahrs_msg;
    std::vector<Simstate> _Simstate_msg;
    std::vector<Hwstatus> _Hwstatus_msg;
    std::vector<Radio> _Radio_msg;
    std::vector<LimitsStatus> _LimitsStatus_msg;
    std::vector<Wind> _Wind_msg;
    std::vector<Data16> _Data16_msg;
    std::vector<Data32> _Data32_msg;
    std::vector<Data64> _Data64_msg;
    std::vector<Data96> _Data96_msg;
    std::vector<Rangefinder> _Rangefinder_msg;

};

}} // namespace

#endif // SimpleFetchServer_H
