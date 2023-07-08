
"use strict";

let RCIn = require('./RCIn.js');
let HilGPS = require('./HilGPS.js');
let Param = require('./Param.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let FileEntry = require('./FileEntry.js');
let DebugValue = require('./DebugValue.js');
let PositionTarget = require('./PositionTarget.js');
let Thrust = require('./Thrust.js');
let VehicleInfo = require('./VehicleInfo.js');
let GPSRAW = require('./GPSRAW.js');
let Waypoint = require('./Waypoint.js');
let ExtendedState = require('./ExtendedState.js');
let BatteryStatus = require('./BatteryStatus.js');
let RTKBaseline = require('./RTKBaseline.js');
let WaypointList = require('./WaypointList.js');
let Trajectory = require('./Trajectory.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let Mavlink = require('./Mavlink.js');
let MountControl = require('./MountControl.js');
let LogEntry = require('./LogEntry.js');
let HomePosition = require('./HomePosition.js');
let CommandCode = require('./CommandCode.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let Altitude = require('./Altitude.js');
let State = require('./State.js');
let WaypointReached = require('./WaypointReached.js');
let ESCInfo = require('./ESCInfo.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let HilSensor = require('./HilSensor.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let GPSRTK = require('./GPSRTK.js');
let RadioStatus = require('./RadioStatus.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let LandingTarget = require('./LandingTarget.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let StatusText = require('./StatusText.js');
let ESCStatus = require('./ESCStatus.js');
let LogData = require('./LogData.js');
let ActuatorControl = require('./ActuatorControl.js');
let Vibration = require('./Vibration.js');
let VFR_HUD = require('./VFR_HUD.js');
let ParamValue = require('./ParamValue.js');
let ManualControl = require('./ManualControl.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let HilControls = require('./HilControls.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let RTCM = require('./RTCM.js');
let RCOut = require('./RCOut.js');

module.exports = {
  RCIn: RCIn,
  HilGPS: HilGPS,
  Param: Param,
  OverrideRCIn: OverrideRCIn,
  OnboardComputerStatus: OnboardComputerStatus,
  OpticalFlowRad: OpticalFlowRad,
  FileEntry: FileEntry,
  DebugValue: DebugValue,
  PositionTarget: PositionTarget,
  Thrust: Thrust,
  VehicleInfo: VehicleInfo,
  GPSRAW: GPSRAW,
  Waypoint: Waypoint,
  ExtendedState: ExtendedState,
  BatteryStatus: BatteryStatus,
  RTKBaseline: RTKBaseline,
  WaypointList: WaypointList,
  Trajectory: Trajectory,
  ADSBVehicle: ADSBVehicle,
  HilStateQuaternion: HilStateQuaternion,
  ESCStatusItem: ESCStatusItem,
  Mavlink: Mavlink,
  MountControl: MountControl,
  LogEntry: LogEntry,
  HomePosition: HomePosition,
  CommandCode: CommandCode,
  GlobalPositionTarget: GlobalPositionTarget,
  Altitude: Altitude,
  State: State,
  WaypointReached: WaypointReached,
  ESCInfo: ESCInfo,
  EstimatorStatus: EstimatorStatus,
  HilSensor: HilSensor,
  PlayTuneV2: PlayTuneV2,
  GPSRTK: GPSRTK,
  RadioStatus: RadioStatus,
  ESCInfoItem: ESCInfoItem,
  LandingTarget: LandingTarget,
  HilActuatorControls: HilActuatorControls,
  StatusText: StatusText,
  ESCStatus: ESCStatus,
  LogData: LogData,
  ActuatorControl: ActuatorControl,
  Vibration: Vibration,
  VFR_HUD: VFR_HUD,
  ParamValue: ParamValue,
  ManualControl: ManualControl,
  AttitudeTarget: AttitudeTarget,
  TimesyncStatus: TimesyncStatus,
  CamIMUStamp: CamIMUStamp,
  WheelOdomStamped: WheelOdomStamped,
  HilControls: HilControls,
  CompanionProcessStatus: CompanionProcessStatus,
  RTCM: RTCM,
  RCOut: RCOut,
};
