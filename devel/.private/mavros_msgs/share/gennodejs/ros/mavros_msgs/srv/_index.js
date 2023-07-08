
"use strict";

let CommandBool = require('./CommandBool.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let WaypointPull = require('./WaypointPull.js')
let CommandHome = require('./CommandHome.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let ParamPush = require('./ParamPush.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let CommandInt = require('./CommandInt.js')
let FileClose = require('./FileClose.js')
let FileMakeDir = require('./FileMakeDir.js')
let WaypointClear = require('./WaypointClear.js')
let FileRename = require('./FileRename.js')
let ParamGet = require('./ParamGet.js')
let MountConfigure = require('./MountConfigure.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let FileOpen = require('./FileOpen.js')
let SetMavFrame = require('./SetMavFrame.js')
let WaypointPush = require('./WaypointPush.js')
let CommandLong = require('./CommandLong.js')
let CommandTOL = require('./CommandTOL.js')
let StreamRate = require('./StreamRate.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let LogRequestList = require('./LogRequestList.js')
let FileTruncate = require('./FileTruncate.js')
let ParamSet = require('./ParamSet.js')
let FileRemove = require('./FileRemove.js')
let FileWrite = require('./FileWrite.js')
let SetMode = require('./SetMode.js')
let MessageInterval = require('./MessageInterval.js')
let FileList = require('./FileList.js')
let FileChecksum = require('./FileChecksum.js')
let ParamPull = require('./ParamPull.js')
let LogRequestData = require('./LogRequestData.js')
let FileRead = require('./FileRead.js')

module.exports = {
  CommandBool: CommandBool,
  CommandTriggerInterval: CommandTriggerInterval,
  WaypointPull: WaypointPull,
  CommandHome: CommandHome,
  FileRemoveDir: FileRemoveDir,
  ParamPush: ParamPush,
  WaypointSetCurrent: WaypointSetCurrent,
  CommandInt: CommandInt,
  FileClose: FileClose,
  FileMakeDir: FileMakeDir,
  WaypointClear: WaypointClear,
  FileRename: FileRename,
  ParamGet: ParamGet,
  MountConfigure: MountConfigure,
  LogRequestEnd: LogRequestEnd,
  CommandVtolTransition: CommandVtolTransition,
  FileOpen: FileOpen,
  SetMavFrame: SetMavFrame,
  WaypointPush: WaypointPush,
  CommandLong: CommandLong,
  CommandTOL: CommandTOL,
  StreamRate: StreamRate,
  CommandTriggerControl: CommandTriggerControl,
  VehicleInfoGet: VehicleInfoGet,
  LogRequestList: LogRequestList,
  FileTruncate: FileTruncate,
  ParamSet: ParamSet,
  FileRemove: FileRemove,
  FileWrite: FileWrite,
  SetMode: SetMode,
  MessageInterval: MessageInterval,
  FileList: FileList,
  FileChecksum: FileChecksum,
  ParamPull: ParamPull,
  LogRequestData: LogRequestData,
  FileRead: FileRead,
};
