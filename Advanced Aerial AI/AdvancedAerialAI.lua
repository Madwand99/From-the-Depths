--[[
Advanced aerial AI, version 5.32
Created by Madwand 2/4/2019
Use and modify this code however you like, however please credit me
if you use this AI or a derivative of it in a tournament, or you publish a blueprint
using it. Also let me know if you make any significant improvements,
so I can add them back into the code. 

Latest version and documentation is available at:
https://github.com/Madwand99/From-the-Depths/tree/master/Advanced%20Aerial%20AI
Forum thread to go for help:
http://www.fromthedepthsgame.com/forum/showthread.php?tid=9108
--]]

-- BASIC OPTIONS
AngleBeforeTurn = 0
AngleBeforeRoll = 70
AttackRunDistance = 1000
AbortRunDistance = 300
ClosingDistance = 1500
ForceAttackTime = 15
CruiseAltitude = 100
MaxDistance = 5000
MaxPitch = 30
MinPitch = -15

-- SPEED CONTROL
MinimumSpeed = 0
MaximumSpeed = 999
CruiseThrottle = 1
AttackRunThrottle = 1
EscapeThrottle = 1
RollingThrottle = 1
ClosingThrottle = 1

-- PID SETTINGS
--                   P,        D,       I,    OutMax,   OutMin,    IMax,    IMin
yawPIDData      = {0.2,     0.05,     0.0,         1,       -1,       1,      -1}
rollPIDData     = {0.2,     0.05,     0.0,         1,       -1,       1,      -1}
pitchPIDData    = {0.2,     0.02,     0.0,         1,       -1,       1,      -1}

-- HELICOPTER OPTIONS
HeliSpinners = {}
HeliDediblades = {}
MinHelicopterBladeSpeed = 10
MaxHelicopterBladeSpeed = 30

-- TERRAIN AVOIDANCE OPTIONS
AvoidTerrain = true
MinAltitude = 100
MaxAltitude = 800
TerrainLookahead = {0,1,2,4}
MaxTerrainThrottle = 1
TerrainAvoidanceWeight = 1

-- WATER START OPTIONS
DeployAlt = 5
ReleaseAlt = 15
EnableEnginesAlt = 0

-- COLLISION AVOIDANCE OPTIONS
CollisionTThreshold = 5
CraftRadius = 16
BufferSize = 50
EnemyRadius = 50
AvoidFriendlies = true
AvoidTarget = true
AvoidOtherEnemies = true
AvoidanceWeight = 10

-- STEERING OPTIONS
NeighborRadius = 0
IgnoreBelowSpeed = 40
FlockWithBlueprintNames = {'Cutlass'}
AlignmentWeight = 0
CohesionWeight = 0
SeparationWeight = 0
InjuredWeight = 0
LongRangeWeight = 0
TargetWeight = 1

-- "DOGFIGHTING" OPTIONS
MatchTargetAltitude = false
MatchAltitudeRange = 1000
MatchAltitudeOffset = 100
MinMatchingAltitude = 50
MaxMatchingAltitude = 800
BroadsideWithin=0
BroadsideAngle=0

-- MISSILE AVOIDANCE OPTIONS
WarningMainframe = -1
RunAwayTTT = 4
DodgeTTT = 2
DangerRadius = 20
NormalSpeed = 75
RunAwayWeight = 2
DodgingWeight = 5

-- VECTOR THRUST/LARGE RUDDER OPTIONS
VTSpinners = nil
MaxVTAngle = {40,40,40,90} -- yaw, roll, pitch, VTOL
VTResponseAngleMax = {20,20,20}
VTResponseAngleMin = {5,5,5}
VTSpeed = 30

--VTOL/HOVER OPTIONS
UseAltitudeJets = false
UseVTOLBeneathSpeed = 0
VTOLEngines = nil
VTOLSpinners = 'all'

-- ADVANCED OPTIONS
DriveMode = 2
HydrofoilMode = 0
UpdateRate = 1
AltitudeClamp = .2
MainDriveControlType = 0
MaxRollAngle = 90
RollTolerance = 30
DebugMode = false
UsePreferredSide = false
UsePredictiveGuidance = false
AltitudeOffset = 0
AngleOfEscape = 0
ExcludeSpinners = nil
OrbitSpawn = true

--Static variables. Do not change these.
YAWDRIVE = 1
ROLLDRIVE = 2
PITCHDRIVE = 3
DRIVEMAIN = 8
DriveTypeDesc = {'Left','Right','RLeft','RRight','PUp','PDown','','','Main'}

firstpass = true
state = "cruise"
onattackrun = false
WaterStarted = false
NextAttackTime = 0
PitchCorrection = 0
RollSpinners = {}
PitchSpinners = {}
YawSpinners = {}
DownSpinners = {}
AllSpinners = {}
ExcludedSpinners = {}
RollEngines = {}
PitchEngines = {}
Hydrofoils=0
ImpulseType = {}
EngineDF = {}
SpinnerStartRotations={}
NumEngines = 0
DesiredAltitude = 0
DesiredAzimuth = 0
OverTerrain = false
Rolling = false
Throttle=CruiseThrottle
Ticks = 0
VTPower = {0,0,0,0} -- yaw, roll, pitch, VTOL
DodgeDir=0

function InitPID(pidData)
    PID = {}
    PID.Kp            = pidData[1]
    PID.Kd            = pidData[2]
    PID.Ki            = pidData[3]
    PID.OutMax        = pidData[4]
    PID.OutMin        = pidData[5]
    PID.IMax          = pidData[6]
    PID.IMin          = pidData[7]
    PID.integral      = 0
    PID.previousError = 0

    return PID
end

function InitPIDs()
    rollPID     = InitPID(rollPIDData)
    pitchPID    = InitPID(pitchPIDData)
    yawPID      = InitPID(yawPIDData)
end

function GetPIDOutput(SetPoint, ProcessVariable, PID)
    local error     = SetPoint - ProcessVariable
    local timeDelta = 0.025
    local derivative
    local output

    PID.integral = PID.integral + (error*timeDelta) * PID.Ki
    if (PID.integral > PID.IMax) then PID.integral = PID.IMax end
    if (PID.integral < PID.IMin) then PID.integral = PID.IMin end

    derivative = (error - PID.previousError)/timeDelta

    output = PID.Kp*error + PID.Kd*derivative + PID.integral
    if (output > PID.OutMax) then output = PID.OutMax end
    if (output < PID.OutMin) then output = PID.OutMin end

    PID.previousError = error
    return output,PID
end

--Try to pitch (or yaw, if necessary) to a given angle of elevation
function AdjustPitchTo(I,DesiredPitch)
  pitchInput,pitchPID = GetPIDOutput(DesiredPitch, Pitch, pitchPID)

  PitchDiff = Pitch - DesiredPitch
  if (Pitch > DesiredPitch) then
    if (math.abs(Roll)>135) then
      Control(I, PITCHDRIVE, pitchInput, PitchDiff)
    elseif (Roll<-45) then
      Control(I, YAWDRIVE, pitchInput, -PitchDiff)
    elseif (Roll>45) then
      Control(I, YAWDRIVE, -pitchInput, -PitchDiff)
    elseif (math.abs(Roll)<45 and state~="rolling") then
      Control(I, PITCHDRIVE, pitchInput, PitchDiff)
    end
  elseif (Pitch < DesiredPitch) then
    if (math.abs(Roll)>135 and state~="rolling") then
      Control(I, PITCHDRIVE, pitchInput, PitchDiff)
    elseif (Roll<-45) then
      Control(I, YAWDRIVE, pitchInput, -PitchDiff)
    elseif (Roll>45) then
      Control(I, YAWDRIVE, -pitchInput, -PitchDiff)
    elseif (math.abs(Roll)<45) then
      Control(I, PITCHDRIVE, pitchInput, PitchDiff)
    end
  end
end

function GetDesiredAltitude(I, Engaged, Pos)
  local NewAltitude = CruiseAltitude

  if Engaged and MatchTargetAltitude and Pos.GroundDistance < MatchAltitudeRange then
    NewAltitude = math.max(math.min(Pos.AltitudeAboveSeaLevel+MatchAltitudeOffset,MaxAltitude,MaxMatchingAltitude),MinMatchingAltitude)
  end

  if AvoidTerrain then
    local VelocityV = I:GetVelocityVector()
    local TerrainAltitude = GetTerrainAltitude(I, VelocityV, {-30,0,30})
    local OverTerrain = Alt < TerrainAltitude + MinAltitude
    return OverTerrain, math.max(NewAltitude, TerrainAltitude + MinAltitude)
  end

  return false, NewAltitude
end

--Calculate desired altitude and go there
function AdjustAltitude(I, NewAltitude, OverTerrain)
  local AltPower = limiter((NewAltitude-Alt)*AltitudeClamp,1)
  if ((Alt < NewAltitude or VTOLEngines) and Speed<UseVTOLBeneathSpeed) then
    VTPower[4]=math.max(.2,AltPower)
    AdjustPitchTo(I, 0)
  else
    local Angle = math.max(MinPitch,math.min(MaxPitch,(NewAltitude-Alt)*AltitudeClamp))
-- pull up/down with full strength if we are below/above min/max altitudes
    if Alt>MaxAltitude and Pitch>0 then Angle = MinPitch end
    if OverTerrain then Angle = MaxPitch end
    AdjustPitchTo(I, Angle)
  end

  AdjustAltitudeJets(I,AltPower)
  AdjustHelicopterBlades(I,AltPower)
end

function AdjustAltitudeJets(I,AltPower)
  if not UseAltitudeJets then return end
  if (AltPower>0) then
    I:RequestThrustControl(4,AltPower)
    vertical = "up"
  else
    I:RequestThrustControl(5,-AltPower)
    vertical = "down"
  end
end

function ControlVTOLPower(I)
  local DriveFraction = {}
  if VTPower[4]~=0 then
    for k,v in pairs(PitchEngines) do DriveFraction[k] = VTPower[4]*EngineDF[k] end
    ComputeEngines(DriveFraction, RollEngines, 2)
    ComputeEngines(DriveFraction, PitchEngines, 3)
  else
    for k,v in pairs(PitchEngines) do DriveFraction[k] = EngineDF[k] end
  end
  for k, v in pairs(PitchEngines) do I:Component_SetFloatLogic(9,k,DriveFraction[k]) end
end

function AdjustHelicopterBlades(I,AltPower)
  if table.getn(HeliSpinners)+table.getn(HeliDediblades)==0 then return end
  local MidSpeed = (MaxHelicopterBladeSpeed-MinHelicopterBladeSpeed)/2
  local SpinSpeed = MinHelicopterBladeSpeed+MidSpeed+AltPower*MidSpeed
  for k,v in pairs(HeliSpinners) do I:SetSpinBlockContinuousSpeed(v, SpinSpeed) end
  for k,v in pairs(HeliDediblades) do I:SetDedibladeContinuousSpeed(v, SpinSpeed) end
end

-- Get the current stats about angles of the vehicle
function GetAngleStats(I)
  Roll = I:GetConstructRoll()
-- transform roll into roll we are familiar with from the game
  if (Roll > 180) then Roll = Roll-360 end
  
  Pitch = -I:GetConstructPitch()
  Yaw = I:GetConstructYaw()
  
-- Many vehicles need to keep their nose pitched upwards relative to velocity.
-- This uses basic machine learning to calculate the upwards pitch correction.
  if not VTPower or VTPower[4]==0 then -- if we haven't been in VTOL mode
    VPitch = math.deg(math.asin(I:GetVelocityVectorNormalized().y)) -- observed pitch from velocity
    PitchCorrection = math.max(MinPitch,math.min(MaxPitch,PitchCorrection + .01*(Pitch-VPitch-PitchCorrection)))
    Pitch = Pitch - PitchCorrection
  end

  CoM = I:GetConstructCenterOfMass()
  Alt = CoM.y
  ForwardV = I:GetConstructForwardVector()
  Speed=I:GetVelocityMagnitude()
end

function sign(x)
  return x>0 and 1 or x<0 and -1 or 0
end

-- returns azimuth modified by an offset angle, and whether rolling will be needed
function GetModifiedAzimuth(I, Azimuth, Offset, Flock)
  Azimuth = Azimuth-(UsePreferredSide and 1 or sign(Azimuth))*Offset
  if math.abs(Azimuth)>180 then Azimuth=Azimuth-sign(Azimuth)*360 end
  if Flock>0 then Azimuth=Flocking(I,Azimuth,Flock>1) end
  return Azimuth, VTPower[4]==0 and math.abs(Azimuth) > AngleBeforeRoll-Offset
end

--Finds roll angle needed to broadside target
function GetTargetElevation(Pos)
  local DAlt=Alt-Pos.AltitudeAboveSeaLevel
  local d=Pos.GroundDistance*math.cos(math.rad(90-math.abs(Pos.Azimuth)))
  return math.deg(math.atan(DAlt/d))
end

function TurnTowardsAzimuth(I, Azimuth, NewAltitude, Rolling)
  if Rolling then -- roll to turn
    RollTowardsAzimuth(I,Azimuth,NewAltitude)
    YawTowardsAzimuth(I,Azimuth)
    state = "rolling"
  else -- yaw to turn
    local RollAngle=0
    if TargetPos.Valid and TargetPos.GroundDistance<BroadsideWithin then
      RollAngle=math.min(MaxRollAngle,(GetTargetElevation(TargetPos)+BroadsideAngle))*(TargetPos.Azimuth>0 and 1 or -1)
    end

    AdjustRollToAngle(I,RollAngle)
    state = "yawing"
    YawTowardsAzimuth(I,Azimuth)
  end
end

-- Try to roll the craft and pull up on pitch controls to face a given azimuth (relative to the vehicles facing)
-- Will try to set roll angle at one that will maintain a given altitude
function RollTowardsAzimuth(I,Azimuth,NewAltitude)
-- depending on our altitude, RollAngle will be set to 90 +/- 40 degrees to climb or descend as appropriate
  RollAngle = sign(Azimuth)*math.min(MaxRollAngle, 90+limiter((Alt-NewAltitude)*.66, 30))
  
  AdjustRollToAngle(I,RollAngle)

  if (sign(Roll)==sign(Azimuth) and Roll >= RollAngle-RollTolerance and Roll <= RollAngle+RollTolerance) then -- start pitching
    Control(I, PITCHDRIVE, 1, Azimuth)
  end
end

-- Roll the vehicle to a specified roll angle
function AdjustRollToAngle(I,Angle)
  rollInput,rollPID = GetPIDOutput(Angle, Roll, rollPID)
  Control(I, ROLLDRIVE, rollInput, Roll-Angle)
end

-- transform an absolute azimuth into one relative to the nose of the vehicle
function GetRelativeAzimuth(AbsAzimuth)
  local Azimuth = Yaw-AbsAzimuth
  if math.abs(Azimuth)>180 then return Azimuth-sign(Azimuth)*360 end
  return Azimuth
end
      
-- Yaw the vehicle towards a given aziumth (relative to the vehicle's facing)
function YawTowardsAzimuth(I,Azimuth)
  if (math.abs(Roll)<30) then
    yawInput,yawPID = GetPIDOutput(0, Azimuth, yawPID)
    Control(I, YAWDRIVE, -yawInput, -Azimuth)
    return true
  end
  return false
end

-- No enemies, just cruise along
function Cruise(I)
  if OrbitSpawn and I:GetNumberOfMainframes()>0 then
    SpawnInfo = I:GetTargetPositionInfoForPosition(0, SpawnPos.x, 0, SpawnPos.z)
    NavigateToPoint(I, false, SpawnInfo)
  else
    if I:GetNumberOfMainframes()==0 then
      I:LogToHud("No AI mainframe found; please add one!")
    end
    state = "cruise"
    OverTerrain, DesiredAltitude = GetDesiredAltitude(I,false,nil)
    AdjustAltitude(I, DesiredAltitude, OverTerrain)
    AdjustRollToAngle(I, 0)
    SetSpeed(I, CruiseThrottle)
    NextAttackTime = I:GetTime()+ForceAttackTime
  end
end

-- Given the current velocity vector and an azimuth, get the expected terrain height in that direction
function GetTerrainAltitude(I, VelocityV, Angles)
  local TerrainAltitude = -999
  for j,Angle in ipairs(Angles) do
    for i,Lookahead in ipairs(TerrainLookahead) do
      local Position = CoM + Quaternion.Euler(0,Angle,0) * VelocityV * Lookahead
      TerrainAltitude = math.max(TerrainAltitude,I:GetTerrainAltitudeForPosition(Position))
    end
  end
  return TerrainAltitude
end

function GetIntercept(I, Pos)
  local TTT = FindConvergence(I, Pos.Position, Pos.Velocity, CoM, Speed, Speed*.75)
  local Prediction = Pos.Position + Pos.Velocity * TTT
  return TTT, I:GetTargetPositionInfoForPosition(0, Prediction.x, 0, Prediction.z).Azimuth, Prediction.y
end

-- Perform a water start check. Returns true if movement is permitted.
function WaterStartCheck(I)
  if (Alt < DeployAlt) then 
    I:Component_SetBoolLogicAll(0, true)
    WaterStarted = true
  elseif (WaterStarted and Alt > ReleaseAlt) then
    I:Component_SetBoolLogicAll(0, false)
    WaterStarted = false
  end
  
  if (not WaterStarted or Alt>=EnableEnginesAlt) then
    return true
  end
  return false
end

function SetSpeed(I, Throttle)
  if (Speed>MaximumSpeed) then Throttle=.1
  elseif (Speed<MinimumSpeed) then Throttle=1 end
  if MainDriveControlType==1 then
    I:RequestThrustControl(0)
  elseif MainDriveControlType==2 then
    I:RequestControl(DriveMode,DRIVEMAIN,1)
    for k, v in pairs(Main) do I:Component_SetFloatLogic(9,v,Throttle) end
  else
    I:RequestControl(DriveMode,DRIVEMAIN,Throttle)
  end
end

function limiter(p,l)
  return math.min(l,math.max(-l,p))
end

function Control(I, DoFType, Impulse, Angle)
  local Type = DoFType*2 - (Impulse<0 and 1 or 2)
  I:RequestControl(DriveMode, Type, math.abs(Impulse))
  DriveDesc[DoFType] = DriveTypeDesc[Type+1]
  ImpulseType[DoFType] = Impulse
  if math.abs(Angle)>VTResponseAngleMin[DoFType] then
    local Max=VTResponseAngleMax[DoFType]-VTResponseAngleMin[DoFType]+.01
    VTPower[DoFType] = limiter(Angle-sign(Angle)*VTResponseAngleMin[DoFType],Max)/Max
  else VTPower[DoFType]=0 end
end

function ComputeSpinners(Rotation, Spinners, Axis)
  for Spinner,Dir in pairs(Spinners) do
    Rotation[Spinner] = Rotation[Spinner] + Dir*VTPower[Axis]*MaxVTAngle[Axis]
  end
end

function ComputeEngines(DriveFraction, Engines, Axis)
  for Engine,Dir in pairs(Engines) do
    if Dir==sign(VTPower[Axis]) then
      DriveFraction[Engine] = DriveFraction[Engine]*(1-math.abs(VTPower[Axis]))
    end
  end
end

function VectorEngines(I)
  local RL,RR,RY = 0,0,0
  
  local Rotation = {}
  for k,v in pairs(AllSpinners) do Rotation[v] = 0 end
  
  if (state=="rolling" and VTPower[3]>0) then
    VTPower[3]=0
  end
  ComputeSpinners(Rotation, YawSpinners, 1)
  ComputeSpinners(Rotation, RollSpinners, 2)
  ComputeSpinners(Rotation, PitchSpinners, 3)
  if (VTOLEngines and VTPower[4]~=0) then -- VTOL Mode
    for Spinner,Dir in pairs(DownSpinners) do Rotation[Spinner] = Dir*MaxVTAngle[4] end
  else
    ComputeSpinners(Rotation, DownSpinners, 4)
  end 

  RotateSpinnersTo(I,Rotation)
  
  --I:LogToHud(string.format("Y: %.02f R: %.02f P: %.02f", VTPower[1], VTPower[2], VTPower[3]))
end

function ClassifySpinner(I,p)
  if VTSpinners=='all' and ExcludedSpinners[p] then return end
  local info = I:GetSubConstructInfo(p)
  local h,a,b = EulerAngles(info.LocalRotation)
  local pos = info.LocalPositionRelativeToCom
  SpinnerStartRotations[p]=I:GetSubConstructIdleRotation(p)
  a=math.floor(a+.5)
  b=math.floor(b+.5)
  if (a==0 and b==0) then
    tinsert(pos.z,YawSpinners,1,-1,p)
    AddSpinner(pos.y,-1,p)
  elseif (a==0 and math.abs(b)>170) then
    tinsert(pos.z,YawSpinners,-1,1,p)
    AddSpinner(pos.y,1,p)
  else
    local h,a,b = EulerAngles(info.LocalRotation*Quaternion.Euler(0, 0, 90))
    h=math.floor(h+.5)
    a=math.floor(a+.5)
    if (h==0 and a==0) then -- pointed right
      tinsert(pos.z,PitchSpinners,1,-1,p)
      if VTOLSpinners=='all' and ExcludedSpinners[p] then tinsert(pos.z,DownSpinners,-1,-1,p) end
      AddSpinner(pos.x,-1,p)
    elseif (a==0 and math.abs(h)>170) then -- pointed left
      tinsert(pos.z,PitchSpinners,-1,1,p)
      if VTOLSpinners=='all' and ExcludedSpinners[p] then tinsert(pos.z,DownSpinners,1,1,p) end
      AddSpinner(pos.x,1,p)
    end
  end
end

function ClassifyVTOLSpinner(I,p)
  local info = I:GetSubConstructInfo(p)
  local h,a,b = EulerAngles(info.LocalRotation*Quaternion.Euler(0, 0, 90))
  local pos = info.LocalPositionRelativeToCom
  h=math.floor(h+.5)
  a=math.floor(a+.5)
  --I:Log(string.format("Spinner: %d Orientation: (%.2f, %.2f, %.2f) Position: (%.2f, %.2f, %.2f)", p, h, a, b, pos.x, pos.y, pos.z))
  if (h==0 and a==0) then
    tinsert(pos.z,DownSpinners,-1,-1,p)
  elseif (a==0 and math.abs(h)>170) then
    tinsert(pos.z,DownSpinners,1,1,p)
  end
end

function GetPosRelativeToCoM(pos)
  local rot = Quaternion.Inverse(Quaternion.LookRotation(ForwardV))
  local rpos = rot*(pos-CoM)
  for i, p in ipairs(rpos) do rpos[i]=math.floor(p*10+.5)/10 end
  return rpos
end

function ClassifyEngine(I,p,force)
  local info = I:Component_GetBlockInfo(9,p)
  if (force or info.LocalForwards.y==1) then
    pos = GetPosRelativeToCoM(info.Position)
    EngineDF[p] = I:Component_GetFloatLogic(9,p)
    tinsert(pos.z,PitchEngines,1,-1,p)
    if pos.x<-1.1 then  -- on left
      tinsert(pos.z,RollEngines,-1,-1,p)
    elseif pos.x>1.1 then -- on right
      tinsert(pos.z,RollEngines,1,1,p)
    end
  end
end

function AddSpinner(comp,dir,p)
  if comp<-1 then  -- on left
    RollSpinners[p] = dir
  elseif comp>1 then -- on right
    RollSpinners[p] = -dir
  end
  table.insert(AllSpinners,p)
end
      
function tinsert(z,s,x,y,p)
  s[p] = z>0 and x or y
end

function ClassifyHydrofoil(I,p)
  local info = I:Component_GetBlockInfo(8,p)
  local pos = info.LocalPositionRelativeToCom
  local h,a,b = EulerAngles(info.LocalRotation)
  h=math.floor(h+.5)
  a=math.floor(a+.5)

  if h/180~=math.floor(h/180) then
    return -- not properly placed
  elseif a==0 then
    PitchHydros[p],RollHydros[p]=HydroTurnRoll(h==0,pos.z,pos.x)
  else
    YawHydros[p],  RollHydros[p]=HydroTurnRoll(a>=0,pos.z,pos.y)
  end
end

function HydroTurnRoll(test,pos1,pos2)
  local forwards=test and 1 or -1
  local HTurn,HRoll = nil,nil
  if pos1>Length/6 then
    HTurn=forwards
  elseif pos1<-Length/6 then
    HTurn=-forwards
  end

  if HydrofoilMode==2 or not HTurn then
    if pos2<0 then
      HRoll=forwards
    elseif pos2>0 then
      HRoll=-forwards
    end
  end
  return HTurn,HRoll
end

function ClassifyHydrofoils(I)
  if HydrofoilMode>0 and Hydrofoils~=I:Component_GetCount(8) then
    Hydrofoils=I:Component_GetCount(8)
    YawHydros,RollHydros,PitchHydros={},{},{}
    for p = 0, Hydrofoils - 1 do ClassifyHydrofoil(I,p) end
  end
end

function AdjustHydrofoils(I)
  local Hydros={}
  local ToYaw=ImpulseType[1]*45
  local ToRoll=-ImpulseType[2]*45
  local ToPitch=ImpulseType[3]*45
  for p = 0, Hydrofoils - 1 do Hydros[p]=0 end
  for p, k in pairs(YawHydros) do Hydros[p]=ToYaw*k end
  for p, k in pairs(PitchHydros) do Hydros[p]=ToPitch*k end
  for p, k in pairs(RollHydros) do Hydros[p]=Hydros[p]+ToRoll*k end
  for p, k in pairs(Hydros) do I:Component_SetFloatLogic(8,p,k) end
end

function EulerAngles(q1)
  local sqw = q1.w*q1.w
  local sqx = q1.x*q1.x
  local sqy = q1.y*q1.y
  local sqz = q1.z*q1.z
  local unit = sqx + sqy + sqz + sqw --if normalised is one, otherwise is correction factor
  local test = q1.x*q1.y + q1.z*q1.w
  local heading, attitude, bank
  if (test > 0.499*unit) then --singularity at north pole
    heading = 2 * math.atan2(q1.x,q1.w)
    attitude = math.pi/2;
    bank = 0
  elseif (test < -0.499*unit) then --singularity at south pole
    heading = -2 * math.atan2(q1.x,q1.w)
    attitude = -math.pi/2
    bank = 0
  else
    heading = math.atan2(2*q1.y*q1.w-2*q1.x*q1.z , sqx - sqy - sqz + sqw)
    attitude = math.asin(2*test/unit)
    bank = math.atan2(2*q1.x*q1.w-2*q1.y*q1.z , -sqx + sqy - sqz + sqw)
  end
  return math.deg(heading), math.deg(attitude), math.deg(bank)
end

function RotateSpinnersTo(I, Spinners)
  for Spinner, NewAngle in pairs(Spinners) do
    local Angle = EulerAngles(Quaternion.Inverse(SpinnerStartRotations[Spinner]) * I:GetSubConstructInfo(Spinner).LocalRotation)
    local DeflectAngle = limiter(limiter(NewAngle,90)-Angle,VTSpeed*43/30)
    if math.abs(DeflectAngle)<1 then DeflectAngle=0 end
    I:SetSpinBlockContinuousSpeed(Spinner,DeflectAngle*30/43)
  end
end

function FindConvergence(I, tPos, tVel, wPos, wSpeed, minConv)
   local relativePosition = wPos - tPos
   local distance = Vector3.Magnitude(relativePosition)
   local targetAngle = I:Maths_AngleBetweenVectors(relativePosition, tVel)
   local tSpeed = Vector3.Magnitude(tVel)

   local a = tSpeed^2 - wSpeed^2
   local b = -2 * tSpeed * distance * math.cos(math.rad(targetAngle))
   local c = distance^2
   local det = math.sqrt(b^2-4*a*c)
   local ttt = distance / (minConv+.01)

   if det > 0 then
      local root1 = math.min((-b + det)/(2*a), (-b - det)/(2*a))
      local root2 = math.max((-b + det)/(2*a), (-b - det)/(2*a))
      ttt = (root1 > 0 and root1) or (root2 > 0 and root2) or ttt
   end
   return ttt
end

function GetMissileWarnings(I)
  local NumWarnings = I:GetNumberOfWarnings(WarningMainframe)
  local OwnVelocity = ForwardV * math.max(NormalSpeed, Speed)
  local TTTs={}
  local Warnings={}
  
  for w = 0, NumWarnings - 1 do
    local Warning = I:GetMissileWarning(WarningMainframe, w)
    if Warning.Valid and I:Maths_AngleBetweenVectors(Warning.Velocity, CoM - Warning.Position) < 90 then
      local mSpeed = Vector3.Magnitude(Warning.Velocity)
      local TTT = FindConvergence(I, CoM, OwnVelocity, Warning.Position, mSpeed, mSpeed*.75)
      local PredictedPosition = CoM + OwnVelocity * TTT
      local Orthogonal = (Warning.Position + Vector3.Normalize(Warning.Velocity)
                         * Vector3.Distance(Warning.Position, PredictedPosition)
                         * math.cos(math.rad(I:Maths_AngleBetweenVectors(Warning.Velocity, PredictedPosition - Warning.Position))))
                         - PredictedPosition
      local AdjustedPosition = PredictedPosition + Vector3.ClampMagnitude(Orthogonal, CraftRadius)
      local Radius = (Vector3.Distance(Warning.Position, AdjustedPosition) / 2)
                     / math.cos(math.rad(I:Maths_AngleBetweenVectors(Warning.Velocity, AdjustedPosition - Warning.Position) - 90))
      if TTT < math.max(RunAwayTTT,DodgeTTT) and Radius > DangerRadius then
        table.insert(TTTs, TTT)
        table.insert(Warnings, Warning)
      end
    end
  end
  return TTTs, Warnings
end

function PrintList(I, name, list)
  str = name..': '
  for i,v in pairs(list) do
    str = str..string.format("%d ", i)
  end
  I:Log(str)
end

function ClassifyEngines(I)
  if NumEngines==I:Component_GetCount(9) then return end
  NumEngines=I:Component_GetCount(9)

  Main={} -- try to figure out which drives are pointed backwards
  for p = 0, NumEngines - 1 do
    local info=I:Component_GetBlockInfo(9,p)
    if (info.LocalForwards.z==1 and GetPosRelativeToCoM(info.Position)==info.LocalPositionRelativeToCom) then
      table.insert(Main,p)
    end
  end

  if VTOLEngines=='all' then
    for p = 0, NumEngines - 1 do ClassifyEngine(I,p,false) end
  end
  PrintList(I,"RollEngines",RollEngines)
  PrintList(I,"PitchEngines",PitchEngines)
  PrintList(I,"Main",Main)
end

function NameMatches(Name)
  if FlockWithBlueprintNames=='all' then return true end
  if type(FlockWithBlueprintNames) == "table" then
    for k,v in pairs(FlockWithBlueprintNames) do
      if v==string.sub(Name,1,string.len(v)) then return true end
    end
  elseif FlockWithBlueprintNames==string.sub(Name,1,string.len(FlockWithBlueprintNames)) then return true
  end
  return false
end

function SteerToAvoidCollisions(I)
  local ThreatCoM, ThreatVel
  local minTime = CollisionTThreshold
  local OwnVelocity = ForwardV * Speed

  if AvoidFriendlies then
    local FCount = I:GetFriendlyCount()
    for f = 0, FCount-1 do
      local FInfo = I:GetFriendlyInfo(f)
      if FInfo.Valid then
        local time, dist = GetApproach(OwnVelocity, FInfo.Velocity, FInfo.CenterOfMass)
        if time>=0 and time<minTime then
          fsize = math.max(unpack(FInfo.PositiveSize-FInfo.NegativeSize))
          if dist < CraftRadius+fsize/2+BufferSize then
            minTime=time
            ThreatCoM, ThreatVel = FInfo.CenterOfMass, FInfo.Velocity
          end
        end
      end
    end
  end

  if (AvoidTarget or AvoidOtherEnemies) and I:GetNumberOfMainframes() > 0 then
    local ECount = I:GetNumberOfTargets(0)
    for e = (AvoidTarget and 0 or 1), (AvoidOtherEnemies and ECount-1 or 0) do
      local EInfo = I:GetTargetPositionInfo(0,e)
      if EInfo.Valid then
        local time, dist, tpos = GetApproach(OwnVelocity, EInfo.Velocity, EInfo.Position)
        if time>=0 and time<minTime and dist < CraftRadius+EnemyRadius+BufferSize then
          minTime=time
          ThreatPos, ThreatCoM, ThreatVel = tpos, EInfo.Position, EInfo.Velocity
        end
      end
    end
  end

  return I:GetConstructRightVector()*Dodge(I, minTime, CollisionTThreshold, ThreatVel, ThreatCoM)
end

-- Given a time-to-target, threat velocity,
-- and threat CoM, returns a dodge vector
function Dodge(I, TTT, TThreshold, TVel, TCoM)
  local side = I:GetConstructRightVector()
  local steer = 0

  if TTT<TThreshold then
    local parallelness = Vector3.Dot(ForwardV, TVel.normalized)
    
    if parallelness<-0.707 then  -- head-on collision
      local TFPos=TCoM+TVel*TTT
      local offset=TFPos-CoM
      local sideDot=Vector3.Dot(offset,side)
      steer = sideDot>0 and -1 or 1
    else--if parallelness>0.707 then  -- parallel paths
      local offset=TCoM-CoM
      local sideDot=Vector3.Dot(offset,side)
      steer = sideDot>0 and -1 or 1      
--    else  --perpendicular paths
--      if TVel.magnitude<=Speed then
--        local sideDot=Vector3.Dot(side,TVel)
--        steer = sideDot>0 and -1 or 1
--      end
    end    
  end

  return steer
end

function GetApproach(OwnVelocity, TVelocity, TPos)
  local relVelocity = TVelocity - OwnVelocity
  local relSpeed = relVelocity.magnitude+.01

  if relSpeed==0 then return 0 end

  local relTangent = relVelocity / relSpeed
  local relPos = CoM - TPos
  local projection = Vector3.Dot(relTangent,relPos)

  local time = projection/relSpeed

  local myFinal = CoM + ForwardV*Speed*time
  local oFinal = TPos + TVelocity*time

  return time, (myFinal-oFinal).magnitude
end

function Flocking(I, Azimuth, Escaping)
  local A = Vector3(0,0,0)
  local C,S,J,L,E,D,M = A,A,A,A,A,A,A
  local Near,Aligning,Injured,Far,Terrain=0,0,0,0,0

  local V=SteerToAvoidCollisions(I)*AvoidanceWeight

  if NeighborRadius>0 then
    local FCount = I:GetFriendlyCount()
    for f = 0, FCount-1 do
      local FInfo = I:GetFriendlyInfo(f)
      if FInfo.Valid then
        local Dist=FInfo.CenterOfMass-CoM
        if Dist.magnitude<NeighborRadius then
          if FInfo.Velocity.magnitude>=IgnoreBelowSpeed and NameMatches(FInfo.BlueprintName) then
            A=A+FInfo.Velocity  -- Alignment
            C=C+FInfo.CenterOfMass -- Cohesion
            J=J+FInfo.CenterOfMass*(1-FInfo.HealthFraction) -- Injured cohesion
            Aligning=Aligning+1
            Injured=Injured+(1-FInfo.HealthFraction)
          end
          S=S+Dist -- Separation
          Near=Near+1
        elseif FInfo.Velocity.magnitude>=IgnoreBelowSpeed and NameMatches(FInfo.BlueprintName) then
          L=L+FInfo.CenterOfMass -- Long-range cohesion
          Far=Far+1
        end
      end
    end

    if Aligning>0 then
      A=(A/Aligning).normalized*AlignmentWeight
      C=(C/Aligning-CoM).normalized*CohesionWeight
    end
    if Injured>0 then J=(J/Injured-CoM).normalized*InjuredWeight end
    if Far>0 then L=(L/Far-CoM).normalized*LongRangeWeight end
    if Near>0 then S=(-S/Near).normalized*SeparationWeight end
  end

  if TerrainAvoidanceWeight~=0 then
    local VelocityV = I:GetVelocityVector()
    local Angles = {0,45,90,135,180,225,270,315}
    for k, a in pairs(Angles) do
      local Position = CoM + Quaternion.Euler(0,a,0) * VelocityV
      local TerrainAltitude = GetTerrainAltitude(I, VelocityV, {a})
      if (TerrainAltitude + MinAltitude > CruiseAltitude) then
        local Dist=Position-CoM
        E=E+Dist*(TerrainAltitude + MinAltitude - CruiseAltitude)
        Terrain=Terrain+1
      end
    end
    if Terrain>0 then E=(-E/Terrain).normalized*TerrainAvoidanceWeight end
  end

  if WarningMainframe>=0 and I:GetNumberOfMainframes() > 0 and RunAwayWeight+DodgingWeight~=0 then 
    local TTTs, Warnings = GetMissileWarnings(I)
    local DodgingTTT=RunAwayTTT
    local DodgeCount=0
    for w,t in ipairs(TTTs) do
      if t<DodgeTTT then DodgeCount=DodgeCount+1 end
      if not Dodging then DodgeDir=DodgeDir+Dodge(I, t, DodgeTTT, Warnings[w].Velocity, Warnings[w].Position)*(1-t/RunAwayTTT) end
      D=D+(CoM-Warnings[w].Position).normalized*(1-t/RunAwayTTT)*RunAwayWeight
    end
    Dodging = DodgeCount~=0
    if Dodging then D=(I:GetConstructRightVector()*DodgeDir).normalized*DodgingWeight
    else DodgeDir=0 end
  end 

  local T=Quaternion.Euler(0,Yaw-Azimuth,0)*Vector3.forward*TargetWeight*(Escaping and 0.1 or 1)
  if TargetPos.Valid and MaxDistance>0 then
    local ToTarget=TargetPos.Position-CoM
    M=ToTarget.normalized*math.max(0,ToTarget.magnitude-MaxDistance)-E
  end

  local AllVectors=A+C+S+J+L+E+T+V+D+M
  local Objective = CoM+AllVectors
  return I:GetTargetPositionInfoForPosition(0, Objective.x, 0, Objective.z).Azimuth
end

function GetNavigationInfo(I,Engaged,Pos)
  Throttle = EscapeThrottle
  local CollisionTTT, CollisionAzimuth, CollisionAlt = GetIntercept(I, Pos)
  local OverTerrain, NewAltitude = GetDesiredAltitude(I,Engaged,Pos)
  local NewAzimuth, Rolling

  if (Pos.GroundDistance > ClosingDistance) then
    NewAzimuth, Rolling=GetModifiedAzimuth(I, CollisionAzimuth, 0, 1)
    Throttle = ClosingThrottle
    state = "closing"
    onattackrun = true
  elseif onattackrun then
    NewAzimuth, Rolling = GetModifiedAzimuth(I, UsePredictiveGuidance and CollisionAzimuth or Pos.Azimuth, AngleBeforeTurn, 1)
    Throttle = AttackRunThrottle
    if (Pos.GroundDistance < AbortRunDistance) then
      onattackrun = false
      NextAttackTime = I:GetTime()+ForceAttackTime
      EscapeAngle = Yaw+(UsePreferredSide and 1 or sign(NewAzimuth))*AngleOfEscape
    end
  else -- not on attack run, just go forwards until we're out of range
    NewAzimuth, Rolling=GetModifiedAzimuth(I, GetRelativeAzimuth(EscapeAngle), 0, 2)
    state = "escaping"
    onattackrun = Pos.GroundDistance > AttackRunDistance or I:GetTime()>NextAttackTime
  end
  
  if Rolling then Throttle=RollingThrottle end
  if OverTerrain then
    Throttle = math.min(Throttle,MaxTerrainThrottle)
  end
  if not Engaged then
    Throttle = math.min(Throttle,CruiseThrottle)
  end

  return OverTerrain, NewAltitude, NewAzimuth, Rolling
end

-- given information about a target, navigates to it
function NavigateToPoint(I, Engaged, Pos)
  if Ticks==0 then
    OverTerrain, DesiredAltitude, DesiredAzimuth, Rolling = GetNavigationInfo(I,Engaged,Pos)
  end
  Ticks = Ticks+1
  if Ticks>=UpdateRate then Ticks=0 end

  AdjustAltitude(I, DesiredAltitude, OverTerrain)
  TurnTowardsAzimuth(I, DesiredAzimuth, DesiredAltitude, Rolling)
  SetSpeed(I, Throttle)
end

function Initialize(I)
  if not firstpass then return end
  firstpass = false

  SpawnPos = CoM
  EscapeAngle = Yaw
  if type(AltitudeOffset) == "table" then
    math.randomseed(I:GetTime()+CoM.x+CoM.y+CoM.z)
    AltitudeOffset=math.floor(math.random()*(AltitudeOffset[2]-AltitudeOffset[1])+AltitudeOffset[1])
    I:Log("AltitudeOffset: "..AltitudeOffset)
  end
  CruiseAltitude = CruiseAltitude+AltitudeOffset
  MaxAltitude = MaxAltitude+AltitudeOffset
  MatchAltitudeOffset = MatchAltitudeOffset+AltitudeOffset
  MinMatchingAltitude = MinMatchingAltitude+AltitudeOffset
  MaxMatchingAltitude = MaxMatchingAltitude+AltitudeOffset
  Length = (I:GetConstructMaxDimensions() - I:GetConstructMinDimensions()).z

  Ticks=math.floor(math.random()*UpdateRate)

  InitPIDs()

  if type(VTOLEngines) == "table" then
    for k, p in pairs(VTOLEngines) do ClassifyEngine(I,p,true) end
  end

  if ExcludeSpinners then
    for k, p in pairs(ExcludeSpinners) do ExcludedSpinners[p]=true end
  end

  if HeliSpinners=='all' then
    HeliSpinners={}
    for k,p in pairs(I:GetAllSubConstructs()) do
      if I:IsSpinBlock(p) then
        table.insert(HeliSpinners, p)
      end
    end
  elseif HeliSpinners then
    for k, p in pairs(HeliSpinners) do ExcludedSpinners[p]=true end
  end

  if HeliDediblades=='all' then
    HeliDediblades={}
    for p = 0, I:GetDedibladeCount() - 1 do
       table.insert(HeliDediblades, p)
    end
  end

  if VTSpinners=='all' then
    for k,p in pairs(I:GetAllSubConstructs()) do
      if I:IsSpinBlock(p) and not ExcludedSpinners[p] then
        ClassifySpinner(I,p)
      end
    end
  elseif type(VTSpinners) == "table" then
    for k, p in pairs(VTSpinners) do
      ClassifySpinner(I,p)
    end
  end

  if type(VTOLSpinners) == "table" then
    for k, p in pairs(VTOLSpinners) do ClassifyVTOLSpinner(I,p) end
  end
  PrintList(I,"RollSpinners",RollSpinners)
  PrintList(I,"PitchSpinners",PitchSpinners)
  PrintList(I,"YawSpinners",YawSpinners)
  PrintList(I,"DownSpinners",DownSpinners)

  I:Log("AI successfully initialized.")
end

-- Calculate all movement for the vehicle
function Movement(I)
  Initialize(I)
  ClassifyEngines(I)
  ClassifyHydrofoils(I)

  DriveDesc = {'','','',''}
    
  TargetPos = I:GetTargetPositionInfo(0,0)
  if TargetPos.Valid then
      NavigateToPoint(I, true, TargetPos)
  else
    Cruise(I)
  end
  
  if VTSpinners then VectorEngines(I) end
  if VTOLEngines then ControlVTOLPower(I) end
  if HydrofoilMode>0 then AdjustHydrofoils(I) end
  
  if DebugMode then
    --I:LogToHud(string.format("%.2f %.2f", DPitch, math.abs(Roll)))
    I:LogToHud(string.format("%s %s %s %s %f", state, DriveDesc[1], DriveDesc[2], DriveDesc[3], Throttle))
    --I:LogToHud(string.format("Y:%.2f R:%.2f P:%.2f", Yaw, Roll, Pitch))
  end
end

-- Main update function. Everything starts here.
function Update(I)
  GetAngleStats(I)
  if not I:IsDocked() and WaterStartCheck(I) and (I.AIMode=='on' or I:GetNumberOfMainframes()==0) then
    Movement(I)
  else
    for k,p in pairs(AllSpinners) do I:SetSpinBlockRotationAngle(p,0) end
  end
end
--

