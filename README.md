Introducing my version of an advanced aerial AI. This Lua script is intended as a drop-in replacement for the standard aerial AI, with many more parameters to set and far more robust altitude matching and terrain handling, as well as methods for mitigating the "wobbling" you can get in high-performance vehicles.

This AI also includes built-in water start AI, collision avoidance AI, "dogfighting" options, vector thrust options, missile avoidance, predictive interception, VTOL takeoff, flocking, and several other features. It can also be used to control naval vessels, helicopters, and many kinds of hovercraft and airships.

## Usage ##

Put a Lua box down somewhere on your vehicle (it's in the "Control" tab of the build menu). Use "Q" on the Lua box. Copy-and-paste (Ctrl-A,Ctrl-C) the Lua code there into the big edit box in the Lua box. Read the documentation, below. Choose appropriate AI settings for your vehicle. Click "Apply" to save the code and new settings. Watch how your vehicle behaves and continue to tweak the AI settings as desired until you get the behavior you want.
Note that when you paste this code in, you will only see a bit less than 500 lines of code because that's how many the Lua box can display. However, all the code should be there.

To use this AI for naval vessels:
Set "AngleBeforeRoll" to 180, so your ship yaws to turn all the time.
Set "CruiseAltitude" to the typical altitude of your ship. This isn't strictly necessary to do but it helps the AI not be confused.
Set "DeployAlt" to some large negative value so it won't try to do a water start and thus turn off the engines.
Set "Mode" to 0 so it uses water mode controls.
Set other parameters as desired to control the behavior of the ship. Good luck!

To use this AI for hovercraft, helicopters, or airships:
Set "AngleBeforeRoll" to 180, so your ship yaws to turn all the time.
Set "MaxElevationAngle" to 0 so that your ship won't pitch up or down.
To use thrusters to control altitude, set UseAltitudeJets = true.
To use helicopter blades to control altitude, set UseSpinnters = true, and set min and max helicopter blade speeds to an appropriate value for your helicopter.
You may gain some benefit from using smaller values of "PitchDamping". If the default value isn't working, try something like 20. This will make a hovercraft more sensitive to deviations in pitch.


## BASIC OPTIONS ##
When the vehicle is within "AngleBeforeRoll" degrees of its target, it will try to yaw towards
(or away!) from its target such that its nose is pointed "AngleBeforeTurn" degrees away from it.
If UsePreferredSide = true, use a positive value of AngleBeforeTurn to approach targets on the right,
a negative value to approach targets on the left. Otherwise, always use a positive value. 
    AngleBeforeTurn = 10
If the vehicle is at greater than this angle from it's target, it will try to roll towards it.
(and stop rolling when AngleBeforeTurn is reached).
Set this to 180 to NEVER use roll.
    AngleBeforeRoll = 30
Outside of this distance, the vehicle will try to close on it's target.
NOTE: All distances are measured along the ground! (unlike the default AI)
    AttackRunDistance = 600
Within this distance, the vehicle will just try to go straight until AttackRunDistance is triggered.
    AbortRunDistance = 300
Outside of this distance, the AI will try to intercept its target by attempting
to predict it's future position.
    ClosingDistance = 1000
If AttackRunDistance hasn't been triggered within this time, force an attack run.
    ForceAttackTime = 15
Altitude we will try to cruise at by default.
    CruiseAltitude = 100
Maximum elevation angle, positive or negative, AI will attempt to use while climbing or descending.
Usually refers to pitch, but AI will also try to yaw to this angle when rolled on it's side.
    MaxElevationAngle = 30
Maximum throttle we'll cruise at when no enemies are present. Should be in the range of 0 to 5.
    CruiseSpeed = 5
Throttle we'll use when engaged with enemies. Should be in the range of 0 to 5.
    AttackRunSpeed = 5 -- when on an attack run (angle to target is less than "AngleBeforeRoll") inside of ClosingDistance 
    EscapeSpeed = 5    -- triggered after coming within "AbortRunDistance" until "AttackRunDistance" is reached
    RollingSpeed = 5   -- when rolling aircraft to face enemy (angle to target is greater than "AngleBeforeRoll")
    ClosingSpeed = 5   -- when outside of ClosingDistance AND not in a roll
Whether to orbit the spawn point when there are no enemies, or just fly off in a straight line
    OrbitSpawn = true

## AIRSHIP/HELICOPTER OPTIONS ##
Use jets pointing up or down to assist in controlling altitude. WARNING: uses Nick's propulsion balancing code.
    UseAltitudeJets = false
Use spinners (hopefully with helicopter blades) to try to control altitude.
Automatically excludes any spinners used for vector thrust.
See the "ExcludeSpinners" settings to specify an additional list of spinners to exclude.
    UseSpinners = false
The helicopter blade speed to use when lowering altitude. Ranges between -30 to 30.
Use AltitudeClamp to control how smoothly these speeds are transitioned between
as your helicopter approaches it's target altitude.
    MinHelicopterBladeSpeed = 10
The helicopter blade speed to use when gaining altitude. Ranges between -30 to 30.
    MaxHelicopterBladeSpeed = 30
A third option for airships is to allow the AI to manually control downward-pointing engine drive
fractions. This option can be turned off above a certain speed (in m/s) for VTOL takeoffs.
Set "UseVTOLBeneathSpeed" to a large number to use VTOL all the time
    UseVTOLBeneathSpeed = 0
A list of engine indices to use for VTOL. Set to 'all' to use all downward-pointing engines.
Use a list format, i.e. {0,1,2,3} to specify exact engine indices. Engines may be attached to
spin blocks for use with vector thrust, and will be rotated backwards when "UseVTOLBeneathSpeed"
is exceeded. If engines are on spin blocks, you must specify their indices if the AI is to
control their drive fraction. An spin block and engine control tool is available on the
FtD forums to assist in identifying indices.
For engines mounted to the hull, set each such engine to "Main".
You can (and should) set the drive fractions of each engine you want to use with this AI,
on the engine itself. The drive fraction on each engine specifies the maximum amount of
power you want to use on that engine.
    VTOLEngines = nil

## TERRAIN AVOIDANCE OPTIONS ##
These settings allow a vehicle to avoid terrain by temporarily increasing it's altitude (only!).
To avoid terrain by turning aside, see the TerrainAvoidanceWeight flocking setting.
0: Ignore terrain. Only recommended for spacecraft.
1: As normal aerial AI, just add CruiseAltitude to terrain when terrain is above sea level.
     This will respect "MinAltitude" and "MaxAltitude" constraints when possible.
2: Adjust altitude by minimum necessary to respect "MinAltitude" constraint.
    TerrainAvoidanceStrategy = 1
The minimum relative altitude a vehicle will maintain above terrain. Higher priority than MaxAltitude.
    MinAltitude = 50
The maximum sea-level altitude the vehicle will attempt when avoiding terrain.
    MaxAltitude = 400
A set of multipliers on current velocity; how far we look ahead to avoid terrain.
We'll look ahead at each of these points to see if terrain is in the way.
    TerrainLookahead = {0,1,2,4}
Cap vehicle speed at this value when a possible collision is sensed.
    MaxTerrainSpeed = 5

## WATER START OPTIONS ##
Water start will deploy balloons when the center of mass is lower than this.
If your vehicle has no water start balloons, set this to some large negative value.
    DeployAlt = 5
Water start will release balloons when the center of mass is higher than this.
    ReleaseAlt = 15
After water start has been triggered, will disable movement until this altitude has been reached.
    EnableEnginesAlt = 10

## COLLISION AVOIDANCE OPTIONS ##
Collision detection and avoidance tries to detect and optionally avoid possible collisions
with the target (only!). The "TThreshold" parameter is how distant in time (in seconds)
a potential collision will be considered dangerous. The "Height"
parameter is how far above and below the vehicles current altitude it will check.
The "Angle" parameter is the desired behavior on detecting a collision. Setting it to "0"
means to go directly towards the enemy. "90" means go at right angles, and "180" means run away.
    CollisionTThreshold = 2
    CollisionDetectionHeight = 20
    CollisionAngle = 20

## FLOCKING OPTIONS ##
Start by reading more about flocking behavior here: http://www.red3d.com/cwr/boids/
Flocking is an advanced behavior that allows formation-like flying and collision avoidance.
Flocking controls azimuth only (not altitude).
Start using flocking by setting "NeighborRadius" to a positive number (try 300, for example)
    NeighborRadius = 0  -- in meters
For alignment and cohesion, ignore craft that are going below this speed (in m/s)
    IgnoreBelowSpeed = 0
Set FlockWithBlueprintNames = 'all' to use alignment and cohesion with all friendly craft
Enclose a comma-delimited list of vehicle names in curly braces {} to use alignment and cohesion
with only craft with those names. Names given will match with any vehicle that starts with that
combination of letters, for example {'Rapier'} will match with any vehicle named
'Rapier', 'Rapier-A', 'Rapier-E', etc.
    FlockWithBlueprintNames = 'all'
The weight given to alignment, or the desire to match headings with friendly craft
    AlignmentWeight = 1
The weight given to cohesion, or the desire to come closer to friendly craft
    CohesionWeight = 1
The weight given to separation, or the desire to avoid coming too close to vehicles.
Will also avoid enemy vehicles within CollisionDetectionHeight
    SeparationWeight = 1.5
A cohesion weight for injured vehicles. May be useful for "medic ships"
    InjuredWeight = 0
A cohesion weight for friendly vehicles at ranges outside NeighborRadius.
    LongRangeWeight = .5
If positive, this weight will examine 8 compass directions around the vehicle at distance
NeighborRadius, and if there is a danger of collision (as determined by the "MinAltitude" setting),
will attempt to angle the aircraft away from the collision.
It will still attempt to avoid terrain by gaining altitude as appropriate.
    TerrainAvoidanceWeight = 0
The weight given to the "normal" target-specific behaviors of attack, escape, etc.
    TargetWeight = 1

## "DOGFIGHTING" OPTIONS (changing behavior based on information about the target) ##
Whether or not to try to match our altitude to the target.
 If set to "false", will use CruiseAltitude during combat.
    MatchTargetAltitude = false
Range within which to attempt altitude matching. Beyond this range, uses CruiseAltitude.
    MatchAltitudeRange = 300
Altitude above (or below, if negative) the target the vehicle will attempt to attain.
    Altitude will still be constrained by "MinAltitude" and "MaxAltitude" terrain avoidance constraints
MatchAltitudeOffset = 50
    The minimum altitude the aircraft will go to when matching altitude
MinMatchingAltitude = 100

## VECTOR THRUST OPTIONS ##
Vector thrust means placing jets on spin blocks, potentially allowing much more powerful yaw, pitch,
or roll authority on a plane. Turn this option on by specifying a number of spinners
to use with VTSpinners. Use VTSpinners = 'all' to use all spinners, or supply a comma-delimited
list like this: {0,1,2}, where the index of a spinner is the order it was placed down on your ship,
starting the count from index 0. Spinners placed vertically are for yaw. Spinners placed horizontally
control roll and pitch (so I suggest placing them farther to the left and right of your CoM).
Note that if you stop/start the AI, you will probably want to reload your aircraft so
the AI can correctly re-learn the default orientation of each spinner.
Also see the "ExcludeSpinners" setting to specify a list of spinners to exclude, if this is set to 'all'.
    VTSpinners = nil
The maximum angle to set any particular spinner to. Roll and pitch angles are cumulative. The
maximum angle you can specify is 90 degrees. Set to 0 to not use that particular kind of vector
thrust. You can use negative angles to reverse the angle direction.
The VTOL angle is the downwards angle VTOL spinners will be set to in VTOL mode.
    MaxVTAngle = {30,30,30,90} -- yaw, roll, pitch, VTOL
These next two settings probably don't need to be messed with. They set the interaction between
a decision to move a particular direction and how powerfully the spinner angles respond to that impulse.
These are roughly similar to the parameters of a PI-controller.
    VTProportional = {1,1,1} 
    VTDelta = {1/12,1/12,0}
The set of VTOL spinners to use when in VTOL mode. Set to 'all' to select all spinners capable of
pointing downwards. Otherwise, choose a list of spinner indices like: {0,1,2,3}. You can
use a tool (also provided) to help you select the correct indices.
VTOL spinners will angle downwards at their given MaxVTAngle when in VTOL mode, otherwise
they will point backwards and operate as normal vectored thrust.
Also see the "ExcludeSpinners" setting to specify a list of spinners to exclude, if this is set to 'all'.
    VTOLSpinners = 'all'

## MISSILE AVOIDANCE OPTIONS ##
Try to avoid missiles. Set "WarningMainframe = -1" to ignore missiles. Set it to the index
of the mainframe with missile warners, if you have missile warners. If you only have one mainframe,
then WarningMainframe = 0.
    WarningMainframe = -1
If missiles are within this time-to-target threshold, ship will try to run away from them
    RunAwayTTT = 2
The angle at which to dodge missiles. You will usually want this at 180 to run away,
but you can run *towards* the missiles by setting this to 0, or at right angles by setting it to 90, etc.
    RunAngle = 180
If missiles get within this time threshold, ship will try a specialized dodging maneuver.
    DodgeTTT = 1
Usually between 10 and 90 degrees, describes how close to the desired roll angle the ship should
be before pitching up during the roll. Set to larger values if you don't care about steady rolls
and just want to get out of the way of a missile ASAP.
    DodgingRollTolerance = 60
The speed (in m/s) your ship is usually able to go when dodging missiles.
    NormalSpeed = 100
The radius (usually half the length of your ship, in meters) of your ship
    CraftRadius = 21
The throttle value you want to use when dodging missiles OR collisions
    DodgingSpeed = 5
You shouldn't need to mess with this. Helps decide when a missile might be dangerous.
    DangerRadius = 20

## ADVANCED OPTIONS (You shouldn't need to mess with these most of the time.) ##
0 for water mode, 1 for land mode, 2 for air mode. You can try it for non-aircraft, but YMMV.
    MODE = 2
This AI automatically clamps "MaxElevationAngle" according to how near the aircraft is to it's target
altitude to prevent overshooting (i.e. going into space) when changing altitude. Higher values of
"AltitudeClamp" decrease this effect, allowing steeper climbs. Lower values help prevent overshooting.
AltitudeClamp also controls how gradually airships approach thier target altitude.
    AltitudeClamp = .2
Damping parameters to mitigate wobbling in high-performance vehicles. Higher values can help
control wobbling, lower values allow faster turns but may overshoot and cause wobble more often.
    PitchDamping = 90
    YawDamping = 90
    RollDamping = 45
The default roll angle to maintain. Set it to 90 to have the vehicle usually on it's side.
You may want to set "AngleBeforeTurn" to 0 if you do this, so you never yaw.
    DefaultRollAngle = 0
There are three kinds of ways this AI can use to control forward speed (i.e., main drive):
MainDriveControlType = 0: Uses standard throttle control. Most ships will want this option.
MainDriveControlType = 1: Use forward (only) propulsion balancing. This may help to compensate
for damage to thrusters, or unbalanced thruster placement compared to the center of mass.
WARNING: Propulsion balancing can be very unreliable. YMMV.
    MainDriveControlType = 2: Control throttle by varying drive fraction. The AI will try to guess
which thrusters are forward thrusters. It will EXCLUDE any thrusters on spin blocks, so vectored
thrust continues to operate at full power for greatest possible maneuverability.
    MainDriveControlType = 0
The minimum speed you want your aircraft to go. If your speed ever goes to less than this,
the throttle will be temporarily set to 5 until you are above the minimum speed.
    MinimumSpeed = 0
The maximum speed you want your aircraft to go. If this is exceeded, the AI will temporarily
reduce throttle to 20% of maximum. Some tournaments have specified a maximum speed.
    MaximumSpeed = 999
The maximum roll angle your vehicle will take during a roll. Set this lower if your vehicle is
going into the sea during a roll.
    MaxRollAngle = 120
How close you want your aircraft to be to the "desired" roll angle before pitching up. Set this too
high and you may gain or lose too much altitude during a turn.
    RollTolerance = 30
Set this to true if you are interested in knowing what the AI is "thinking" as it operates.
    DebugMode = false
Prefer to approach the enemy on only one side of the aircraft or not. Set AngleBeforeTurn positive'
or negative to choose the side.
    UsePreferredSide = false
Predictive guidance tries to guess where the enemy will be by the time the aircraft will reach it,
and head in that direction instead of the current position of the enemy. Set this to "true"
when you don't care about pointing directly at the enemy, and you wish to prioritize "cutting the
circle" on a fast orbiter, or if you want to ram the enemy. Set to "false" if it is more important
to orient yourself in relation to the enemies current position during the attack run, for example if
all your weapons are in a fixed-forward firing position. Future altitude is also estimated, for
when "MatchTargetAltitude" is true. 
    UsePredictiveGuidance = false
AltitudeOffset is useful when you wish to create multiple vehicles of the same type, and you
don't want to individually set all the various altitude settings so they won't crash into each
other. This value adjusts CruiseAltitude, MaxAltitude, MatchAltitudeOffset, and MinMatchingAltitude
simultaneously. You can also give it two table values, e.g. {0,100}. This will randomly generate
an offset in the range [0,100], so your aircraft may be less likely to collide with other's of it's kind.
    AltitudeOffset = 0
AngleOfEscape is the angle relative to the enemy that is set when going from "attack mode" to "escape mode".
It is set once when entering escape mode, and does not adjust to enemy movements.
By default, it is set to AngleBeforeTurn (and follows the same rules). You can set it to anything
you like though, for example "AngleOfEscape = 90" will turn off at 90 degrees, or
"AngleOfEscape = 180" will have the vehicle try to turn around.
    AngleOfEscape = AngleBeforeTurn
An additional option for helicopters, vector thrust, and VTOL, ExcludeSpinners allows you
to specify spinner indices to be excluded from use by this AI (if you use the 'all' option
for VTSpinners or VTOLSpinners). If used, this must be a comma-delimited list surrounded
by curly braces, for example "ExcludeSpinners = {0,5}"
    ExcludeSpinners = nil
