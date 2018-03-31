Introducing my version of an advanced aerial AI. This Lua script is intended as a drop-in replacement for the standard aerial AI, with many more parameters to set and far more robust altitude matching and terrain handling, as well as methods for mitigating the "wobbling" you can get in high-performance vehicles.

This AI also includes built-in water start AI, collision avoidance AI, "dogfighting" options, vector thrust options, missile avoidance, predictive interception, VTOL takeoff, flocking, and several other features. It can also be used to control naval vessels, helicopters, and many kinds of hovercraft and airships.

## Usage ##

Put a Lua box down somewhere on your vehicle (it's in the "Control" tab of the build menu). Use "Q" on the Lua box. Copy-and-paste (Ctrl-A,Ctrl-C) the Lua code there into the big edit box in the Lua box. Read the documentation, below. Choose appropriate AI settings for your vehicle. Click "Apply" to save the code and new settings. Watch how your vehicle behaves and continue to tweak the AI settings as desired until you get the behavior you want.
Note that when you paste this code in, you will only see a bit less than 500 lines of code because that's how many the Lua box can display. However, all the code should be there.

#### To use this AI for orbiters:

An "orbiter" is a vehicle that continually circles their target instead of making strafing runs. To do this, set "AngleBeforeTurn" to a larger value, usually between 40-90 degrees depending on your vehicles maneuverability. The smaller the number, the tighter the orbit, and the more maneuverable your vehicle will need to be. Most orbiters use only yawing to turn, so you will also usually need to set "AngleBeforeRoll" to 180 so your vehicle won't try to roll.

#### To use this AI for naval and land vehicles:

* Set "AngleBeforeRoll" to 180, so your ship yaws to turn all the time.
* Set "CruiseAltitude" to the typical altitude of your vehicle. This isn't strictly necessary to do but it helps the AI not be confused.
* Set "DeployAlt" to some large negative value so it won't try to do a water start and thus turn off the engines.
* Set "Mode" to 0 or 1 for water/land mode controls respectively.
* You will probably want to make your vehicle an orbiter (see above) so it won't try to strafe the enemy and collide with them.
* Set other parameters as desired to control the behavior of the ship. Good luck!

#### To use this AI for hovercraft, helicopters, or airships:

* Set "AngleBeforeRoll" to 180, so your ship yaws to turn all the time.
* Set "MaxElevationAngle" to 0 so that your ship won't pitch up or down.
* To use thrusters to control altitude, set UseAltitudeJets = true.
* To use helicopter blades to control altitude, set UseSpinnters = true, and set min and max helicopter blade speeds to an appropriate value for your helicopter.
* You may gain some benefit from using smaller values of "PitchDamping". If the default value isn't working, try something like 20. This will make a hovercraft more sensitive to deviations in pitch.

## BASIC OPTIONS ##
When the vehicle is within "AngleBeforeRoll" degrees of its target, it will try to yaw towards
(or away!) from its target such that its nose is pointed "AngleBeforeTurn" degrees away from it.
If UsePreferredSide = true, use a positive value of AngleBeforeTurn to approach targets on the right,
a negative value to approach targets on the left. Otherwise, always use a positive value. 

    AngleBeforeTurn = 10

If the vehicle is at greater than this angle from it's target, it will try to roll towards it.
(and stop rolling when AngleBeforeTurn is reached).
Set this to 180 to NEVER use rolling to turn the vehicle.

    AngleBeforeRoll = 30

Outside of this distance, the vehicle will try to close on it's target.
NOTE: All distances are measured along the ground!

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

Maximum (straight-line) distance AI will try to permit vehicle to wander from it's target. This is mostly useful for tournaments that impose a distance limit, and vehicles that might run away for various reasons (i.e. missile evasion). After the vehicle goes beyond this limit, will impose an increasingly greater steering force to try to get the vehicle pointed back on target.

    MaxDistance = 1000

Maximum positive and negative pitch angles AI will attempt to use while climbing or descending (respectively).
Also limits yaw when vehicle is rolled on it's side.

    MaxPitch = 30
    MinPitch = -15

## SPEED CONTROL

This AI has various options for controlling speed during various conditions of it's flight. Speed may either be controlled via an absolute value in meters/second, or a throttle value, which is a 0-1 value expressed as a percentage of the drive's power output.

The minimum speed (in m/s) you want your vehicle to go. If your speed ever goes to less than this (perhaps due to low throttle settings and/or battle damage), the throttle will be temporarily set to maximum until you are above the minimum speed.

    MinimumSpeed = 0

The maximum speed you want your aircraft to go. If this is exceeded, the AI will temporarily
reduce throttle to 10% of maximum. Some tournaments specify a maximum speed, making this limit helpful.

    MaximumSpeed = 999

Maximum throttle we'll cruise at when no enemies are present.

    CruiseThrottle = 1

Throttle we'll use when engaged with enemies.

    AttackRunThrottle = 1 -- when on an attack run (angle to target is less than "AngleBeforeRoll") inside of ClosingDistance 
    EscapeThrottle = 1    -- triggered after coming within "AbortRunDistance" until "AttackRunDistance" is reached
    RollingThrottle = 1   -- when rolling aircraft to face enemy (angle to target is greater than "AngleBeforeRoll")
    ClosingThrottle = 1   -- when outside of ClosingDistance AND not in a roll

## HELICOPTER OPTIONS

A list of subconstruct ID's for the individual spinners you want to use for altitude control on a helicopter.

    HeliSpinners = {}

The helicopter blade speed to use when lowering altitude. Ranges between -30 to 30.
Use AltitudeClamp to control how smoothly these speeds are transitioned between
as your helicopter approaches it's target altitude.

    MinHelicopterBladeSpeed = 10

The helicopter blade speed to use when gaining altitude. Ranges between -30 to 30.

    MaxHelicopterBladeSpeed = 30


## TERRAIN AVOIDANCE OPTIONS

Allow a vehicle to avoid terrain by temporarily increasing it's altitude.

    AvoidTerrain = true

The minimum relative altitude a vehicle will maintain above terrain. Higher priority than MaxAltitude.

    MinAltitude = 50

The maximum sea-level altitude the vehicle will attempt.

    MaxAltitude = 400

A set of multipliers on current velocity; how far we look ahead to avoid terrain.
We'll look ahead at each of these points to see if terrain is in the way. Most vehicles won't need to change these values.

    TerrainLookahead = {0,1,2,4}

Cap vehicle throttle at this value when a possible collision is sensed.

    MaxTerrainThrottle = 1

Use the advanced steering system to avoid difficult terrain by flying around, instead of over. Not guaranteed to work as this weight governs a priority that must be balanced with other steering priorities the vehicle may have, but sometimes helps to avoid difficult mountains.
If positive, this weight will examine 8 compass directions around the vehicle at distances determined by velocity and TerrainLookahead, and if there is a danger of collision (as determined by the "MinAltitude" setting),
will attempt to angle the aircraft away from the terrain.
It will still attempt to avoid terrain by gaining altitude if needed.

    TerrainAvoidanceWeight = 1

## WATER START OPTIONS

Water start will deploy balloons when the center of mass is lower than this.
If your vehicle has no water start balloons, set this to some large negative value.

    DeployAlt = 5

Water start will release balloons when the center of mass is higher than this.

    ReleaseAlt = 15

After water start has been triggered, will disable movement until this altitude has been reached.

    EnableEnginesAlt = 10

## COLLISION AVOIDANCE OPTIONS

Collision detection and avoidance tries to detect and avoid possible collisions with other vehicles using the advanced
steering system.
The "CollisionTThreshold" parameter is how distant in time (in seconds)
a potential collision will be considered dangerous -- the less maneuverable your vehicle, the higher you may want to set this.
Set to 0 if you don't care about collisions.

    CollisionTThreshold = 5

How large your vehicle is. Usually set it to half the longest dimension.

    CraftRadius = 27

How much buffer space to give between itself and other vehicles. Higher values are safer.

    BufferSize = 50

We don't have any way to know the size of an enemy in Lua, so you can estimate it here. Set this value to half the largest dimension of the enemies you expect to fight.

    EnemyRadius = 50

Use the advanced steering system to avoid collisions with friendly vehicles. Can be expensive if you have many friendly vehicles in play.

    AvoidFriendlies = true

Avoid collisions with your target. Set to "false" if you have a melee vehicle.

    AvoidTarget = true

Avoid collisions with other enemies. Can be expensive if there are many enemy vehicles in play.

    AvoidOtherEnemies = true

The weight, or priority, to set on avoiding collisions for the advanced steering system. Set to 0 if you don't care. Any large value (10 is fine in most cases) will set a high priority on avoiding collisions.

    AvoidanceWeight = 10

## STEERING OPTIONS

The advanced steering system governs almost all changes in azimuth, or heading, of a vehicle.
This system allows the vehicle designer to set different weights, or priorities, on various activities a vehicle AI might care about, from attacking the target to dodging missiles. To understand how steering works, it will be helpful to read these links: 

* [Steering Behaviors For Autonomous Characters](http://www.red3d.com/cwr/steer/gdc99/)
* [Boids](http://www.red3d.com/cwr/boids/)

Flocking is an advanced behavior that allows formation-like flying and collision avoidance with friendly vehicles in the "flock".
Start using flocking by setting "NeighborRadius" to a positive number (try 300, for example). This will tell your vehicle to "notice" other freindly vehicles within this radius and form a flock with them.

    NeighborRadius = 0  -- in meters

For alignment and cohesion, ignore craft that are going below this speed (in m/s). Useful if one of the vehicle becomes damaged and has to drop out of formation.

    IgnoreBelowSpeed = 0

Set FlockWithBlueprintNames = 'all' to use alignment and cohesion with all friendly craft
Enclose a comma-delimited list of vehicle names in curly braces {} to use alignment and cohesion
with only craft with those names. Names given will match with any vehicle that starts with that
combination of letters, for example {'Rapier'} will match with any vehicle named
'Rapier', 'Rapier-A', 'Rapier-E', etc.

    FlockWithBlueprintNames = 'all'  -- or, for example {'Rapier', 'Cutlass'}, etc.

The weight given to alignment, or the desire to match headings with friendly craft. Usually 1 or 0 to turn this on or off, but other values are fine too and will change steering priorities accordingly.

    AlignmentWeight = 1

The weight given to cohesion, or the desire to come closer to friendly craft.

    CohesionWeight = 1

The weight given to separation, or the desire to avoid coming too close to vehicles.

    SeparationWeight = 1

A cohesion weight for injured vehicles. May be useful for "medic ships"

    InjuredWeight = 0

A cohesion weight for friendly vehicles at ranges outside NeighborRadius. It isn't necessary to use this, but it will help vehicles "find each other" if they get separated.

    LongRangeWeight = .5

The weight given to the "normal" target-specific behaviors of attack, escape, etc. Usually you should leave this at 1. A value of 0 means it ignores enemies when it comes to navigation.

    TargetWeight = 1

## "DOGFIGHTING" OPTIONS

"Dogfighting" for this AI means changing behavior of the AI according to behavior of the enemy target.

Whether or not to try to match our altitude to the target.
If set to "false", will use CruiseAltitude during combat.

    MatchTargetAltitude = false

Range within which to attempt altitude matching. Beyond this range, uses CruiseAltitude.

    MatchAltitudeRange = 300

Altitude in meters above (or below, if negative) the target the vehicle will attempt to attain.
Altitude will still be constrained by "MinAltitude" and "MaxAltitude" terrain avoidance constraints.

    MatchAltitudeOffset = 0

The minimum altitude the aircraft will go to when matching altitude

    MinMatchingAltitude = 100

Use vehicle roll to try to "broadside" a target. This is very different to a naval broadside (which uses yaw). This is intended for vehicles which have weapons that don't have good elevation control. Set "BroadsideWithin" to a positive number to start broadsiding when the target is within a certain range -- usually this is similar to the effective range of your weapon. "BroadsideAngle" then controls the roll angle relative to the target you want your vehicle to take. "0" means you want either side of your vehicle pointed at the enemy. "90" means the bottom of your vehicle, "-90" the top. Roll angles will respect the "MaxRollAngle" setting, in the advanced options.

    BroadsideWithin=0  -- in meters
    BroadsideAngle=0

## MISSILE AVOIDANCE OPTIONS

If you allow it to, this AI will try to avoid enemy missiles by running away from them using the advanced steering system. This is generally only useful on maneuverable vehicles (so that it can turn to run away in time), and fast vehicles (so it can buy itself time or just completely outrun missiles).

Set "WarningMainframe = -1" to ignore missiles. Set it to the index
of the mainframe with missile warners, if you have missile warners. If you only have one mainframe,
then WarningMainframe = 0.

    WarningMainframe = -1

If missiles are within this time-to-target threshold, vehicle will try to run away from them.

    RunAwayTTT = 4

You shouldn't need to mess with this. Helps decide when a missile might be dangerous.

    DangerRadius = 20

The speed (in m/s) your ship is usually able to go when dodging missiles.

    NormalSpeed = 100

The weight (or priority) to give dodging missiles for the advanced steering system. This is on a *per missile* basis, so a large missile barrage is considered very high-priority. Also, the nearer to impact a missile is, the more importance it is given. "0" will ignore missiles, larger values will give running away increasingly high priority (2 works fine in my tests).

    DodgingWeight = 2

## VECTOR THRUST OPTIONS

Vector thrust means placing jets on spin blocks, potentially allowing much more powerful yaw, pitch,
or roll authority on a vehicle. Turn this option on by specifying a number of sub-construct IDs
to use with VTSpinners. Use VTSpinners = 'all' to use all spinners, or supply a comma-delimited
list like this: {0,1,2}. Spinners placed vertically are for yaw. Spinners placed horizontally
control roll and pitch (so I suggest placing them farther to the left and right of your CoM).
Also see the "ExcludeSpinners" setting to specify a list of spinners to exclude, if this is set to 'all'.

    VTSpinners = nil

The maximum angle to set any particular spinner to. Roll and pitch angles are cumulative (so make sure they sum to at most 90 degrees). The maximum angle you can specify is 90 degrees. Set to 0 to not use that particular kind of vector
thrust. You can use negative angles to reverse the angle direction.
The VTOL angle is the downwards angle VTOL spinners will be set to in VTOL mode (see next section).

    MaxVTAngle = {30,30,30,90} -- yaw, roll, pitch, VTOL

These next two settings may not need to be changed. They control the interaction between
a decision to move a particular direction and how powerfully the spinner angles respond to that impulse.
These are roughly similar to the parameters of a PID-controller.

    VTProportional = {1,1,1} 
    VTDelta = {1/12,1/12,0}

## VTOL/HOVER OPTIONS

While not yet as extensive as the hover options afforded by some other scripts, this AI does allow for control of hover jets to control altitude, as well as VTOL takeoff at low altitudes (i.e. decks of carriers and the like).

Use jets pointing up or down to assist in controlling altitude. 

    UseAltitudeJets = false

An option for airships is to allow the AI to manually control downward-pointing engine drive
fractions. This option can be turned off above a certain speed (in m/s) for VTOL takeoffs.
Set "UseVTOLBeneathSpeed" to a large number to use VTOL all the time

    UseVTOLBeneathSpeed = 0

A list of engine indices to use for VTOL. Set to 'all' to use all downward-pointing engines.
Use a list format, i.e. {0,1,2,3} to specify exact engine indices. Engines may be attached to
spin blocks for use with vector thrust, and will be rotated backwards for normal use when "UseVTOLBeneathSpeed"
is exceeded. If engines are on spin blocks, you must specify their indices if the AI is to
control their drive fraction.
For engines mounted to the hull, set each such engine to "Main".
You can (and should) set the drive fractions of each engine you want to use with this AI,
on the engine itself. The drive fraction on each engine specifies the maximum amount of
power you want to use on that engine.

    VTOLEngines = nil

The set of VTOL spinners to use when in VTOL mode. Set to 'all' to select all spinners capable of
pointing downwards. Otherwise, choose a list of spinners ub-construct indices like: {0,1,2,3}.
VTOL spinners will angle downwards at their given MaxVTAngle when in VTOL mode, otherwise
they will point backwards and operate as normal vectored thrust.
Also see the "ExcludeSpinners" setting to specify a list of spinners to exclude, if this is set to 'all'.

    VTOLSpinners = 'all'


## ADVANCED OPTIONS

These are options most users shouldn't need to change.

0 for water mode, 1 for land mode, 2 for air mode. You can try it for non-aircraft, but YMMV.

    MODE = 2

This AI automatically clamps "MaxPitch" and "MinPitch" according to how near the aircraft is to it's target
altitude to prevent overshooting when changing altitude. Higher values of
"AltitudeClamp" decrease this effect, allowing steeper changes in altitude. Lower values help prevent overshooting.
AltitudeClamp also controls how gradually airships approach thier target altitude when using altitude jets.

    AltitudeClamp = .2

Damping parameters to mitigate wobbling in high-performance vehicles. Higher values can help
control wobbling, lower values allow faster turns but may overshoot and cause wobble more often.

    PitchDamping = 90
    YawDamping = 90
    RollDamping = 45

The default roll angle to maintain. Set it to 90 to have the vehicle usually on it's side.
You may want to set "AngleBeforeTurn" to 180 if you do this, so you never yaw.

    DefaultRollAngle = 0

There are three kinds of ways this AI can use to control forward speed (i.e., main drive):
* MainDriveControlType = 0: Uses standard throttle control. Most ships will want this option.
* MainDriveControlType = 1: Use forward (only) propulsion balancing. This may help to compensate
for damage to thrusters, or unbalanced thruster placement compared to the center of mass.
WARNING: Propulsion balancing can be very unreliable. YMMV.
* MainDriveControlType = 2: Control throttle by varying drive fraction. The AI will try to guess
which thrusters are forward thrusters. It will EXCLUDE any thrusters on spin blocks, so vectored
thrust continues to operate at full power for greatest possible maneuverability.

    MainDriveControlType = 0

The maximum roll angle your vehicle will take during a roll. Set this lower if your vehicle is
going into the sea during a roll.

    MaxRollAngle = 120

How close you want your aircraft to be to the "desired" roll angle before pitching up. Set this too
high and you may gain or lose too much altitude during a turn.

    RollTolerance = 30

Set this to true if you are interested in knowing what the AI is "thinking" as it operates.

    DebugMode = false

Prefer to approach the enemy on only one side of the aircraft or not. Set AngleBeforeTurn positive
or negative to choose the side.

    UsePreferredSide = false

Predictive guidance tries to guess where the enemy will be by the time the aircraft will reach it,
and head in that direction instead of the current position of the enemy. Set this to "true"
when you don't care about pointing directly at the enemy, and you wish to prioritize "cutting the
circle" on a fast orbiter, or if you want to ram the enemy. Set to "false" if it is more important
to orient yourself in relation to the enemies current position during the attack run, for example if
all your weapons are in a fixed-forward firing position.

    UsePredictiveGuidance = true

AltitudeOffset is useful when you wish to create multiple vehicles of the same type, and you
don't want to individually set all the various altitude settings so they won't crash into each
other. This value adjusts CruiseAltitude, MaxAltitude, MatchAltitudeOffset, and MinMatchingAltitude
simultaneously. You can also give it two table values, e.g. {0,100}. This will randomly generate
an offset in the range between 0-100, so your aircraft may be less likely to collide with other's of it's kind.

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

Whether to orbit the spawn point when there are no enemies, or just fly off in a straight line

    OrbitSpawn = true
