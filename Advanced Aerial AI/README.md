Introducing my version of an advanced aerial AI. This Lua script is intended as a drop-in replacement for the standard aerial AI, with many more parameters to set and far more robust altitude matching and terrain handling, as well as PID controllers for more stable flight.

This AI also includes built-in water start AI, collision avoidance AI, "dogfighting" options, vector thrust options, missile avoidance, predictive interception, VTOL takeoff, flocking, and several other features. It can also be used to control naval vessels, helicopters, and many kinds of hovercraft and airships.

## Usage

Put a Lua box down somewhere on your vehicle (it's in the "Control" tab of the build menu). Use "Q" on the Lua box. Copy-and-paste (Ctrl-A,Ctrl-C) the Lua code there into the big edit box in the Lua box. Read the documentation, below. Choose appropriate AI settings for your vehicle. Click "Apply" to save the code and new settings. Watch how your vehicle behaves and continue to tweak the AI settings as desired until you get the behavior you want.
Note that when you paste this code in, you will only see a bit less than 500 lines of code because that's how many the Lua box can display. However, all the code should be there.

The AI mainframe must be set to "On" for this AI to be enabled. Any other setting will allow the normal AI behaviors (allowing, for example, patrol mode to work).

## Frequently Asked Questions

<details>
<summary>How to use this AI for orbiters?</summary>

An "orbiter" is a vehicle that continually circles their target instead of making strafing runs. To do this, set "AngleBeforeTurn" to a larger value, usually between 40-90 degrees depending on your vehicles maneuverability. The smaller the number, the tighter the orbit, and the more maneuverable your vehicle will need to be. Most orbiters use only yawing to turn, so you will also usually need to set "AngleBeforeRoll" to 180 so your vehicle won't try to roll.
</details>

<details>
<summary>How to use this AI for ships, submarines, and land vehicles?</summary>

* Set `AngleBeforeRoll = 180`, so your ship yaws to turn all the time.
* Set `CruiseAltitude` to the typical altitude of your vehicle. This will be particularly helpful for submarines.
* Set `DeployAlt` to some large negative value so it won't try to do a water start and thus turn off the engines.
* Set `DriveMode` to `0` or `1` for water/land mode controls respectively.
* You will probably want to make your vehicle an orbiter (see above) so it won't try to strafe the enemy and collide with them.
* If you use large rudders, the vector thrust/large rudder options can optionally control them
* If you use hydrofoils, set HydrofoilMode to 1 or 2 as appropriate
* Set other parameters as desired to control the behavior of the vehicle. Good luck!
</details>

<details>
<summary>How to use this AI for hovercraft, helicopters, or airships?</summary>

* Set `AngleBeforeRoll = 180`, so your ship yaws to turn all the time.
* Set `MaxElevationAngle = 0` so that your ship won't pitch up or down.
* To use thrusters to control altitude, set `UseAltitudeJets = true`.
* To use helicopter blades to control altitude, choose some spinners to set in `HeliSpinners` and/or some dediblades to set in `HeliDediblades`, or set either of these options to `'all'`, and set min and max helicopter blade speeds to an appropriate value for your helicopter.
</details>

<details>
<summary>How to use this AI for ramming/kamikaze vehicles?</summary>

* In the dogfighting options, set `MatchTargetAltitude = true` and `MatchAltitudeOffset = 0`. This will make the AI try to match altitudes with the target vehicle. `MatchAltitudeRange` will control how far away the AI will try to do this.
* Set `UsePredictiveGuidance = true` so the vehicle aims where the target will be, not where it is.
* Set `AngleBeforeTurn = 0` so that the vehicle heads directly towards the enemy.
* Set `AvoidTarget = false` OR `CollisionTThreshold = 0` so the collision avoidance code does not try to steer away from a collision.
</details>

<details>
<summary>How to improve frames per second?</summary>

This AI does a lot of "thinking", potentially considering information about every vehicle and enemy missile in play, many times per second. Under some conditions this can make the AI slow to run, reducing FPS -- particularly this can occur in "swarm builds" in which many copies of this AI are operating in parallel. There are several options which can be used to mitigate this problem:

* Turn off missile avoidance (set `WarningMainframe = -1`) if your vehicle doesn't need it.
* Turn off collision detection if you don't think it will be helpful (set `AvoidFriendlies` and/or `AvoidOtherEnemies` to `false` as appropriate).
* Finally, and perhaps most importantly, increase `UpdateRate` to `2` or more. This will do the most to help, but it will slightly reduce the responsiveness of your vehicle. Your vehicle will still fly, but it won't update it's desired heading or altitude as often.
</details>

<details>
<summary>Where to find SubConstruct IDs?</summary>

SubConstruct IDs are needed for the `HeliSpinners`, `VTSpinners`, `VTOLSpinners`, and `ExcludeSpinners` options.
These IDs are attached to spinner blocks. You can find them in the lower-left hand corner of the Spinner GUI, as follows:

<p align="center">
<img src="http://i.imgur.com/P3bf6XT.jpg" alt="SubConstruct IDs" width="600"/>
</p>
</details>

## BASIC OPTIONS
When the vehicle is within `AngleBeforeRoll` degrees of its target, it will try to yaw towards
(or away!) from its target such that its nose is pointed `AngleBeforeTurn` degrees away from it.
If `UsePreferredSide = true`, use a positive value of `AngleBeforeTurn` to approach targets on the right,
a negative value to approach targets on the left. Otherwise, always use a positive value. 

```lua
AngleBeforeTurn = 10
```

If the vehicle is at greater angle than this from it's target, it will try to roll towards it.
(and stop rolling when `AngleBeforeTurn` is reached).
Set this to `180` to NEVER use rolling to turn the vehicle.

```lua
AngleBeforeRoll = 30
```

Outside of this distance, the vehicle will try to close on it's target.
NOTE: All distances are measured along the ground, in meters.

```lua
AttackRunDistance = 600
```

Within this distance, the vehicle will just try to go straight until `AttackRunDistance` is triggered.

```lua
AbortRunDistance = 300
```

Outside of this distance, the AI will try to intercept its target by attempting
to predict it's future position.

```lua
ClosingDistance = 1000
```

If `AttackRunDistance` hasn't been triggered within this time, force an attack run.

```lua
ForceAttackTime = 15
```

Altitude we will try to cruise at by default.

```lua
CruiseAltitude = 100
```

Maximum (straight-line) distance AI will try to permit vehicle to wander from it's target. This is mostly useful for tournaments that impose a distance limit, and vehicles that might run away for various reasons (i.e. missile evasion). After the vehicle goes beyond this limit, will impose an increasingly greater steering force to try to get the vehicle pointed back on target.

```lua
MaxDistance = 5000
```

Maximum positive and negative pitch angles AI will attempt to use while climbing or descending (respectively).
Also limits yaw when vehicle is rolled on it's side.

```lua
MaxPitch = 30
MinPitch = -15
```

## SPEED CONTROL

This AI has various options for controlling speed during various conditions of it's flight. Speed settings may either be expressed as a value in meters/second, or a throttle value, which is a 0-1 value representing as a percentage of the drives power output.

The minimum speed (in m/s) you want your vehicle to go. If your speed ever goes to less than this (perhaps due to low throttle settings and/or battle damage), the throttle will be temporarily set to maximum until you are above the minimum speed.

```lua
MinimumSpeed = 0
```

The maximum speed you want your aircraft to go. If this is exceeded, the AI will temporarily
reduce throttle to 10% of maximum. Some tournaments specify a maximum speed, making this limit helpful.

```lua
MaximumSpeed = 999
```

Maximum throttle we'll cruise at when no enemies are present.

```lua
CruiseThrottle = 1
```

Throttle we'll use when engaged with enemies.

```lua
AttackRunThrottle = 1 -- when on an attack run (angle to target is less than "AngleBeforeRoll") inside of ClosingDistance 
EscapeThrottle = 1    -- triggered after coming within "AbortRunDistance" until "AttackRunDistance" is reached
RollingThrottle = 1   -- when rolling aircraft to face enemy (angle to target is greater than "AngleBeforeRoll")
ClosingThrottle = 1   -- when outside of ClosingDistance AND not in a roll
```

## PID SETTINGS

These settings work similarly to the PID control block available for the normal FtD AI. PID control allows for much more stable flight (if desired). Most people may not need to edit these settings at all. The "P" setting is kP gain, or how powerfully the AI responds to a request for a change in angle. The "D" setting is Td derivative time and helps the AI smooth out overcorrections. Most of the time, the other settings won't need to be changed at all. Note that vector thrust is not controlled by PID, but a separate control mechanism.

```lua
--                   P,        D,       I,    OutMax,   OutMin,    IMax,    IMin
yawPIDData      = {0.2,     0.05,     0.0,         1,       -1,       1,      -1}
rollPIDData     = {0.2,     0.05,     0.0,         1,       -1,       1,      -1}
pitchPIDData    = {0.2,     0.02,     0.0,         1,       -1,       1,      -1}
```

My thanks to Draba, goduranus, and CP75 for their previous work on PID controllers for FtD, which this code also uses.

## HELICOPTER OPTIONS

A list of subconstruct ID's for the individual spinners you want to use for altitude control on a helicopter. Use `'all'` if you want to use all spinners. These do NOT include Dediblades; use the next setting for that.

```lua
HeliSpinners = {}
```

A list of ID's for the individual dediblades you want to use for altitude control on a helicopter. Use `'all'` if you want to use all dediblades. Right now you'll have to guess a bit to test the index; they are usually numbered from `0` starting from the first dediblade you place on the hull.

```lua
HeliDediblades = {}
```

The helicopter blade speeds to use for controlling altitude. These should range between `-30` to `30`.
Use AltitudeClamp to control how smoothly these speeds are transitioned between
as your helicopter approaches it's target altitude.

```lua
MinHelicopterBladeSpeed = 10 -- when lowering altitude
MaxHelicopterBladeSpeed = 30 -- when gaining altitude
```

## TERRAIN AVOIDANCE OPTIONS

Allow a vehicle to avoid terrain by temporarily increasing it's altitude.

```lua
AvoidTerrain = true
```

The minimum relative altitude a vehicle will maintain above terrain. Higher priority than MaxAltitude.

```lua
MinAltitude = 50
```

The maximum altitude above sea-level the vehicle will attempt.

```lua
MaxAltitude = 400
```

A set of multipliers on current velocity; how far we look ahead to avoid terrain.
We'll look ahead at each of these points to see if terrain is in the way. Most vehicles won't need to change these values.

```lua
TerrainLookahead = {0,1,2,4}
```

Cap vehicle throttle at this value when a possible collision is sensed.

```lua
MaxTerrainThrottle = 1
```

Use the advanced steering system to avoid difficult terrain by flying around, instead of over. Not guaranteed to work as this weight governs a priority that must be balanced with other steering priorities the vehicle may have, but sometimes helps to avoid difficult mountains.
If positive, this weight will examine 8 compass directions around the vehicle at distances determined by velocity and `TerrainLookahead`, and if the terrain is judged to be too high,
will attempt to angle the aircraft away from the terrain.
It will still attempt to avoid terrain by gaining altitude if needed.

```lua
TerrainAvoidanceWeight = 1
```

## WATER START OPTIONS

Water start will deploy balloons when the center of mass is lower than this.
If your vehicle has no water start balloons, set this to some large negative value.

```lua
 DeployAlt = 5
```

Water start will release balloons when the center of mass is higher than this.

```lua
ReleaseAlt = 15
```

After water start has been triggered, will disable movement until this altitude has been reached.

```lua
EnableEnginesAlt = 10
```

## COLLISION AVOIDANCE OPTIONS

Collision detection and avoidance tries to detect and avoid possible collisions with other vehicles using the advanced
steering system. The AI will always focus on avoiding the most imminent collision threat.

The `CollisionTThreshold` parameter is how distant in time (in seconds)
a potential collision will be considered dangerous -- the less maneuverable your vehicle, the higher you may want to set this.
Set to `0` if you don't care about collisions.

```lua
CollisionTThreshold = 5
```

How large your vehicle is. Usually set it to half the longest dimension.

```lua
CraftRadius = 27
```

How much buffer space to give between itself and other vehicles. Higher values are safer.

```lua
BufferSize = 50
```

We don't have any way to know the size of an enemy in Lua, so you can estimate it here. Set this value to half the largest dimension of the enemies you expect to fight.

```lua
EnemyRadius = 50
```

Use the advanced steering system to avoid collisions with friendly vehicles. Can be expensive to calculate if you have many friendly vehicles in play.

```lua
AvoidFriendlies = true
```

Avoid collisions with your target. Set to `false` if you have a melee vehicle.

```lua
AvoidTarget = true
```

Avoid collisions with other enemies. Can be expensive to calculate if there are many enemy vehicles in play.

```lua
AvoidOtherEnemies = true
```

The weight, or priority, to set on avoiding collisions for the advanced steering system. Set to `0` if you don't care. Any large value (`10` is fine in most cases) will set a high priority on avoiding collisions.

```lua
AvoidanceWeight = 10
```

## STEERING OPTIONS

The advanced steering system governs almost all changes in azimuth, or heading, of a vehicle.
This system allows the vehicle designer to set different weights, or priorities, on various activities a vehicle AI might care about, from attacking the target to dodging missiles. To understand how steering works, it will be helpful to read these links: 

* [Steering Behaviors For Autonomous Characters](http://www.red3d.com/cwr/steer/gdc99/)
* [Boids](http://www.red3d.com/cwr/boids/)

Note that weights are relative. That means arbitrarily increasing a weight may not do what you expect: two weights at `1` and `2` have the same relative importance as the same weights at `100` and `200`.

Flocking is an advanced behavior that allows formation-like flying and collision avoidance with friendly vehicles in the "flock".
See below a pair of demo Cutlass's using flocking to maintain a formation:

<p align="center">
<img src="http://i.imgur.com/iNHJDhO.jpg" alt="Formation flying" width="600"/>
</p>

Start using flocking by setting `NeighborRadius` to a positive number (try `300`, for example). This will tell your vehicle to "notice" other friendly vehicles within this radius and form a flock with them.

```lua
NeighborRadius = 0  -- in meters
```

For alignment and cohesion, ignore craft that are going below this speed (in m/s). Useful if one of the vehicle becomes damaged and has to drop out of formation.

    IgnoreBelowSpeed = 0

Set `FlockWithBlueprintNames = 'all'` to use alignment and cohesion with all friendly craft.
Enclose a comma-delimited list of vehicle names in curly braces {} to use alignment and cohesion
with only craft with those names. Names given will match with any vehicle that starts with that
combination of letters, for example `{'Rapier'}` will match with any vehicle named
'Rapier', 'Rapier-A', 'Rapier-E', etc.

```lua
FlockWithBlueprintNames = 'all'  -- or, for example {'Rapier', 'Cutlass'}, etc.
```

The weight given to alignment, or the desire to match headings with friendly craft. Usually `1` or `0` to turn this on or off respectively, but other values are fine too and will change steering priorities accordingly.

```lua
AlignmentWeight = 1
```

The weight given to cohesion, or the desire to come closer to friendly craft.

```lua
CohesionWeight = 1
```

The weight given to separation, or the desire to avoid coming too close to other friendly vehicles.

```lua
SeparationWeight = 1
```

A cohesion weight for injured vehicles. May be useful for "medic ships".

```lua
InjuredWeight = 0
```

A cohesion weight for friendly vehicles at ranges outside NeighborRadius. It isn't necessary to use this, but it will help vehicles "find each other" if they get separated.

```lua
LongRangeWeight = .5
```

The weight given to the "normal" target-specific behaviors of attack, escape, etc. Usually you should leave this at `1`. A value of `0` means it ignores enemies when it comes to navigation.

```lua
TargetWeight = 1
```

## "DOGFIGHTING" OPTIONS

"Dogfighting" for this AI means changing behavior of the AI according to behavior of the enemy target.

Whether or not to try to match our altitude to the target.
If set to `false`, will use `CruiseAltitude` during combat.

```lua
MatchTargetAltitude = false
```

Range within which to attempt altitude matching. Beyond this range, uses `CruiseAltitude`.

```lua
MatchAltitudeRange = 1000
```

Altitude in meters above (or below, if negative) the target the vehicle will attempt to attain.
Altitude will still be constrained by `MinAltitude` and `MaxAltitude` terrain avoidance constraints.

```lua
MatchAltitudeOffset = 0
```

The minimum and maximum altitudes the aircraft will go to when matching altitude

```lua
MinMatchingAltitude = 100
MaxMatchingAltitude = 800
```

Use vehicle roll to try to "broadside" a target. This is very different to a naval broadside (which uses yaw). This is intended for vehicles which have weapons that don't have good elevation control. Set `BroadsideWithin` to a positive number to start broadsiding when the target is within a certain range -- usually would set to the effective range of your weapons. `BroadsideAngle` then controls the roll angle relative to the target you want your vehicle to take. `0` means you want either side of your vehicle pointed at the enemy. `90` means the bottom of your vehicle, `-90` the top. Roll angles will respect the `MaxRollAngle` setting, in the advanced options. The AI will not attempt to broadside while performing a roll to turn, so this option may work best for vehicles that only yaw to turn.

```lua
BroadsideWithin = 0  -- in meters
BroadsideAngle = 0
```

## MISSILE AVOIDANCE OPTIONS

If you allow it to, this AI will try to avoid enemy missiles by running away or dodging them using the advanced steering system. The effectiveness of these strategies depends on many factors like speed, maneuverability, and use of decoys and flares.

Set `WarningMainframe = -1` to ignore missiles. Set it to the index
of the mainframe with missile warners, if you have missile warners. If you only have one mainframe,
then `WarningMainframe = 0`.

```lua
WarningMainframe = -1
```
Missile evasion is a two-stage maneuver. First, the vehicle will try to run away if it detects missiles within the given time-to-target threshold. If this fails and the missiles reach the `DodgeTTT` threshold, the vehicle will attempt to make a sharp turn to the left or right to shake off the missiles. This "dodge" maneuver will last until the AI detects there are no more dangerous missiles on it's tail. You can turn off dodging by setting `DodgeTTT = 0`.

```lua
RunAwayTTT = 4  -- in seconds
DodgeTTT = 2
```

You shouldn't need to mess with this. Helps decide when a missile might be dangerous.

```lua
DangerRadius = 20
```

The speed (in m/s) your ship is usually able to go when dodging missiles.

```lua
NormalSpeed = 100
```

The weights (or priorities) to give avoiding missiles for the advanced steering system. `RunAwayWeight` is on a *per missile* basis, so a large missile barrage is considered very high-priority in total. Also, the closer to impact a missile is, the more importance it is given. `0` will ignore missiles, larger values will give running away increasingly high priority (`2` works fine in my tests, though you may want it higher if even one missile is dangerous to your vehicle). `DodgingWeight` works differently: if there is even one missile within the `DodgeTTT` threshold, the dodge maneuver will be activated with this priority. Thus, this priority should be set high but probably not as high as `AvoidanceWeight`.

```lua
RunAwayWeight = 2
DodgingWeight = 5
```

## VECTOR THRUST/LARGE RUDDER OPTIONS

Vector thrust means placing jets on spin blocks, potentially allowing much more powerful yaw, pitch,
or roll authority on a vehicle. See below a demo Cutlass using vector thrust for roll:

<p align="center">
<img src="http://i.imgur.com/5ieUbRY.jpg" alt="Vector thrust" width="600"/>
</p>

Vector thrust code may also be used to control large rudders, as we see below. These rudders permit control of yaw, pitch, and roll:

<p align="center">
<img src="http://i.imgur.com/Ha84b4r.jpg" alt="Large Rudders" width="600"/>
</p>

It is reccommended that you use conventional control methods (ailerons, tailplanes, thrusters) in addition to vector
thrust for the most stable (PID-controlled) flight, as vector thrust does not use PID controls.

Turn this option on by specifying a number of SubConstruct IDs
to use with `VTSpinners`. Use `VTSpinners = 'all'` to use all spinners, or supply a comma-delimited
list as follows: `{0,1,2}`. Spinners placed vertically are for yaw. Spinners placed horizontally
control roll and pitch (so I suggest placing them farther to the left and right of your CoM).
Also see the `ExcludeSpinners` option to specify a list of spinners to exclude, if this is set to `'all'`.

```lua
VTSpinners = nil
```

The maximum angle to set any particular spinner to. Roll and pitch angles are cumulative (so make sure they sum to at most 90 degrees).
The maximum angle you can specify is 90 degrees. Set to `0` to not use that particular kind of vector
thrust. You can use negative angles to reverse the angle direction.
The VTOL angle is the downwards angle VTOL spinners will be set to in VTOL mode (see next section).

```lua
MaxVTAngle = {30,30,30,90} -- yaw, roll, pitch, VTOL
```

As PID controls do not affect vector thrust, a separate control system governs these.
The `VTResponseAngle` parameters govern the sensitivity of vector thrust and rudders to desired changes
in yaw, roll, and pitch (respectively). `VTResponseAngleMin` is the minimum deflection (in degrees) from the desired
control angle at which the spinners begin to react. At `VTResponseAngleMax` deflection from the desired
control angle, vector thrust control will be pegged at the maximum angle given by `MaxVTAngle`. In between
these ranges, vector thrust responds linearly to differences in desired control angle.

```lua
VTResponseAngleMax = {20,20,20}
VTResponseAngleMin = {5,5,5}
```

The maximum speed at which vector thrust spinners change direction. Ranges between `1` and `30`. You will want this at `30` for best vehicle performance but may want to set it lower for the sake of smoother visual changes.

```lua
VTSpeed = 30
```

You may experience "flapping", or rapid changes in vector thrust angle on some settings. To mitigate this, lower the Derivative value in the PID settings corresponding to the particular type of control you want to adjust - sometimes very small values are necessary. You can also try reducing `MaxVTAngle` and/or `VTSpeed`, but these all change different behaviors.

## VTOL/HOVER OPTIONS

While not yet as extensive as the hover options afforded by some other scripts, this AI does allow for control of hover jets to control altitude, as well as VTOL takeoff at low speeds (i.e. decks of carriers and the like).

Use jets pointing up or down to assist in controlling altitude. 

```lua
UseAltitudeJets = false
```

An option for airships is to allow the AI to manually control downward-pointing engine drive
fractions. This option can be turned off above a certain speed (in m/s) for VTOL takeoffs.
Set `UseVTOLBeneathSpeed` to a large number to use VTOL all the time

```lua
 UseVTOLBeneathSpeed = 0
```

A list of engine indices to use for VTOL. Set to `'all'` to use all downward-pointing engines.
Use a list format, i.e. `{0,1,2,3}` to specify exact engine indices -- generally engines are numbered in the order you place them on the vehicle. Engines may be attached to
spin blocks for use with vector thrust, and will be rotated backwards for normal use when `UseVTOLBeneathSpeed`
is exceeded. If engines are on spin blocks, you must specify their indices if the AI is to
control their drive fraction.
For engines mounted to the hull, set each such engine to "Main".
You can (and should) set the drive fractions of each engine you want to use with this AI,
on the engine itself. The drive fraction on each engine specifies the maximum amount of
power you want to use on that engine.

```lua
VTOLEngines = nil
```

The set of VTOL SubConstruct spinner IDs to use when in VTOL mode. Set to `'all'` to select all spinners capable of
pointing downwards. Otherwise, choose a list of spinner SubConstruct IDs as follows: `{0,1,2,3}`.
VTOL spinners will angle downwards at their given MaxVTAngle when in VTOL mode, otherwise
they will point backwards and operate as normal vectored thrust.
Also see the `ExcludeSpinners` setting to specify a list of spinners to exclude, if this is set to `'all'`.

```lua
VTOLSpinners = 'all'
```

## ADVANCED OPTIONS

These are options most users shouldn't need to change.

Set to `0` for water mode, `1` for land mode, `2` for air mode.

```lua
DriveMode = 2
```

This AI will optionally use PID-controlled hydrofoils for yaw, roll and pitch (depending on placement of hydrofoils).
With `HydrofoilMode = 0`, hydrofoil control is in "default" mode, not directly controlling hydrofoils, though they may
be controlled indirectly by the FtD propulsion AI. With `HydrofoilMode = 1`, pitch/yaw control is separated from roll control.
Hydrofoils near the center of mass (along the length of the vehicle) will be used only for roll. Hydrofoils near the front
or back of the vehicle are used for pitch/yaw according to orientation (although front, back, up, or down placement does not matter).
With `HydrofoilMode = 2`, all hydrofoils will be used for roll control, while again only hydrofoils near the front or back of the
vehicle will be used for pitch/yaw. Sharing roll control with pitch/yaw has advantages and disadvantages, so the choice is up to you.

```lua
HydrofoilMode = 0
```

How often to recalculate heading and altitude. At `1` (the minimum), these will recalculate every update. At `10`,
they will recalculate every 10th update. The lower UpdateRate is, the more
responsive the vehicle will be, but it will also take more processing time. Thus higher values may increase FPS.
See more information in [How to improve FPS](https://github.com/Madwand99/From-the-Depths/tree/master/Advanced%20Aerial%20AI#how-to-improve-fps).

```lua
UpdateRate = 1
```

This AI automatically clamps `MaxPitch` and `MinPitch` according to how near the aircraft is to it's target altitude to prevent overshooting when changing altitude. Higher values of `AltitudeClamp` decrease this effect, allowing steeper changes in altitude. Lower values help prevent overshooting. AltitudeClamp also controls how gradually airships approach their target altitude when using altitude jets or helicopter blades.

```lua
AltitudeClamp = .2
```

The default roll angle to maintain. Set it to `90` to have the vehicle usually on it's side.
You may want to set `AngleBeforeTurn = 180` if you do this, so you never yaw.

```lua
DefaultRollAngle = 0
```

There are three kinds of ways this AI can use to control forward speed (i.e., main drive):
* `MainDriveControlType = 0`: Uses standard throttle control. Most vehicles will want this option.
* `MainDriveControlType = 1`: Use forward (only) propulsion balancing. This may help to compensate
for damage to thrusters, or unbalanced thruster placement compared to the center of mass.
WARNING: Propulsion balancing can be very unreliable. YMMV.
* `MainDriveControlType = 2`: Control throttle by varying drive fraction. The AI will try to guess
which thrusters are forward thrusters. It will EXCLUDE any thrusters on spin blocks, so vectored
thrust continues to operate at full power for greatest possible maneuverability.

Choose whatever works for you. I suggest `0` for most vehicles, but vehicles using vector thrust may want to try `2`. This decision only matters if Thottle values you choose aren't always `1`.

```lua
MainDriveControlType = 0
```

The maximum roll angle your vehicle will take during a roll. Set this lower if your vehicle is
going into the sea during a roll.

```lua
MaxRollAngle = 120
```

How close you want your aircraft to be to the "desired" roll angle before pitching up. Set this too
high and you may gain or lose too much altitude during a turn.

```lua
RollTolerance = 30
```

Set this to true if you are interested in knowing what the AI is "thinking" as it operates.

```lua
DebugMode = false
```

Prefer to approach the enemy on only one side of the aircraft or not. Set `AngleBeforeTurn` positive
or negative to choose the side.

```lua
UsePreferredSide = false
```

Predictive guidance tries to guess where the enemy will be by the time the aircraft will reach it,
and head in that direction instead of the current position of the enemy. Set this to `true`
when you don't care about pointing directly at the enemy, and you wish to prioritize "cutting the
circle" on a fast orbiter, or if you want to ram the enemy. Set to `false` if it is more important
to orient yourself in relation to the enemies current position during the attack run, for example if
all your weapons are in a fixed-forward firing position.

```lua
UsePredictiveGuidance = true
```

`AltitudeOffset` is useful when you wish to create multiple vehicles of the same type, and you
don't want to individually set all the various altitude settings so they won't crash into each
other. This value adjusts `CruiseAltitude`, `MaxAltitude`, `MatchAltitudeOffset`, and `MinMatchingAltitude`
simultaneously. You can also give it two table values, e.g. `{0,100}`. This will randomly generate
an offset in the range between 0-100, so your aircraft may be less likely to collide with others of it's kind.

```lua
AltitudeOffset = 0
```

`AngleOfEscape` is the angle relative to the current heading that is set when going from "attack mode" to "escape mode".
It is set once when entering escape mode, and does not adjust to enemy movements.
By default, it is set to `0`, which means to keep flying in the original direction. You can set it to anything
you like though, for example `AngleOfEscape = 90` will turn off at 90 degrees, or
`AngleOfEscape = 180` will have the vehicle try to turn around. Note that if the steering system has other priorities, this escape angle is likely to be ignored.

```lua
AngleOfEscape = 0
```

An additional option for helicopters, vector thrust, and VTOL, `ExcludeSpinners` allows you
to specify spinner SubConstruct IDs to be excluded from use by this AI (if you use the `'all'` option
for `VTSpinners` or `VTOLSpinners`). If used, this must be a comma-delimited list surrounded
by curly braces, for example `ExcludeSpinners = {0,5}`. Note that `HeliSpinners` are automatically excluded
from use by vector thrust if they are specified.

```lua
ExcludeSpinners = nil
```

Whether to orbit the spawn point when there are no enemies, or just fly off in a straight line

```lua
OrbitSpawn = true
```

## Useful links

* [Cutlass demo on Steam workshop](https://steamcommunity.com/sharedfiles/filedetails/?id=1354628015) - This is a demo of the AI, showing off concepts like missile evasion and vector thrust.
* [From the Depths on Steam](http://store.steampowered.com/app/268650/From_the_Depths/)
* [Forum thread for this AI where you can go for help.](http://www.fromthedepthsgame.com/forum/showthread.php?tid=9108) - Note: if your issue is "my plane doesn't fly", make sure it DOES fly with the normal aerial AI -- this AI can't overcome bad design, though it can help in some cases. Exception: vector thrust. Normal AI can't do that (without a lot of ACBs anyway). This code is not magic and can't make a plane fly that couldn't otherwise.
* [Draba's hover AI](http://www.fromthedepthsgame.com/forum/showthread.php?tid=15393) - My code borrows PID control code from this AI. For some types of vehicles -- particularly some hovercraft -- Draba's AI is very helpful.
* [From the Depths Discord](https://discord.gg/pKgnWdf) - I can often be found here. Feel free to message me about this AI.
