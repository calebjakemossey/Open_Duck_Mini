# Open Duck Mini - Hardware and Simulation Model Reference

Source repository: `/home/lakieb/Documents/open_duck_mini_research/Open_Duck_Mini`

---

## 1. Robot Morphology

Open Duck Mini is a miniature biped inspired by Disney's BDX Droid. At full leg extension the robot stands approximately **42 cm tall**. Total BOM cost target is under **$400 USD**.

### Version lineage

Two generations exist in the repository:

| Feature | v1 (bdx/) | v2 (open_duck_mini_v2/) |
|---|---|---|
| Servo type | Feetech DC15-A01 (Dynamixel-style) | Feetech STS3215 (ST3215) |
| Servo bus | U2D2 RS-485 adapter | Direct half-duplex serial (half-duplex means the bus can only send OR receive at any given moment, not both at the same time; every servo read requires three phases: send the "read position" command, wait for the servo to process it, then receive the reply - this adds latency and is part of why the 50 Hz control loop has a tight timing budget) |
| Hip roll implementation | Separate roll bracket with dual horn | Dedicated roll motor with roll_bearing |
| Foot design | Single flat foot with `foot_contact` geom | Multi-part foot: TPU sole + PLA structure |
| Head DOFs | 3 (neck_pitch, head_pitch, head_yaw) | 4 (neck_pitch, head_pitch, head_yaw, head_roll) |
| Antenna actuation | SG90 RC servo (PWM) | SG90 RC servo (PWM) |
| Compute | Raspberry Pi Zero W (on body) | Raspberry Pi Zero 2W (in head) |
| Motor comms | Dynamixel-protocol via U2D2 | Feetech STS protocol at 1 Mbit/s |

### Degrees of freedom

**v2 (primary version) - 16 actuated DOF total:**

| Group | Joint | Count |
|---|---|---|
| Left leg | left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee, left_ankle | 5 |
| Right leg | right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee, right_ankle | 5 |
| Neck/head | neck_pitch, head_pitch, head_yaw, head_roll | 4 |
| Antennas | left_antenna, right_antenna | 2 |
| **Total** | | **16** |

The RL policy controls 14 DOF (legs + neck/head excluding antennas). The antennas are decorative expression actuators driven separately via PWM.

**v1 - 15 actuated DOF total:**

Same leg structure but only 3 head DOF (no head_roll). No head_roll joint in v1.

---

## 2. MJCF Model - Detailed Parameters

### 2.1 v2 model files

Two sets of MJCF files exist for v2, in different repositories:

**Open Duck Mini (reference robot model)** - `mini_bdx/robots/open_duck_mini_v2/`:

| File | Purpose |
|---|---|
| `robot.xml` | Position-controlled model, `<position>` actuators with kp/forcerange |
| `robot_motors.xml` | BAM-identified model, `<motor>` actuators with identified dynamics |
| `scene.xml` | Scene wrapper including `robot_motors.xml` with ground plane |
| `scene_position.xml` | Scene wrapper including `robot.xml` |

The BAM-identified `robot_motors.xml` is the canonical model for RL training.

**Open Duck Playground (training files)** - `playground/open_duck_mini_v2/xmls/`:

| File | Purpose |
|---|---|
| `open_duck_mini_v2.xml` | Symmetric re-exported robot model (current, see §2.10) |
| `scene_flat_terrain.xml` | Scene wrapper with updated home keyframe |
| `open_duck_mini_v2_backlash.xml` | Variant with backlash modelling |
| `scene_flat_terrain_backlash.xml` / `scene_rough_terrain_backlash.xml` | Scene wrappers for backlash variant |
| `joints_properties.xml` | Joint parameter includes |
| `sensors.xml` | Sensor definitions |

`open_duck_mini_v2.xml` and `scene_flat_terrain.xml` are the files used for RL training. Old asymmetric files are backed up in `xmls.backup/`.

### 2.2 Default joint parameters

**robot.xml (position-controlled):**
```xml
<joint damping="1.0" frictionloss="0.01" armature="0.01"/>
<position kp="9.5" kv="0.0" forcerange="-5.2 5.2"/>
```

**robot_motors.xml (BAM-identified):**
```xml
<joint damping="0.0" armature="0.027" frictionloss="0.083"/>
```
Actuators are `<motor>` type (torque-controlled). The BAM layer sits on top to translate position commands to torques.

**v1 bdx/robot.xml (BAM-tuned):**
```xml
<joint damping="0.095" frictionloss="0.058" armature="0.0018"/>
<position kp="2.54" kv="0.0" forcerange="-0.8 0.8"/>
```

### 2.3 Joint table - v2

All joints use `axis="0 0 1"` (revolute about Z). Angles in radians. The symmetric MJCF uses a true mirror convention for the knee and ankle joints: a positive value on the left leg and a negative value on the right leg both represent the same physical configuration. This is encoded via body orientation in the kinematic tree rather than by changing the axis vector - the right-side bodies are rotated 180° relative to the left, so the Z-axis already points in the mirrored direction.

| Joint | Range min (rad) | Range max (rad) | Range (deg) |
|---|---|---|---|
| left_hip_yaw | -0.5236 | +0.5236 | ±30° |
| left_hip_roll | -0.4363 | +0.4363 | ±25° |
| left_hip_pitch | -1.2217 | +0.5236 | -70° to +30° |
| left_knee | -1.5708 | +1.5708 | ±90° |
| left_ankle | -1.5708 | +1.5708 | ±90° |
| right_hip_yaw | -0.5236 | +0.5236 | ±30° |
| right_hip_roll | -0.4363 | +0.4363 | ±25° |
| right_hip_pitch | -1.2217 | +0.5236 | -70° to +30° (same range, mirrored semantics) |
| right_knee | -1.5708 | +1.5708 | ±90° |
| right_ankle | -1.5708 | +1.5708 | ±90° |
| neck_pitch | -0.3491 | +1.1345 | -20° to +65° |
| head_pitch | -0.7854 | +0.7854 | ±45° |
| head_yaw | -2.7925 | +2.7925 | ±160° |
| head_roll | -0.5236 | +0.5236 | ±30° |
| left_antenna | -1.5708 | +1.5708 | ±90° |
| right_antenna | -1.5708 | +1.5708 | ±90° |

Note on hip pitch: the old asymmetric model had left_hip_pitch at -70°/+30° and right_hip_pitch at -30°/+70°. The current symmetric model gives both legs the same range with mirrored semantics - a forward swing produces a negative value on the left and a positive value on the right. The home keyframe reflects this: left hip pitch = -0.63 rad, right hip pitch = +0.63 rad.

### 2.4 Actuator configuration

**robot.xml** uses position actuators with `inheritrange="1"` (limits copied from joint):
```xml
<position name="left_hip_yaw" joint="left_hip_yaw" inheritrange="1"/>
```
No explicit gear ratio or kp per-actuator - all inherit the defaults (kp=9.5, forcerange=±5.2 N·m).

**robot_motors.xml** uses motor (torque) actuators:
```xml
<motor name="left_hip_yaw" joint="left_hip_yaw"/>
```
No forcerange set at the XML level - the BAM MujocoController applies torques directly via the identified motor model.

### 2.5 Body masses and inertia - v2

Bodies with meaningful mass (zero-mass bodies are coordinate-frame anchors):

| Body | Mass (kg) | Notes |
|---|---|---|
| trunk_assembly | 0.6985 | Main torso with electronics, battery cells, BMS, board |
| knee_and_ankle_assembly (left thigh) | 0.1241 | Upper leg segment |
| knee_and_ankle_assembly_2 (left shin) | 0.0726 | Lower leg segment |
| foot_assembly (left) | 0.0752 | Ankle + foot structure |
| left_roll_to_pitch_assembly | 0.0752 | Hip roll bracket |
| hip_roll_assembly | 0.0665 | Hip yaw bracket |
| neck_pitch_assembly | 0.0662 | Neck link |
| head_assembly | 0.3526 | Full head with Pi Zero 2W, bearing, SGS90s |
| head_pitch_to_yaw | 0.0169 | Head pitch-to-yaw link |
| neck_yaw_assembly | 0.0918 | Head yaw structure |
| left/right_antenna_holder | 0.0042 | Each antenna |

Right leg assemblies mirror left leg masses identically.

**Estimated total robot mass** (summing all non-negligible bodies): approximately 1.6-1.8 kg. The trunk alone is 699 g and the head 353 g - these are the two heaviest assemblies.

### 2.6 Contact parameters

From `scene.xml`:
```xml
<geom friction="1.5 0.01 0.0006"/>
```
Sliding friction coefficient: 1.5, torsional: 0.01, rolling: 0.0006. MuJoCo's default contact model (`condim=3`) calculates three components: normal force (pushing surfaces apart) plus sliding friction in two directions (forward/backward and left/right). It ignores torsional friction (twisting) which would require `condim=4` or `condim=6`. For a robot walking on flat ground, this is sufficient.

Contact exclusions prevent self-collision between adjacent bodies:
- trunk_assembly ↔ neck_yaw_assembly, hip_roll_assembly, hip_roll_assembly_2
- hip_roll_assembly ↔ left_roll_to_pitch_assembly
- hip_roll_assembly_2 ↔ right_roll_to_pitch_assembly
- trunk_assembly ↔ left/right_roll_to_pitch_assembly
- neck_yaw_assembly ↔ head_pitch_to_yaw, head_assembly
- head_pitch_to_yaw ↔ head_assembly

All geoms: `contype="1" conaffinity="1"`.

### 2.7 Solver settings

No explicit solver settings are defined in these model files. MuJoCo defaults apply. The simulation timestep is set programmatically; scripts use `model.opt.timestep = 0.005` (200 Hz) in `onnx_AWD_mujoco.py`. Comments in scene files suggest 100 Hz (`timestep=0.01`) as an alternative.

### 2.8 v1 joint table (bdx/)

| Joint | Range min (rad) | Range max (rad) |
|---|---|---|
| right_hip_yaw | -0.6981 | +0.6981 |
| right_hip_roll | -1.5708 | +0.4363 |
| right_hip_pitch | -0.5236 | +1.5708 |
| right_knee | -2.0944 | +2.0944 |
| right_ankle | -1.5708 | +1.5708 |
| left_hip_yaw | -0.6981 | +0.6981 |
| left_hip_roll | -1.5708 | +0.4363 |
| left_hip_pitch | -0.5236 | +1.5708 |
| left_knee | -2.0944 | +2.0944 |
| left_ankle | -1.5708 | +1.5708 |
| neck_pitch | -1.3090 | +0.1745 |
| head_pitch | -0.6981 | +0.6981 |
| head_yaw | -2.0944 | +2.0944 |
| left_antenna | -1.1345 | +2.0944 |
| right_antenna | -2.0944 | +1.1345 |

v1 hip roll has a much larger range (-90°/+25°) compared to v2 (±25°). v1 knee range is ±120° versus v2 ±90°.

### 2.9 IMU site (v1 only)

The v1 `robot.xml` defines an explicit IMU site on the `base` body at origin:
```xml
<site name='imu' size='0.01' pos='0.0 0 0.0'/>
```
with sensors:
```xml
<gyro name='angular-velocity' site='imu' noise='0.005' cutoff='34.9'/>
<velocimeter name='linear-velocity' site='imu' noise='0.001' cutoff='30'/>
<accelerometer name='linear-acceleration' site='imu' noise='0.005' cutoff='157'/>
```
The v2 models do not define a dedicated IMU site in the XML; IMU data is read from `qpos`/`qvel` in simulation scripts.

### 2.10 Symmetric v2 MJCF model (current)

The MJCF used for training (`playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml`) is a re-export from the corrected OnShape CAD. The original CAD had a 37 mm positional offset in the right lower leg that caused asymmetric body positions in the exported MJCF. That offset was fixed at source in OnShape and the model re-exported. All symmetric body pairs now have 0.000 mm positional difference.

**What changed in the re-export:**

- All left/right body position pairs are now exact mirrors (previously the right lower leg was offset by 37 mm along the parent axis).
- Joint axis directions follow a true mirror convention. Both knees and ankles declare `axis="0 0 1"` in XML, but because the right-side bodies are rotated 180° relative to their left-side counterparts in the kinematic tree, the effective rotation direction is mirrored - positive left_knee and negative right_knee represent the same physical posture.
- The `right_cache` body that appears in the MJCF is an artefact of the export pipeline rather than a named CAD part (see §2.11).

**Home keyframe (`scene_flat_terrain.xml`):**

The home pose was recalculated so both feet sit at identical height. Right-leg joint values are exact negations of their left-leg counterparts where the mirror convention applies:

```
Left leg:   hip_yaw=0.002  hip_roll=0.053  hip_pitch=-0.63  knee=1.368  ankle=-0.784
Right leg:  hip_yaw=-0.002 hip_roll=-0.053 hip_pitch=0.63   knee=-1.368 ankle=0.784
```

The z-difference between foot contact points is 0.000 mm in this pose.

**Backed-up files:**

The asymmetric files are preserved in `playground/open_duck_mini_v2/xmls.backup/` (contains `open_duck_mini_v2.xml.bak` and `assets.bak/`). Do not delete these; they are the reference for understanding what the original asymmetry looked like.

### 2.11 OnShape-to-MJCF export pipeline conventions

The export uses Rhoban's `onshape-to-robot` tool, which reads the OnShape assembly's Mate connectors and generates the kinematic tree. Several conventions are critical to understand when modifying or re-exporting the model:

**Mate naming determines joint type:**
The `onshape-to-robot` exporter uses the mate NAME PREFIX to decide what to do with each mate. The actual OnShape mate type (Fastened/Revolute/etc) does NOT affect the export decision.
- `dof_` prefix - exports as a revolute joint. The exporter creates a `<joint type="hinge">` in the MJCF.
- `fix_` prefix - exports as a fixed/welded connection. The exporter merges the two connected bodies into a single MJCF body rather than creating a joint.
- `frame_` prefix - exports as a site (sensor attachment point), not a joint.

This means a mate can be Revolute in OnShape (so the parts move correctly in CAD assembly) but be named with `fix_` prefix to export as a fixed connection in MJCF. This is how the antennas are handled - they're Revolute mates in OnShape (the physical antennas flex) but named `fix_left_antenna` / `fix_right_antenna` so they get merged into `head_assembly` for training simplicity.

**Mate entity order:**
The first entity listed in a Mate definition becomes the parent in the kinematic chain, and the second becomes the child. Getting the order wrong inverts the parent-child relationship, which changes which body moves relative to which when the joint actuates.

**Body merging and `right_cache`:**
When two bodies are connected by a fixed mate (`fix_` prefix), the exporter merges them into a single MJCF body. The `right_cache` body in the MJCF is the result of the upper leg assembly and its cache component being merged this way. The name `right_cache` comes from the OnShape part name and is functionally correct - it represents the right upper leg assembly as a single rigid body.

**Inertia and mass:**
The exporter reads material density from OnShape and computes inertia tensors. If material properties are not set in OnShape, the exporter falls back to defaults that may not reflect the real 3D-printed part (which has reduced density due to infill). Mass can be overridden in `config.json` using slicer-measured values.

---

## 3. Servo Characterisation and BAM

### 3.1 Servo hardware

**v2:** Feetech STS3215 (also referred to as ST3215 or STS3215_7.4V in BAM parameters)
- 12-bit position encoder (4096 counts per revolution)
- RS-485 half-duplex serial bus at 1 Mbit/s
- Motor resistance R ≈ 2.04 Ω (identified)
- Motor torque constant kt ≈ 1.43 N·m/A (identified)

**v1:** Feetech DC15-A01
- Similar form-factor but different electronics
- Accessed via U2D2 Dynamixel adapter

### 3.2 BAM - Behavioural Actuator Modelling

BAM (Behavioural Actuator Modelling) is a library developed by Rhoban (https://github.com/Rhoban/bam) that identifies a physics-based actuator model from hardware measurements. The identified model captures effects that naive PD simulation misses:

- **Static and Coulomb friction** (friction_base, friction_stribeck)
- **Viscous friction** (friction_viscous)
- **Load-dependent friction** - friction that increases with applied torque, both from motor-side and external load:
  - load_friction_motor, load_friction_external
  - Stribeck-regime variants: load_friction_motor_stribeck, load_friction_external_stribeck
  - Quadratic variants: load_friction_motor_quad, load_friction_external_quad
- **Stribeck velocity** (dtheta_stribeck) - threshold speed below which static friction dominates
- **Armature inertia** (armature) - In MuJoCo, "armature" represents the rotational inertia of the motor's spinning parts (the rotor and gears). Higher armature means the joint resists changes in speed more - it takes longer to speed up and slow down. Real motors have significant rotor inertia that simple models ignore, so this parameter makes the simulation more realistic.
- **Motor electrical parameters** (kt, R)
- **Alpha** - gain parameter in the firmware PD controller internal loop

### 3.3 Identified parameters (params_m6.json)

From `experiments/v2/params_m6.json`, the "m6" model for STS3215 at 7.4V:

| Parameter | Value | Unit |
|---|---|---|
| kt (torque constant) | 1.4303 | N·m/A |
| R (resistance) | 2.0431 | Ω |
| armature | 0.009878 | kg·m² |
| friction_base | 0.010907 | N·m |
| friction_stribeck | 0.024827 | N·m |
| friction_viscous | 0.023287 | N·m·s/rad |
| load_friction_motor | 0.19020 | dimensionless |
| load_friction_external | 0.14528 | dimensionless |
| load_friction_motor_stribeck | 0.045660 | dimensionless |
| load_friction_external_stribeck | 0.73407 | dimensionless |
| load_friction_motor_quad | 0.006871 | dimensionless |
| load_friction_external_quad | 0.006948 | dimensionless |
| dtheta_stribeck | 0.099859 | rad/s |
| alpha | 2.6539 | - |

### 3.4 Translation to MuJoCo

BAM exports five parameters to MuJoCo format via `bam.to_mujoco`:
- `damping` - maps from friction_viscous. These are two different types of friction. "Damping" is velocity-proportional friction - it increases smoothly with speed, like a car shock absorber (faster movement = more resistance). "Frictionloss" is Coulomb friction - a constant force opposing motion regardless of speed, like the friction you feel when sliding a box across a floor. Real servos have both types, and they affect motion quality differently: damping slows fast movements, while frictionloss makes it harder to start moving at all (contributing to the "stick then jump" behaviour).
- `kp` - PD position gain
- `frictionloss` - maps from friction_base
- `armature` - rotor inertia reflected to joint
- `forcerange` - derived from stall torque. Forcerange is the maximum torque (rotational force) the motor can exert, in Newton-metres. This represents the physical torque limit of the real servo. When the simulation tries to exceed this, the motor "saturates" - it is commanding more force than the real motor could produce. This constraint affects what gaits are physically achievable.

For `robot_motors.xml` the identified values are:
```xml
<joint damping="0.0" armature="0.027" frictionloss="0.083"/>
```
(damping set to 0 here; the full friction model is handled by BAM's `MujocoController` at runtime rather than by MuJoCo's built-in damping.)

### 3.5 Identification procedure

The `experiments/v2/identification.py` script drives a single servo (ID 24, the left ankle) through step commands (0° → 90° → 0°) at maximum acceleration (254), recording position, load, current, and speed at each timestep. This raw data is then fed into BAM's fitting routines to extract the model parameters.

### 3.6 Firmware control law

From `docs/feetech_identification.md`:

The servo firmware operates a position error loop:
```
ε = θ_desired_firmware - θ_present_firmware
λ = Kp * ε_firmware    (duty cycle / PWM)
```
with 12-bit firmware units (4096 counts per 2π rad). The combined gain including the firmware scaling is:
```
λ = Kp * Kg * ε
Kg = λ / (ε * Kp)
```
where R = 2.5 Ω nominal. This internal PD loop runs entirely in firmware; the host sends only goal position.

---

## 4. Mechanical Design

### 4.1 CAD source

All CAD lives on Onshape: `https://cad.onshape.com/documents/64074dfcfa379b37d8a47762/w/3650ab4221e215a4f65eb7fe/e/0505c262d882183a25049d05`

Export uses Rhoban's `onshape-to-robot` tool, generating both URDF and MJCF. The `config.json` files in each robot directory specify the Onshape document ID.

A 37 mm positional offset in the right lower leg that was present in the original CAD has been corrected. The current MJCF in `playground/open_duck_mini_v2/xmls/` was re-exported from this corrected CAD. For export conventions (mate naming, body merging, entity order) see §2.11.

### 4.2 Key design decisions

**Leg kinematic chain (v2):**
- Hip yaw via a dedicated roll-bearing block (`roll_motor_bottom/top`) that mounts inline with the trunk
- Hip roll via `left_roll_to_pitch` / `right_roll_to_pitch` bracket - a 3D-printed orange linkage
- Hip pitch, knee, and ankle each served by a single STS3215 with a passive palonier (horn) on the opposing side for structural support
- Knee-to-ankle connection uses twin parallel sheet-metal-style flat links (`left_knee_to_ankle_left_sheet`, `left_knee_to_ankle_right_sheet`) - these are the main structural leg members
- Leg spacer (`leg_spacer`) separates the two sheets at each joint

**Foot design (v2):**
- Three-layer construction: `foot_top` (structural, yellow PLA) + `foot_side` + `foot_bottom_pla` (sole, PLA) + `foot_bottom_tpu` (compliant contact, TPU 40% infill)
- TPU sole provides compliance to improve ground contact and reduce sim2real gap
- Foot switches (physical micro-switches) are press-fit into the foot sole

**Trunk:**
- `trunk_bottom` + `trunk_top` printed parts assembled with M3×10 screws and M3 heat-set inserts
- Three STS3215 motors mounted in the trunk: one for the neck pitch (top-centre), two for the hip yaws (left/right)
- Roll bearings provide lateral support for the hip yaw axes

**Head (v2):**
- 4-DOF neck/head mechanism: neck_pitch → head_pitch → head_yaw → head_roll
- Head assembly contains the Raspberry Pi Zero 2W, a custom PCB board, two SG90 RC servos (antenna actuation), and the BNO055 IMU
- The head is relatively heavy (353 g) due to the compute board

### 4.3 Motor ID assignment (v2)

| Joint | Motor ID |
|---|---|
| right_hip_yaw | 10 |
| right_hip_roll | 11 |
| right_hip_pitch | 12 |
| right_knee | 13 |
| right_ankle | 14 |
| left_hip_yaw | 20 |
| left_hip_roll | 21 |
| left_hip_pitch | 22 |
| left_knee | 23 |
| left_ankle | 24 |
| neck_pitch | 30 |
| head_pitch | 31 |
| head_yaw | 32 |
| head_roll | 33 |

### 4.4 Print guide summary

All parts printed in **PLA at 15% infill**, except `foot_bottom_tpu.stl` printed in **TPU at 40% infill**.

Unique printed parts (not counting mirrored duplicates): approximately 36 distinct STL files.

Parts requiring quantity > 1:
- foot_top × 2, foot_side × 2, foot_bottom_pla × 2, foot_bottom_tpu × 2
- knee_to_ankle_left_sheet × 4, knee_to_ankle_right_sheet × 4
- leg_spacer × 4
- roll_motor_bottom × 2, roll_motor_top × 2

### 4.5 v1 vs v2 mechanical differences

**Servo upgrade:** v1 used DC15-A01 (smaller Feetech servo) accessed via U2D2 Dynamixel adapter. v2 switched to STS3215 (higher-torque, 12-bit, direct serial). This required a complete mechanical redesign of all motor mounts.

**Hip yaw mechanism:** v1 used a `double_u` bracket (a 3D-printed U-shaped fork) with the hip yaw motor's dual horns engaging both sides. v2 uses a simpler roll-bearing block design with the motor cantilevered on one side.

**Hip roll range:** v1 allowed -90° of roll (much greater range), v2 restricted to ±25°. The v2 design is more conservative, better suited to the STS3215's torque capability.

**Foot contact sensing:** v1 had a `foot_contact` mesh geom (separate collision geometry for the black rubber sole area) but no documented electrical foot switch. v2 has explicit press-fit micro-switches with GPIO wiring documented.

**Compute location:** v1 placed the Raspberry Pi Zero W on the body. v2 moved it to the head, which contributes to the head's 353 g mass.

**DOF count:** v1 = 15 actuated DOF (no head_roll). v2 = 16 actuated DOF.

---

## 5. Sensor Setup

### 5.1 IMU

**Sensor:** Bosch BNO055
- 9-axis (accelerometer + gyroscope + magnetometer with on-chip sensor fusion)
- Provides orientation quaternion, angular velocity, linear acceleration

**Placement (v2):** Mounted on the trunk assembly, visible as `bno055` mesh at position `(-0.08711, 0, 0.0417909)` in trunk-local coordinates. This is approximately mid-torso height, towards the rear.

The assembly guide notes a known issue: the BNO055 is sometimes mounted upside-down (flipped along X) in early builds, and the documentation says this can be corrected in software configuration later. The guide explicitly acknowledges this: *"It's actually better to mount the IMU with the correct natural orientation, which would be flipped along the X axis compared to the pictures below."*

**Pin connections (Raspberry Pi Zero header):**
- VIN → Pin 1 (3.3V)
- GND → Pin 9
- SDA → Pin 3 (GPIO 2)
- SCL → Pin 5 (GPIO 3)

**Simulation (v1):** Three MuJoCo sensor elements defined - gyroscope (noise 0.005, cutoff 34.9 rad/s), velocimeter (noise 0.001, cutoff 30 m/s), accelerometer (noise 0.005, cutoff 157 m/s²).

**Simulation (v2):** No dedicated sensor block in the MJCF. IMU readings are simulated by reading `qpos[3:7]` (quaternion) and `qvel[3:6]` (angular velocity) from the freejoint.

### 5.2 Foot contact sensors

**Hardware:** Physical micro-switches press-fit into each foot sole, activated when the foot contacts the ground.

**GPIO connections:**
- Left foot → Pin 15 (GPIO 22)
- Right foot → Pin 13 (GPIO 27)
- GND → Pin 9

**Simulation:** Contact detection uses MuJoCo's contact detection API (`check_contact` in `mujoco_utils.py`), checking for contact between `foot_assembly`/`foot_assembly_2` and the `floor` geom. This returns a boolean per foot, matching the binary hardware switch signal.

The RL observation vector (56 elements in the AWD policy) includes the two foot contact booleans.

### 5.3 Other sensors

**Eye LEDs (expression):**
- Left eye anode → Pin 16 (GPIO 23)
- Right eye anode → Pin 18 (GPIO 24)
- Projector anode → Pin 22 (GPIO 25)
- Common cathode → Pin 6 (GND)

**Antenna servos (SG90 PWM):**
- Left antenna PWM → Pin 32 (GPIO 12, hardware PWM)
- Right antenna PWM → Pin 33 (GPIO 13, hardware PWM)

**Audio (MAX98357A I2S amplifier):**
- LRC → Pin 35 (GPIO 19)
- BCLK → Pin 12 (GPIO 18)
- DIN → Pin 40 (GPIO 21)

Camera, microphone, and speaker are listed as planned expression features but described as not yet implemented in the documented assembly guide (as of early 2025).

---

## 6. Assembly Documentation

### 6.1 Available documents

| Document | Path | Status |
|---|---|---|
| Assembly guide | `docs/assembly_guide.md` | Incomplete - covers trunk, feet, shins, thighs, hips, neck, head mechanism, electronics, body. Missing: servo driver board photo, expression features (camera, antennas, eye LEDs, projector, speaker) |
| Print guide | `docs/print_guide.md` | Complete - all STL files listed with quantities and material specifications |
| Motor configuration | `docs/configure_motors.md` | Complete - lists all motor IDs and points to runtime script |
| Sim2real guide | `docs/sim2real.md` | Partial draft - covers model export, BAM identification, and training framework |
| Wiring diagrams | `docs/open_duck_mini_v2_wiring_diagram.png`, `docs/wiring.png` | Image files (not readable as text) |
| Feetech identification | `docs/feetech_identification.md` | Mathematical derivation only - brief |

### 6.2 Completeness assessment

The assembly guide covers the mechanical assembly sequence adequately for someone who also has the Onshape CAD model open for reference. Several gaps exist:

- Exact screw count and bill of screws is not finalised ("X m3 screws - TODO")
- Servo driver board mounting lacks a photo
- Expression features (camera, antennas, LED eyes, projector, speaker) are explicitly deferred with "TODO" markers
- No torque specification for fasteners
- The `loctite_threadlocker_blue_243` instruction is present but ad-hoc

A third-party build guide exists at `https://tnkr.ai/explore/docs/open-duck-mini/open-duck-mini-v2` and is referenced in the README as potentially more complete.

Chinese-language documentation with more detail is hosted externally on Feishu.

### 6.3 Motor configuration procedure

Each servo must be individually configured before assembly. The configuration script sets:
- Motor ID (unique per servo)
- Zero position (horn alignment)
- D-gain set to 0 (no derivative term in firmware PD)
- Maximum acceleration set to 254 (maximum)

This must be done prior to mechanical assembly because the servo horn position is set at electrical zero, and the CAD design assumes specific horn orientations.

---

## 7. Known Hardware Issues

### 7.1 Servo backlash and accuracy

The STS3215 servos are consumer-grade plastic-gear servos. The BAM identification explicitly characterises significant friction nonlinearity:

- **Stribeck friction** (load_friction_external_stribeck = 0.734) is very high, indicating substantial stiction at low speeds
- **Load-dependent friction** (load_friction_motor = 0.190, load_friction_external = 0.145) means effective friction increases significantly under load
- **Position resolution** is 12-bit (4096 counts/rev ≈ 0.088°), but actual positioning accuracy is degraded by the plastic gear train

The sim2real documentation explicitly states: *"It's a hard problem, especially for us since we are using cheap servomotors that are hard to model and not overly powerful."*

### 7.2 IMU mounting orientation inconsistency

The assembly guide documents a known issue where the BNO055 has been mounted upside-down (flipped along X) in at least some builds. The guide acknowledges: *"It probably doesn't really matter a lot if you mount it upside down or not. You can configure how you mounted it later."* This is a build consistency risk - different builds may have different IMU orientations requiring per-robot calibration.

### 7.3 Vibration-induced screw loosening

The assembly guide explicitly warns about this: *"Everytime you screw something in the motors metal against metal, you want to use a little loctite threadlocker."* This is particularly called out for plastic-to-metal interfaces around the servo horns. The vibration during locomotion is sufficient to back out screws over time if not treated.

### 7.4 Thermal issues

No thermal issues are documented in any file in the repository. The STS3215 servos have no documented thermal protection behaviour in the BAM model or identification scripts. At 7.4V nominal bus voltage with R ≈ 2.04 Ω, stall current would be approximately 3.6A per motor, which could cause heating under sustained load. No temperature monitoring is implemented in the documented runtime.

### 7.5 Structural fragility of leg sheets

The thigh and shin are connected by twin parallel flat sheets (`left_knee_to_ankle_left_sheet`, `left_knee_to_ankle_right_sheet`) printed in PLA. These thin planar structures are the most mechanically critical and most fragile printed parts. A fall or unexpected motion at high speed could snap these. Community modifications (Jaime's Mods in `print/mods/v2_Jaimes_Mods/`) include reinforced `trunk_top_front` and `trunk_top_back` variants, suggesting the trunk joint area is also a failure point.

### 7.6 Cable routing complexity

The assembly guide notes that motor cables must be threaded through the `knee_to_ankle_right_sheet` during shin assembly - this must be done before the shin is assembled, as the cable cannot be rerouted afterwards. This is a non-obvious assembly-order dependency.

### 7.7 Knee range inconsistency between v1 and v2 simulation

v1 MuJoCo model has knee range ±120° (`-2.0944 2.0944`), while v2 is limited to ±90° (`-1.5708 1.5708`). The physical servo can rotate further, so the v2 restriction is a conservative software limit. Policies trained on v2 limits cannot directly transfer back to v1.

---

## 8. Simulation-to-Reality Pipeline Summary

The full pipeline from hardware to deployable policy:

1. **CAD** - Onshape model with material properties assigned per part; the current model has been corrected to remove a 37 mm right-leg offset that caused asymmetric MJCF output
2. **Export** - `onshape-to-robot` generates MJCF/URDF with masses and inertias from the CAD (overriding material defaults with slicer-estimated mass for infill accuracy); mate naming (`dof_`/`fix_` prefixes), entity order, and mate type all affect the output kinematic tree (see §2.11)
3. **Motor identification** - BAM (`experiments/v2/identification.py`) captures step-response data from physical STS3215; BAM fitting produces `params_m6.json`
4. **MuJoCo model** - `robot_motors.xml` uses BAM-identified joint dynamics; `bam.to_mujoco` provides damping, armature, frictionloss; BAM's `MujocoController` is optionally used at runtime to apply torques
5. **Reference motion** - Parametric walk engine (Open Duck Reference Motion Generator) produces polynomial spline trajectories used as imitation targets
6. **RL training** - Open Duck Playground (MuJoCo Playground framework) trains a policy with imitation + task rewards
7. **Deployment** - Policy exported as ONNX; Open Duck Mini Runtime on Raspberry Pi Zero 2W runs inference at 200 Hz with 4× decimation (50 Hz control)

The RL observation vector (56 elements): projected gravity vector (3) + joint positions (16) + joint velocities (16) + foot contacts (2) + previous action (16) + velocity commands (3).

---

*Sources: files read from `/home/lakieb/Documents/open_duck_mini_research/Open_Duck_Mini/` and `/home/lakieb/Documents/open_duck_mini_research/Open_Duck_Playground/`. Key files: `mini_bdx/robots/open_duck_mini_v2/robot.xml`, `robot_motors.xml`, `scene.xml`; `mini_bdx/robots/bdx/robot.xml`; `docs/assembly_guide.md`, `docs/sim2real.md`, `docs/configure_motors.md`, `docs/print_guide.md`, `docs/feetech_identification.md`; `experiments/v2/params_m6.json`, `experiments/v2/identification.py`; `mini_bdx/mini_bdx/utils/rl_utils.py`; `playground/open_duck_mini_v2/xmls/open_duck_mini_v2.xml`, `scene_flat_terrain.xml`.*
