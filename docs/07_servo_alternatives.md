# Servo Motor Alternatives for Open Duck Mini

**Purpose:** Evaluate servo motor options to replace the Feetech STS3215 and achieve smoother, more natural duckling-waddle motion. The primary complaint is stiction - a tendency to stick in place and then suddenly lurch forward - which produces jerky, unnatural movement.

**Glossary of terms used throughout this document:**

- **Stiction** - Short for "static friction." The resistance a motor has to starting movement from a standstill. High stiction means the motor holds still until enough force builds up, then snaps suddenly to the new position rather than gliding smoothly.
- **Backlash** - The amount of "slop" or free play in a gear train. If you hold the motor shaft still and rotate the output shaft back and forth, the distance it moves before the gears engage is the backlash. High backlash = imprecise positioning and a rattling sensation during direction changes.
- **Gear ratio** - How many times the motor's internal shaft must rotate for the output shaft to complete one turn. A ratio of 1:345 means the motor spins 345 times for every one rotation of the output. High ratios give more torque but slower speed and, generally, more stiction from the gearbox.
- **Protocol** - The language the controller (e.g., Raspberry Pi) uses to talk to the servos. Different brands speak incompatible languages. Swapping servo brands usually means rewriting or replacing software.
- **BAM** - Better Actuator Models. A tool developed by Rhoban that measures how a specific servo actually behaves (friction, springiness, damping), then encodes those measurements into the MuJoCo simulator so that trained policies transfer correctly to the real robot.
- **TTL serial bus** - The communication standard used by Feetech servos. Multiple servos share a single wire, each identified by a numeric ID.
- **Current-based position control** - A control mode where the servo moves toward a target but limits how hard it pushes. This produces compliant, yielding motion rather than rigid snapping.
- **Coreless motor** - A motor with no iron in the spinning part. Iron-core motors have "cogging" - the magnets grab onto iron teeth causing small lurches at low speed. Coreless motors have zero cogging, giving much smoother slow motion.
- **Profile velocity/acceleration** - A setting that tells a servo to ramp up and ramp down speed gradually when moving, rather than jumping instantly to full speed. Produces smoother, more organic-looking motion.

---

## Part 1 - Current Servo: Feetech STS3215

### Specifications

The Open Duck Mini v2 uses 10 STS3215 servos (5 per leg). The STS3215 comes in several variants with different gear ratios and voltages:

| Variant | Voltage | Gear Ratio | Stall Torque | Rated Torque | Speed (no-load) |
|---------|---------|------------|-------------|--------------|-----------------|
| C001 | 7.4V | 1:345 | 19.5 kg·cm | 5.0 kg·cm | ~46 RPM |
| C044 | 7.4V | 1:191 | 27.4 kg·cm | 9.0 kg·cm | ~82 RPM |
| C046 | 7.4V | 1:147 | 14.4 kg·cm | 4.8 kg·cm | ~107 RPM |
| C018 | 12V | 1:345 | 30.0 kg·cm | 10.0 kg·cm | ~46 RPM |

**Physical:** 45.2 × 24.7 × 35 mm, weight 55 ± 1 g  
**Encoder:** 12-bit magnetic encoder, 4096 steps per 360°, resolution 0.088°/step  
**Communication:** Half-duplex TTL serial, IDs 0-253, daisy-chainable  
**Gear material:** Metal  
**Motor type:** Marketed as coreless on some listings; one source (RobotShop) states "core motor" - this is unconfirmed and the point is contested  
**Price:** approximately $14-$27 per unit depending on variant and quantity  

### What Independent Testing Revealed

Independent testing by Robo9 (robonine.com) measured the following on a real unit:

**Backlash:** 0.87° measured - this is nearly double the manufacturer's rated spec of ≤0.5°. This measured value represents mechanical slop from the gear train. When the robot changes direction (e.g., moving a leg forward then backward), the gears must travel through this 0.87° of free play before they engage. This produces the characteristic "loose" feel and contributes to jerky movement.

**Firmware dead zone:** A built-in 10 encoder count dead zone means the servo ignores commanded positions within ±0.88° of its current location. This combines with mechanical backlash to create a combined effective dead zone where no corrective action occurs. For fine, smooth trajectory following, this is significant.

**Repeatability:** Despite the above, repeat positioning is actually good - mean deviation of ~0.17° (2 encoder steps) across multiple cycles. The servo is consistent, but consistent within a dead zone.

**Dynamic deflection under load:** At 1.5 kg load (relevant to leg joint loads during walking), position deviates by 20-30 encoder steps (1.8-2.6°) from the commanded position. This means the servo is consistently behind where the policy expects it to be during movement.

**Temperature:** Reaches 60-71°C under oscillating load - worth noting for continuous operation.

**Speed accuracy:** Maximum ~46 RPM, with 7% fluctuation. This fluctuation contributes to irregular motion.

### Why This Causes the Duck-Waddle Stiction Problem

The STS3215 has a high gear ratio (1:345 on the C001). This is the core mechanical problem:

1. **High gear ratio magnifies stiction.** The motor's own static friction is multiplied 345 times through the gearbox. Even a small amount of friction in the motor becomes substantial resistance at the output.
2. **The dead zone means no response to small commands.** When the policy sends a small corrective command (within 10 encoder counts), nothing happens. The joint stays still. When the accumulated error grows large enough to exceed the dead zone, the servo suddenly moves to catch up - producing the lurching behaviour.
3. **Backlash compounds this.** The combined effect of 0.87° mechanical backlash plus 0.88° firmware dead zone means the servo is effectively blind to commanded movements within roughly ±1.75° of its current position.
4. **No current-based position control.** The STS3215 positions using a PID loop that drives toward a target at full available torque. There is no "soft landing" mode that would let it yield and settle gradually.

### Community-Reported Issues

- Communication errors (`[TxRxResult] Incorrect status packet!`) reported by multiple users on the HuggingFace LeRobot community, particularly during high-frequency control loops
- Chinese-only documentation with no official English SDK; community has produced Python wrappers, but these are unofficial
- The Waveshare ST3215 is electrically and firmware-identical - same problems apply

### BAM Characterisation Status

The Feetech STS3215 at 7.4V has been pre-characterised and parameters are available in the Rhoban BAM repository at `params/feetech_sts3215_7_4V`. These five parameters (damping, kp, frictionloss, armature, forcerange) are used directly in the MuJoCo MJCF model. If a different servo is chosen, new BAM characterisation must be run - this involves building a pendulum test rig, recording multiple trajectory datasets, and running a computational optimisation. The process requires hardware setup time and some days of iterative fitting.

---

## Part 2 - Alternative Options

### Category 1: Drop-in Feetech-Compatible Replacements

These servos use the same TTL serial protocol as the STS3215, meaning the Open Duck Mini Runtime's software stack would need minimal changes - only servo IDs and possibly a few register addresses would need updating.

---

#### Option 1A: Feetech STS3215 C046 (Lower Gear Ratio Variant)

**The core idea:** The standard Open Duck Mini uses the C001 with a 1:345 gear ratio. Switching to the C046 (1:147 ratio) uses the same servo body, same protocol, same physical mounting, but with a gearbox that is 2.3× less reduction.

| Spec | C001 (current) | C046 (alternative) |
|------|----------------|---------------------|
| Gear ratio | 1:345 | 1:147 |
| Stall torque | 19.5 kg·cm | 14.4 kg·cm |
| Rated torque | 5.0 kg·cm | 4.8 kg·cm |
| No-load speed | ~46 RPM | ~107 RPM |
| Price | ~$15-20 | ~$15-20 |

**Why this might help:** A lower gear ratio means stiction is multiplied fewer times. The motor's static friction is divided by 147 to get the effective threshold at the output shaft - substantially lower than with a 345:1 ratio. The servo will be more responsive to small commands and less prone to "sticking then jumping."

**Why this might not be enough:** The underlying motor is the same. The dead zone firmware problem (10 encoder counts) remains. The backlash of the gearbox with 147 stages will be different - potentially better or worse than the measured 0.87° on the C001. No independent backlash measurements for the C046 have been found.

**Torque consideration:** 14.4 kg·cm stall versus 19.5 kg·cm. For a 42 cm, ~1 kg robot, worst-case joint torque needs are roughly 5-8 kg·cm. The rated torque of 4.8 kg·cm on the C046 is close to this limit. Marginal.

**Software changes:** None beyond confirming register compatibility (same protocol, same register map).

**BAM recharacterisation:** Required - different gear ratio and motor dynamics.

**Price impact on BOM:** Neutral - same price point.

---

#### Option 1B: Waveshare ST3215-HS (High Speed Variant)

This is Waveshare's "high speed" variant of their ST3215, which itself is firmware-identical to the Feetech STS3215.

| Spec | Value |
|------|-------|
| Stall torque | 20 kg·cm @ 12V |
| No-load speed | 106 RPM (0.094 s/60°) |
| Voltage | 6-12.6V |
| Encoder | 12-bit magnetic, 4096 steps |
| Protocol | TTL serial bus (identical to Feetech) |
| Weight | Similar to STS3215 (~55 g, unconfirmed) |
| Dimensions | Similar to STS3215 (unconfirmed identical mount) |
| Price | ~$23-24 per unit |

The HS variant explicitly advertises an "acceleration start-stop function that makes the action softer" - this is a firmware-level ramp feature similar to a profile acceleration setting.

**Why this might help:** Higher no-load speed (106 RPM vs 46 RPM) implies a lower effective gear ratio, which means lower stiction amplification. The built-in acceleration ramp reduces sudden starts and stops.

**Why to be cautious:** This is the same product family - same underlying Chinese OEM hardware. The firmware dead zone and backlash problems likely persist. "Acceleration start-stop" is a feature of the STS3215 too (listed as a selectable function), so the improvement may be marginal.

**Software changes:** Protocol-compatible. Would need physical mounting verification.

**BAM recharacterisation:** Required.

**Price impact on BOM:** Slight increase (~$40-90 more for 10 servos), still well within $400 target.

---

#### Option 1C: Feetech STS3250 (Larger, Heavier, More Precise)

A larger servo in the Feetech family, the STS3250 has been independently tested and shows significantly better backlash than the STS3215.

| Spec | Value |
|------|-------|
| Stall torque | 50 kg·cm @ 12V |
| No-load speed | 77.6 RPM measured |
| Voltage | 12V |
| Weight | 74.5 g |
| Dimensions | 45.22 × 24.72 × 35 mm (same width/depth as STS3215) |
| Encoder | 12-bit magnetic, 4096 steps |
| Protocol | TTL serial bus (Feetech compatible) |
| Backlash (measured) | 0.43° - within manufacturer spec and well below STS3215's 0.87° |
| Repeatability (measured) | ±0.02 mm at 95 mm radius (±0.012°) - exceptional |
| Price | ~$73-84 per unit (eBay); ~$37-74 on Alibaba depending on quantity |

**Why this would help:** The measured backlash of 0.43° is roughly half the STS3215's measured 0.87°. Repeatability at ±0.012° is dramatically better than the STS3215's ±0.17°. The higher torque rating means it will operate further from its limits, reducing the tendency for the PID loop to saturate and "snap."

**Why to be cautious:** 74.5 g per servo vs 55 g for the STS3215 means a 20 g increase per joint. With 10 servos, that is 200 g added to the robot's mass - not trivial for a ~1 kg bipedal robot. At ~$73-84 per unit, 10 servos cost $730-840, which alone exceeds the $400 BOM target. The dimensions are reportedly similar but this has not been independently verified as a direct drop-in.

**Software changes:** Protocol-compatible. Physical mounting needs verification.

**BAM recharacterisation:** Required.

**Price impact on BOM:** Severely negative - potentially doubles the servo cost alone.

---

### Category 2: Better Hobby Servos (Different Protocol)

These require replacing both the servo hardware and rewriting the servo communication layer in the Open Duck Mini Runtime. The BAM characterisation must also be re-run from scratch.

---

#### Option 2A: Dynamixel XL430-W250-T

This is Robotis's mid-range smart servo. It has been used in small walking robots and is well-supported in both hardware and software.

| Spec | Value |
|------|-------|
| Stall torque | 1.4 Nm (14.3 kg·cm) |
| No-load speed | 57 RPM |
| Voltage | 11.1V |
| Weight | 65 g |
| Dimensions | 28.5 × 46.5 × 34 mm |
| Gear material | Metal |
| Protocol | Dynamixel Protocol 2.0 (TTL) |
| Control modes | Velocity, position, extended position, PWM |
| Backlash | Not specified by Robotis |
| Price | $27.50 per unit |

The XL430 has the same body dimensions as the XM430 and XC430, so upgrading later is straightforward.

**Key limitation:** The XL430 lacks current-based position control (current-based position mode). This means it still uses a stiff PID-to-target approach and cannot yield gracefully under load. Compared to the STS3215, however, the Dynamixel firmware is significantly better documented and the SDK (DynamixelSDK, Python and C++) is mature, allowing precise tuning of profile velocity and profile acceleration registers. These allow you to set how quickly the servo ramps up and slows down on every movement, which is a major tool for smoothing out motion.

**Why this helps smoothness:** Dynamixel's profile velocity / profile acceleration system creates a trapezoidal velocity curve - the servo accelerates smoothly into movement and decelerates smoothly into the target. The STS3215's control loop lacks this sophistication. The Dynamixel SDK also supports synchronised write commands, enabling all 10 servos to receive their new target positions simultaneously and precisely timed.

**Software changes:** Complete servo driver rewrite. Replace Feetech SCServo SDK with DynamixelSDK. The Open Duck Mini Runtime would need a new servo interface module. This is a significant but well-trodden path - DynamixelSDK has Python bindings for Raspberry Pi.

**Physical mounting:** The XL430 at 28.5 × 46.5 × 34 mm is larger in one dimension than the STS3215 (45.2 × 24.7 × 35 mm). Some 3D-printed parts would need to be redesigned for the different bolt pattern and body shape.

**BAM recharacterisation:** Required. Dynamixel MX-64 and MX-106 already have BAM parameters in the Rhoban repository, so the process is proven.

**Price impact on BOM:** With 10 servos at $27.50 each = $275 for servos. The STS3215 at $15/unit = $150. Net increase: ~$125. This is within the $400 target if other BOM items remain similar.

---

#### Option 2B: Dynamixel XC330-T288-T (Recommended for Smoothness)

The XC330 is the upgraded version of the XL330, adding metal gears, bearings, and - critically - a coreless DC motor. This is the servo that the Open Duck Mini project author considered for their own development (they noted switching to xc330-M288-T for the legs, describing them as "more powerful").

| Spec | Value |
|------|-------|
| Stall torque | 0.92 Nm (9.4 kg·cm) @ 11.1V |
| No-load speed | 65 RPM @ 11.1V |
| Voltage | 6.5-12V (recommended 11.1V) |
| Weight | 23 g |
| Dimensions | 20 × 34 × 26 mm |
| Gear ratio | 288.35:1 |
| Gear material | Full metal, 2 bearings |
| Motor type | Coreless DC motor |
| Protocol | Dynamixel Protocol 2.0 (TTL) |
| Control modes | Current, velocity, position, extended position, current-based position, PWM |
| Profile support | Yes - velocity-based and time-based profiles with configurable acceleration |
| Backlash | Not specified |
| Price | $103.39 per unit |

The T-suffix means TTL (3.3V/5V compatible); the M-suffix model (XC330-M288-T, $103.39) uses a 5V power source rather than 11.1V.

**Why this is the best motion-quality option in this price tier:**

1. **Coreless motor.** The absence of iron teeth in the rotor eliminates cogging - the small magnetic "catching" that causes jerkiness at low speeds and is a primary contributor to stiction in iron-core motors. At the low speeds used in walking, a coreless motor glides instead of notching through magnetic poles.

2. **Current-based position control.** This is a fundamentally different way of moving. Rather than commanding "go to position X with maximum force," you command "go to position X but only use up to Y amps of current." The servo yields to external forces instead of rigidly fighting them. For bipedal walking, this means leg joints can absorb ground contact softly rather than snapping to position.

3. **Acceleration and velocity profiles.** The servo can be configured to ramp smoothly into and out of every movement, using Dynamixel's Profile Acceleration (register 108) and Profile Velocity (register 112). This software feature alone can produce more natural-looking motion even before any RL policy changes.

4. **Lighter weight.** At 23 g vs the STS3215's 55 g, each joint is 32 g lighter - a total of 320 g saved across 10 servos. This significantly changes the robot's inertia profile.

**Key concern - torque:** 9.4 kg·cm stall torque versus the STS3215's 19.5 kg·cm. For hip and knee joints bearing the robot's weight, this may be marginal. The Open Duck Mini project's own author noted this concern ("they are more expensive but way more powerful" - suggesting the XC330 was chosen precisely because the XL330 torque was insufficient). Detailed torque analysis per joint is needed before committing.

**Software changes:** Complete servo driver rewrite (same as XL430). Same DynamixelSDK path.

**Physical mounting:** Substantially smaller than the STS3215 (20 × 34 × 26 mm vs 45.2 × 24.7 × 35 mm). All 3D-printed servo mounts would need redesign. This is a significant mechanical rework.

**BAM recharacterisation:** Required. The XC330-M288-T is apparently what the project author used or tested, which may mean parameters exist informally in the community.

**Price impact on BOM:** 10 servos at $103.39 = $1,034. This is approximately $880 more than the STS3215 servo budget, and alone far exceeds the $400 total BOM target. This is the most significant obstacle.

---

#### Option 2C: Dynamixel XL330-M288-T (Budget Coreless Option)

The XL330 is the plastic-gear, lower-cost version of the XC330. Same coreless motor, same protocol, same dimensions, but with plastic gears.

| Spec | Value |
|------|-------|
| Stall torque | 0.52 Nm (5.3 kg·cm) |
| No-load speed | 103 RPM |
| Voltage | 5V |
| Weight | 18 g |
| Dimensions | 20 × 34 × 26 mm |
| Gear material | Plastic |
| Motor type | Coreless DC motor |
| Protocol | Dynamixel Protocol 2.0 (TTL) |
| Control modes | Current, velocity, position, extended position, current-based position, PWM |
| Price | $27.49 per unit |

**Why relevant:** Same coreless motor and current-based position control as the XC330, at a quarter of the price. Provides most of the motion quality benefits.

**Why concerning:** Plastic gears will wear quickly under the cyclic loads of walking. The project author explicitly moved away from XL330 to XC330 for the legs due to insufficient power and durability. At 5.3 kg·cm stall torque, this may be insufficient for hip and knee joints.

**Price impact on BOM:** 10 servos at $27.49 = $275. Similar to XL430, manageable within $400 budget.

---

### Category 3: Premium Options

These are included for completeness but are generally impractical for the Open Duck Mini's $400 BOM target.

---

#### Option 3A: Dynamixel XM430-W350-T

| Spec | Value |
|------|-------|
| Stall torque | 4.1 Nm (41.8 kg·cm) @ 12V |
| No-load speed | 46 RPM |
| Voltage | 12V |
| Weight | 82 g |
| Dimensions | 28.5 × 46.5 × 34 mm |
| Motor type | Coreless motor |
| Protocol | Dynamixel Protocol 2.0 (TTL) |
| Control modes | All modes including current-based position |
| Backlash | 15 arcminutes (0.25°) - specified by manufacturer |
| Price | $310.39 per unit |

The XM430 is the gold standard for small robot joints: coreless motor, low specified backlash, current-based position control, and abundant torque. At $310 per servo, 10 servos cost $3,100. This is entirely outside the $400 BOM envelope.

---

#### Option 3B: CubeMars AK60-6 (Quasi-Direct Drive)

Quasi-direct drive (QDD) actuators use a brushless motor with a very low gear ratio (6:1 in this case) rather than the 150-345:1 ratios in hobby servos. The low ratio means the motor's own cogging and friction have minimal effect on the output shaft.

| Spec | Value |
|------|-------|
| Rated torque | 3 Nm (30.6 kg·cm) |
| Peak torque | 9 Nm (91.7 kg·cm) |
| No-load speed | 320 RPM @ 24V |
| Voltage | 24V |
| Weight | 368 g |
| Dimensions | 79 mm diameter × 39.5 mm depth |
| Gear ratio | 6:1 |
| Backlash | 0.55° |
| Price | $298.90 per unit |

This actuator is used on quadruped robots like MIT Mini Cheetah. The 6:1 ratio means the motor is "almost" direct-drive - the gearbox barely amplifies friction, giving back-driveable, compliant joints.

**Why impractical for Open Duck Mini:** The AK60-6 weighs 368 g, is 79 mm in diameter, and needs 24V. The entire Open Duck Mini robot weighs approximately 1 kg. One AK60-6 would weigh more than a third of the whole robot. The form factor is completely incompatible with the existing design.

---

### Category 4: Unconventional Approaches

---

#### Option 4A: Brushless Gimbal Motor + Encoder (Custom QDD)

Small brushless motors from the drone/camera gimbal market (e.g., 2804, 3510 size motors) can be used as quasi-direct-drive actuators with a custom 3D-printed frame and an AS5048 magnetic encoder on the shaft. This approach is used by some small bipedal research robots.

**Concept:** A gimbal motor spinning at low RPM directly drives a joint. No gearbox means zero backlash and zero gear-induced stiction. The encoder provides position feedback to a BLDC driver (e.g., ODrive, SimpleFOC).

**Why interesting:** Theoretically the smoothest possible motion - direct drive with no intermediate mechanics. The robot would feel genuinely compliant.

**Why extremely difficult:**
- No existing mounting solution for Open Duck Mini - complete mechanical redesign
- Requires per-motor BLDC driver board (e.g., SimpleFOC Mini at ~$30 each - 10 units = $300 in driver boards alone before motors)
- Gimbal motors at the required scale (~$15-30 each) typically produce only 0.1-0.3 Nm, which may be insufficient
- Software stack must be written from scratch (no community support for this configuration in the duck context)
- BAM characterisation must be done for entirely novel hardware
- Extremely high integration risk

This is a viable research direction for a future hardware version but is not a practical near-term swap.

---

#### Option 4B: Herkulex DRS-0201

A Korean smart servo from Dongbu Robot with built-in trapezoidal velocity profiling.

| Spec | Value |
|------|-------|
| Stall torque | ~24 kg·cm @ 7.4V |
| Protocol | Proprietary UART (4-wire) |
| Built-in profile | Trapezoidal velocity profile (automatic) |
| Price | ~$40-60 per unit |

The DRS-0201 automatically creates a trapezoidal speed profile for every move, which inherently suppresses vibration from sudden acceleration. However, Dongbu Robot has reduced support for this product line, availability is limited, and the proprietary protocol has no Open Duck Mini community support. This option has poor risk-to-reward.

---

## Part 3 - Comparison Summary

| Servo | Stall Torque | Weight | Price/unit | 10-servo cost | Protocol change? | Mount change? | BAM needed? | Smoothness improvement |
|-------|-------------|--------|------------|---------------|-----------------|---------------|-------------|----------------------|
| **STS3215 C001** (current) | 19.5 kg·cm | 55 g | ~$15-20 | ~$150-200 | - | - | - | Baseline |
| STS3215 C046 (lower ratio) | 14.4 kg·cm | 55 g | ~$15-20 | ~$150-200 | No | No | Yes | Marginal |
| Waveshare ST3215-HS | 20 kg·cm | ~55 g | ~$24 | ~$240 | No | Verify | Yes | Marginal |
| Feetech STS3250 | 50 kg·cm | 74.5 g | ~$73-84 | ~$730-840 | No | Verify | Yes | Moderate |
| Dynamixel XL430-W250-T | 14.3 kg·cm | 65 g | $27.50 | $275 | Yes | Yes | Yes | Moderate |
| **Dynamixel XC330-T288-T** | 9.4 kg·cm | 23 g | $103.39 | ~$1,034 | Yes | Yes | Yes | High |
| Dynamixel XL330-M288-T | 5.3 kg·cm | 18 g | $27.49 | $275 | Yes | Yes | Yes | Moderate-High |
| Dynamixel XM430-W350-T | 41.8 kg·cm | 82 g | $310.39 | ~$3,100 | Yes | Yes | Yes | High |
| CubeMars AK60-6 | 30.6 kg·cm rated | 368 g | $298.90 | ~$2,990 | Yes - CAN bus | Full redesign | Yes | Very high - impractical |

---

## Part 4 - Recommendations

Three options are worth serious consideration, in order of practicality:

### Recommendation 1: Dynamixel XL430-W250-T - Best Trade-off Within Budget

**Why:** Profile velocity and acceleration registers allow smooth, shaped motion trajectories. Mature DynamixelSDK with Python support on Raspberry Pi. Same price per servo as the STS3215 C046 variant. The motion quality improvement comes primarily from software (profile acceleration) rather than hardware changes, and this approach is well-proven in small robot walking research.

**What it doesn't fix:** The XL430 lacks current-based position control. It is still stiff-position control but with better trajectory shaping. The motor type is not specified as coreless by Robotis for this model.

**Steps required:**
1. Rewrite servo driver in Open Duck Mini Runtime to use DynamixelSDK Python library
2. Redesign 3D-printed servo brackets (XL430 is a different form factor)
3. Run BAM characterisation on a pendulum test rig for XL430
4. Retrain policy or perform sim-to-real transfer tuning with new BAM parameters
5. Tune profile acceleration/velocity registers experimentally

**Budget impact:** ~$275 for 10 servos, which is within the $400 BOM target if other costs are controlled.

---

### Recommendation 2: Dynamixel XC330-T288-T - Best Motion Quality, Requires Budget Expansion

**Why:** Coreless motor with zero cogging, current-based position control for genuinely compliant movement, full profile support, and the lightest option (23 g per joint). This is exactly what the Open Duck Mini project author themselves moved toward. The motion quality difference between a coreless motor with current-based position control and the STS3215 is substantial and perceptible.

**The obstacle:** At $103.39 per servo, 10 servos cost over $1,000. This is 2.5× the entire target BOM. This option only makes sense if the $400 BOM target is abandoned for a research-grade build.

**Steps required:**
1. Same driver rewrite as XL430
2. Redesign 3D-printed mounts (the XC330 is smaller - 20 × 34 × 26 mm - which may actually simplify some designs)
3. Run BAM characterisation (or check if project community has already done this for the XC330)
4. Retrain or re-tune policy
5. Configure current limits and acceleration profiles

**Budget impact:** Exceeds $400 BOM target by approximately $700-800 in servo costs alone.

---

### Recommendation 3: STS3215 C046 (1:147 Ratio) - Lowest Risk Experiment

**Why:** Same protocol, same dimensions, same control board, same software. The only change is opening the servo and using a variant with a lower gear ratio. This is the least disruptive test of whether gear ratio is the dominant cause of stiction. If the duck moves more smoothly, it confirms the stiction hypothesis at minimal cost.

**What it doesn't fix:** The firmware dead zone remains. If the dead zone (rather than gear ratio) is the primary cause of jerky motion, this will show limited improvement.

**Steps required:**
1. Purchase a pair of C046 units (~$15-20 each)
2. Test on one leg
3. Run BAM characterisation for C046 (different gear dynamics)
4. Compare motion quality against C001 baseline

**Budget impact:** Identical to current BOM - neutral.

---

### Notes on BAM Recharacterisation

Regardless of which servo is chosen, changing servos requires re-running BAM. The process is:

1. Build a pendulum test rig (a weighted arm attached to the servo output, free to swing)
2. Record servo behaviour during five trajectory types: sin_time_square, sin_sin, lift_and_drop, up_and_down, and passive release
3. Post-process the recorded data
4. Run the optimisation fitting (computationally intensive - potentially hours)
5. Export the five parameters (damping, kp, frictionloss, armature, forcerange) to the MJCF model

The Rhoban BAM repository provides pre-characterised parameters for Dynamixel MX-64 and MX-106. No pre-characterised parameters exist in the public BAM repository for the XL430, XC330, or XL330 series, meaning this work would need to be done. The `zeroth-robotics/bam-feetech` fork suggests the broader community has extended BAM to other servo types, so tooling exists.

---

## Part 5 - What Other Small Bipedal Robots Use

The Open Duck Mini project author explicitly noted they switched to XC330 servos for their own development legs ("more powerful" and "more expensive"), indicating this is the intended upgrade path within the project itself.

Academic small bipedal robots (NU-Biped-4.5 at 1.1 m scale) use brushless DC motors with 14-bit encoders and planetary gearboxes - architecturally similar to quasi-direct drive but at a much larger scale. The Zippy robot (world's smallest autonomous biped, 2025) uses a single actuator with custom mechanical design - not transferable.

The small bipedal community broadly converges on two camps: cheap Feetech/clone servos accepting imperfect motion, or Dynamixel XC/XM series accepting higher cost for research quality. There is no widely-adopted middle ground.

---

*Sources consulted during research:*
- [Testing of Feetech STS3215 Servomotor: Backlash, Repeatability, and Torque - Robo9](https://robonine.com/testing-of-feetech-sts3215-servomotor-backlash-repeatability-and-torque/)
- [Feetech STS3250 Smart Actuator: Evaluation of Accuracy, Torque and Backlash - Robo9](https://robonine.com/feetech-sts3250-smart-actuator-evaluation-of-accuracy-torque-and-backlash/)
- [DYNAMIXEL XC330-T288-T - ROBOTIS](https://www.robotis.us/dynamixel-xc330-t288-t/)
- [DYNAMIXEL XL430-W250-T - ROBOTIS](https://www.robotis.us/dynamixel-xl430-w250-t/)
- [DYNAMIXEL XM430-W350-T - ROBOTIS](https://www.robotis.us/dynamixel-xm430-w350-t/)
- [XC330-T288-T eManual](https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/)
- [Open Duck Mini GitHub](https://github.com/apirrone/Open_Duck_Mini)
- [Open Duck Mini sim2real documentation](https://github.com/apirrone/Open_Duck_Mini/blob/v2/docs/sim2real.md)
- [Rhoban BAM Repository](https://github.com/Rhoban/bam)
- [STS3215 - Best Motors For Your Next Robot - Indystry.cc](https://indystry.cc/sts3215-best-motors-for-your-next-robot/)
- [Waveshare ST3215 Servo](https://www.waveshare.com/st3215-servo.htm)
- [Waveshare ST3215-HS specifications](https://www.waveshare.com/st3215-hs-servo-motor.htm)
- [How small motors allow a robot to walk with a natural gait - Orbray](https://orbray.com/special/en/columns/05_motor.html)
- [CubeMars AK60-6 V1.1 KV80](https://www.cubemars.com/goods-1142-AK60-6+V11+KV80.html)
- [Playing with Serial Bus Servo Motors - Aditya Kamath](https://adityakamath.github.io/2022-05-08-akros-final-update/)
- [SCServo Linux SDK](https://github.com/adityakamath/SCServo_Linux)
- [zeroth-robotics bam-feetech](https://github.com/zeroth-robotics/bam-feetech)
- [FeeTech 12V 30kg.cm Magnetic Encoding Servo STS3215 - RobotShop](https://www.robotshop.com/products/feetech-12v-30kgcm-magnetic-encoding-servo-sts3215)
- [Feetech STS3215 specifications - Seeed Studio](https://www.seeedstudio.com/STS3215-19kg-cm-7-4V-Serial-Servo-p-6338.html)
- [ROBOTIS DYNAMIXEL XL330 Current Based Position Mode - Lofaro Lab](http://wiki.lofarolabs.com/index.php/ROBOTIS_DYNAMIXEL_XL330_Current_Based_Position_Mode_(Compliant_Mode))
