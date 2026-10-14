# Mobility and mechanical design

The 2026 vehicle, **Napoleon**, replaces the LEGO-Technic hybrid of the national
final with a screw-jointed monocoque, an Ackermann steering linkage and a rigid
rear axle. Three limitations of the old platform triggered the redesign: play in
the steering, parallel steering without Ackermann geometry, and a vehicle size
that was set by the standard Jetson developer kit. Moving to the compact Seeed A603 carrier board and an SMD-assembled main PCB
([chapter 2](03-power-sensors.md#pcb-implementation)) freed the space that made
a complete mechanical redesign possible. The key figures of both vehicles are
compared in [the overview](01-overview.md#the-vehicle-at-a-glance).

Since the national final the main assemblies went through about 205 CAD versions
(chassis v96, steering v40, C-Profile knuckle v54, tires v11, body v4), against
about 120 design cycles for the entire previous robot.

Tables that compare Napoleon with an earlier
vehicle always list the earlier vehicle first and Napoleon last; in the PDF the
earlier vehicles are shaded grey and Napoleon blue. The state of every test is
listed under [Validation status](#validation-status). Values without a test are
CAD data, data sheets or measurements documented in chapters 2 and 3.

## Chassis

### Structural concept

Every component of Napoleon is screwed directly to a single monocoque. The
national-final robot combined LEGO Technic with printed parts. Every LEGO
interface was a fit with clearance and therefore one more link in the tolerance
chain, so LEGO was removed completely. The detachable body carries no load
([Body](#body)).

The parts are joined in three ways. SLA parts get modelled threads. FDM parts get plain holes sized with
the screw tolerance, in which the screw cuts its own thread. Nuts are used only
in the steering linkage, as jam nuts, where the joints must stay free to rotate.
The ball bearings are pressed into the front knuckles (the C-Profiles) with a
vice and into the rear by hand. All fits are global user parameters in Fusion 360 and are
referenced by every part. Changing one value updates every fit in the robot. This
replaces the single LEGO scaling parameter of the previous design.

The table compares both chassis aspect by aspect:

| Aspect | National final (LEGO hybrid) | Napoleon | Effect |
|---|---|---|---|
| Base structure | LEGO frame + FDM parts | FDM monocoque + SLA precision parts, cable routing and screws modelled | no LEGO interface left in the tolerance chain |
| Structural joints | LEGO pins, at least 3 per printed part | ≈14 × M2, 2 × M2.5, 3 × M3 | defined preload instead of pin clearance |
| Bearings | none (plastic on a LEGO axle) | steel ball bearings 3 × 8 × 4 mm, pressed in | defined axis, low friction |
| Tolerances | one scaling parameter for LEGO holes | screw 0.22 mm, clearance fit 0.30 mm, press fit 0.08 mm (SLA: half the FDM values) | the same fit on every part |
| Replacing a component | partial disassembly, sometimes destructive | at most 2 screws, no other part removed first | fast repair during testing and competition |
| Full assembly | not recorded, cyanoacrylate needed | 30–40 min | faster rebuilds |

### Layout in four levels

The components are stacked vertically in four levels instead of being spread
out flat:

1. Base plate: carries the drive motor, recessed into the plate, and the IMU
   directly above the rear axle.
2. Steering level: sits directly on the base plate and holds the steering
   servo and the linkage. Its cut-outs follow the parts of the levels above and
   below.
3. Jetson level: the Jetson Orin Nano module on the A603 carrier, with the
   fan blowing downwards through the open base plate.
4. Top level: main PCB, camera holder and all connectors.

The RPLIDAR S3 sits in the front section on its own mount at the height of the
Jetson level. Its scan plane at 55 mm runs at the height of the main PCB, which
is why the PCB blocks the rear sector of the scan
([Sensor mounting](#sensor-mounting)). The 4S LiPo sits at the rear, above the
motor and the drive axle.

<img src="../figures/mobility_levels.png" width="90%" />
*Section views through the four levels: base plate, steering level, Jetson level (with the LiDAR mount in front), top level with the VW T1 body.*

<img src="../figures/mobility_top_view.png" width="50%" />
*Top view. Motor (left) next to the Jetson stack, bevel gears at the rear axle, LiDAR in front.*

The drive motor lies lengthways beside the Jetson module. A transverse motor
directly on the rear axle was the obvious alternative and was designed first
(V1 base plate, June, https://a360.co/4yrcmVN), but the Jetson stack ruled it
out. A transverse motor does not fit between the rear wheels: their inner faces
are 76 mm apart, the motor alone is 70 mm long, and the bevel or spur stage on
the axle needs another ≈10 mm, so the track and the width of the robot would
have had to grow. It would not have made the robot shorter either. The Jetson
module ends 3.4 mm in front of the rear axle, so a transverse motor would sit on
or behind the axle and add up to 25 mm of overhang, or push the Jetson forward
and lengthen the wheelbase.

Lying lengthways, the motor uses space that exists anyway: a strip beside the
Jetson module that is exactly as long as the module (70 mm against 69.6 mm) and
that the carrier board overhangs. The battery sits above it, so the motor adds
neither length nor width.

This was only possible after the differential was removed
([Rigid rear axle](#rigid-rear-axle)). The result is a vehicle about 20 mm
shorter than the transverse variant.

### Mass and centre of gravity

The mass and the centre of gravity (CoG) were measured on the robot ready to
drive, with the race battery and the camera on its holder, without the body
(tests T01/T02). Two kitchen scales of equal height give the axle loads $m_f$,
$m_r$ and the side loads. With the rear axle raised by $\Delta h = 40$ mm the
robot tilts by $\theta = \arcsin(\Delta h / L) = 23.1°$, and the front-axle load
rises to $m_f'$. With the effective wheel radius $r = 15$ mm:

$$x = L\,\frac{m_f}{m_f + m_r}, \qquad z = r + \frac{L\,(m_f' - m_f)}{(m_f + m_r)\tan\theta}$$

| Quantity | Measured |
|---|---|
| Mass, weighed as a whole | 586 g |
| Front / rear axle load | 322 g / 257 g (sum 579 g) |
| Left / right side load | 284 g / 295 g |
| Front-axle load with the rear raised by 40 mm | 377 g |
| CoG ahead of the rear axle, $x$ | 56.7 mm (55.6 % front-axle load) |
| CoG off the centre line | 0.9 mm to the right |
| CoG above ground, $z$ | 38 mm (22.7 mm above the axle) |

The CoG is computed from the sum of the axle loads, so that all values come from
the same pair of scales; weighed as a whole on another scale the robot showed
7 g more. One gram on the scale moves $z$ by about 0.4 mm. The robot is almost
exactly centred sideways. The CoG height is used for the traction limit
([Speed and acceleration](#speed-and-acceleration)) and the tipping limit
([Tires](#tires)).
Data: [`mobility_measurements.xlsx`](../data/manual/mobility_measurements.xlsx),
sheet `Mass_CoG`.

### Materials and manufacturing

Each part group gets the process that fits its load and precision, instead of one
material for the whole robot. The national-final robot used only PLA, because the
LEGO pins needed an accuracy of about 0.1 mm and PLA has a 21.1 % higher tensile
strength than PETG \[[1](99-references.md#ref-1)\]. Without LEGO interfaces the
process can be chosen per part:

| Part group | Process / material | Key property | Reason |
|---|---|---|---|
| Monocoque, front section, large parts | FDM, Bambu Lab PLA Matte (X1 Carbon; 0.2 mm layers, 3 walls, 12 % infill) | dimensionally stable, fast to iterate; slightly flexible | large volume, low cost, short print time |
| Steering knuckles (C-Profiles), servo horn, small linkage parts | SLA, Anycubic ABS-Like Resin Pro 2 (Photon Mono M7 Pro) | tensile strength 35–45 MPa, Shore D 82–84 \[[2](99-references.md#ref-2)\] | features too small for FDM; better surface accuracy, at the cost of washing and curing |
| Wheel rims | FDM, SUNLU PA6-CF (20 % carbon fibre) | flexural modulus 8.6 GPa, tensile strength 112 MPa, HDT 203 °C \[[3](99-references.md#ref-3)\] | the rim must not deform, so the tire is the only compliant element |
| Axles, tie rod | steel, purchased | – | no bending, replaceable, adjustable |
| Bevel gears | brass, purchased | – | wear-free tooth contact; standard bore, so the fit on the shaft is defined |
| Bearings | steel ball bearings 3 × 8 × 4 mm, purchased | – | defined rotation axis, fits the axle directly |

PA6-CF is hygroscopic and is dried in an AMS dryer before printing.

The steering parts moved to SLA because of wear. The first steering ball joints
were printed in FDM. After a few days of testing they showed measurable wear and therefore play.
At the same time the linkage parts became smaller with every iteration, until
their features were below what the FDM printers could reproduce reliably. A resin
printer was bought for this reason. Later the tie rod and the ball joints were
replaced by purchased steel tie-rod ends, because the SLA tie rod bent over time.

The C-Profile knuckles failed once by design: in one iteration their mounting
holes sat too close to the outer wall and the parts cracked. The
wall around the holes was thickened in the following versions (v44). A static
FEA of the failed and the current version is planned (test T16).

### Sensor mounting

The trade-off is the same as at the national final, LiDAR field of view against
compact electronics, but its weighting changed. The camera now gives every LiDAR
point a colour ([chapter 3](04-software.md#colour)), so it has to sit as close
as possible to the LiDAR scan plane, and the LiDAR has to be mounted low enough
that the fields of view of camera and LiDAR overlap. The national-final robot
tapered the front to the width of the servo driver; since the servo, motor and
LiDAR drivers moved onto the main PCB, that constraint is gone.

Coordinates are relative to the rear-axle centre on the ground, $x$ forward,
$z$ up.

| Sensor | National final (LEGO hybrid) | Napoleon | Mounting on Napoleon |
|---|---|---|---|
| LiDAR | InnoMaker STL-19P, scan plane $z \approx 60$ mm | Slamtec RPLIDAR S3, $x \approx 111$ mm, scan plane $z \approx 55$ mm | screwed to the monocoque, no isolation |
| Usable LiDAR field of view | ≈250° | 240° | the main PCB blocks the rear (measured from −33° to +59° around the rear); the software cuts ±60° |
| Camera | directional CSI camera, $z = 80$ mm | IMX219-200 fisheye on CSI (200° lens, facing the ceiling; until October a PiCam360 on USB), directly above the LiDAR | held by the body; separate holder without body |
| IMU | BNO055 near the rear axle | BNO055 above the rear-axle centre ($x \approx 0$ mm, $z \approx 5$ mm) | screwed to the base plate |

The LiDAR sits lower than before, which keeps the offset to the camera small. The
price is the field of view: the main PCB at scan height blocks the rear, so the
software uses 240° ([chapter 3](04-software.md#colour)), about 10° less than the
national-final robot. A full 360° view would need a lower Jetson and therefore a
custom cooler; a higher LiDAR would increase the camera offset again. The current
position is a deliberate compromise: what the robot needs is the view ahead and
to the sides, where the next straight and its pillars are.

The IMU sits above the rear-axle centre, the reference point of the kinematic
bicycle model, where the lateral velocity is zero as long as the tires do not
slip. Its yaw rate needs no lever-arm correction. Neither the IMU nor the LiDAR is
isolated:
[chapter 2](03-power-sensors.md#interference-measured-and-it-is-mechanical) shows
that the drivetrain vibration stays out of the yaw axis.

### Body

Napoleon carries a detachable body shaped like a VW T1 bus
(https://a360.co/3U2vnis). It has no structural function but holds the fisheye
camera and the LED lighting. With the body the vehicle measures 182 × 111 × 84 mm
(CAD). The body is designed and will be finished for the European Open in Zagreb;
all measurements in this chapter were taken without it.

The body sits on pogo pins, which locate it and power the LEDs, so no cable has
to be plugged in when it is mounted. When fitted, it replaces the separate camera
holder (7.35 g); the camera itself weighs 23.05 g with its USB cable.
Aerodynamics play no role at our speeds.

The shape was first modelled in Fusion 360 Alias. The final version is based on
a public model, scaled and cut to the wheelbase, the track and the Jetson level.

### CAD model and test stand

The Fusion 360 model is used as a digital twin, not only as geometry. Every
connection is a joint (rigid, revolute, slider or ball), so the steering can be
moved through its full range. Interference checks and section analyses run on the
moving assembly. Materials are assigned to all parts, so the CoG is read directly
from Fusion and only validated on the real robot.

The project consists of six top-level designs, each viewable in the browser:
[full assembly "Napoleon"](https://a360.co/3U2vnis),
[chassis](https://a360.co/46wfcwD),
[steering](https://a360.co/4j0TWGF),
[C-Profile knuckle](https://a360.co/4hYC528),
[tires](https://a360.co/4ynFopl) and
[robot stand](https://a360.co/4xTdXmt).
Bodywork, purchased parts (with a subfolder for the electronics) and obsolete
versions are kept in separate folders. Two scripts support the work: one names
sketches, joints and components automatically, one highlights under-constrained
sketches.

For seven versions the steering model did not match the physical linkage, because
the modelled joints did not reproduce the real kinematics. Since then every
steering change is first moved through its range in the digital twin and can then
be printed as an isolated steering test rig, which saves material and turnaround
time.

A dedicated stand lifts the wheels off the ground. Drivetrain and steering can be
analysed on it, and the software can drive without the robot moving; the
vibration sweeps in
[chapter 2](03-power-sensors.md#interference-measured-and-it-is-mechanical) were
recorded on it.

## Steering

### Development of the steering

The steering went through three stages. The LEGO rack and pinion of the regional
final had 2–4° of play from its tolerance chain (servo horn, axle, gear, rack),
and its gears skipped under load, which could have ended runs. The direct printed link of the national final reduced the play below 0.5°, but it was parallel steering
(0 % Ackermann), limited to ±35°, and occasionally broke or came loose at the
LEGO H-profiles. The new Ackermann linkage removes these weaknesses: it has a much larger steering lock, it is more compact, and, because of its custom fit, the part that holds the servo also positions the LiDAR.

![The three steering generations compared. Each cell gives the value; the colour and the symbol grade it from −− to ++.](../figures/steering_generations.svg)

All three generations use the same Waveshare SC09 servo. In the linkage it is
rotated by 12° to keep the tie-rod ends inside their articulation range
([Kinematic chain](#kinematic-chain)), and with paper inserts filling the joint clearances
its gearbox is now the largest remaining source of play.

### Kinematic chain

The SC09 drives an SLA servo horn. A steel tie-rod end connects the horn to one of the C-Profiles and to the tie rod, which links both steering arms. Each steering arm is part of a C-Profile knuckle that pivots on a steel rivet as its kingpin. The wheel axle runs in a press-fit ball bearing in the knuckle and is held axially by a retaining ring.
Before the ring was added, the front wheels slid out of their bearings; this was
the only mechanical failure in test runs since the switch to the monocoque.

The purchased tie-rod ends allow 20° of articulation, the printed ball joints
allowed 44°. To keep the joints inside their range at full lock, the servo was
rotated by 12°. As a result its travel is asymmetric: from the centre at 15.69° it
turns 67.9° to the right stop (83.62°) and 97.9° to the left stop (−82.16°). This
is one reason why the steering table
([chapter 3](04-software.md#lane-following)) is measured separately for each
side.

Camber, caster and toe are 0° by design. With cast tires, 0° camber gives the
largest contact patch.

### Ackermann geometry

The steering arms follow the classic Ackermann rule
\[[4](99-references.md#ref-4)\]: their extensions meet at the centre of the rear
axle. With the kingpin distance $k = 59.3$ mm and the wheelbase $L = 102$ mm, the
ideal steering-arm angle is

$$\beta = \arctan\frac{k}{2L} = 16.2°$$

The CAD model has 15.3° (left) and 16.0° (right), which is ≈100 % Ackermann by
this rule.

A four-bar linkage meets the ideal condition exactly only near straight ahead. The
ideal relation between the inner wheel angle $\delta_i$ and the outer wheel angle
$\delta_o$ is

$$\cot\delta_o - \cot\delta_i = \frac{k}{L}$$

The real linkage was solved numerically from the joint coordinates. It reproduces the angles of the Fusion motion study (58° / 36.5°).

| Inner wheel $\delta_i$ | Outer, linkage | Outer, ideal | Deviation | Local Ackermann share |
|---|---|---|---|---|
| 10° | 9.5° | 9.1° | +0.4° | 60 % |
| 20° | 17.8° | 16.7° | +1.1° | 66 % |
| 30° | 25.0° | 23.4° | +1.7° | 75 % |
| 40° | 30.9° | 29.4° | +1.4° | 86 % |
| 50° | 34.9° | 35.1° | −0.3° | 102 % |
| 58° (full lock) | 36.5° | 39.7° | −3.1° | 117 % |

![Outer wheel angle of the linkage against the ideal Ackermann angle.](../figures/mobility_ackermann.png)

Up to about 49° the outer wheel turns slightly too far (local share below 100 %),
near full lock too little. Below 30°, where the robot drives almost all the time,
the deviation stays under 1.7°. Geometrically the smallest radius at the
rear-axle centre is $R = L/\tan\delta_i + k/2 = 93$ mm; how much of it can be
driven is the topic of [Steering angle while driving](#steering-angle-while-driving).

### Static wheel angles and play

On the stand, with the front wheels free, the servo is commanded to four
positions in the software unit (−1 … +1, positive = left).

The line of each front wheel is traced on paper and measured with a set square
against the line of the rear axle (test T05, reading resolution ≈0.5°). The
bicycle-equivalent angle $\delta$ follows from
$\cot\delta = (\cot\delta_L + \cot\delta_R)/2$, the ideal outer angle from the
measured inner angle with the Ackermann relation above.

| Command | Left wheel | Right wheel | Bicycle $\delta$ | Ideal outer | Outer turns too far (+) / too little (−) |
|---|---|---|---|---|---|
| +1.0 (full lock left) | 41.0° (inner) | 33.5° (outer) | 36.9° | 30.0° | +3.5° |
| +0.3 | 13.5° | 13.5° | 13.5° | 11.9° | +1.6° |
| −0.3 | −12.5° | −12.5° | −12.5° | −11.1° | +1.4° |
| −1.0 (full lock right) | −30.0° (outer) | −47.0° (inner) | −36.9° | −33.4° | −3.4° |

The servo does not drive the linkage to its mechanical stop. The inner wheel
reaches 41° (left) and 47° (right) instead of 58° (CAD), and the
bicycle-equivalent angle is 36.9° on both sides instead of 45°. The smallest
static radius at the rear-axle centre is therefore $L/\tan\delta = 136$ mm
(CAD: 101 mm by the same formula). Although the inner angles differ by 6°, the
bicycle-equivalent angles agree to 0.1°, so for the vehicle as a whole the
steering is symmetric.

The Ackermann share matches the design only on average. At full lock left the
outer wheel turns 3.5° too far (local share 62 %), at full lock right 3.4° too
little (138 %). The CAD linkage predicts 87 % and 97 % at these inner angles, so
the mean of both sides (100 %) matches, but the split between the sides does
not. One degree at the outer wheel moves the share by about 10 %, so the
difference is larger than the reading error. The cause is not yet identified;
candidates are a tie rod that is slightly too long or too short and the
different steering-arm angles of the CAD (15.3° left, 16.0° right). At ±0.3 both
wheels read the same; the expected difference of 1.5° is close to the reading
resolution.

The steering is not linear in the command. At ±0.3 the wheels turn 43° per unit
of command, between 0.3 and 1.0 only 33° per unit: the curve flattens towards
the lock. A linear map through the CAD lock (0.45° per percent) is right at ±0.3
within 1° but 8° too high at full lock. The 1° difference between +0.3 and −0.3
is the straight-ahead trim: straight ahead is at −0.02, so +0.3 is 0.32 and −0.3
is 0.28 away from it, which gives 42° and 45° per unit.

Each position was approached from one side only, so the reversal play (same
command, approached from the left and from the right) is still to be
measured.

Data: sheet `Steering_Target_Actual`, which also holds the series of the LEGO
rack and the direct link for comparison.

The play comes from four links. A linear clearance $s$ along the tie-rod path
turns the steering arm (radius $r = 15.55$ mm, CAD) by
$\Delta\delta = \arctan(s/r)$. The clearances below are design estimates:

| Link | Pairing | Clearance | Wheel angle without insert | Wheel angle with paper insert |
|---|---|---|---|---|
| S | servo gearbox and spline (SC09) | ≈1° at the horn | 0.80° | 0.80° |
| G1 | servo horn ↔ tie-rod end | 0.2 mm | 0.74° | ≈0° |
| G2 | tie-rod end ↔ steering arm | 0.3 mm | 1.11° | ≈0° |
| G3 | kingpin in knuckle | 0.2 mm | 0.74° | ≈0° |
| Worst case (sum) | – | – | 3.38° (±1.69°) | 0.80° (±0.40°) |
| Statistical (root sum square) | – | – | 1.72° (±0.86°) | 0.80° (±0.40°) |

A paper insert of 0.08–0.10 mm fills the hole clearances of G1–G3. What remains is
the servo gearbox, about 0.8° at the wheel. Without closed-loop
correction, play causes a curvature error $\kappa = \Delta\delta/L$; after 0.5 m
that is 1.8 cm of lateral offset without inserts and 0.9 cm with them, which the
controller has to correct continuously.

The linkage gives up some of the directness of the national-final direct link in
exchange for correct Ackermann angles and a much larger lock. Against the LEGO
rack the play is clearly lower. The next step is a servo with a magnetic encoder
and a stiffer gearbox (e.g. Feetech STS3032, 12-bit, 4.5 kg·cm stall torque).

### Steering angle while driving

The mechanics reach 58° at the inner wheel, a bicycle-equivalent angle of 45°
(CAD, from the formula above). The servo drives the linkage to 36.9° on both
sides (measured, [Static wheel angles and play](#static-wheel-angles-and-play)).
While driving, the robot reaches only about two thirds of that. The steering calibration
([chapter 3](04-software.md#lane-following)) derives the effective angle from the
measured yaw rate and speed:

| Full lock | Static, measured (CAD) | Driven, 0.35 m/s | Driven, 0.50 m/s | Driven, 0.75 m/s |
|---|---|---|---|---|
| Left | 36.9° (45°) | 21.9° | 22.1° | 19.4° |
| Right | 36.9° (45°) | 24.7° | 23.8° | 23.4° |
| Smallest radius at the rear-axle centre | 136 mm (101 mm) | 217–248 mm | 227–246 mm | 231–284 mm |

The difference is too large for steering play. The cause is the rigid rear axle
([Rigid rear axle](#rigid-rear-axle)). At a radius of 0.1 m the
inner and outer rear wheel would need speeds that differ by a factor of three, but
the axle forces them to turn equally, so both scrub. The scrub creates a yaw
moment against the turn, and the robot follows a wider circle than the front
wheels point to.

The loss is not simply proportional to the angle. Comparing the static
measurement with the driven table at 0.35 m/s (static value at ±0.35 linearly
interpolated between the measured points):

| Command | Static | Driven, 0.35 m/s | Loss |
|---|---|---|---|
| +1.0 | 36.9° | 21.9° | 15.0° |
| +0.35 | 15.2° | 3.8° | 11.4° |
| −0.35 | −14.2° | −8.8° | 5.4° |
| −1.0 | −36.9° | −24.7° | 12.2° |

On the right the loss grows with the angle, as scrub predicts. On the left the
driven angle stays close to zero up to a command of about 0.2 and then rises
parallel to the static curve, like an offset. Scrub alone does not explain this;
open candidates are the straight-ahead trim of the calibration, the slip angle
of the front tyres and play under load. Speed adds a smaller share on top:
between 0.35 and 0.75 m/s the effective full lock drops by 1.3° (right) to 2.5°
(left), i.e. 0.7–1.7° per m/s² of lateral acceleration.

This is why the software never uses a fixed servo-to-angle model: the measured
steering table per speed already contains the scrub.

### Steering speed

According to the data sheet the SC09 needs 0.1 s per 60° without load, so its
165.8° of travel take 0.28 s. On the stand, a step from the centre to the left
stop (97.9°) took 0.35 s, including 50 ms until the horn started to move. At
0.5 m/s the robot travels about 20 cm during a full lock-to-lock change, which
limits the speed in S-curves between pillar rows.

### Circular test drives

Two circular runs on 25 September were logged with the drive monitor. The
software commanded a constant speed and yaw rate; the speed came from the
encoder, the yaw rate from the IMU.

| Run | $v$ commanded | $v$ measured | Yaw rate commanded | Yaw rate measured | Ratio | Radius | Oscillation | Wheel frequency |
|---|---|---|---|---|---|---|---|---|
| 1 | 0.30 m/s | 0.300 m/s | 0.30 rad/s | 0.589 rad/s | 1.94 | 0.51 m | 3.18 Hz | 2.98 Hz |
| 2 | 0.50 m/s | 0.505 m/s | 0.50 rad/s | 1.132 rad/s | 2.21 | 0.45 m | 5.31 Hz | 5.02 Hz |

- The speed control deviated by less than 1 % in steady state.
- The steering turned about twice as far as commanded, more so at higher speed.
  These two runs still used the steering table of the previous vehicle. With
  the table measured again on Napoleon
  ([chapter 3](04-software.md#lane-following)), the ratio of measured to
  commanded yaw rate over all test runs from 26 September is 1.13 in the median
  (0.9–1.5, [chapter 3](04-software.md#lane-following)).
- The robot came back to within a few millimetres of its start point after each
  lap. The error was systematic, so a calibration could remove it.
- A superimposed oscillation scaled exactly with speed, once per wheel
  revolution (ratio 1.06). Its cause was the tire casting, not the steering:
  [Tires](#tires) describes how it was found and removed.

## Drivetrain

Napoleon uses a 25GA370 gear motor (1000 rpm at 12 V, sold as "BORDSTRACT"), with
an integrated Hall encoder that gives 408 counts per wheel revolution
([chapter 2](03-power-sensors.md#wheel-encoder)). It drives the rear axle 1:1.
The encoder closes the speed loop, which the national-final robot did not have,
and it allowed the 12 V motor rail to be removed from the PCB
([chapter 2](03-power-sensors.md#the-12-v-rail-was-removed--and-software-is-why)).

![Drivetrain. Motor with bevel gear, second bevel gear on the D-shaft, rear wheels with cast silicone tires.](../figures/mobility_powertrain.png)

### Motor selection

On the national-final robot the position of the motor tied the length of the
robot directly to the length of the motor. A motor with an encoder is longer, so
it would have made the robot too long. Napoleon decouples the two: the motor lies
lengthways beside the Jetson module, off-centre and recessed into the base plate
([Layout](#layout-in-four-levels)).

![Drive motor candidates. Values from the data sheets \[[5](99-references.md#ref-5)–[7](99-references.md#ref-7)\]; the colour and the symbol grade each value from −− to ++ for this robot.](../figures/motor_selection.svg)

The 25D 4.4:1 is geared for 3.7 m/s and has the lowest stall torque of all candidates. At our driving speeds of 0.35–0.75 m/s it would run at 10–20 % of its speed range. Between the regional and the national final the robot drove with the faster Pololu 25:1 for several iterations, before the 20D 31:1 replaced it. That motor had already shown that a small torque reserve at low duty makes the launch non-linear. Its encoder also gives only half the resolution.

The 25D 9.7:1 is in the same class as the 25GA370 in speed and torque. It was rejected because its encoder does not work with the 3.3 V logic level of the ESP32-S3. Adapting it would have meant a new PCB revision with two to three weeks of lead time, for a more expensive motor, although the 25GA370 is fully sufficient.

The 37D 10:1 has the most torque and the finest encoder, but it weighs 190 g, about twice the 25GA370, and its 37 mm diameter would have needed a deeper recess or a higher Jetson level. More torque also buys nothing on this robot: driving into a wall at full duty, the tires lose grip at a winding current of about 0.53 A, far below stall ([chapter 2](03-power-sensors.md#the-operating-envelope-is-bounded-by-traction-not-by-stall)).
The drivetrain is limited by traction, not by the motor.

The 25GA370 leaves enough headroom for faster speed profiles: at 100 % PWM on 4S
the speed controller works with 1.65 m/s, more than twice the 0.75 m/s currently
used on straights.

### Speed and acceleration

The theoretical no-load speed $v_0$ follows from the output speed $n$, the gear
ratio $i$ and the wheel diameter $d$:

$$v_0 = \frac{n \cdot \pi \cdot d}{60 \cdot i} = \frac{1000 \cdot \pi \cdot 0.032\ \text{m}}{60 \cdot 1} \approx 1.68\ \text{m/s}$$

The 4S pack (14.8 V nominal) is above the rated 12 V; with speed proportional to
voltage, 100 % PWM would give 2.07 m/s without load.

Acceleration can be limited by traction or by the motor. Only the rear axle is driven, so traction depends on the rear-axle load, which rises with acceleration through load transfer. With the friction coefficient $\mu$, the CoG height $h$ and the distance $l_f$ from the CoG to the front axle:

$$a_\text{traction} = \frac{\mu \cdot g \cdot l_f / L}{1 - \mu \cdot h / L}$$

| Quantity | Method | Value |
|---|---|---|
| Theoretical top speed, 12 V | calculation | 1.68 m/s |
| Theoretical top speed, 14.8 V, 100 % PWM | calculation | 2.07 m/s |
| Top speed at 100 % PWM used by the speed controller | speed calibration of the bridge (parameter `v_max`) | 1.65 m/s |
| Top speed, 100 % PWM, 15.9 V | **estimate**, the full-throttle step (T07) is not yet measured | ≈ 1.72 m/s |
| Time constant $\tau$ (63.2 %) | **estimate**, T07 not yet measured | ≈ 0.34 s |
| Max. acceleration | **estimate**, T07 not yet measured | ≈ 4.6 m/s² |
| Traction limit, rear-wheel drive | formula above with the measured CoG and $\mu_\text{long} = 1.18$ (T08, [Grip](#tires)) | ≈ 9 m/s² (6.7 m/s² with the lateral $\mu = 0.98$ as a lower bound) |
| Deceleration after a halt command | fitted to the stopping distances of runs 38–42 ([chapter 3](04-software.md#obstacle-strategy)) | 0.57 m/s² |
| Rolling resistance incl. drivetrain drag | $c_r = a/g$ | ≤ 0.058 (upper bound, it also contains the drag of gearbox and motor) |
| Limiting factor at launch | wall test ([chapter 2](03-power-sensors.md#the-operating-envelope-is-bounded-by-traction-not-by-stall)) | traction |

The estimated acceleration stays well below the traction limit, and the wall
test shows the tires slipping long before the motor stalls. The full-throttle
step (T07) that will replace the three estimates is prepared: the test script
[`step_test.py`](../../src/esp_bridge/esp_bridge/step_test.py) and its export
[`t07_export.py`](../analysis/t07_export.py) are in the repository. Stall torque and stall
current were therefore not measured: they are never reached. The vehicle has no
active brake; it coasts, and the controller triggers every halt early by the
coasting distance ([chapter 3](04-software.md#obstacle-strategy)).

### Power transmission

The motor is held by two screws and rests in a recess along its full length,
which takes the reaction torque straight into the monocoque. A pair of brass
bevel gears turns the drive by 90° onto a continuous steel D-shaft that carries
both rear wheels. The rims are press-fitted onto the D-shaft; no slipping has been
observed. The front axles run in press-fit ball bearings in the knuckles and are
secured with retaining rings.

The national-final robot used LEGO ABS axles, which bent under load, and LEGO
gears, which skipped and could fall out. Both were replaced by steel axles and
brass bevel gears.

The rear bevel gear first sat on the shaft through an
improvised adapter and ran out of true. The IMU vibration sweep in
[chapter 2](03-power-sensors.md#iteration-locating-and-removing-the-vibration-source)
found it before the part was inspected. Repairing the adapter cut the pitch noise
by 23–86 % but added 4–20 % friction from the tighter fit. The adapter was then
replaced by a gear with the same tooth count whose bore fits the D-shaft
directly: the low-speed friction penalty is largely gone and the yaw noise, the
axis the EKF uses, fell by another 23–73 %. The remaining resonance at
0.4–0.6 m/s comes from the wheels.

### Rigid rear axle

The differential was removed on purpose. Purchased differentials were too large,
and a ball differential designed for this robot did not fit either. Without it
the motor could lie lengthways and the chassis became shorter. With a rigid axle
both rear wheels turn at the same speed; in a corner of radius $R$ with the track
width $T$ the inner wheel has to slip forwards and the outer wheel backwards by
about

$$s \approx \pm \frac{T}{2R}$$

| Radius at the rear-axle centre | Slip per rear wheel | Required speed ratio inner / outer |
|---|---|---|
| 150 mm | ±32 % | 0.68 / 1.32 |
| 300 mm | ±16 % | 0.84 / 1.16 |
| 450–510 mm (circular test drives) | ±9–11 % | 0.90 / 1.10 |
| 1000 mm | ±5 % | 0.95 / 1.05 |

This is the central trade-off of the drivetrain: a shorter, simpler vehicle
against tire scrub and understeer in tight corners
([Steering angle while driving](#steering-angle-while-driving)). Above
500 mm the slip is below 10 %, and the planner keeps every arc at
$R \geq 0.30$ m ([chapter 3](04-software.md#lane-following)), so the effect was
accepted and is handled by the measured steering table. Tighter turns are possible at reduced speed. A custom differential for next season is in development.

### Tires

The LEGO Spike tires were replaced by self-cast silicone tires on PA6-CF rims,
Ø 32 × 15 mm (https://a360.co/4ynFopl). No purchased tire of this size had the grip we
needed, and a smaller wheel lowers the whole vehicle and leaves more room for the
steering lock inside the knuckles.

The tires are cast from TFC Troll Factory BL200
\[[8](99-references.md#ref-8)\], a two-component moulding silicone mixed 1:1,
medium-hard at Shore A 35 and translucent. The hardness is a compromise: softer
silicone would grip more but compress further under load and change the rolling
radius, harder silicone would lose grip on the mat. The translucent material
shows air bubbles before a tire is mounted, so faulty casts can be sorted out early.

The rim is deliberately stiff (flexural modulus 8.6 GPa
\[[3](99-references.md#ref-3)\]), so all compliance sits in the tire and the
rolling radius depends only on the silicone.

A tire is cast in five steps:

1. Print the rim (PA6-CF) and the casting mould.
2. Treat the mould with release agent.
3. Centre the rim in the mould.
4. Mix both components 1:1 and pour the silicone in through a separate funnel, so
   the tread surface stays free of a sprue.
5. Remove the funnel and let the tire cure for about 45 minutes.

The casting mould went through one important iteration. The once-per-revolution
oscillation of the test drives ([Circular test drives](#circular-test-drives), 3.18 Hz at 0.30 m/s against a wheel frequency of 2.98 Hz) pointed at the wheels. The moulds had been printed with an aligned seam: every layer started at the same angle, which left a small ridge across the mould wall and therefore a bump on every tire at the same position. The mould was reprinted with a random seam, and the centring of the rim was improved. With the new tires the oscillation was gone.

The finished tires were measured as follows:

| Quantity | Value | Source |
|---|---|---|
| Nominal diameter | 32.0 mm | mould |
| Measured diameter, 4 tires, 3 positions each | 31.8–32.1 mm (mean per tire), single readings 31.6–32.1 mm | calipers, test T12 |
| Runout (max − min per tire) | ≤ 0.4 mm | calipers, test T12 |
| Mass per wheel (rim + tire) | 9 g, all four (scale resolution 1 g) | test T17 |
| Effective rolling radius under load | 15.0 mm | encoder model of the EKF ([chapter 3](04-software.md#localisation)) |
| Effective diameter from 10 × 2.00 m | 30.03 mm | test T04 |
| Lateral deviation after 3 m straight, steering at 0 | 21 mm | test T12 |

The effective radius is 6 % smaller than the nominal one: the Shore A 35 silicone
is compressed under the robot's weight. This is why the encoder needs to be calibrated on the driven distance and not on the mould diameter.

The grip of the tires, the friction coefficient $\mu$, is measured on an inclined board covered with
competition mat (test T08, 02.10., 3 runs each). $\alpha$ is the angle, read to
1° from a phone inclinometer, at which the robot starts to slide. "Clean" means
the tires were washed with water; "after 3 runs" means three runs on the mat
afterwards without cleaning.

For the lateral value the robot stands across the slope with the steering held
at 0. No wheel can roll sideways, so $\mu = \tan\alpha$. The longitudinal value
is harder to get, because the front wheels cannot be blocked. The robot
therefore stands front downhill with only the rear wheels blocked. Only the rear
axle holds it, and the slope shifts load from it to the front axle. The balance
of forces with $l_f = L - x = 45.3$ mm and $h = 38$ mm gives

$$\mu_\text{long} = \frac{L \tan\alpha}{l_f - h \tan\alpha}$$

| Direction | $\alpha$ clean | $\mu$ clean | $\alpha$ after 3 runs | $\mu$ after 3 runs |
|---|---|---|---|---|
| Lateral | 44.3° (43–45°) | 0.98 | 44.3° (43–47°) | 0.98 |
| Longitudinal (front downhill, rear wheels blocked) | 20.0° (19–21°) | 1.18 | 21.3° (21–22°) | 1.30 |

The lateral value is direct: one degree changes it by about 0.035. The
longitudinal value is much less certain, because the denominator shrinks as
$\alpha$ grows. One degree changes it by about 0.09 (1.09–1.27 over the three
clean runs), and an error in $h$ adds to that. Within this accuracy the silicone
grips about equally in both directions, $\mu \approx 1$. Facing uphill, the
same test would be less sensitive and would give the traction limit directly
as $g \tan\alpha$.

Three runs without cleaning changed neither direction by more than the reading
resolution, so on this mat dust costs no measurable grip. Washing the tires
before every calibration is kept as a precaution. The test does not show that it
is needed.

With the measured CoG height $h = 38$ mm and the track $T = 96.2$ mm the robot
would only tip at a lateral acceleration of $g \cdot (T/2)/h = 12.5$ m/s². With
$\mu = 0.98$ it slides sideways at 9.6 m/s², so it always slides before it tips,
with a margin of 1.30. Even the highest single reading (47°, $\mu = 1.07$) leaves
a margin of 1.19.

### Ground clearance and suspension

The ground clearance dropped from 8 mm to about 2 mm; the lowest points are the
tie-rod ends. This lowers the CoG further. The WRO field is a flat mat, so the chassis
is rigid: a suspension would only cost space. 2 mm are enough on a
properly laid mat.

### Kinematic model

The software describes the vehicle with a kinematic bicycle model
\[[9](99-references.md#ref-9)\] around the rear-axle centre. The CAD wheelbase is
$L = 102$ mm; the software rounds it to 0.10 m. The real steering angle comes from
the measured table ([chapter 3](04-software.md#lane-following)), which already
contains the scrub of the rigid axle.

## Iterations

The mechanical design took about seven months from a first component layout to
"Napoleon". The steering was developed in parallel on its own test rig.

| Date | Milestone |
|---|---|
| 5 March | first concept: components arranged without a chassis, still partly LEGO |
| 21 June | V1 base plate, motor still transverse (previous motor) |
| 4 July | ball joints in the steering |
| 7 July | Jetson mounted with the fan facing down |
| 15 July | new steering geometry, new drive motor |
| 25 July | "Chassis Vertikal" started in parallel to the base plate; steering is now built as a separate test rig when needed|
| 8 August | LiDAR, servo and motor drivers moved onto the main PCB |
| 14 August | screws modelled, wheels adapted, materials assigned in the digital twin |
| 8 September | steel tie rod purchased, servo rotated by 12°; reliable test runs close to competition-level from here on |
| 24 September | chassis named "Napoleon" |

### Mechanical trade-offs

![Mechanical trade-offs: what each decision gained and what it gave up.](../figures/mobility_tradeoffs.svg)

### Rejected iterations

- Transverse motor on the rear axle: did not fit between the rear wheels and
  made the robot wider without making it shorter
  ([Layout](#layout-in-four-levels)).
- Ball differential: designed in-house, too large for the space.
- Printed ball joints (FDM): worn after a few days; replaced by SLA, then by
  steel tie-rod ends.
- Seven steering versions on an unrealistic digital twin: the CAD kinematics
  did not match the real linkage; since then every change is checked on the
  separate test rig.
- Thin C-Profile knuckles: cracked at holes too close to the outer wall; wall
  thickened.
- Aligned seam in the tire mould: once-per-revolution bump; mould reprinted
  with a random seam.
- Gear adapter on the drive shaft: ran out of true; found by the IMU sweep
  ([chapter 2](03-power-sensors.md#iteration-locating-and-removing-the-vibration-source)).

### Lessons learned

- Every removed interface removes a link from the tolerance chain. This was the
  main reason to drop LEGO completely.
- Global tolerance parameters make fits reproducible across materials and
  printers; one value changes every fit.
- Wear parts that can be bought (tie-rod ends, gears, axles) should be bought, but
  only once the prototype geometry is final.
- A second robot would let us develop without taking apart the only working
  vehicle. For next season we will print 1:1 replicas of the expensive
  electronics to build one.

### Future work

Each item below answers a limitation that is measured or described earlier in
this chapter.

- Custom aluminium cooler for the Jetson: The LiDAR scan plane runs at the
  height of the main PCB, so the robot loses 120° of view at the rear
  ([Sensor mounting](#sensor-mounting)). A flatter cooler would allow a lower
  Jetson and with it a free 360° scan, without raising the LiDAR and increasing
  the camera offset again.
- Body as heat exchanger: The body already covers the whole robot. Using it
  as the heat-exchanging surface of that cooler adds cooling area without adding
  height or a separate part.
- Steering servo with a magnetic encoder (e.g. Feetech STS3032): With the
  paper inserts, the SC09 gearbox is the largest remaining source of steering
  play (≈0.8°, [Static wheel angles and play](#static-wheel-angles-and-play)).
  A stiffer gearbox and 12-bit position feedback would reduce it further.
- Balanced wheels: With the adapter-free drive gear, the remaining vibration
  is a resonance at 0.4–0.6 m/s that disappears when the wheels are taken off
  ([Power transmission](#power-transmission)). Balancing the rims and tires, or
  checking their runout on the stand, is the next step.
- Custom differential: The rigid axle is the reason why the robot drives only
  about half of its mechanical steering lock
  ([Steering angle while driving](#steering-angle-while-driving)).
  Purchased and ball differentials were too large, so it has to be designed to
  fit the existing package without making the robot longer or wider.

## Validation status

| Test | Status | Section |
|---|---|---|
| T01/T02 masses, axle loads, CoG | done without the body (01.10.); single masses only for the wheels, the camera and its holder | [Layout](#layout-in-four-levels) |
| T04 encoder distance calibration | done: effective diameter 30.03 mm from 10 × 2.00 m | [Tires](#tires) |
| T05 static wheel angles and play | done (01.10.): lock, Ackermann share, linearity; reversal play not measured | [Static wheel angles and play](#static-wheel-angles-and-play) |
| T07 full-throttle step | open: top speed, $\tau$ and acceleration are estimates; test script and export ready | [Speed and acceleration](#speed-and-acceleration) |
| T08 inclined board | done (02.10.): $\mu$ lateral and longitudinal, clean and after 3 runs, sliding and tipping limits; LEGO comparison not measured | [Tires](#tires) |
| T12/T17 tire geometry and mass | done (01.10.): diameters, runout, 9 g per wheel; straight-line deviation | [Tires](#tires) |
| T13 servo step | steering step time | [Steering speed](#steering-speed) |
| T14 LiDAR field of view with and without body | – (240° from the software cut, see [Sensor mounting](#sensor-mounting)) | [Body](#body) |
| T16 FEA C-Profile old vs. v54 | – | [Materials and manufacturing](#materials-and-manufacturing) |

The driven steering angles (formerly tests T06/T09) come from the measured
steering calibration ([chapter 3](04-software.md#lane-following)).
