# Model assumptions and reference results

Start with the visual [model summary](SIMULATION_DESIGN_SUMMARY.html) and [control summary](CONTROL_SUMMARY.html) for an overview.

For the implementation structure, force application and integration order, see [Simulation model design](SIMULATION_DESIGN.md). For feedback equations, state transitions and nozzle allocation, see [Control laws](CONTROL.md).

## Miniature profiles

The basic vehicle is 0.147 × 0.090 × 0.041 m with a fixed 0.120 kg body and box-estimated inertia. Its four wheel shapes are fixed to the body. In the basic experiments the initial gas is considered included in that fixed mass; gas loss does not reduce body mass, a deliberate approximation.

The half-pipe profile instead uses 0.120 kg dry mass (including four 0.001 kg wheels) plus approximately 0.04597 kg of gas from a 200 cm³, 20 MPa tank. Its joints/motors are active, and body mass/inertia are updated as gas is used. Gas is assumed symmetric about the chassis center with the same inertia-shape ratio; the actual distribution and hardware packaging are unverified.

The miniature fan has 450 Pa shutoff pressure, 0.003 m³/s free flow, 8 W maximum input and a 1000 J battery. Fan limits follow affinity laws, not a rectangular pressure/flow envelope. The skirt is 125 × 80 mm with 0.01 m² effective area, a 0.1 mm residual leak and, for the curved track only, up to 6 mm of assumed compliant travel. Compliance work and reverse-flow dynamics are not modeled. A perfect seal at zero gap assumes reverse flow through the stopped fan is blocked.

Four underside jets produce +z body force, two front lateral jets produce ±y force, and the rear jet produces +x force. Underside jets have a 3 N initial limit each, side jets 0.5 N each, and the rear jet 0.8 N. The instantaneous allocation matrix has rank five: front lateral force and yaw torque are coupled. Bottom-jet differential thrust creates roll/pitch. During inversion those jets can accelerate the car toward the ground; no top jet compensates for gravity.

## Experimental full-size profile

The example dimensions are 4.536 × 1.971 × 1.260 m; their limited external basis is documented in README. All apparatus values below are project assumptions:

- Dry mass 1600 kg, including four 20 kg wheels; initial gas approximately 482.72 kg by the ideal-gas equation.
- 600 L tank, 70 MPa absolute initial pressure, 293.15 K, nitrogen-like R = 296.8 J/(kg K), effective exhaust velocity 600 m/s.
- Four underside jets limited initially to 60 kN each, two front side jets to 20 kN each and the rear jet to 8 kN.
- Effective suction area 5 m², fan shutoff pressure 16 kPa, free flow 2 m³/s, input 30 kW, battery 36 MJ.
- Half-pipe radius approximately 13.89 m and height 27.77 m, built from the same independent block construction as the miniature course.

At 70 MPa the ideal-gas mass estimate can have substantial error. Constant exhaust velocity and an isothermal tank also omit nozzle/choked-flow behavior, cooling, valve limits and heat supply. Tank/vehicle strength, dry hardware mass and available volume are not established. This profile demonstrates a mathematical system under assumed inputs, not a practical full-size vehicle design.

## Numerical and logging conventions

Physics uses a 1 ms MuJoCo step with implicitfast integration. Pressure uses a monotone backward-Euler solve. The basic position is initialized once; sequences subsequently integrate actual forces/torques. Static camera previews replay already recorded physical poses for visual checks.

The half-pipe controller is specific to the documented seven-nozzle geometry/order. It uses position/attitude feedback, optional acceleration feed-forward in the full-size example, and pressure-dependent limits. A positive common bottom-jet bias may be necessary to achieve a torque without negative thrust, creating a force-allocation residual. That residual's mixed force/torque norm has no single physical unit. These are not arbitrary-wrench or general six-axis controllers.

CSV quaternions are w,x,y,z; linear velocities are world-frame and free-joint angular velocities are body-frame. Forces and seal information describe the preceding step, while pose/contact data is refreshed at the reported time. Contact load sums normal magnitudes. Maximum touchdown speed includes repeated floor contacts, not just the first touch.

## Reference checks

| Check | Reference / tolerance |
|---|---|
| 118 Pa × 0.01 m² | 1.18 N, absolute tolerance 1e-9 N |
| 235 Pa × 0.01 m² | 2.35 N, absolute tolerance 1e-9 N |
| Perfect seal, fan stopped | Retain 235 Pa for 1 s within 1e-6 Pa |
| Miniature steady leak | Approximately 353.034 Pa and 0.000646442 m³/s; flow balance relative tolerance 1e-5 |
| Force equals 0.120 × 9.81 N | Initial no-contact vertical acceleration approximately zero, tolerance 1e-8 m/s² |
| 1.5 × weight thrust | Initial vertical acceleration 4.905 m/s², tolerance 1e-8 m/s² |
| Offset nozzle | Generalized torque matches r × F within 1e-9 N m |
| Basic ceiling | Near 0.9795 m until fan shutdown, then fall |
| Basic lift | Peak center height approximately 0.20486 m; approximately 1.4889 g gas consumed |
| Miniature half-pipe | Minimum flight-recovery height approximately 0.155 m, first touchdown approximately 0.050 m/s |
| Full-size half-pipe | Minimum flight-recovery height approximately 7.24 m, maximum touchdown approximately 0.282 m/s |

These are implementation regression references, not empirical validation. Existing tests also cover invalid seals, wall force direction, motor momentum conservation, depletion and incomplete/failing scenarios.

Gravity remains 9.81 m/s² at both scales. Without support and with zero initial vertical velocity, falling for 0.25 s gives approximately 0.307 m at either mass. With geometrically similar vehicles and equal thrust-to-weight ratios, rotational acceleration scales roughly as 1/L; roll time therefore scales as sqrt(L), and fall distance during that roll scales as L. The current presets also change hardware, proportions and control, so their output difference does not isolate size alone.
