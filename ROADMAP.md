# Roadmap

## Implemented examples

- Miniature floor downforce, initialized ceiling attachment/release, and a thrust-driven lift pulse.
- Lumped fan P–Q/leak/pressure dynamics, battery use, finite ideal-gas blow-down, point forces and torques.
- Seven-nozzle miniature and experimental full-size half-pipe sequences: wheel-driven suction climb, edge detection, roll-180, controlled recovery and soft landing.
- Scenario-specific PD control, pressure-aware constrained allocation, changing mass/inertia, first/repeated touchdown checks, CSV logging and Viewer exhaust arrows.
- Side/axial camera recordings with an appended slow-motion airborne replay.

## Model validation

1. Compare fan curves, efficiency, sealing gaps and skirt compliance with measurements; examine step-size sensitivity.
2. Add real-gas compressibility, adiabatic blow-down, heat transfer, nozzle/valve flow limits and regulator behavior. The present full-size high-pressure ideal-gas case needs substantial validation.
3. Account for hardware mass, tank strength, packaging, energy supply and the gas distribution's effect on center of mass/inertia.
4. Compare scales with controlled geometry, thrust-to-weight ratio and dimensionless parameters. The current two profiles are different assumed systems, not an isolated scale study.

## Motion and control

- Extend wheel-drive validation to flat walls and ceilings; current flat-ceiling behavior is attachment/release, not powered driving.
- Add a standalone hover scenario and disturbance-recovery criteria; generalize allocation without claiming unavailable instantaneous control authority.
- Implement continuous floor launch, flight, approach, inversion, fan-assisted ceiling capture and holding, with state-based success/failure checks.
- Add manual controls and trajectory/energy diagnostics.
- Add calibrated full-size vehicle profiles alongside the miniature examples.

## Publication and maintenance

- Review whether the retained Git author email may be published; see the publication review.
- Review future contributions and assets for provenance and license obligations.
- Verify additional supported Python/dependency combinations before advertising them as tested.

CFD, ROS 2, reinforcement learning and a web frontend remain outside the current scope.
