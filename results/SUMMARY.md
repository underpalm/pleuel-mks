# Connecting rod: real part vs. equivalent system (MuJoCo 3.17.0)

## Connecting rod (Onshape, steel)
- Mass m = 467.79 g, eye distance L = 140.0 mm
- Centre of mass s = 44.04 mm from the big end
- Moment of inertia about S (axis ⟂ plane of motion) J_S = 1.50691e-03 kg·m²

## Equivalent systems
| Model | m_A (big end) | m_S (centre of mass) | m_B (small end) | J about S | Error J |
|---|---|---|---|---|---|
| Two-mass (statically equivalent) | 320.64 g | – | 147.15 g | 1.9769e-03 | +31.2 % |
| Three-mass (dynamically equivalent) | 244.41 g | 111.21 g | 112.17 g | 1.5069e-03 | 0 % |

## Simulation
Crank radius 40 mm, piston mass 350 g, speed 3000 rpm (held constant),
time step 1e-05 s, gravity off (inertia forces dominate), no gas force, frictionless.

| Quantity | Peak (real) | max. deviation two-mass | max. deviation three-mass |
|---|---|---|---|
| Crank torque M [Nm] | 53.26 | 1.897 (3.56 %) | 0.000 (0.00 %) |
| Crank pin force \|F_A\| [N] | 3789.21 | 101.435 (2.68 %) | 0.007 (0.00 %) |
| Piston pin force \|F_B\| [N] | 1776.46 | 96.906 (5.45 %) | 0.003 (0.00 %) |
| Piston side force F_N [N] | 361.48 | 103.079 (28.52 %) | 0.002 (0.00 %) |

## Validation
- Piston acceleration vs. analytic solution: max. error 0.005 %
- Speed ripple of the drive: 0.0017 %
- Mean crank torque over one revolution (lossless → 0): 8.37e-04 Nm
- Torque error of the two-mass model: simulation 1.897 Nm, theory ½·ΔJ·λ²·ω² = 1.893 Nm

## Figures
- comparison_forces_torques.png – curves over one revolution
- deviations.png – difference equivalent system − real rod
