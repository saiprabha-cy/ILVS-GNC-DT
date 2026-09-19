# Simplifications (documented, not hidden)

- **Non-rotating, spherically-symmetric Earth.** No J2 oblateness term, no
  Earth rotation (and therefore no rotational velocity contribution to
  orbital insertion, no launch-site-latitude/azimuth effects on achievable
  inclination). A rotating-Earth, J2-aware gravity model is a natural
  extension once the non-rotating baseline is validated.

- **Simplified exponential atmosphere**, not the full US Standard Atmosphere
  1976 tabulated model with its distinct tropospheric/stratospheric/
  mesospheric lapse rates. Accurate to roughly 10-15% through the regions
  that matter for this project's dynamic-pressure and drag estimates, but a
  real avionics team would use the tabulated model or better.

- **Planar (2-DOF translation + 1-DOF pitch), not full 6-DOF.** No yaw, no
  roll, no cross-range motion, no coupling between axes. The "3D" trajectory
  plot (`src/visualization/plot_trajectory_3d.m`) renders the planar
  trajectory embedded in a 3D axes system and says so explicitly in its own
  docstring, rather than implying true out-of-plane motion the model
  doesn't produce.

- **Constant thrust per stage**, not a thrust curve (real solid motors in
  particular have a time-varying thrust profile; even liquid engines have
  startup/shutdown transients). A `thrust(t)` curve is a straightforward
  extension of `stage_props.thrust` once a specific reference profile is
  chosen.

- **Constant Cd, constant Cn_alpha per stage**, not Mach-dependent
  aerodynamic coefficient tables. Real launch vehicles see significant
  Cd variation through the transonic regime; a simplified constant is used
  here to keep the aerodynamics model tractable for a first-pass educational
  project.

- **Linear, small-angle normal force model** (`N = 0.5*rho*V^2*Cn_alpha*
  alpha*area`), valid only for the small angles of attack this ascent
  profile is designed to produce (pitch program kick is 3 degrees). A large
  disturbance producing a large angle of attack would exceed this model's
  validity.

- **Single, shared PID gain set for both stages**, despite stage 2 having a
  much smaller pitch-axis inertia (9,000 vs. 120,000 kg·m²) than stage 1.
  Re-tuning gains per stage (or gain-scheduling on inertia) is a documented
  extension; the current gains are not claimed to be optimal for stage 2,
  only stable enough to demonstrate the closed-loop architecture.

- **Fixed, precomputed pitch program**, not a targeting algorithm that
  computes the pitch profile from a desired insertion state. A real ascent
  guidance system (e.g. a linear-tangent-law or explicit-guidance scheme)
  solves for the pitch profile; this project uses a fixed schedule as the
  guidance reference the control loop tracks.

- **No wind/gust disturbance actually applied**, despite being mentioned as
  a modeling goal. The atmosphere model has no wind field; robustness of
  the closed-loop pitch controller to a real disturbance is untested. A
  straightforward extension: add a wind-induced angle-of-attack perturbation
  term to `ascent_derivatives.m` and re-run the RMS-tracking-error check
  under it.

- **Vehicle performance is not claimed to reach orbital insertion energy.**
  See `docs/02_results_analysis.md` for how the MECO velocity deficit
  against local circular velocity should be reported honestly once you've
  actually run `main_ascent_full_run.m` -- matching real launch-vehicle mass
  fractions needed for LEO insertion is out of scope for this two-stage
  educational vehicle sizing, and the insertion-targeting *logic* (computing
  and reporting the deficit correctly) is what's being demonstrated, not a
  claim of achieving actual orbital energy.

- **Embedded Coder target is generic portable C**, not a specific flight
  computer. See `docs/01_design_decisions.md`.

- **The Simulink model covers only the pitch guidance/control loop**, not
  the full 7-state translational+rotational ascent dynamics. Porting
  `ascent_derivatives.m` into block-diagram form is a documented next step,
  not part of this delivery.