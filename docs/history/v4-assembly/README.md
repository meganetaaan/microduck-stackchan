# MicroDuck × Stack-chan / Tab5 MuJoCo prototypes

## Revision 3: rounded enclosure and openings

The latest scene is `prototype/rounded/scene_tab5_lowcube_rounded.xml`. All previous delivered models and the angular cutout candidate are preserved.

The box-like silhouette now uses **real curved mesh geometry**, rather than shading applied to square boxes:

- Outer case radius: **6 mm**; front foot-arch radius: **5 mm**; side-opening radius: **8 mm** with 1 mm additional relief
- The front bridge beneath Tab5 retains its **6 mm height**. Two hidden **4 × 3 mm support ribs** behind the device connect the chin to the roof
- Existing leg openings are preserved or enlarged. The Tab5 size, landscape orientation, position and inertia, and every original leg body, mesh, joint and inertial property are unchanged
- The shell is one connected, watertight solid with nominal **1.5 mm walls**. Rounded lower edges remove about 2.05 mm from the old lowest shell extremity; the result stays inside the prior overall bounding envelope
- Shell mass/inertia are integrated from the actual solid mesh using assumed PLA density 1240 kg/m³. Shell mass is **78.83 g**; total simulated robot mass **709.54 g**. Fastenings, detailed brackets, cables, material strength and printability are still unverified

### Collision representation and checks

The visual solid is partitioned into cells, with a separate convex collision hull around every connected cell piece. This produces **648 conservative pieces**, not a single hull that would fill the openings. Boolean checks confirm coverage within **0.000018 mm exported-coordinate tolerance**, with zero intrusion into the established angular leg-opening voids. Independent exported-mesh validation and direct compiled-model checks are included in `prototype/rounded/validation_report.json`.

The summed collision-hull volume is **6.11% larger** than the actual solid volume. This is a conservative approximation: contact can occur earlier than the visible curved surface within a cell. The collision mass is not used; explicit inertia comes from the actual watertight mesh. No collision was disabled to make the tests pass.

All **303 saved posture-envelope samples** and **181 feet-flat crouch samples** have zero enclosure/Tab5-to-leg contacts. Direct convex support-plane calculations give minimum sampled shell gaps of **5.42 mm** across the posture paths and **10.50 mm** across the crouch; Tab5 remains at least **11.44 mm** clear. These are numerical bounds for the tested convex proxies and discrete configurations, not proof of every continuous motion or manufacturing tolerance.

Using the unchanged finite-force BAM actuator model and policy adapter:

- Nominal 10 s walk: **1.014 m** forward, **0.169 m** sideways drift, maximum tilt **3.38°**, no fall or shell contact
- Noisy-start 10 s walk: **1.011 m** forward, no fall or shell contact
- Zero-command active-policy idle: 10 s upright, maximum tilt **0.99°**, no shell contact

The original SIT/FOLD and crouch cautions below still apply: SIT is not a validated seated equilibrium, the extreme FOLD pose also flags original-leg convex-proxy overlaps, and the feet-flat crouch starts from a different stance. A balanced/no-slip sit/crouch/fold transition has not been trained or demonstrated. Walking still has substantial tracking error. No hardware control or manufacturing approval is implied.

Latest presentation files:

- `prototype/rounded/rounded_closeup.png`: high-resolution angular/rounded comparison
- `prototype/rounded/rounded_before_after.png`: matched front, side and three-quarter views
- `prototype/rounded/walking_demo.mp4`: actual six-second MuJoCo walking rollout
- `prototype/rounded/geometry_report.json`: solid properties, conservative-cover construction and per-piece mapping
- `prototype/rounded/validation_report.json`: independent invariants, contact tests and clearance bounds

```bash
# bootstrap.sh now builds every preserved revision and the rounded variant
bash bootstrap.sh
.venv/bin/python prototype/render_rounded_comparison.py
.venv/bin/python prototype/rounded/validate_rounded.py
.venv/bin/python prototype/run_probe.py \
  --scene prototype/rounded/scene_tab5_lowcube_rounded.xml \
  --policy policies/alpha_walking.onnx \
  --out prototype/rounded/my_walk --seconds 10 --speed .3
```

## Revision 2: lower cube-like silhouette

The new separate variant is `prototype/models/scene_tab5_lowcube.xml`. Both first-version models and their results below are preserved.

**Requested shape changes are implemented without shortening or moving the legs:**

- Tab5 stays landscape at its real 128 × 80 × 12 mm size. Its center is lowered **47 mm**; the enclosure top in the same STAND pose falls from 240 to 193 mm above the floor
- Case depth increases **70 → 120 mm**. Width remains 132 mm; height becomes 118 mm. Including the face, the bounding envelope is approximately **132 × 132 × 118 mm**
- A **36 mm-deep chin** extends below the screen. Side hip openings and the open underside let the original legs emerge from the sides/lower body
- Walls are individual, genuinely hollow 1.5 mm panels, with matching collision geometry and mass/inertia. Side cutouts extend from trunk-relative Z −45 to +10 mm. No filled cube is hidden inside the leg mechanism
- The new model contains explicit shell/Tab5-to-moving-leg mesh contact pairs, including housings that were only visual meshes in the upstream floor-contact model. Contacts were not disabled to obtain walking

The 97.56 g shell is calculated with an **assumed PLA density of 1240 kg/m³** and solid 1.5 mm walls. This is an estimate, not a strength/printing certification. Existing 20 g mount and 35 g relocated-compute allowances remain assumptions. Total simulated mass rises **680.72 → 728.28 g** (+47.56 g). At the identical spawn pose the COM is **10.89 mm lower and 5.48 mm farther rearward**. Fixings, cables, actual component COM, electronics packaging, deflection, and power design are still unverified.

### Revision 2 verification

Same policy adapter, BAM model, 7.4 V, gravity, original leg chains, 5 ms timestep and 0.3 m/s command as the first comparison:

| Test | First bare-Tab5 model | Lower-cube model |
|---|---:|---:|
| Nominal 10 s forward travel | 0.950 m | 1.035 m |
| Nominal sideways displacement | −0.172 m | −0.159 m |
| Maximum trunk tilt | 3.64° | 3.43° |
| Falls / shell–leg contacts | 0 / not instrumented | 0 / 0, checked at 200 Hz |
| Three noisy-start 10 s trials | 0.963–0.969 m, no falls | 1.033–1.040 m, no falls or shell–leg contacts |

The lower-cube trials retain sideways drift of 0.159–0.255 m and do not track commanded speed accurately. A separate **active-policy zero-command idle** stayed upright for 10 s (maximum tilt 0.97°). A **fixed joint-target hold**, without policy feedback, instead fell backward in all three noisy-start tests after 0.64–1.28 s. Holding nominal joint angles is not a safe fallback for this revision.

Independent clearance audit of the compiled geometry:

- STAND: minimum shell-to-leg gap **10.50 mm**
- Nominal 10 s walking trace, 501 samples at 50 Hz: shell gap **10.18 mm**, at the chin/right hip bracket; Tab5-to-leg gap **12.79 mm**
- Bounded 243-pose joint grid: shell gap **8.00 mm**, Tab5 gap **10.51 mm**. Each leg independently varies yaw ±10°, roll ±5°, and hip pitch/knee/ankle ±10° about STAND; all 3⁵ combinations are evaluated with mirrored legs
- **Original SIT and FOLD poses collide with the lower shell and are unsupported.** They remain in the scene for negative tests, not as usable commands. The design does not preserve unrestricted full-range articulation

Distances are independently optimized **convex-mesh proxy** distances, using transformed compiled vertices, Qhull hulls, AABB pruning and constrained optimization. All tested solves converged. Discrete pose/50 Hz trajectory checks do not prove every continuous motion is clear, and convex hulls can conservatively overstate overlap for concave parts. Cables, tolerances and flex are excluded. `audit_lowcube_clearance.py` and its JSON report record the exact poses, hashes and tolerances.

Revision 2 deliverables:

- `prototype/results/lowcube_before_after.png`: equal-scale front/side/three-quarter renders, with dimensions
- `prototype/results/lowcube_walking_demo.mp4`: six seconds of actual dynamic walking
- `prototype/parameters_lowcube.json` and `build_lowcube.py`: adjustable enclosure parameters and generator
- `prototype/results/lowcube_validation_summary.json`: walking and hold metrics
- `prototype/results/lowcube_clearance_audit.json`: reproducible clearance audit

```bash
.venv/bin/python prototype/build_lowcube.py
.venv/bin/python prototype/run_probe.py \
  --scene prototype/models/scene_tab5_lowcube.xml \
  --policy policies/alpha_walking.onnx \
  --out prototype/results/lowcube_recheck --seconds 10 --speed .3 --save-states
.venv/bin/python prototype/test_models.py
.venv/bin/python prototype/audit_lowcube_clearance.py
```

No training, hardware control, remote repository changes, or deployment was performed. This remains a simulated enclosure concept; resolve measured clearances and the unsupported postures before manufacturing or physical tests.

## First revision: preserved baseline

This concept **walks in the tested simulation**, with significant limitations below. It is not manufacturing CAD, an electrically validated design, or a hardware deployment package.

## What changed

- Removed the complete four-joint neck/head subtree from the official ground-contact MJCF
- Kept original pelvis, base battery, both five-joint legs, foot collision meshes, limits, and inertial data
- Added a fixed box body and landscape Tab5 face with real collision geometry and explicit mass/inertia
- Ten real actuators remain; the trunk is free floating under normal gravity
- Created bare-Tab5 and second-battery variants

The controller is the official v5 `alpha_walking.onnx`, used through a deliberately simple **zero-shot diagnostic adapter**. Head position observations are fixed at the original reference position (zero relative offset); head velocities and past applied actions are zero. Only the ten leg outputs are applied. There are no phantom actuators or dynamically moving head, no body anchoring, no external force, and no pose animation. This is not a newly trained ten-output policy.

## Verified results

All numbers are for the current **simulation model**, not a weighed robot. Flat floor, BAM XL330 M6 actuator model, 7.4 V, 200 firmware gain, 5 ms physics / 50 Hz policy, and command ramp from zero to 0.3 m/s over 0.5 s. No domain randomization, sensor delay, backlash, pushes, or uneven terrain were applied. Normalizer is already baked into the official ONNX. Its metadata specifies projected gravity, which the runner uses.

| Model | Simulated mass | Nominal 10-s forward distance | Sideways displacement | Max trunk tilt | Falls |
|---|---:|---:|---:|---:|---:|
| Original MicroDuck | 737.24 g | 0.939 m | -0.271 m | 4.28° | 0 |
| Bare Tab5 concept | 680.72 g | 0.950 m | -0.172 m | 3.64° | 0 |
| Tab5 + its battery | 779.62 g | 1.094 m | -0.141 m | 6.57° | 0 |

With 0.01 rad Gaussian initial joint perturbations, seeds 1, 2, 3 each completed 10 seconds without falls. Bare concept forward travel was 0.963–0.969 m, sideways displacement -0.174 to -0.085 m; both feet repeatedly lifted (27–28 lift-offs each). A separate 30-second bare-concept rollout, seed 4, completed 2.893 m forward with 0.853 m sideways drift, no non-foot floor contacts, and max tilt 3.48°. Thus the gait is physically simulated and sustained in these trials, but **direction and speed tracking are poor**: 0.3 m/s command produces only about 0.095 m/s forward speed.

At 0.1–0.2 m/s commands the original and modified models mostly stood still. This was reproduced with the upstream inference class and with its legacy XML actuator fallback as a diagnostic, rather than being attributed to the body change. No physics parameters were changed to make the gait work. The alternative v5 `velstand` policy also had poor command tracking in our probe. The displayed clip uses `alpha_walking` at 0.3 m/s.

A fixed-reference-pose hold is a different test from active walking. Across three 3-second noisy-start holds, the bare model remained upright (maximum tilt 4.8–5.1°), whereas the original and kit model fell after approximately 0.9–1.3 seconds. **Do not assume the kit is passively stable or that holding a pose is a safe fallback.**

The nominal BAM voltage-bounded torque limit is ±0.9634 N·m; bare-model peak torque was 0.4353 N·m. Training/inference default does not enforce the firmware current limiter. An additional 1.75 A simulation sensitivity test produced the same nominal bare rollout, but the correct real servo voltage/current settings remain unverified. These figures are not hardware torque ratings or approval to power the robot.

Fall test: terminate if trunk tilt exceeds 60°, trunk origin height falls below 55 mm, or state becomes nonfinite. Every trial also records non-foot floor contacts and foot lift-offs. This is a feasibility probe, not a statistical reliability claim.

## Assumptions to replace with measurements

`prototype/parameters.json` controls sizes, masses, COM positions, and box placement.

- Current M5Stack documentation: Tab5 128 × 80 × 12 mm, 118.4 g; kit 217.3 g including 98.9 g battery. The older official PDF lists conflicting weights, so weigh the actual unit before mechanical decisions
- Shell 50 g, mount 20 g, relocated compute/cabling allowance 35 g are estimates; the head contained the original compute hardware, which must be replaced or relocated
- Box external dimensions: 70 mm deep × 132 mm wide × 84 mm high; shell inertia is a uniform 2 mm-wall hollow cuboid
- Tab5 and compute inertias use uniform cuboid approximations. Battery footprint and all component COM offsets are assumed
- The kit scenario adds the Tab5 battery while retaining the original base battery, deliberately exposing the extra-mass penalty. Bare scenario assumes a suitable regulated power solution, not yet designed
- The official MJCF contains older Pi Zero / battery mesh labels, while current real hardware documentation refers to another compute revision. Neither original nor replacement mass is a verified hardware measurement

The original removed neck/head cluster is 279.93 g in this MJCF; remaining pelvis + legs total 457.32 g. Replacement mass below 279.93 g alone does not prove balance, actuator margin, or walking feasibility.

## Files

- `prototype/build_models.py`: parametric model generator, leaving upstream source untouched
- `prototype/models/scene_tab5_bare.xml` and `scene_tab5_kit.xml`: loadable scene wrappers
- `prototype/run_probe.py`: reproducible headless BAM + ONNX rollout, CSV/JSON metrics and optional MP4
- `prototype/test_models.py`: load, joint-count, free-body/gravity, original leg limits/inertia/contact preservation checks
- `prototype/results/tab5_bare_walking_demo.mp4`: six-second rendered physics run
- `prototype/results/validation_summary.json`: seeded walking and pose-hold tests
- `prototype/results/*_030.json`: original / modified nominal 0.3 m/s command tests

## Reproduce

Python 3.12, git, curl, and a CPU are sufficient. EGL rendering needs a working OpenGL/EGL setup; headless physics does not need a GPU. No full RL/GPU training dependencies are installed.

```bash
bash bootstrap.sh
.venv/bin/python prototype/run_probe.py \
  --scene prototype/models/scene_tab5_bare.xml \
  --policy policies/alpha_walking.onnx \
  --out prototype/results/my_probe --seconds 10 --speed .3 --seed 1 --noise .01

# Optional: append --render for a video
# Original baseline:
.venv/bin/python prototype/run_probe.py \
  --scene microduck_rl/src/mjlab_microduck/robot/microduck/scene.xml \
  --policy policies/alpha_walking.onnx \
  --out prototype/results/my_baseline --seconds 10 --speed .3
```

Archive intentionally excludes upstream meshes and policy weights. The bootstrap fetches pinned official sources and checks the ONNX hash; review the upstream licenses before reuse. No remote repository was modified, no firmware flashed, no paid compute started, and no physical robot was controlled.

## Next technical steps

1. Weigh Tab5, enclosure, bracket, and relocated electronics; measure their COM and exact mounting position
2. Sweep those measured tolerances in this model, then train or adapt a proper 10-action policy with matching observations/rewards; repair low-command and yaw tracking
3. Test zero-command stopping, turning, starts, fall detection, model uncertainty, real servo/current limits, latency and backlash
4. Only after a separate hardware safety review consider restrained/tethered physical testing

Full upstream GPU training requires CUDA. The official README suggests roughly 1–2 hours for a usable stock gait at 4096 environments, but that is not an estimate for this custom morphology. No retraining was attempted here.

## Sources and provenance

- [MicroDuck RL pinned source](https://github.com/pollen-robotics/microduck_rl/tree/8d0db74916a4f833d1d9b95d6a1d7f4d13b9d5ec)
- [BAM lockfile revision](https://github.com/Rhoban/bam/tree/62bd8ce12154340be97e06f7f41a0ca8f116d967)
- [Official policies v5](https://huggingface.co/pollen-robotics/microduck-policies/tree/v5)
- [M5Stack Tab5 specifications](https://docs.m5stack.com/en/core/Tab5)
- [Current runtime source](https://github.com/pollen-robotics/microduck/tree/9060e81a6e598504003ff31f9f39e075e2c5290f)
- [MicroDuck hardware publication caveat](https://pollen-robotics.com/microduck/press-kit/)

MuJoCo 3.10.0; ONNX Runtime 1.30.0; policy SHA-256 `e36332d383997d51401897734cd3e79cf5038406feddb18b4d57ecfb141daa6c`.
