# Selected portrait design: printable curved body concept

The latest model is `prototype/portrait_print/scene_tab5_portrait_print.xml`.
Earlier portrait and landscape models remain preserved. The previously delivered
rounded-front print design is preserved at `prototype/portrait_print_v1/`. This revision develops
actual solid geometry for trial printing; it is not a validated hardware kit.

## Shape and visual language

The original MicroDuck thigh has a broad flat face, a tapered outline, and a
smooth perimeter roll. A circular fit to the supplied thigh mesh gives roughly
R4 outside and R3 inside at the sampled edge. These are estimates from the
triangulated mesh, **not official CAD dimensions**.

The new body follows that approach:

- 80 ×80 ×80 mm body, with a square width/depth footprint
- Broad flat side surfaces and actual R6 side/rear corners, with R3.6 inner corners
- The front Tab5 mating rim ends square on X41 mm: zero depth-direction fillet.
  Its R6 outline is retained so the corners stay within the Tab5 rear silhouette
- 2.4 mm nominal walls; large R10 rounded side-window corners
- A removable rear panel with recessed fasteners, keeping the original simple
  mechanical appearance
- Body underside raised from Z43 to46 mm, above retained housings and leg space
- Unchanged portrait Tab5 at (47,0,62) mm, camera at the top, face only in the
  upper display half. Tab5 adds12 mm to the front: overall upper depth92 mm

The openings are empty in the solid model and collision geometry. The feet,
servos, leg lengths, joints and original pelvis are unchanged.

## Files and units

`prototype/portrait_print/parameters.json` controls the main body, window,
material and mounting dimensions. `build_print_design.py` is the editable
parametric Boolean-solid source. Hardware details are also explicit in that
source and remain provisional.

Four printable components are supplied as watertight, single-component STL:

1. Body cage
2. Rear cover
3. Central mount
4. Compute support pedestal

`stl/` uses assembly coordinates. `stl_print_oriented/` contains suggested build
orientations, with the lowest point at Z0. **Both STL sets use millimetres.**
MuJoCo OBJ assets use metres. `print_orientation.json` records the reversible
rotation and translation for each print-oriented part.

The main cage is oriented with its front toward the build plate. Rear cover lies flat; the tall central mount lies on its side. Local supports are
expected at the cage’s opening roofs. These are suggested orientations, not a
verified slicer profile or support-free claim. No print G-code is provided.

Nominal wall2.4 mm, rear counterbore residual floor1.4 mm, small mount radii and
pilot bores should be checked in the intended printer/material workflow. Print a
hole/nut-pocket fit coupon before printing the whole assembly. The design has not
been strength-tested or printed on a physical printer.

## Front interface revision

Only the final6 mm of the body depth is changed. The front rim and its mounting
lands meet the nominal Tab5 rear plane at X41 mm; no material extends into its
12 mm bounding envelope. The side and rear curves, device position and mounting
hole positions are unchanged. The60 ×64 mm R8 center opening is retained for
rear components and access; the back is not covered by a full slab.

This is a planar nominal mating surface, **not a zero-tolerance press fit**. The
official rear drawing shows components, sockets and fasteners but does not specify
all relief depths or rear flatness. Those details, cable clearance and M3 screw
engagement still need a physical fit measurement before printing for installation.
The simple Tab5 simulation solid does not resolve each rear-board component. The unchanged
top mounting lands have an inherited0.006 mm² polygon-outline discrepancy against
the simplified Tab5 mesh; the revised rim adds no outside footprint.

## Assembly concept and unresolved interfaces

- The body rests on a separate rounded central mount. The upper interface has
  nominal Ø2.2 mm pilot/clearance features; the final fastening method and pilot
  fit must be selected and checked on printed parts. It is not a certified
  threaded attachment.
- A thin metal wide washer and M2 central screw clamp the existing trunk slot. This avoids
  assuming that the earlier posts correspond to stock screw holes. The modeled
  original plate is about1 mm thick: clamp load capacity, local bending, cable
  access and physical stock geometry must be checked before robot use.
- The underside uses a nominal Ø8 / Ø2.7 ×0.8 mm steel wide washer, a catalog
  M2.5 washer size, with the modeled M2 clamp screw/nut. Its actual bearing, fit
  and clamp strength remain provisional; it is hardware, not a printed washer.
- The rear cover uses nominal M2 clearance holes, recessed heads and captive-nut
  pockets. These are modeled assembly dimensions, not a purchase-ready screw
  specification; actual nut tolerances and print shrinkage need checking.
- Tab5’s upper and middle rear mounting pairs are taken from its official
  dimension drawing. Camera-up portrait coordinates relative to the device
  center are Y±36 mm, Z+60/+13 mm; in this model Z122/75 mm. Four reinforced
  rear lands and Ø3.4 mm clearance bores follow those locations.
- The drawing’s `M3*10` callout must not be treated as a10 mm screw insertion
  depth. **Allowable M3 engagement is unknown** and needs measurement on the
  actual Tab5. Connectors, cables and removable battery packaging also need a
  physical fit check. The model uses the bare Tab5 and retains the robot battery.

## Simulation and verification

`geometry_report.json` integrates mass and inertia from the actual solids using
assumed plastic density1240 kg/m³ and steel density7850 kg/m³. Component weights,
computing allowance35 g and unspecified mounting-hardware allowance4 g are
simulation assumptions. They are not measured robot weights.

The independent reports audit exported STL/OBJ integrity, unchanged stock parts,
actual fixed-CAD intersections, and conservative collision coverage. Collision
hulls cover small connected cells; no single hull closes an entire window. Their
extra occupied volume can produce conservative early contacts. Explicit inertias
come from the real solids, not the collision-hull volumes.

The saved303 STAND/SIT/FOLD path samples,181 feet-flat crouch states, and final
walk/idle traces are checked for new-part interference. These are discrete
geometric checks, not a guarantee over every continuous joint configuration.
Original SIT ground penetration and FOLD same-leg convex-hull flags are preserved;
sitting/folding balance is not demonstrated.

Walking uses the same official ONNX policy with the existing head-removal
adapter. Full gravity, a free body, foot contacts and finite BAM actuation remain
active. No retraining, animation, anchors or external body force is used. Short
upright rollouts still have speed error and sideways drift; they do not establish
hardware reliability.

## Current numerical results

- Four printed parts: estimated84.518 g at the stated plastic density
- Total simulated model:701.469 g
-1,486 saved pose/walk/idle states, no new-part-to-leg contacts
- Smallest independently evaluated sampled lower bound:3.0974 mm at the clamp
  hardware, compared with about2.883 mm in the earlier open-frame model
- Body clearance≥16.4576 mm; Tab5≥16.3001 mm across the audited samples
-10-second walk:0.958 m forward,0.104 m sideways drift, maximum tilt3.35°, no fall
-10-second active-policy idle:maximum tilt1.64°, no fall

The six-second video is a faithful rendering of the first six seconds of that
completed full-physics rollout. Its metadata records the source state-file and
physical-model hashes. It does not represent a separately invented trajectory.

## Reproduction

```sh
bash bootstrap_print_concept.sh
ORT_DISABLE_TELEMETRY=1 .venv/bin/python prototype/portrait/run_local_probe.py \
  --scene prototype/portrait_print/scene_tab5_portrait_print.xml \
  --policy policies/alpha_walking.onnx --out prototype/portrait_print/my_walk \
  --speed .3 --seconds 10 --save-states
MUJOCO_GL=egl .venv/bin/python prototype/portrait_print/render_print_design.py
```

The local probe wrapper disables ONNX Runtime telemetry **before import**.
Rendering generates a clearly named render-only copy without invisible collider
meshes for speed; every simulation and clearance audit uses the full physical
model. Upstream assets and policy weights are fetched by the preserved pinned
bootstrap; they are not redistributed in the archive.

## Sources

- [Official Tab5 documentation](https://docs.m5stack.com/en/core/Tab5)
- [Official Tab5 dimension drawing](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1132/C145_Model_Size_page_01.png)
- Pinned official MicroDuck mesh/source provenance and licenses remain in
  `README.md`, `README-PORTRAIT.md`, `bootstrap.sh` and `third_party_licenses/`
- [ORT1.30 process-lifetime telemetry opt-out](https://raw.githubusercontent.com/microsoft/onnxruntime/v1.30.0/docs/Privacy.md)

Washer dimension reference: [Würth M2.5 wide stainless washer, 2.7×8×0.8 mm](https://eshop.wurth.fr/Rondelle-a-grand-diametre-exterieur-DIN-9021-inox-A4-brut-ROND-DIN9021-A4-D27/04129225.sku/fr/FR/EUR/). This identifies modeled dimensions, not a purchase or validated clamp specification.
