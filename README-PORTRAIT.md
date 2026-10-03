# Separate alternative: portrait Tab5, open square-footprint frame

This is a saved-as alternative. The previously delivered rounded design and all
824 files from its saved archive remain byte-for-byte unchanged. Start here for
the portrait model; `README.md` continues to describe the earlier designs.

## Arrangement

- Old cosmetic cover and skirts are absent. An open 12-bar cuboid frame is
  **80 mm wide ×80 mm deep ×83 mm high**. It occupies trunk-frame
  X−39…41, Y−40…40, Z43…126 mm. The frame's empty sides remain empty in collision
  geometry; no invisible solid box fills them.
- Tab5 is **80 mm wide ×128 mm high ×12 mm deep**, in portrait, facing +X.
  Its center is (47,0,62) mm. The upper assembly including Tab5 has an
  **80×92 mm footprint** and spans Z−2…126 mm. Thus the equal-width/depth rule
  applies to the frame, not to the extra front tablet thickness.
- The physical camera short edge is at the top: 90° counterclockwise from the
  official landscape front drawing, viewed from the front. The lens aperture is
  Ø5 mm and its center is 4.45 mm below the portrait top edge.
- Face graphics occupy only the upper half of the portrait display. Display
  width/height and bezel insets are drawing-derived estimates, not exact display
  manufacturing dimensions. Device size, lens dimensions and orientation follow
  the official drawing. The visible tablet corners are genuinely R4 geometry.
- Two narrow center posts attach the frame tray above the unchanged trunk plate.
  Raising the frame base above the retained shells removes the initial packaging
  interferences. Screw holes, fastening details, wiring and structural design
  are not yet developed.

## Mass and physics assumptions

Expected total model mass: **657.8729 g**, including:

- unchanged original modeled pelvis/legs/base battery:457.31578 g
- bare Tab5:118.4 g; no extra Tab5 battery
- open frame:18.01472 g and tray:7.1424 g, estimated uniform PLA1240 kg/m³
- two mounting posts/fastener allowance:20 g total
- relocated computing/cabling allowance:35 g; compute support:2 g

Box inertias use the stated uniform mass distributions. Tab5 uses a uniform
12×80×128 mm cuboid approximation; its real component COM/inertia is unknown.
The opaque green block represents an allowance, not a selected or electrically
validated computing module. This is simulation concept geometry, not print-ready
or strength-certified hardware. The retained physical hardware revision may
differ from the upstream model.

All ten leg joints, link shapes, inertias, actuator parameters, feet and original
contact behavior are preserved. Added physical pieces have explicit contact
pairs against every moving leg CAD convex hull, including the hip shells normally
missed by parent-body filtering. The Tab5 rounded visible body uses a conservative
full-size box collider; lens/screen/eye graphics are zero-mass display details.

## Validation and limits

See `prototype/portrait/validation_report.json` for final geometry checks and
`walk_10s.json`, `idle_10s.json`, `walk_seed1.json`, `walking_demo.json` for actual
physics runs. The same official ONNX policy and earlier zero-padded head-removal
adapter are used; no training or posture animation was introduced.

The free-floating model uses full gravity, foot friction/contact and finite BAM
voltage/torque limits. The gait command is0.3 m/s, not the achieved speed. Nominal
10-second walking advanced0.891 m while drifting0.260 m sideways, with maximum
tilt3.69°. Active zero-command idle stayed upright10 seconds, maximum tilt1.68°.
One noisy initial-state seed also stayed upright10 seconds. None of these runs
certifies hardware, long-horizon reliability, speed tracking or direction control.

The 303 original STAND/SIT/FOLD path samples and181 feet-flat crouch samples are
kinematic clearance tests. They do not prove every joint combination or the
continuous paths collision-free. Original FOLD same-leg convex-proxy overlaps and
floating feet remain; original SIT is not a validated flat-foot seated posture.
Crouch balance, sitting control, cables, tolerances and deformation are untested.

Across those484 sampled postures, independent conservative lower bounds are
12.0409 mm for the open frame and16.3001 mm for Tab5. **The limiting added part is
a central mounting-post corner at2.8834 mm from a moving hip part.** Do not treat
the larger frame clearance as a margin for the entire assembly. Real fasteners,
wire routing, tolerances and flex need a separate bracket review.

## Reproduce, CPU only

Run from the extracted `microduck-stackchan` directory, with Python3.12 and git:

```sh
bash bootstrap_portrait.sh
ORT_DISABLE_TELEMETRY=1 .venv/bin/python prototype/portrait/run_local_probe.py \
  --scene prototype/portrait/scene_tab5_portrait.xml \
  --policy policies/alpha_walking.onnx --out prototype/portrait/walk_reproduced \
  --speed .3 --seconds 10 --save-states
MUJOCO_GL=egl .venv/bin/python prototype/portrait/render_portrait.py
.venv/bin/python prototype/portrait/validate_portrait.py
```

The portrait wrapper sets `ORT_DISABLE_TELEMETRY=1` before importing ONNX Runtime.
ORT1.30 official builds can initialize telemetry on import; the later Python
`disable_telemetry_events()` API alone is insufficient. Keep the process-lifetime
opt-out for all local tests. This uses the vendor-supported setting and does not
change network permissions. [ORT1.30 privacy documentation](https://raw.githubusercontent.com/microsoft/onnxruntime/v1.30.0/docs/Privacy.md)

Bootstrap fetches pinned official upstream assets and the checksum-verified
policy. They are not redistributed in this ZIP. The earlier bootstrap may rebuild
its deterministic generated model files locally; the saved archive copies are
retained for comparison. No commands connect to a physical robot or train on GPU.

## Sources

- [Official Tab5 specifications](https://docs.m5stack.com/en/core/Tab5)
- [Official dimension drawing](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1132/C145_Model_Size_page_01.png)
- [Official front product image](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1132/K145_01.webp)
- Original source pins, policy checksums and license notices remain in the
  preserved `README.md`, `bootstrap.sh` and `third_party_licenses/`
