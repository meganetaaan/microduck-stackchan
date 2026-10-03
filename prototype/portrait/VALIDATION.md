# Portrait open-frame validation

Model SHA-256: `8bc79dfd9e5aefa14b8dd4d3549625616f3cfc1f5e0f0ca71d46fdf1aec4b163`

Independent geometry audit of all 18 new physical boxes against all 34 original moving leg CAD hulls. All 612 explicit collision pairs are present.

1486 saved states checked; zero new-part/leg contacts. All sampled states have positive independent separation lower bounds.

| Saved states | Count | All new parts ≥ mm | Frame only ≥ mm | Tab5 ≥ mm |
|---|---:|---:|---:|---:|
| STAND_to_SIT | 101 | 2.883395 | 45.940749 | 16.300104 |
| STAND_to_FOLD | 101 | 2.883395 | 12.040949 | 16.300104 |
| SIT_to_FOLD | 101 | 2.883395 | 12.040949 | 16.300103 |
| feet_flat_crouch_181 | 181 | 2.883395 | 31.834912 | 16.300103 |
| walk_10s | 501 | 2.883395 | 45.592330 | 16.376022 |
| idle_10s | 501 | 2.882975 | 45.906934 | 16.378281 |

The smallest geometric gap is at a mount post, not the open-frame rails. Conservative rollout bounds may be lower than exact sampled closest-point minima. See each JSON row’s distance_method.

## Geometry and physics
- Frame measures 80 mm wide × 80 mm deep × 83 mm high
- Tab5 physical bounds are 80 mm wide × 128 mm high × 12 mm deep; mounted upper-assembly depth is 92 mm
- Total modeled mass: 657.8729 g; all added masses and uniform-box inertias match parameters
- Original legs, joints, actuator parameters, pelvis, battery and fixed servo CAD are unchanged from the saved rounded variant
- No meaningful intersection with retained fixed CAD; two posts have intended surface contact with the trunk plate
- All added parts form one face-contact-connected assembly. Rounded Tab5 body contacts the front rails; no hidden full-frame collider closes its openings
- Float32 compiled Tab5 vertices exceed the ideal collision bounds by a few millionths of a millimetre; the recorded 0.00001 mm analytical coverage tolerance covers them

## Limits
- This is a sampled numerical audit, not continuous collision or full joint-range certification
- Saved SIT retains about 4.44094 mm original foot/ground penetration; FOLD retains eight original nonadjacent leg convex-hull overlap flags. These predate this design
- Some far-pair SLSQP status warnings remain in the JSON; independently evaluated positive support bounds, not optimizer status, establish the reported sampled separations
- Mounting interfaces are conceptual. Fasteners, holes, wiring, manufacturing tolerances, stiffness, structural strength and hardware safety remain unvalidated
- No training, inference, hardware action or prior-model editing was performed by this audit
