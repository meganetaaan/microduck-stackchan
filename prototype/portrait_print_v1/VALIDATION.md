# Independent printable portrait validation

The final model was checked at 1,486 saved states: 303 posture-interpolation states, 181 feet-flat crouch states, 501 walking states, and 501 idle states. No contacts were found between added physical parts and the 34 original moving-leg meshes.

| Part group | Minimum numerical lower bound | Corresponding upper bound |
| --- | ---: | ---: |
| Mounting, including metal hardware | 3.097806 mm | 3.097806 mm |
| Body, cover, pedestal and compute | 16.457641 mm | 16.457641 mm |
| Tab5 | 16.300103 mm | 16.300104 mm |

These are independent numerical bounds for the sampled states. Moving geometry uses conservative convex hulls, AABB bounds, support-plane bounds, and feasible closest points. This is not a hardware certification or continuous-motion guarantee.

The original mechanism, joints, actuators and Tab5 match the prior portrait model exactly. Every new physical collider has an explicit collision pair with every original moving-leg guard. Triangle-mesh Manifold boolean tests found no new-to-original-fixed or new-to-new solid intersections above 0.001 mm³. Nominal bearing contact exists between the stock plate, upper printed mount and steel washer.

All four final printed parts are single-component, watertight, consistently wound solids. Their original and print-oriented STL inventories agree. Oriented files sit on Z=0 and match the original parts after rigid transforms, within 0.00000382 mm of vertex rounding. STL coordinates are millimetres; STL itself carries no unit metadata. Estimated printed mass is 82.742 g at the stated 1,240 kg/m³ density; total modeled mass is 699.694 g. Exported-solid mass, center of mass and full inertia agree with the compiled model to export precision.

Collider coverage was independently checked by rereading and combining exported convex pieces. After a 0.00001 mm analytical coordinate tolerance, the largest remaining numerical uncovered volume is 0.000001346 mm³. This tolerance does not change the actual collision geometry.

353 optimizer status flags are retained in the detailed report. Separation conclusions use independent positive support/AABB lower bounds, including for flagged solves. The largest support-to-feasible-distance interval for an examined pair was 0.000105 mm.

The original SIT pose still penetrates the floor by about 4.441 mm, and FOLD retains eight original nonadjacent convex-overlap flags. They were reproduced unchanged. Print fit, supports, strength, clamp preload, frictional anti-rotation, cable routing and screw engagement remain unvalidated. A separate 4 g allowance approximates unspecified mounting fastener mass and inertia.

Detailed evidence is in `validation_report.json`, `validation_summary.json`, `geometry_validation_report.json`, `mount_interface_validation.json`, and `oriented_stl_validation_report.json`. Recorded model, trace and oriented-STL hashes match the final files.
