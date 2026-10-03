# MicroDuck / Tab5 browser physics reference

Generated directly from the frozen project. Run `generate_reference.py` with the project's existing `.venv/bin/python -B`; all writes stay in this directory. The original source files and mesh assets are not modified.

## Exact simulator contract

- MuJoCo 3.10.0 native reference; use the matching WASM version.
- ONNX Runtime 1.30.0, input `obs` float32 `[1,39]`, output `actions` float32 `[1,10]`. The trained delivery policy includes its observation normalization.
- 50 Hz policy; four 0.005 s MuJoCo steps per action. No low-pass action filter.
- Command is body-local `[vx,vy,yaw_rate]`; multiply by `min(1,control_step * .02 / .5)` when making observations. On reset the command observation is zero.
- Root is a free joint under gravity, no external stabilization.
- Default positions are float32 `[0,-.0873,-.4579,-.0049,.453,0,.0873,.4579,.0049,-.453]`. Use the explicit values in JSON, which retain float32 rounding.
- All arrays use MuJoCo native row-major ordering. Native qpos quaternion is scalar-first `[w,x,y,z]`.

## Observation

0:3 = `sensordata[gyro_sensor_adr:gyro_sensor_adr+3]`

3:6 = trunk rotation transpose times `[0,0,-1]`, i.e. `[-xmat[6],-xmat[7],-xmat[8]]`

6:16 = actuated qpos minus float32 default

16:26 = actuated qvel

26:36 = previous clipped action, all zero on reset

36:39 = ramped command

Cast the final 39-element observation to float32. Do not rotate translational velocity into it; that quantity is not part of this policy's observations.

## Action application and substep ordering

1. ONNX inference on current float32 observation
2. Clip action elementwise to [-1.2,1.2] in float32
3. Add float32 defaults in float32 (`Math.fround(default + clippedAction)`), then put those exact values into BAM's double q_target
4. Repeat four times: compute BAM update using the current MuJoCo data and previous dynamics arrays, then call `mj_step`
5. Call `mj_forward` once after all four steps
6. Store previous clipped float32 action, increment control_step, build next observation

During each BAM update:

- Effective voltage = max(6, 7.4 - 0 * sum(abs(previous_motor_torque))) = 7.4 V in nominal mode
- Voltage command = vin * clip((target-q) * kp * error_gain,-1,1)
- New ctrl = kt * voltage / R - kt^2 * dq / R
- Write raw torque to data.ctrl; do not preclip it. MuJoCo applies its actuator forcerange itself
- External torque = -qfrc_bias + qfrc_constraint - friction_constraint_force
- Friction-constraint force is the sum of `efc_force[j]` where `efc_type[j] == mjCNSTR_FRICTION_DOF` AND `efc_id[j] == joint_index`. Preserve the source's joint-index comparison exactly; do not substitute DOF indices
- Friction input motor torque is the OLD `qfrc_actuator`, not the new ctrl value. Both this and old constraint/bias fields are taken from data before the ensuing mj_step
- Use formulas below to update model.dof_frictionloss and model.dof_damping for the 10 actuated DOFs
- Record last_ts=data.time and previous_motor_torque=new raw torque (dt is unused by the XL330 proportional controller)

## BAM M6 friction formula

With motor torque M, external torque E, velocity v, and parameter names from bam_constants.json:

S = exp(-(abs(v/dtheta_stribeck)^alpha))

F = friction_base + abs(E*load_friction_external - M*load_friction_motor)

F += S * friction_stribeck

F += S * abs(E*load_friction_external_stribeck - M*load_friction_motor_stribeck)

When sign(E) != sign(M):

- If abs(E) < abs(M): F += S * load_friction_external_quad * abs(E)^2
- If abs(E) > abs(M): F += S * load_friction_motor_quad * abs(M)^2
- If equal, neither quadratic branch contributes

Damping = friction_viscous

q_offset is a fitted testbench parameter and is not used in this controller. kp=200 is firmware gain; it is not a direct Nm/rad gain. No current limiter is enabled.

## Independent zero-noise reset

1. Zero model.dof_frictionloss and model.dof_damping at the actuated DOFs (the independent evaluation script does this before reset)
2. `mj_resetDataKeyframe(model,data,STAND_key_id)`
3. Set qpos[2]=.125 and actuated qpos to the exact float32 default values
4. Set q_target to those defaults, previous_motor_torque to zeros, last_ts=0
5. `mj_forward(model,data)`
6. Set previous_action=zeros float32, control_step=0, command as requested

Do not zero data.ctrl after the STAND keyframe reset. The frozen source leaves the keyframe position-valued ctrl contents in the motor ctrl array until the first BAM update. That means its first qfrc_actuator is nonzero and contributes to first-update friction. The initial state is provided in trajectory_reference.json.

## Assets and render data

screen_after_bam.xml contains full frozen contacts and meshes, with only the exact BAM actuator conversion/armature and 0.005 timestep applied. It uses paths under `assets/`.

assets.bin and assets_index.json pack byte-identical original assets into one download. Populate the WASM virtual FS with each original byte range at its indexed path before model loading. Do not replace collision mesh assets with render meshes or combine contact pairs.

renderer_meshes.bin and renderer_metadata.json contain compiled buffers for visible groups 0,1,2 and alpha>0. Mesh offsets are byte offsets, little-endian float32 or int32; face indices are local to the mesh. The metadata names the mesh vertices, faces, normals, normal indices, texture coordinates, and texture-coordinate indices. Use data.geom_xpos and data.geom_xmat transforms. Mesh processing has already centered/oriented vertices to MuJoCo's mesh frame. Screen UVs must use facetexcoord, not assume one UV per vertex.

Screen image is assets/stackchan_microduck_screen_720x1280.png. Material metadata includes texture IDs by MuJoCo texture role and the exact role enum.

## Verification artifacts

- *_compiled_metadata.json: sizes, simulator options, SHA-256/dtype/shape of every compiled ndarray, body/joint/geom names, all explicit contacts
- *_physics_arrays.json: full-precision runtime physical arrays for audit/precision recovery
- nominal_screen_comparison.json: verifies physical arrays unaffected by screen artwork
- xml_reload_comparison.json: measures XML serialization roundtrip effects, which must be considered for strict parity
- trajectory_reference.json: initial reset and 50 trained-policy actions (1 second), every pre-step data state, voltage, torque, friction and damping update, every post-control state/observation
- nominal_screen_trajectory_parity.json: same fixed action sequence on baseline and screen models
- provenance.json: exact source/policy hashes and Python/native dependency versions

For open-loop engine parity, replay the stored float32 actions instead of re-running ONNX. Check initial state first, then the first BAM ctrl/friction update, then one mj_step, before attempting a long run. Neural inference parity can be tested separately using each stored observation/action pair.

## Exact XML fallback result

`exact_xml.py` creates `screen_after_bam_exact.xml` by retaining original source XML numeric text and applying only the BAM settings. Reload verification against the exact MJB found every physical and mesh ndarray bit-identical. Only `_sizes`, `mesh_pathadr`, and `tex_pathadr` differ because the packed paths are shorter. Use this exact XML, not the rounded MjSpec serialization, if MJB cannot load. `screen_after_bam_mj_saveLastXML.xml` is an additional audit output from the explicitly requested API.

NumPy 2 float32 command multiplication also rounds the scalar ramp to float32 before multiplication. Match it with `Math.fround(Math.fround(command[i]) * Math.fround(ramp))`.

Independent JS friction review used 135 seeded fixtures in `bam_unit_fixtures.json` and found a maximum discrepancy of 5.55e-17 Nm against the native BAM M6 formulas. `bam_js_review.json` records this check.
