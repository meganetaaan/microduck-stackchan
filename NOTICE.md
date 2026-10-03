# Attribution and provenance

This is an independent MicroDuck-derived Tab5/Stack-chan simulation and mechanical-concept project. It is not endorsed by the upstream organizations. Simulation results are not evidence of real-hardware safety or manufacturing readiness.

## MicroDuck software

Copyright 2026 Pollen Robotics. Apache License 2.0.

- Training/model source: https://github.com/pollen-robotics/microduck_rl/tree/8d0db74916a4f833d1d9b95d6a1d7f4d13b9d5ec
- Runtime/documentation reference: https://github.com/pollen-robotics/microduck/tree/9060e81a6e598504003ff31f9f39e075e2c5290f
- Full software license: `LICENSES/MicroDuckRL-Apache-2.0.txt`

The separate MicroDuck 3D model declaration is preserved in `MODEL-LICENSE.md`. It is not Apache-2.0 and its version is not specified by the reviewed upstream declaration.

## Pretrained and fine-tuned policy weights

Upstream author: Pollen Robotics. Model repository declares Apache-2.0.

- Parent: https://huggingface.co/pollen-robotics/microduck-policies/tree/v5
- Parent file: `alpha_walking.onnx`
- Parent SHA-256: `e36332d383997d51401897734cd3e79cf5038406feddb18b4d57ecfb141daa6c`
- Preserved parent model card: `training/tab5_gaits/teacher_model_card.md`
- Released trained policy SHA-256: `f533adcfd76fa7a3c0251d001e192e2c1bde9b58aa09f4d16e70e03065c63585`

The project's policy is a modified derivative of the parent, not a from-scratch policy. The parent network weights and observation normalizer were adapted from 61 observations / 14 actions to 39 observations / 10 leg actions, followed by CPU PPO fine-tuning and MuJoCo command calibration. The training archive records 768,000 PPO control steps and 105,600 calibration control steps. Observation normalization and calibration are baked into the exported ONNX graph. Retain Apache-2.0 and this attribution with the ONNX/PT weights and checkpoints.

## BAM actuator model

Copyright 2025 Marc Duclusaud & Grégoire Passault. Apache License 2.0.

https://github.com/Rhoban/bam/tree/62bd8ce12154340be97e06f7f41a0ca8f116d967

`physics.js` adapts BAM's actuator, model, and MuJoCo update logic to scalar JavaScript, including the XL330 M6 friction/voltage model. It preserves the original attribution and identifies the adaptation. Full text: `LICENSES/BAM-Apache-2.0.txt`.

## Browser runtimes and icons

- MuJoCo 3.10.0: Copyright DeepMind Technologies Limited, Apache-2.0. https://github.com/google-deepmind/mujoco/tree/3.10.0
- ONNX Runtime Web 1.30.0: Copyright Microsoft Corporation, MIT. https://github.com/microsoft/onnxruntime/tree/v1.30.0
- Three.js 0.180.0, including OrbitControls: Copyright 2010–2025 three.js authors, MIT. https://github.com/mrdoob/three.js/tree/r180
- Lucide 0.468.0: ISC, with the preserved Feather/Cole Bemis attribution in its upstream license. https://github.com/lucide-icons/lucide/tree/0.468.0

Bundled binary runtimes also incorporate their own dependencies. Preserve `ONNXRuntime-ThirdPartyNotices.txt`, the MuJoCo dependency notice texts, and `MuJoCo-ThirdParty-README.txt` alongside any redistributable browser bundle. Merely including Apache/MIT top-level texts is not a replacement for these notices.

## Reference architecture and artwork

The official MicroDuck Sandbox was consulted as an architecture reference:
https://huggingface.co/spaces/pollen-robotics/microduck-simulator

The project's browser application is a separate implementation. No sandbox application source, audio, props, decorative images, or other sandbox assets are included. The simulator repository's accessibility is not treated as a license grant for its application code.

The portrait screen image is newly drawn vector artwork based on the supplied simple Stack-chan face reference, with original decorative panels. The reference photograph is not included. This does not grant rights in anyone else's photograph, branding, or character designs. The project does not bundle font files.

