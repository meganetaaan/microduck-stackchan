# Learned policy: no-arms Tab5 baseline

This policy is a modified Apache-2.0 derivative of Pollen Robotics' `alpha_walking.onnx` v5. Teacher provenance, hash, tensor adaptation, PPO training, command calibration and license attribution are recorded in [the project NOTICE](../../NOTICE.md), [teacher card](teacher_model_card.md) and [training README](README.md).

The released policy has 39 observations and 10 leg actions. It was trained and evaluated on the **no-arms 736.537 g assembly**, with the documented light/heavy and actuator/ground variations. It has not been trained or evaluated with the selected thin flap arms, and the visual-only arm scenes are not dynamics/training models.

Final ONNX SHA-256: `f533adcfd76fa7a3c0251d001e192e2c1bde9b58aa09f4d16e70e03065c63585`.

The `.pt` checkpoints are PyTorch serialization files intended only for this trusted project. Use ONNX for inference when possible; do not load arbitrary untrusted checkpoint files.
