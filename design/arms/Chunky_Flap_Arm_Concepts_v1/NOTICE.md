# Arm-concept attribution

This is an independent experimental Tab5/Stack-chan arm-concept package using MicroDuck-derived base geometry. It is not an official Pollen Robotics product or a mechanically certified design.

## Base model and geometry

Source author: Pollen Robotics

https://github.com/pollen-robotics/microduck_rl/tree/8d0db74916a4f833d1d9b95d6a1d7f4d13b9d5ec

The upstream README states: “3D model files are licensed under Creative Commons BY-SA-NC.” It specifies no version. The same attribution, noncommercial and share-alike conditions are retained for the inherited robot meshes and the derivative assembled models, including the geometry embedded in GLB, MJCF/XML, STL/OBJ and rendered illustrations. See the adjacent `MODEL-LICENSE.md` for the exact upstream declaration and its unresolved version detail. Do not replace it with an assumed Apache or CC 4.0 license.

Project changes are the custom Tab5 enclosure/base adaptation and the alternative arm/flap/armor assemblies, source generators, poses, materials, mesh conversions and comparison renderings described in this package's README files. Changes do not imply the original author endorses them. Preserve the attribution and change descriptions when sharing.

## Software

The project's Python model-generation and rendering programs are distributed under Apache-2.0; see `third_party_licenses/Apache-2.0.txt`. Their software license does not override the separate license of inherited geometry or geometry incorporated into generated output.

Referenced upstream software:

- MicroDuck RL: Copyright 2026 Pollen Robotics. Apache-2.0. Full text: `third_party_licenses/microduck_rl_APACHE_2.txt`
- BAM: Copyright 2025 Marc Duclusaud & Grégoire Passault. Apache-2.0. https://github.com/Rhoban/bam/tree/62bd8ce12154340be97e06f7f41a0ca8f116d967 . Full text: `third_party_licenses/bam_APACHE_2.txt`

Runtime tools installed separately retain their own licenses. This arm-concept package does not contain the trained Tab5 gait or browser runtime; those are separate parts of the complete repository and have their own notices.

