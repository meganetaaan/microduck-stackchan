# Licenses

This repository is a mixed-license collection. No single software license covers all robot geometry, trained weights, artwork, and bundled runtimes.

## Project software and policy weights

The project's original software contributions and its modified MicroDuck policy weights are distributed under Apache License 2.0. See `LICENSES/Apache-2.0.txt` and `NOTICE.md`. Existing third-party copyright notices and licenses remain applicable.

This scope includes the Python training/evaluation and model-generation programs, JavaScript application and simulation-control programs, and the project's fine-tuned `.onnx` and `.pt` policy artifacts. It does not relicense the geometric content those programs read or generate. A generator's software license is distinct from the license of geometry copied into its output.

## Robot models and geometry-bearing material

Pollen Robotics' MicroDuck 3D model files have a separate Creative Commons noncommercial, share-alike declaration. The upstream declaration does not identify a license version. See `MODEL-LICENSE.md`; do not interpret an Apache license file as permission to use these models commercially.

The same upstream model conditions are retained for MicroDuck-derived meshes, mechanical designs, assembled models, and their converted or compiled representations, including geometry embedded in GLB, MuJoCo MJB, compressed model bundles, and model renderings. Our modifications to that material are shared subject to those same conditions. Original mechanical additions delivered together with these derivatives are offered on that same noncommercial, share-alike basis, without granting broader rights in third-party material.

## Third-party components

Bundled MuJoCo, ONNX Runtime, Three.js, Lucide, and their included dependencies keep their own licenses. Their notices and full texts are in `LICENSES/` and the browser app's `licenses/` directory. Third-party components are not relicensed by this file.

Product and organization names are used to identify provenance or compatibility. This project is not an official Pollen Robotics, Hugging Face, M5Stack, Radxa, or ROBOTIS release. No trademark, endorsement, hardware-safety certification, or rights in unpublished manufacturer CAD are granted.

