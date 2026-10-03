# GitHub publication notes

## Scope

This public repository integrates the existing assembly, trained gait package, web simulator and arm design studies. The selected arm is **original option 4 / comparison A (thin flap)**. Existing numerical results remain the **no-arms 736.537 g baseline**. No new training, paid compute, real-hardware run, arm-inertia model or arm-gait certification is included in this publication.

## Preserved assets

- Assembly physical MJCF and its 1,632 locally distributed frozen dependencies retain their original SHA-256 values
- 22 upstream dependencies are fetched from the pinned `microduck_rl` commit by the bootstrap script
- Final policy SHA-256 remains `f533adcfd76fa7a3c0251d001e192e2c1bde9b58aa09f4d16e70e03065c63585`
- Prior variants and distinct GLB design outputs are retained; 59 historical renders/videos omitted from the compact assembly download are restored under their original paths
- Scientific training configuration/source snapshots, evaluation tables, traces and checkpoints remain included

## Publication-only changes

- Added Japanese root quickstart, selected-arm index, scoped model/policy cards and full third-party license notices
- Replaced origin-machine path prefixes in archived JSON with repository-relative paths; adapted validators to those relative paths
- Removed private reference identifiers from screen/document metadata and from code that emitted them
- Adjusted the fallback arm-generator source path to the integrated repository layout
- The frozen physical assets and learned weights were not regenerated or retrained
- Historical source-hash and archive manifests describe their original delivery revision; public documentation/path cleanup can change nonphysical source/metadata hashes
- Excluded local dependencies, private hosting configuration, credentials and previous Git history

## Checks

Run `python3 scripts/verify_release.py` for Python syntax, well-formed MJCF, preserved baseline asset hashes, final ONNX identity, required notices and selected design assets. This check does not run native dynamics and explicitly reports upstream assets that need bootstrap.

The Web application has its own unit, worker, project-subpath HTTP, numerical parity, long closed-loop and transition tests. See `web/README.md` and `web/test-results/`. Numeric Node/WASM tests must not be confused with visual browser or mobile-device tests. GitHub CI builds the static site and performs its configured checks before deployment.

## Model license ambiguity

The pinned upstream README declares the model license as “Creative Commons BY-SA-NC” without a version number. The repository preserves that exact declaration and noncommercial/share-alike scope; it does not invent a version or claim broader commercial rights. See `MODEL-LICENSE.md` and `NOTICE.md`.

## Static asset packaging

The browser compiled-model gzip is distributed as four chunks under 7 MB each to fit the publication transport limit. Concatenated gzip SHA-256 remains `2fd094db6bafd69aa32c46b87656756d0e9ba9a38b731f47a0a15c4eeee7f5b9`. The decompressed model and numerical physics/policy are unchanged. Unit, input, Worker, parity and project-subpath tests were rerun after this packaging-only change.
