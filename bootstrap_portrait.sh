#!/usr/bin/env bash
# Independent portrait alternative; CPU-only, no robot connection.
set -euo pipefail
cd "$(dirname "$0")"
# Must precede any import or launch of ONNX Runtime1.30.
export ORT_DISABLE_TELEMETRY=1
bash bootstrap.sh
.venv/bin/python prototype/portrait/build_portrait.py
echo 'Portrait model ready. See README-PORTRAIT.md; use run_local_probe.py for all rollouts.'
