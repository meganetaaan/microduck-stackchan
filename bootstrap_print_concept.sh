#!/usr/bin/env bash
# CPU-only selected portrait print concept; no physical robot connection.
set -euo pipefail
cd "$(dirname "$0")"
export ORT_DISABLE_TELEMETRY=1
bash bootstrap_portrait.sh
.venv/bin/python prototype/portrait_print/build_print_design.py
echo 'Built printable concept. See README-PRINT-CONCEPT.md before slicing or assembly.'
