#!/usr/bin/env bash
# CPU-only geometry reproduction; no robot connection, firmware or deployment.
set -euo pipefail
cd "$(dirname "$0")"
export ORT_DISABLE_TELEMETRY=1
bash bootstrap_print_concept.sh
.venv/bin/python prototype/assembly_uart/build_assembly.py
.venv/bin/python prototype/assembly_uart/make_mass_scenarios.py
echo 'UART assembly built. See README-ASSEMBLY-UART-ja.md and docs/BOM-ja.md.'
