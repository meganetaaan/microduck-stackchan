#!/usr/bin/env bash
# CPU-only reproduction. Does not connect to, flash, or command a real robot.
set -euo pipefail
cd "$(dirname "$0")"
clone_pin() {
  local url="$1" dir="$2" sha="$3"
  if [[ ! -d "$dir/.git" ]]; then git clone --depth 1 "$url" "$dir"; fi
  if [[ -n "$(git -C "$dir" status --porcelain)" ]]; then
    echo "Refusing to overwrite changes in $dir" >&2; exit 1
  fi
  git -C "$dir" fetch --depth 1 origin "$sha"
  git -C "$dir" checkout --detach "$sha"
}
clone_pin https://github.com/pollen-robotics/microduck_rl.git microduck_rl 8d0db74916a4f833d1d9b95d6a1d7f4d13b9d5ec
clone_pin https://github.com/Rhoban/bam.git bam 62bd8ce12154340be97e06f7f41a0ca8f116d967
python3.12 -m venv .venv
.venv/bin/python -m pip install -r cpu-requirements.txt
.venv/bin/python -m pip install -e ./bam
mkdir -p policies
curl -fL --retry 2 https://huggingface.co/pollen-robotics/microduck-policies/resolve/v5/alpha_walking.onnx -o policies/alpha_walking.onnx
echo 'e36332d383997d51401897734cd3e79cf5038406feddb18b4d57ecfb141daa6c  policies/alpha_walking.onnx' | sha256sum -c -
.venv/bin/python prototype/build_models.py
.venv/bin/python prototype/build_lowcube.py
.venv/bin/python prototype/diagnostics/build_cutout_candidate.py
.venv/bin/python prototype/build_rounded_shell.py
.venv/bin/python prototype/test_models.py
echo 'Ready. See README.md for the headless rollout command.'
