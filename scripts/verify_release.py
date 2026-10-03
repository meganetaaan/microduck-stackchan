"""Dependency-free checks for the public release; does not command hardware."""
from pathlib import Path
import ast, hashlib, json, xml.etree.ElementTree as ET
ROOT = Path(__file__).resolve().parents[1]
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()
for name in ['LICENSE.md', 'MODEL-LICENSE.md', 'NOTICE.md', 'LICENSES/Apache-2.0.txt']:
    assert (ROOT / name).is_file(), name
syntax_count = 0
for path in ROOT.rglob('*.py'):
    if any(p in {'node_modules', '.venv', 'microduck_rl', 'bam'} for p in path.relative_to(ROOT).parts):
        continue
    ast.parse(path.read_text(), filename=str(path.relative_to(ROOT)))
    syntax_count += 1
manifest = json.loads((ROOT / 'prototype/assembly_uart/frozen_model_manifest.json').read_text())
verified = 0
upstream_pending = 0
for name, expected in manifest['files'].items():
    path = ROOT / name
    if not path.exists():
        assert name.startswith('microduck_rl/'), name
        upstream_pending += 1
        continue
    assert sha(path) == expected, f'Frozen asset changed: {name}'
    verified += 1
policy = ROOT / 'training/tab5_gaits/results/policy.onnx'
assert sha(policy) == 'f533adcfd76fa7a3c0251d001e192e2c1bde9b58aa09f4d16e70e03065c63585'
selected = ROOT / 'design/arms/MicroDuck_Stackchan_Arm_Concepts_v1'
for name in ['glb/04_flap_display.glb', 'glb/04_flap_stowed.glb', 'scene_04_flap_display.xml', 'scene_04_flap_stowed.xml']:
    assert (selected/name).is_file(), name
xml_count = 0
for path in ROOT.rglob('*.xml'):
    if any(p in {'node_modules', '.venv', 'microduck_rl', 'bam'} for p in path.relative_to(ROOT).parts):
        continue
    ET.parse(path)
    xml_count += 1
report = {'python_syntax_files': syntax_count, 'xml_well_formed_files': xml_count,
          'frozen_local_files_verified': verified,
          'pinned_upstream_files_available_after_bootstrap': upstream_pending,
          'policy_sha256': sha(policy),
          'selected_arm': 'original option 4 / comparison A, visual concept only',
          'physics_policy_scope': 'no-arms 736.537 g baseline'}
print(json.dumps(report, indent=2))
