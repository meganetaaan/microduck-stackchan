"""Compact working release: preserve prior model/source files, move older media to durable history."""
from pathlib import Path
import argparse,hashlib,json,zipfile,xml.etree.ElementTree as ET,posixpath
H=Path(__file__).resolve().parent;ROOT=H.parents[1]
def sha(b):return hashlib.sha256(b).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();prefix=ROOT.name+'/';media_suffixes={'.png','.jpg','.jpeg','.mp4'}
 with zipfile.ZipFile(a.baseline)as z:
  assert z.testzip()is None;old={n:z.read(n)for n in z.namelist()if not n.endswith('/')}
 omitted={n:{'sha256':sha(b),'bytes':len(b)}for n,b in old.items()if Path(n).suffix.lower()in media_suffixes};out={n:b for n,b in old.items()if n not in omitted}
 rootfiles=['README-ASSEMBLY-UART-ja.md','README-PRINT-CONCEPT.md','bootstrap_assembly_uart.sh','docs/BOM-ja.md','docs/DESIGN-ASSUMPTIONS-ja.md','docs/history/v3-wifi-plan/BOM-ja.md','docs/history/v3-wifi-plan/DESIGN-ASSUMPTIONS-ja.md','docs/history/v3-wifi-plan/README-PRINT-CONCEPT.md']
 local=['parameters.json','build_assembly.py','make_mass_scenarios.py','render_assembly.py','export_upper_assembly.py','validate_assembly.py','package_assembly.py','assembly_report.json','frozen_model_manifest.json','mass_scenarios.json','mass_COM_comparison.json','source_reproduction_check.json','validation_report.json','geometry_validation_report.json','collision_coverage_report.json','print_orientation.json','upper_assembly.glb','upper_assembly_export.json','export_validation_report.json','assembly_views.png','assembly_service_view.png']
 for suffix in ['', '_light','_heavy']:local +=[f'tab5_assembly_uart{suffix}.xml',f'scene_tab5_assembly_uart{suffix}.xml']
 files=[ROOT/n for n in rootfiles]+[H/n for n in local];report=json.loads((H/'assembly_report.json').read_text());files +=[H/p['visual_file']for p in report['parts']]+[H/p['file']for p in report['collision_parts']]
 files+=sorted((H/'stl').glob('*.stl'))+sorted((H/'stl_print_oriented').glob('*.stl'))+sorted((H/'sources').glob('*.json'))
 for f in files:assert f.is_file(),f;out[prefix+str(f.relative_to(ROOT))]=f.read_bytes()
 changed={n:{'old_sha256':sha(b),'new_sha256':sha(out[n])}for n,b in old.items()if n in out and out[n]!=b}
 assert set(changed)=={prefix+n for n in ['README-PRINT-CONCEPT.md','docs/BOM-ja.md','docs/DESIGN-ASSUMPTIONS-ja.md']},changed.keys()
 for n in changed:
  archive=prefix+'docs/history/v3-wifi-plan/'+Path(n).name
  assert out[archive]==old[n]
 manifest={'baseline_zip_sha256':sha(a.baseline.read_bytes()),'history_reference':'Full prior portrait archive version3 retains all omitted media; no durable history deleted.','omitted_historical_media':omitted,'changed_documentation':changed,'preserved_prior_nonmedia_file_count':sum(n not in omitted and n not in changed for n in old),'nominal_model_sha256':sha((H/'tab5_assembly_uart.xml').read_bytes()),'file_hashes':{n:sha(b)for n,b in out.items()}}
 mf=H/'working_release_manifest.json';mf.write_text(json.dumps(manifest,indent=2)+'\n');out[prefix+str(mf.relative_to(ROOT))]=mf.read_bytes()
 text='''# Working release history\n\nThis archive contains the current UART assembly and earlier model/source/assets.\nTo keep the working download compact, duplicate images and videos from earlier\nversions are omitted. The complete previous portrait archive, version3 in Library,\nretains those media. No previous Library version was deleted. The release manifest\nlists every omitted file and hash. Historical rendering scripts can regenerate\nviews after the pinned official assets are obtained.\n\nGait training checkpoints, evaluations and videos are a separate deliverable and\nare not implied to have passed by the assembly geometry checks. Use their recorded\nmodel hashes with this release.\n'''
 out[prefix+'HISTORY-MEDIA.md']=text.encode()
 # Resolve every custom mesh in current models; official robot meshes are bootstrapped.
 names=set(out)
 for suffix in ['', '_light','_heavy']:
  n=prefix+f'prototype/assembly_uart/tab5_assembly_uart{suffix}.xml';root=ET.fromstring(out[n])
  for asset in root.find('asset').findall('mesh'):
   ref=posixpath.normpath(posixpath.join(posixpath.dirname(n),asset.get('file')))
   assert '/microduck_rl/'in ref or ref in names,(n,ref)
 with zipfile.ZipFile(a.output,'w',zipfile.ZIP_DEFLATED,compresslevel=8)as z:
  for n,b in out.items():z.writestr(n,b)
 with zipfile.ZipFile(a.output)as z:
  assert z.testzip()is None and len(z.namelist())==len(set(z.namelist()))
  for n,b in old.items():
   if n not in omitted and n not in changed:assert z.read(n)==b
 assert a.output.stat().st_size<20_000_000
 print(json.dumps({'path':str(a.output),'bytes':a.output.stat().st_size,'sha256':sha(a.output.read_bytes()),'files':len(out),'preserved_prior_nonmedia_files':manifest['preserved_prior_nonmedia_file_count'],'omitted_old_media':len(omitted),'changed_documents_preserved_in_history':len(changed)},indent=2))
if __name__=='__main__':main()
