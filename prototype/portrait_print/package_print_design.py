"""Preserve prior models; move the delivered v1 print design to a sibling archive."""
from pathlib import Path
import argparse,hashlib,json,zipfile
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
def sha(b):return hashlib.sha256(b).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();prefix=ROOT.name+'/';rows=[]
 top=['parameters.json','build_print_design.py','render_print_design.py','render_flush_interface.py','render_recorded_walk.py','validate_print_design.py','validate_mount_interface.py','validate_oriented_stls.py','validate_flush_interface.py','test_print_design.py','package_print_design.py','summarize_validation.py','scene_tab5_portrait_print.xml','tab5_portrait_print.xml','geometry_report.json','geometry_validation_report.json','validation_report.json','validation_summary.json','oriented_stl_validation_report.json','mount_interface_validation.json','flush_interface_validation.json','print_orientation.json','print_views.png','print_comparison.png','flush_interface_comparison.png','exploded_assembly.png','print_parts_orientation.png','style_reference.png','front.png','side.png','three-quarter.png','result_summary.json','VALIDATION.md','walking_frame3s.png']
 for stem in ['walk_10s','idle_10s','walking_demo']:top +=[stem+ext for ext in ['.json','.csv','_states.npz']]
 top+=['walking_demo.mp4']
 assets=json.loads((HERE/'geometry_report.json').read_text());files=[ROOT/'README-PRINT-CONCEPT.md',ROOT/'bootstrap_print_concept.sh']+[HERE/n for n in top if (HERE/n).exists()]
 files +=[HERE/'assets'/(p['name']+'.obj')for p in assets['parts']]+[HERE/p['file']for p in assets['collision_parts']]
 files +=sorted((HERE/'stl').glob('*.stl'))+sorted((HERE/'stl_print_oriented').glob('*.stl'))
 additions={prefix+str(f.relative_to(ROOT)):f for f in files}
 with zipfile.ZipFile(a.baseline)as old:
  archived={}
  for info in old.infolist():
   if info.is_dir():continue
   source=info.filename;dest=source
   if source.startswith(prefix+'prototype/portrait_print/'):dest=source.replace('/portrait_print/','/portrait_print_v1/',1)
   elif source==prefix+'README-PRINT-CONCEPT.md':dest=prefix+'prototype/portrait_print_v1/README-PRINT-CONCEPT.md'
   b=old.read(info)
   if dest in additions:
    assert additions[dest].read_bytes()==b,'Unexpected mutation of prior common file: '+dest
    continue
   archived[dest]=b;rows.append({'original_path':source,'archive_path':dest,'sha256':sha(b)})
  manifest={'baseline_zip_sha256':sha(a.baseline.read_bytes()),'preserved_previous_file_count':len(rows),'prior_print_variant':'prototype/portrait_print_v1/; original bytes preserved at sibling paths so relative mesh references stay valid','prior_files':rows,'current_files':{str(f.relative_to(ROOT)):sha(f.read_bytes())for f in files}}
  mp=HERE/'preservation_manifest.json';mp.write_text(json.dumps(manifest,indent=2));additions[prefix+str(mp.relative_to(ROOT))]=mp
  with zipfile.ZipFile(a.output,'w',zipfile.ZIP_DEFLATED,compresslevel=8)as z:
   for dest,b in archived.items():z.writestr(dest,b)
   for dest,f in additions.items():z.write(f,dest)
 with zipfile.ZipFile(a.output)as z:
  assert z.testzip()is None
  for row in rows:assert sha(z.read(row['archive_path']))==row['sha256']
  assert len(z.namelist())==len(set(z.namelist()))
  assert a.output.stat().st_size<20_000_000
  print(json.dumps({'file':str(a.output),'bytes':a.output.stat().st_size,'sha256':sha(a.output.read_bytes()),'file_count':len(z.namelist()),'preserved_previous_file_count':len(rows)},indent=2))
if __name__=='__main__':main()
