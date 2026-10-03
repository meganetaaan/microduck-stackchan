"""Append provisional design documentation, preserving delivered CAD and variants."""
from pathlib import Path
import argparse,hashlib,json,zipfile
ROOT=Path(__file__).resolve().parents[1]
def sha(data):return hashlib.sha256(data).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();prefix=ROOT.name+'/'
 paths=['README-PRINT-CONCEPT.md','docs/BOM-ja.md','docs/DESIGN-ASSUMPTIONS-ja.md','docs/package_documentation_update.py']
 files={prefix+n:(ROOT/n).read_bytes()for n in paths};old_readme=prefix+'README-PRINT-CONCEPT.md';history=prefix+'docs/history/README-PRINT-CONCEPT-before-provisional-wiring.md'
 with zipfile.ZipFile(a.baseline)as z:
  assert z.testzip()is None
  old={n:z.read(n)for n in z.namelist()if not n.endswith('/')}
 assert all(n==old_readme or n not in old for n in files),'Unexpected replacement outside main README'
 assert (ROOT/'docs/history/README-PRINT-CONCEPT-before-provisional-wiring.md').read_bytes()==old[old_readme]
 files[history]=old[old_readme]
 preserved={n:sha(b)for n,b in old.items()if n!=old_readme}
 manifest={'baseline_zip_sha256':sha(a.baseline.read_bytes()),'baseline_library_version':2,'change_scope':'Documentation only; all CAD/STL/model/assets/rollouts and previous variants preserved byte-for-byte. Main README adds links; prior README preserved under docs/history.','unchanged_original_files':preserved,'prior_readme':{'original':old_readme,'archive':history,'sha256':sha(old[old_readme])},'documentation_files':{n:sha(b)for n,b in files.items()}}
 mp=ROOT/'docs/documentation-update-manifest.json';mp.write_text(json.dumps(manifest,indent=2)+'\n');files[prefix+'docs/documentation-update-manifest.json']=mp.read_bytes()
 result=old|files
 with zipfile.ZipFile(a.output,'w',zipfile.ZIP_DEFLATED,compresslevel=8)as z:
  for n,b in result.items():z.writestr(n,b)
 with zipfile.ZipFile(a.output)as z:
  assert z.testzip()is None
  assert len(z.namelist())==len(set(z.namelist()))
  for n,h in preserved.items():assert sha(z.read(n))==h
  assert z.read(history)==old[old_readme]
 assert a.output.stat().st_size<20_000_000
 print(json.dumps({'path':str(a.output),'bytes':a.output.stat().st_size,'sha256':sha(a.output.read_bytes()),'file_count':len(result),'unchanged_previous_files':len(preserved),'prior_readme_preserved':True},indent=2))
if __name__=='__main__':main()
