"""Save-as packager: preserve every original v2 byte, add only this alternative.

Usage: python package_portrait.py --baseline ORIGINAL.zip --output NEW.zip
Excludes fetched third-party assets, weights, environments and incidental logs.
"""
from pathlib import Path
import argparse,hashlib,json,zipfile
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
def sha(data):return hashlib.sha256(data).hexdigest()
def main():
 a=argparse.ArgumentParser();a.add_argument('--baseline',type=Path,required=True);a.add_argument('--output',type=Path,required=True);args=a.parse_args()
 prefix=ROOT.name+'/';records=[]
 with zipfile.ZipFile(args.baseline)as z:
  for i in z.infolist():
   if i.is_dir():continue
   original=z.read(i);local=ROOT.parent/i.filename
   assert local.read_bytes()==original,'Prior artifact changed: '+i.filename
   records.append({'path':i.filename,'sha256':sha(original)})
  manifest={'baseline_zip_sha256':sha(args.baseline.read_bytes()),'preserved_original_file_count':len(records),'all_original_files_byte_identical':True,'original_files':records}
  (HERE/'preservation_manifest.json').write_text(json.dumps(manifest,indent=2))
  with zipfile.ZipFile(args.output,'w',zipfile.ZIP_DEFLATED,compresslevel=8)as out:
   for i in z.infolist():out.writestr(i,z.read(i))
   for path in [ROOT/'README-PORTRAIT.md',ROOT/'bootstrap_portrait.sh',*sorted(HERE.rglob('*'))]:
    if not path.is_file()or '__pycache__'in path.parts or path.suffix=='.log' or path.name.startswith('local_only_smoke'):continue
    out.write(path,prefix+str(path.relative_to(ROOT)))
 with zipfile.ZipFile(args.output)as z:
  assert z.testzip()is None
  for r in records:assert sha(z.read(r['path']))==r['sha256']
  print(json.dumps({'file':str(args.output),'size_bytes':args.output.stat().st_size,'sha256':sha(args.output.read_bytes()),'file_count':len(z.namelist()),'unchanged_baseline_files':len(records)},indent=2))
if __name__=='__main__':main()
