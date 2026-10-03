"""Overlay a screen-only visual release on an existing compact assembly archive."""
from pathlib import Path
import argparse,json,hashlib,zipfile,posixpath,xml.etree.ElementTree as E
H=Path(__file__).resolve().parent;ROOT=H.parents[2]
def sha(b):return hashlib.sha256(b).hexdigest()
def main():
 p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();prefix=ROOT.name+'/'
 with zipfile.ZipFile(a.baseline) as z:
  assert z.testzip() is None;old={n:z.read(n) for n in z.namelist()}
 out=dict(old);docs=['README.md','README-ASSEMBLY-UART-ja.md']
 for n in docs:
  history='docs/history/v6-screen/'+n;assert (ROOT/history).read_bytes()==old[prefix+n]
  out[prefix+n]=(ROOT/n).read_bytes();out[prefix+history]=(ROOT/history).read_bytes()
 allow={'.py','.json','.md','.png','.svg','.obj','.glb'}
 files=[f for f in H.rglob('*') if f.is_file() and f.suffix in allow and not any(c.startswith('_') for c in f.relative_to(H).parts) and f.name!='screen_release_manifest.json']
 for f in files:out[prefix+str(f.relative_to(ROOT))]=f.read_bytes()
 for name in ['tab5_assembly_uart_screen.xml','scene_tab5_assembly_uart_screen.xml']:out[prefix+str((H/name).relative_to(ROOT))]=(H/name).read_bytes()
 frozen=json.loads((H.parent/'frozen_model_manifest.json').read_text());assert all(sha((ROOT/n).read_bytes())==s for n,s in frozen['files'].items())
 report={'base_archive_sha256':sha(a.baseline.read_bytes()),'base_library_version':6,'purpose':'Screen-only Stack-chan face and MicroDuck-color lower robot panel','frozen_physics_files_verified':len(frozen['files']),'baseline_physics_sha256':frozen['nominal_model_sha256'],'visual_xml_sha256':sha((H/'tab5_assembly_uart_screen.xml').read_bytes()),'unchanged_original_physics_assets':True,'training_addon_included':False,'changed_documents':{n:{'before':sha(old[prefix+n]),'after':sha(out[prefix+n]),'history':'docs/history/v6-screen/'+n}for n in docs},'new_visual_files':{prefix+str(f.relative_to(ROOT)):sha(f.read_bytes())for f in files}}
 rp=H/'screen_release_manifest.json';rp.write_text(json.dumps(report,indent=2)+'\n');out[prefix+str(rp.relative_to(ROOT))]=rp.read_bytes()
 mp=H.parent/'working_release_manifest.json';mn=prefix+str(mp.relative_to(ROOT));mf=json.loads(old[mn]);mf['screen_visual_update']=report;mf['file_hashes']={n:sha(b) for n,b in out.items() if n!=mn};mp.write_text(json.dumps(mf,indent=2)+'\n');out[mn]=mp.read_bytes()
 assert not any(n.startswith(prefix+'training/') for n in out)
 allowed={prefix+n for n in docs}|{mn};assert all(out[n]==b for n,b in old.items() if n not in allowed)
 robotname=prefix+str((H/'tab5_assembly_uart_screen.xml').relative_to(ROOT))
 for asset in E.fromstring(out[robotname]).find('asset'):
  if not asset.get('file'):continue
  ref=posixpath.normpath(posixpath.join(posixpath.dirname(robotname),asset.get('file')));assert '/microduck_rl/' in ref or ref in out,ref
 with zipfile.ZipFile(a.output,'w',zipfile.ZIP_DEFLATED,compresslevel=8) as z:
  for n,b in out.items():z.writestr(n,b)
 with zipfile.ZipFile(a.output) as z:assert z.testzip() is None and len(z.namelist())==len(set(z.namelist()))
 assert a.output.stat().st_size<20_000_000
 print(json.dumps({'path':str(a.output),'bytes':a.output.stat().st_size,'sha256':sha(a.output.read_bytes()),'files':len(out),'frozen_files_verified':len(frozen['files']),'old_files_preserved_byte_identical':len(old)-len(allowed)},indent=2))
if __name__=='__main__':main()
