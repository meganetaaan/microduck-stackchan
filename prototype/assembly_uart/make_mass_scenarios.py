"""Create uncertainty scenarios without changing the frozen nominal assembly."""
from pathlib import Path
import json,xml.etree.ElementTree as ET,hashlib
H=Path(__file__).resolve().parent
r=json.loads((H/'assembly_report.json').read_text());out={}
for label,index in [('light',0),('heavy',1)]:
 tree=ET.parse(H/'tab5_assembly_uart.xml');root=tree.getroot();changes=[]
 for p in r['parts']:
  bound=p.get('uncertainty_mass_g')
  if bound is None:continue
  old=p['mass_kg'];new=max(1e-7,bound[index]/1000);b=root.find(f".//body[@name='assembly_{p['name']}']");i=b.find('inertial');i.set('mass',str(new));i.set('fullinertia',' '.join(str(float(v)*new/old)for v in i.get('fullinertia').split()));changes.append({'part':p['name'],'nominal_mass_kg':old,'scenario_mass_kg':new})
 root.set('model','tab5_uart_assembly_'+label);ET.indent(tree);fn=H/f'tab5_assembly_uart_{label}.xml';tree.write(fn,encoding='utf-8',xml_declaration=True);s=ET.parse(H/'scene_tab5_assembly_uart.xml');s.getroot().find('include').set('file',fn.name);s.write(H/f'scene_tab5_assembly_uart_{label}.xml')
 out[label]={'total_mass_kg':sum(float(i.get('mass'))for i in root.iter('inertial')),'changes':changes,'model_sha256':hashlib.sha256(fn.read_bytes()).hexdigest()}
out['note']='Assumed electronic/wiring/hardware mass bounds, not measured probabilities. Same shape/COM perpart; inertia scaled with mass. Printed density and original robot/Tab5 masses unchanged. Zero-lower-bound allowance uses1e−7kg numericalminimum. Nominaltrainingmodelnotmodified.'
(H/'mass_scenarios.json').write_text(json.dumps(out,indent=2));print({k:v['total_mass_kg']for k,v in out.items()if isinstance(v,dict)})
