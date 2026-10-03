"""Editable SVG display art + UV-mapped visual variant; no physical model edits."""
from pathlib import Path
import xml.etree.ElementTree as E
import hashlib,json,subprocess,os
H=Path(__file__).resolve().parent; B=H.parent
SVG='''<svg xmlns="http://www.w3.org/2000/svg" width="720" height="1280" viewBox="0 0 720 1280">
<title>Stack-chan face and MicroDuck-inspired lower robot panel</title>
<desc>Portrait Tab5 720 by 1280 pixels. Top half: two small white eyes and a short white neutral mouth, redrawn from the supplied reference. Bottom half: purely decorative mechanical panels in warm white, gray and orange.</desc>
<defs>
 <linearGradient id="panel" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#eeeee8"/><stop offset="1" stop-color="#d4d6d2"/></linearGradient>
 <linearGradient id="inset" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#b7bfbd"/><stop offset="1" stop-color="#d5d9d4"/></linearGradient>
</defs>
<!-- Exact upper half is reserved for the face. No labels or UI telemetry. -->
<rect width="720" height="640" fill="#303137"/>
<circle cx="215" cy="264" r="16" fill="#fcfcfa"/>
<circle cx="505" cy="264" r="16" fill="#fcfcfa"/>
<rect x="277" y="391" width="166" height="23" rx="11.5" fill="#fcfcfa"/>
<!-- Lower half: broad softened panels repeat the robot's casing language. -->
<rect y="640" width="720" height="640" fill="#525959"/>
<path d="M28 665 Q28 648 45 648 H675 Q692 648 692 665 V1235 Q692 1256 671 1256 H49 Q28 1256 28 1235Z" fill="url(#panel)"/>
<path d="M52 676 H668" stroke="#fafaf5" stroke-width="4"/>
<path d="M48 1237 H672" stroke="#a2aaa6" stroke-width="5"/>
<!-- Orange yoke and gray inset have no functional control meaning. -->
<path d="M76 710 H644 V799 L600 829 H120 L76 799Z" fill="#fc7b16"/>
<path d="M92 712 H628" stroke="#ffb060" stroke-width="4"/>
<rect x="122" y="730" width="476" height="62" rx="18" fill="#626a69"/>
<rect x="138" y="741" width="444" height="4" rx="2" fill="#818b87"/>
<path d="M117 862 H603 Q627 862 627 886 V1169 Q627 1192 604 1192 H116 Q93 1192 93 1169 V886 Q93 862 117 862Z" fill="url(#inset)" stroke="#a1aaa6" stroke-width="3"/>
<path d="M116 868 H602" stroke="#f5f6ef" stroke-width="3"/>
<!-- Low-profile circular panel detail echoes the orange foot hardware. -->
<circle cx="360" cy="1010" r="102" fill="#7b8581"/>
<circle cx="360" cy="1007" r="90" fill="#e9ece4"/>
<circle cx="360" cy="1007" r="66" fill="#c3cbc5"/>
<circle cx="360" cy="1007" r="48" fill="#eff0e9"/>
<path d="M299 926 A102 102 0 0 1 421 926" fill="none" stroke="#fc7b16" stroke-width="14" stroke-linecap="round"/>
<!-- Three shallow vent marks per side, no dense fake UI. -->
<g fill="#87918c"><rect x="129" y="963" width="78" height="11" rx="5.5"/><rect x="129" y="996" width="78" height="11" rx="5.5"/><rect x="129" y="1029" width="78" height="11" rx="5.5"/><rect x="513" y="963" width="78" height="11" rx="5.5"/><rect x="513" y="996" width="78" height="11" rx="5.5"/><rect x="513" y="1029" width="78" height="11" rx="5.5"/></g>
<rect x="260" y="1143" width="200" height="9" rx="4.5" fill="#a5aea9"/>
<g fill="#8c9790"><circle cx="63" cy="690" r="6"/><circle cx="657" cy="690" r="6"/><circle cx="63" cy="1214" r="6"/><circle cx="657" cy="1214" r="6"/></g>
</svg>'''
(H/'assets/stackchan_microduck_screen.svg').write_text(SVG)
inkscape_env=dict(os.environ)
inkscape_env['INKSCAPE_PROFILE_DIR']=str(H/'_inkscape_profile')
inkscape_env['XDG_CACHE_HOME']=str(H/'_render_cache')
subprocess.run(['inkscape',str(H/'assets/stackchan_microduck_screen.svg'),'--export-type=png','--export-filename='+str(H/'assets/stackchan_microduck_screen_720x1280.png'),'--export-width=720','--export-height=1280'],check=True,stdout=subprocess.DEVNULL,env=inkscape_env)
# Existing screen-box dimensions and transform are preserved exactly. Front is +X.
verts=[(x,y,z) for x in [-.00015,.00015] for y in [-.0311,.0311] for z in [-.0552,.0552]]
# Front UVs: image top follows +Z, camera remains above the display.
faces=[(5,7,8),(5,8,6),(1,2,4),(1,4,3),(1,5,6),(1,6,2),(3,4,8),(3,8,7),(1,3,7),(1,7,5),(2,6,8),(2,8,4)]
uv=[(0,0),(0,1),(1,0),(1,1)]
lines=['# Visual-only screen box, metres, original extents']+['v %.9f %.9f %.9f'%v for v in verts]+['vt %.1f %.1f'%t for t in uv]
# y=- maps left u=0, z=+ maps top v=1. Seam-free on visible front.
idx={1:1,2:2,3:3,4:4,5:1,6:2,7:3,8:4}
lines += ['f '+' '.join(f'{v}/{idx[v]}' for v in f) for f in faces]
(H/'assets/screen_surface.obj').write_text('\n'.join(lines)+'\n')
root=E.parse(B/'tab5_assembly_uart.xml').getroot()
for a in root.find('asset'):
 if a.get('file'):a.set('file','../'+a.get('file'))
a=root.find('asset');E.SubElement(a,'mesh',name='stackchan_screen_surface',file='assets/screen_surface.obj');E.SubElement(a,'texture',name='stackchan_screen_texture',type='2d',file='assets/stackchan_microduck_screen_720x1280.png');E.SubElement(a,'material',name='stackchan_screen_material',texture='stackchan_screen_texture',texuniform='false',rgba='1 1 1 1',emission='1',specular='0',shininess='0',reflectance='0')
g=root.find(".//geom[@name='screen']");g.attrib.pop('size');g.set('type','mesh');g.set('mesh','stackchan_screen_surface');g.set('rgba','1 1 1 1');g.set('material','stackchan_screen_material')
for n in ['eye_left','eye_right']:
 g=root.find(f".//geom[@name='{n}']");v=g.get('rgba').split();v[-1]='0';g.set('rgba',' '.join(v))
E.indent(root);E.ElementTree(root).write(H/'tab5_assembly_uart_screen.xml',encoding='utf-8',xml_declaration=True)
s=E.parse(B/'scene_tab5_assembly_uart.xml');s.getroot().find('include').set('file','tab5_assembly_uart_screen.xml');s.write(H/'scene_tab5_assembly_uart_screen.xml',encoding='utf-8',xml_declaration=True)
report={'physical_baseline':'../tab5_assembly_uart.xml','physical_baseline_sha256':hashlib.sha256((B/'tab5_assembly_uart.xml').read_bytes()).hexdigest(),'visual_variant_sha256':hashlib.sha256((H/'tab5_assembly_uart_screen.xml').read_bytes()).hexdigest(),'texture_pixels':[720,1280],'camera_orientation':'top','face_region_pixels':[0,0,720,640],'panel_region_pixels':[0,640,720,1280],'reference_observed':'Two small white circular eyes and a short rounded horizontal white mouth on charcoal. Redrawn as editable vector artwork; source photograph unmodified.','changes':['screen visual box becomes UV-mapped mesh with identical extents','old eye visuals alpha set to zero','texture and material assets added'],'physics_claim':'No mass, inertia, joint, actuator, contact, observation, camera or physical envelope change; independent verifier required.','resolution_source':'https://docs.m5stack.com/en/core/Tab5'}
(H/'screen_design.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
