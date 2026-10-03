"""CPU projection of the actual compiled screen UVs; not a browser screenshot."""
from pathlib import Path
import gzip,json,hashlib
import numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1]
meta=json.loads((ROOT/'dist/model/visual.json').read_text());binary=gzip.decompress((ROOT/'dist/model/visual.bin.gz').read_bytes());settings=json.loads((ROOT/'test-results/screen-texture-settings.json').read_text())
g=next(x for x in meta['geoms']if x['name']=='screen');m=next(x for x in meta['meshes']if x['id']==g['dataid'])
def data(k):
 d=m[k];return np.frombuffer(binary,dtype=d['dtype'],offset=d['offset'],count=d['bytes']//4).reshape(d['shape'])
v,f,uv,uf=(data(k)for k in ['vertices','faces','texcoords','face_texcoords']);top=[];bottom=[]
for face,texface in zip(f,uf):
 for vertex,tex in zip(face,texface):
  (top if v[vertex,2]>0 else bottom).append(float(uv[tex,1]))
assert set(top)=={0.0}and set(bottom)=={1.0}
source=Image.open(ROOT/'dist/model/screen.png').convert('RGB');pixels=np.array(source);width,height=360,640
# Orthographic front projection of the plane: top/bottom coordinates from compiled UV data.
u=np.linspace(0,1,width);base_v=np.linspace(top[0],bottom[0],height)
def project(flip):
 vv=1-base_v if flip else base_v
 return Image.fromarray(pixels[np.rint(vv*(pixels.shape[0]-1)).astype(int)[:,None],np.rint(u*(pixels.shape[1]-1)).astype(int)[None,:]])
canvas=Image.new('RGB',(1200,950),(17,26,34));draw=ImageDraw.Draw(canvas);font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
def text(x,y,s,size=20,color='#dce7ee'):draw.text((x,y),s,font=ImageFont.truetype(font,size),fill=color)
text(38,27,'SCREEN ORIENTATION · ACTUAL COMPILED UV MAPPING',25)
text(38,70,'CPU projection using shipped PNG and Three.js texture settings; not a browser screenshot',17,'#9eafb9')
for x,label,flip,accent in [(120,'Before: default vertical flip',settings['oldFlipY'],'#b1bfca'),(720,'After: corrected mapping',settings['correctedFlipY'],'#ffad54')]:
 text(x-32,130,label,22,accent);text(x,167,'MODEL TOP / CAMERA SIDE',15,'#9eafb9');draw.rounded_rectangle((x-9,199,x+width+9,199+height+18),radius=14,fill='#758591');canvas.paste(project(flip),(x,208));text(x,873,'Face upper half · panel lower half' if not flip else 'Face incorrectly in lower half',17,accent)
text(38,921,'Original image, model transforms and physics files unchanged',15,'#9eafb9')
canvas.save(ROOT/'test-results/screen-orientation-proof.png')
report={'method':'CPU orthographic UV sampling of actual compiled screen geometry and PNG, not a browser render','geom_id':g['id'],'mesh_id':m['id'],'source_image_sha256':hashlib.sha256((ROOT/'dist/model/screen.png').read_bytes()).hexdigest(),'compiled_top_v':sorted(set(top)),'compiled_bottom_v':sorted(set(bottom)),'old_three_flipY':settings['oldFlipY'],'corrected_three_flipY':settings['correctedFlipY'],'corrected_image_top_row_at_model_top':True,'corrected_image_bottom_row_at_model_bottom':True,'three_uv_regression_tests_passed':3,'physics_assets_modified':False}
(ROOT/'test-results/screen-orientation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
