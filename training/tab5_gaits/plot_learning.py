"""Plot every recorded iteration from measured logs (not illustrative data)."""
import argparse,json,csv
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
p=argparse.ArgumentParser();p.add_argument('--run',required=True);p.add_argument('--out',required=True);p.add_argument('--phase-boundary',type=int);a=p.parse_args();r=Path(a.run);rows=[json.loads(s)for s in (r/'learning.jsonl').read_text().splitlines()];out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
font='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf';title=ImageFont.truetype(font,28);normal=ImageFont.truetype(font,18);small=ImageFont.truetype(font,15);im=Image.new('RGB',(1280,870),'white');d=ImageDraw.Draw(im);d.text((55,25),'Tab5 10-action PPO: measured learning curves',font=title,fill='#172432');d.text((55,65),'Warm-started from MicroDuck; frozen assembly, full MuJoCo/BAM dynamics',font=normal,fill='#56636f')
def chart(rect,key,label,color,limits=None,mult=1):
 x0,y0,x1,y1=rect;d.text((x0,y0-32),label,font=normal,fill='#172432');v=[(q['steps'],q.get(key))for q in rows if q.get(key)is not None];xs=np.array([q[0]for q in v]);ys=np.array([q[1]*mult for q in v]);lo,hi=limits if limits else (float(ys.min()),float(ys.max()));span=max(hi-lo,.01);lo-=span*.08 if not limits else 0;hi+=span*.08 if not limits else 0
 for t in np.linspace(lo,hi,5):
  y=y1-(t-lo)/(hi-lo)*(y1-y0);d.line([(x0,y),(x1,y)],fill='#e5ebef',width=1);d.text((x0-55,y-9),f'{t:.3f}'if mult==1 else f'{t:.0f}%',font=small,fill='#56636f')
 for t in np.linspace(0,rows[-1]['steps'],6):
  x=x0+t/rows[-1]['steps']*(x1-x0);d.text((x-15,y1+12),f'{t/1000:.0f}k',font=small,fill='#56636f')
 if a.phase_boundary:
  boundary=x0+a.phase_boundary/rows[-1]['steps']*(x1-x0)
  for y in range(y0,y1,12):d.line([(boundary,y),(boundary,min(y+6,y1))],fill='#9b6b21',width=2)
  d.text((boundary+8,y0+5),'Robustness phase',font=small,fill='#9b6b21')
 points=[(x0+x/rows[-1]['steps']*(x1-x0),y1-(y-lo)/(hi-lo)*(y1-y0))for x,y in zip(xs,ys)];d.line(points,fill='#c2d5e0',width=1)
 smooth=np.array([np.mean(ys[max(0,i-19):i+1])for i in range(len(ys))]);d.line([(x0+x/rows[-1]['steps']*(x1-x0),y1-(y-lo)/(hi-lo)*(y1-y0))for x,y in zip(xs,smooth)],fill=color,width=3);d.text((x0+(x1-x0)//2-55,y1+36),'Control steps',font=small,fill='#56636f')
chart((110,155,1220,365),'rollout_mean_reward','Reward per 20 ms control step','#25648a')
chart((110,485,1220,710),'recent_success_rate','Training success over the most recent 100 episodes (not final validation)','#167865',(0,100),100)
d.text((55,790),'Thin line: every iteration. Thick line: 20-iteration moving average. No points omitted.',font=small,fill='#56636f');d.text((55,819),'Commands are mixed; the robustness phase changes randomization and yaw reward. See separate seeded tests.',font=small,fill='#56636f');im.save(out/'learning_curves.png')
fields=['iteration','steps','phase','elapsed_wall_s','cumulative_wall_s','control_steps_per_wall_s','rollout_mean_reward','loss','approx_kl','recent_return','recent_fall_rate','recent_success_rate']
with (out/'learning_curves.csv').open('w')as f:
 w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows({k:r.get(k)for k in fields}for r in rows)
