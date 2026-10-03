// Real-browser QA. Requires `npm install --no-save --package-lock=false playwright@1.51.1`
// and its supported Chromium installation. Run against a permitted local preview URL:
// npm run serve:pages (in another terminal)
// node scripts/check-browser.mjs http://127.0.0.1:8000/microduck-stackchan/
// Optional CHROMIUM_PATH selects an already-installed Chromium binary.
// Never substitute a production URL or bypass a preview/access restriction.
import assert from 'node:assert/strict';
import {mkdir,writeFile} from 'node:fs/promises';
import {chromium} from 'playwright';
const url=process.argv[2];
assert.ok(url&&/^http:\/\/(localhost|127\.0\.0\.1)(:\d+)?\/(?:[a-zA-Z0-9_-]+\/)*$/.test(url),'Supply the authorized localhost preview URL');
await mkdir('test-results/browser',{recursive:true});
const browser=await chromium.launch(process.env.CHROMIUM_PATH?{executablePath:process.env.CHROMIUM_PATH}:{});
const results=[];
try{
 for(const size of[{name:'desktop',width:1440,height:900,mobile:false},{name:'mobile',width:390,height:844,mobile:true}]){
  const context=await browser.newContext({viewport:{width:size.width,height:size.height},deviceScaleFactor:1,isMobile:size.mobile,hasTouch:size.mobile});
  const page=await context.newPage(),failed=[],assets=[];
  const prefix=new URL(url).pathname;
  page.on('pageerror',e=>failed.push(e.message));
  page.on('response',r=>{if(r.url().includes('/model/')||r.url().includes('/vendor/'))assets.push({path:new URL(r.url()).pathname,status:r.status()});});
  await page.goto(url,{waitUntil:'domcontentloaded'});
  await page.getByRole('button',{name:'開始',exact:true}).waitFor({state:'visible'});
  await page.waitForFunction(()=>!document.getElementById('pause').disabled,{timeout:120000});
  assert.deepEqual(failed,[]);
  const layout=await page.evaluate(()=>{
   const box=el=>{const r=el.getBoundingClientRect();return{x:r.x,y:r.y,w:r.width,h:r.height}};
   const ids=['joystick','slow','normal','turn-left','turn-right','pause','stand','reset'];
   return{viewport:box(document.querySelector('#viewport')),scrollWidth:document.documentElement.scrollWidth,width:innerWidth,controls:ids.map(id=>({id,...box(document.getElementById(id))}))};
  });
  assert.ok(layout.scrollWidth<=layout.width,'horizontal overflow');
  assert.ok(layout.viewport.w>=size.width-1,'scene must remain full bleed');
  assert.ok(layout.viewport.h>size.height*.55,'scene must dominate viewport');
  for(let i=0;i<layout.controls.length;i++)for(let j=i+1;j<layout.controls.length;j++){
   const a=layout.controls[i],b=layout.controls[j];
   assert.ok(a.x+a.w<=b.x+.5||b.x+b.w<=a.x+.5||a.y+a.h<=b.y+.5||b.y+b.h<=a.y+.5,`${a.id}/${b.id} overlap`);
  }
  const pixels=()=>page.evaluate(()=>new Promise(resolve=>requestAnimationFrame(()=>{
   const canvas=document.querySelector('#viewport canvas'),gl=canvas.getContext('webgl2');
   const data=new Uint8Array(canvas.width*canvas.height*4);gl.readPixels(0,0,canvas.width,canvas.height,gl.RGBA,gl.UNSIGNED_BYTE,data);
   let opaque=0,dark=0,orange=0,minX=canvas.width,maxX=0,minY=canvas.height,maxY=0,hash=2166136261;
   for(let i=0;i<data.length;i+=4){const[r,g,b,a]=data.subarray(i,i+4);if(a>0)opaque++;if(r<100&&g<110&&b<110)dark++;if(r>g*1.35&&r>b*1.5&&r>100){orange++;const x=(i/4)%canvas.width,y=Math.floor(i/4/canvas.width);minX=Math.min(x,minX);maxX=Math.max(x,maxX);minY=Math.min(y,minY);maxY=Math.max(y,maxY);}hash=Math.imul(hash^r,16777619);hash=Math.imul(hash^g,16777619);hash=Math.imul(hash^b,16777619);}
   resolve({width:canvas.width,height:canvas.height,opaque,dark,orange,bounds:{minX,maxX,minY,maxY},hash:hash>>>0});
  })));
  const initial=await pixels();assert.ok(initial.opaque>initial.width*initial.height*.95,'blank/transparent canvas');assert.ok(initial.dark>150,'dark robot/screen pixels missing');assert.ok(initial.orange>20,'actual orange robot assets missing');assert.ok(initial.bounds.minX>5&&initial.bounds.maxX<initial.width-5&&initial.bounds.minY>5&&initial.bounds.maxY<initial.height-5,'robot orange regions clipped');
  await page.screenshot({path:`test-results/browser/${size.name}-paused.png`});
  await page.getByRole('button',{name:'開始',exact:true}).click();
  await page.keyboard.down('w');
  await page.waitForFunction(()=>parseFloat(document.getElementById('sim-time').textContent)>2);
  const moving=await pixels();assert.notEqual(moving.hash,initial.hash,'render did not move');
  await page.screenshot({path:`test-results/browser/${size.name}-moving.png`});
  await page.keyboard.up('w');await page.getByRole('button',{name:'一時停止',exact:true}).click();
  assert.ok(assets.some(a=>a.path.endsWith('/screen.png')&&a.status===200),'screen asset not loaded');
  assert.ok(assets.some(a=>a.path.endsWith('/policy.onnx')&&a.status===200),'policy asset not loaded');
  assert.ok(assets.every(a=>a.path.startsWith(prefix)&&a.status===200),'all assets must load within the project subpath');
  results.push({name:size.name,layout,pixels:{initial,moving},assets,errors:failed});await context.close();
 }
 await writeFile('test-results/browser/results.json',JSON.stringify({completed:true,base:new URL(url).pathname,browser:await browser.version(),mobile_coverage:'Emulated Chromium mobile viewport and touch capability; physical mobile devices not tested',results},null,2));
}finally{await browser.close();}
