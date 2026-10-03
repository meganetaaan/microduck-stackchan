import loadMujoco from './vendor/mujoco/mujoco.js';
import * as ort from './vendor/ort/ort.wasm.min.mjs';
import {Physics} from './physics.js';
import {CommandRamp,GAITS} from './control.js';
let physics,session,visual,paused=true,ready=false,busy=false,next=0,heartbeat=0,lastPublish=0,epoch=0,lastMeasure=0,lastSim=0,ratio=0;const ramp=new CommandRamp();
const emit=(type,body={})=>postMessage({type,...body});
async function json(p){const r=await fetch(new URL(p,import.meta.url));if(!r.ok)throw new Error(`${p}: HTTP ${r.status}`);return r.json();}
async function binary(p){const r=await fetch(new URL(p,import.meta.url));if(!r.ok)throw new Error(`${p}: HTTP ${r.status}`);return new Uint8Array(await r.arrayBuffer());}
function publish(force=false){if(!ready||(!force&&performance.now()-lastPublish<32))return;lastPublish=performance.now();const transforms=physics.transforms(visual.geoms.map(g=>g.id));postMessage({type:'state',transforms,telemetry:physics.telemetry(),paused,mode:ramp.mode,ratio},[transforms.buffer]);}
function stop(){ramp.set('stand');heartbeat=0;}
function pause(){paused=true;stop();epoch++;publish(true);}
async function init(){try{emit('progress',{message:'MuJoCo WASMを読み込んでいます'});ort.env.wasm.numThreads=1;ort.env.wasm.wasmPaths=new URL('./vendor/ort/',import.meta.url).href;ort.env.wasm.proxy=false;
 const [mj,config,v]=await Promise.all([loadMujoco({locateFile:path=>new URL('./vendor/mujoco/'+path,import.meta.url).href}),json('./model/config.json'),json('./model/visual.json')]);
 if(mj.mj_versionString()!=='3.10.0')throw new Error('Unexpected physics engine version');visual=v;
 emit('progress',{message:'機体の物理モデルを準備しています'});const parts=await Promise.all(['part1','part2','part3','part4'].map(async part=>{const r=await fetch(new URL('./model/model.mjb.gz.'+part,import.meta.url));if(!r.ok)throw new Error('Model download failed');return r.arrayBuffer();}));const bytes=new Uint8Array(await new Response(new Blob(parts).stream().pipeThrough(new DecompressionStream('gzip'))).arrayBuffer());mj.FS.writeFile('/model.mjb',bytes);const vfs=new mj.MjVFS();const model=mj.MjModel.from_binary_path('/model.mjb',vfs);vfs.delete();mj.FS.unlink('/model.mjb');physics=new Physics(mj,model,config);
 emit('progress',{message:'学習済み方策を準備しています'});session=await ort.InferenceSession.create(await binary('./model/policy.onnx'),{executionProviders:['wasm'],graphOptimizationLevel:'all'});
 ready=true;lastMeasure=performance.now();lastSim=0;emit('ready',{version:mj.mj_versionString(),counts:{nq:model.nq,nv:model.nv,nu:model.nu,ngeom:model.ngeom,npair:model.npair}});publish(true);setTimeout(tick,0);
 }catch(error){emit('error',{message:error.message,stack:error.stack});}}
async function tick(){if(!ready)return;const now=performance.now();if(!paused&&!busy){if(now-heartbeat>500)stop();if(now>=next){busy=true;const runEpoch=epoch;try{const command=ramp.step();const obs=physics.observation(command);const output=await session.run({[session.inputNames[0]]:new ort.Tensor('float32',obs,[1,39])});if(runEpoch===epoch&&!paused){physics.step(output[session.outputNames[0]].data,command);if(physics.fallen||physics.unsafeContact){paused=true;stop();emit('fall',{message:physics.unsafeContact?'自己接触を検出しました。リセットしてください':'転倒を検出しました。リセットしてください'});}publish();}next=Math.max(next+20,performance.now()-20);}catch(error){pause();emit('error',{message:error.message,stack:error.stack});}finally{busy=false;}}}else next=now;
 if(now-lastMeasure>1000){ratio=(physics.data.time-lastSim)/((now-lastMeasure)/1000);lastSim=physics.data.time;lastMeasure=now;publish();}setTimeout(tick,Math.max(0,Math.min(10,next-performance.now())));}
onmessage=event=>{const m=event.data;if(m.type==='init'){if(!physics)init();return;}if(!ready)return;if(m.type==='command'){if(!paused&&Object.hasOwn(GAITS,m.mode)){heartbeat=performance.now();ramp.set(m.mode);}}else if(m.type==='pause')pause();else if(m.type==='resume'){epoch++;ramp.reset();physics.lastCommand=[0,0,0];heartbeat=performance.now();paused=false;next=performance.now();publish(true);}else if(m.type==='reset'){pause();physics.reset();ramp.reset();emit('reset');publish(true);}else if(m.type==='stand'){stop();publish(true);}};
