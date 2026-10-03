import assert from 'node:assert/strict';
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {startServer} from './serve.mjs';
const {server,url}=await startServer({base:'/microduck-stackchan/',port:0});
try{
  const manifest=JSON.parse(await readFile('reference/baseline-manifest.json'));
  const assets=['index.html','app.js','style.css','viewer.js','sim-worker.js','physics.js','control.js','icons.js','screen-texture.js','validation-report.json','baseline-manifest.json','NOTICE.txt','vendor/mujoco/mujoco.js','vendor/mujoco/mujoco.wasm','vendor/ort/ort.wasm.min.mjs','vendor/ort/ort-wasm-simd-threaded.mjs','vendor/ort/ort-wasm-simd-threaded.wasm','vendor/three/three.module.js','vendor/three/three.core.js','vendor/three/OrbitControls.js',...Object.keys(manifest.files).filter(p=>p.startsWith('model/'))];
  const results=[];
  for(const asset of assets){
    const response=await fetch(new URL(asset,url));assert.equal(response.status,200,asset);
    const bytes=Buffer.from(await response.arrayBuffer());assert.ok(bytes.length,asset);
    if(asset.endsWith('.wasm'))assert.equal(response.headers.get('content-type'),'application/wasm',asset);
    if(manifest.files[asset])assert.equal(createHash('sha256').update(bytes).digest('hex'),manifest.files[asset].sha256,asset);
    results.push({asset,status:response.status,bytes:bytes.length});
  }
  const html=await(await fetch(url)).text();
  for(const match of html.matchAll(/(?:src|href)="([^"]+)"/g)){
    const resolved=new URL(match[1],url);assert.ok(resolved.pathname.startsWith('/microduck-stackchan/'),match[1]);
    assert.equal((await fetch(resolved)).status,200,match[1]);
  }
  assert.equal((await fetch(new URL('/model/policy.onnx',url))).status,404,'domain-root leaks must fail in this test');
  await mkdir('test-results',{recursive:true});
  await writeFile('test-results/pages-paths.json',JSON.stringify({pass:true,base:'/microduck-stackchan/',assets:results,note:'HTTP asset-path and byte-integrity check, not browser execution'},null,2)+'\n');
  console.log(`GitHub project subpath check passed: ${assets.length} assets`);
}finally{await new Promise(resolve=>server.close(resolve));}
