import './generate-icons.mjs';
import {cp,mkdir,copyFile,readFile,writeFile,rm} from 'node:fs/promises';
import {createHash} from 'node:crypto';
// Packaging may change, but the exact concatenated gzip must not change.
const manifest=JSON.parse(await readFile('reference/baseline-manifest.json'));
const modelParts=await Promise.all(manifest.model_archive.parts.map(name=>readFile('src/'+name)));
if(modelParts.length!==4||modelParts.some(bytes=>bytes.length>7000000))throw new Error('Expected four model archive parts below 7 MB');
if(createHash('sha256').update(Buffer.concat(modelParts)).digest('hex')!==manifest.model_archive.sha256)throw new Error('Model archive integrity mismatch');
// A clean build must depend only on tracked sources and pinned npm packages.
await rm('dist',{recursive:true,force:true});
await mkdir('dist/vendor/mujoco',{recursive:true});
await mkdir('dist/vendor/ort',{recursive:true});
await mkdir('dist/vendor/three',{recursive:true});
for(const p of ['mujoco.js','mujoco.wasm'])await copyFile('node_modules/@mujoco/mujoco/'+p,'dist/vendor/mujoco/'+p);
for(const p of ['ort.wasm.min.mjs','ort-wasm-simd-threaded.mjs','ort-wasm-simd-threaded.wasm'])await copyFile('node_modules/onnxruntime-web/dist/'+p,'dist/vendor/ort/'+p);
for(const p of ['three.module.js','three.core.js'])await copyFile('node_modules/three/build/'+p,'dist/vendor/three/'+p);
await copyFile('node_modules/three/examples/jsm/controls/OrbitControls.js','dist/vendor/three/OrbitControls.js');
await cp('src','dist',{recursive:true});
await copyFile('reference/baseline-manifest.json','dist/baseline-manifest.json');
await writeFile('dist/.nojekyll','');
console.log('Static app built. All asset URLs are relative; GitHub project subpaths are supported.');
