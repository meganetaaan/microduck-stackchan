import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {gunzipSync} from 'node:zlib';
const manifest=JSON.parse(await readFile(new URL('../reference/baseline-manifest.json',import.meta.url)));
test('trained no-arms physics, controller, model and assets remain byte-identical',async()=>{
  assert.equal(manifest.baseline,'NOARMS');
  for(const [name,expected]of Object.entries(manifest.files)){
    const bytes=await readFile(new URL('../src/'+name,import.meta.url));
    assert.equal(bytes.length,expected.bytes,name);
    assert.equal(createHash('sha256').update(bytes).digest('hex'),expected.sha256,name);
  }
});
test('visible no-arms baseline and unvalidated arm distinction are retained',async()=>{
  const html=await readFile(new URL('../src/index.html',import.meta.url),'utf8');
  assert.match(html,/<strong>NOARMS · 736\.537 g<\/strong>/);
  assert.match(html,/案4 \/ A/);
  assert.match(html,/再学習・検証がまだ行われていない/);
});
test('four upload-safe chunks preserve the exact original gzip and compiled model',async()=>{
  const archive=manifest.model_archive;
  assert.equal(archive.parts.length,4);
  const parts=await Promise.all(archive.parts.map(name=>readFile(new URL('../src/'+name,import.meta.url))));
  assert.ok(parts.every(bytes=>bytes.length<=7000000));
  const joined=Buffer.concat(parts),model=gunzipSync(joined);
  assert.equal(joined.length,archive.bytes);
  assert.equal(createHash('sha256').update(joined).digest('hex'),archive.sha256);
  assert.equal(model.length,archive.uncompressed_bytes);
  assert.equal(createHash('sha256').update(model).digest('hex'),archive.uncompressed_sha256);
});
