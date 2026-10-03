import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const html=await readFile(new URL('../src/index.html',import.meta.url),'utf8');
const css=await readFile(new URL('../src/style.css',import.meta.url),'utf8');
test('compact work surface keeps all established controls and unique IDs',()=>{
 const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(m=>m[1]);assert.equal(new Set(ids).size,ids.length);
 for(const id of['viewport','joystick','stick','slow','normal','turn-left','turn-right','pause','stand','reset','camera-home','follow','loading','alert'])assert.ok(ids.includes(id),id);
 assert.ok(!/keyboard-hint|control-heading|model-tag|session-note|<footer|BROWSER PHYSICS/.test(html));
});
test('icon controls have accessible names and binary tracking uses a switch',()=>{
 for(const match of html.matchAll(/<button\b[^>]*>/g))assert.ok(/aria-label=/.test(match[0])||/id="(?:slow|normal)"/.test(match[0]),match[0]);
 assert.match(html,/id="follow"[^>]*role="switch"[^>]*aria-checked="true"/);
 assert.match(html,/id="slow"[^>]*aria-pressed="true"/);
 assert.match(html,/id="normal"[^>]*aria-pressed="false"/);
});
test('layout has stable typography and no framed scene or floating section cards',()=>{
 assert.ok(!/font-size:[^;}]*(?:vw|vh)/.test(css));assert.ok(!/linear-gradient|radial-gradient/.test(css));
 assert.match(css,/\.lab\{[^}]*height:100dvh/);assert.match(css,/#viewport\{[^}]*inset:0/);
 assert.ok(!/\.stage\{[^}]*(?:border-radius|margin:)/.test(css));
});
