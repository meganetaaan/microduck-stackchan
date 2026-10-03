import { readFile } from 'node:fs/promises';
import { pathToFileURL, fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

// Read-only UI event review. No browser, renderer, inference, or Site writes.
const root = process.argv[2] || fileURLToPath(new URL('..', import.meta.url)).replace(/\/$/, '');
const { GAITS, LABELS, snapJoystick } = await import(pathToFileURL(`${root}/src/control.js`));
const source = (await readFile(`${root}/src/app.js`, 'utf8'))
  .replace(/^import .*$/gm, '')
  .replaceAll('import.meta.url', JSON.stringify(pathToFileURL(`${root}/src/app.js`).href))
  .replace(/start\(\);\s*$/, 'const started = start(); return {started, snapshot:()=>({ready,paused,mode,slow,point,keys:[...keys],activeTurn,faulted})};');

class Element {
  constructor(id = '') {
    this.id = id; this.handlers = {}; this.style = {}; this.dataset = {}; this.attributes = {};
    this.disabled = false; this.hidden = false; this.open = false; this.textContent = ''; this.innerHTML = '';
    this.firstElementChild = { icon: '' }; this.captures = new Set();
    this.classList = {
      values: new Set(), add(v) { this.values.add(v); }, remove(v) { this.values.delete(v); },
      contains(v) { return this.values.has(v); },
      toggle(v, on) { if (on === undefined) on = !this.values.has(v); on ? this.values.add(v) : this.values.delete(v); }
    };
  }
  addEventListener(type, fn) { (this.handlers[type] ??= []).push(fn); }
  dispatch(type, init = {}) {
    const event = { defaultPrevented: false, preventDefault() { this.defaultPrevented = true; }, ...init };
    for (const fn of this.handlers[type] ?? []) fn(event);
    return event;
  }
  click() { if (!this.disabled) this.dispatch('click'); }
  getBoundingClientRect() { return { left: 0, top: 0, width: 100, height: 100 }; }
  setPointerCapture(id) { this.captures.add(id); }
  hasPointerCapture(id) { return this.captures.has(id); }
  releasePointerCapture(id) { if (this.captures.delete(id)) this.dispatch('lostpointercapture', { pointerId: id }); }
  setAttribute(key, value) { this.attributes[key] = String(value); }
  getAttribute(key) { return this.attributes[key] ?? null; }
  showModal() { this.open = true; }
  close() { this.open = false; }
}

async function make({ running = true } = {}) {
  const elements = new Map(), outgoing = [], inbox = [], intervals = [];
  const el = id => { if (!elements.has(id)) elements.set(id, new Element(id)); return elements.get(id); };
  const document = new Element('document'); document.body = new Element('body'); document.hidden = false;
  document.getElementById = el;
  const window = new Element('window'); Object.assign(window, { WebAssembly: {}, Worker: {}, DecompressionStream: {} });
  let mock;
  class Worker {
    constructor() { mock = this; this.paused = true; this.mode = 'stand'; this.terminated = false; }
    postMessage(message) { outgoing.push(message); }
    terminate() { this.terminated = true; }
    state(paused = this.paused) { return { type: 'state', paused, transforms: [], telemetry: { time: 0, vx: 0, vy: 0, tilt: 0 }, ratio: 1 }; }
    emit(message) { inbox.push(message); }
    consume(message) {
      if (message.type === 'init') { this.emit({ type: 'ready' }); this.emit(this.state()); }
      else if (message.type === 'command') { if (!this.paused) this.mode = message.mode; }
      else if (message.type === 'pause' || message.type === 'reset') { this.paused = true; this.mode = 'stand'; this.emit(this.state()); }
      else if (message.type === 'resume') { this.paused = false; this.mode = 'stand'; this.emit(this.state()); }
      else if (message.type === 'stand') { this.mode = 'stand'; this.emit(this.state()); }
    }
  }
  class Viewer { async load() {} update() {} home() {} }
  const api = new Function('document', 'window', 'Worker', 'Viewer', 'GAITS', 'LABELS', 'snapJoystick', 'mountIcons', 'setIcon', 'setInterval', source)
    (document, window, Worker, Viewer, GAITS, LABELS, snapJoystick, () => {}, (node, icon) => { node.icon = icon; }, fn => intervals.push(fn));
  await api.started;
  const flushWorker = () => { while (outgoing.length) mock.consume(outgoing.shift()); };
  const flushUI = () => { while (inbox.length) mock.onmessage({ data: inbox.shift() }); };
  const flush = () => { let guard = 100; while ((outgoing.length || inbox.length) && guard-- > 0) { flushWorker(); flushUI(); } assert.ok(guard > 0, 'message loop converges'); };
  flush();
  if (running) { el('pause').click(); flush(); }
  outgoing.length = 0;
  const key = (key, repeat = false) => window.dispatch('keydown', { key, repeat });
  const up = key => window.dispatch('keyup', { key });
  const down = (id, pointerId, x = 50, y = 5) => el(id).dispatch('pointerdown', { pointerId, clientX: x, clientY: y });
  const release = (id, pointerId, type = 'pointerup') => el(id).dispatch(type, { pointerId });
  const neutral = () => {
    assert.equal(api.snapshot().mode, 'stand');
    for (const id of ['joystick', 'turn-left', 'turn-right']) {
      assert.equal(el(id).classList.contains('active'), false, `${id} neutral styling`);
      assert.equal(el(id).captures.size, 0, `${id} pointer capture released`);
    }
    assert.equal(el('stick').style.transform, '');
  };
  return { api, window, document, el, mock, inbox, outgoing, intervals, flush, flushWorker, flushUI, key, up, down, release, neutral };
}

let passed = 0, failed = 0;
async function test(name, fn) { try { await fn(); console.log(`PASS ${name}`); passed++; } catch (error) { console.log(`FAIL ${name}: ${error.message}`); failed++; } }

await test('four axes, diagonal snap, center deadzone and speed switch', async () => {
  const t = await make();
  for (const [x, y, mode] of [[50,5,'forward_slow'],[50,95,'backward'],[5,50,'left'],[95,50,'right'],[5,5,'forward_slow'],[50,50,'stand']]) {
    t.down('joystick', 1, x, y); assert.equal(t.api.snapshot().mode, mode); t.release('joystick', 1); t.neutral();
  }
  t.down('joystick', 1); t.el('normal').click(); assert.equal(t.api.snapshot().mode, 'forward');
  t.el('slow').click(); assert.equal(t.api.snapshot().mode, 'forward_slow'); t.release('joystick', 1); t.neutral();
});
await test('turn to joystick takeover clears all controls', async () => { const t = await make(); t.down('turn-left',1); t.down('joystick',2); t.release('joystick',2); t.release('turn-left',1); t.neutral(); });
await test('joystick to keyboard takeover clears all controls', async () => { const t = await make(); t.down('joystick',1); t.key('d'); t.up('d'); t.release('joystick',1); t.neutral(); });
await test('keyboard to turn takeover clears keys; old repeats stay suppressed', async () => { const t = await make(); t.key('w'); t.down('turn-left',1); t.key('w',true); assert.equal(t.api.snapshot().mode,'turn_left'); t.release('turn-left',1); t.neutral(); });
await test('turn pointer-ID ownership ignores earlier release', async () => { const t = await make(); t.down('turn-left',1); t.down('turn-left',2); t.release('turn-left',1); assert.equal(t.api.snapshot().mode,'turn_left'); t.release('turn-left',2); t.neutral(); });
await test('second joystick finger cannot take ownership', async () => { const t = await make(); t.down('joystick',1); t.down('joystick',2,95,50); t.release('joystick',2); assert.equal(t.api.snapshot().mode,'forward_slow'); t.release('joystick',1); t.neutral(); });
await test('pointercancel and lostpointercapture return to stand', async () => { for (const type of ['pointercancel','lostpointercapture']) for (const id of ['joystick','turn-left','turn-right']) { const t=await make();t.down(id,1);t.release(id,1,type);t.neutral(); } });
await test('Space and stand suppress held-key repeat until fresh press', async () => { for (const action of [t=>t.key(' '),t=>t.el('stand').click()]) { const t=await make();t.key('w');action(t);t.key('w',true);t.neutral();t.up('w');t.key('w');assert.equal(t.api.snapshot().mode,'forward_slow');t.up('w');t.neutral(); } });
await test('blur pauses immediately, flush stays paused, resume starts stand', async () => { const t=await make();t.down('joystick',1);t.window.dispatch('blur');t.neutral();assert.equal(t.api.snapshot().paused,true);t.flush();assert.equal(t.mock.paused,true);t.el('pause').click();t.flush();t.neutral();assert.equal(t.api.snapshot().paused,false); });
await test('hidden tab pauses; visible tab does not auto-resume', async () => { const t=await make();t.key('w');t.document.hidden=true;t.document.dispatch('visibilitychange');t.flush();t.neutral();t.document.hidden=false;t.document.dispatch('visibilitychange');t.flush();assert.equal(t.api.snapshot().paused,true); });
await test('details modal stops input; dismiss does not resume', async () => { const t=await make();t.down('turn-right',1);t.el('details-open').click();t.flush();t.key('w');t.neutral();assert.equal(t.api.snapshot().paused,true);assert.equal(t.el('details').open,true);t.el('details-close').click();assert.equal(t.el('details').open,false);assert.equal(t.api.snapshot().paused,true); });
await test('reset during movement pauses and clears input', async () => { const t=await make();t.key('w');t.el('reset').click();t.flush();t.neutral();assert.equal(t.mock.paused,true);assert.equal(t.api.snapshot().paused,true);t.key('w',true);t.neutral(); });
await test('BFCache preserves worker and pauses on restore', async () => { const t=await make();t.window.dispatch('pagehide',{persisted:true});t.flush();assert.equal(t.mock.terminated,false);assert.equal(t.api.snapshot().paused,true);t.el('pause').click();t.flush();assert.equal(t.api.snapshot().paused,false); });
await test('non-persisted pagehide terminates worker', async () => { const t=await make();t.window.dispatch('pagehide',{persisted:false});assert.equal(t.mock.terminated,true); });
await test('repeated acknowledged pause/resume stays synchronized', async () => { const t=await make({running:false});for(let i=0;i<10;i++){t.el('pause').click();t.flush();assert.equal(t.api.snapshot().paused,i%2===1);assert.equal(t.api.snapshot().paused,t.mock.paused);t.neutral();} });
await test('rapid start then stop before acknowledgment ends paused', async () => { const t=await make({running:false});t.el('pause').click();t.el('pause').click();const sent=t.outgoing.filter(x=>x.type==='resume'||x.type==='pause').map(x=>x.type);t.flush();assert.equal(t.mock.paused,true,`sent ${sent.join(', ')}; worker paused=${t.mock.paused}`); });
await test('queued old running state cannot undo local blur pause', async () => { const t=await make();t.mock.emit(t.mock.state(false));t.window.dispatch('blur');t.flushUI();assert.equal(t.api.snapshot().paused,true,'stale running state temporarily restored running UI'); });
await test('queued old running state cannot undo modal pause', async () => { const t=await make();t.mock.emit(t.mock.state(false));t.el('details-open').click();t.flushUI();assert.equal(t.api.snapshot().paused,true,'stale running state temporarily restored running UI inside modal'); });
console.log(`\n${passed} passed; ${failed} failed. DOM/worker mocks only; no real browser pointer or physics coverage.`);
process.exitCode=failed ? 1 : 0;
