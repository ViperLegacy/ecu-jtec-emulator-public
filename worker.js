// Runs the emulator's own Python (cpu16, machine, board, gui) under Pyodide,
// inside a Web Worker so the page stays responsive. The page sends the same
// requests it would send to the local gui.py server; gui.api() answers them.
const PYODIDE = 'https://cdn.jsdelivr.net/pyodide/v0.26.4/full/';
importScripts(PYODIDE + 'pyodide.js');

let handle = null, slice = null;
const queue = [];
const tick = new MessageChannel();          // zero-delay scheduling (setTimeout clamps to 4 ms)
let scheduled = false;
const schedule = () => { if (!scheduled) { scheduled = true; tick.port2.postMessage(0); } };

function say(text) { postMessage({type: 'progress', text}); }

async function boot() {
  say('Loading Python (about 10 MB, cached after the first visit)…');
  const py = await loadPyodide({indexURL: PYODIDE});
  const manifest = await (await fetch('files.json', {cache: 'no-cache'})).json();
  let n = 0;
  for (const f of manifest.python) {
    say('Loading emulator files ' + (++n) + '/' + manifest.python.length + '…');
    const buf = new Uint8Array(await (await fetch('py/' + f)).arrayBuffer());
    const path = '/jtec/' + f;
    py.FS.mkdirTree(path.slice(0, path.lastIndexOf('/')));
    py.FS.writeFile(path, buf);
  }
  say('Booting the PCM…');
  py.runPython(`
import sys
sys.path.insert(0, '/jtec/emulator')
import gui, emu
session = gui.Session(emu.ROM_PATH, threaded=False)

def handle(method, path, query, body, filename):
    data = bytes(body.to_py()) if body is not None else b""
    code, ctype, out = gui.api(session, method, path, query, data, filename)
    return [code, ctype, out.decode("utf-8")]

def run_slice(n):
    return session.slice(n)
`);
  handle = py.globals.get('handle');
  slice = py.globals.get('run_slice');
  postMessage({type: 'ready'});
  schedule();
}

// Keep each slice short so requests are answered promptly.
let budget = 20000;
tick.port1.onmessage = () => {
  scheduled = false;
  while (queue.length) answer(queue.shift());
  const t0 = performance.now();
  const ran = slice(budget);            // instructions run; 0 when paused or held back by the speed limit
  const dt = performance.now() - t0;
  if (ran >= budget) budget = Math.max(2000, Math.min(200000, Math.round(budget * (30 / Math.max(dt, 1)))));
  if (ran > 0) schedule(); else setTimeout(schedule, 10);
};

function answer(m) {
  let res;
  try {
    const out = handle(m.method, m.path, m.query, m.body || null, m.filename || 'upload.bin');
    res = out.toJs();
    out.destroy();
  } catch (e) {
    res = [500, 'application/json', JSON.stringify({error: String(e)})];
  }
  postMessage({type: 'reply', id: m.id, code: res[0], ctype: res[1], text: res[2]});
}

onmessage = e => {
  queue.push(e.data);
  if (handle) schedule();
};

boot().catch(e => postMessage({type: 'failed', text: String(e && e.message || e)}));
