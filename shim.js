// Published-site glue: the page was written for the local gui.py server, so
// every fetch to /api/... is handed to the in-browser emulator (worker.js)
// instead. Other requests (driver.js, driver.css) go to the network as usual.
(function () {
  const worker = new Worker('worker.js');
  const pending = new Map();
  let nextId = 0, resolveReady, failed = null;
  const ready = new Promise(r => { resolveReady = r; });
  const realFetch = window.fetch.bind(window);

  function overlay(text, isError) {
    let o = document.getElementById('bootOverlay');
    if (!o) return;
    o.querySelector('span').textContent = text;
    o.classList.toggle('err', !!isError);
  }

  worker.onmessage = e => {
    const m = e.data;
    if (m.type === 'progress') overlay(m.text);
    else if (m.type === 'ready') { const o = document.getElementById('bootOverlay'); if (o) o.remove(); resolveReady(); }
    else if (m.type === 'failed') { failed = m.text; overlay('The emulator could not start: ' + m.text, true); }
    else if (m.type === 'reply') {
      const res = pending.get(m.id);
      pending.delete(m.id);
      if (res) res(new Response(m.text, {status: m.code, headers: {'Content-Type': m.ctype}}));
    }
  };

  window.fetch = async function (input, init) {
    init = init || {};
    const url = new URL(typeof input === 'string' ? input : input.url, location.href);
    if (!url.pathname.startsWith('/api/')) return realFetch(input, init);
    await ready;
    if (failed) throw new Error(failed);
    let body = null;
    if (init.body != null) {
      body = init.body instanceof Blob ? new Uint8Array(await init.body.arrayBuffer())
        : new TextEncoder().encode(String(init.body));
    }
    const headers = init.headers || {};
    const filename = headers['X-Filename'] || 'upload.bin';
    return new Promise(res => {
      const id = ++nextId;
      pending.set(id, res);
      worker.postMessage({id, method: init.method || 'GET', path: url.pathname, query: url.search.slice(1), body, filename},
        body ? [body.buffer] : []);
    });
  };
})();
