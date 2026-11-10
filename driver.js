// Driver view: the emulated PCM as a car person would want to see it.
// Everything here is presentation over data files that may be missing or
// partial (gauges.json, faults.json, outputs.json, tables_view.json,
// sensor_curves.json, scenarios.json); each panel says so when it has nothing.
(function () {
  const D = {gauges: null, faults: null, outputs: null, tables: null, sensors: null, scenarios: null, connectors: null};
  const el = (tag, cls, html) => { const e = document.createElement(tag); if (cls) e.className = cls; if (html != null) e.innerHTML = html; return e; };
  let built = false, curTable = null, lastCar = null;
  const lastGood = {};      // last in-range converter value per sensor, restored when a fault is cleared

  // ---- unit conversion through a sampled sensor curve ----------------------
  function interp(points, x, from, to) {
    // points: [[adc, value], ...]; map along column `from` to column `to`.
    const p = [...points].sort((a, b) => a[from] - b[from]);
    if (x <= p[0][from]) return p[0][to];
    for (let i = 1; i < p.length; i++) {
      if (x <= p[i][from]) {
        const span = p[i][from] - p[i - 1][from];
        const t = span ? (x - p[i - 1][from]) / span : 0;
        return p[i - 1][to] + t * (p[i][to] - p[i - 1][to]);
      }
    }
    return p[p.length - 1][to];
  }
  const fmt = (v, digits) => v == null || isNaN(v) ? '–' : (Math.abs(v) >= 100 || digits === 0 ? Math.round(v).toLocaleString() : v.toFixed(digits == null ? 1 : digits));

  // ---- build static structure once the data files are in --------------------
  function build() {
    const root = $('driver');
    root.innerHTML = '';
    const tabs = el('div', 'tablist dtabs');
    tabs.setAttribute('role', 'tablist');
    const panes = {};
    for (const [id, label] of [['dash', 'Dashboard'], ['maps', 'Maps & limits'], ['conn', 'Connectors']]) {
      const b = el('button', '', label);
      b.setAttribute('role', 'tab'); b.id = 'dtab-' + id; b.dataset.pane = id;
      b.setAttribute('aria-selected', id === 'dash');
      b.onclick = () => { for (const k in panes) { panes[k].hidden = k !== id; $('dtab-' + k).setAttribute('aria-selected', k === id); } };
      tabs.appendChild(b);
      panes[id] = el('div', 'dpane'); panes[id].hidden = id !== 'dash'; panes[id].id = 'dpane-' + id;
      panes[id].setAttribute('role', 'tabpanel');
    }
    root.appendChild(tabs);
    for (const k in panes) root.appendChild(panes[k]);

    // Dashboard ---------------------------------------------------------------
    const dash = panes.dash;
    const left = el('div', 'dcol'), right = el('div', 'dcol');
    dash.appendChild(left); dash.appendChild(right);

    const ctl = el('section', '', '<h2>Controls</h2>');
    ctl.appendChild(el('div', 'row', '<label><input type="checkbox" id="dIgn"> Ignition on</label>' +
      '<span id="dRpmWrap" hidden><label for="dRpm">Engine speed</label> <input type="range" id="dRpm" min="0" max="7000" step="50" value="0"> <span class="mono" id="dRpmV">0 rpm</span></span>'));
    ctl.appendChild(el('div', 'row', '<span id="dSpdWrap" hidden><label for="dGear">Gear</label> <select id="dGear" aria-label="Gear">' +
      '<option value="0">not linked</option><option>1</option><option>2</option><option>3</option><option>4</option><option>5</option><option>6</option><option value="R">R</option></select>' +
      ' <label for="dSpd">Road speed</label> <input type="range" id="dSpd" min="0" max="300" step="1" value="0"> <span class="mono" id="dSpdV">0 km/h</span></span>'));
    ctl.appendChild(el('div', 'row', '', ));
    const scen = el('div', 'row'); scen.id = 'dScen'; ctl.appendChild(scen);
    const sens = el('div', ''); sens.id = 'dSens'; ctl.appendChild(sens);
    ctl.appendChild(el('p', 'note', 'Sensor sliders set the voltage on the PCM pin that the firmware treats as that sensor; the units are what the firmware itself then reports. Engine speed drives a synthetic crank and cam signal; road speed drives the speed-sensor input, either directly or from engine speed through the selected gear (T56 ratios, 3.07 axle, 335/35 ZR17 rear tyres).'));
    left.appendChild(ctl);

    const g = el('section', '', '<h2>Live values</h2><div id="dGauges"></div>');
    left.appendChild(g);
    const gauges = (D.gauges && D.gauges.gauges) || [];
    if (!gauges.length) $('dGauges') || 0;
    const groups = {};
    for (const x of gauges) (groups[x.group || 'other'] = groups[x.group || 'other'] || []).push(x);
    const gbox = g.querySelector('#dGauges');
    if (!gauges.length) gbox.innerHTML = '<p class="note">No gauge definitions yet (gauges.json).</p>';
    for (const [name, list] of Object.entries(groups)) {
      gbox.appendChild(el('div', 'ghead', esc(name)));
      const grid = el('div', 'ggrid');
      for (const x of list) {
        const c = el('div', 'gauge' + (x.raw ? ' raw' : ''),
          '<div class="gname">' + esc(x.name) + (x.raw ? ' <span class="tag">raw</span>' : '') + '</div>' +
          '<div class="gval"><span id="gv-' + x.id + '">–</span> <small>' + esc(x.unit || '') + '</small></div>' +
          '<div class="gbar"><i id="gb-' + x.id + '"></i></div>');
        c.title = (x.confidence || '') + (x.evidence ? ' — ' + x.evidence : '') + ' [' + x.address + ']';
        grid.appendChild(c);
      }
      gbox.appendChild(grid);
    }

    right.appendChild(el('section', '', '<h2>Stored fault codes</h2><div id="dFaults"></div>' +
      '<div class="row" style="margin-top:8px"><button id="dClear" title="Sends SCI command 17 repeatedly, as a scan tool does">Erase codes</button><span class="note" id="dClearMsg" style="margin:0"></span></div>' +
      '<p class="note">With no engine signal and stand-in hardware the PCM logs faults a real car would not. Treat them as a demonstration of the fault logic.</p>'));
    right.appendChild(el('section', '', '<h2>What the PCM is commanding</h2><div id="dOutputs" class="lights"></div>' +
      '<p class="note">A lit lamp means the firmware is requesting that load. Many loads are switched by the co-processor, so this is the request, not proof the relay closed.</p>'));

    // Sensors in real units
    const sensors = (D.sensors && D.sensors.sensors) || [];
    const sbox = ctl.querySelector('#dSens');
    if (!sensors.length) sbox.innerHTML = '<p class="note">No sensor curves yet (sensor_curves.json). Use Engineer view, Inputs tab, to set pin voltages.</p>';
    for (const sdef of sensors) {
      const row = el('div', 'srow', '<label for="ds-' + sdef.id + '">' + esc(sdef.name) + '</label>' +
        '<input type="range" id="ds-' + sdef.id + '" min="' + sdef.min + '" max="' + sdef.max + '" step="' + (sdef.step || 1) + '">' +
        '<span class="mono" id="dsv-' + sdef.id + '"></span>' +
        (sdef.faults ? '<select id="dsf-' + sdef.id + '" aria-label="' + esc(sdef.name) + ' circuit">' +
          '<option value="ok">circuit OK</option><option value="0">0 V: short to ground</option>' +
          '<option value="5">5 V: open or short to 5 V</option></select>' : ''));
      sbox.appendChild(row);
      const send = adc => fetch('/api/cmd', {method: 'POST', headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({cmd: 'analog', set: sdef.input.set, ch: sdef.input.ch, value: adc})});
      row.querySelector('input').oninput = e => {
        const v = +e.target.value, adc = Math.round(interp(sdef.points, v, 1, 0));
        $('dsv-' + sdef.id).textContent = fmt(v, sdef.digits) + ' ' + sdef.unit;
        lastGood[sdef.id] = adc;
        cmdQuiet({cmd: 'analog', set: sdef.input.set, ch: sdef.input.ch, value: adc});
      };
      const sel = row.querySelector('select');
      if (sel) sel.onchange = () => {
        if (sel.value === '0') send(0);
        else if (sel.value === '5') send(1023);
        else send(lastGood[sdef.id] != null ? lastGood[sdef.id] : 512);
        poll();
      };
    }
    const kc = el('div', 'row', '<button id="dKey" title="Ignition off, then on: the PCM restarts and forgets substituted sensor values">Key off / on</button>' +
      '<button id="dWarm" title="Advance the PCM run timer past the point where the delayed monitors start">Skip 4 min of running</button>' +
      '<span class="note" style="margin:0">After a circuit fault the PCM keeps using a default value for that sensor until the next key cycle, even once the circuit is OK again.</span>');
    sbox.appendChild(kc);
    kc.querySelector('#dKey').onclick = () => cmd({cmd: 'reset'});
    kc.querySelector('#dWarm').onclick = () => cmd({cmd: 'skip_warmup'});
    sbox.appendChild(el('p', 'note', 'Some checks only run after the engine has been running about 4 minutes with coolant at 82 °C or more: the oxygen sensor shorted-to-voltage checks are among them. The skip button jumps the PCM\'s run timer there. Most circuit faults take tens of seconds to mature before a code is stored.'));
    const scenarios = (D.scenarios && D.scenarios.scenarios) || [];
    for (const sc of scenarios) {
      const b = el('button', '', esc(sc.name)); b.title = sc.description || '';
      b.onclick = () => applyScenario(sc);
      scen.appendChild(b);
    }
    ctl.querySelector('#dIgn').onchange = e => cmd({cmd: 'ignition', on: e.target.checked});
    ctl.querySelector('#dSpd').oninput = e => { $('dSpdV').textContent = e.target.value + ' km/h'; cmdQuiet({cmd: 'speed', value: +e.target.value}); };
    ctl.querySelector('#dGear').onchange = e => { const v = e.target.value; cmd({cmd: 'gear', value: v === 'R' ? 'R' : +v}); };
    ctl.querySelector('#dRpm').oninput = e => { $('dRpmV').textContent = (+e.target.value).toLocaleString() + ' rpm'; cmdQuiet({cmd: 'rpm', value: +e.target.value}); };
    right.querySelector('#dClear').onclick = async () => {
      $('dClearMsg').textContent = 'sending…';
      const r = await fetch('/api/cmd', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({cmd: 'erase_codes'})});
      $('dClearMsg').textContent = r.ok ? 'requested' : (await r.json()).error;
    };

    // Maps ---------------------------------------------------------------------
    const maps = panes.maps;
    const tv = D.tables || {};
    const list = el('section', 'tlist', '<h2>Tables</h2>'), view = el('section', 'tview', '<p class="note" style="margin:0">Pick a table.</p>');
    view.id = 'dTable';
    maps.appendChild(list); maps.appendChild(view);
    const tables = tv.tables || [];
    if (!tables.length) list.appendChild(el('p', 'note', 'No table data yet (tables_view.json).'));
    if ((tv.scalars || []).length) {
      const ul = el('ul', 'tl'); const x = el('li', '', '<b>Limits and thresholds</b>'); x.tabIndex = 0; x.dataset.id = '__scalars';
      x.onclick = () => showScalars(false); x.onkeydown = e => { if (e.key === 'Enter') showScalars(false); };
      ul.appendChild(x); list.appendChild(ul);
    }
    const tg = {};
    for (const t of tables) (tg[t.group || 'other'] = tg[t.group || 'other'] || []).push(t);
    const order = ['spark', 'fuel', 'limits', 'idle', 'feedback', 'transmission', 'other'];
    for (const name of Object.keys(tg).sort((a, b) => order.indexOf(a) - order.indexOf(b))) {
      const arr = tg[name];
      const det = el('details', 'tgroup', '<summary>' + esc(name) + ' <span class="tag">' + arr.length + '</span></summary>');
      if (name === 'spark' || name === 'fuel' || name === 'limits') det.open = true;
      const ul = el('ul', 'tl');
      for (const kind of ['map', 'curve']) for (const t of arr.filter(x => x.kind === kind)) {
        const li = el('li', '', (kind === 'map' ? '▦ ' : '∿ ') + esc(t.name)); li.tabIndex = 0; li.dataset.id = t.id; li.title = t.kind;
        li.onclick = () => showTable(t); li.onkeydown = e => { if (e.key === 'Enter') showTable(t); };
        ul.appendChild(li);
      }
      det.appendChild(ul); list.appendChild(det);
    }
    if (tables.length) showTable(tables.find(t => t.id === 'base_spark_a') || tables[0]);

    // Connectors ---------------------------------------------------------------
    const conn = panes.conn;
    const cs = (D.connectors && D.connectors.connectors) || [];
    if (!cs.length) conn.appendChild(el('p', 'note', 'No connector data found.'));
    for (const c of cs) {
      const sec = el('section', 'connector', '<h2>PCM connector ' + esc(c.name) + ' (' + esc(c.color) + ') · manual page ' + esc(c.printedPage || '') + '</h2>');
      const byCav = {}; for (const p of c.pins) byCav[p.cavity] = p;
      const grid = el('div', 'cav');
      // Drawn as in the manual: 22-32 on top, 12-21 in the middle, 1-11 at the bottom.
      for (const row of [[22, 32], [12, 21], [1, 11]]) {
        const r = el('div', 'cavrow');
        for (let n = row[0]; n <= row[1]; n++) {
          const p = byCav[n];
          const cell = el('div', 'c' + (p ? '' : ' empty'), '<b>' + n + '</b><span>' + esc(p ? p.function : '') + '</span><em id="cv-' + c.name + '-' + n + '"></em>');
          cell.title = p ? c.name + '-' + n + ' ' + p.circuit + ': ' + p.function : c.name + '-' + n + ' not used';
          r.appendChild(cell);
        }
        grid.appendChild(r);
      }
      sec.appendChild(grid);
      conn.appendChild(sec);
    }
    conn.appendChild(el('p', 'note', 'Cavities are from the 1997 service manual. A live value appears only where the firmware signal has been tied to that cavity (see INPUT_MAP.md); a question mark means the tie is an inference.'));
    built = true;
  }

  function applyScenario(sc) {
    // The server converts units through the sensor curves and key-cycles the PCM.
    cmd({cmd: 'scenario', id: sc.id});
  }

  // ---- tables -----------------------------------------------------------------
  const heat = t => 'hsl(' + Math.round(215 - 195 * t) + ' 65% ' + (matchMedia('(prefers-color-scheme: dark)').matches ? 30 : 78) + '%)';
  const axisTitle = ax => esc(ax.label || '') + (ax.unit ? ' (' + esc(ax.unit) + ')' : ' (raw)');
  function showTable(t) {
    curTable = t;
    document.querySelectorAll('#dpane-maps .tl li').forEach(li => li.classList.toggle('sel', li.dataset.id === t.id));
    const v = $('dTable');
    const bits = 8 * (t.cell_bytes || 1);
    const sgn = x => t.cell_signed && x >= 2 ** (bits - 1) ? x - 2 ** bits : x;
    const sc = x => sgn(x) * (t.cell_scale == null ? 1 : t.cell_scale) + (t.cell_offset || 0);
    const flat = t.cells.flat().map(sgn), lo = Math.min(...flat), hi = Math.max(...flat);
    const unit = t.cell_unit || (t.cell_label ? t.cell_label + ', raw' : 'raw');
    let h = '<h2>' + esc(t.name) + '</h2><p class="note" style="margin:0 0 8px">' + esc(t.notes || '') + '</p>';
    if (t.kind === 'curve' || t.cells.length === 1) {
      const xs = t.x.values, ys = t.cells[0].map(sc), W = 560, H = 220, P = 40;
      const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
      const px = x => P + (x1 > x0 ? (x - x0) / (x1 - x0) : 0.5) * (W - 2 * P), py = y => H - P - (y1 > y0 ? (y - y0) / (y1 - y0) : 0.5) * (H - 2 * P);
      const step = t.lookup === 'step';
      const pts = [];
      xs.forEach((x, k) => { if (step && k) pts.push(px(x).toFixed(1) + ',' + py(ys[k - 1]).toFixed(1)); pts.push(px(x).toFixed(1) + ',' + py(ys[k]).toFixed(1)); });
      h += '<svg viewBox="0 0 ' + W + ' ' + H + '" class="curve" role="img" aria-label="' + esc(t.name) + '">' +
        '<polyline fill="none" stroke="currentColor" stroke-width="2" points="' + pts.join(' ') + '"/>' +
        xs.map((x, k) => '<circle cx="' + px(x).toFixed(1) + '" cy="' + py(ys[k]).toFixed(1) + '" r="3"><title>' + fmt(x) + ' → ' + fmt(ys[k]) + '</title></circle>').join('') +
        '<line id="curveCursor" x1="0" x2="0" y1="' + P / 2 + '" y2="' + (H - P) + '" class="cursor" visibility="hidden"/>' +
        '<text x="' + P + '" y="' + (H - 10) + '">' + fmt(x0) + '</text><text x="' + (W - P) + '" y="' + (H - 10) + '" text-anchor="end">' + fmt(x1) + '</text>' +
        '<text x="2" y="' + (H - P) + '">' + fmt(y0) + '</text><text x="2" y="' + (P / 2 + 8) + '">' + fmt(y1) + '</text></svg>';
      h += '<table class="scalars"><tr><th>' + axisTitle(t.x) + '</th>' + xs.map(x => '<td class="mono">' + fmt(x) + '</td>').join('') + '</tr><tr><th>' + esc(unit) + '</th>' + ys.map(y => '<td class="mono">' + fmt(y) + '</td>').join('') + '</tr></table>';
      h += '<p class="note">' + (t.x.live_address ? 'The dashed line is where the engine is now.' : 'No live cursor: ' + esc(t.x.live_evidence || 'the firmware variable behind the axis is not identified') + '.') + '</p>';
      v._px = px;
    } else {
      h += '<div class="heatwrap"><table class="heat"><tr><th>' + axisTitle(t.y) + ' \\ ' + axisTitle(t.x) + '</th>' + t.x.values.map(x => '<th>' + fmt(x) + '</th>').join('') + '</tr>';
      t.cells.forEach((row, r) => {
        h += '<tr><th>' + fmt(t.y.values[r]) + '</th>' + row.map((c, ci) => '<td id="hc-' + r + '-' + ci + '" style="background:' + heat(hi > lo ? (sgn(c) - lo) / (hi - lo) : 0.5) + '">' + fmt(sc(c)) + '</td>').join('') + '</tr>';
      });
      h += '</table></div><p class="note">Cells in ' + esc(unit) + '. ' + (t.x.live_address && t.y.live_address ? 'The outlined cell is the one nearest to where the engine is now.' : 'No live cursor: the firmware variable behind one or both axes is not identified.') + '</p>';
    }
    h += '<p class="note">ROM ' + esc(t.table_address || t.rom_address) + ' · ' + esc(t.confidence || '') + ' · ' + esc(t.evidence || '') + '</p>';
    v.innerHTML = h;
    updateCursor();
  }
  function showScalars(all) {
    curTable = null;
    document.querySelectorAll('#dpane-maps .tl li').forEach(li => li.classList.toggle('sel', li.dataset.id === '__scalars'));
    const list = (D.tables.scalars || []).filter(x => all || x.headline);
    const byGroup = {};
    for (const x of list) (byGroup[x.group || 'other'] = byGroup[x.group || 'other'] || []).push(x);
    let h = '<h2>Limits and thresholds</h2><div class="row"><label><input type="checkbox" id="scAll"' + (all ? ' checked' : '') + '> Show all ' + (D.tables.scalars || []).length + ' single values, not just the headline ones</label></div>';
    for (const [g, arr] of Object.entries(byGroup)) {
      h += '<div class="ghead">' + esc(g) + '</div><table class="scalars"><tr><th>Setting</th><th>Value</th><th>ROM</th><th>Notes</th></tr>' +
        arr.map(x => '<tr><td>' + esc(x.name) + '</td><td class="mono" style="white-space:nowrap">' + (x.unit ? fmt(x.value) + ' ' + esc(x.unit) : fmt(x.value, 0) + ' <span class="tag">raw</span>') + '</td><td class="mono">' + esc(x.rom_address) + '</td><td>' + esc(x.notes || '') + '</td></tr>').join('') + '</table>';
    }
    $('dTable').innerHTML = h;
    $('scAll').onchange = e => showScalars(e.target.checked);
  }
  // Fractional position along an axis from the firmware's own lookup input.
  function axisPos(ax, live) {
    if (!ax || !ax.live_address || live[ax.live_address] == null) return null;
    const v = live[ax.live_address] / (ax.live_divisor || 1), lv = ax.live_values || ax.raw;
    if (!lv || !lv.length) return null;
    const up = lv[lv.length - 1] >= lv[0];
    if (up ? v <= lv[0] : v >= lv[0]) return 0;
    for (let k = 1; k < lv.length; k++) {
      if (up ? v <= lv[k] : v >= lv[k]) return k - 1 + (lv[k] !== lv[k - 1] ? (v - lv[k - 1]) / (lv[k] - lv[k - 1]) : 0);
    }
    return lv.length - 1;
  }
  function updateCursor() {
    const t = curTable, live = lastCar && lastCar.live;
    if (!t || !live) return;
    document.querySelectorAll('.heat td.now').forEach(c => c.classList.remove('now'));
    const xp = axisPos(t.x, live);
    if (t.kind === 'curve' || t.cells.length === 1) {
      const line = document.getElementById('curveCursor'), v = $('dTable');
      if (line && xp != null && v._px) {
        const k = Math.floor(xp), f = xp - k, xs = t.x.values;
        const x = v._px(xs[k] + (k + 1 < xs.length ? f * (xs[k + 1] - xs[k]) : 0));
        line.setAttribute('x1', x); line.setAttribute('x2', x); line.setAttribute('visibility', 'visible');
      }
      return;
    }
    const yp = axisPos(t.y, live);
    if (xp == null || yp == null) return;
    const c = document.getElementById('hc-' + Math.round(yp) + '-' + Math.round(xp));
    if (c) c.classList.add('now');
  }

  // ---- live update ----------------------------------------------------------------
  window.renderDriver = function (s) {
    if (!built || $('driver').hidden) return;
    const car = s.car || {}; lastCar = car;
    for (const x of (D.gauges && D.gauges.gauges) || []) {
      const v = car.gauges && car.gauges[x.id];
      const n = $('gv-' + x.id); if (!n) continue;
      n.textContent = v ? fmt(v[1], x.digits) : '–';
      const span = (x.max - x.min) || 1;
      $('gb-' + x.id).style.width = (v ? Math.max(0, Math.min(1, (v[1] - x.min) / span)) * 100 : 0) + '%';
    }
    const F = (D.faults && D.faults.faults) || {};
    const ids = car.faults || [];
    $('dFaults').innerHTML = ids.length ? ids.map(id => {
      const f = F[id] || {};
      return '<div class="fault"><span class="fcode">' + esc(f.pcode || '—') + (f.blink ? ' · flash ' + esc(f.blink) : '') + '</span><span class="fname">' + esc(f.name || 'Not identified') +
        '</span><span class="fid" title="' + esc((f.detail || '') + ' ' + (f.confidence || '')) + '">internal ' + id + '</span></div>';
    }).join('') : '<p class="ok">No stored codes.</p>';
    const O = (D.outputs && D.outputs.outputs) || [];
    const lampHtml = list => list.map(o => {
      const on = car.outputs && car.outputs[o.id];
      const tip = esc((o.cavity ? o.cavity + ' · ' : '') + (o.path || '') + ' · ' + (o.confidence || ''));
      if (o.kind === 'value') return '<div class="lamp val" title="' + tip + '"><b class="mono">' + (on == null ? '–' : on) + '</b>' + esc(o.name) + '</div>';
      return '<div class="lamp ' + (on ? 'on' : '') + (o.group === 'fuel cut' ? ' cut' : '') + '" title="' + tip + '"><i></i>' + esc(o.name) + '</div>';
    }).join('');
    const cuts = O.filter(o => o.group === 'fuel cut'), loads = O.filter(o => o.group !== 'fuel cut');
    $('dOutputs').innerHTML = O.length ? (cuts.length ? '<div class="ghead" style="grid-column:1/-1">Fuel cut</div>' + lampHtml(cuts) +
      '<div class="ghead" style="grid-column:1/-1">Relays, lamps and solenoids</div>' : '') + lampHtml(loads) : '<p class="note">No output map yet (outputs.json).</p>';
    if (document.activeElement !== $('dIgn')) $('dIgn').checked = s.ignition;
    $('dRpmWrap').hidden = !s.engine_model;
    if (s.engine_model && document.activeElement !== $('dRpm')) { $('dRpm').value = s.rpm_setting || 0; $('dRpmV').textContent = (s.rpm_setting || 0).toLocaleString() + ' rpm'; }
    $('dSpdWrap').hidden = !s.speed_model;
    if (s.speed_model) {
      const linked = s.gear && s.gear !== 0;
      if (document.activeElement !== $('dGear')) $('dGear').value = String(s.gear || 0);
      $('dSpd').disabled = !!linked;
      $('dSpd').title = linked ? 'Follows engine speed in the selected gear' : '';
      if (document.activeElement !== $('dSpd')) $('dSpd').value = Math.round(s.road_speed);
      $('dSpdV').textContent = Math.round(s.road_speed) + ' km/h' + (linked ? ' (from rpm)' : '');
    }
    for (const sdef of (D.sensors && D.sensors.sensors) || []) {
      const inp = $('ds-' + sdef.id); if (!inp) continue;
      const adc = (sdef.input.set === 'b' ? s.analog_b : s.analog)[sdef.input.ch];
      // A pin at a rail reads as a circuit fault rather than a value.
      const fault = adc <= 1 ? '0' : adc >= 1022 ? '5' : 'ok';
      const sel = $('dsf-' + sdef.id);
      if (sel && document.activeElement !== sel) sel.value = fault;
      inp.disabled = fault !== 'ok' && sdef.unit !== 'V';
      if (fault === 'ok' && lastGood[sdef.id] == null) lastGood[sdef.id] = adc;
      if (document.activeElement === inp) continue;
      const v = interp(sdef.points, adc, 0, 1);
      inp.value = v;
      $('dsv-' + sdef.id).textContent = fault !== 'ok' && sdef.unit !== 'V' ? (adc * 5 / 1023).toFixed(2) + ' V' : fmt(v, sdef.digits) + ' ' + sdef.unit;
    }
    for (const [cav, text] of Object.entries(car.cavities || {})) { const n = $('cv-' + cav); if (n) n.textContent = text; }
    updateCursor();
  };

  window.setViewMode = function (mode) {
    const driver = mode === 'driver';
    $('driver').hidden = !driver;
    document.querySelector('main').hidden = driver;
    $('modeDriver').setAttribute('aria-pressed', String(driver));
    $('modeEngineer').setAttribute('aria-pressed', String(!driver));
    try { localStorage.setItem('emu-mode', mode); } catch (e) {}
  };

  window.loadDriverData = async function () {
    for (const k of Object.keys(D)) { try { D[k] = await api('/api/data?name=' + k); } catch (e) { D[k] = {}; } }
    build();
  };
})();
