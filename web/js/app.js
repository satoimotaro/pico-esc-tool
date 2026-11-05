// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// app — the page: connect, read/edit/write ESC settings, presets, motor test, firmware flash.
// Works with anything that speaks Betaflight's MSP + BLHeli 4-way passthrough: the pico-esc-tool
// firmware or a Betaflight flight controller.

import { SerialPipe, SocketPipe, Msp, MSP, FEATURE_3D, FourWay, MODE } from './link.js';
import { readEsc, writeSettings, flashCheck, flash } from './esc.js';
import { parseHex } from './hex.js';
import { presetFamily } from './layouts.js';
import * as presets from './presets.js';

const $ = (s, el = document) => el.querySelector(s);
const h = (tag, attrs = {}, ...kids) => {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') el.className = v;
    else if (k.startsWith('on')) el.addEventListener(k.slice(2), v);
    else if (v !== false && v != null) el.setAttribute(k, v === true ? '' : v);
  }
  for (const c of kids.flat()) if (c != null) el.append(c.nodeType ? c : document.createTextNode(c));
  return el;
};
const hex2 = (v) => v.toString(16).padStart(2, '0');

const st = {
  pipe: null, msp: null, fw: null,
  inPassthrough: false, motors: 0, threeD: false, board: '',
  escs: [],            // results of readEsc, by channel (null = not answering)
  edits: [],           // per channel: Uint8Array being edited
  busy: false,
  test: { on: false, values: [], timer: null },
};

// ------------------------------------------------------------------------------- log / status
function log(msg, cls = '') {
  const line = h('div', { class: cls }, `${new Date().toLocaleTimeString()}  ${msg}`);
  $('#log').append(line);
  $('#log').scrollTop = 1e9;
}
function status() {
  $('#connect').textContent = st.pipe ? '切断' : '接続';
  $('#conn').textContent = st.pipe
    ? `接続中: ${st.board} / モーター ${st.motors} 個${st.threeD ? ' / 3D' : ''}${st.inPassthrough ? ' / パススルー中' : ''}`
    : '未接続';
  document.body.classList.toggle('connected', !!st.pipe);
  document.body.classList.toggle('busy', st.busy);
}
async function task(what, fn) {
  if (st.busy) return;
  st.busy = true; status();
  try { return await fn(); } catch (e) { log(`${what}: ${e.message}`, 'err'); } finally { st.busy = false; status(); }
}

// ------------------------------------------------------------------------------- connection
async function connect() {
  await task('接続', async () => {
    const ws = new URLSearchParams(location.search).get('ws');
    const pipe = ws ? new SocketPipe(ws) : new SerialPipe();
    if (!ws && !SerialPipe.supported()) throw new Error('このブラウザは Web Serial に対応していません (Chrome / Edge を使ってください)');
    await pipe.open();
    pipe.onclose = () => { if (st.pipe === pipe) dropped(); };
    st.pipe = pipe; st.msp = new Msp(pipe); st.fw = new FourWay(pipe);
    try { await identify(pipe); } catch (e) {
      st.pipe = null;                  // not a board we can talk to: give the port back
      await pipe.close();
      throw e;
    }
  });
  if (!st.pipe) return;
  status();
}

async function identify(pipe) {
  await new Promise((r) => setTimeout(r, 300));
  pipe.drain();
  let api;
  for (let i = 0; i < 3 && !api; i++) api = await st.msp.request(MSP.API_VERSION).catch(() => null);
  if (!api) throw new Error('MSP の応答がありません (ポートやボーレートを確認)');
  const variant = String.fromCharCode(...await st.msp.request(MSP.FC_VARIANT));
  const board = String.fromCharCode(...(await st.msp.request(MSP.BOARD_INFO)).slice(0, 4));
  st.board = `${variant} ${board}`.trim();
  const feat = await st.msp.request(MSP.FEATURE_CONFIG).catch(() => [0, 0, 0, 0]);
  st.threeD = !!((feat[0] | feat[1] << 8 | feat[2] << 16 | feat[3] << 24) & FEATURE_3D);
  const mc = await st.msp.request(MSP.MOTOR_CONFIG).catch(() => null);
  if (mc && mc.length > 6) st.motors = mc[6];
  else {
    const m = await st.msp.request(MSP.MOTOR);
    st.motors = 0;
    for (let i = 0; i < m.length / 2; i++) if (m[2 * i] | m[2 * i + 1] << 8) st.motors++;
  }
  log(`接続: ${st.board}, API ${api[1]}.${api[2]}, モーター ${st.motors} 個${st.threeD ? ' (3D)' : ''}`, 'ok');
  st.pico = board === 'PICO';
  $('#test-proto-box').hidden = !st.pico;
  if (!st.pico) $('#test-proto').value = 'dshot';
  renderTest();
}

function dropped() {
  stopTest('接続が切れました');
  st.pipe = null; st.inPassthrough = false;
  log('切断されました', 'err');
  status();
}

async function disconnect() {
  await stopTest('切断');
  if (st.inPassthrough) await st.fw.exit().catch(() => {});
  const p = st.pipe;
  st.pipe = null; st.inPassthrough = false;
  await p?.close();
  status();
}

async function passthrough(on) {
  if (on === st.inPassthrough) return;
  if (on) {
    await stopTest('パススルーに入るため');
    const d = await st.msp.request(MSP.SET_PASSTHROUGH, [], 3000);
    st.inPassthrough = true;
    log(`パススルー開始 (ESC ${d[0]} 個)`);
    await new Promise((r) => setTimeout(r, 500));
  } else {
    await st.fw.exit();
    st.inPassthrough = false;
    log('パススルー終了 (ESC は通常動作に戻りました)');
    await new Promise((r) => setTimeout(r, 1000));
    st.pipe.drain();
  }
  status();
}

// ------------------------------------------------------------------------------- read / write
async function readAll() {
  await task('読み込み', async () => {
    await passthrough(true);
    st.escs = []; st.edits = [];
    for (let ch = 0; ch < st.motors; ch++) {
      const t0 = performance.now();
      try {
        const esc = await readEsc(st.fw, ch);
        st.escs[ch] = esc; st.edits[ch] = Uint8Array.from(esc.block);
        log(`ESC ${ch + 1}: ${esc.family.name} ${esc.version} ${esc.layoutTag} ${esc.name} (${((performance.now() - t0) / 1000).toFixed(1)} s)`);
      } catch (e) {
        st.escs[ch] = null;
        log(`ESC ${ch + 1}: 応答なし (${e.message})`, 'err');
      }
    }
    renderEscs(); renderFlash();
    if (!st.test.on) renderTest();
  });
}

async function writeEsc(ch) {
  const esc = st.escs[ch];
  await task(`ESC ${ch + 1} 書き込み`, async () => {
    await passthrough(true);
    await st.fw.initFlash(ch);
    await writeSettings(st.fw, esc, st.edits[ch]);
    st.edits[ch] = Uint8Array.from(esc.block);
    log(`ESC ${ch + 1}: 書き込み・照合 OK`, 'ok');
  });
  renderEscs();
}

async function writeSelected() {
  for (const ch of selected()) if (dirty(ch)) await writeEsc(ch);
}

const selected = () => [...document.querySelectorAll('.esc-sel:checked')].map((c) => +c.dataset.ch);
const dirty = (ch) => st.escs[ch] && st.edits[ch].some((v, i) => v !== st.escs[ch].block[i]);

function applyPreset(ch, name) {
  const esc = st.escs[ch];
  const p = presets.all().find((x) => x.name === name);
  if (!esc || !p) return;
  const fam = presetFamily(esc);
  if (fam !== p.family) {
    log(`ESC ${ch + 1}: ${p.name} は ${presets.FAMILIES[p.family]} 用です (この ESC は ${fam ? presets.FAMILIES[fam] : esc.family.name})`, 'err');
    return;
  }
  for (const [o, v] of p.bytes) st.edits[ch][o] = v;
  log(`ESC ${ch + 1}: プリセット ${p.name} を反映しました (まだ書き込んでいません)`);
  renderEscs();
}

function savePreset(ch) {
  const esc = st.escs[ch];
  const fam = presetFamily(esc);
  if (!fam) { log(`ESC ${ch + 1}: この設定マップはプリセットにできません`, 'err'); return; }
  const name = prompt('プリセット名 (a-z 0-9 _ -)', '');
  if (!name) return;
  try {
    presets.save(presets.capture(name, fam, st.edits[ch], prompt('メモ (モーター名など)', '') || ''));
    log(`プリセット ${name} を保存しました`, 'ok');
    renderPresets(); renderEscs();
  } catch (e) { log(e.message, 'err'); }
}

// ------------------------------------------------------------------------------- ESC cards
function fieldInput(ch, f) {
  const edit = st.edits[ch], base = st.escs[ch].block;
  const v = f.get ? f.get(edit) : edit[f.off];
  const set = (x) => { if (f.put) f.put(edit, x); else edit[f.off] = x; renderEscs(); };
  let input;
  if (f.type === 'enum') {
    const known = f.opts.some(([k]) => k === v);
    input = h('select', { onchange: (e) => set(+e.target.value) },
      ...f.opts.map(([k, label]) => h('option', { value: k, selected: k === v }, label)),
      known ? null : h('option', { value: v, selected: true }, `不明 (0x${hex2(v)})`));
  } else if (f.type === 'bool') {
    input = h('input', { type: 'checkbox', checked: v === 1, onchange: (e) => set(e.target.checked ? 1 : 0) });
  } else {
    input = h('input', { type: 'number', min: f.min, max: f.max, value: v,
      onchange: (e) => set(Math.max(0, Math.min(255, +e.target.value | 0))) });
  }
  const was = f.get ? f.get(base) : base[f.off];
  return h('label', { class: `field${v !== was ? ' changed' : ''}`, title: f.help || '' },
    h('span', { class: 'fname' }, f.label),
    input,
    f.fmt ? h('span', { class: 'fmt' }, f.fmt(v)) : null);
}

function renderEscs() {
  const box = $('#escs');
  box.replaceChildren();
  if (!st.escs.length) { box.append(h('p', { class: 'hint' }, '「ESC を読み込む」を押すと、つながっている ESC の設定を読みます。')); return; }
  st.escs.forEach((esc, ch) => {
    if (!esc) { box.append(h('div', { class: 'card off' }, h('h3', {}, `ESC ${ch + 1}`), h('p', {}, '応答なし'))); return; }
    const fam = presetFamily(esc);
    const usable = presets.all().filter((p) => p.family === fam);
    const card = h('div', { class: `card${dirty(ch) ? ' dirty' : ''}` },
      h('div', { class: 'card-head' },
        h('label', {}, h('input', { type: 'checkbox', class: 'esc-sel', 'data-ch': ch, checked: true }), h('h3', {}, `ESC ${ch + 1}`)),
        h('span', { class: 'tag' }, `${esc.family.name} ${esc.version}`),
        h('span', { class: 'meta' }, [esc.layoutTag, esc.mcu, esc.name].filter(Boolean).join(' · '))),
      esc.family.fields.length
        ? h('div', { class: 'fields' }, ...esc.family.fields.map((f) => fieldInput(ch, f)))
        : h('p', { class: 'warn' }, 'この設定マップは知らないため、編集できません (読み取りのみ)。'),
      h('div', { class: 'card-foot' },
        h('select', { class: 'preset-pick', onchange: (e) => { if (e.target.value) applyPreset(ch, e.target.value); } },
          h('option', { value: '' }, usable.length ? 'プリセットを反映…' : '使えるプリセットなし'),
          ...usable.map((p) => h('option', { value: p.name }, `${p.name} — ${p.note}`))),
        h('button', { onclick: () => savePreset(ch), disabled: !fam }, '今の値をプリセットに保存'),
        h('button', { onclick: () => { st.edits[ch] = Uint8Array.from(esc.block); renderEscs(); }, disabled: !dirty(ch) }, '元に戻す'),
        h('button', { class: 'primary', onclick: () => writeEsc(ch), disabled: !dirty(ch) }, '書き込む')),
      h('details', {}, h('summary', {}, '生データ'), h('pre', {}, hexdump(st.edits[ch], esc.block))));
    box.append(card);
  });
}

function hexdump(bytes, base) {
  let s = '';
  for (let r = 0; r < bytes.length; r += 16) {
    s += r.toString(16).padStart(3, '0') + ': ';
    for (let i = r; i < Math.min(r + 16, bytes.length); i++) s += hex2(bytes[i]) + (bytes[i] !== base[i] ? '*' : ' ');
    s += '\n';
  }
  return s;
}

// ------------------------------------------------------------------------------- presets tab
function renderPresets() {
  const box = $('#preset-list');
  box.replaceChildren();
  for (const p of presets.all()) {
    box.append(h('div', { class: 'preset' },
      h('div', {}, h('b', {}, p.name), ' ', h('span', { class: 'tag' }, presets.FAMILIES[p.family]),
        p.builtin ? h('span', { class: 'tag dim' }, '組み込み') : null),
      h('div', { class: 'meta' }, p.note),
      h('details', {}, h('summary', {}, `${p.bytes.length} バイト`),
        h('pre', {}, p.bytes.map(([o, v]) => `${hex2(o)}:${hex2(v)}`).join(' '))),
      p.builtin ? null : h('button', { onclick: () => { presets.remove(p.name); renderPresets(); renderEscs(); } }, '削除')));
  }
}

function exportPresets() {
  const blob = new Blob([presets.exportJson()], { type: 'application/json' });
  const a = h('a', { href: URL.createObjectURL(blob), download: 'esc-presets.json' });
  a.click();
  URL.revokeObjectURL(a.href);
}

async function importPresets(file) {
  try {
    const n = presets.importJson(await file.text());
    log(`プリセットを ${n} 件読み込みました`, 'ok');
  } catch (e) { log(`プリセットの読み込み: ${e.message}`, 'err'); }
  renderPresets(); renderEscs();
}

// ------------------------------------------------------------------------------- motor test
// MSP_SET_MOTOR goes out every 100 ms while the test is on: the pico-esc-tool firmware (like a
// cautious FC) stops the motors when MSP goes quiet, so a closed tab or a dead link stops them.
// PWM (pico-esc-tool only) sends the same slider values as CLI lines, `pwmch <motor> <value>`, also
// every 100 ms: the firmware maps each to that ESC's own pulse (a 3D ESC stops at 1500 us) and falls
// back to the stop pulse after 500 ms without one. SET_MOTOR is not sent then: it would re-arm DShot.
// Each slider stops where ITS ESC stops: 1500 on a 3D ESC (up forward, down reverse), 1000 on a
// one-way one. On pico-esc-tool the page knows that per motor from the ESCs it read, and tells the
// firmware (`mtmode <mask>`) so the values mean the same thing on both sides; a Betaflight FC has one
// 3D setting for every motor (FEATURE_3D), so there it is that.
function motor3d(i) {   // from the last read; undefined = not read
  const e = st.escs[i];
  if (!e) return undefined;
  return e.mode === MODE.ARM_BLB ? e.block[18] === 1 : (e.block[0x0B] === 3 || e.block[0x0B] === 4);
}
const is3d = (i) => (st.pico ? motor3d(i) ?? st.threeD : st.threeD);
const stopOf = (i) => (is3d(i) ? 1500 : 1000);
const unread = () => st.pico && Array.from({ length: st.motors }, (_, i) => st.escs[i]).some((e) => e === undefined);

function renderTest() {
  const box = $('#sliders');
  box.replaceChildren();
  st.test.values = Array.from({ length: 8 }, (_, i) => stopOf(i));
  $('#test-all').value = 0;
  for (let i = 0; i < st.motors; i++) {
    const stop = stopOf(i);
    const out = h('output', {}, String(stop));
    const known = !st.pico || motor3d(i) !== undefined;
    box.append(h('label', { class: 'slider' }, h('span', {}, `M${i + 1}`),
      h('input', { type: 'range', min: 1000, max: 2000, step: 1, value: stop, 'data-m': i, disabled: !st.test.on,
        oninput: (e) => { st.test.values[i] = +e.target.value; out.textContent = e.target.value; } }),
      out,
      h('span', { class: known ? 'tag' : 'tag dim' }, (is3d(i) ? '3D (1500 停止)' : '正転のみ (1000 停止)') + (known ? '' : ' 未読込'))));
  }
  $('#test-note').textContent = (unread() ? '開始すると、先に ESC を読んで 3D かどうかを確かめます。' : '')
    + '少しずつ動かしてください。3D の ESC は 1500 が停止で、上が正転、下が逆転です。'
    + (pwmMode() ? ' PWM: 切り替え後 5 秒は停止パルスで待機状態に入れます。テレメトリは出ません。Bluejay は PWM を受け付けません。' : '');
}

const pwmMode = () => st.pico && $('#test-proto').value === 'pwm';
const sendLine = (line) => st.pipe.write(Array.from(new TextEncoder().encode(line + '\n')));

async function startTest() {
  if (!$('#test-safe').checked) { log('プロペラを外したことを確認してください', 'err'); return; }
  if (unread()) { log('3D かどうかを確かめるため、先に ESC を読み込みます'); await readAll(); }
  await task('モーターテスト', async () => {
    await passthrough(false);
    if (st.pico) {
      let mask = 0;
      for (let i = 0; i < st.motors; i++) if (is3d(i)) mask |= 1 << i;
      await sendLine(`mtmode ${mask}`);
    }
    st.test.on = true;
    st.test.pwm = pwmMode();
    $('#test-proto').disabled = true;
    renderTest();
    if (st.test.pwm) {
      st.test.timer = setInterval(() => {
        if (!st.pipe || !st.test.on) return;
        for (let i = 0; i < st.motors; i++) sendLine(`pwmch ${i} ${st.test.values[i]}`).catch(() => {});
      }, 100);
      log('モーターテスト開始 (PWM)');
      return;
    }
    // One SET_MOTOR in flight at a time. The pico-esc-tool firmware answers nothing while an arm
    // reads an ESC it does not know yet (a bootloader visit, up to ~4 s each, one ESC after another),
    // so only a link silent for longer than that counts as lost. The firmware stops the motors on
    // its own when MSP goes quiet.
    let inflight = false, heard = performance.now();
    st.test.timer = setInterval(() => {
      if (!st.pipe || !st.test.on || inflight) return;
      if (performance.now() - heard > 10000) { stopTest('ボードの応答が 10 秒ありません'); return; }
      const d = [];
      for (const v of st.test.values) d.push(v & 0xFF, v >> 8);
      inflight = true;
      st.msp.request(MSP.SET_MOTOR, d, 1000)
        .then(() => { heard = performance.now(); }, () => {})
        .finally(() => { inflight = false; });
    }, 100);
    log('モーターテスト開始');
  });
  $('#test-toggle').textContent = st.test.on ? '停止' : '開始';
}

async function stopTest(why) {
  if (!st.test.on) return;
  st.test.on = false;
  clearInterval(st.test.timer);
  st.test.values = st.test.values.map((_, i) => stopOf(i));
  $('#test-proto').disabled = false;
  if (st.pipe && st.test.pwm) {
    for (let i = 0; i < st.motors; i++) await sendLine(`pwmch ${i} stop`).catch(() => {});
  } else if (st.pipe) {
    const d = [];
    for (const v of st.test.values) d.push(v & 0xFF, v >> 8);
    await st.msp.request(MSP.SET_MOTOR, d, 500).catch(() => {});
  }
  renderTest();
  $('#test-toggle').textContent = '開始';
  log(`モーターテスト停止: ${why}`);
}

// ------------------------------------------------------------------------------- telemetry
// MSP_MOTOR_TELEMETRY (Betaflight's layout): per motor rpm u32, invalid % x100 u16, temp u8,
// volts x100 u16, amps x100 u16, mAh u16. Polled while the motor-test tab is open.
const SERIES = ['#2563eb', '#d97706', '#059669', '#db2777', '#7c3aed', '#0891b2', '#65a30d', '#dc2626'];
const tele = { timer: null, hist: [], span: 15000 };

function parseTele(d) {
  const out = [];
  for (let i = 0; i < d[0]; i++) {
    const q = d.slice(1 + 13 * i, 14 + 13 * i);
    if (q.length < 13) break;
    out.push({ rpm: (q[0] | q[1] << 8 | q[2] << 16) + q[3] * 0x1000000, bad: (q[4] | q[5] << 8) / 100,
      temp: q[6], volt: (q[7] | q[8] << 8) / 100, amp: (q[9] | q[10] << 8) / 100 });
  }
  return out;
}

async function pollTele() {
  if (!st.pipe || st.inPassthrough || st.busy || tele.inflight) return;
  let rows;
  tele.inflight = true;
  try { rows = parseTele(await st.msp.request(MSP.MOTOR_TELEMETRY, [], 1000)); } catch (e) { rows = null; } finally { tele.inflight = false; }
  if (!rows && st.test.on) return;   // busy arming: keep the last table
  const body = $('#tele tbody');
  body.replaceChildren();
  const now = performance.now();
  if (!rows) { body.append(h('tr', {}, h('td', { colspan: 7 }, 'このボードはテレメトリを返しません'))); return; }
  rows.forEach((r, i) => {
    const ok = r.bad < 100;
    body.append(h('tr', {},
      h('td', {}, h('span', { class: 'swatch', style: `background:${SERIES[i % 8]}` }), `M${i + 1}`),
      h('td', {}, String(st.test.values[i] ?? '')),
      h('td', { class: 'rpm' }, ok ? String(r.rpm) : '—'),
      h('td', {}, r.volt ? r.volt.toFixed(2) : '—'),
      h('td', {}, r.amp ? r.amp.toFixed(2) : '—'),
      h('td', {}, r.temp ? String(r.temp) : '—'),
      h('td', { class: ok ? 'ok' : 'meta' }, ok ? '受信中' : '停止中 / 信号なし')));
  });
  tele.hist.push({ t: now, rpm: rows.map((r) => (r.bad < 100 ? r.rpm : 0)) });
  while (tele.hist.length && now - tele.hist[0].t > tele.span) tele.hist.shift();
  drawChart(rows.length);
}

function drawChart(n) {
  const cv = $('#rpm-chart');
  const w = cv.clientWidth, hgt = cv.clientHeight, dpr = devicePixelRatio || 1;
  cv.width = w * dpr; cv.height = hgt * dpr;
  const g = cv.getContext('2d');
  g.scale(dpr, dpr);
  const ink = getComputedStyle(document.body).color;
  const max = Math.max(500, ...tele.hist.flatMap((p) => p.rpm)) * 1.1;
  const now = performance.now();
  const x = (t) => 40 + (w - 50) * (1 - (now - t) / tele.span), y = (v) => hgt - 20 - (hgt - 30) * v / max;
  g.font = '11px system-ui'; g.fillStyle = ink; g.globalAlpha = 0.6;
  for (let k = 0; k <= 4; k++) {
    const v = max * k / 4;
    g.fillText(String(Math.round(v / 10) * 10), 2, y(v) + 4);
    g.globalAlpha = 0.12; g.fillRect(40, y(v), w - 50, 1); g.globalAlpha = 0.6;
  }
  g.fillText('rpm / 直近 15 秒', 44, 12);
  g.globalAlpha = 1; g.lineWidth = 2;
  for (let i = 0; i < n; i++) {
    g.strokeStyle = SERIES[i % 8];
    g.beginPath();
    tele.hist.forEach((p, k) => (k ? g.lineTo(x(p.t), y(p.rpm[i] || 0)) : g.moveTo(x(p.t), y(p.rpm[i] || 0))));
    g.stroke();
  }
  $('#rpm-legend').replaceChildren(...Array.from({ length: n }, (_, i) =>
    h('span', {}, h('span', { class: 'swatch', style: `background:${SERIES[i % 8]}` }), `M${i + 1}`)));
}

function teleRun(on) {
  clearInterval(tele.timer);
  tele.timer = on ? setInterval(pollTele, 200) : null;
}

// ------------------------------------------------------------------------------- flash tab
let hexMem = null, hexName = '';

function renderFlash() {
  const box = $('#flash-escs');
  box.replaceChildren();
  if (!st.escs.length) { box.append(h('p', { class: 'hint' }, '先に「ESC 設定」タブで ESC を読み込んでください。')); return; }
  st.escs.forEach((esc, ch) => {
    if (!esc) return;
    const chk = hexMem ? flashCheck(esc, hexMem) : null;
    box.append(h('div', { class: 'flash-row' },
      h('label', {}, h('input', { type: 'checkbox', class: 'flash-sel', 'data-ch': ch, checked: !!chk && !chk.problems.length }),
        ` ESC ${ch + 1}: ${esc.family.name} ${esc.version} ${esc.layoutTag} ${esc.mcu}`),
      chk ? h('div', { class: chk.problems.length ? 'warn' : 'ok' },
        chk.problems.length ? `⚠ ${chk.problems.join(' / ')}` : `✓ 適合 (${chk.hexFamily ? chk.hexFamily.name + ' ' + chk.hexVersion : chk.hexLayout || 'AM32'})`,
        ...chk.notes.map((n) => h('div', { class: 'meta' }, n))) : null,
      h('progress', { id: `prog-${ch}`, max: 1, value: 0 })));
  });
}

async function loadHex(file) {
  try {
    hexMem = parseHex(await file.text());
    hexName = file.name;
    log(`HEX 読み込み: ${file.name} (${hexMem.size} バイト)`);
  } catch (e) { hexMem = null; log(`HEX: ${e.message}`, 'err'); }
  renderFlash();
}

async function flashSelected() {
  if (!hexMem) { log('HEX ファイルを選んでください', 'err'); return; }
  const chans = [...document.querySelectorAll('.flash-sel:checked')].map((c) => +c.dataset.ch);
  if (!chans.length) return;
  const force = $('#flash-force').checked;
  if (!confirm(`${hexName} を ESC ${chans.map((c) => c + 1).join(', ')} に書き込みます。途中で電源を切らないでください。`)) return;
  await task('書き込み', async () => {
    await passthrough(true);
    for (const ch of chans) {
      const esc = st.escs[ch];
      const fresh = await readEsc(st.fw, ch);        // the state right now, not at the last read
      const chk = flashCheck(fresh, hexMem);
      if (chk.problems.length && !force) { log(`ESC ${ch + 1}: 中止 — ${chk.problems.join(' / ')}`, 'err'); continue; }
      const bar = $(`#prog-${ch}`);
      const t0 = performance.now();
      const kept = await flash(st.fw, fresh, hexMem, {
        migrate: $('#flash-migrate').checked,
        progress: (f) => { if (bar) bar.value = f; },
      });
      await st.fw.reset(ch).catch(() => {});
      log(`ESC ${ch + 1}: 書き込み完了 ${((performance.now() - t0) / 1000).toFixed(0)} s, 引き継ぎ: ${kept}`, 'ok');
      Object.assign(esc, await readEsc(st.fw, ch));
      st.edits[ch] = Uint8Array.from(esc.block);
    }
  });
  renderEscs(); renderFlash();
}

// ------------------------------------------------------------------------------- wiring
function tabs() {
  document.querySelectorAll('nav button').forEach((b) => b.addEventListener('click', () => {
    document.querySelectorAll('nav button').forEach((x) => x.classList.toggle('active', x === b));
    document.querySelectorAll('main > section').forEach((s) => { s.hidden = s.id !== b.dataset.tab; });
    if (b.dataset.tab !== 'test') stopTest('タブを離れた');
    teleRun(b.dataset.tab === 'test');
  }));
}

function init() {
  tabs();
  $('#connect').addEventListener('click', () => (st.pipe ? disconnect() : connect()));
  $('#read').addEventListener('click', readAll);
  $('#write-sel').addEventListener('click', writeSelected);
  $('#exit-pt').addEventListener('click', () => task('パススルー終了', () => passthrough(false)));
  $('#test-toggle').addEventListener('click', () => (st.test.on ? stopTest('停止ボタン') : startTest()));
  $('#test-proto').addEventListener('change', renderTest);
  $('#test-all').addEventListener('input', (e) => {   // 0..1000 forward, from each motor's own stop
    const p = +e.target.value;
    document.querySelectorAll('#sliders input').forEach((s) => {
      const i = +s.dataset.m;
      s.value = stopOf(i) + (is3d(i) ? p / 2 : p); s.dispatchEvent(new Event('input'));
    });
  });
  $('#hexfile').addEventListener('change', (e) => e.target.files[0] && loadHex(e.target.files[0]));
  $('#flash-go').addEventListener('click', flashSelected);
  $('#preset-export').addEventListener('click', exportPresets);
  $('#preset-import').addEventListener('change', (e) => e.target.files[0] && importPresets(e.target.files[0]));
  document.addEventListener('keydown', (e) => { if (e.key === 'Escape' || (e.key === ' ' && st.test.on)) { e.preventDefault(); stopTest('キー操作'); } });
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopTest('ページが隠れた'); });
  window.addEventListener('beforeunload', () => { stopTest('ページを閉じた'); });
  if (!SerialPipe.supported() && !new URLSearchParams(location.search).get('ws')) {
    $('#conn').textContent = 'このブラウザは Web Serial に対応していません。PC の Chrome か Edge で開いてください。';
  }
  renderEscs(); renderPresets(); renderFlash(); renderTest(); status();
  window.escweb = { st, presets };   // for the console and headless tests
}

init();
