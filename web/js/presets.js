// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// presets — named ESC settings for one motor, the same set the firmware's `preset` command carries
// (src/apps/presets.h; host/tests/test_web_presets.py keeps the two equal). A preset is
// (offset, value) bytes in one firmware family's settings block; identity bytes are never in one.
// Where each built-in came from, and what it did on the bench: docs/motor-settings.md.
// Presets you save here live in this browser (localStorage) and export/import as JSON.

export const FAMILIES = { bluejay: 'Bluejay 0.21', bluegill: 'BlueGill', am32: 'AM32' };

const BUILTIN_SRC = [
  { name: 'f2838', family: 'bluejay', note: 'F2838 350KV 12N14P, one-way (potted pod, 2026-10-23)',
    bytes: '03:ff 04:1e 05:01 06:ff 07:19 08:ff 09:00 0a:18 0b:01 0c:ff 0d:55 0e:aa 0f:ff 10:ff 11:ff 12:ff 13:ff 14:ff 15:04 16:ff 17:ff 18:ff 19:ff 1a:ff 1b:28 1c:50 1d:04 1e:ff 1f:03 20:ff 21:ff 22:ff 23:00 24:ff 25:ff 26:ff 27:00 28:00 29:02 2a:00 2b:ff 2c:ff 2d:ff 2e:ff 2f:ff 30:ff 31:ff 32:ff 33:ff 34:ff 35:ff 36:ff 37:ff 38:ff 39:ff 3a:ff 3b:ff 3c:ff 3d:ff 3e:ff 3f:ff' },
  { name: 'f2838-3d', family: 'bluejay', note: 'F2838 350KV / 930KV 12N14P, bidirectional',
    bytes: '03:ff 04:1e 05:01 06:ff 07:19 08:ff 09:00 0a:18 0b:03 0c:ff 0d:55 0e:aa 0f:ff 10:ff 11:ff 12:ff 13:ff 14:ff 15:04 16:ff 17:ff 18:ff 19:ff 1a:ff 1b:28 1c:50 1d:04 1e:ff 1f:03 20:ff 21:ff 22:ff 23:00 24:ff 25:ff 26:ff 27:00 28:00 29:02 2a:00 2b:ff 2c:ff 2d:ff 2e:ff 2f:ff 30:ff 31:ff 32:ff 33:ff 34:ff 35:ff 36:ff 37:ff 38:ff 39:ff 3a:ff 3b:ff 3c:ff 3d:ff 3e:ff 3f:ff' },
  { name: 'sine-3d', family: 'bluegill', note: 'BlueGill S4 forced-sine start, bidirectional',
    bytes: '03:ff 04:1e 05:01 06:ff 07:19 08:ff 09:00 0a:18 0b:03 0c:ff 0d:55 0e:aa 0f:ff 10:ff 11:ff 12:ff 13:ff 14:ff 15:04 16:ff 17:ff 18:ff 19:ff 1a:ff 1b:28 1c:50 1d:04 1e:ff 1f:03 20:ff 21:ff 22:ff 23:00 24:ff 25:ff 26:ff 27:00 28:00 29:02 2a:00 2b:00 2c:00 2d:00 2e:02 2f:18 30:3c 31:10 32:24 33:ef 34:ff 35:ff 36:ff 37:ff 38:ff 39:ff 3a:ff 3b:ff 3c:ff 3d:ff 3e:ff 3f:ff' },
  { name: 'f2838-am32', family: 'am32', note: 'F2838 350KV on AM32, one-way (Aria, 2026-11-03)',
    bytes: '05:a0 06:01 07:00 08:0a 09:64 0a:00 0b:64 0c:00 0d:00 11:00 12:00 13:01 14:01 15:01 16:01 17:1a 18:18 19:96 1a:08 1b:0e 1c:00 1d:00 1e:05 1f:00 20:80 21:80 22:80 23:32 24:00 25:32 26:00 27:00 28:05 29:0a 2a:0a 2b:ff 2c:ff 2d:05 2e:01 2f:00' },
  { name: 'f2838-am32-3d', family: 'am32', note: 'F2838 350KV on AM32, bidirectional',
    bytes: '05:a0 06:01 07:00 08:0a 09:64 0a:00 0b:64 0c:00 0d:00 11:00 12:01 13:01 14:01 15:01 16:01 17:1a 18:18 19:96 1a:08 1b:0e 1c:00 1d:00 1e:05 1f:00 20:80 21:80 22:80 23:32 24:00 25:32 26:00 27:00 28:05 29:0a 2a:0a 2b:ff 2c:ff 2d:05 2e:01 2f:00' },
];

const parse = (s) => s.split(' ').map((p) => p.split(':').map((x) => parseInt(x, 16)));
export const BUILTIN = BUILTIN_SRC.map((p) => ({ ...p, bytes: parse(p.bytes), builtin: true }));

const KEY = 'esc-web.presets';
const NAME = /^[a-z0-9_-]{1,24}$/;

export function saved() {
  try { return JSON.parse(localStorage.getItem(KEY) || '[]'); } catch (e) { return []; }
}
function store(list) {
  try { localStorage.setItem(KEY, JSON.stringify(list)); } catch (e) { /* private window */ }
}
export function all() { return [...BUILTIN, ...saved()]; }

export function check(p) {
  if (!p || !NAME.test(p.name || '')) return 'name must be [a-z0-9_-], 1..24 characters';
  if (!FAMILIES[p.family]) return `unknown family ${p.family}`;
  if (!Array.isArray(p.bytes) || !p.bytes.length || p.bytes.length > 64) return 'bytes: 1..64 [offset, value] pairs';
  const lo = p.family === 'am32' ? 5 : 0x03, hi = p.family === 'am32' ? 47 : 0x3F;
  for (const [o, v] of p.bytes) {
    if (!(o >= lo && o <= hi) || !(v >= 0 && v <= 255)) return `byte ${o}=${v} outside the settings range`;
  }
  return null;
}

export function save(p) {
  const err = check(p);
  if (err) throw new Error(err);
  if (BUILTIN.some((b) => b.name === p.name)) throw new Error(`${p.name} is a built-in`);
  store([...saved().filter((s) => s.name !== p.name), { name: p.name, family: p.family, note: p.note || '', bytes: p.bytes }]);
}
export function remove(name) { store(saved().filter((s) => s.name !== name)); }

export function importJson(text) {
  const list = JSON.parse(text);
  let n = 0;
  for (const p of Array.isArray(list) ? list : [list]) { save(p); n++; }
  return n;
}
export function exportJson(list = saved()) { return JSON.stringify(list, null, 1); }

// The bytes a preset would capture from a settings block
export function capture(name, family, block, note = '') {
  const lo = family === 'am32' ? 5 : 0x03, hi = family === 'am32' ? 47 : 0x3F;
  const skip = family === 'am32' ? new Set([14, 15, 16]) : new Set();
  const bytes = [];
  for (let o = lo; o <= hi; o++) if (!skip.has(o)) bytes.push([o, block[o]]);
  return { name, family, note, bytes };
}
