// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// hex — Intel HEX to a sparse byte map, addresses as the ESC's bootloader sees them (ARM images are
// linked at 0x08000000; the AM32 bootloader takes offsets from there).

export function parseHex(text) {
  const mem = new Map();
  let upper = 0;
  for (const raw of text.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line) continue;
    if (line[0] !== ':') throw new Error(`not an Intel HEX line: ${line.slice(0, 20)}`);
    const rec = [];
    for (let i = 1; i < line.length; i += 2) rec.push(parseInt(line.slice(i, i + 2), 16));
    if (rec.reduce((a, b) => a + b, 0) & 0xFF) throw new Error(`bad checksum: ${line.slice(0, 20)}`);
    const [n, aH, aL, type] = rec;
    const data = rec.slice(4, 4 + n);
    if (type === 0) {
      const base = upper + (aH << 8 | aL);
      data.forEach((b, k) => mem.set(base + k, b));
    } else if (type === 2) {
      upper = (data[0] << 8 | data[1]) << 4;
    } else if (type === 4) {
      upper = (data[0] << 8 | data[1]) * 0x10000;
    } else if (type === 1) {
      break;
    }
  }
  if (!mem.size) throw new Error('HEX has no data');
  // ARM images: rebase to the flash start
  const min = Math.min(...mem.keys());
  if (min >= 0x08000000) {
    const m = new Map();
    for (const [a, b] of mem) m.set(a - 0x08000000, b);
    return m;
  }
  return mem;
}

// [start, end) of a map as bytes, 0xFF where the HEX has nothing
export function slice(mem, start, end) {
  const out = new Uint8Array(end - start).fill(0xFF);
  for (const [a, b] of mem) if (a >= start && a < end) out[a - start] = b;
  return out;
}

export function ascii(bytes) {
  let s = '';
  for (const b of bytes) { if (b === 0 || b === 0xFF) break; s += String.fromCharCode(b); }
  return s.trim();
}
