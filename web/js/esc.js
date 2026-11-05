// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc — read, write and flash one ESC through the 4-way interface.
//
// SiLabs (BLHeli_S / Bluejay / BlueGill): the settings live in the 512-byte page at 0x1A00 with the
// startup tune; a write erases that page, so the whole page is read first and written back whole.
// Flashing follows the order the BLHeli tools use so a power cut mid-way leaves a bootable ESC:
// mark the settings page failed, park an LJMP-to-bootloader on page 1, write pages 2..12, then 0..1,
// then the settings page last.
// AM32: the settings block is 192 bytes at 0x7C00 / 0xF800 (by the bootloader's flash-size code), the
// firmware name 32 bytes below it. The bootloader erases a page when a write lands on its first
// byte, and rewrites byte 2 with its own version on every write.

import { MODE } from './link.js';
import { SILABS, AM32_MAP, silabsFamily, am32Family } from './layouts.js';
import { slice, ascii } from './hex.js';

export async function readEsc(fw, ch) {
  const info = await fw.initFlash(ch);
  const esc = { ch, info, mode: info.mode };
  if (info.mode === MODE.SIL_BLB) {
    esc.page = await fw.read(SILABS.eeprom, SILABS.pageSize);
    esc.block = esc.page.slice(0, SILABS.size);
    esc.family = silabsFamily(esc.block);
    esc.layoutTag = ascii(esc.block.slice(SILABS.layoutTag, SILABS.layoutTag + 16));
    esc.mcu = ascii(esc.block.slice(SILABS.mcuTag, SILABS.mcuTag + 16));
    esc.name = ascii(esc.block.slice(SILABS.name, SILABS.name + 16));
    esc.version = `${esc.block[0]}.${esc.block[1]}`;
  } else if (info.mode === MODE.ARM_BLB) {
    const code = info.signature >> 8;
    esc.eeprom = AM32_MAP.offsets[code];
    if (esc.eeprom === undefined) throw new Error(`AM32: unknown flash-size code 0x${code.toString(16)}`);
    esc.block = await fw.read(esc.eeprom, AM32_MAP.size);
    esc.family = am32Family(esc.block);
    esc.name = ascii(await fw.read(esc.eeprom - AM32_MAP.nameLen, 16));
    esc.layoutTag = esc.name;
    esc.mcu = `flash code 0x${code.toString(16)}`;
    esc.version = `${esc.block[3]}.${esc.block[4]}`;
  } else {
    throw new Error(`interface mode ${info.mode} (Atmel?) is not supported`);
  }
  return esc;
}

function same(a, b, skip = new Set()) {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) if (!skip.has(i) && a[i] !== b[i]) return false;
  return true;
}

// Write a settings block (esc.block layout) back. Returns what reads back.
export async function writeSettings(fw, esc, block) {
  if (esc.mode === MODE.SIL_BLB) {
    const page = Uint8Array.from(esc.page);
    page.set(block.slice(0, SILABS.size), 0);
    await fw.erasePage(SILABS.eepromPage);
    await fw.write(SILABS.eeprom, page);
    const back = await fw.read(SILABS.eeprom, SILABS.pageSize);
    if (!same(back, page)) throw new Error('read-back differs from what was written');
    esc.page = back; esc.block = back.slice(0, SILABS.size);
    return esc.block;
  }
  if (block[0] !== 0x01 || block.length !== AM32_MAP.size) throw new Error('AM32: refusing a block that does not start 0x01');
  await fw.write(esc.eeprom, block);
  const back = await fw.read(esc.eeprom, AM32_MAP.size);
  if (!same(back, block, new Set([2]))) throw new Error('read-back differs from what was written');
  esc.block = back;
  return back;
}

// --- flashing ---------------------------------------------------------------------------------

// What flashing `mem` would do to this ESC, before anything is written
export function flashCheck(esc, mem) {
  const out = { problems: [], notes: [] };
  if (esc.mode === MODE.SIL_BLB) {
    if (!SILABS.flashable.has(esc.info.signature)) {
      out.problems.push(`MCU 0x${esc.info.signature.toString(16)} はこのツールが書き込める 512 バイトページ品ではありません`);
    }
    const img = slice(mem, SILABS.eeprom, SILABS.eeprom + SILABS.size);
    out.hexLayout = ascii(img.slice(SILABS.layoutTag, SILABS.layoutTag + 16));
    out.hexMcu = ascii(img.slice(SILABS.mcuTag, SILABS.mcuTag + 16));
    out.hexFamily = silabsFamily(img);
    out.hexVersion = `${img[0]}.${img[1]}`;
    if (!out.hexLayout) out.problems.push('HEX の 0x1A40 にレイアウト名がありません (BLHeli_S / Bluejay 用ではない)');
    else if (out.hexLayout !== esc.layoutTag) out.problems.push(`レイアウト不一致: HEX ${out.hexLayout} / ESC ${esc.layoutTag}`);
    if (out.hexMcu !== esc.mcu) out.problems.push(`MCU 不一致: HEX ${out.hexMcu || '(なし)'} / ESC ${esc.mcu}`);
    if ([...mem.keys()].some((a) => a >= SILABS.bootloader && a < 0x2000)) out.notes.push('HEX 内のブートローダー部分は書きません');
  } else {
    const nameAt = esc.eeprom - AM32_MAP.nameLen;
    const keys = [...mem.keys()];
    if (Math.min(...keys) < AM32_MAP.appStart) {
      out.problems.push('AM32 用の HEX ではありません (0x1000 より下、ブートローダー領域に書き込む内容です)');
      return out;
    }
    if (keys.some((a) => a >= esc.eeprom)) out.notes.push('HEX 内の設定領域は書きません (ESC の設定はそのまま)');
    out.hexLayout = ascii(slice(mem, nameAt, nameAt + 16));
    if (!out.hexLayout) out.notes.push('HEX にファーム名がありません (ESC の名前は元のまま)');
    else if (out.hexLayout !== esc.name) out.problems.push(`ターゲット不一致: HEX ${out.hexLayout} / ESC ${esc.name}`);
  }
  return out;
}

async function verify(fw, addr, data) {
  const back = await fw.read(addr, data.length);
  if (!same(back, data)) {
    const i = back.findIndex((b, k) => b !== data[k]);
    throw new Error(`verify failed at 0x${(addr + i).toString(16)}`);
  }
}

// migrate: keep the old settings (and tune) when the new image has the same settings layout
export async function flash(fw, esc, mem, { migrate = true, progress = () => {} } = {}) {
  if (esc.mode === MODE.ARM_BLB) return flashAm32(fw, esc, mem, progress);
  const P = SILABS.pageSize;
  const image = slice(mem, 0, SILABS.bootloader);
  const old = esc.page;
  const fresh = image.slice(SILABS.eeprom, SILABS.eeprom + P);
  let kept = 'なし (ファームの初期値)';
  if (migrate) {
    if (old[0] === fresh[0] && old[1] === fresh[1] && old[2] === fresh[2]) {
      fresh.set(old.slice(0x03, 0x40), 0x03);   // settings
      fresh.set(old.slice(0x70, 0x100), 0x70);  // startup tune
      kept = '全設定と起動音';
    } else {
      for (const o of [0x0B, 0x1B, 0x1C, 0x1D]) fresh[o] = old[o];   // direction, beeps: same on every layout
      kept = '回転方向とビープ設定のみ (設定レイアウトが違うため)';
    }
  }
  const pages = SILABS.eepromPage;          // app pages 0..12
  const steps = pages + 3;
  let step = 0;
  const tick = (what) => progress(++step / steps, what);

  // 1. settings page: mark it so a half-done flash is visible
  const guard = Uint8Array.from(old.slice(0, SILABS.size));
  guard.set(Array.from('**FLASH*FAILED**', (c) => c.charCodeAt(0)), SILABS.name);
  await fw.erasePage(SILABS.eepromPage);
  await fw.write(SILABS.eeprom, guard);
  tick('settings page marked');
  // 2. LJMP bootloader at page 1 (reached from the old reset vector while page 0 is intact)
  const ljmp = [0x02, SILABS.bootloader >> 8, SILABS.bootloader & 0xFF];
  await fw.erasePage(1);
  await fw.write(P, ljmp);
  await verify(fw, P, Uint8Array.from(ljmp));
  tick('bootloader failsafe');
  // 3. pages 2..12, then 4. pages 0..1
  for (const pg of [...Array.from({ length: pages - 2 }, (_, i) => i + 2), 0, 1]) {
    const data = image.slice(pg * P, pg * P + P);
    await fw.erasePage(pg);
    if (data.some((b) => b !== 0xFF)) {
      await fw.write(pg * P, data);
      await verify(fw, pg * P, data);
    }
    tick(`page ${pg}`);
  }
  // 5. settings page
  await fw.erasePage(SILABS.eepromPage);
  await fw.write(SILABS.eeprom, fresh);
  await verify(fw, SILABS.eeprom, fresh);
  tick('settings');
  return kept;
}

async function flashAm32(fw, esc, mem, progress) {
  const nameAt = esc.eeprom - AM32_MAP.nameLen;
  const addrs = [...mem.keys()].filter((a) => a >= AM32_MAP.appStart && a < nameAt + AM32_MAP.nameLen);
  if (!addrs.length) throw new Error('the HEX has no application bytes');
  let end = Math.max(...addrs) + 1;
  end += end % 2;
  const data = slice(mem, AM32_MAP.appStart, end);
  // 256-byte writes from 0x1000: every page start is a chunk start, so each page is erased once
  for (let o = 0; o < data.length; o += 256) {
    await fw.write(AM32_MAP.appStart + o, data.slice(o, o + 256));
    progress((o + 256) / data.length / 2, `0x${(AM32_MAP.appStart + o).toString(16)}`);
  }
  for (let o = 0; o < data.length; o += 1024) {
    await verify(fw, AM32_MAP.appStart + o, data.slice(o, o + 1024));
    progress(0.5 + (o + 1024) / data.length / 2, 'verify');
  }
  return '全設定 (設定領域は書き換えません)';
}
