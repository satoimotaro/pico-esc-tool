// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// link — the byte pipe to the flight controller (or the Pico), and the two protocols spoken over it:
// MSP v1 to find the board and drive the motor test, then the BLHeli 4-way interface after
// MSP_SET_PASSTHROUGH (Betaflight src/main/io/serial_4way.c, protocol 108).
//
// The pipe is Web Serial in the browser. A WebSocket pipe (`?ws=ws://host:port`) exists for driving
// the page headless against a serial bridge in tests; it is not needed to use the tool.

export class Pipe {
  constructor() { this.buf = []; this.waiters = []; this.onclose = null; }
  push(bytes) {
    for (const b of bytes) this.buf.push(b);
    const w = this.waiters; this.waiters = [];
    w.forEach((f) => f());
  }
  // Wait until `pred(buf)` returns a result (not undefined) or `ms` passes.
  async until(pred, ms) {
    const end = performance.now() + ms;
    for (;;) {
      const r = pred(this.buf);
      if (r !== undefined) return r;
      const left = end - performance.now();
      if (left <= 0) return undefined;
      await new Promise((res) => {
        const t = setTimeout(res, left);
        this.waiters.push(() => { clearTimeout(t); res(); });
      });
    }
  }
  drain() { this.buf.length = 0; }
}

export class SerialPipe extends Pipe {
  static supported() { return 'serial' in navigator; }
  async open(baud = 115200) {
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: baud });
    this.writer = this.port.writable.getWriter();
    this.reading = this.readLoop();
  }
  async readLoop() {
    try {
      while (this.port.readable) {
        this.reader = this.port.readable.getReader();
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          this.push(value);
        }
        this.reader.releaseLock();
      }
    } catch (e) { /* unplugged */ }
    if (this.onclose) this.onclose();
  }
  async write(bytes) { await this.writer.write(Uint8Array.from(bytes)); }
  async close() {
    try { await this.reader?.cancel(); } catch (e) { /* already gone */ }
    try { this.writer?.releaseLock(); await this.port?.close(); } catch (e) { /* already gone */ }
  }
}

export class SocketPipe extends Pipe {
  constructor(url) { super(); this.url = url; }
  open() {
    return new Promise((res, rej) => {
      this.ws = new WebSocket(this.url);
      this.ws.binaryType = 'arraybuffer';
      this.ws.onopen = () => res();
      this.ws.onerror = () => rej(new Error(`cannot open ${this.url}`));
      this.ws.onmessage = (e) => this.push(new Uint8Array(e.data));
      this.ws.onclose = () => this.onclose && this.onclose();
    });
  }
  async write(bytes) { this.ws.send(Uint8Array.from(bytes)); }
  async close() { this.ws?.close(); }
}

// ------------------------------------------------------------------------------------- MSP v1
export const MSP = {
  API_VERSION: 1, FC_VARIANT: 2, FC_VERSION: 3, BOARD_INFO: 4, FEATURE_CONFIG: 36,
  MOTOR: 104, BATTERY_STATE: 130, MOTOR_CONFIG: 131, MOTOR_TELEMETRY: 139, SET_MOTOR: 214, SET_PASSTHROUGH: 245,
};
export const FEATURE_3D = 1 << 12;

export class Msp {
  constructor(pipe) { this.pipe = pipe; this.busy = Promise.resolve(); }
  // One request at a time: replies carry no sequence number.
  request(cmd, data = [], ms = 1000) {
    const run = this.busy.then(() => this.send(cmd, data, ms));
    this.busy = run.catch(() => {});
    return run;
  }
  async send(cmd, data, ms) {
    let ck = data.length ^ cmd;
    for (const b of data) ck ^= b;
    await this.pipe.write([0x24, 0x4D, 0x3C, data.length, cmd, ...data, ck]);
    const r = await this.pipe.until((buf) => {
      for (;;) {
        const i = buf.indexOf(0x24);
        if (i < 0) { buf.length = 0; return undefined; }
        if (i > 0) buf.splice(0, i);
        if (buf.length < 6) return undefined;
        if (buf[1] !== 0x4D || (buf[2] !== 0x3E && buf[2] !== 0x21)) { buf.shift(); continue; }
        const n = buf[3];
        if (buf.length < 6 + n) return undefined;
        const frame = buf.splice(0, 6 + n);
        let c = 0;
        for (let k = 3; k < 5 + n; k++) c ^= frame[k];
        if (c !== frame[5 + n] || frame[4] !== cmd) continue;   // stray or stale frame
        return { ok: frame[2] === 0x3E, data: frame.slice(5, 5 + n) };
      }
    }, ms);
    if (!r) throw new Error(`MSP ${cmd}: no reply`);
    if (!r.ok) throw new Error(`MSP ${cmd}: not supported`);
    return r.data;
  }
}

// ------------------------------------------------------------------------------------- 4-way
export const FW = {
  TEST_ALIVE: 0x30, PROTOCOL_VERSION: 0x31, INTERFACE_NAME: 0x32, INTERFACE_VERSION: 0x33,
  EXIT: 0x34, RESET: 0x35, INIT_FLASH: 0x37, PAGE_ERASE: 0x39, READ: 0x3A, WRITE: 0x3B,
};
// Interface modes in DeviceInfo[3]
export const MODE = { SIL_BLB: 1, ATM_BLB: 2, ATM_SK: 3, ARM_BLB: 4 };
const ACK = { 0x00: 'OK', 0x02: 'invalid command', 0x03: 'bad CRC', 0x04: 'verify failed',
  0x08: 'invalid channel', 0x09: 'invalid parameter', 0x0F: 'general error' };

export function crc16(bytes, crc = 0) {
  for (const b of bytes) {
    crc ^= b << 8;
    for (let i = 0; i < 8; i++) crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
  }
  return crc;
}

export class FourWay {
  constructor(pipe) { this.pipe = pipe; this.busy = Promise.resolve(); }
  command(cmd, addr = 0, params = [0], ms = 3000) {
    const run = this.busy.then(() => this.send(cmd, addr, params, ms));
    this.busy = run.catch(() => {});
    return run;
  }
  async send(cmd, addr, params, ms) {
    const f = [0x2F, cmd, (addr >> 8) & 0xFF, addr & 0xFF, params.length & 0xFF, ...params];
    const c = crc16(f);
    this.pipe.drain();
    await this.pipe.write([...f, c >> 8, c & 0xFF]);
    const r = await this.pipe.until((buf) => {
      for (;;) {
        const i = buf.indexOf(0x2E);
        if (i < 0) { buf.length = 0; return undefined; }
        if (i > 0) buf.splice(0, i);
        if (buf.length < 5) return undefined;
        const n = buf[4] || 256;
        if (buf.length < 5 + n + 3) return undefined;
        const frame = buf.slice(0, 5 + n + 3);
        if (frame[1] !== cmd || crc16(frame.slice(0, 5 + n + 1)) !== (frame[6 + n] << 8 | frame[7 + n])) {
          buf.shift(); continue;
        }
        buf.splice(0, frame.length);
        return { ack: frame[5 + n], data: frame.slice(5, 5 + n) };
      }
    }, ms);
    if (!r) throw new Error(`4-way 0x${cmd.toString(16)}: no reply`);
    if (r.ack !== 0) throw new Error(`4-way 0x${cmd.toString(16)}: ${ACK[r.ack] || 'ack ' + r.ack}`);
    return r.data;
  }
  // DeviceInfo: [sigLo, sigHi, bootVersion, mode]
  async initFlash(ch) {
    const d = await this.command(FW.INIT_FLASH, 0, [ch], 20000);
    return { signature: d[1] << 8 | d[0], mode: d[3], raw: d };
  }
  async read(addr, n) {
    const out = [];
    while (out.length < n) {
      const k = Math.min(256, n - out.length);
      const d = await this.command(FW.READ, addr + out.length, [k & 0xFF]);
      out.push(...d.slice(0, k));
    }
    return Uint8Array.from(out);
  }
  async write(addr, bytes) {
    for (let o = 0; o < bytes.length; o += 256) {
      await this.command(FW.WRITE, addr + o, Array.from(bytes.slice(o, o + 256)), 5000);
    }
  }
  erasePage(page) { return this.command(FW.PAGE_ERASE, 0, [page], 5000); }
  reset(ch) { return this.command(FW.RESET, 0, [ch]); }
  exit() { return this.command(FW.EXIT, 0, [0], 5000); }
}
