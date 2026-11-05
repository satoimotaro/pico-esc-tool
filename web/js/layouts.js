// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// layouts — where each firmware keeps its settings, and what the bytes mean.
//   BLHeli_S 16.x (layout 32/33) and Bluejay (layout 200..209) share the block at 0x1A00 on SiLabs;
//   BlueGill (our Bluejay fork, layout 226) appends its sine settings at 0x2B..0x33.
//   AM32 keeps a 192-byte block at the top of flash (host/pico_esc/am32.py, AM32 Inc/eeprom.h).
// Offsets and value tables follow the firmware sources and the esc-configurator field tables.
// A field is {key, off, label, type: 'enum'|'num'|'bool', opts, min, max, fmt, help}.

const opts = (list) => list.map((x, i) => (Array.isArray(x) ? x : [i + 1, x]));

const DIRECTION = { key: 'MOTOR_DIRECTION', off: 0x0B, label: '回転方向', type: 'enum',
  opts: opts(['Normal', 'Reversed', '双方向 (3D)', '双方向 (3D) 逆']) };
const TIMING = { key: 'COMMUTATION_TIMING', off: 0x15, label: '転流タイミング', type: 'enum',
  opts: opts(['0° Low', '7.5° MediumLow', '15° Medium', '22.5° MediumHigh', '30° High']) };
const DEMAG = { key: 'DEMAG_COMPENSATION', off: 0x1F, label: '減磁補償', type: 'enum', opts: opts(['Off', 'Low', 'High']) };
const BEEP = { key: 'BEEP_STRENGTH', off: 0x1B, label: 'ビープ音量', type: 'num', min: 0, max: 255 };
const BEACON = { key: 'BEACON_STRENGTH', off: 0x1C, label: 'ビーコン音量', type: 'num', min: 0, max: 255 };
const BEACON_DELAY = { key: 'BEACON_DELAY', off: 0x1D, label: 'ビーコン開始', type: 'enum',
  opts: opts(['1 分', '2 分', '5 分', '10 分', 'なし']) };
const TEMP = { key: 'TEMPERATURE_PROTECTION', off: 0x23, label: '温度保護', type: 'enum',
  opts: [[0, 'Off'], [1, '80 °C'], [2, '90 °C'], [3, '100 °C'], [4, '110 °C'], [5, '120 °C'], [6, '130 °C'], [7, '140 °C']] };
const BRAKE = { key: 'BRAKE_ON_STOP', off: 0x27, label: '停止時ブレーキ', type: 'bool' };
const LED = { key: 'LED_CONTROL', off: 0x28, label: 'LED', type: 'enum',
  opts: [[0, 'Off'], [0x03, 'Blue'], [0x0C, 'Green'], [0x30, 'Red'], [0x0F, 'Cyan'], [0x33, 'Magenta'], [0x3C, 'Yellow'], [0x3F, 'White']] };

// --- BLHeli_S 16.x ---------------------------------------------------------------------------
const BLHELI_S = [
  { key: 'STARTUP_POWER', off: 0x09, label: '起動出力', type: 'enum',
    opts: opts(['0.031', '0.047', '0.063', '0.094', '0.125', '0.188', '0.25', '0.38', '0.50', '0.75', '1.00', '1.25', '1.50']) },
  DIRECTION, TIMING, DEMAG,
  { ...TEMP, off: 0x23 },
  { key: 'LOW_RPM_POWER_PROTECTION', off: 0x24, label: '低回転時の出力保護', type: 'bool' },
  BRAKE, BEEP, BEACON, BEACON_DELAY,
  { key: 'PROGRAMMING_BY_TX', off: 0x0F, label: '送信機からの設定', type: 'bool' },
  { key: 'PPM_MIN_THROTTLE', off: 0x19, label: 'PWM 最小 (µs)', type: 'num', min: 0, max: 125, fmt: (v) => `${1000 + v * 4} µs` },
  { key: 'PPM_MAX_THROTTLE', off: 0x1A, label: 'PWM 最大 (µs)', type: 'num', min: 126, max: 255, fmt: (v) => `${1000 + v * 4} µs` },
];

// --- Bluejay, by layout revision -------------------------------------------------------------
const SP_MIN = { key: 'STARTUP_POWER_MIN', off: 0x04, label: '起動出力 最小', type: 'num', min: 0, max: 255,
  fmt: (v) => `≈ ${(1000 + v * 1000 / 2047).toFixed(0)}`, help: '起動時に出す最低の出力。重いプロペラや水中では上げる' };
const SP_MAX = { key: 'STARTUP_POWER_MAX', off: 0x07, label: '起動出力 最大', type: 'num', min: 0, max: 255,
  fmt: (v) => `≈ ${1000 + v * 4}`, help: '起動中に許す最大の出力' };
const RPM_SLOPE = { key: 'RPM_POWER_SLOPE', off: 0x09, label: '低回転時の出力制限', type: 'enum',
  opts: [[0, 'Off (制限なし)'], ...Array.from({ length: 13 }, (_, i) => [i + 1, `${i + 1}x${i === 0 ? ' (強い制限)' : i === 12 ? ' (弱い制限)' : ''}`])],
  help: '回転が低いときに出力を絞る。Off にしないと重い負荷で低回転に張り付くことがある' };
const STARTUP_BEEP = { key: 'STARTUP_BEEP', off: 0x05, label: '起動音', type: 'bool' };
const DITHER = { key: 'DITHERING', off: 0x06, label: 'Dithering', type: 'bool' };
const BRAKING = { key: 'BRAKING_STRENGTH', off: 0x10, label: 'ブレーキの強さ', type: 'num', min: 0, max: 255 };
const POWER_RATING = { key: 'POWER_RATING', off: 0x29, label: '電源', type: 'enum', opts: [[1, '1S'], [2, '2S 以上']] };
const EDT_ARM = { key: 'FORCE_EDT_ARM', off: 0x2A, label: 'EDT を必須にする', type: 'bool' };
const PWM_209 = { key: 'PWM_FREQUENCY', off: 0x0A, label: 'PWM 周波数', type: 'enum',
  opts: [[24, '24 kHz'], [48, '48 kHz'], [96, '96 kHz'], [0, '可変']] };

// The startup melody sits in the same block (SiLabs 0x70..0xEF, AM32 tune[] at 48..175). A field
// with get/put stands for something that is not one byte; put(edit, v, base) gets the block as it was
// read, so a choice can be undone without losing bytes. offer(base, k) hides an option it cannot give.
// Bluejay skips the melody when its first byte is 0xFF (Fx.asm play_beep_melody); the notes stay,
// so turning it back on only restores that byte (as read, or 0x02 if it was read off).
const MELODY_END = 0xF0;
const MELODY = { key: 'startup_melody', label: '起動音', type: 'enum', opts: [[1, 'メロディ'], [0, 'なし']],
  get: (b) => (b[0x70] === 0xFF ? 0 : 1),
  put: (b, v, base) => { b[0x70] = v ? (base && base[0x70] !== 0xFF ? base[0x70] : 0x02) : 0xFF; },
  offer: (base, k) => k !== 1 || base[0x70] !== 0xFF || base.subarray(0x71, MELODY_END).some((x) => x !== 0xFF && x !== 0),
  help: '電源投入時のメロディ。なしでも、信号を受けたときの短い音は鳴る' };
// AM32 plays tune[] when tune[0] is not 0xFF and stops at the first all-zero note (sounds.c
// playBlueJayTune); with tune[0] = 0xFF it plays its own three short beeps instead.
const AM32_TUNE = 48, AM32_TUNE_END = 176;
const TUNE = { key: 'startup_tune', label: '起動音', type: 'enum',
  opts: [[2, 'メロディ'], [1, '短いビープ 3 回'], [0, 'なし']],
  get: (b) => (b[AM32_TUNE] === 0xFF ? 1 : (b[AM32_TUNE + 4] || b[AM32_TUNE + 5] ? 2 : 0)),
  put: (b, v, base) => {
    if (v === 2) { b.set(base.subarray(AM32_TUNE, AM32_TUNE_END), AM32_TUNE); return; }   // the tune as read
    if (v === 1) { b[AM32_TUNE] = 0xFF; return; }
    if (v === 0) { if (b[AM32_TUNE] === 0xFF) b.fill(0, AM32_TUNE, AM32_TUNE + 4); b[AM32_TUNE + 4] = 0; b[AM32_TUNE + 5] = 0; }
  },
  offer: (base, k) => k !== 2 || TUNE.get(base) === 2,   // a melody only if one was read
  help: '電源投入時の音。AM32 は信号が 2 秒途切れると再起動するので、そのたびに鳴る' };

function bluejayFields(rev) {
  const f = [DIRECTION, SP_MIN];
  if (rev >= 201) f.push(SP_MAX);
  f.push(RPM_SLOPE, TIMING, DEMAG, TEMP);
  if (rev === 200) f.push({ key: 'LOW_RPM_POWER_PROTECTION', off: 0x24, label: '低回転時の出力保護', type: 'bool' });
  f.push(BRAKE);
  if (rev >= 204) f.push(BRAKING);
  if (rev >= 206) f.push(POWER_RATING);
  if (rev >= 207) f.push(EDT_ARM);
  if (rev >= 209) {
    f.push(PWM_209,
      { key: 'THRESHOLD_96to48', off: 0x2C, label: '96→48 kHz の閾値', type: 'num', min: 0, max: 255 },
      { key: 'THRESHOLD_48to24', off: 0x2B, label: '48→24 kHz の閾値', type: 'num', min: 0, max: 255 });
  }
  f.push(BEEP, BEACON, BEACON_DELAY);
  if (rev <= 202 || rev === 205) f.push(STARTUP_BEEP);
  f.push(MELODY);
  if (rev <= 207) f.push(DITHER);
  f.push(LED);
  return f;
}

// --- BlueGill (layout 226): Bluejay 0.21 + forced-sine start (host/pico_esc/config.py) ----------
const BLUEGILL_EXTRA = [
  { key: 'sine_mode', off: 0x2E, label: 'sine 起動 (BlueGill)', type: 'enum',
    opts: [[0, 'Off (6-step)'], [1, 'S1 低速 sine'], [2, 'S2 sine 起動'], [0xFF, 'Off (未設定)']] },
  { key: 'sine_hold_amp', off: 0x2F, label: 'sine 保持振幅', type: 'num', min: 0, max: 254 },
  { key: 'sine_amp_max', off: 0x30, label: 'sine 最大振幅', type: 'num', min: 0, max: 254 },
  { key: 'sine_ramp', off: 0x31, label: 'sine ランプ', type: 'num', min: 0, max: 254 },
  { key: 'sine_cross_up', off: 0x32, label: 'sine→BEMF 切替 (上り)', type: 'num', min: 0, max: 255,
    fmt: (v) => (v ? `≈ ${(v * 39.0625).toFixed(0)} eRPM` : 'off') },
  { key: 'sine_cross_dn', off: 0x33, label: 'BEMF→sine 切替 (下り)', type: 'num', min: 0, max: 0xEF,
    fmt: (v) => (v ? `≈ ${(312500 / v).toFixed(0)} eRPM` : 'off') },
  { key: 'comm_timing_angle', off: 0x2B, label: 'タイミング角 (BlueGill)', type: 'num', min: 0, max: 255 },
  { key: 'max_erpm', off: 0x2C, label: '最大 eRPM (×1000)', type: 'num', min: 0, max: 255 },
  { key: 'lowspeed_damping', off: 0x2D, label: '低速ダンピング', type: 'num', min: 0, max: 255 },
];

// --- AM32 -------------------------------------------------------------------------------------
const b = (key, off, label, help) => ({ key, off, label, type: 'bool', help });
const n = (key, off, label, min, max, fmt, help) => ({ key, off, label, type: 'num', min, max, fmt, help });
const AM32 = [
  b('dir_reversed', 17, '回転方向を反転'),
  b('bi_direction', 18, '双方向 (3D)'),
  b('use_sine_start', 19, 'sine 起動', '低速からなめらかに起動する。comp_pwm が必要'),
  b('comp_pwm', 20, '相補 PWM'),
  { key: 'variable_pwm', off: 21, label: 'PWM 周波数の自動変更', type: 'enum', opts: [[0, '固定'], [1, '自動'], [2, '自動 (2)']] },
  b('stuck_rotor_protection', 22, 'ロック保護'),
  n('advance_level', 23, 'タイミング進角', 10, 42, (v) => `${((v - 10) * 0.9375).toFixed(1)}°`),
  n('pwm_frequency', 24, 'PWM 周波数 (kHz)', 8, 144),
  n('startup_power', 25, '起動出力', 50, 150),
  n('motor_kv', 26, 'モーター KV', 0, 255, (v) => `${v * 40 + 20} KV`),
  n('motor_poles', 27, '極数', 2, 36),
  { key: 'brake_on_stop', off: 28, label: '停止時ブレーキ', type: 'enum', opts: [[0, 'Off'], [1, 'Drag'], [2, 'Active']] },
  b('stall_protection', 29, '低回転の失速保護'),
  n('beep_volume', 30, 'ビープ音量', 0, 11),
  TUNE,
  b('telemetry_on_interval', 31, '定期テレメトリ'),
  n('max_ramp', 5, '最大ランプ', 10, 250, (v) => `${(v / 10).toFixed(1)} %/ms`),
  n('minimum_duty_cycle', 6, '最小デューティ', 1, 50, (v) => `${(v / 5).toFixed(1)} %`),
  n('sine_mode_changeover_thottle_level', 40, 'sine 切替スロットル', 5, 25, (v) => `${v} %`),
  n('sine_mode_power', 45, 'sine 出力', 1, 10),
  { key: 'low_voltage_cut_off', off: 36, label: '低電圧カット', type: 'enum', opts: [[0, 'Off'], [1, 'セル電圧'], [2, '絶対電圧']] },
  n('low_cell_volt_cutoff', 37, 'セル電圧カット', 0, 255, (v) => `${(2.5 + v / 100).toFixed(2)} V`),
  n('absolute_voltage_cutoff', 8, '絶対電圧カット', 0, 255, (v) => `${(v / 2).toFixed(1)} V`),
  n('limit_temperature', 43, '温度制限', 70, 255, (v) => (v > 140 ? 'off' : `${v} °C`)),
  n('limit_current', 44, '電流制限', 0, 255, (v) => (v > 100 || !v ? 'off' : `${v} A`)),
  n('drag_brake_strength', 41, 'ドラッグブレーキ', 1, 10),
  n('driving_brake_strength', 42, '走行ブレーキ', 1, 10),
  n('active_brake_power', 12, 'アクティブブレーキ', 0, 5),
  { key: 'brake_on_zero_throttle', off: 13, label: 'ゼロスロットル時', type: 'num', min: 0, max: 9 },
  b('rc_car_reverse', 38, 'RC カー式リバース'),
  b('use_hall_sensors', 39, 'ホールセンサー'),
  { key: 'input_type', off: 46, label: '入力', type: 'enum', opts: [[0, '自動'], [1, 'DShot'], [2, 'Servo'], [3, 'Serial'], [4, 'EDT arm']] },
  b('auto_advance', 47, '自動進角'),
  b('disable_stick_calibration', 7, 'スティック校正を無効'),
  n('servo_low_threshold', 32, 'Servo 下限', 0, 255),
  n('servo_high_threshold', 33, 'Servo 上限', 0, 255),
  n('servo_neutral', 34, 'Servo 中立', 0, 255),
  n('servo_dead_band', 35, 'Servo 不感帯', 0, 255),
  n('current_P', 9, '電流 P', 0, 255),
  n('current_I', 10, '電流 I', 0, 255),
  n('current_D', 11, '電流 D', 0, 255),
];

// --- the families -----------------------------------------------------------------------------
export const SILABS = {
  eeprom: 0x1A00, size: 0x100, layoutTag: 0x40, mcuTag: 0x50, name: 0x60,
  pageSize: 0x200, eepromPage: 0x0D, bootloader: 0x1C00,
  // 512-byte-page parts whose app ends at 0x1A00: the only ones flashing is offered for
  flashable: new Set([0xE8B1, 0xE8B2, 0xF330, 0xF390, 0xF850]),
};
export const AM32_MAP = {
  offsets: { 0x1F: 0x7C00, 0x35: 0xF800 }, size: 192, nameLen: 32, appStart: 0x1000,
};

// Which map a SiLabs block uses, from its revision bytes
export function silabsFamily(block) {
  const main = block[0], layout = block[2];
  if (layout === 226) return { id: 'bluegill', name: 'BlueGill', fields: [...bluejayFields(208).filter((f) => f !== MELODY), ...BLUEGILL_EXTRA] };
  if (layout >= 200 && layout <= 209) return { id: 'bluejay', name: 'Bluejay', fields: bluejayFields(layout) };
  if (main === 16 && (layout === 32 || layout === 33)) return { id: 'blheli_s', name: 'BLHeli_S', fields: BLHELI_S };
  return { id: 'unknown', name: '不明', fields: [] };
}

export function am32Family(block) {
  // eeprom_version 2+ is the layout above (max_ramp at 5); older blocks put a name there
  const ok = block[0] === 0x01 && block[1] >= 2 && block[1] !== 0xFF;
  return { id: ok ? 'am32' : 'unknown', name: 'AM32', fields: ok ? AM32 : [] };
}

// Preset family the CLI uses (src/apps/presets.h): Bluejay 0.21 = layout 208, BlueGill = 226
export function presetFamily(esc) {
  if (esc.family.id === 'am32') return 'am32';
  if (esc.family.id === 'bluegill') return 'bluegill';
  if (esc.family.id === 'bluejay' && esc.block[2] === 208) return 'bluejay';
  return null;
}
