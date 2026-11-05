// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_tool_app — the "tool" surface as a COMPOSABLE module: config / flash + a USB-serial CLI +
// (SETUP mode) a Wi-Fi web UI, over a set of ESCs. It does NOT own the ESCs — the composition root
// (main.cpp) declares the Thruster objects and hands EscTool an array of pointers, so the same
// Thrusters can be driven directly (a bare ROV loop) WITHOUT this module. This keeps responsibilities
// split: Thruster = one ESC (config + drive + velocity); EscTool = the operator/host-facing surface.
//   * config / flash BLHeli-S ESCs over the 1-wire bootloader (shared escs:: engine),
//   * relays RAW (direct thrust/throttle) AND RPM (closed velocity loop) drive to each Thruster,
//   * a USB-serial CLI (host/esctool.py) AND, in SETUP mode, a Wi-Fi web UI.
// The escs:: engine is the genuine singleton underneath (2 PIO SMs + one core1 1-wire worker);
// Thruster/EscTool are the OO layer on top, delegating hardware to escs:: by index.
//
// MODES (unchanged from esc_tool): SETUP (default) = Wi-Fi AP + full serial API;
// DRIVE = Wi-Fi off (save power / no RF), serial API only. Chosen at boot by ESC_MODE_PIN, switchable
// live with the `mode` command.
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include "presets.h"
#include <Wire.h>
#include "esc.h"
#include "as5600.h"
#include "esc_flash.h"
#include "thruster.h"
#include "azimuth.h"
#include "settings.h"
#include "fourway.h"
#if ESC_LCD
#include "lcd1602.h"
#endif
#if ESC_WIFI_STA
#include "net_console.h"
#include "wifi_secrets.h"
#endif

// ---- small helpers (unchanged from esc_tool.cpp) ----
static void printHex(const uint8_t* p, uint16_t n) {
	for (uint16_t i = 0; i < n; i++) { console::out().print("0123456789ABCDEF"[p[i] >> 4]); console::out().print("0123456789ABCDEF"[p[i] & 0xF]); }
}
static int hexVal(char c) { if (c>='0'&&c<='9') return c-'0'; if (c>='A'&&c<='F') return c-'A'+10; if (c>='a'&&c<='f') return c-'a'+10; return -1; }
static int parseHex(const char* s, uint8_t* buf, int cap) {
	int n = 0; for (; s[0] && s[1]; s += 2) { if (n >= cap) return -1; int hi=hexVal(s[0]),lo=hexVal(s[1]); if(hi<0||lo<0) return -1; buf[n++]=(uint8_t)((hi<<4)|lo); } return n;
}
static String jesc(const char* s) { String o; for (; *s; s++) { if (*s=='"'||*s=='\\') o+='\\'; o+=*s; } return o; }

// [B.8] Print an OPTIONAL soft-sensor field: the value, or the literal `n/a` when there is no reading.
// The soft-sensor's accessors fall back to 0.0f when their gate is shut, and every one of those zeros
// is also a plausible MEASUREMENT — 0 V is a dead supply, 0 V of droop is an unloaded prop. So on the
// wire "no reading" and "I measured zero" were the same token, and on 2026-08-18 a `vbatt=0.00` from a
// motor that simply was not spinning got quoted as evidence the supply was off (`valid=0`/`vbok=0` sat
// in the same line and were not read — which is the point: a field that reads as a number is believed
// on its own). Making the absent case unprintable as a number is the fix; documenting it harder is not.
// Hosts parse key=value so no key moves, but anything float()ing these must tolerate a non-number
// (`_f()` in host/driftsoak.py and host/loadtest.py). Caller supplies the buffer — one PER FIELD:
// printf's arguments are unsequenced, so a shared buffer would race between them.
static const char* optNum(char* buf, size_t n, bool has, float v) {
	if (has) snprintf(buf, n, "%.2f", v); else snprintf(buf, n, "n/a");
	return buf;
}

// writable settings exposed in the web UI (raw byte per config offset) — verbatim from esc_tool.cpp
struct Field { const char* name; uint16_t off; };
static const Field FIELDS[] = {
	{ "motor_direction",0x0B },{ "comm_timing",0x15 },{ "demag_compensation",0x1F },
	{ "startup_beep",0x05 },{ "beep_strength",0x1B },{ "beacon_strength",0x1C },
	{ "beacon_delay",0x1D },{ "temperature_protection",0x23 },
	{ "low_rpm_power_protection",0x24 },{ "brake_on_stop",0x27 },
	// BlueGill S1 forced-commutation stepper params (0xFF = off/default on stock/older fw).
	{ "sine_mode",0x2E },{ "sine_hold_amp",0x2F },{ "sine_amp_max",0x30 },{ "sine_ramp",0x31 },
	// BlueGill S3 sine<->BEMF crossover thresholds (0 = off).
	{ "sine_cross_up",0x32 },{ "sine_cross_dn",0x33 },
	// NOTE (pre-existing, do NOT fix here): this array lists low_rpm_power_protection at
	// 0x24 while host/esctool.py uses 0x09 for the same field, and it omits B1's 0x2B-0x2D.
};
static const int NFIELD = sizeof(FIELDS) / sizeof(FIELDS[0]);

// on-device firmware library location (LittleFS)
static const char* FW_DIR = "/fw";

// `curve` upload staging capacity — must match Thruster's owned FF buffer (and settings::CFG_MAX_CURVE).
static const int CURVE_STAGE_CAP = settings::CFG_MAX_CURVE;

// ================= Web UI (SETUP mode) =================
// Verbatim from esc_tool.cpp except for the ADDED per-ESC "Set RPM" control in the spin-test panel
// (rpm number input + button -> POST /api/rpm) and its setRpm() helper.
static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>pico-esc-tool</title>
<style>body{font-family:system-ui,sans-serif;margin:1em;max-width:680px}button{padding:.4em .8em;margin:.2em}
label{display:block;margin:.3em 0}input[type=number]{width:6em}.esc{border:1px solid #ccc;border-radius:8px;padding:.5em;margin:.5em 0}
#msg{color:#070}.stop{background:#c22;color:#fff}</style></head><body><h1>pico-esc-tool</h1>
<button onclick=scan()>Scan</button><button onclick=disc()>Disconnect</button>
<button class=stop onclick=stopAll()>STOP ALL</button><span id=msg></span>
<div id=list></div><div id=cfg></div>
<div id=fw class=esc><h3>Firmware</h3>
 <b>Library</b> (stored on the Pico) <button onclick=loadFwList()>refresh</button>
 <div id=fwlib><i>scan first</i></div>
 <div style="margin-top:.4em"><b>Add / one-off:</b>
  <input type=file id=fwfile accept=.hex> name <input id=fwname size=8 placeholder=label>
  ESC <select id=fwesc onchange=loadFwList()></select>
  <label style="display:inline"><input type=checkbox id=fwforce> force</label>
  <button onclick=saveFw()>Save to library</button>
  <button onclick=flashFw()>Flash uploaded now</button> <span id=fwmsg></span></div>
 <p style="color:#888;font-size:.85em">App-only + firmware defaults; bootloader preserved. Get HEX: BLHeli-S (bitdump/BLHeli) or Bluejay (bird-sanctuary/bluejay); layout must match the ESC. Uploaded files persist in the library.</p></div>
<script>
const g=id=>document.getElementById(id),msg=t=>g('msg').textContent=t;let tick=null;
async function scan(){msg('scanning...');let d=await(await fetch('/api/scan')).json();
 g('list').innerHTML=d.map((e,i)=>e.present?`<div class=esc><b>ESC ${i}</b> pin ${e.pin} sig ${e.sig} layout ${e.layout} "${e.name}" fw ${e.fw}
   <button onclick=read(${i})>Edit</button><button onclick=spinUI(${i})>Spin test</button></div>`
   :`<div class=esc>ESC ${i}: none</div>`).join('');
 g('fwesc').innerHTML=d.map((e,i)=>e.present?`<option value=${i} data-layout="${e.layout}">ESC ${i} (${e.layout})</option>`:'').join('');
 loadFwList();msg('');}
const fm=t=>g('fwmsg').textContent=t;
function pollFlash(){let t=setInterval(async()=>{let s=await(await fetch('/api/flashstatus')).json();
  if(s.state=='run'||s.state=='start')fm(`flashing ${s.done}/${s.total}...`);
  else if(s.state=='ok'){clearInterval(t);fm('OK: '+s.msg);scan();}
  else if(s.state=='err'){clearInterval(t);fm('FAILED: '+s.msg);}},400);}
function escLayout(){let o=g('fwesc').selectedOptions[0];return o?o.dataset.layout:'';}
async function loadFwList(){let d=await(await fetch('/api/fwlist')).json();let cl=escLayout();
 g('fwlib').innerHTML=d.length?d.map(f=>`<div>${f.name} <small>[${f.layout||'?'}]</small>
   <button onclick="flashStored('${f.name}')">Flash to ESC ${g('fwesc').value}</button>
   <button onclick="delFw('${f.name}')">del</button>${(f.layout&&f.layout!=cl)?' <small style=color:#c60>layout&ne;ESC</small>':''}</div>`).join('')
  :'<i>empty &mdash; add a .hex below</i>';}
async function saveFw(){let f=g('fwfile').files[0];if(!f){fm('pick a .hex');return;}
 let name=g('fwname').value||f.name.replace(/\.hex$/i,'');let fd=new FormData();fd.append('hex',f);fm('saving...');
 let r=await(await fetch('/api/fwsave?name='+encodeURIComponent(name),{method:'POST',body:fd})).json();
 fm(r.ok?('saved '+r.name+' ['+r.layout+']'):('error: '+(r.err||'?')));loadFwList();}
async function flashStored(name){let i=g('fwesc').value;if(i===''){fm('scan first');return;}
 let force=g('fwforce').checked?1:0;
 let r=await(await fetch('/api/flashstored?name='+encodeURIComponent(name)+'&i='+i+'&force='+force,{method:'POST'})).json();
 if(!r.ok){fm('error: '+(r.err||'?'));return;}pollFlash();}
async function delFw(name){await fetch('/api/fwdelete?name='+encodeURIComponent(name),{method:'POST'});loadFwList();}
async function flashFw(){let f=g('fwfile').files[0];if(!f){fm('pick a .hex first');return;}
 let i=g('fwesc').value;if(i===''){fm('scan for an ESC first');return;}
 let force=g('fwforce').checked?1:0,fd=new FormData();fd.append('hex',f);fm('uploading...');
 let r=await(await fetch('/api/flash?i='+i+'&force='+force,{method:'POST',body:fd})).json();
 if(!r.ok){fm('error: '+(r.err||'?'));return;}pollFlash();}
async function read(i){stopTick();let d=await(await fetch('/api/read?i='+i)).json();if(d.error){msg(d.error);return;}
 let s=d.settings;g('cfg').innerHTML=`<div class=esc><h3>ESC ${i} settings (fw ${d.fw})</h3>`
  +Object.keys(s).map(k=>`<label>${k} <input id=f_${k} type=number min=0 max=255 value="${s[k]}"></label>`).join('')
  +`<button onclick=save(${i})>Save</button></div>`;}
async function save(i){let p=[...document.querySelectorAll('[id^=f_]')].map(el=>el.id.slice(2)+'='+el.value).join('&');
 let d=await(await fetch('/api/set?i='+i+'&'+p,{method:'POST'})).json();msg(d.error?d.error:('saved: '+d.result));}
function spinUI(i){stopTick();g('cfg').innerHTML=`<div class=esc><h3>ESC ${i} spin test</h3>
  <p><b>Props off / motor secured.</b> Arm, wait ~3s, then move the slider.</p>
  <button onclick="armEsc(${i})">Arm</button> <span id=smode></span><br>
  <input id=thr type=range min=0 max=2000 value=0 oninput="g('tv').textContent=this.value">
  <span id=tv>0</span> <button onclick="g('thr').value=0;g('tv').textContent=0">Center/Stop</button>
  <button class=stop onclick="disarmEsc(${i})">Disarm / Stop</button>
  <div style="margin-top:.3em">closed-loop RPM <input id=rpmv type=number value=0 step=100>
   <button onclick="setRpm(${i})">Set RPM</button></div>
  <p id=tele>not armed</p></div>`;}
async function armEsc(i){let a=await(await fetch('/api/arm?i='+i,{method:'POST'})).json();msg('arming ~3s...');
  let s=g('thr');
  if(a.rev){s.min=-1000;s.max=1000;}else{s.min=0;s.max=2000;}s.value=0;g('tv').textContent=0;
  g('smode').textContent='DShot: '+a.mode+(a.rev?' — reversible (−1000..+1000, 0=stop)':' — one-way (0..2000)');
  stopTick();
  tick=setInterval(async()=>{await fetch('/api/spin?i='+i+'&v='+g('thr').value,{method:'POST'});
   let t=await(await fetch('/api/tele?i='+i)).json();
   g('tele').textContent=t.ok?(`${t.armed?'ARMED':'arming...'}  `+(t.tele?`rpm ${t.rpm}  ${t.volt}V  ${t.amp}A  ${t.temp}C  stress ${t.stress}`:'(no telemetry — normal DShot)')):'no telemetry';},200);}
async function setRpm(i){await fetch('/api/rpm?i='+i+'&v='+g('rpmv').value,{method:'POST'});msg('rpm '+g('rpmv').value);}
async function disarmEsc(i){stopTick();g('thr').value=0;g('tv').textContent=0;await fetch('/api/disarm?i='+i,{method:'POST'});msg('disarmed');}
function stopTick(){if(tick){clearInterval(tick);tick=null;}}
async function stopAll(){stopTick();await fetch('/api/spinstop',{method:'POST'});msg('stopped');}
async function disc(){stopTick();await fetch('/api/run',{method:'POST'});g('cfg').innerHTML='';msg('ESC restarted');}
scan();
</script></body></html>)HTML";

// ===================================================================================================
// EscManager — the single integrated firmware object (one instance in main.cpp).
// ===================================================================================================
class EscTool {
public:
	enum Mode { SETUP, DRIVE };

	// Composed by main: an array of the Thrusters this tool operates on (main owns them, binds them, and
	// sets their per-motor gains BEFORE calling begin()). n = how many.
	EscTool(Thruster** thrusters, uint8_t n) : th_(thrusters), n_(n) {}

	// io = where the CLI lives (see console.h). USB CDC by default; pass an already-begun UART for a
	// board with no USB host. USB is started either way so a laptop can still watch the boot.
	void begin(Stream& io = Serial) {
		Serial.begin(115200);
		console::io = &io;
#if ESC_LCD
		lcd_.begin();
		lcd_.row(0, "pico-esc-tool");
		lcd_.row(1, "booting...");
		for (uint8_t k = 0; k < 16; k++) lcd_.poll(4);
#endif
#if ESC_WIFI_STA
		net_.begin(io, WIFI_SSID, WIFI_PASS, ESC_HOSTNAME);   // CLI on USB and TCP from here on
		console::io = &net_;
#endif
#if ESC_MODE_PIN >= 0
		pinMode(ESC_MODE_PIN, INPUT_PULLUP);
#endif
		delay(50);
		if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }   // firmware library store
		{ static const uint8_t pins[] = ESC_SIGNAL_PINS;                  // GP16/17 wired as an ESC signal:
		  for (uint8_t p : pins) if (p == 16 || p == 17) encOff_ = true; } // the encoder's I2C would take it
		if (!encOff_) enc_.begin();                                      // AS5600 encoder (I2C0, GP16/17)
		LittleFS.mkdir("/fw");
		// Restore the saved runtime config (motor identity, gains, DOB, vbatt scale) — so `sense vcal`
		// and `motor` are no longer per-power-cycle. main.cpp has already applied the compile-time
		// defaults, so a stored config deliberately wins over them; an absent/corrupt one changes nothing.
		LittleFS.mkdir(settings::CFG_DIR);
		uint16_t storedVer = 0;
		int restored = settings::load(th_, n_, &storedVer);
		if (restored > 0)
			console::out().printf("# cfg: restored %d ESC config(s) from %s\n", restored, settings::CFG_PATH);
		else if (storedVer && storedVer != settings::CFG_VERSION)
			console::out().printf("# cfg: IGNORED a stored config written as v%u (this build wants v%u) — "
			              "gains/vcal are at compile-time defaults; re-save with `cfg save`\n",
			              storedVer, settings::CFG_VERSION);
		// The azimuth's hull reference, from its own blob. Absent is not an error — it only means
		// `az gain absolute 1` will refuse until `az zero` has been run against the rig.
		// Bind at boot, not on the first `az on`: `az status` / `az zero show` read the encoder
		// through the same pointers, and unbound they reported the heading as "inf".
		if (n_ >= 2) az_.bind(th_[0], th_[1], &enc_);
		loadPresent();
		az_.zero_raw = settings::azLoadZero();
		if (az_.zero_raw >= 0)
			console::out().printf("# az: heading zero restored (raw %ld)\n", (long)az_.zero_raw);

		// Wi-Fi routes: lambdas capturing `this` bind each HTTP path to a member handler.
		server_.on("/", [this]{ hIndex(); });
		server_.on("/api/scan", [this]{ hScan(); });
		server_.on("/api/read", [this]{ hRead(); });
		server_.on("/api/set", HTTP_POST, [this]{ hSet(); });
		server_.on("/api/run", HTTP_POST, [this]{ hRun(); });
		server_.on("/api/arm", HTTP_POST, [this]{ hArm(); });
		server_.on("/api/spin", HTTP_POST, [this]{ hSpin(); });
		server_.on("/api/rpm", HTTP_POST, [this]{ hRpm(); });
		server_.on("/api/disarm", HTTP_POST, [this]{ hDisarm(); });
		server_.on("/api/spinstop", HTTP_POST, [this]{ hSpinStop(); });
		server_.on("/api/tele", [this]{ hTele(); });
		server_.on("/api/flash", HTTP_POST, [this]{ hFlashStart(); }, [this]{ hFlashUpload(); });
		server_.on("/api/flashstatus", [this]{ hFlashStatus(); });
		server_.on("/api/fwsave", HTTP_POST, [this]{ hFwSave(); }, [this]{ hFlashUpload(); });
		server_.on("/api/fwlist", [this]{ hFwList(); });
		server_.on("/api/flashstored", HTTP_POST, [this]{ hFlashStored(); });
		server_.on("/api/fwdelete", HTTP_POST, [this]{ hFwDelete(); });

#if ESC_MODE_PIN >= 0
		setMode(digitalRead(ESC_MODE_PIN) == LOW ? DRIVE : SETUP);  // LOW=drive; unconnected(HIGH)=setup
#else
		setMode(SETUP);                                             // mode pin disabled: boot into SETUP
#endif
	}

	// core0 loop.
	void poll() {
#if ESC_WIFI_STA
		net_.poll();
#endif
		handleSerial();
		if (!encOff_) enc_.poll();                              // high-rate AS5600 sampling (self-gated)
		if (escs::armMask() != armSaved_) savePresent(escs::presentMask());   // a connect learned AM32
		motorTestPoll();
		idlePoll();
		az_.poll();                     // on-device azimuth loop — runs at the tracker's rate, not 50 Hz
		if (fl_ == FL_START || fl_ == FL_RUN) flashStep();      // flashing: one page per loop (no spin)
		else {
			for (uint8_t i = 0; i < n_; i++) { th_[i]->poll(); thermalNotice(i); }   // RPM submode closes the loop
			escs::spinPoll();                                          // keep DShot frames flowing
		}
		linkWatch();
		if (wifiUp_) server_.handleClient();
#if ESC_LCD
		lcdPoll();
#endif
	}

	// core1 loop.
	void pollCore1() { escs::core1Poll(); }

private:
	// HOST-LINK DEADMAN (`link <ms>`, 0 = off, the default). escs::'s own 500 ms deadman cannot see a
	// lost host in RPM submode: the closed loop itself re-sends throttle every tick, so a host that
	// sets `rpm 1 2000` and then dies leaves the motor at 2000 rpm indefinitely. This watches the CLI
	// instead — ANY complete command line counts as alive, so a soak that polls `sense` once a second
	// keeps it fed without resending its target — and stops every ESC once when the silence outlasts
	// the timeout while something is spinning. Opt-in so the existing bench scripts keep their
	// behaviour; NOT persisted, so a power cycle always comes back with it off.
	uint32_t linkTimeoutMs_ = 0, linkLastMs_ = 0;
	bool     linkTripped_   = false;
	void linkWatch() {
		if (!linkTimeoutMs_ || linkTripped_ || !driving()) return;
		if (millis() - linkLastMs_ <= linkTimeoutMs_) return;
		for (uint8_t k = 0; k < n_; k++) th_[k]->stop();
		escs::spinStopAll();
		linkTripped_ = true;
		console::out().printf("# link: no command for %lu ms (limit %lu) — all ESCs stopped\n",
		                      (unsigned long)(millis() - linkLastMs_), (unsigned long)linkTimeoutMs_);
	}

	// [B.7] One-shot serial notice whenever the thermal limiter CHANGES state. The two states worth
	// waking an operator for are DERATE (the firmware is now withholding command — a `rpm` that no longer
	// tracks is protection, not a fault) and STALE (temperature was being observed and is not any more,
	// so the limiter has FAILED OPEN — deliberately, but it must never be silent). Printed once per
	// transition rather than per tick: this runs at the core0 loop rate.
	void thermalNotice(uint8_t i) {
		if (i >= escs::COUNT) return;
		vel::TempState s = th_[i]->vc.tempState();
		// NONE is the one failure mode with no transition to announce, and the most likely one in
		// practice: EDT is only enabled post-arm (esc_session.h sends DShot 13 a few times after the arm
		// window), so if that never takes, the whole session runs with zero thermal protection and the
		// only evidence is the ABSENCE of a line nobody was waiting for. "Fails open but never silently"
		// has to cover this case too, so time it out and say so once.
		if (s == vel::TempState::NONE && th_[i]->submode() == Thruster::RPM && th_[i]->armed()) {
			if (!tempNoneSince_[i]) tempNoneSince_[i] = millis();
			else if (!tempNoneSaid_[i] && millis() - tempNoneSince_[i] > THERMAL_NONE_WARN_MS) {
				tempNoneSaid_[i] = true;
				console::out().printf("# ESC %u no EDT temperature after %lus of closed-loop drive — NO thermal "
				              "protection (limiter idle). Re-arm to retry EDT enable.\n",
				              i, (unsigned long)(THERMAL_NONE_WARN_MS / 1000));
			}
		} else {
			tempNoneSince_[i] = 0;
			if (s != vel::TempState::NONE) tempNoneSaid_[i] = false;   // re-arm the warning for next time
		}
		if ((uint8_t)s == tempSeen_[i]) return;
		tempSeen_[i] = (uint8_t)s;
		if (s == vel::TempState::DERATE)
			console::out().printf("# ESC %u thermal DERATE: ESC MCU %.0fC (warn %.0f, abort %.0f) — command ceiling %.0f%%\n",
			              i, th_[i]->vc.temp(), th_[i]->vc.warn_temp, th_[i]->vc.max_temp,
			              th_[i]->vc.tempLimit() * 100.0f);
		else if (s == vel::TempState::STALE)
			console::out().printf("# ESC %u thermal telemetry LOST (last %.0fC) — limiter released, NO thermal protection\n",
			              i, th_[i]->vc.temp());
		else if (s == vel::TempState::OK)
			console::out().printf("# ESC %u thermal ok: ESC MCU %.0fC, full command authority\n", i, th_[i]->vc.temp());
	}
	uint8_t  tempSeen_[escs::COUNT] = {0};       // last ANNOUNCED TempState per ESC (0 == TempState::NONE)
	uint32_t tempNoneSince_[escs::COUNT] = {0};  // millis() this ESC started driving with no temperature
	bool     tempNoneSaid_[escs::COUNT] = {false};
	// How long a thruster may run the closed loop with NO temperature before we call it out. EDT enable
	// is 10 DShot-13 frames sent right after the arm window, so a working ESC reports within well under a
	// second; 5 s is generous enough that a slow first frame is not mistaken for a dead channel, and short
	// enough that the operator hears about it near the start of a run rather than after it.
	static const uint32_t THERMAL_NONE_WARN_MS = 5000;

	// ---- presets (presets.h): named settings per motor ------------------------------------------
	//   preset                      list built-in and saved
	//   preset show <name>          its bytes
	//   preset apply <i> <name>     write it to ESC i (checks the firmware family first)
	//   preset save <i> <name>      capture ESC i's current settings under <name>
	//   preset rm <name>
	// apply/save leave ESC i running its app again.
	uint8_t presetFamily(uint8_t i) {   // connects ESC i; 0 = no answer / not a known settings map
		escs::Info in;
		if (!th_[i]->scan(in)) return 0;
		if (escs::am32Eeprom(i)) return presets::AM32;
		uint8_t cfg[esc_setup::kEepromLen];
		if (!th_[i]->readConfig(cfg)) return 0;
		return cfg[2] == 0xD0 ? presets::BLUEJAY : cfg[2] == 0xE2 ? presets::BLUEGILL : 0;
	}
	void presetCmd() {
		auto& o = console::out();
		char* sub = strtok(nullptr, " ");
		if (!sub) {
			for (const auto& p : presets::BUILTIN)
				o.printf("preset|%s|%s|builtin|%u|%s\n", p.name, presets::familyName(p.family), p.n, p.note);
			Dir d = LittleFS.openDir(presets::PRESET_DIR);
			while (d.next()) { presets::Loaded l;
				if (presets::find(d.fileName().c_str(), l)) o.printf("preset|%s|%s|saved|%u|\n", l.name, presets::familyName(l.family), l.n); }
			o.println("ok"); return;
		}
		if (!strcmp(sub, "show") || !strcmp(sub, "rm")) {
			char* name = strtok(nullptr, " "); presets::Loaded l;
			if (!name || !presets::find(name, l)) { o.println("err no-preset"); return; }
			if (sub[0] == 'r') { o.println(!l.builtin && presets::remove(name) ? "ok" : "err builtin"); return; }
			o.printf("preset|%s|%s|%s|%s\n", l.name, presets::familyName(l.family), l.builtin ? "builtin" : "saved", l.note);
			for (uint8_t k = 0; k < l.n; k++) o.printf("%s%02X:%02X", k ? "," : "bytes|", l.b[k].off, l.b[k].val);
			o.println(); o.println("ok"); return;
		}
		bool apply = !strcmp(sub, "apply");
		if (!apply && strcmp(sub, "save")) { o.println("err bad-args"); return; }
		int i = -1; { char* a = strtok(nullptr, " "); if (a) i = atoi(a); }
		char* name = strtok(nullptr, " ");
		if (i < 0 || i >= n_ || !name) { o.println("err bad-args"); return; }
		presets::Loaded l;
		if (apply && !presets::find(name, l)) { o.println("err no-preset"); return; }
		if (!apply && !presets::validName(name)) { o.println("err bad-name ([a-z0-9_-]{1,24})"); return; }
		uint8_t fam = presetFamily((uint8_t)i);
		if (!fam) { escs::release(); o.println("err no-connect or unknown firmware"); return; }
		if (apply && fam != l.family) {
			escs::release();
			o.printf("err wrong-family (ESC %d runs %s, preset is %s)\n", i, presets::familyName(fam), presets::familyName(l.family));
			return;
		}
		bool ok;
		if (fam == presets::AM32) ok = apply ? presetApplyAm32((uint8_t)i, l) : presetSaveAm32((uint8_t)i, name);
		else                      ok = apply ? presetApplyBj((uint8_t)i, l)   : presetSaveBj((uint8_t)i, name, fam);
		escs::release();
		o.println(ok ? "ok" : "err failed");
	}
	bool presetApplyBj(uint8_t i, const presets::Loaded& l) {
		uint16_t offs[presets::MAX_BYTES]; uint8_t vals[presets::MAX_BYTES];
		for (uint8_t k = 0; k < l.n; k++) { offs[k] = l.b[k].off; vals[k] = l.b[k].val; }
		bool ch = false; int r = th_[i]->editConfig(offs, vals, l.n, ch);
		if (r < 0) return false;
		console::out().printf("preset|%s|esc %u|%s\n", l.name, i, ch ? "written, verified" : "already set");
		return true;
	}
	bool presetSaveBj(uint8_t i, const char* name, uint8_t fam) {
		uint8_t cfg[esc_setup::kEepromLen];
		if (!th_[i]->readConfig(cfg)) return false;
		presets::Byte b[presets::MAX_BYTES]; uint8_t n = 0;
		for (uint8_t a = presets::BJ_FIRST; a <= presets::BJ_LAST; a++) b[n++] = { a, cfg[a] };
		if (!presets::save(name, fam, b, n)) return false;
		console::out().printf("preset|%s|saved %u bytes from esc %u (%s)\n", name, n, i, presets::familyName(fam));
		return true;
	}
	// AM32: rewrite the whole 192-byte block from its page start (the bootloader erases the page on
	// that write), then read it back. Byte 2 is the bootloader's own version, rewritten on every write.
	bool presetApplyAm32(uint8_t i, const presets::Loaded& l) {
		uint16_t eep = escs::am32Eeprom(i);
		uint8_t blk[192], back[192];
		if (!th_[i]->readFlash(eep, blk, 192)) return false;
		if (blk[0] != 0x01) { console::out().println("# preset: AM32 block not initialised (byte 0 != 1); use host am32tool defaults"); return false; }
		bool ch = false;
		for (uint8_t k = 0; k < l.n; k++) if (l.b[k].off < 192 && blk[l.b[k].off] != l.b[k].val) { blk[l.b[k].off] = l.b[k].val; ch = true; }
		if (ch) {
			if (!th_[i]->writeFlash(eep, blk, 192) || !th_[i]->readFlash(eep, back, 192)) return false;
			back[2] = blk[2];
			if (memcmp(back, blk, 192)) { console::out().println("# preset: AM32 read-back differs"); return false; }
		}
		console::out().printf("preset|%s|esc %u|%s\n", l.name, i, ch ? "written, verified" : "already set");
		return true;
	}
	bool presetSaveAm32(uint8_t i, const char* name) {
		uint8_t blk[presets::AM_LAST + 1];
		if (!th_[i]->readFlash(escs::am32Eeprom(i), blk, sizeof blk)) return false;
		presets::Byte b[presets::MAX_BYTES]; uint8_t n = 0;
		for (uint8_t a = presets::AM_FIRST; a <= presets::AM_LAST; a++) if (a < 14 || a > 16) b[n++] = { a, blk[a] };   // 14..16 reserved
		if (!presets::save(name, presets::AM32, b, n)) return false;
		console::out().printf("preset|%s|saved %u bytes from esc %u (am32)\n", name, n, i);
		return true;
	}

	// ---- configurator motor test (MSP_SET_MOTOR via fw_) ----------------------------------------
	// Like Betaflight's motor_disarmed: the last slider value holds until the next one. It stops,
	// disarming every ESC it armed, when
	//   - the USB link drops (DTR falls: the browser closed the port), and the input still queued
	//     for it is dropped, so a stale SET_MOTOR cannot restart a motor (Betaflight tasks.c,
	//     checkUsbLinkLost);
	//   - no MSP at all for MT_TIMEOUT_MS (esc-configurator polls BATTERY_STATE every 1 s while the
	//     motor panel is open, so silence means it is gone or wedged);
	//   - a passthrough starts. An ESC the CLI already armed is left alone, and a CLI `disarm` of
	//     one the test armed hands it back.
	// Spinning goes through Thruster::arm/setRaw like the CLI's `throttle`: the same 3 s arm, the
	// same 500 ms deadman (fed here every loop), the same reversible handling.
	static const uint32_t MT_TIMEOUT_MS = 2000;
	uint32_t mtOwned_ = 0;          // ESCs this motor test armed
	uint32_t mtArmedMs_ = 0;        // when an arm() returned: it can block for a bootloader visit
	bool     mtDtr_ = false;
	// IDLE HOLD. AM32 resets itself after 2 s without a valid frame (am32 Src/main.c, NVIC_SystemReset)
	// and plays its startup tune on every boot, so an AM32 left alone beeps every few seconds. A flight
	// controller never leaves it alone: it streams zero throttle. So does this, for the wired AM32 ESCs
	// while nothing else needs them — not in a passthrough, a flash or a bootloader session (a connect
	// frees the pin itself). The motor test takes an idle-held ESC over; a CLI throttle drives it as
	// it is. `idle off` turns it off. One arm per 5 s: an arm can block while it learns the ESC.
	uint32_t idle_ = 0, idleNextMs_ = 0;   // idle_: bit i = ESC i is held at zero throttle by this
	bool     idleOn_ = true;
	bool driving() {   // anything spinning that the idle hold does not own (PWM included)
		for (uint8_t i = 0; i < n_; i++) {
			if (escs::pwmActive(i)) return true;
			if ((escs::spinArmed(i) || escs::spinArming(i)) && !(idle_ >> i & 1)) return true;
		}
		return false;
	}
	void idlePoll() {
		for (uint8_t i = 0; i < n_; i++) {   // drop the ones that stopped, or that someone gave a throttle
			bool on = escs::spinArmed(i) || escs::spinArming(i);
			if (!on || escs::spinTarget(i)) idle_ &= ~(1u << i);
		}
		if (!idleOn_ || fw_.active() || fl_ == FL_START || fl_ == FL_RUN || escs::sessionOpen()) return;
		// An arm may read the ESC through its bootloader, which stops every DShot stream on the board
		// for seconds: never while something else is driven, and never again for an ESC whose read
		// already failed (unpowered, unplugged, broken) until `scan` or a configurator read succeeds.
		if (driving() || (int32_t)(millis() - idleNextMs_) < 0) return;
		for (uint8_t i = 0; i < n_; i++) {
			if (!(escs::armMask() >> i & 1) || !escs::isPresent(i) || escs::identFailed(i)) continue;
			if (escs::spinArmed(i) || escs::spinArming(i) || escs::pwmActive(i)) continue;
			th_[i]->arm(); th_[i]->setRaw(0);
			idle_ |= 1u << i; idleNextMs_ = millis() + 5000;
			return;
		}
	}
	void idleStop() { for (uint8_t i = 0; i < n_; i++) if (idle_ >> i & 1) th_[i]->stop(); idle_ = 0; }

	void motorTestStop(const char* why) {
		for (uint8_t i = 0; i < n_; i++) if (mtOwned_ >> i & 1) th_[i]->stop();
		// not during a passthrough: the line would land inside the configurator's 4-way replies
		if (mtOwned_ && !fw_.active()) console::out().printf("# motor test stopped: %s\n", why);
		mtOwned_ = 0; fw_.clearMotors();
	}
	void motorTestPoll() {
		bool reversibleWired = false;
		for (uint8_t i = 0; i < n_; i++) if (escs::isPresent(i) && escs::knownReversible(i)) reversibleWired = true;
		fw_.setThreeD(reversibleWired);
#if !ESC_CONSOLE_UART
		bool dtr = Serial.dtr();
		if (mtDtr_ && !dtr) {
			if (mtOwned_ || fw_.motorWanted()) {
				while (Serial.available()) Serial.read();
				motorTestStop("USB link lost");
			}
		}
		mtDtr_ = dtr;
#endif
		if (!mtOwned_ && !fw_.motorWanted()) return;
		if (fw_.active())                              { motorTestStop("passthrough"); return; }
		// Silence counts from the later of the last MSP and the last arm(): an arm that had to read
		// the ESC's settings held this loop (and the configurator's queued polls) for seconds.
		uint32_t heard = fw_.mspMs(); if ((int32_t)(mtArmedMs_ - heard) > 0) heard = mtArmedMs_;
		if (millis() - heard > MT_TIMEOUT_MS)          { motorTestStop("configurator silent"); return; }
		uint16_t stop = fw_.stopValue();
		for (uint8_t ch = 0; ch < 8; ch++) {
			int8_t e = FourWay::channel(ch);
			if (e < 0 || e >= n_) continue;
			uint8_t i = (uint8_t)e; Thruster* th = th_[i];
			uint16_t v = fw_.motor(ch);
			bool owned = mtOwned_ >> i & 1;
			if (!owned) {
				if (v <= stop) continue;
				bool idle = idle_ >> i & 1;                                   // held at zero: free to take
				if (!idle && (escs::spinArmed(i) || escs::spinArming(i))) continue;   // the CLI has it
				if (!idle) th->arm();
				idle_ &= ~(1u << i); mtOwned_ |= 1u << i; mtArmedMs_ = millis();
				console::out().printf("# motor test: arming ESC %u (%s)\n", i, th->spinMode());
			} else if (!escs::spinArmed(i) && !escs::spinArming(i)) {   // stopped elsewhere (CLI disarm)
				mtOwned_ &= ~(1u << i); continue;
			}
			// 1000..2000 (stop 1000) or 3D 1000..1500..2000 -> the CLI's raw units
			int raw;
			if (th->reversible()) raw = fw_.threeD() ? (int(v) - 1500) * 2 : (v > 1000 ? (int(v) - 1000) : 0);
			else                  raw = fw_.threeD() ? (v > 1500 ? (int(v) - 1500) * 4 : 0) : (v > 1000 ? (int(v) - 1000) * 2 : 0);
			th->setRaw(raw);
		}
	}

	// A motor-test value (1000..2000, or 3D 1000..1500..2000 when the configurator was told 3D) as a
	// servo pulse for ESC i, by that ESC's own direction mode — the same mapping as the DShot test.
	uint16_t pwmFromTest(uint8_t i, uint16_t v) {
		if (escs::knownReversible(i)) return fw_.threeD() ? v : (v > 1000 ? 1500 + (v - 1000) / 2 : 1500);
		return fw_.threeD() ? (v > 1500 ? 1000 + (v - 1500) * 2 : 1000) : v;
	}

	// Which ESCs are wired (escs::presentMask) and which of them run AM32 (escs::armMask), kept
	// across reboots. Wired is set by `scan` or `escs`; AM32 is learned on every bootloader connect,
	// and tells the passthrough how long to wait for that ESC right after a reboot.
	static constexpr const char* PRESENT_PATH = "/cfg/present";
	void loadPresent() {
		File f = LittleFS.open(PRESENT_PATH, "r");
		if (!f) return;
		uint32_t m = 0, a = 0;
		if (f.read((uint8_t*)&m, sizeof m) == sizeof m) escs::setPresent(m);
		if (f.read((uint8_t*)&a, sizeof a) == sizeof a) escs::setArmMask(a);   // absent in older files
		f.close();
		armSaved_ = escs::armMask();
		console::out().print("# escs: "); printPresent();
	}
	void savePresent(uint32_t m) {
		escs::setPresent(m);
		uint32_t a = armSaved_ = escs::armMask();
		File f = LittleFS.open(PRESENT_PATH, "w");
		if (f) { f.write((const uint8_t*)&m, sizeof m); f.write((const uint8_t*)&a, sizeof a); f.close(); }
	}
	void printPresent() {
		console::out().printf("escs|%u/%u|", escs::presentCount(), n_);
		bool first = true;
		for (uint8_t i = 0; i < n_; i++) if (escs::isPresent(i)) { console::out().printf(first ? "%u" : ",%u", i); first = false; }
		console::out().println();
	}

#if ESC_LCD
	// STATUS LCD. With an ESC armed (or arming) it shows the lowest-index one in detail:
	//   0A T 600 R 1115       index, a=arming / A=armed, DShot value sent, rpm (--- once stale > 0.5 s)
	//   11.2V c 11  32C       EDT volts, raw current code (the AM32 fork's code x0.05 A), ESC temp
	// otherwise every ESC's state (off / BL = in its bootloader / FL = flashing) and the mode.
	// The text is rebuilt at 5 Hz; Lcd1602::poll() trickles the changed characters out.
	static constexpr uint8_t LCD_D_[8] = ESC_LCD_D;
	Lcd1602  lcd_{ESC_LCD_RS, ESC_LCD_RW, ESC_LCD_E, LCD_D_};
	uint32_t lcdMs_ = 0;

	void lcdPoll() {
		lcd_.poll();
		if (millis() - lcdMs_ < 200) return;
		lcdMs_ = millis();
		char r0[24], r1[24];
		int shown = -1;
		for (uint8_t i = 0; i < n_ && shown < 0; i++) if (escs::spinArmed(i) || escs::spinArming(i)) shown = i;
		if (shown >= 0) {
			uint8_t i = (uint8_t)shown;
			escs::Telem t;
			bool tele = escs::spinTele(i, t);
			bool fresh = tele && t.valid && millis() - t.rpmStampMs < 500;
			if (fresh) snprintf(r0, sizeof r0, "%u%c T%4u R%5lu", i, escs::spinArmed(i) ? 'A' : 'a',
			                    escs::spinTarget(i), (unsigned long)t.rpm);
			else       snprintf(r0, sizeof r0, "%u%c T%4u R  ---", i, escs::spinArmed(i) ? 'A' : 'a',
			                    escs::spinTarget(i));
			if (tele) snprintf(r1, sizeof r1, "%4.1fV c%3lu %3luC", t.voltage, (unsigned long)t.current,
			                   (unsigned long)t.tempC);
			else      snprintf(r1, sizeof r1, "no telemetry");
		} else if (fw_.active()) {
			snprintf(r0, sizeof r0, "4WAY passthrough");
			if (fw_.selected() >= 0) snprintf(r1, sizeof r1, "ESC %d in BL", fw_.selected());
			else                     snprintf(r1, sizeof r1, "configurator");
		} else {
			int k = 0;
			// one cell per ESC: index + state (- idle, B bootloader, F flashing, x not wired)
			for (uint8_t i = 0; i < n_ && k < 16; i++) {
				bool fl = (fl_ == FL_START || fl_ == FL_RUN) && flIdx_ == i;
				k += snprintf(r0 + k, sizeof r0 - k, "%u%c ", i,
				              fl ? 'F' : escs::inBootloader(i) ? 'B' : escs::isPresent(i) ? '-' : 'x');
			}
#if ESC_WIFI_STA
			// S/D = mode, then the LAN address to point the host at; * = a TCP client is attached
			if (net_.connected()) snprintf(r1, sizeof r1, "%c %s%s", mode_ == DRIVE ? 'D' : 'S',
			                               net_.ip().toString().c_str(), net_.hasClient() ? "*" : "");
			else snprintf(r1, sizeof r1, "%s  joining...", mode_ == DRIVE ? "DRIVE" : "SETUP");
#else
			snprintf(r1, sizeof r1, "%s%s", mode_ == DRIVE ? "DRIVE" : "SETUP", wifiUp_ ? "  wifi" : "");
#endif
		}
		lcd_.row(0, r0);
		lcd_.row(1, r1);
	}
#endif

	// ---- members (were esc_tool.cpp file-scope globals) ----
	FourWay         fw_;         // esc-configurator / AM32 configurator passthrough on the CLI stream
	Thruster**      th_;         // NOT owned — main declares the Thrusters and passes them in
	uint8_t         n_;          // how many
	As5600Tracker   enc_;
	uint32_t        armSaved_ = 0;      // escs::armMask() as last written to PRESENT_PATH
	bool            encOff_ = false;    // an ESC signal sits on GP16/17, so the encoder stays off
	Azimuth         az_;
	Mode            mode_   = SETUP;
	bool            wifiUp_ = false;
	WebServer       server_{80};

	// `curve` upload staging — ONE table at a time (a curve is uploaded interactively by one host tool),
	// so this lives here rather than costing every Thruster a second buffer. Nothing reaches the live
	// controller until `commit` validates the whole set.
	vel::CurvePoint stage_[CURVE_STAGE_CAP];
	vel::Regime     stageReg_[CURVE_STAGE_CAP];
	int             stageIdx_    = -1;      // which ESC the staged table is for (-1 = nothing staged)
	int             stageWant_   = 0;       // points declared by `begin`
	int             stageN_      = 0;       // points received so far
	bool            stageTagged_ = true;    // every point carried an explicit regime tag

	// firmware-flash state machine (verbatim from esc_tool.cpp, now members)
	char   hexBuf_[32768];
	size_t hexLen_ = 0;
	bool   hexOverflow_ = false, hexFresh_ = false;
	esc_flash::HexImage img_;
	enum FlashState { FL_IDLE, FL_START, FL_RUN, FL_OK, FL_ERR };
	volatile FlashState fl_ = FL_IDLE;
	uint8_t  flIdx_ = 0; bool flForce_ = false;
	uint16_t flPage_ = 0, flLast_ = 0, flTotal_ = 0, flDone_ = 0;
	char     flMsg_[140] = {0};

	// ---- Wi-Fi lifecycle ----
#if ESC_WIFI_STA
	// The station link belongs to net_ and stays up; SETUP/DRIVE only open/close the web UI on it.
	void wifiStart() { if (wifiUp_) return; server_.begin(); wifiUp_ = true; }
	void wifiStop()  { if (!wifiUp_) return; server_.stop(); wifiUp_ = false; }
	NetConsole net_{ESC_TCP_PORT};
#else
	void wifiStart() { if (wifiUp_) return; WiFi.mode(WIFI_AP); WiFi.softAP(ESC_AP_SSID, ESC_AP_PASS); server_.begin(); wifiUp_ = true; }
	void wifiStop()  { if (!wifiUp_) return; server_.stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); wifiUp_ = false; }
#endif
	void setMode(Mode m) {
		mode_ = m;
		if (m == SETUP) wifiStart();
		else { escs::spinStopAll(); wifiStop(); }     // DRIVE: stop spins on entry, radio off
	}

	// ================= Web UI handlers (SETUP mode) — ported from esc_tool.cpp =================
	void hIndex() { server_.send_P(200, "text/html", INDEX_HTML); }
	void hScan() {
		String j = "["; uint32_t m = 0;
		for (uint8_t i = 0; i < n_; i++) {
			escs::Info in; bool ok = th_[i]->scan(in); if (i) j += ",";
			if (ok) m |= 1u << i;
			if (!ok) { j += "{\"present\":false,\"pin\":"; j += Esc::pin(i); j += "}"; }
			else { char sg[8]; snprintf(sg,sizeof(sg),"%04X",in.sig);
				j += "{\"present\":true,\"pin\":"; j += in.pin; j += ",\"sig\":\""; j += sg;
				j += "\",\"layout\":\""; j += jesc(in.layout); j += "\",\"name\":\""; j += jesc(in.name);
				j += "\",\"fw\":\""; j += in.fwMain; j += "."; j += in.fwSub; j += "\"}"; }
		}
		j += "]"; savePresent(m); server_.send(200, "application/json", j);
	}
	void hRead() {
		int i = server_.arg("i").toInt(); uint8_t cfg[esc_setup::kEepromLen];
		if (i<0||i>=n_||!th_[i]->readConfig(cfg)) { server_.send(200,"application/json","{\"error\":\"no-connect\"}"); return; }
		esc_setup::Settings s; esc_setup::decode(cfg, esc_setup::kEepromLen, s);
		String j = "{\"name\":\""; j += jesc(s.name); j += "\",\"fw\":\""; j += s.mainRevision; j += "."; j += s.subRevision; j += "\",\"settings\":{";
		for (int k=0;k<NFIELD;k++){ if(k)j+=","; j+="\""; j+=FIELDS[k].name; j+="\":"; j+=cfg[FIELDS[k].off]; }
		j += "}}"; server_.send(200, "application/json", j);
	}
	void hSet() {
		int i = server_.arg("i").toInt();
		if (i<0||i>=n_) { server_.send(200,"application/json","{\"error\":\"bad-index\"}"); return; }
		uint16_t offs[NFIELD]; uint8_t vals[NFIELD]; int n=0;
		for (int k=0;k<NFIELD;k++) if (server_.hasArg(FIELDS[k].name)) { offs[n]=FIELDS[k].off; vals[n]=(uint8_t)server_.arg(FIELDS[k].name).toInt(); n++; }
		if (!n) { server_.send(200,"application/json","{\"error\":\"no-fields\"}"); return; }
		bool ch=false; int r = th_[i]->editConfig(offs, vals, n, ch);
		server_.send(200, "application/json", r<0 ? "{\"error\":\"write-failed\"}" : (ch?"{\"result\":\"written\"}":"{\"result\":\"unchanged\"}"));
	}
	void hRun()      { escs::release(); server_.send(200,"application/json","{\"result\":\"restarted\"}"); }
	void hArm()      { int i=server_.arg("i").toInt();
		if (i<0||i>=n_) { server_.send(200,"application/json","{\"ok\":false}"); return; }
		escs::Drive m = escs::Drive::AUTO; String md = server_.arg("mode");
		if (md=="normal") m=escs::Drive::NORMAL; else if (md=="bidir") m=escs::Drive::BIDIR;
		th_[i]->arm(m);
		String j="{\"ok\":true,\"mode\":\""; j+=th_[i]->spinMode();
		j+="\",\"rev\":"; j+=th_[i]->reversible()?"true":"false"; j+="}";
		server_.send(200,"application/json",j); }
	void hSpin()     { int i=server_.arg("i").toInt(); int v=server_.arg("v").toInt();   // armed only
		if (i>=0&&i<n_) th_[i]->setRaw(v);                      // RAW: setRaw picks thrust/throttle
		server_.send(200,"application/json","{\"ok\":true}"); }
	void hRpm()      { int i=server_.arg("i").toInt(); float v=server_.arg("v").toFloat();  // closed loop
		if (i<0||i>=n_) { server_.send(200,"application/json","{\"ok\":false}"); return; }
		if (!th_[i]->armed()) { server_.send(200,"application/json","{\"ok\":false,\"err\":\"not-armed\"}"); return; }
		th_[i]->setRpm(v); server_.send(200,"application/json","{\"ok\":true}"); }
	void hDisarm()   { int i=server_.arg("i").toInt(); if(i>=0&&i<n_) th_[i]->stop(); server_.send(200,"application/json","{\"ok\":true}"); }
	void hSpinStop() { escs::spinStopAll(); server_.send(200,"application/json","{\"ok\":true}"); }
	void hTele() {
		int i=server_.arg("i").toInt();
		if (i<0||i>=n_) { server_.send(200,"application/json","{\"ok\":false}"); return; }
		String j = "{\"ok\":true,\"armed\":"; j += th_[i]->armed()?"true":"false";
		j += ",\"mode\":\""; j += th_[i]->spinMode(); j += "\",\"rev\":"; j += th_[i]->reversible()?"true":"false";
		escs::Telem t;
		if (th_[i]->tele(t)) {                              // telemetry only on bidir DShot
			j += ",\"tele\":true,\"rpm\":"; j+=t.rpm; j+=",\"volt\":"; j+=t.voltage; j+=",\"amp\":"; j+=t.current;
			j += ",\"temp\":"; j+=t.tempC; j+=",\"stress\":"; j+=t.stress;
		} else j += ",\"tele\":false";
		j += "}"; server_.send(200,"application/json",j);
	}

	// ================= Firmware flash from the browser (state machine, verbatim from esc_tool) =====
	void flStop(FlashState s, const char* msg) {
		escs::release(); strncpy(flMsg_, msg, sizeof(flMsg_)-1); flMsg_[sizeof(flMsg_)-1]=0; fl_ = s;
	}
	// Flash one 512B page from the parsed image: app pages from data[], the eeprom page (0x1A00) from
	// identity[] (= the firmware's default config). Erase, write 2x256, read back, compare.
	bool flPage(uint8_t idx, uint16_t p) {
		using namespace esc_flash;
		uint8_t buf[kPageSize]; memset(buf, 0xFF, sizeof(buf)); bool any = false;
		if (p < kAppEnd) { for (uint16_t k=0;k<kPageSize;k++) if (img_.used[p+k]) { buf[k]=img_.data[p+k]; any=true; } }
		else { if (!img_.hasIdentity) return true; memcpy(buf, img_.identity, kPageSize); any = true; }
		if (!any) return true;
		if (!th_[idx]->erasePage(p)) return false;
		if (!th_[idx]->writeFlash(p, buf, 256) || !th_[idx]->writeFlash((uint16_t)(p+256), buf+256, 256)) return false;
		uint8_t rb[kPageSize];
		if (!th_[idx]->readFlash(p, rb, 256) || !th_[idx]->readFlash((uint16_t)(p+256), rb+256, 256)) return false;
		return memcmp(rb, buf, kPageSize) == 0;
	}
	void flashStep() {   // advance the flash job one step; called from poll()
		using namespace esc_flash;
		if (fl_ == FL_START) {
			escs::spinStopAll();
			escs::Info in;
			if (!th_[flIdx_]->connect(in)) { flStop(FL_ERR, "could not connect to ESC"); return; }
			Compat c = checkCompatibility(in.sig, in.layout, img_);
			if (!c.ok && !flForce_) { flStop(FL_ERR, c.detail); return; }
			flPage_  = (uint16_t)((img_.minAddr / kPageSize) * kPageSize);
			flLast_  = kEepromBase;                       // last page = the eeprom default-config page
			flTotal_ = (uint16_t)((flLast_ - flPage_) / kPageSize + 1);
			flDone_  = 0; fl_ = FL_RUN;
		} else if (fl_ == FL_RUN) {
			if (!flPage(flIdx_, flPage_)) { flStop(FL_ERR, "flash/verify failed"); return; }
			flDone_++;
			if (flPage_ >= flLast_) { flStop(FL_OK, "flashed + verified; default config applied"); return; }
			flPage_ += kPageSize;
		}
	}
	void hFlashUpload() {                          // receives the multipart file body in chunks
		HTTPUpload& up = server_.upload();
		if (up.status == UPLOAD_FILE_START) { hexLen_ = 0; hexOverflow_ = false; hexFresh_ = true; }
		else if (up.status == UPLOAD_FILE_WRITE) {
			if (hexLen_ + up.currentSize > sizeof(hexBuf_)) { hexOverflow_ = true; return; }
			memcpy(hexBuf_ + hexLen_, up.buf, up.currentSize); hexLen_ += up.currentSize;
		}
	}
	void hFlashStart() {                           // POST /api/flash?i=<idx>&force=<0|1> (after upload)
		if (fl_ == FL_RUN || fl_ == FL_START) { server_.send(200,"application/json","{\"ok\":false,\"err\":\"busy\"}"); return; }
		int i = server_.arg("i").toInt(); flForce_ = server_.arg("force") == "1";
		if (i < 0 || i >= n_) { server_.send(200,"application/json","{\"ok\":false,\"err\":\"bad-index\"}"); return; }
		if (hexOverflow_) { server_.send(200,"application/json","{\"ok\":false,\"err\":\"file too large\"}"); return; }
		if (!hexFresh_ || hexLen_ == 0) { server_.send(200,"application/json","{\"ok\":false,\"err\":\"no .hex uploaded\"}"); return; }
		hexFresh_ = false;
		const char* perr = nullptr;
		if (!esc_flash::parseIntelHex(hexBuf_, hexLen_, img_, &perr)) {
			String j="{\"ok\":false,\"err\":\""; j+=jesc(perr); j+="\"}"; server_.send(200,"application/json",j); return;
		}
		flIdx_ = (uint8_t)i; flMsg_[0]=0; fl_ = FL_START;
		server_.send(200,"application/json","{\"ok\":true}");
	}
	void hFlashStatus() {
		const char* st = fl_==FL_IDLE?"idle":fl_==FL_START?"start":fl_==FL_RUN?"run":fl_==FL_OK?"ok":"err";
		String j = "{\"state\":\""; j+=st; j+="\",\"done\":"; j+=flDone_; j+=",\"total\":"; j+=flTotal_;
		j += ",\"msg\":\""; j+=jesc(flMsg_); j+="\"}"; server_.send(200,"application/json",j);
	}

	// ---- On-device firmware library (LittleFS) ----
	static String fwSafe(const String& in) {                 // -> safe short filename [A-Za-z0-9._-], <=24
		String o; for (uint16_t k=0; k<in.length() && o.length()<24; k++) {
			char c=in[k]; o += (isalnum(c)||c=='.'||c=='_'||c=='-') ? c : '_';
		}
		return o.length() ? o : String("fw");
	}
	static String fwHex(const String& n){ return String(FW_DIR)+"/"+n+".hex"; }
	static String fwTag(const String& n){ return String(FW_DIR)+"/"+n+".tag"; }

	void hFwSave() {   // POST /api/fwsave?name=<label>  (hex body via hFlashUpload); validate + store
		if (hexOverflow_) { server_.send(200,"application/json","{\"ok\":false,\"err\":\"file too large\"}"); return; }
		if (!hexFresh_ || hexLen_ == 0) { server_.send(200,"application/json","{\"ok\":false,\"err\":\"no .hex uploaded\"}"); return; }
		hexFresh_ = false;
		String name = fwSafe(server_.arg("name"));
		const char* perr = nullptr;
		if (!esc_flash::parseIntelHex(hexBuf_, hexLen_, img_, &perr)) {
			String j="{\"ok\":false,\"err\":\""; j+=jesc(perr); j+="\"}"; server_.send(200,"application/json",j); return;
		}
		LittleFS.mkdir(FW_DIR);
		File f = LittleFS.open(fwHex(name), "w");
		if (!f) { server_.send(200,"application/json","{\"ok\":false,\"err\":\"fs write failed\"}"); return; }
		f.write((const uint8_t*)hexBuf_, hexLen_); f.close();
		File t = LittleFS.open(fwTag(name), "w"); if (t) { t.print(img_.fwLayoutTag); t.close(); }
		String j="{\"ok\":true,\"name\":\""; j+=jesc(name.c_str()); j+="\",\"layout\":\""; j+=jesc(img_.fwLayoutTag); j+="\"}";
		server_.send(200,"application/json",j);
	}
	void hFwList() {   // GET /api/fwlist -> [{name,layout,size}]
		String j = "["; bool first = true;
		Dir dir = LittleFS.openDir(FW_DIR);
		while (dir.next()) {
			String fn = dir.fileName(); int sl = fn.lastIndexOf('/'); if (sl>=0) fn = fn.substring(sl+1);
			if (!fn.endsWith(".hex")) continue;
			String name = fn.substring(0, fn.length()-4), layout;
			File t = LittleFS.open(fwTag(name), "r"); if (t) { layout = t.readString(); layout.trim(); t.close(); }
			if (!first) j += ","; first = false;
			j += "{\"name\":\""; j+=jesc(name.c_str()); j+="\",\"layout\":\""; j+=jesc(layout.c_str());
			j += "\",\"size\":"; j+=(unsigned)dir.fileSize(); j+="}";
		}
		j += "]"; server_.send(200,"application/json",j);
	}
	void hFlashStored() {   // POST /api/flashstored?name=<label>&i=<idx>&force=<0|1>
		if (fl_ == FL_RUN || fl_ == FL_START) { server_.send(200,"application/json","{\"ok\":false,\"err\":\"busy\"}"); return; }
		int i = server_.arg("i").toInt(); flForce_ = server_.arg("force") == "1";
		if (i < 0 || i >= n_) { server_.send(200,"application/json","{\"ok\":false,\"err\":\"bad-index\"}"); return; }
		File f = LittleFS.open(fwHex(fwSafe(server_.arg("name"))), "r");
		if (!f) { server_.send(200,"application/json","{\"ok\":false,\"err\":\"not found\"}"); return; }
		hexLen_ = f.read((uint8_t*)hexBuf_, sizeof(hexBuf_)); f.close();
		const char* perr = nullptr;
		if (!hexLen_ || !esc_flash::parseIntelHex(hexBuf_, hexLen_, img_, &perr)) {
			String j="{\"ok\":false,\"err\":\""; j+=jesc(hexLen_?perr:"empty"); j+="\"}"; server_.send(200,"application/json",j); return;
		}
		flIdx_ = (uint8_t)i; flMsg_[0]=0; fl_ = FL_START;
		server_.send(200,"application/json","{\"ok\":true}");
	}
	void hFwDelete() {   // POST /api/fwdelete?name=<label>
		String n = fwSafe(server_.arg("name"));
		LittleFS.remove(fwHex(n)); LittleFS.remove(fwTag(n));
		server_.send(200,"application/json","{\"ok\":true}");
	}

	// ================= USB-serial API (always) — ported VERBATIM from esc_tool.cpp =================
	// Every command string and response literal is unchanged (host/esctool.py keeps working); the
	// `esc.` facade calls are retargeted to th_[i]->/escs::, and NEW commands (rpm, gain) are appended.
	void handleSerial() {
		static char line[600]; static uint16_t len = 0; static uint8_t flbuf[256];
		while (console::out().available()) {
			int c = console::out().read();
			if (fw_.feed((uint8_t)c, len == 0)) continue;   // stock configurator: MSP / 4-way passthrough
			if (c == '\r') continue;
			if (c != '\n') { if (len < sizeof(line)-1) line[len++]=(char)c; continue; }
			line[len]='\0'; len=0;
			linkLastMs_ = millis(); linkTripped_ = false;   // any command line = the host is alive
			char* cmd = strtok(line, " "); if (!cmd) continue;
			auto argi = []() { char* a=strtok(nullptr," "); return a?atoi(a):-1; };

			if (!strcmp(cmd,"ping")) { console::out().println("id esc_tool v1"); console::out().println("ok"); }
			else if (!strcmp(cmd,"pins")) { console::out().printf("pins %u", n_); for(uint8_t i=0;i<n_;i++)console::out().printf(" %u",Esc::pin(i)); console::out().println(); console::out().println("ok"); }
			else if (!strcmp(cmd,"fwlist")) { Dir dir=LittleFS.openDir(FW_DIR); int n=0;   // list on-device firmware library
				while(dir.next()){ String fn=dir.fileName(); int sl=fn.lastIndexOf('/'); if(sl>=0)fn=fn.substring(sl+1);
					if(fn.endsWith(".hex")){ console::out().printf("fw| %s  %u bytes\n", fn.c_str(), (unsigned)dir.fileSize()); n++; } }
				console::out().printf("fwlist %d\n", n); console::out().println("ok"); }
#if ESC_WIFI_STA
			else if (!strcmp(cmd,"net")) { console::out().printf("net|%s|%s|%s:%u|%s\n", net_.ssid(),
				net_.connected() ? "up" : "down", net_.ip().toString().c_str(), net_.port(),
				net_.hasClient() ? "client" : "-"); console::out().println("ok"); }
#endif
			else if (!strcmp(cmd,"mode")) { char* a=strtok(nullptr," ");
				if (a && !strcmp(a,"drive")) setMode(DRIVE); else if (a && !strcmp(a,"setup")) setMode(SETUP);
				console::out().printf("mode %s (wifi %s)\n", mode_==SETUP?"setup":"drive", wifiUp_?"on":"off"); console::out().println("ok"); }
			else if (!strcmp(cmd,"scan")) { uint32_t m=0; for(uint8_t i=0;i<n_;i++){ escs::Info in;
				if(!th_[i]->scan(in)) console::out().printf("esc|%u|%u|0\n",i,Esc::pin(i));
				else { m|=1u<<i; console::out().printf("esc|%u|%u|1|%04X|%u|%s|%s|%u.%u\n",i,in.pin,in.sig,in.bootVer,in.layout,in.name,in.fwMain,in.fwSub); } }
				savePresent(m); printPresent(); console::out().println("ok"); }
			else if (!strcmp(cmd,"preset")) presetCmd();
			else if (!strcmp(cmd,"idle")) { char* a=strtok(nullptr," ");   // idle [on|off]: zero-throttle hold for AM32
				if (a && !strcmp(a,"off")) { idleOn_=false; idleStop(); } else if (a && !strcmp(a,"on")) idleOn_=true;
				console::out().printf("idle|%s|held=%lx\n", idleOn_?"on":"off", (unsigned long)idle_); console::out().println("ok"); }
			else if (!strcmp(cmd,"escs")) { char* a=strtok(nullptr," ");   // escs [all | <i> <i> ...]: which are wired
				if (a) { uint32_t m=0; bool bad=false;
					if (!strcmp(a,"all")) m=~0u;
					else for (; a; a=strtok(nullptr," ")) { int i=atoi(a); if(i<0||i>=n_) bad=true; else m|=1u<<i; }
					if (bad) { console::out().println("err bad-index"); continue; }
					savePresent(m); }
				printPresent(); console::out().println("ok"); }
			else if (!strcmp(cmd,"read")) { int i=argi(); uint8_t cfg[esc_setup::kEepromLen];
				if(i<0||i>=n_) console::out().println("err bad-index");
				else if(!th_[i]->readConfig(cfg)) console::out().println("err no-connect");
				else { console::out().print("cfg|"); printHex(cfg,esc_setup::kEepromLen); console::out().println(); console::out().println("ok"); } }
			else if (!strcmp(cmd,"enter")) { int i=argi(); escs::Info in;
				if(i<0||i>=n_) console::out().println("err bad-index");
				else if(!th_[i]->connect(in)) console::out().println("err no-connect");
				else { console::out().printf("dev|%04X|%u|%u\n",in.sig,in.bootVer,in.bootPages); console::out().println("ok"); } }
			else if (!strcmp(cmd,"run")||!strcmp(cmd,"disconnect")) { escs::release(); console::out().println("ok"); }
			else if (!strcmp(cmd,"editpage")) { int i=argi(); char* ovr=strtok(nullptr," ");
				if(i<0||i>=n_||!ovr){ console::out().println("err bad-args"); continue; }
				uint16_t offs[160]; uint8_t vals[160]; int n=0; bool bad=false;
				for(char* tok=strtok(ovr,",");tok&&!bad;tok=strtok(nullptr,",")){ char* col=strchr(tok,':');
					if(!col||n>=160){bad=true;break;} *col='\0'; long off=strtol(tok,nullptr,16),val=strtol(col+1,nullptr,16);
					if(off<0||off>=(long)esc_setup::kPageLen||val<0||val>255){bad=true;break;} offs[n]=(uint16_t)off; vals[n]=(uint8_t)val; n++; }
				if(bad){ console::out().println("err bad-override"); continue; }
				bool ch=false; int r=th_[i]->editConfig(offs,vals,n,ch);
				if(r==-1)console::out().println("err no-connect"); else if(r==-2)console::out().println("err read-failed");
				else if(r==-3)console::out().println("err bad-override"); else if(r==-4)console::out().println("err write-verify-failed");
				else if(r==0){ console::out().println("unchanged (flash write skipped)"); console::out().println("ok"); }
				else { console::out().printf("edited %d byte(s)\n",n); console::out().println("ok"); } }
			else if (!strcmp(cmd,"erase")) { int i=argi(); char* ad=strtok(nullptr," ");
				if(i<0||i>=n_||!ad) console::out().println("err bad-args");
				else console::out().println(th_[i]->erasePage((uint16_t)strtol(ad,nullptr,16))?"ok":"err erase-failed"); }
			else if (!strcmp(cmd,"writeflash")) { int i=argi(); char* ad=strtok(nullptr," "); char* hx=strtok(nullptr," ");
				if(i<0||i>=n_||!ad||!hx){ console::out().println("err bad-args"); continue; }
				int n=parseHex(hx,flbuf,sizeof(flbuf));
				if(n<=0)console::out().println("err bad-hex");
				else console::out().println(th_[i]->writeFlash((uint16_t)strtol(ad,nullptr,16),flbuf,(uint16_t)n)?"ok":"err write-failed"); }
			else if (!strcmp(cmd,"blprobe")) { int i=argi(); char* h=strtok(nullptr," ");   // blprobe <i> [holdMs]
				if(i<0||i>=n_){ console::out().println("err bad-index"); continue; }
				uint8_t raw[8]; uint32_t e,l,t; int n=escs::probe((uint8_t)i, h?(uint32_t)atoi(h):2600, raw, e, l, t);
				console::out().printf("blprobe|n=%d|raw=",n); printHex(raw,8);
				console::out().printf("|edges=%lu|low=%lu/%lu\n",(unsigned long)e,(unsigned long)l,(unsigned long)t); console::out().println("ok"); }
			else if (!strcmp(cmd,"readflash")) { int i=argi(); char* ad=strtok(nullptr," "); char* ln=strtok(nullptr," ");
				int rl=ln?atoi(ln):-1;
				if(i<0||i>=n_||!ad||rl<1||rl>(int)sizeof(flbuf)) console::out().println("err bad-args");
				else if(!th_[i]->readFlash((uint16_t)strtol(ad,nullptr,16),flbuf,(uint16_t)rl)) console::out().println("err read-failed");
				else { console::out().print("data|"); printHex(flbuf,rl); console::out().println(); console::out().println("ok"); } }
			else if (!strcmp(cmd,"arm")) { int i=argi(); char* m=strtok(nullptr," ");   // arm <i> [normal|bidir]
				if(i<0||i>=n_) console::out().println("err bad-index");
				else { escs::Drive md=escs::Drive::AUTO;
					if(m&&!strcmp(m,"normal"))md=escs::Drive::NORMAL; else if(m&&!strcmp(m,"bidir"))md=escs::Drive::BIDIR;
					th_[i]->arm(md);
					if(!th_[i]->initOk()) { console::out().println("err dshot-init-failed (no free PIO SM?)"); continue; }
					console::out().printf("arming ~3s (mode %s, %s)\n",th_[i]->spinMode(),th_[i]->reversible()?"reversible":"one-way");
					console::out().println("ok"); } }
			else if (!strcmp(cmd,"throttle")||!strcmp(cmd,"spin")) { int i=argi(); char* v=strtok(nullptr," ");  // 0..2000
				if(i<0||i>=n_||!v) console::out().println("err bad-args");
				else if(!th_[i]->armed()) console::out().println("err not-armed");
				else { th_[i]->setRaw(atoi(v)); console::out().println("ok"); } }
			else if (!strcmp(cmd,"thrust")) { int i=argi(); char* v=strtok(nullptr," ");   // signed -1000..1000 (reversible/3D)
				if(i<0||i>=n_||!v) console::out().println("err bad-args");
				else if(!th_[i]->armed()) console::out().println("err not-armed");
				else { th_[i]->setRaw(atoi(v)); console::out().println("ok"); } }
			else if (!strcmp(cmd,"rpm")) { int i=argi(); char* v=strtok(nullptr," ");   // NEW: closed-loop velocity target (mech RPM, signed)
				if(i<0||i>=n_||!v) console::out().println("err bad-args");
				else if(!th_[i]->armed()) console::out().println("err not-armed");
				else { th_[i]->setRpm(atof(v)); console::out().println("ok"); } }
			else if (!strcmp(cmd,"gain")) { int i=argi(); char* g=strtok(nullptr," "); char* v=strtok(nullptr," ");   // gain <i> <kp|ki|kd|dtau|trim|slew> <v>
				if(i<0||i>=n_||!g||!v) console::out().println("err bad-args");
				else { float fv=atof(v);
					if(!strcmp(g,"kp")) th_[i]->vc.kp=fv; else if(!strcmp(g,"ki")) th_[i]->vc.ki=fv;
					else if(!strcmp(g,"kd")) th_[i]->vc.kd=fv; else if(!strcmp(g,"dtau")) th_[i]->vc.d_tau=fv;
					else if(!strcmp(g,"trim")) th_[i]->vc.trim_max=fv; else if(!strcmp(g,"slew")) th_[i]->vc.slew_rpm_s=fv;
					else if(!strcmp(g,"dob")) th_[i]->vc.dob=fv; else if(!strcmp(g,"dobtau")) th_[i]->vc.dob_tau=fv;
					else if(!strcmp(g,"dobmax")) th_[i]->vc.dob_max=fv; else if(!strcmp(g,"dobsettle")) th_[i]->vc.dob_settle_secs=fv;
					// [2026-08-01] The two STRUCTURAL authority limits, reachable from the wire on purpose:
					// both are ON by default and both are safety-relevant, so an operator who finds one
					// misbehaving on a bench must be able to back it out without a reflash.
					//   fbmax  <=0 => the derived max(trim, dobmax); a positive value overrides the total
					//   trackfrac 0 => disable the corroboration gate (pre-2026-08-01 liveness-only)
					// NOT persisted by `cfg save`, for the same reason warntemp/maxtemp are not: a power
					// cycle must always restore the guard, so disabling one is an explicit act each time.
					else if(!strcmp(g,"fbmax")) th_[i]->vc.fb_max=fv;
					else if(!strcmp(g,"trackfrac")) th_[i]->vc.track_min_frac=fv;
					// [B.7] thermal thresholds, in C. warntemp 0 disables the derate, maxtemp 0 disables
					// the temp check entirely. NOT persisted by `cfg save` on purpose (src/main.cpp): a
					// power cycle always restores the conservative compile-time band.
					else if(!strcmp(g,"warntemp")) th_[i]->vc.warn_temp=fv; else if(!strcmp(g,"maxtemp")) th_[i]->vc.max_temp=fv;
					else { console::out().println("err bad-gain (kp|ki|kd|dtau|trim|slew|dob|dobtau|dobmax|dobsettle|"
						"fbmax|trackfrac|warntemp|maxtemp)"); continue; }
					console::out().println("ok"); } }
			else if (!strcmp(cmd,"motor")) { int i=argi(); char* kv=strtok(nullptr," "); char* pp=strtok(nullptr," "); char* vv=strtok(nullptr," ");   // motor <i> <kv> <pp> <v>: regenerate the FF curve parametrically (MotorModel)
				float kvf=kv?atof(kv):0.0f, vvf=vv?atof(vv):0.0f; int ppi=pp?atoi(pp):0;   // [#6.2] validate VALUES not token presence
					if(i<0||i>=n_||kvf<=0.0f||ppi<1||vvf<=0.0f) console::out().println("err bad-args (motor <i> <kv> <pp> <v>): need kv>0 pp>=1 v>0");   // kv/pp/v=0 -> div-by-zero -> NaN into the live FF curve
					else if(th_[i]->armed()) console::out().println("err motor: disarm first (replaces the FF curve)");
				// Optional 4th arg = the ESC's `sine_cross_up`. The shape constants were fitted at 34, and
				// Cross_Up shifts the 6-step duty by a constant, so leaving it wrong silently biases the FF
				// (up to +44% at low cmd, measured) AND the soft-sensor that inverts the same map. Omitted
				// => keep the current value, so an existing `motor kv pp v` behaves exactly as before.
				else { char* cu=strtok(nullptr," "); float cuf=cu?atof(cu):0.0f;
					if(cu && (cuf < 1.0f || cuf > 255.0f)) { console::out().println("err motor: cross_up out of range (1..255)"); continue; }
					// false = no parametric curve exists for this cross_up (its handoff is refused, or is
					// above the cmd 700 the curve is sampled to — cross_up >= 48). The PREVIOUS curve is
					// kept and the seam metadata is still updated, so the soft-sensor gate stays correct;
					// what the operator must not get is an `ok` for a `motor` that installed nothing.
					// Say what DID land, because it is not nothing: mm_ (hence the soft-sensor duty map,
					// estimateV/estimateLoad and the reported kv/pp/v/cross_up) and the seam have all
					// moved, while motorConfigured() stays false and the ENGINE's pole count was not
					// touched — so a pp change in the same failing command leaves tele scaled by the old
					// one. Advice mirrors host/pico_esc/motor_model.py:to_profile()'s ValueError.
					if(!th_[i]->setMotor(kvf, ppi, vvf, cuf)) {
						console::out().printf("err motor: no 6-step curve at cross_up %.0f (its handoff is refused, or "
							"above the cmd 700 the curve is sampled to). FF curve UNCHANGED and still on the "
							"old pole count; soft-sensor duty map + seam DID move to kv=%s pp=%s v=%s. Use "
							"cross_up %d..%d (above that it hands off past the cmd 700 this curve is sampled "
							"to; below it the ESC never hands off), or push a measured curve "
							"(host/curvepush.py)\n", th_[i]->motorCrossUp(), kv, pp, vv,
							vel::minHandoffCrossUp(), vel::maxEmittableCrossUp(700.0f)); continue; }
					console::out().printf("motor|%d|kv=%s|pp=%s|v=%s|cross_up=%.0f\n", i, kv, pp, vv,
						th_[i]->motorCrossUp()); console::out().println("ok"); } }
			// crossup <i> <v>: the ESC's sine_cross_up, WITHOUT replacing the feed-forward curve.
			// It shifts the parametric 6-step duty map, which the soft-sensor inverts — so it matters
			// even when a MEASURED curve is driving the FF, and `motor` (which discards that curve) is
			// the wrong place to set it. Refused while armed: it can regenerate the parametric curve.
			else if (!strcmp(cmd,"crossup")) { int i=argi(); char* v=strtok(nullptr," ");
				if(i<0||i>=n_||!v) console::out().println("err bad-args (crossup <i> <sine_cross_up>)");
				else if(th_[i]->armed()) console::out().println("err crossup: disarm first");
				else if(!th_[i]->setCrossUp(atof(v))) console::out().println("err crossup: out of range (1..255)");
				// The curve field has to distinguish four outcomes, because three of them are NOT a fresh
				// parametric curve and the operator cannot tell from the outside: a measured curve is
				// kept as-is, no `motor` has ever run (the compile-time profile is live), the parametric
				// curve was regenerated, or it could NOT be (cross_up >= 48 has no curve below cmd 700) —
				// in which case the STALE cross_up-36-era curve is still driving the loop. It used to say
				// "regenerated" for all four.
				else { const char* cs = th_[i]->curveMeasured()   ? "measured (kept)"
				                      : !th_[i]->motorConfigured() ? "compile-time (run `motor` to generate one)"
				                      : th_[i]->curveStale()       ? "parametric (STALE)"
				                                                   : "parametric (regenerated)";
					console::out().printf("crossup|%d|cross_up=%.0f|curve=%s (persist with: cfg save)\n", i,
					th_[i]->motorCrossUp(), cs);
					// Gated on motorConfigured() TOO, not on curveStale() alone: after a first-ever `motor`
					// that failed, motorSet_ stays false while curveStale_ is true, so the bare gate made
					// this warning contradict the line above it — it claimed a PREVIOUS parametric curve
					// was driving the loop when the live FF is still the compile-time profile.
					// The remedy is a BAND, not "lower the byte": the low-side failure (cross_up < 34, the
					// firmware refuses to hand off at all) needs the byte RAISED, and only the high-side
					// one needs it lowered. 34..47 is the band that can actually emit a curve — 48..67 can
					// hand off but does it above the cmd 700 fillProfile samples to.
					if(th_[i]->curveStale() && th_[i]->motorConfigured())
						console::out().printf("# WARNING: no 6-step curve exists at cross_up %.0f, so the FF is still the "
							"one generated at the PREVIOUS cross_up — it under-commands and the loop may never "
							"reach 6-step. Push a measured curve (host/curvepush.py), or use cross_up %d..%d\n",
							th_[i]->motorCrossUp(), vel::minHandoffCrossUp(),
							vel::maxEmittableCrossUp(700.0f));
					// The duty map moved by ~0.042/unit, and `sense vcal` is a MULTIPLICATIVE scale on the
					// estimate that inverts it — so a stored vbattScale calibrated at the old cross_up is
					// now wrong by the same shift. Say so; it is not recoverable by inspection.
					console::out().println("# note: soft-sensor duty map moved — re-run `sense <i> vcal <V>`");
					console::out().println("ok"); } }
			else if (!strcmp(cmd,"sense")) { int i=argi(); char* sub=strtok(nullptr," ");   // sense <i>: soft-sensor — infer supply V + relative load from eRPM (no voltage/current/force sensor)
				if(i<0||i>=n_) console::out().println("err bad-args (sense <i> [zero|vcal <V>])");
					// [B.6] the refusal now NAMES the blocking condition (why=): the gate is stricter than
					// the old rpm-only test, so "not BEMF-live" alone would send an operator hunting the
					// wrong thing. The captured COMMAND is echoed because `load` is only comparable within
					// a band around it — the caller has to know where the baseline was taken.
					else if(sub && !strcmp(sub,"zero")) { if(!th_[i]->senseZero()) console::out().printf("err sense-zero: soft-sensor not valid here (why=%s, need |cmd|>=%.0f) — spin steady into 6-step first\n", th_[i]->senseWhy(), th_[i]->senseCmdFloor()); else { console::out().printf("sense|%d|zeroed vref=%.2f|zerocmd=%.0f\n", i, th_[i]->senseVref(), th_[i]->senseZeroCmd()); console::out().println("ok"); } }
					else if(sub && !strcmp(sub,"vcal")) { char* v=strtok(nullptr," "); float kv=v?atof(v):0.0f;   // [vbatt] calibration to a known battery voltage (per power cycle unless `cfg save`d)
						if(kv<=0.0f) console::out().println("err sense-vcal: need a known battery voltage (sense <i> vcal 11.2)");
						// [#7] parity with `sense zero`: refuse instead of silently no-op'ing when there is no
						// rolling-max estimate yet to calibrate against (the CLI used to print ok + vbatt=0.00).
						// [B.6] the rolling max only advances inside the gate's LIGHT-LOAD tier (vbok), so
						// "no estimate yet" now specifically means that tier never opened — say which one.
						else if(!th_[i]->senseVcal(kv)) console::out().printf("err sense-vcal: no vbatt estimate yet — the light-load tier (vbok) has never opened (vbok=%d why=%s); spin steady, lightly loaded, above the 6-step floor first\n", th_[i]->senseVbattOk()?1:0, th_[i]->senseWhy());
						else { console::out().printf("sense|%d|vcal to %.2f V -> vbatt=%.2f (persist with: cfg save)\n", i, kv, th_[i]->senseVbatt()); console::out().println("ok"); } }
				else {
					// [B.8] The four gated fields print `n/a`, never `0.00`, when they have no reading —
					// see optNum() above for why the two had to stop being the same token. Each carries
					// its OWN predicate, because they open at different times: vest needs the validity
					// window, vref needs `sense zero` to have run, load needs both plus the off-point
					// band, and vbatt needs the light-load tier. Everything below this line (dhat, trim,
					// cmd, temp…) is a genuine live number whose zero IS a measurement — those stay numeric.
					char b0[12], b1[12], b2[12], b3[12];
					const char* ve = optNum(b0, sizeof b0, th_[i]->senseValid(),    th_[i]->senseVoltage());
					const char* vr = optNum(b1, sizeof b1, th_[i]->senseVrefHas(),  th_[i]->senseVref());
					const char* ld = optNum(b2, sizeof b2, th_[i]->senseLoadHas(),  th_[i]->senseLoad());
					const char* vb = optNum(b3, sizeof b3, th_[i]->senseVbattHas(), th_[i]->senseVbatt());
					// dhat = the DOB's load estimate in rpm (0 when the observer is off or gated). Exposed
					// alongside `load` because it is the same physical quantity in different units, and it
					// is what an upper layer (attitude / allocation) actually wants per thruster.
					// trim/cmd = the CONTROL EFFORT. rpm tracking is blind to plant drift by design (the
					// loop hides it), the effort is not: at a fixed setpoint, a drifting R/KV shows up as
					// trim walking away from its cold value long before rpm moves. cmd = ff(setpoint)+trim
					// is logged too so a drift trace is self-contained (no need to re-evaluate the FF).
					// [B.6/B.7] New fields are APPENDED, never inserted: the prefix and the existing key
					// ORDER are a contract with host/driftsoak.py, loadtest.py and vbatt.py (all three
					// parse key=value, but two of them slice by position first).
					//   why      = which condition is closing the gate (ok|notlive|cmdlow|dutyhigh|rpmlow|
					//              settling). Mandatory: the gate is stricter than the old one, so a
					//              legitimate valid=0 must not look like a regression.
					//   cmdfloor = the |command| needed to OPEN the gate (derived from the seam, not typed)
					//   loadok   = `load` is comparable to the zero point; vbok = light-load (vcal) tier
					//   temp/tpeak/tlim/tstate = the thermal limiter: last ESC MCU temperature (NOT the
					//              winding), its high-water mark, the command-ceiling scale it is applying
					//              (1.00 = none) and its state (none=never seen, ok, derate, stale=lost it).
					console::out().printf("sense|%d|vest=%s|vref=%s|load=%s|vbatt=%s|valid=%d|floor=%.0f|dhat=%.1f|settled=%.2f|trim=%.1f|cmd=%d"
					              "|why=%s|cmdfloor=%.0f|loadok=%d|vbok=%d|temp=%.0f|tpeak=%.0f|tlim=%.2f|tstate=%s\n",
						i, ve, vr, ld, vb, th_[i]->senseValid()?1:0, th_[i]->senseRpmFloor(),
						th_[i]->vc.dhat(), th_[i]->vc.settled(),
						th_[i]->vc.trim(), th_[i]->vc.command(),
						th_[i]->senseWhy(), th_[i]->senseCmdFloor(),
						th_[i]->senseLoadOk()?1:0, th_[i]->senseVbattOk()?1:0,
						th_[i]->vc.temp(), th_[i]->vc.peakTemp(), th_[i]->vc.tempLimit(),
						vel::tempStateName(th_[i]->vc.tempState())); console::out().println("ok"); } }
			// curve: upload a MEASURED feed-forward curve (velcal/sysid -> host/curvepush.py), replacing
			// the parametric `motor` prediction. Sent ONE POINT PER LINE on purpose: the input buffer is
			// 600 B and silently truncates past it, so a whole table on one line would be a data-loss
			// trap. Staged then committed atomically, so a partial upload never goes live.
			//   curve <i> begin <n> | curve <i> add <thrust> <rpm> [s|l] | curve <i> commit <pp> <up> <dn>
			//   curve <i>            -> dump the live curve for readback
			else if (!strcmp(cmd,"curve")) { int i=argi(); char* sub=strtok(nullptr," ");
				if(i<0||i>=n_) console::out().println("err bad-args (curve <i> [begin <n>|add <t> <r> [s|l]|commit <pp> <up> <dn>])");
				// [#11-1] report the pp the LIVE CURVE carries, not the parametric model's — otherwise a
				// commit/model mismatch is invisible to the host's read-back check, which is the one
				// place it could have been caught.
				else if(!sub) { console::out().printf("curve|%d|n=%d|measured=%d|pp=%d|up=%.0f|dn=%.0f\n", i, th_[i]->curveCount(),
						th_[i]->curveMeasured()?1:0, th_[i]->curvePolePairs(), th_[i]->crossUpErpm(), th_[i]->crossDnErpm());
					for(int k=0;k<th_[i]->curveCount();k++) console::out().printf("pt|%d|%.1f|%.1f|%c\n", k,
						th_[i]->curvePoint(k).thrust, th_[i]->curvePoint(k).rpm,
						th_[i]->curveRegime(k)==vel::Regime::LINE?'l':'s');
					console::out().println("ok"); }
				else if(!strcmp(sub,"begin")) { char* v=strtok(nullptr," "); int want=v?atoi(v):0;
					if(want<2||want>CURVE_STAGE_CAP) console::out().printf("err curve-begin: need 2..%d points\n", CURVE_STAGE_CAP);
					else { stageIdx_=i; stageWant_=want; stageN_=0; stageTagged_=true;
						console::out().printf("curve|%d|staging %d point(s)\n", i, want); console::out().println("ok"); } }
				else if(!strcmp(sub,"add")) { char* t=strtok(nullptr," "); char* r=strtok(nullptr," "); char* g=strtok(nullptr," ");
					if(stageIdx_!=i||stageWant_<=0) console::out().println("err curve-add: run `curve <i> begin <n>` first");
					else if(stageN_>=stageWant_) console::out().println("err curve-add: more points than begin declared");
					else if(!t||!r) console::out().println("err curve-add: need <thrust> <rpm>");
					else { stage_[stageN_].thrust=atof(t); stage_[stageN_].rpm=atof(r);
						// regime tag optional: 'l'/'s' from the profile YAML, else derived at commit
						stageReg_[stageN_] = (g && (*g=='l'||*g=='L')) ? vel::Regime::LINE : vel::Regime::SINE;
						if(!g) stageTagged_=false;
						stageN_++; console::out().println("ok"); } }
				else if(!strcmp(sub,"commit")) { char* pp=strtok(nullptr," "); char* up=strtok(nullptr," "); char* dn=strtok(nullptr," ");
					int ppi=pp?atoi(pp):0; float upf=up?atof(up):0.0f, dnf=dn?atof(dn):0.0f;
					if(stageIdx_!=i||stageWant_<=0) console::out().println("err curve-commit: nothing staged");
					else if(stageN_!=stageWant_) console::out().printf("err curve-commit: staged %d of %d points\n", stageN_, stageWant_);
					else if(ppi<1||upf<=0.0f) console::out().println("err curve-commit: need <pole_pairs> <up_erpm> <dn_erpm>");
					else if(th_[i]->armed()) console::out().println("err curve-commit: disarm first (replaces the FF curve)");
					else if(!th_[i]->setCurve(stage_, stageN_, ppi, upf, dnf, stageTagged_?stageReg_:nullptr))
						console::out().println("err curve-commit: rejected (need >=2 points, thrust strictly increasing, rpm non-decreasing)");
					else { console::out().printf("curve|%d|installed %d point(s) pp=%d up=%.0f dn=%.0f\n", i, stageN_, ppi, upf, dnf);
						// [#11-1] A motor has ONE pole count, so a curve committed at a different pp than
						// `motor` configured is an operator error (usually curvepush without --motor).
						// The curve's pp wins from here on; say so rather than letting it diverge quietly.
						if(th_[i]->motorConfigured() && th_[i]->motorPolePairs()!=ppi)
							console::out().printf("# warn: curve pp=%d differs from `motor` pp=%d — the curve's wins\n",
							              ppi, th_[i]->motorPolePairs());
						stageWant_=0; stageN_=0; stageIdx_=-1; stageTagged_=true; console::out().println("ok"); } }
				else console::out().println("err bad-args (curve <i> [begin <n>|add <t> <r> [s|l]|commit <pp> <up> <dn>])"); }
			// cfg: persist / restore the RUNTIME config (motor identity, gains, DOB, vbatt scale) on the
			// Pico's LittleFS. Whole-config, EXPLICIT — there is no autosave (flash wear) and no per-ESC
			// slice (one blob keeps the CRC meaningful). `cfg load` replaces the live FF curve, so it is
			// refused while any ESC is armed.
			else if (!strcmp(cmd,"cfg")) { char* sub=strtok(nullptr," ");
				if(!sub || !strcmp(sub,"show")) {
					// [B.7] warntemp/maxtemp are APPENDED (host `cfg` parsers are key=value) and are
					// deliberately NOT part of the persisted Record — see the note in src/main.cpp.
					for(uint8_t i=0;i<n_;i++) console::out().printf("cfg|%u|motor=%d|kv=%.0f|pp=%d|v=%.2f|kp=%.3f|ki=%.3f|kd=%.4f|trim=%.0f|slew=%.0f|dob=%.2f|dobtau=%.3f|dobmax=%.0f|dobsettle=%.2f|vscale=%.4f|cross_up=%.0f|warntemp=%.0f|maxtemp=%.0f\n",
						i, th_[i]->motorConfigured()?1:0, th_[i]->motorKv(), th_[i]->motorPolePairs(), th_[i]->motorVolts(),
						th_[i]->vc.kp, th_[i]->vc.ki, th_[i]->vc.kd, th_[i]->vc.trim_max, th_[i]->vc.slew_rpm_s,
						th_[i]->vc.dob, th_[i]->vc.dob_tau, th_[i]->vc.dob_max, th_[i]->vc.dob_settle_secs,
						th_[i]->vbattScale(), th_[i]->motorCrossUp(),
						th_[i]->vc.warn_temp, th_[i]->vc.max_temp);
					console::out().printf("cfg|stored=%d\n", settings::exists()?1:0); console::out().println("ok"); }
				else if(!strcmp(sub,"save")) { if(settings::save(th_,n_)) { console::out().printf("cfg|saved %u ESC(s) to %s\n", n_, settings::CFG_PATH); console::out().println("ok"); }
					else console::out().println("err cfg-save: filesystem write failed"); }
				else if(!strcmp(sub,"load")) { bool anyArmed=false; for(uint8_t i=0;i<n_;i++) if(th_[i]->armed()) anyArmed=true;
					if(anyArmed) console::out().println("err cfg-load: disarm first (replaces the FF curve)");
					else { uint16_t sv=0; int k=settings::load(th_,n_,&sv); if(k>0) { console::out().printf("cfg|restored %d ESC(s)\n", k); console::out().println("ok"); }
						else if(sv && sv!=settings::CFG_VERSION) console::out().printf("err cfg-load: stored config is v%u, this build wants v%u — re-save\n", sv, settings::CFG_VERSION);
						else console::out().println("err cfg-load: no valid stored config (magic/size/CRC)"); } }
				else if(!strcmp(sub,"clear")) { if(settings::clear()) console::out().println("ok"); else console::out().println("err cfg-clear: nothing stored"); }
				else console::out().println("err bad-args (cfg [show|save|load|clear])"); }
			// Stop-all goes through each Thruster rather than escs::spinStopAll() alone: the engine call
			// silences the DShot frames but leaves the objects' submode / soft-sensor validity untouched,
			// so a bare `disarm` used to leave `sense` reporting a stale estimate as valid. The trailing
			// spinStopAll() is then redundant for the bound Thrusters and DELIBERATELY kept: it also
			// covers any escs:: index with no Thruster attached, and a stop path is worth belt-and-braces.
			else if (!strcmp(cmd,"disarm")||!strcmp(cmd,"spinstop")) { int i=argi();
				if(i<0) { for(uint8_t k=0;k<n_;k++) th_[k]->stop(); escs::spinStopAll(); }
				else if(i<n_) th_[i]->stop();
				console::out().println("ok"); }
			else if (!strcmp(cmd,"link")) { char* v=strtok(nullptr," ");   // link [<ms>|off] — host-link deadman, see linkWatch()
				if(v) linkTimeoutMs_ = strcmp(v,"off") ? (uint32_t)atol(v) : 0;
				console::out().printf("link|timeout_ms=%lu\n", (unsigned long)linkTimeoutMs_); console::out().println("ok"); }
			else if (!strcmp(cmd,"pwm") || !strcmp(cmd,"pwmch")) {   // servo PWM (50 Hz) instead of DShot: pwm <i> <us|stop>
				bool byCh = cmd[3] == 'c';                                // pwmch <motor-test channel> <us|stop> (web tool)
				int a=argi(); int i = byCh ? (a >= 0 ? FourWay::channel((uint8_t)a) : -1) : a; char* v=strtok(nullptr," ");
				if(i<0||i>=n_||!v) console::out().println("err bad-args");
				else if(fw_.active()||fl_==FL_START||fl_==FL_RUN) console::out().println("err busy");
				else if(!strcmp(v,"stop")){ th_[i]->stop(); console::out().printf("pwm|%d|off\n",i); console::out().println("ok"); }
				else { if(!escs::pwmActive(i)){ th_[i]->stop(); idle_&=~(1u<<i);
						if(!escs::pwmBegin(i)){ console::out().println("err not-read (scan first: the stop pulse depends on 3D)"); continue; } }
					int us = constrain(atoi(v),0,3000);
					if (byCh) us = pwmFromTest(i, (uint16_t)us);              // the motor test's value, mapped per ESC
					escs::pwmSet(i,(uint16_t)us);                             // fed like `throttle`: 500 ms deadman -> stop pulse
					console::out().printf("pwm|%d|%uus|%s\n",i,escs::pwmTarget(i),escs::pwmArming(i)?"arming":"armed"); console::out().println("ok"); } }
			else if (!strcmp(cmd,"enc")) {   // AS5600 read: enc|raw|ang|deg|md|ml|mh|agc|mag  (err if no sensor)
				if(!enc_.present()) console::out().println("err no-encoder");
				else { uint8_t st=enc_.status(); int raw=enc_.raw();
					console::out().printf("enc|%d|%d|%.1f|%d|%d|%d|%d|%d\n", raw, raw, raw*360.0f/4096.0f,
					             (st>>5)&1,(st>>4)&1,(st>>3)&1, enc_.agc(), enc_.mag()); console::out().println("ok"); } }
			else if (!strcmp(cmd,"encv")) {  // de-aliased encoder velocity: encv|accum|rpm|samples|md (err if none)
				if(!enc_.present()) console::out().println("err no-encoder");
				else { uint8_t st=enc_.status();
					console::out().printf("encv|%ld|%.2f|%lu|%d\n", (long)enc_.accum(), (double)enc_.rpm(),
					             (unsigned long)enc_.samples(), (st>>5)&1); console::out().println("ok"); } }
			// encd [reset] — the tracker's ALIASING diagnostics. `encv` reports a velocity; this
			// reports whether that velocity can be believed. The unwrap in As5600Tracker::poll folds
			// each step into +/-2048 counts, so any stall longer than half a revolution of travel
			// silently subtracts a whole turn — no error, no `miss`, and barely a dent in the mean
			// sample rate. On 2026-08-15 the mean rate (1004 at rest -> 626/s under load) was read as
			// proof the tracker was healthy; it is not, and this is the number that decides.
			//   encd|samples|miss|maxgap_us|maxstep|alias_margin
			// alias_margin = 2048/maxstep: how many times the worst observed step would have to grow
			// before the accumulator starts losing revolutions. Below ~2 the data is suspect.
			else if (!strcmp(cmd,"encd")) {
				if(!enc_.present()) console::out().println("err no-encoder");
				else { char* r=strtok(nullptr," ");
					uint16_t ms=enc_.maxStep();
					console::out().printf("encd|%lu|%lu|%lu|%u|%.2f\n",
					              (unsigned long)enc_.samples(), (unsigned long)enc_.misses(),
					              (unsigned long)enc_.maxGapUs(), ms,
					              ms ? 2048.0 / (double)ms : 999.0);
					if(r && !strcmp(r,"reset")) enc_.resetDiag();
					console::out().println("ok"); } }
			// i2cscan — what is ACTUALLY on I2C0, and is the bus even usable?
			// `enc` only ever probes 0x36, so a silent sensor and a shorted bus look identical from it.
			// This separates them: it reports the IDLE LEVELS of SDA/SCL first (both must read HIGH — a
			// line stuck LOW with nothing driving it means a dead device is holding it or the pull-up is
			// missing/shorted, and NO address can ever ACK in that state), then walks 0x08..0x77.
			// An AS5600 is fixed at 0x36; an AS5600L is configurable, so a device at some OTHER address
			// is a different part, not a wiring fault.
			else if (!strcmp(cmd,"az")) {   // az on <carrier> <deg> | off | tgt <deg> | gain <k> <v> | status
				char* sub=strtok(nullptr," ");
				if(!sub || !strcmp(sub,"status")) {
					console::out().printf("az|%d|tgt=%.2f|pos=%.2f|err=%.2f|vel=%.1f|s=%.1f|kp=%.2f|ki=%.2f|kd=%.2f|"
					              "kff=%.4f|trim=%.1f|smax=%.0f|vmax=%.0f|carrier=%.0f|maxrate=%.0f|verrclamp=%.0f|"
					              "absolute=%d|zeroraw=%ld|head=%.2f|stop=%s\n",
					              az_.on()?1:0, az_.target(), az_.pos(), az_.target()-az_.pos(), az_.vel(),
					              az_.s(), az_.kp, az_.ki, az_.kd, az_.kff, az_.s_trim, az_.smax, az_.vmax,
					              az_.carrier, az_.max_rate, az_.verr_clamp,
					              az_.absolute?1:0, (long)az_.zero_raw,
					              az_.zero_raw < 0 ? 0.0 : (double)az_.heading(),
					              Azimuth::stopWhyName(az_.stopWhy()));
					console::out().println("ok"); }
				// az zero [<deg>|show|clear] — the HULL heading reference. `az zero` declares the pod's
				// current physical heading to be 0 (or <deg>), and persists it as a RAW_ANGLE count so
				// it survives a power cycle. Without it, `absolute` mode refuses to start.
				else if(!strcmp(sub,"zero")) {
					char* g=strtok(nullptr," ");
					az_.bind(th_[0], n_>1?th_[1]:th_[0], &enc_);
					if(g && !strcmp(g,"show")) {
						// A missing reference prints as `unset`, never as a number: `%f` of NAN reads
						// as "inf"/"nan" and both look like a heading that was measured.
						if(az_.zero_raw < 0)
							console::out().printf("az|zero|raw=-1|stored=%d|head=unset|absolute=%d\n",
							              settings::azZeroStored()?1:0, az_.absolute?1:0);
						else
							console::out().printf("az|zero|raw=%ld|stored=%d|head=%.2f|absolute=%d\n",
							              (long)az_.zero_raw, settings::azZeroStored()?1:0,
							              (double)az_.heading(), az_.absolute?1:0);
						console::out().println("ok"); }
					else if(g && !strcmp(g,"clear")) {
						az_.zero_raw = -1; settings::azClearZero();
						console::out().println("ok"); }
					else if(az_.on()) console::out().println("err az: stop the loop before re-zeroing");
					else if(!enc_.present()) console::out().println("err az: no encoder");
					else if(!az_.setZeroHere(g?atof(g):0.0f)) console::out().println("err az: no encoder");
					else if(!settings::azSaveZero(az_.zero_raw)) console::out().println("err az: zero not saved");
					else { console::out().printf("az|zero|raw=%ld|head=%.2f\n",
					                     (long)az_.zero_raw, (double)az_.heading());
					       console::out().println("ok"); } }
				else if(!strcmp(sub,"on")) {
					char* c=strtok(nullptr," "); char* g=strtok(nullptr," ");
					if(n_<2) { console::out().println("err az: needs two ESCs"); }
					else if(!th_[0]->armed()||!th_[1]->armed()) console::out().println("err az: arm 0 and arm 1 first");
					else { if(c) az_.carrier=atof(c);
						az_.bind(th_[0], th_[1], &enc_);
						// Name the refusal: "no encoder" for a missing reference sent the operator
						// looking at the I2C bus for a calibration that was simply never taken.
						if(az_.absolute && az_.zero_raw < 0)
							console::out().println("err az: absolute mode has no heading zero — run `az zero` first");
						else if(!az_.begin(g?atof(g):0.0f)) console::out().println("err az: no encoder");
						else { console::out().printf("az|on|carrier=%.0f|tgt=%.2f\n", az_.carrier, az_.target()); console::out().println("ok"); } } }
				else if(!strcmp(sub,"off")) { az_.stop(); console::out().println("ok"); }
				// az kick <counts> <ms> — inject a KNOWN disturbance while the loop runs, so rejection
				// can be measured instead of hand-poked.
				else if(!strcmp(sub,"kick")) {
					char* c=strtok(nullptr," "); char* m=strtok(nullptr," ");
					if(!c||!m) console::out().println("err az: kick <counts> <ms>");
					else if(!az_.kick(atof(c), (uint32_t)atol(m)))
						console::out().println("err az: kick needs the hold running, 0 < ms <= 2000");
					else { console::out().printf("az|kick|s=%.1f|ms=%ld\n", atof(c), atol(m));
					       console::out().println("ok"); } }
				else if(!strcmp(sub,"tgt")) { char* g=strtok(nullptr," ");
					if(!g) console::out().println("err az: tgt <deg>"); else { az_.setTarget(atof(g)); console::out().println("ok"); } }
				else if(!strcmp(sub,"gain")) { char* k=strtok(nullptr," "); char* v=strtok(nullptr," ");
					if(!k||!v) { console::out().println("err az: gain <kp|ki|kd|kff|trim|smax|vmax|carrier|maxrate|verrclamp|signb|spinsecs|spinsmax> <v>"); }
					else { float f=atof(v);
						if(!strcmp(k,"kp")) az_.kp=f; else if(!strcmp(k,"ki")) az_.ki=f;
						else if(!strcmp(k,"kd")) az_.kd=f; else if(!strcmp(k,"kff")) az_.kff=f;
						else if(!strcmp(k,"trim")) az_.s_trim=f; else if(!strcmp(k,"smax")) az_.smax=f;
						else if(!strcmp(k,"vmax")) az_.vmax=f; else if(!strcmp(k,"carrier")) az_.carrier=f;
						else if(!strcmp(k,"maxrate")) az_.max_rate=f;
						else if(!strcmp(k,"spinmaxrate")) az_.spin_max_rate=f;
						else if(!strcmp(k,"spinguard")) az_.spin_guard_secs=f;
						else if(!strcmp(k,"wraperr")) az_.wrap_err=(f>0.5f);
						else if(!strcmp(k,"absolute")) az_.absolute=(f>0.5f);
						else if(!strcmp(k,"snaprpm")) az_.snap_rpm=f;
						else if(!strcmp(k,"snaphold")) az_.snap_hold=f;
						else if(!strcmp(k,"spinrmax")) az_.spin_r_max=f;
						else if(!strcmp(k,"spincorrmargin")) az_.spin_corr_margin=f;
						else if(!strcmp(k,"spinasymsecs")) az_.spin_asym_secs=f;
						else if(!strcmp(k,"tgtslew")) az_.tgt_slew=f;
						else if(!strcmp(k,"rpmfloor")) az_.rpm_floor=f;
						else if(!strcmp(k,"crosszero")) az_.cross_zero_offset=(f>0.5f);
						else if(!strcmp(k,"thrustrpm")) az_.thrust_rpm=f;
						else if(!strcmp(k,"thrustsecs")) az_.thrust_secs=f;
						else if(!strcmp(k,"oversecs")) az_.over_secs=f;
						else if(!strcmp(k,"hardrate")) az_.hard_rate=f;
						else if(!strcmp(k,"mixswap")) az_.mix_swap=(f>0.5f);
						else if(!strcmp(k,"nullratio")) az_.null_ratio=f;
						else if(!strcmp(k,"kipos")) az_.ki_pos=f;
						else if(!strcmp(k,"spinokfrac")) az_.spin_ok_frac=f;
						else if(!strcmp(k,"trimsecs")) az_.trim_secs=f;
						else if(!strcmp(k,"iposmax")) az_.ipos_max=f;
						else if(!strcmp(k,"verrclamp")) az_.verr_clamp=f;
						else if(!strcmp(k,"signb")) az_.sign_b=(f<0)?-1:1;
						else if(!strcmp(k,"spinsecs")) az_.spin_secs=f;
						else if(!strcmp(k,"spinsmax")) az_.spin_smax=f;
							else if(!strcmp(k,"seamkeep")) az_.seam_keep=f;
							else if(!strcmp(k,"spinhold")) az_.spin_hold=f;
							else if(!strcmp(k,"spinrpm")) az_.spin_rpm=f;
							else if(!strcmp(k,"spinrpmok")) az_.spin_rpm_ok=f;
							else if(!strcmp(k,"rpmdrive")) az_.rpm_drive=(f>0.5f);
							else if(!strcmp(k,"rpmpercount")) az_.rpm_per_count=f;
							else if(!strcmp(k,"rpmoffmax")) az_.rpm_offset_max=f;
							else if(!strcmp(k,"rpmtrim")) az_.rpm_trim=f;
							else if(!strcmp(k,"vellpf")) az_.vel_lpf_hz=f;
							else if(!strcmp(k,"spinslew")) az_.spin_slew=f;
							else if(!strcmp(k,"holdslew")) az_.hold_slew=f;
						else { console::out().println("err az: unknown gain"); continue; }
						console::out().println("ok"); } }
					// az step <carrier> <s> <ms> — OPEN LOOP, for plant identification. No feedback:
					// both motors carried, a known s applied, the trace recording. Self-terminates.
					else if(!strcmp(sub,"step")) {
						char* c=strtok(nullptr," "); char* v=strtok(nullptr," "); char* t=strtok(nullptr," ");
						char* pr=strtok(nullptr," ");            // pre-roll at s=0, default 3000 ms
						if(!c||!v||!t) console::out().println("err az: step <carrier> <s> <ms> [pre_ms]");
						else if(n_<2) console::out().println("err az: needs two ESCs");
						else if(!th_[0]->armed()||!th_[1]->armed()) console::out().println("err az: arm 0 and arm 1 first");
						else { az_.bind(th_[0], th_[1], &enc_);
							if(!az_.beginStep(atof(c), atof(v), (uint32_t)atol(t),
							                  pr?(uint32_t)atol(pr):3000UL))
								console::out().println("err az: step needs an encoder, 0 < ms <= 10000, pre_ms <= 10000");
							// Report the CLAMPED step, not what was asked for: beginStep holds it inside
							// +/-smax, and a gain fit that divides by the requested value instead of the
							// delivered one is wrong by exactly that ratio.
							else { console::out().printf("az|step|carrier=%.0f|s=%.1f|asked=%.1f|ms=%ld|pre_ms=%ld\n",
							                     az_.carrier, (double)az_.stepS(), (double)atof(v), atol(t),
							                     pr?atol(pr):3000L);
							       console::out().println("ok"); } } }
					// az log [decim] — how often poll() writes to the trace ring (1 = every tick).
					else if(!strcmp(sub,"log")) { char* d=strtok(nullptr," ");
						if(d) az_.setDecim((uint16_t)atoi(d));
						console::out().printf("az|log|decim=%u|n=%u|cap=%u\n", az_.decim(), az_.traceCount(),
						              (unsigned)Azimuth::TRACE_N); console::out().println("ok"); }
					// az dump — the whole ring, oldest first. REFUSED while the loop is running: this
					// prints thousands of lines and poll() does not run meanwhile, which would both
					// freeze the loop and trip the spin deadman. Stop first, then read.
					// The header carries the FULL configuration on purpose. A sweep once inherited
					// vmax=120 from a previous trial and the result was recorded against the wrong
					// number; a trace that cannot say what produced it is not evidence.
					else if(!strcmp(sub,"dump")) {
						if(az_.on()) { console::out().println("err az: stop the loop first (az off) — dump blocks poll()"); }
						else {
							uint16_t n = az_.traceCount();
							console::out().printf("azd|n=%u|decim=%u|kp=%.3f|ki=%.3f|kd=%.3f|kff=%.5f|trim=%.1f|"
							              "smax=%.0f|vmax=%.0f|carrier=%.0f|maxrate=%.0f|verrclamp=%.0f|"
							              "signa=%d|signb=%d|seam=%d|tgt=%.2f|spinsecs=%.2f|spinsmax=%.0f|seamkeep=%.0f|spinrpm=%.0f|rpmdrive=%d|rpmpercount=%.2f|rpmoffmax=%.0f|rpmtrim=%.0f|vellpf=%.2f|spinslew=%.0f|holdslew=%.0f"
							              "|spinmaxrate=%.0f|spinguard=%.2f|wraperr=%d|rpmfloor=%.0f|crosszero=%d|thrustrpm=%.0f|thrustsecs=%.1f|oversecs=%.2f|hardrate=%.0f|mixswap=%d|nullratio=%.2f|kipos=%.4f|absolute=%d|zeroraw=%ld|step_s=%.1f|step_ms=%.1f|hand_ms=%.1f|kick_ms=%.1f\n",
							              n, az_.decim(), az_.kp, az_.ki, az_.kd, az_.kff, az_.s_trim,
							              az_.smax, az_.vmax, az_.carrier, az_.max_rate, az_.verr_clamp,
							              az_.sign_a, az_.sign_b, Azimuth::SEAM_CMD, az_.target(),
							              az_.spin_secs, az_.spin_smax, az_.seam_keep, az_.spin_rpm, az_.rpm_drive?1:0, az_.rpm_per_count, az_.rpm_offset_max, az_.rpm_trim, az_.vel_lpf_hz, az_.spin_slew, az_.hold_slew, az_.spin_max_rate, az_.spin_guard_secs, az_.wrap_err?1:0, az_.rpm_floor, az_.cross_zero_offset?1:0, az_.thrust_rpm, az_.thrust_secs, az_.over_secs, az_.hard_rate, az_.mix_swap?1:0, az_.null_ratio, az_.ki_pos, az_.absolute?1:0, (long)az_.zero_raw, az_.stepS(),
							              n ? (double)(int32_t)(az_.stepBeginUs() - az_.traceAt(0).t_us) / 1000.0
							                : 0.0,
							              (n && az_.handUs())
							                ? (double)(int32_t)(az_.handUs() - az_.traceAt(0).t_us) / 1000.0
							                : -1.0,
							              (n && az_.kickUs())
							                ? (double)(int32_t)(az_.kickUs() - az_.traceAt(0).t_us) / 1000.0
							                : -1.0);
							console::out().println("t_ms,pos,vel,s,ivel,carrier,rpm_a,rpm_b");
							uint32_t t0 = n ? az_.traceAt(0).t_us : 0;
							for (uint16_t i = 0; i < n; i++) {
								const Azimuth::Sample& x = az_.traceAt(i);
								console::out().printf("%.3f,%.2f,%.1f,%.1f,%.1f,%d,%d,%d\n",
								              (double)(uint32_t)(x.t_us - t0) / 1000.0,
								              (double)x.pos, (double)x.vel,
								              (double)x.s_q / 10.0, (double)x.ivel_q / 10.0, (int)x.c_q,
								              (int)x.ra, (int)x.rb);
							}
							console::out().println("ok"); } }
					else console::out().println("err az: on|off|tgt|gain|status|log|dump|step"); }
			else if (!strcmp(cmd,"i2cscan")) {
				// Probe the lines against an INTERNAL PULL-DOWN, not a bare input. Wire.begin() enables
				// the RP2040's internal pull-UPs on these pads and Wire.end() does not undo them, so a
				// plain digitalRead reports HIGH whether or not anything is attached — it measures our
				// own pull-up and says nothing. The internal pulls are ~50-80k and an I2C bus pull-up is
				// 2.2-10k, so a pull-down loses to a real external pull-up and wins against nothing:
				//   sda/scl = 1 -> a POWERED external pull-up is present (sensor board has power)
				//   sda/scl = 0 -> no external pull-up: unplugged, unpowered, or the line is not landed
				Wire.end();
				pinMode(16, INPUT_PULLDOWN); pinMode(17, INPUT_PULLDOWN);
				delay(2);                                     // let the RC of the bus settle
				int sda = digitalRead(16), scl = digitalRead(17);
				console::out().printf("i2c|pulldown|sda=%d|scl=%d\n", sda, scl);
				if (sda && scl) console::out().println("i2c|external pull-ups PRESENT and powered on both lines");
				else if (!sda && !scl) console::out().println("i2c|NO external pull-up on either line — the sensor "
				                                      "board is unplugged, unpowered, or neither line is landed");
				else {
					// Both pull-ups sit on the SAME VCC on an AS5600 breakout, so one line up and the
					// other down cannot be a power fault — the board HAS power and the dead line is
					// simply not landed. Worth calling out because SCL open makes every address fail to
					// ACK, which looks exactly like a dead part.
					console::out().printf("i2c|only %s has an external pull-up — %s is OPEN, and both pull-ups "
					              "share one VCC, so the board IS powered and that line is just not "
					              "connected\n", sda ? "SDA" : "SCL", sda ? "SCL (GP17, physical pin 22)"
					                                                     : "SDA (GP16, physical pin 21)");
					if (!scl) console::out().println("i2c|SCL is the clock: with it open NOTHING can ever ACK. "
					                         "Check GP17 = PHYSICAL PIN 22 (physical 17 is GP13).");
				}
				// then the idle levels with the pull-down removed: a line stuck LOW here is being held
				pinMode(16, INPUT); pinMode(17, INPUT);
				delayMicroseconds(200);
				console::out().printf("i2c|float|sda=%d|scl=%d\n", digitalRead(16), digitalRead(17));
				Wire.setSDA(16); Wire.setSCL(17); Wire.begin(); Wire.setClock(100000);
				int found = 0;
				for (uint8_t a = 0x08; a <= 0x77; a++) {
					Wire.beginTransmission(a);
					if (Wire.endTransmission() == 0) { console::out().printf("i2c|found|0x%02X\n", a); found++; }
				}
				console::out().printf("i2c|devices=%d\n", found);
				if (!found && sda && scl)
					console::out().println("i2c|pull-ups are powered but NOTHING answers — the part is dead, or it "
					               "is a different device than expected.");
				else if (!found && !sda && !scl)
					console::out().println("i2c|nothing answers and there is no external pull-up — check 3V3/GND at "
					               "the sensor and that SDA=GP16 (physical 21) / SCL=GP17 (physical 22).");
				else if (!found)
					console::out().println("i2c|nothing answers, but that is expected while a line is open — fix "
					               "the wiring above and re-run before suspecting the part.");
				Wire.setClock(400000);                        // restore the tracker's rate
				console::out().println("ok"); }
			else if (!strcmp(cmd,"tele")) { int i=argi(); escs::Telem t;
				if(i<0||i>=n_||!th_[i]->tele(t)) console::out().println("err no-telem");
				else { console::out().printf("tele|%lu|%.2f|%lu|%lu|%lu\n",(unsigned long)t.rpm,t.voltage,(unsigned long)t.current,(unsigned long)t.tempC,(unsigned long)t.stress); console::out().println("ok"); } }
			else console::out().println("err unknown-cmd");
		}
	}
};
