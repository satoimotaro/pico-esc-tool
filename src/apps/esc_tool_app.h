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
#include <Wire.h>
#include "esc.h"
#include "as5600.h"
#include "esc_flash.h"
#include "thruster.h"

// ---- small helpers (unchanged from esc_tool.cpp) ----
static void printHex(const uint8_t* p, uint16_t n) {
	for (uint16_t i = 0; i < n; i++) { Serial.print("0123456789ABCDEF"[p[i] >> 4]); Serial.print("0123456789ABCDEF"[p[i] & 0xF]); }
}
static int hexVal(char c) { if (c>='0'&&c<='9') return c-'0'; if (c>='A'&&c<='F') return c-'A'+10; if (c>='a'&&c<='f') return c-'a'+10; return -1; }
static int parseHex(const char* s, uint8_t* buf, int cap) {
	int n = 0; for (; s[0] && s[1]; s += 2) { if (n >= cap) return -1; int hi=hexVal(s[0]),lo=hexVal(s[1]); if(hi<0||lo<0) return -1; buf[n++]=(uint8_t)((hi<<4)|lo); } return n;
}
static String jesc(const char* s) { String o; for (; *s; s++) { if (*s=='"'||*s=='\\') o+='\\'; o+=*s; } return o; }

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

	void begin() {
		Serial.begin(115200);
#if ESC_MODE_PIN >= 0
		pinMode(ESC_MODE_PIN, INPUT_PULLUP);
#endif
		delay(50);
		if (!LittleFS.begin()) { LittleFS.format(); LittleFS.begin(); }   // firmware library store
		enc_.begin();                                                    // AS5600 encoder (I2C0, GP16/17)
		LittleFS.mkdir("/fw");

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
		handleSerial();
		enc_.poll();                                            // high-rate AS5600 sampling (self-gated)
		if (fl_ == FL_START || fl_ == FL_RUN) flashStep();      // flashing: one page per loop (no spin)
		else {
			for (uint8_t i = 0; i < n_; i++) th_[i]->poll();   // RPM submode closes the loop
			escs::spinPoll();                                          // keep DShot frames flowing
		}
		if (wifiUp_) server_.handleClient();
	}

	// core1 loop.
	void pollCore1() { escs::core1Poll(); }

private:
	// ---- members (were esc_tool.cpp file-scope globals) ----
	Thruster**      th_;         // NOT owned — main declares the Thrusters and passes them in
	uint8_t         n_;          // how many
	As5600Tracker   enc_;
	Mode            mode_   = SETUP;
	bool            wifiUp_ = false;
	WebServer       server_{80};

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
	void wifiStart() { if (wifiUp_) return; WiFi.mode(WIFI_AP); WiFi.softAP(ESC_AP_SSID, ESC_AP_PASS); server_.begin(); wifiUp_ = true; }
	void wifiStop()  { if (!wifiUp_) return; server_.stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); wifiUp_ = false; }
	void setMode(Mode m) {
		mode_ = m;
		if (m == SETUP) wifiStart();
		else { escs::spinStopAll(); wifiStop(); }     // DRIVE: stop spins on entry, radio off
	}

	// ================= Web UI handlers (SETUP mode) — ported from esc_tool.cpp =================
	void hIndex() { server_.send_P(200, "text/html", INDEX_HTML); }
	void hScan() {
		String j = "[";
		for (uint8_t i = 0; i < n_; i++) {
			escs::Info in; bool ok = th_[i]->scan(in); if (i) j += ",";
			if (!ok) { j += "{\"present\":false,\"pin\":"; j += Esc::pin(i); j += "}"; }
			else { char sg[8]; snprintf(sg,sizeof(sg),"%04X",in.sig);
				j += "{\"present\":true,\"pin\":"; j += in.pin; j += ",\"sig\":\""; j += sg;
				j += "\",\"layout\":\""; j += jesc(in.layout); j += "\",\"name\":\""; j += jesc(in.name);
				j += "\",\"fw\":\""; j += in.fwMain; j += "."; j += in.fwSub; j += "\"}"; }
		}
		j += "]"; server_.send(200, "application/json", j);
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
		while (Serial.available()) {
			int c = Serial.read();
			if (c == '\r') continue;
			if (c != '\n') { if (len < sizeof(line)-1) line[len++]=(char)c; continue; }
			line[len]='\0'; len=0;
			char* cmd = strtok(line, " "); if (!cmd) continue;
			auto argi = []() { char* a=strtok(nullptr," "); return a?atoi(a):-1; };

			if (!strcmp(cmd,"ping")) { Serial.println("id esc_tool v1"); Serial.println("ok"); }
			else if (!strcmp(cmd,"pins")) { Serial.printf("pins %u", n_); for(uint8_t i=0;i<n_;i++)Serial.printf(" %u",Esc::pin(i)); Serial.println(); Serial.println("ok"); }
			else if (!strcmp(cmd,"fwlist")) { Dir dir=LittleFS.openDir(FW_DIR); int n=0;   // list on-device firmware library
				while(dir.next()){ String fn=dir.fileName(); int sl=fn.lastIndexOf('/'); if(sl>=0)fn=fn.substring(sl+1);
					if(fn.endsWith(".hex")){ Serial.printf("fw| %s  %u bytes\n", fn.c_str(), (unsigned)dir.fileSize()); n++; } }
				Serial.printf("fwlist %d\n", n); Serial.println("ok"); }
			else if (!strcmp(cmd,"mode")) { char* a=strtok(nullptr," ");
				if (a && !strcmp(a,"drive")) setMode(DRIVE); else if (a && !strcmp(a,"setup")) setMode(SETUP);
				Serial.printf("mode %s (wifi %s)\n", mode_==SETUP?"setup":"drive", wifiUp_?"on":"off"); Serial.println("ok"); }
			else if (!strcmp(cmd,"scan")) { for(uint8_t i=0;i<n_;i++){ escs::Info in;
				if(!th_[i]->scan(in)) Serial.printf("esc|%u|%u|0\n",i,Esc::pin(i));
				else Serial.printf("esc|%u|%u|1|%04X|%u|%s|%s|%u.%u\n",i,in.pin,in.sig,in.bootVer,in.layout,in.name,in.fwMain,in.fwSub); } Serial.println("ok"); }
			else if (!strcmp(cmd,"read")) { int i=argi(); uint8_t cfg[esc_setup::kEepromLen];
				if(i<0||i>=n_) Serial.println("err bad-index");
				else if(!th_[i]->readConfig(cfg)) Serial.println("err no-connect");
				else { Serial.print("cfg|"); printHex(cfg,esc_setup::kEepromLen); Serial.println(); Serial.println("ok"); } }
			else if (!strcmp(cmd,"enter")) { int i=argi(); escs::Info in;
				if(i<0||i>=n_) Serial.println("err bad-index");
				else if(!th_[i]->connect(in)) Serial.println("err no-connect");
				else { Serial.printf("dev|%04X|%u|%u\n",in.sig,in.bootVer,in.bootPages); Serial.println("ok"); } }
			else if (!strcmp(cmd,"run")||!strcmp(cmd,"disconnect")) { escs::release(); Serial.println("ok"); }
			else if (!strcmp(cmd,"editpage")) { int i=argi(); char* ovr=strtok(nullptr," ");
				if(i<0||i>=n_||!ovr){ Serial.println("err bad-args"); continue; }
				uint16_t offs[160]; uint8_t vals[160]; int n=0; bool bad=false;
				for(char* tok=strtok(ovr,",");tok&&!bad;tok=strtok(nullptr,",")){ char* col=strchr(tok,':');
					if(!col||n>=160){bad=true;break;} *col='\0'; long off=strtol(tok,nullptr,16),val=strtol(col+1,nullptr,16);
					if(off<0||off>=(long)esc_setup::kPageLen||val<0||val>255){bad=true;break;} offs[n]=(uint16_t)off; vals[n]=(uint8_t)val; n++; }
				if(bad){ Serial.println("err bad-override"); continue; }
				bool ch=false; int r=th_[i]->editConfig(offs,vals,n,ch);
				if(r==-1)Serial.println("err no-connect"); else if(r==-2)Serial.println("err read-failed");
				else if(r==-3)Serial.println("err bad-override"); else if(r==-4)Serial.println("err write-verify-failed");
				else if(r==0){ Serial.println("unchanged (flash write skipped)"); Serial.println("ok"); }
				else { Serial.printf("edited %d byte(s)\n",n); Serial.println("ok"); } }
			else if (!strcmp(cmd,"erase")) { int i=argi(); char* ad=strtok(nullptr," ");
				if(i<0||i>=n_||!ad) Serial.println("err bad-args");
				else Serial.println(th_[i]->erasePage((uint16_t)strtol(ad,nullptr,16))?"ok":"err erase-failed"); }
			else if (!strcmp(cmd,"writeflash")) { int i=argi(); char* ad=strtok(nullptr," "); char* hx=strtok(nullptr," ");
				if(i<0||i>=n_||!ad||!hx){ Serial.println("err bad-args"); continue; }
				int n=parseHex(hx,flbuf,sizeof(flbuf));
				if(n<=0)Serial.println("err bad-hex");
				else Serial.println(th_[i]->writeFlash((uint16_t)strtol(ad,nullptr,16),flbuf,(uint16_t)n)?"ok":"err write-failed"); }
			else if (!strcmp(cmd,"readflash")) { int i=argi(); char* ad=strtok(nullptr," "); char* ln=strtok(nullptr," ");
				int rl=ln?atoi(ln):-1;
				if(i<0||i>=n_||!ad||rl<1||rl>(int)sizeof(flbuf)) Serial.println("err bad-args");
				else if(!th_[i]->readFlash((uint16_t)strtol(ad,nullptr,16),flbuf,(uint16_t)rl)) Serial.println("err read-failed");
				else { Serial.print("data|"); printHex(flbuf,rl); Serial.println(); Serial.println("ok"); } }
			else if (!strcmp(cmd,"arm")) { int i=argi(); char* m=strtok(nullptr," ");   // arm <i> [normal|bidir]
				if(i<0||i>=n_) Serial.println("err bad-index");
				else { escs::Drive md=escs::Drive::AUTO;
					if(m&&!strcmp(m,"normal"))md=escs::Drive::NORMAL; else if(m&&!strcmp(m,"bidir"))md=escs::Drive::BIDIR;
					th_[i]->arm(md);
					if(!th_[i]->initOk()) { Serial.println("err dshot-init-failed (no free PIO SM?)"); continue; }
					Serial.printf("arming ~3s (mode %s, %s)\n",th_[i]->spinMode(),th_[i]->reversible()?"reversible":"one-way");
					Serial.println("ok"); } }
			else if (!strcmp(cmd,"throttle")||!strcmp(cmd,"spin")) { int i=argi(); char* v=strtok(nullptr," ");  // 0..2000
				if(i<0||i>=n_||!v) Serial.println("err bad-args");
				else if(!th_[i]->armed()) Serial.println("err not-armed");
				else { th_[i]->setRaw(atoi(v)); Serial.println("ok"); } }
			else if (!strcmp(cmd,"thrust")) { int i=argi(); char* v=strtok(nullptr," ");   // signed -1000..1000 (reversible/3D)
				if(i<0||i>=n_||!v) Serial.println("err bad-args");
				else if(!th_[i]->armed()) Serial.println("err not-armed");
				else { th_[i]->setRaw(atoi(v)); Serial.println("ok"); } }
			else if (!strcmp(cmd,"rpm")) { int i=argi(); char* v=strtok(nullptr," ");   // NEW: closed-loop velocity target (mech RPM, signed)
				if(i<0||i>=n_||!v) Serial.println("err bad-args");
				else if(!th_[i]->armed()) Serial.println("err not-armed");
				else { th_[i]->setRpm(atof(v)); Serial.println("ok"); } }
			else if (!strcmp(cmd,"gain")) { int i=argi(); char* g=strtok(nullptr," "); char* v=strtok(nullptr," ");   // gain <i> <kp|ki|kd|dtau|trim|slew> <v>
				if(i<0||i>=n_||!g||!v) Serial.println("err bad-args");
				else { float fv=atof(v);
					if(!strcmp(g,"kp")) th_[i]->vc.kp=fv; else if(!strcmp(g,"ki")) th_[i]->vc.ki=fv;
					else if(!strcmp(g,"kd")) th_[i]->vc.kd=fv; else if(!strcmp(g,"dtau")) th_[i]->vc.d_tau=fv;
					else if(!strcmp(g,"trim")) th_[i]->vc.trim_max=fv; else if(!strcmp(g,"slew")) th_[i]->vc.slew_rpm_s=fv;
					else { Serial.println("err bad-gain (kp|ki|kd|dtau|trim|slew)"); continue; }
					Serial.println("ok"); } }
			else if (!strcmp(cmd,"disarm")||!strcmp(cmd,"spinstop")) { int i=argi(); if(i<0) escs::spinStopAll(); else if(i<n_) th_[i]->stop(); Serial.println("ok"); }
			else if (!strcmp(cmd,"pwm")) { int i=argi(); char* v=strtok(nullptr," ");   // servo-PWM test (50Hz, hw PWM, not DShot); pwm <i> <us|stop>
				if(i<0||i>=n_||!v) Serial.println("err bad-args");
				else if(!strcmp(v,"stop")){ analogWrite(Esc::pin(i),0); pinMode(Esc::pin(i),INPUT); Serial.println("ok pwm-stop (reboot to use DShot again)"); }
				else { th_[i]->stop(); escs::release();          // free DShot + run the ESC app
					int us=atoi(v); if(us<900)us=900; if(us>2100)us=2100;
					analogWriteFreq(50); analogWriteRange(20000); analogWrite(Esc::pin(i),(uint32_t)us);   // 20000us period => value=us
					Serial.printf("pwm|%d|%dus\n",i,us); Serial.println("ok"); } }
			else if (!strcmp(cmd,"enc")) {   // AS5600 read: enc|raw|ang|deg|md|ml|mh|agc|mag  (err if no sensor)
				if(!enc_.present()) Serial.println("err no-encoder");
				else { uint8_t st=enc_.status(); int raw=enc_.raw();
					Serial.printf("enc|%d|%d|%.1f|%d|%d|%d|%d|%d\n", raw, raw, raw*360.0f/4096.0f,
					             (st>>5)&1,(st>>4)&1,(st>>3)&1, enc_.agc(), enc_.mag()); Serial.println("ok"); } }
			else if (!strcmp(cmd,"encv")) {  // de-aliased encoder velocity: encv|accum|rpm|samples|md (err if none)
				if(!enc_.present()) Serial.println("err no-encoder");
				else { uint8_t st=enc_.status();
					Serial.printf("encv|%ld|%.2f|%lu|%d\n", (long)enc_.accum(), (double)enc_.rpm(),
					             (unsigned long)enc_.samples(), (st>>5)&1); Serial.println("ok"); } }
			else if (!strcmp(cmd,"tele")) { int i=argi(); escs::Telem t;
				if(i<0||i>=n_||!th_[i]->tele(t)) Serial.println("err no-telem");
				else { Serial.printf("tele|%lu|%.2f|%lu|%lu|%lu\n",(unsigned long)t.rpm,t.voltage,(unsigned long)t.current,(unsigned long)t.tempC,(unsigned long)t.stress); Serial.println("ok"); } }
			else Serial.println("err unknown-cmd");
		}
	}
};
