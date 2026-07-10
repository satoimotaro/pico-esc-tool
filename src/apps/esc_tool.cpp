// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_tool — unified RP2040 ESC-tool firmware (Pico W). One build, no reflashing between jobs:
//   * config / flash BLHeli-S ESCs over the 1-wire bootloader (shared esc_session),
//   * spin thrusters over DShot with telemetry (individual test / per-thruster drive),
//   * driven from the USB-serial CLI (host/esctool.py) AND, in SETUP mode, a Wi-Fi web UI.
//
// MODES (the "setup vs drive" split):
//   SETUP (default): Wi-Fi AP ON (browser configurator + individual spin test) + full serial API.
//   DRIVE          : Wi-Fi OFF (save power / no RF) — accepts per-thruster commands; deadman armed.
// Mode is chosen at boot by MODE_PIN (internal pull-up: unwired/HIGH => SETUP, tie LOW => DRIVE, so
// leaving it unconnected is fine) and can be changed at runtime with the `mode` command.
//
// The Pico is a generic per-thruster driver: cmd_vel->thruster mixing lives on the host/Pi (keeps
// it RL/sim-friendly). Wi-Fi is a surface/bench affordance (2.4 GHz does not travel underwater).
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esc_session.h"

#define MODE_PIN 22                    // tie LOW at boot for DRIVE; unconnected (pull-up) => SETUP
static const char* AP_SSID = "pico-esc-tool";
static const char* AP_PASS = "esctool1234";   // >= 8 chars (WPA2). Change before real use.

enum Mode { SETUP, DRIVE };
static Mode mode   = SETUP;
static bool wifiUp = false;
static WebServer server(80);

// ---- small helpers ----
static void printHex(const uint8_t* p, uint16_t n) {
	for (uint16_t i = 0; i < n; i++) { Serial.print("0123456789ABCDEF"[p[i] >> 4]); Serial.print("0123456789ABCDEF"[p[i] & 0xF]); }
}
static int hexVal(char c) { if (c>='0'&&c<='9') return c-'0'; if (c>='A'&&c<='F') return c-'A'+10; if (c>='a'&&c<='f') return c-'a'+10; return -1; }
static int parseHex(const char* s, uint8_t* buf, int cap) {
	int n = 0; for (; s[0] && s[1]; s += 2) { if (n >= cap) return -1; int hi=hexVal(s[0]),lo=hexVal(s[1]); if(hi<0||lo<0) return -1; buf[n++]=(uint8_t)((hi<<4)|lo); } return n;
}
static String jesc(const char* s) { String o; for (; *s; s++) { if (*s=='"'||*s=='\\') o+='\\'; o+=*s; } return o; }

// writable settings exposed in the web UI (raw byte per config offset)
struct Field { const char* name; uint16_t off; };
static const Field FIELDS[] = {
	{ "motor_direction",0x0B },{ "comm_timing",0x15 },{ "demag_compensation",0x1F },
	{ "startup_beep",0x05 },{ "beep_strength",0x1B },{ "beacon_strength",0x1C },
	{ "beacon_delay",0x1D },{ "temperature_protection",0x23 },
	{ "low_rpm_power_protection",0x24 },{ "brake_on_stop",0x27 },
};
static const int NFIELD = sizeof(FIELDS) / sizeof(FIELDS[0]);

// ---- Wi-Fi lifecycle ----
static void wifiStart() { if (wifiUp) return; WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID, AP_PASS); server.begin(); wifiUp = true; }
static void wifiStop()  { if (!wifiUp) return; server.stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); wifiUp = false; }
static void setMode(Mode m) {
	mode = m;
	if (m == SETUP) wifiStart();
	else { escs::spinStopAll(); wifiStop(); }     // DRIVE: stop spins on entry, radio off
}

// ================= Web UI (SETUP mode) =================
static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>pico-esc-tool</title>
<style>body{font-family:system-ui,sans-serif;margin:1em;max-width:680px}button{padding:.4em .8em;margin:.2em}
label{display:block;margin:.3em 0}input[type=number]{width:6em}.esc{border:1px solid #ccc;border-radius:8px;padding:.5em;margin:.5em 0}
#msg{color:#070}.stop{background:#c22;color:#fff}</style></head><body><h1>pico-esc-tool</h1>
<button onclick=scan()>Scan</button><button onclick=disc()>Disconnect</button>
<button class=stop onclick=stopAll()>STOP ALL</button><span id=msg></span>
<div id=list></div><div id=cfg></div>
<script>
const g=id=>document.getElementById(id),msg=t=>g('msg').textContent=t;let tick=null;
async function scan(){msg('scanning...');let d=await(await fetch('/api/scan')).json();
 g('list').innerHTML=d.map((e,i)=>e.present?`<div class=esc><b>ESC ${i}</b> pin ${e.pin} sig ${e.sig} layout ${e.layout} "${e.name}" fw ${e.fw}
   <button onclick=read(${i})>Edit</button><button onclick=spinUI(${i})>Spin test</button></div>`
   :`<div class=esc>ESC ${i}: none</div>`).join('');msg('');}
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
  <p id=tele>not armed</p></div>`;}
async function armEsc(i){let a=await(await fetch('/api/arm?i='+i,{method:'POST'})).json();msg('arming ~3s...');
  let s=g('thr');
  if(a.rev){s.min=-1000;s.max=1000;}else{s.min=0;s.max=2000;}s.value=0;g('tv').textContent=0;
  g('smode').textContent='DShot: '+a.mode+(a.rev?' — reversible (−1000..+1000, 0=stop)':' — one-way (0..2000)');
  stopTick();
  tick=setInterval(async()=>{await fetch('/api/spin?i='+i+'&v='+g('thr').value,{method:'POST'});
   let t=await(await fetch('/api/tele?i='+i)).json();
   g('tele').textContent=t.ok?(`${t.armed?'ARMED':'arming...'}  `+(t.tele?`rpm ${t.rpm}  ${t.volt}V  ${t.amp}A  ${t.temp}C  stress ${t.stress}`:'(no telemetry — normal DShot)')):'no telemetry';},200);}
async function disarmEsc(i){stopTick();g('thr').value=0;g('tv').textContent=0;await fetch('/api/disarm?i='+i,{method:'POST'});msg('disarmed');}
function stopTick(){if(tick){clearInterval(tick);tick=null;}}
async function stopAll(){stopTick();await fetch('/api/spinstop',{method:'POST'});msg('stopped');}
async function disc(){stopTick();await fetch('/api/run',{method:'POST'});g('cfg').innerHTML='';msg('ESC restarted');}
scan();
</script></body></html>)HTML";

static void hIndex() { server.send_P(200, "text/html", INDEX_HTML); }
static void hScan() {
	String j = "[";
	for (uint8_t i = 0; i < escs::COUNT; i++) {
		escs::Info in; bool ok = escs::scan(i, in); if (i) j += ",";
		if (!ok) { j += "{\"present\":false,\"pin\":"; j += escs::PINS[i]; j += "}"; }
		else { char sg[8]; snprintf(sg,sizeof(sg),"%04X",in.sig);
			j += "{\"present\":true,\"pin\":"; j += in.pin; j += ",\"sig\":\""; j += sg;
			j += "\",\"layout\":\""; j += jesc(in.layout); j += "\",\"name\":\""; j += jesc(in.name);
			j += "\",\"fw\":\""; j += in.fwMain; j += "."; j += in.fwSub; j += "\"}"; }
	}
	j += "]"; server.send(200, "application/json", j);
}
static void hRead() {
	int i = server.arg("i").toInt(); uint8_t cfg[esc_setup::kEepromLen];
	if (i<0||i>=escs::COUNT||!escs::readConfig((uint8_t)i,cfg)) { server.send(200,"application/json","{\"error\":\"no-connect\"}"); return; }
	esc_setup::Settings s; esc_setup::decode(cfg, esc_setup::kEepromLen, s);
	String j = "{\"name\":\""; j += jesc(s.name); j += "\",\"fw\":\""; j += s.mainRevision; j += "."; j += s.subRevision; j += "\",\"settings\":{";
	for (int k=0;k<NFIELD;k++){ if(k)j+=","; j+="\""; j+=FIELDS[k].name; j+="\":"; j+=cfg[FIELDS[k].off]; }
	j += "}}"; server.send(200, "application/json", j);
}
static void hSet() {
	int i = server.arg("i").toInt();
	if (i<0||i>=escs::COUNT) { server.send(200,"application/json","{\"error\":\"bad-index\"}"); return; }
	uint16_t offs[NFIELD]; uint8_t vals[NFIELD]; int n=0;
	for (int k=0;k<NFIELD;k++) if (server.hasArg(FIELDS[k].name)) { offs[n]=FIELDS[k].off; vals[n]=(uint8_t)server.arg(FIELDS[k].name).toInt(); n++; }
	if (!n) { server.send(200,"application/json","{\"error\":\"no-fields\"}"); return; }
	bool ch=false; int r = escs::editConfig((uint8_t)i, offs, vals, n, ch);
	server.send(200, "application/json", r<0 ? "{\"error\":\"write-failed\"}" : (ch?"{\"result\":\"written\"}":"{\"result\":\"unchanged\"}"));
}
static void hRun()      { escs::release(); server.send(200,"application/json","{\"result\":\"restarted\"}"); }
static void hArm()      { int i=server.arg("i").toInt();
	if (i<0||i>=escs::COUNT) { server.send(200,"application/json","{\"ok\":false}"); return; }
	escs::Drive m = escs::Drive::AUTO; String md = server.arg("mode");
	if (md=="normal") m=escs::Drive::NORMAL; else if (md=="bidir") m=escs::Drive::BIDIR;
	escs::spinArm((uint8_t)i, m);
	String j="{\"ok\":true,\"mode\":\""; j+=escs::spinMode((uint8_t)i);
	j+="\",\"rev\":"; j+=escs::spinReversible((uint8_t)i)?"true":"false"; j+="}";
	server.send(200,"application/json",j); }
static void hSpin()     { int i=server.arg("i").toInt(); int v=server.arg("v").toInt();   // armed only
	if (i>=0&&i<escs::COUNT) {                                       // reversible ESC => v is signed thrust
		if (escs::spinReversible((uint8_t)i)) escs::spinThrust((uint8_t)i,(int16_t)v);
		else                                  escs::spinThrottle((uint8_t)i,(uint16_t)v);
	}
	server.send(200,"application/json","{\"ok\":true}"); }
static void hDisarm()   { int i=server.arg("i").toInt(); if(i>=0&&i<escs::COUNT) escs::spinStop((uint8_t)i); server.send(200,"application/json","{\"ok\":true}"); }
static void hSpinStop() { escs::spinStopAll(); server.send(200,"application/json","{\"ok\":true}"); }
static void hTele() {
	int i=server.arg("i").toInt();
	if (i<0||i>=escs::COUNT) { server.send(200,"application/json","{\"ok\":false}"); return; }
	String j = "{\"ok\":true,\"armed\":"; j += escs::spinArmed((uint8_t)i)?"true":"false";
	j += ",\"mode\":\""; j += escs::spinMode((uint8_t)i); j += "\",\"rev\":"; j += escs::spinReversible((uint8_t)i)?"true":"false";
	escs::Telem t;
	if (escs::spinTele((uint8_t)i,t)) {                              // telemetry only on bidir DShot
		j += ",\"tele\":true,\"rpm\":"; j+=t.rpm; j+=",\"volt\":"; j+=t.voltage; j+=",\"amp\":"; j+=t.current;
		j += ",\"temp\":"; j+=t.tempC; j+=",\"stress\":"; j+=t.stress;
	} else j += ",\"tele\":false";
	j += "}"; server.send(200,"application/json",j);
}

// ================= USB-serial API (always) =================
static void handleSerial() {
	static char line[600]; static uint16_t len = 0; static uint8_t flbuf[256];
	while (Serial.available()) {
		int c = Serial.read();
		if (c == '\r') continue;
		if (c != '\n') { if (len < sizeof(line)-1) line[len++]=(char)c; continue; }
		line[len]='\0'; len=0;
		char* cmd = strtok(line, " "); if (!cmd) continue;
		auto argi = []() { char* a=strtok(nullptr," "); return a?atoi(a):-1; };

		if (!strcmp(cmd,"ping")) { Serial.println("id esc_tool v1"); Serial.println("ok"); }
		else if (!strcmp(cmd,"pins")) { Serial.printf("pins %u", escs::COUNT); for(uint8_t i=0;i<escs::COUNT;i++)Serial.printf(" %u",escs::PINS[i]); Serial.println(); Serial.println("ok"); }
		else if (!strcmp(cmd,"mode")) { char* a=strtok(nullptr," ");
			if (a && !strcmp(a,"drive")) setMode(DRIVE); else if (a && !strcmp(a,"setup")) setMode(SETUP);
			Serial.printf("mode %s (wifi %s)\n", mode==SETUP?"setup":"drive", wifiUp?"on":"off"); Serial.println("ok"); }
		else if (!strcmp(cmd,"scan")) { for(uint8_t i=0;i<escs::COUNT;i++){ escs::Info in;
			if(!escs::scan(i,in)) Serial.printf("esc|%u|%u|0\n",i,escs::PINS[i]);
			else Serial.printf("esc|%u|%u|1|%04X|%u|%s|%s|%u.%u\n",i,in.pin,in.sig,in.bootVer,in.layout,in.name,in.fwMain,in.fwSub); } Serial.println("ok"); }
		else if (!strcmp(cmd,"read")) { int i=argi(); uint8_t cfg[esc_setup::kEepromLen];
			if(i<0||i>=escs::COUNT) Serial.println("err bad-index");
			else if(!escs::readConfig((uint8_t)i,cfg)) Serial.println("err no-connect");
			else { Serial.print("cfg|"); printHex(cfg,esc_setup::kEepromLen); Serial.println(); Serial.println("ok"); } }
		else if (!strcmp(cmd,"enter")) { int i=argi(); escs::Info in;
			if(i<0||i>=escs::COUNT) Serial.println("err bad-index");
			else if(!escs::connect((uint8_t)i,in)) Serial.println("err no-connect");
			else { Serial.printf("dev|%04X|%u|%u\n",in.sig,in.bootVer,in.bootPages); Serial.println("ok"); } }
		else if (!strcmp(cmd,"run")||!strcmp(cmd,"disconnect")) { escs::release(); Serial.println("ok"); }
		else if (!strcmp(cmd,"editpage")) { int i=argi(); char* ovr=strtok(nullptr," ");
			if(i<0||i>=escs::COUNT||!ovr){ Serial.println("err bad-args"); continue; }
			uint16_t offs[160]; uint8_t vals[160]; int n=0; bool bad=false;
			for(char* tok=strtok(ovr,",");tok&&!bad;tok=strtok(nullptr,",")){ char* col=strchr(tok,':');
				if(!col||n>=160){bad=true;break;} *col='\0'; long off=strtol(tok,nullptr,16),val=strtol(col+1,nullptr,16);
				if(off<0||off>=(long)esc_setup::kPageLen||val<0||val>255){bad=true;break;} offs[n]=(uint16_t)off; vals[n]=(uint8_t)val; n++; }
			if(bad){ Serial.println("err bad-override"); continue; }
			bool ch=false; int r=escs::editConfig((uint8_t)i,offs,vals,n,ch);
			if(r==-1)Serial.println("err no-connect"); else if(r==-2)Serial.println("err read-failed");
			else if(r==-3)Serial.println("err bad-override"); else if(r==-4)Serial.println("err write-verify-failed");
			else if(r==0){ Serial.println("unchanged (flash write skipped)"); Serial.println("ok"); }
			else { Serial.printf("edited %d byte(s)\n",n); Serial.println("ok"); } }
		else if (!strcmp(cmd,"erase")) { int i=argi(); char* ad=strtok(nullptr," ");
			if(i<0||i>=escs::COUNT||!ad) Serial.println("err bad-args");
			else Serial.println(escs::erasePage((uint8_t)i,(uint16_t)strtol(ad,nullptr,16))?"ok":"err erase-failed"); }
		else if (!strcmp(cmd,"writeflash")) { int i=argi(); char* ad=strtok(nullptr," "); char* hx=strtok(nullptr," ");
			if(i<0||i>=escs::COUNT||!ad||!hx){ Serial.println("err bad-args"); continue; }
			int n=parseHex(hx,flbuf,sizeof(flbuf));
			if(n<=0)Serial.println("err bad-hex");
			else Serial.println(escs::writeFlash((uint8_t)i,(uint16_t)strtol(ad,nullptr,16),flbuf,(uint16_t)n)?"ok":"err write-failed"); }
		else if (!strcmp(cmd,"readflash")) { int i=argi(); char* ad=strtok(nullptr," "); char* ln=strtok(nullptr," ");
			int rl=ln?atoi(ln):-1;
			if(i<0||i>=escs::COUNT||!ad||rl<1||rl>(int)sizeof(flbuf)) Serial.println("err bad-args");
			else if(!escs::readFlash((uint8_t)i,(uint16_t)strtol(ad,nullptr,16),flbuf,(uint16_t)rl)) Serial.println("err read-failed");
			else { Serial.print("data|"); printHex(flbuf,rl); Serial.println(); Serial.println("ok"); } }
		else if (!strcmp(cmd,"arm")) { int i=argi(); char* m=strtok(nullptr," ");   // arm <i> [normal|bidir]
			if(i<0||i>=escs::COUNT) Serial.println("err bad-index");
			else { escs::Drive md=escs::Drive::AUTO;
				if(m&&!strcmp(m,"normal"))md=escs::Drive::NORMAL; else if(m&&!strcmp(m,"bidir"))md=escs::Drive::BIDIR;
				escs::spinArm((uint8_t)i,md);
				Serial.printf("arming ~3s (mode %s, %s)\n",escs::spinMode((uint8_t)i),escs::spinReversible((uint8_t)i)?"reversible":"one-way");
				Serial.println("ok"); } }
		else if (!strcmp(cmd,"throttle")||!strcmp(cmd,"spin")) { int i=argi(); char* v=strtok(nullptr," ");  // 0..2000
			if(i<0||i>=escs::COUNT||!v) Serial.println("err bad-args");
			else if(!escs::spinArmed((uint8_t)i)) Serial.println("err not-armed");
			else { escs::spinThrottle((uint8_t)i,(uint16_t)atoi(v)); Serial.println("ok"); } }
		else if (!strcmp(cmd,"thrust")) { int i=argi(); char* v=strtok(nullptr," ");   // signed -1000..1000 (reversible/3D)
			if(i<0||i>=escs::COUNT||!v) Serial.println("err bad-args");
			else if(!escs::spinArmed((uint8_t)i)) Serial.println("err not-armed");
			else { escs::spinThrust((uint8_t)i,(int16_t)atoi(v)); Serial.println("ok"); } }
		else if (!strcmp(cmd,"disarm")||!strcmp(cmd,"spinstop")) { int i=argi(); if(i<0) escs::spinStopAll(); else if(i<escs::COUNT) escs::spinStop((uint8_t)i); Serial.println("ok"); }
		else if (!strcmp(cmd,"tele")) { int i=argi(); escs::Telem t;
			if(i<0||i>=escs::COUNT||!escs::spinTele((uint8_t)i,t)) Serial.println("err no-telem");
			else { Serial.printf("tele|%lu|%.2f|%lu|%lu|%lu\n",(unsigned long)t.rpm,t.voltage,(unsigned long)t.current,(unsigned long)t.tempC,(unsigned long)t.stress); Serial.println("ok"); } }
		else Serial.println("err unknown-cmd");
	}
}

void setup() {
	Serial.begin(115200);
	pinMode(MODE_PIN, INPUT_PULLUP);
	delay(50);
	server.on("/", hIndex);
	server.on("/api/scan", hScan);
	server.on("/api/read", hRead);
	server.on("/api/set", HTTP_POST, hSet);
	server.on("/api/run", HTTP_POST, hRun);
	server.on("/api/arm", HTTP_POST, hArm);
	server.on("/api/spin", HTTP_POST, hSpin);
	server.on("/api/disarm", HTTP_POST, hDisarm);
	server.on("/api/spinstop", HTTP_POST, hSpinStop);
	server.on("/api/tele", hTele);
	setMode(digitalRead(MODE_PIN) == LOW ? DRIVE : SETUP);   // LOW=drive; unconnected(HIGH)=setup
}
void loop() {
	handleSerial();
	escs::spinPoll();
	if (wifiUp) server.handleClient();
}
void setup1() {}
void loop1()  { escs::core1Poll(); }
