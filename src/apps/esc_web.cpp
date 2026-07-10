// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_web — Wi-Fi web front-end for the RP2040 ESC tool (Pico W). Brings up a Wi-Fi Access Point
// and an HTTP server that serves a single-page BLHeli-Configurator-style UI plus a small JSON API,
// all on the shared esc_session core (same 1-wire bootloader logic as esc_host). Connect a
// phone/PC to the "pico-esc-tool" Wi-Fi, open http://192.168.4.1, and list / read / edit ESCs.
//
// Slice 1: scan / read / set / disconnect. (Firmware flash-from-browser is the next slice.)
// NOTE: Pico W only. The CYW43 radio and DShot both use PIO; if they contend, adjust NUM_PIOS /
// pin allocation. Untested by the author over Wi-Fi — validate on the bench.
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esc_session.h"

static const char* AP_SSID = "pico-esc-tool";
static const char* AP_PASS = "esctool1234";   // >= 8 chars (WPA2). Change before real use.

static WebServer server(80);

// writable settings exposed in the web UI: name -> config-block offset (raw byte value)
struct Field { const char* name; uint16_t off; };
static const Field FIELDS[] = {
	{ "motor_direction", 0x0B }, { "comm_timing", 0x15 }, { "demag_compensation", 0x1F },
	{ "startup_beep", 0x05 },    { "beep_strength", 0x1B }, { "beacon_strength", 0x1C },
	{ "beacon_delay", 0x1D },    { "temperature_protection", 0x23 },
	{ "low_rpm_power_protection", 0x24 }, { "brake_on_stop", 0x27 },
};
static const int NFIELD = sizeof(FIELDS) / sizeof(FIELDS[0]);

static String jesc(const char* s) {           // minimal JSON string escape
	String o;
	for (; *s; s++) { if (*s == '"' || *s == '\\') o += '\\'; o += *s; }
	return o;
}

static const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>pico-esc-tool</title>
<style>body{font-family:system-ui,sans-serif;margin:1em;max-width:680px}
button{padding:.4em .8em;margin:.2em}label{display:block;margin:.3em 0}input{width:6em}
.esc{border:1px solid #ccc;border-radius:8px;padding:.5em;margin:.5em 0}#msg{color:#070}</style></head>
<body><h1>pico-esc-tool</h1>
<button onclick=scan()>Scan</button><button onclick=disc()>Disconnect (restart ESC)</button>
<span id=msg></span><div id=list></div><div id=cfg></div>
<script>
const g=id=>document.getElementById(id), msg=t=>g('msg').textContent=t;
async function scan(){msg('scanning...');let d=await(await fetch('/api/scan')).json();
 g('list').innerHTML=d.map((e,i)=>e.present
  ?`<div class=esc><b>ESC ${i}</b> pin ${e.pin} sig ${e.sig} layout ${e.layout} name "${e.name}" fw ${e.fw}
     <button onclick=read(${i})>Edit</button></div>`
  :`<div class=esc>ESC ${i}: no ESC</div>`).join('');msg('');}
async function read(i){msg('reading...');let d=await(await fetch('/api/read?i='+i)).json();
 if(d.error){msg('error: '+d.error);return;}let s=d.settings;
 g('cfg').innerHTML=`<div class=esc><h3>ESC ${i} — ${d.name||'(no name)'} (fw ${d.fw})</h3>`
  +Object.keys(s).map(k=>`<label>${k} <input id=f_${k} type=number min=0 max=255 value="${s[k]}"></label>`).join('')
  +`<button onclick=save(${i})>Save</button></div>`;msg('');}
async function save(i){let p=[...document.querySelectorAll('[id^=f_]')].map(el=>el.id.slice(2)+'='+el.value).join('&');
 msg('writing...');let d=await(await fetch('/api/set?i='+i+'&'+p,{method:'POST'})).json();
 msg(d.error?('error: '+d.error):('saved: '+d.result));}
async function disc(){await fetch('/api/run',{method:'POST'});g('cfg').innerHTML='';msg('ESC restarted');}
scan();
</script></body></html>)HTML";

static void handleIndex() { server.send_P(200, "text/html", INDEX_HTML); }

static void handleScan() {
	String j = "[";
	for (uint8_t i = 0; i < escs::COUNT; i++) {
		escs::Info in; bool ok = escs::scan(i, in);
		if (i) j += ",";
		if (!ok) { j += "{\"present\":false,\"pin\":"; j += escs::PINS[i]; j += "}"; }
		else {
			char sig[8]; snprintf(sig, sizeof(sig), "%04X", in.sig);
			j += "{\"present\":true,\"pin\":"; j += in.pin;
			j += ",\"sig\":\""; j += sig;
			j += "\",\"layout\":\""; j += jesc(in.layout);
			j += "\",\"name\":\""; j += jesc(in.name);
			j += "\",\"fw\":\""; j += in.fwMain; j += "."; j += in.fwSub; j += "\"}";
		}
	}
	j += "]";
	server.send(200, "application/json", j);
}

static void handleRead() {
	int i = server.arg("i").toInt();
	uint8_t cfg[esc_setup::kEepromLen];
	if (i < 0 || i >= escs::COUNT || !escs::readConfig((uint8_t)i, cfg)) {
		server.send(200, "application/json", "{\"error\":\"no-connect\"}"); return;
	}
	esc_setup::Settings s; esc_setup::decode(cfg, esc_setup::kEepromLen, s);
	String j = "{\"name\":\""; j += jesc(s.name);
	j += "\",\"layout\":\""; j += jesc(s.layoutTag);
	j += "\",\"fw\":\""; j += s.mainRevision; j += "."; j += s.subRevision; j += "\",\"settings\":{";
	for (int k = 0; k < NFIELD; k++) { if (k) j += ","; j += "\""; j += FIELDS[k].name; j += "\":"; j += cfg[FIELDS[k].off]; }
	j += "}}";
	server.send(200, "application/json", j);
}

static void handleSet() {
	int i = server.arg("i").toInt();
	if (i < 0 || i >= escs::COUNT) { server.send(200, "application/json", "{\"error\":\"bad-index\"}"); return; }
	uint16_t offs[NFIELD]; uint8_t vals[NFIELD]; int n = 0;
	for (int k = 0; k < NFIELD; k++)
		if (server.hasArg(FIELDS[k].name)) { offs[n] = FIELDS[k].off; vals[n] = (uint8_t)server.arg(FIELDS[k].name).toInt(); n++; }
	if (!n) { server.send(200, "application/json", "{\"error\":\"no-fields\"}"); return; }
	bool changed = false;
	int r = escs::editConfig((uint8_t)i, offs, vals, n, changed);
	if (r < 0) server.send(200, "application/json", "{\"error\":\"write-failed\"}");
	else server.send(200, "application/json", changed ? "{\"result\":\"written\"}" : "{\"result\":\"unchanged\"}");
}

static void handleRun() { escs::release(); server.send(200, "application/json", "{\"result\":\"restarted\"}"); }

void setup() {
	Serial.begin(115200);
	WiFi.mode(WIFI_AP);
	WiFi.softAP(AP_SSID, AP_PASS);
	server.on("/", handleIndex);
	server.on("/api/scan", handleScan);
	server.on("/api/read", handleRead);
	server.on("/api/set", HTTP_POST, handleSet);
	server.on("/api/run", HTTP_POST, handleRun);
	server.begin();
}
void loop() { server.handleClient(); }

void setup1() {}
void loop1()  { escs::core1Poll(); }
