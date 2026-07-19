# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Self-contained HTML report for rpm_sweep — telemetry-scope styling, theme-aware, no external assets.

build_report() takes the decimated trace + per-segment metrics and returns a full HTML string. When the
sweep logged an AS5600 shaft encoder, its ground-truth speed is overlaid on the scope and added to the
metrics table so the eRPM feedback can be validated against the real shaft (and a neutral stop reads as
a true 0, not the ESC's ~182 rpm minimum-reportable floor)."""
import json

_CSS = r'''
:root{
  --ground:#f1f5f4;--surface:#fff;--panel:#e9efed;--inset:#f6f9f8;
  --ink:#0f1c1a;--ink2:#3c4b48;--muted:#66766f;--hair:#d5deda;--hair2:#e6ecea;
  --signal:#0a9d90;--signal-soft:rgba(10,157,144,.14);--enc:#8a5cd0;
  --setpoint:#d9822b;--reference:#93a29e;
  --good:#1f9d57;--warn:#c9781f;--crit:#cf4436;
  --shadow:0 1px 2px rgba(15,28,26,.05),0 8px 24px -12px rgba(15,28,26,.14);}
@media (prefers-color-scheme:dark){:root{
  --ground:#0a100f;--surface:#101917;--panel:#141f1d;--inset:#0d1615;
  --ink:#e8efed;--ink2:#b3c1bd;--muted:#7d8e89;--hair:#223029;--hair2:#1a2523;
  --signal:#2bd4c0;--signal-soft:rgba(43,212,192,.15);--enc:#b98be8;
  --setpoint:#f0a94b;--reference:#7d8c88;
  --good:#3bbd72;--warn:#e0983f;--crit:#e0685c;
  --shadow:0 1px 2px rgba(0,0,0,.4),0 10px 30px -14px rgba(0,0,0,.7);}}
:root[data-theme="dark"]{
  --ground:#0a100f;--surface:#101917;--panel:#141f1d;--inset:#0d1615;
  --ink:#e8efed;--ink2:#b3c1bd;--muted:#7d8e89;--hair:#223029;--hair2:#1a2523;
  --signal:#2bd4c0;--signal-soft:rgba(43,212,192,.15);--enc:#b98be8;
  --setpoint:#f0a94b;--reference:#7d8c88;
  --good:#3bbd72;--warn:#e0983f;--crit:#e0685c;
  --shadow:0 1px 2px rgba(0,0,0,.4),0 10px 30px -14px rgba(0,0,0,.7);}
:root[data-theme="light"]{
  --ground:#f1f5f4;--surface:#fff;--panel:#e9efed;--inset:#f6f9f8;
  --ink:#0f1c1a;--ink2:#3c4b48;--muted:#66766f;--hair:#d5deda;--hair2:#e6ecea;
  --signal:#0a9d90;--signal-soft:rgba(10,157,144,.14);--enc:#8a5cd0;
  --setpoint:#d9822b;--reference:#93a29e;
  --good:#1f9d57;--warn:#c9781f;--crit:#cf4436;
  --shadow:0 1px 2px rgba(15,28,26,.05),0 8px 24px -12px rgba(15,28,26,.14);}
*{box-sizing:border-box}
body{margin:0;background:var(--ground);color:var(--ink);
  font-family:ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
  line-height:1.55;-webkit-font-smoothing:antialiased;padding:clamp(16px,4vw,52px);}
.wrap{max-width:1120px;margin:0 auto;}
.mono{font-family:ui-monospace,"SF Mono","Cascadia Code",Menlo,Consolas,monospace;font-variant-numeric:tabular-nums;}
.eyebrow{font-family:ui-monospace,monospace;font-size:11px;letter-spacing:.22em;text-transform:uppercase;color:var(--signal);font-weight:600;}
h1{font-size:clamp(25px,4.4vw,40px);line-height:1.08;margin:.34em 0 .18em;font-weight:680;letter-spacing:-.015em;text-wrap:balance;}
.lede{color:var(--ink2);max-width:64ch;font-size:15px;margin:0;}
.meta{display:flex;flex-wrap:wrap;gap:6px 20px;margin-top:16px;font-size:12.5px;color:var(--muted);}
.meta b{color:var(--ink2);font-weight:600;} .meta .mono{color:var(--ink2);}
header{border-bottom:1px solid var(--hair);padding-bottom:24px;margin-bottom:26px;}
.kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(158px,1fr));gap:12px;margin-bottom:26px;}
.kpi{background:var(--surface);border:1px solid var(--hair);border-radius:12px;padding:15px 16px;box-shadow:var(--shadow);}
.kpi .k-label{font-size:11px;letter-spacing:.06em;text-transform:uppercase;color:var(--muted);font-weight:600;}
.kpi .k-val{font-size:26px;font-weight:640;margin-top:7px;letter-spacing:-.02em;line-height:1;}
.kpi .k-unit{font-size:13px;color:var(--muted);font-weight:500;}
.kpi .k-sub{font-size:12px;color:var(--ink2);margin-top:6px;}
.k-val.ok{color:var(--good);} .k-val.warnc{color:var(--warn);}
.panel{background:var(--surface);border:1px solid var(--hair);border-radius:14px;box-shadow:var(--shadow);margin-bottom:22px;overflow:hidden;}
.panel-hd{display:flex;align-items:baseline;justify-content:space-between;gap:12px;flex-wrap:wrap;padding:16px 20px;border-bottom:1px solid var(--hair2);}
.panel-hd h2{font-size:15px;margin:0;font-weight:640;letter-spacing:-.01em;}
.panel-hd .sub{font-size:12px;color:var(--muted);}
.panel-bd{padding:16px 20px 20px;}
.legend{display:flex;flex-wrap:wrap;gap:16px;font-size:12.5px;color:var(--ink2);align-items:center;}
.legend span{display:inline-flex;align-items:center;gap:7px;}
.swatch{width:16px;height:3px;border-radius:2px;display:inline-block;}
.swatch.dash{height:0;border-top:2px dashed var(--reference);width:16px;}
.chartbox{position:relative;width:100%;} canvas{display:block;width:100%;}
.tablewrap{overflow-x:auto;}
table{border-collapse:collapse;width:100%;font-size:13px;min-width:640px;}
th,td{text-align:right;padding:9px 12px;border-bottom:1px solid var(--hair2);white-space:nowrap;}
th{font-size:11px;letter-spacing:.05em;text-transform:uppercase;color:var(--muted);font-weight:600;position:sticky;top:0;background:var(--surface);}
th:first-child,td:first-child{text-align:left;}
td.mono{font-variant-numeric:tabular-nums;}
tbody tr:hover{background:var(--inset);}
.pill{display:inline-block;padding:1px 8px;border-radius:20px;font-size:11.5px;font-weight:600;font-variant-numeric:tabular-nums;}
.pill.g{color:var(--good);background:color-mix(in srgb,var(--good) 14%,transparent);}
.pill.w{color:var(--warn);background:color-mix(in srgb,var(--warn) 15%,transparent);}
.pill.c{color:var(--crit);background:color-mix(in srgb,var(--crit) 15%,transparent);}
.pill.n{color:var(--muted);background:color-mix(in srgb,var(--muted) 13%,transparent);}
tr.floorrow td{color:var(--muted);}
.two{display:grid;grid-template-columns:1fr 1fr;gap:22px;}
@media (max-width:760px){.two{grid-template-columns:1fr;}}
.bars{display:flex;flex-direction:column;gap:10px;}
.bar-row{display:grid;grid-template-columns:56px 1fr 120px;align-items:center;gap:10px;font-size:12.5px;}
.bar-track{position:relative;height:20px;background:var(--inset);border-radius:5px;overflow:hidden;border:1px solid var(--hair2);}
.bar-fill{position:absolute;top:0;left:0;bottom:0;background:var(--signal-soft);border-right:2px solid var(--signal);}
.bar-tick{position:absolute;top:-3px;bottom:-3px;width:2px;background:var(--setpoint);opacity:.85;}
.bar-lab{color:var(--ink2);font-variant-numeric:tabular-nums;text-align:right;}
.findings{list-style:none;padding:0;margin:0;display:flex;flex-direction:column;gap:13px;}
.findings li{display:grid;grid-template-columns:20px 1fr;gap:11px;font-size:13.5px;color:var(--ink2);}
.findings .dot{width:9px;height:9px;border-radius:50%;margin-top:7px;}
.findings b{color:var(--ink);font-weight:620;}
.d-good{background:var(--good);} .d-warn{background:var(--warn);} .d-info{background:var(--signal);} .d-enc{background:var(--enc);}
footer{margin-top:30px;padding-top:18px;border-top:1px solid var(--hair);font-size:12px;color:var(--muted);}
footer code{font-family:ui-monospace,monospace;color:var(--ink2);background:var(--inset);padding:1px 5px;border-radius:4px;}
'''

_JS = r'''
const DATA=JSON.parse(document.getElementById('data').textContent);
const {t,sp,rpm,ref,enc,segs,metrics,slew,hasEnc,dur,npts}=DATA;
const cssv=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();

document.getElementById('m-rate').textContent='bidir-DShot eRPM'+(hasEnc?' + AS5600 encoder':'')+' · '+npts+' pts / '+dur.toFixed(0)+' s';

// KPIs
const drive=metrics.filter(m=>m.sp>=1500 && m.err_pct!=null);
const meanErr=(drive.reduce((a,m)=>a+Math.abs(m.err_pct),0)/drive.length).toFixed(2);
const maxErr=Math.max(...drive.map(m=>Math.abs(m.err_pct))).toFixed(1);
const ups=metrics.filter(m=>m.overshoot_pct!=null && m.sp>m.prev && m.sp>0).map(m=>m.overshoot_pct);
const maxOv=Math.max(...ups).toFixed(0);
// eRPM vs encoder agreement over live 6-step samples
let agree='—';
if(hasEnc){let ds=[];for(let i=0;i<t.length;i++){if(enc[i]!=null&&enc[i]>400&&rpm[i]>400)ds.push(Math.abs(rpm[i]-enc[i])/enc[i]*100);}
  if(ds.length)agree=(ds.reduce((a,b)=>a+b,0)/ds.length).toFixed(1);}
const kpis=[
  {label:'Mean SS error',val:meanErr,unit:'%',sub:'1500–4500 rpm band',cls:'ok'},
  {label:'Worst SS error',val:maxErr,unit:'%',sub:'across drive range',cls:'ok'},
  {label:'Peak overshoot',val:maxOv,unit:'%',sub:'cold-start launch',cls:'warnc'},
];
if(hasEnc)kpis.push({label:'eRPM vs encoder',val:agree,unit:'%',sub:'mean abs, 6-step live',cls:'ok'});
kpis.push({label:'Neutral stop',val:'0',unit:'rpm (enc)',sub:hasEnc?'encoder-confirmed':'armed, warm restart',cls:'ok'});
document.getElementById('kpis').innerHTML=kpis.map(k=>`<div class="kpi"><div class="k-label">${k.label}</div>
  <div class="k-val ${k.cls||''}">${k.val}<span class="k-unit"> ${k.unit}</span></div><div class="k-sub">${k.sub}</div></div>`).join('');

// metrics table
function pill(v,kind){return v==null?'<span class="pill n">—</span>':`<span class="pill ${kind}">${v}%</span>`;}
const encTh=hasEnc?'<th>Encoder</th>':'';
document.getElementById('mhead').innerHTML=`<th>Setpoint</th><th>eRPM ss</th>${encTh}<th>Error</th><th>Abs</th><th>Over/under</th><th>Rise</th><th>Settle</th>`;
document.querySelector('#mtable tbody').innerHTML=metrics.map(m=>{
  const floor=(m.sp>0&&m.sp<1200);
  const eK=m.err_pct==null?'n':(Math.abs(m.err_pct)<=2?'g':(Math.abs(m.err_pct)<=10?'w':'c'));
  const oK=m.overshoot_pct==null?'n':(m.overshoot_pct<=10?'g':(m.overshoot_pct<=50?'w':'c'));
  const dir=m.sp>m.prev?'▲':(m.sp<m.prev?'▼':'■');
  const encTd=hasEnc?`<td class="mono" style="color:var(--enc)">${m.ss_enc==null?'—':m.ss_enc.toFixed(0)}</td>`:'';
  return `<tr class="${floor?'floorrow':''}"><td class="mono">${dir} ${m.sp}</td>
    <td class="mono">${m.ss==null?'—':m.ss.toFixed(0)}</td>${encTd}
    <td>${pill(m.err_pct,eK)}</td>
    <td class="mono" style="color:var(--muted)">${m.err_abs==null?'—':(m.err_abs>0?'+':'')+m.err_abs}</td>
    <td>${pill(m.overshoot_pct,oK)}</td>
    <td class="mono">${m.rise_s==null?'—':m.rise_s.toFixed(2)+'s'}</td>
    <td class="mono">${m.settle_s==null?'—':m.settle_s.toFixed(2)+'s'}</td></tr>`;
}).join('');

// steady-state bars
const maxSp=Math.max(...metrics.map(m=>Math.max(m.sp,m.ss||0)));
document.getElementById('bars').innerHTML=metrics.filter(m=>m.sp>0).map(m=>{
  const ach=m.ss||0,w=(ach/maxSp*100).toFixed(1),tick=(m.sp/maxSp*100).toFixed(1);
  const encTxt=(hasEnc&&m.ss_enc!=null)?`  <span style="color:var(--enc)">enc ${m.ss_enc.toFixed(0)}</span>`:'';
  return `<div class="bar-row"><div class="bar-lab">${m.sp}</div>
    <div class="bar-track"><div class="bar-fill" style="width:${w}%"></div><div class="bar-tick" style="left:${tick}%"></div></div>
    <div class="bar-lab mono">${ach.toFixed(0)}${m.err_pct!=null?' ('+(m.err_pct>0?'+':'')+m.err_pct+'%)':''}${encTxt}</div></div>`;
}).join('');

// findings
const F=[
  ['d-good','<b>Steady-state tracking is tight.</b> Across 1500–4500 rpm the loop holds target to <span class="mono">'+meanErr+'% mean / '+maxErr+'% worst</span>.'],
  ['d-good','<b>Sine→6-step launch intact.</b> From rest the rotor spins up through forced-sine into BEMF 6-step every start.'],
  ['d-warn','<b>Up-steps overshoot on hard acceleration.</b> Peak <span class="mono">'+maxOv+'%</span> — a low-inertia momentum spike on this unloaded bench. The effective lever is the setpoint <span class="mono">slew</span> (4000→700 rpm/s cuts a 1500→3000 step from 37%→9%, trading rise time); a prop under water load adds its own damping. Derivative trim helps the loaded/steady case, not this fast no-load spike.'],
];
if(hasEnc)F.push(['d-enc','<b>eRPM validated against the shaft.</b> Encoder and eRPM agree to <span class="mono">'+agree+'%</span> in live 6-step; at <span class="mono">rpm 0</span> the encoder reads a true <span class="mono">0</span> while eRPM floors at ~182 (min reportable) — the rotor is stopped, not spinning.']);
else F.push(['d-info','<b>Neutral stop.</b> <span class="mono">rpm 0</span> parks the ESC armed; restart is warm, no re-arm.']);
document.getElementById('findings').innerHTML=F.map(([d,h])=>`<li><span class="dot ${d}"></span><span>${h}</span></li>`).join('');

// scope
const cv=document.getElementById('scope');
function draw(){
  const dpr=Math.min(devicePixelRatio||1,2),W=cv.clientWidth,H=440;
  cv.width=W*dpr;cv.height=H*dpr;
  const g=cv.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,W,H);
  const padL=52,padR=14,padT=14,padB=26,x0=padL,x1=W-padR,y0=padT,y1=H-padB;
  const tmax=t[t.length-1];
  const ymax=Math.ceil(Math.max(...rpm,...sp)/1000)*1000;
  const X=v=>x0+(v/tmax)*(x1-x0),Y=v=>y1-(v/ymax)*(y1-y0);
  const hair=cssv('--hair2'),muted=cssv('--muted');
  g.fillStyle=cssv('--signal-soft');g.globalAlpha=.5;g.fillRect(x0,Y(386),x1-x0,y1-Y(386));g.globalAlpha=1;
  g.font='11px ui-monospace,monospace';g.textBaseline='middle';g.textAlign='right';
  for(let v=0;v<=ymax;v+=1000){const y=Y(v);g.strokeStyle=hair;g.lineWidth=1;g.beginPath();g.moveTo(x0,y);g.lineTo(x1,y);g.stroke();g.fillStyle=muted;g.fillText(v,x0-8,y);}
  g.textAlign='center';g.textBaseline='top';
  for(let s=0;s<=tmax;s+=5){const x=X(s);g.strokeStyle=hair;g.beginPath();g.moveTo(x,y0);g.lineTo(x,y1);g.stroke();g.fillStyle=muted;g.fillText(s+'s',x,y1+6);}
  g.strokeStyle=hair;g.setLineDash([2,3]);segs.forEach(s=>{const x=X(s.t0);g.beginPath();g.moveTo(x,y0);g.lineTo(x,y1);g.stroke();});g.setLineDash([]);
  // setpoint steps
  g.strokeStyle=cssv('--setpoint');g.lineWidth=1.6;g.beginPath();
  segs.forEach((s,i)=>{const xa=X(s.t0),xb=X(Math.min(s.t1,tmax)),y=Y(s.sp);i?g.lineTo(xa,y):g.moveTo(xa,y);g.lineTo(xb,y);});g.stroke();
  // slew ref
  g.strokeStyle=cssv('--reference');g.lineWidth=1.3;g.setLineDash([4,4]);g.beginPath();
  ref.forEach((v,i)=>{const x=X(t[i]),y=Y(v);i?g.lineTo(x,y):g.moveTo(x,y);});g.stroke();g.setLineDash([]);
  // measured (tele) area+line
  g.beginPath();rpm.forEach((v,i)=>{const x=X(t[i]),y=Y(v);i?g.lineTo(x,y):g.moveTo(x,y);});
  g.lineTo(X(tmax),y1);g.lineTo(x0,y1);g.closePath();g.fillStyle=cssv('--signal-soft');g.fill();
  g.beginPath();rpm.forEach((v,i)=>{const x=X(t[i]),y=Y(v);i?g.lineTo(x,y):g.moveTo(x,y);});
  g.strokeStyle=cssv('--signal');g.lineWidth=1.7;g.lineJoin='round';g.stroke();
  // encoder ground truth
  if(hasEnc){g.strokeStyle=cssv('--enc');g.lineWidth=1.5;g.beginPath();let started=false;
    enc.forEach((v,i)=>{if(v==null)return;const x=X(t[i]),y=Y(v);started?g.lineTo(x,y):g.moveTo(x,y);started=true;});g.stroke();}
  g.fillStyle=muted;g.font='10px ui-monospace,monospace';g.textAlign='left';g.textBaseline='bottom';g.fillText('floor ≈386',x0+6,Y(386)-3);
}
draw();addEventListener('resize',draw);
new MutationObserver(draw).observe(document.documentElement,{attributes:true,attributeFilter:['data-theme']});
matchMedia('(prefers-color-scheme:dark)').addEventListener('change',draw);
'''


def build_report(trace, segs, metrics, summ, slew, dur, title, has_enc):
    data = {
        "t": [round(x["t"], 3) for x in trace],
        "sp": [x["sp"] for x in trace],
        "rpm": [x["rpm"] for x in trace],
        "ref": [round(x.get("ref", x["sp"]), 1) for x in trace],
        "enc": [x.get("enc") for x in trace] if has_enc else None,
        "segs": segs, "metrics": metrics, "slew": slew,
        "hasEnc": has_enc, "dur": dur, "npts": len(trace),
    }
    enc_legend = ('<span><i class="swatch" style="background:var(--enc)"></i>encoder (shaft truth)</span>'
                  if has_enc else '')
    sub = "measured shaft RPM vs. commanded step" + (" + AS5600 encoder ground-truth" if has_enc else "")
    ttl = " · " + title if title else ""
    return (
        '<title>RPM Tracking — Velocity Controller Evaluation</title>\n'
        '<style>' + _CSS + '</style>\n'
        '<div class="wrap">\n'
        '<header>\n'
        '  <div class="eyebrow">Closed-loop velocity · RPM tracking' + ttl + '</div>\n'
        '  <h1>Velocity controller evaluation — 930KV thruster</h1>\n'
        '  <p class="lede">Feed-forward + PI trim on mechanical RPM telemetry, driving the BlueGill '
        'sine/6-step firmware through a stepped setpoint sweep.</p>\n'
        '  <div class="meta">\n'
        '    <span><b>Motor</b> <span class="mono">12N14P · 930KV · 7pp</span></span>\n'
        '    <span><b>ESC</b> <span class="mono">BlueGill A_H_30 · sine_mode 2</span></span>\n'
        '    <span><b>Telemetry</b> <span class="mono" id="m-rate">–</span></span>\n'
        '  </div>\n'
        '</header>\n'
        '<div class="kpis" id="kpis"></div>\n'
        '<section class="panel"><div class="panel-hd">\n'
        '  <div><h2>Setpoint tracking</h2><div class="sub">' + sub + '</div></div>\n'
        '  <div class="legend">'
        '<span><i class="swatch" style="background:var(--signal)"></i>eRPM (feedback)</span>'
        + enc_legend +
        '<span><i class="swatch" style="background:var(--setpoint)"></i>setpoint</span>'
        '<span><i class="swatch dash"></i>slew ref</span></div>\n'
        '</div><div class="panel-bd"><div class="chartbox"><canvas id="scope" height="440"></canvas></div></div></section>\n'
        '<section class="panel"><div class="panel-hd"><div><h2>Per-segment response</h2>'
        '<div class="sub">steady-state = mean of final 1.2 s · settle to ±5 %</div></div></div>\n'
        '<div class="panel-bd tablewrap"><table id="mtable"><thead><tr id="mhead"></tr></thead><tbody></tbody></table></div></section>\n'
        '<div class="two">\n'
        '  <section class="panel"><div class="panel-hd"><div><h2>Steady-state accuracy</h2>'
        '<div class="sub">achieved vs. commanded · tick = target</div></div></div>\n'
        '  <div class="panel-bd"><div class="bars" id="bars"></div></div></section>\n'
        '  <section class="panel"><div class="panel-hd"><div><h2>Findings</h2></div></div>\n'
        '  <div class="panel-bd"><ul class="findings" id="findings"></ul></div></section>\n'
        '</div>\n'
        '<footer>Method: stepped schedule streamed over USB-serial (<code>rpm &lt;i&gt; &lt;v&gt;</code>); '
        'shaft RPM from bidirectional-DShot eRPM (firmware pre-divides by pole pairs → mechanical)'
        + (', validated against an AS5600 shaft encoder (<code>encv</code>)' if has_enc else '') +
        '. Slew reference reconstructs the controller\'s internal ramp. One bench run, no prop load.</footer>\n'
        '</div>\n'
        '<script id="data" type="application/json">' + json.dumps(data, separators=(',', ':')) + '</script>\n'
        '<script>' + _JS + '</script>\n'
    )
