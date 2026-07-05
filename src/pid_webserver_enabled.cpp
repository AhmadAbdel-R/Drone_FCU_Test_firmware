#include "pid_webserver.h"

#if ENABLE_PID_WEBSERVER

#if defined(FCU_DISABLE_FAILSAFES) && (FCU_DISABLE_FAILSAFES)
#warning "PID-web build default is failsafe bypass; runtime PID-web setting can re-enable, but treat as BENCH-ONLY until verified."
#endif

#include <Arduino.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <esp_http_server.h>

#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "arming_flags.h"
#include "fcu_config.h"
#include "pidweb_dashboard_html.h"

namespace pid_webserver {

namespace {

constexpr const char* kTag = "PIDWEB";
// SoftAP fixed IP: the FCU broadcasts its OWN WiFi network instead of joining
// one, and serves the tuner at http://<this-ip>/. Gateway == this IP, /24 mask.
// Override via build_flags (e.g. -DFCU_PID_AP_IP="10,0,0,1") if desired.
#ifndef FCU_PID_AP_IP
#define FCU_PID_AP_IP 192, 168, 4, 1
#endif
#ifndef FCU_PID_WIFI_RECONNECT_BACKOFF_MS
#define FCU_PID_WIFI_RECONNECT_BACKOFF_MS 3000U
#endif
#ifndef FCU_PID_WIFI_HEALTH_LOG_MS
#define FCU_PID_WIFI_HEALTH_LOG_MS 5000U
#endif
#ifndef FCU_PID_HTTPD_STACK_BYTES
#define FCU_PID_HTTPD_STACK_BYTES (6 * 1024)
#endif
#ifndef FCU_PID_TELEM_STACK_BYTES
#define FCU_PID_TELEM_STACK_BYTES 4096
#endif

Callbacks gCb;
httpd_handle_t gServer = nullptr;
std::atomic<bool> gSafeToWrite{false};
std::atomic<bool> gRunning{false};
const char* gSsid = "";
const char* gPassword = "";
uint32_t gNextReconnectMs = 0;  // backoff gate before re-asserting a downed AP
uint32_t gLastHealthLogMs = 0;
uint32_t gLastIp = 0;

// Shared-secret token required on every mutating request via the X-Auth-Token
// header. Set once before start(); only read thereafter, so a plain buffer is
// fine. Empty token => all mutating requests rejected (fail-closed).
char gAuthToken[96] = {};

// Constant-time string compare so a wrong token can't be teased out by timing.
// Returns true iff a and b are equal and the same length.
bool constantTimeEquals(const char* a, const char* b) {
  const size_t la = strlen(a);
  const size_t lb = strlen(b);
  uint8_t diff = static_cast<uint8_t>(la ^ lb);
  const size_t n = (la > lb) ? la : lb;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t ca = (i < la) ? static_cast<uint8_t>(a[i]) : 0;
    const uint8_t cb = (i < lb) ? static_cast<uint8_t>(b[i]) : 0;
    diff |= static_cast<uint8_t>(ca ^ cb);
  }
  return diff == 0;
}

// Authorize a mutating request: requires the X-Auth-Token header to match the
// configured token. Fails closed when no token is configured.
bool authorized(httpd_req_t* req) {
  if (gAuthToken[0] == '\0') {
    return false;
  }
  char hdr[96] = {};
  if (httpd_req_get_hdr_value_str(req, "X-Auth-Token", hdr, sizeof(hdr)) != ESP_OK) {
    return false;
  }
  return constantTimeEquals(hdr, gAuthToken);
}

// ---- HTML page (embedded). Tiny so it fits in PROGMEM trivially. ----
// Single-file: HTML+CSS+JS, polls /api/pid + /api/state every 500ms, exposes
// 12 sliders + Save/Revert/Reset buttons. Intentionally minimalist — no
// external CDN, no fonts, works on a phone.
const char kIndexHtml[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>FCU PID Tuner</title>
<style>
 body{font-family:system-ui,-apple-system,sans-serif;margin:0;padding:12px;background:#101418;color:#e6e8ec}
 h1{font-size:18px;margin:4px 0 14px}
 .row{display:flex;align-items:center;gap:8px;margin:6px 0}
 .row label{flex:0 0 110px;font-size:13px;color:#8c92a0}
 .row input[type=range]{flex:1}
 .row .v{flex:0 0 70px;text-align:right;font-variant-numeric:tabular-nums}
 .grp{background:#1a1f26;border:1px solid #2a2f38;border-radius:8px;padding:8px 12px;margin:10px 0}
 .grp h2{font-size:14px;margin:2px 0 6px;color:#4da3ff}
 .btn{padding:10px 14px;border-radius:6px;border:1px solid #2a2f38;background:#1a1f26;color:#e6e8ec;font-size:14px;cursor:pointer}
 .btn:hover{background:#222831}
 .btn.danger{border-color:#7a2e2e;color:#ffb3b3}
 .actions{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
 .status{font-size:12px;color:#8c92a0;margin-top:8px}
 .warn{color:#ffb020} .ok{color:#3ce074} .err{color:#ff4d4d}
</style></head><body>
<h1>FCU PID Tuner <span id="safe" class="warn">checking...</span></h1>
<div class="status" style="margin:-4px 0 10px;display:flex;gap:8px;align-items:center;flex-wrap:wrap">
 <span id="authState">auth: ?</span>
 <button class="btn" style="padding:4px 10px;font-size:12px" onclick="setToken()">Set token</button>
 <button class="btn" style="padding:4px 10px;font-size:12px" onclick="clearToken()">Clear</button>
 <span style="color:#5a6070">Token persists in this browser. Required for any edit/save.</span>
</div>
<div id="grps"></div>
<div class="grp"><h2>Motor orientation</h2><div class="actions">
 <button class="btn motor" onclick="spinMotor(1)">Spin motor 1</button>
 <button class="btn motor" onclick="spinMotor(2)">Spin motor 2</button>
 <button class="btn motor" onclick="spinMotor(3)">Spin motor 3</button>
 <button class="btn motor" onclick="spinMotor(4)">Spin motor 4</button>
</div></div>
<div class="grp"><h2>Mixer — pitch front bias</h2>
 <div class="row">
  <label>Front × pitch</label>
  <input type="number" id="mixBias" min="1.00" max="2.00" step="0.01" value="1.30" style="flex:1;background:#222831;color:#e6e8ec;border:1px solid #2a2f38;border-radius:4px;padding:6px 8px">
  <div class="v" id="mixBiasNow">--</div>
 </div>
 <div class="actions">
  <button class="btn" onclick="pushMix()">Apply (RAM)</button>
  <button class="btn" onclick="saveMix()">Save to NVS</button>
 </div>
 <div class="status">Range 1.00–2.00. Rear motors always ×1.00. Higher = stronger front pitch authority (forward-CG compensation).</div>
</div>
<div class="actions">
 <button class="btn" onclick="save()">Save to NVS</button>
 <button class="btn" onclick="revert()">Revert from NVS</button>
 <button class="btn" onclick="reset()">Reset defaults</button>
 <button class="btn danger" onclick="calImu()">Calibrate IMU</button>
</div>
<div class="status" id="state">--</div>
<div class="grp"><h2>Tune log <span class="status" style="margin-left:8px">1 Hz (last 60 s shown)</span>
 <button class="btn" style="float:right;padding:4px 10px;font-size:12px" onclick="openFullLog()">Full log</button>
</h2>
 <pre id="tuneLog" style="font:11px/1.35 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;background:#0c0f12;color:#cdd2db;border:1px solid #2a2f38;border-radius:6px;padding:8px 10px;margin:0;max-height:240px;overflow-y:auto;white-space:pre">awaiting tune data...</pre>
</div>
<div id="fullLogPanel" class="grp" style="display:none">
 <h2>Full tune log <span class="status" id="fullLogStats" style="margin-left:8px">0 entries</span>
  <button class="btn" style="float:right;padding:4px 10px;font-size:12px;margin-left:6px" onclick="closeFullLog()">Close</button>
  <button class="btn" style="float:right;padding:4px 10px;font-size:12px;margin-left:6px" onclick="clearFullLog()">Clear</button>
  <button class="btn" style="float:right;padding:4px 10px;font-size:12px" onclick="copyFullLog()">Copy to clipboard</button>
 </h2>
 <div class="status" style="margin:4px 0 8px">Select-all + copy also works (click in box, Ctrl+A, Ctrl+C). Newest entries first. Up to ~10 minutes of history kept in browser memory.</div>
 <textarea id="fullLogText" readonly style="width:100%;height:380px;background:#0c0f12;color:#cdd2db;border:1px solid #2a2f38;border-radius:6px;padding:8px 10px;font:11px/1.35 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;resize:vertical;white-space:pre"></textarea>
</div>
<script>
// Auth token: priority is URL ?token=... > localStorage > prompt.
// Mutating (PUT/POST) requests carry it as the X-Auth-Token header. Read-only
// GETs (poll/state/tune) are not gated, so the page always renders.
let TOKEN=new URLSearchParams(location.search).get('token')||localStorage.getItem('fcuToken')||'';
if(TOKEN){try{localStorage.setItem('fcuToken',TOKEN);}catch(e){}}
function authHdrs(extra){const h=Object.assign({},extra||{});if(TOKEN)h['X-Auth-Token']=TOKEN;return h;}
function updateAuthBadge(){const el=document.getElementById('authState');if(!el)return;el.textContent=TOKEN?('auth: OK ('+TOKEN.length+' chars)'):'auth: NO TOKEN - click Set token';el.className=TOKEN?'ok':'err';}
function setToken(){const t=prompt('Paste FCU auth token (value of FCU_PID_AUTH_TOKEN in pidweb_secrets.h):',TOKEN);if(t===null)return;TOKEN=t.trim();try{if(TOKEN)localStorage.setItem('fcuToken',TOKEN);else localStorage.removeItem('fcuToken');}catch(e){}updateAuthBadge();poll();}
function clearToken(){TOKEN='';try{localStorage.removeItem('fcuToken');}catch(e){}updateAuthBadge();}
let auth401Notified=false;
async function afetch(url,opts){
 const r=await fetch(url,opts);
 if(r.status===401 && !auth401Notified){
  auth401Notified=true;
  setTimeout(()=>{auth401Notified=false;},5000);
  if(!TOKEN){alert('Auth required. Click Set token at top and paste FCU_PID_AUTH_TOKEN.');}
  else{alert('Auth rejected (401). Token is wrong - click Set token to update.');}
 }
 return r;
}
// Per-field slider ceilings match the FCU's safety clamps in main.cpp:
//   MAX_PID_KP_MILLI = 5000   (rate P)
//   MAX_PID_KI_MILLI = 3000   (rate I)
//   MAX_PID_KD_MILLI = 1000   (rate D — kept tighter, D amplifies gyro noise)
//   angle gains      = 10000  (clamped via safeAngleGainFromMilli)
// FCU re-clamps anything that gets through anyway, so the UI just tells the
// truth about what's accepted. Bumped from the original 1500 ceiling on rate
// gains because heavier airframes can need P/I above 1.5 to fight gravity moment.
const F=[
 {k:'rrP',l:'Rate Roll P', min:0,max:5000, step:25, grp:'Roll rate'},
 {k:'rrI',l:'Rate Roll I', min:0,max:3000, step:25, grp:'Roll rate'},
 {k:'rrD',l:'Rate Roll D', min:0,max:1000, step:5,  grp:'Roll rate'},
 {k:'rpP',l:'Rate Pitch P',min:0,max:5000, step:25, grp:'Pitch rate'},
 {k:'rpI',l:'Rate Pitch I',min:0,max:3000, step:25, grp:'Pitch rate'},
 {k:'rpD',l:'Rate Pitch D',min:0,max:1000, step:5,  grp:'Pitch rate'},
 {k:'ryP',l:'Rate Yaw P',  min:0,max:5000, step:25, grp:'Yaw rate'},
 {k:'ryI',l:'Rate Yaw I',  min:0,max:3000, step:25, grp:'Yaw rate'},
 {k:'ryD',l:'Rate Yaw D',  min:0,max:1000, step:5,  grp:'Yaw rate'},
 {k:'aR', l:'Angle Roll P',min:0,max:10000,step:100,grp:'Angle'},
 {k:'aP', l:'Angle Pitch P',min:0,max:10000,step:100,grp:'Angle'},
 {k:'aY', l:'Angle Yaw P', min:0,max:10000,step:100,grp:'Angle'}];
let gains=new Array(12).fill(0);
function buildUI(){
 const groups={};
 F.forEach((f,i)=>{(groups[f.grp]=groups[f.grp]||[]).push({...f,i});});
 const root=document.getElementById('grps'); root.innerHTML='';
 for(const g in groups){
  const d=document.createElement('div');d.className='grp';d.innerHTML=`<h2>${g}</h2>`;
  groups[g].forEach(f=>{
   const r=document.createElement('div');r.className='row';
   r.innerHTML=`<label>${f.l}</label><input type="range" min="${f.min}" max="${f.max}" step="${f.step}" id="r${f.i}"><div class="v" id="v${f.i}">--</div>`;
   d.appendChild(r);
   const inp=r.querySelector('input');
   inp.addEventListener('input',e=>{gains[f.i]=+e.target.value;document.getElementById('v'+f.i).textContent=(gains[f.i]/1000).toFixed(3);push();});
  });
  root.appendChild(d);
 }
}
function setUI(g){ F.forEach((f,i)=>{const r=document.getElementById('r'+i);if(r){r.value=g[i];document.getElementById('v'+i).textContent=(g[i]/1000).toFixed(3);}}); gains=g.slice();}
async function poll(){
 try{ const r=await fetch('/api/pid'); const j=await r.json(); if(j.gains&&j.gains.length===12){setUI(j.gains);} document.getElementById('safe').textContent=j.safe?'safe to edit':'ARMED - writes refused';document.getElementById('safe').className=j.safe?'ok':'err'; }catch(e){document.getElementById('safe').textContent='link down';document.getElementById('safe').className='err';}
 try{ const r=await fetch('/api/state'); const j=await r.json(); document.getElementById('state').textContent=`r=${j.roll.toFixed(1)}° p=${j.pitch.toFixed(1)}° y=${j.yaw.toFixed(1)}° thr=${j.thr}% loop=${j.loopHz}Hz link=${j.link?'up':'down'}${j.failsafe?' FAILSAFE':''}`; }catch(e){}
 try{ const r=await fetch('/api/mix'); const j=await r.json(); if(typeof j.bias==='number'){document.getElementById('mixBiasNow').textContent=j.bias.toFixed(3);} }catch(e){}
}
async function pushMix(){
 const v=parseFloat(document.getElementById('mixBias').value);
 if(!(v>=1.0&&v<=2.0)){alert('Bias must be 1.00–2.00');return;}
 try{ const r=await afetch('/api/mix',{method:'PUT',headers:authHdrs({'content-type':'application/json'}),body:JSON.stringify({bias:v})});
  if(r.ok){poll();}else{let msg='Apply failed';try{const j=await r.json();if(j.error)msg+=': '+j.error;}catch(e){}alert(msg);}
 }catch(e){alert('Apply failed: '+e);}
}
async function saveMix(){
 await pushMix();  // ensure RAM matches UI before persisting
 await postAction('/api/mix/save','Mixer bias saved to NVS','Save failed');
}
let pushTimer=null;
function push(){ clearTimeout(pushTimer); pushTimer=setTimeout(async()=>{ try{await afetch('/api/pid',{method:'PUT',headers:authHdrs({'content-type':'application/json'}),body:JSON.stringify({gains})});}catch(e){} },120);}
async function save(){
 const r=await afetch('/api/pid/save',{method:'POST',headers:authHdrs()});
 if(r.ok){ alert('Saved to NVS'); return; }
 let msg='Save failed';
 try{ const j=await r.json(); if(j.error) msg += ': '+j.error; }catch(e){}
 alert(msg);
}
async function postAction(url, okText, failText){
 const r=await afetch(url,{method:'POST',headers:authHdrs()});
 if(r.ok){ if(okText) alert(okText); poll(); return true; }
 let msg=failText;
 try{ const j=await r.json(); if(j.error) msg += ': '+j.error; }catch(e){}
 alert(msg); return false;
}
async function revert(){ await postAction('/api/pid/revert','Reverted from NVS','Revert failed');}
async function reset(){ if(!confirm('Reset NVS to compile-time defaults?'))return; await postAction('/api/pid/reset','Defaults restored','Reset failed');}
async function calImu(){ if(!confirm('Keep drone completely still. Throttle must be zero. Start IMU calibration?'))return; await postAction('/api/imu/calibrate','IMU calibration requested. Keep still for 1 second.','IMU calibration failed');}
async function spinMotor(n){ await postAction('/api/motor/spin?m='+n,'Motor '+n+' pulse requested','Motor pulse failed');}
// ---- Tune log (1 Hz). Newest line on top so you don't have to scroll.
// kTuneCompactMax = how many entries to show in the always-visible compact
// view (last 60 s).  kTuneLogMax = how many to keep in memory for the
// "Full log" panel (10 min). Old entries past kTuneLogMax fall off.
const kTuneCompactMax = 60;
const kTuneLogMax = 600;
const tuneLog = [];
function fmt(n,w,d){ const s=(n>=0?' ':'')+n.toFixed(d===undefined?2:d); return s.padStart(w); }
async function pollTune(){
 try{
  const r=await fetch('/api/tune');
  if(!r.ok) return;
  const j=await r.json();
  const ts=new Date();
  const hh=String(ts.getHours()).padStart(2,'0');
  const mm=String(ts.getMinutes()).padStart(2,'0');
  const ss=String(ts.getSeconds()).padStart(2,'0');
  const head=`${hh}:${mm}:${ss} thr=${String(j.thr).padStart(3)}% bias=${j.bias.toFixed(2)} loop=${String(j.loopHz).padStart(3)}Hz`;
  const att =`  att P/R/Y =${fmt(j.att[1],8)}°/${fmt(j.att[0],8)}°/${fmt(j.att[2],8)}°`;
  const gyr =`  gyro xyz =${fmt(j.gyro[0],8)}/${fmt(j.gyro[1],8)}/${fmt(j.gyro[2],8)} dps`;
  const psp =`  rateSP P/R =${fmt(j.rateSp[1],8)}/${fmt(j.rateSp[0],8)} dps`;
  const pp  =`  pitchPID  P=${fmt(j.pitch.P,7)}  I=${fmt(j.pitch.I,7)}  D=${fmt(j.pitch.D,7)}  out=${fmt(j.pitch.out,7)}`;
  const rp  =`  rollPID   P=${fmt(j.roll.P,7)}   I=${fmt(j.roll.I,7)}   D=${fmt(j.roll.D,7)}   out=${fmt(j.roll.out,7)}  yawOut=${fmt(j.yawOut,7)}`;
  const mot =`  motors    M1=${String(j.motors[0]).padStart(4)}  M2=${String(j.motors[1]).padStart(4)}  M3=${String(j.motors[2]).padStart(4)}  M4=${String(j.motors[3]).padStart(4)}  (spread F/R: ${Math.abs(j.motors[1]-j.motors[0])})`;
  tuneLog.unshift([head,att,gyr,psp,pp,rp,mot,''].join('\n'));
  if(tuneLog.length>kTuneLogMax) tuneLog.length=kTuneLogMax;
  // Compact view (last 60 entries) — always rendered.
  document.getElementById('tuneLog').textContent=tuneLog.slice(0,kTuneCompactMax).join('\n');
  // Full-log textarea — only updated while the panel is visible to save CPU.
  const fp=document.getElementById('fullLogPanel');
  if(fp && fp.style.display!=='none'){
   document.getElementById('fullLogText').value=tuneLog.join('\n');
   document.getElementById('fullLogStats').textContent=tuneLog.length+' entries (~'+tuneLog.length+' s of history)';
  }
 }catch(e){ /* silent — webserver may be busy */ }
}
function openFullLog(){
 document.getElementById('fullLogPanel').style.display='block';
 document.getElementById('fullLogText').value=tuneLog.join('\n');
 document.getElementById('fullLogStats').textContent=tuneLog.length+' entries (~'+tuneLog.length+' s of history)';
 // Scroll the panel into view so the user sees it.
 document.getElementById('fullLogPanel').scrollIntoView({behavior:'smooth',block:'start'});
}
function closeFullLog(){ document.getElementById('fullLogPanel').style.display='none'; }
function clearFullLog(){
 if(!confirm('Clear all '+tuneLog.length+' log entries from browser memory?')) return;
 tuneLog.length=0;
 document.getElementById('tuneLog').textContent='cleared';
 document.getElementById('fullLogText').value='';
 document.getElementById('fullLogStats').textContent='0 entries';
}
async function copyFullLog(){
 const text=tuneLog.join('\n');
 if(text.length===0){ alert('Log is empty — wait at least 1 s after page load.'); return; }
 // Try modern Clipboard API first. Falls back to textarea+execCommand on
 // older browsers or non-secure HTTP contexts where navigator.clipboard
 // isn't exposed.
 try{
  if(navigator.clipboard && window.isSecureContext){
   await navigator.clipboard.writeText(text);
   alert('Copied '+tuneLog.length+' entries ('+text.length+' chars) to clipboard.');
   return;
  }
 }catch(e){ /* fall through to legacy */ }
 const ta=document.getElementById('fullLogText');
 ta.value=text;  // ensure it has latest
 ta.focus();
 ta.select();
 const ok=document.execCommand('copy');
 if(ok){ alert('Copied '+tuneLog.length+' entries to clipboard.'); }
 else{ alert('Auto-copy blocked by browser. Use Ctrl+A then Ctrl+C in the text box.'); }
}
buildUI(); updateAuthBadge();
if(!TOKEN){ setTimeout(()=>{ if(!TOKEN) setToken(); }, 300); }
poll(); setInterval(poll,500); setInterval(pollTune,1000); pollTune();
</script></body></html>)HTML";

// ---- helpers ----
esp_err_t sendJson(httpd_req_t* req, const char* body) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_sendstr(req, body);
}

esp_err_t sendError(httpd_req_t* req, int httpStatus, const char* reason) {
  char status[16];
  snprintf(status, sizeof(status), "%d", httpStatus);
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, "application/json");
  char body[96];
  snprintf(body, sizeof(body), "{\"error\":\"%s\"}", reason);
  return httpd_resp_sendstr(req, body);
}

// Receive up to maxLen bytes of request body into buf. Returns bytes read
// (terminated with \0) or -1 on error.
int recvBody(httpd_req_t* req, char* buf, size_t maxLen) {
  if (req->content_len == 0 || req->content_len >= maxLen) {
    return -1;
  }
  size_t total = 0;
  while (total < req->content_len) {
    int n = httpd_req_recv(req, buf + total, req->content_len - total);
    if (n <= 0) {
      if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
      return -1;
    }
    total += static_cast<size_t>(n);
  }
  buf[total] = '\0';
  return static_cast<int>(total);
}

// Minimalist JSON scalar-field parser for {"key":<number>}. Returns true if
// the field was found and parsed; out gets the float value. Doesn't handle
// arbitrary JSON; just enough for the mixer bias endpoint.
bool parseJsonFloat(const char* body, const char* key, float& out) {
  // Find the key with the quotes intact, e.g. "bias"
  char needle[24];
  const int n = snprintf(needle, sizeof(needle), "\"%s\"", key);
  if (n < 0 || n >= (int)sizeof(needle)) return false;
  const char* p = strstr(body, needle);
  if (!p) return false;
  p += n;
  // Skip whitespace and the colon
  while (*p == ' ' || *p == ':' || *p == '\t' || *p == '\n') ++p;
  char* end = nullptr;
  const float v = strtof(p, &end);
  if (end == p) return false;
  out = v;
  return true;
}

bool parseJsonFloat4(const char* body, const char* key, float out[4]) {
  if (!body || !key || !out) return false;
  char needle[24];
  const int n = snprintf(needle, sizeof(needle), "\"%s\"", key);
  if (n < 0 || n >= static_cast<int>(sizeof(needle))) return false;
  const char* p = strstr(body, needle);
  if (!p) return false;
  p = strchr(p + n, '[');
  if (!p) return false;
  ++p;
  for (uint8_t i = 0; i < 4; ++i) {
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == ',') ++p;
    char* end = nullptr;
    const float v = strtof(p, &end);
    if (end == p || !isfinite(v)) return false;
    out[i] = v;
    p = end;
  }
  while (*p == ' ' || *p == '\t' || *p == '\n') ++p;
  return *p == ']';
}

// Minimalist JSON-array parser for {"gains":[12 ints]}. Doesn't handle
// arbitrary JSON; just enough to extract the gains. Returns true on success.
bool parseGainsArray(const char* body, int16_t out[12]) {
  const char* p = strstr(body, "\"gains\"");
  if (!p) return false;
  p = strchr(p, '[');
  if (!p) return false;
  ++p;
  int count = 0;
  while (count < 12) {
    while (*p == ' ' || *p == ',' || *p == '\t' || *p == '\n') ++p;
    if (*p == ']') break;
    char* end = nullptr;
    long v = strtol(p, &end, 10);
    if (end == p) return false;
    if (v < INT16_MIN) v = INT16_MIN;
    if (v > INT16_MAX) v = INT16_MAX;
    out[count++] = static_cast<int16_t>(v);
    p = end;
  }
  return count == 12;
}

bool parseRollPitch(const char* body, float& roll, float& pitch) {
  return parseJsonFloat(body, "roll", roll) && parseJsonFloat(body, "pitch", pitch);
}

// ============================================================================
// Live telemetry: JSON frame builder + WebSocket push.
// ----------------------------------------------------------------------------
// The dashboard receives the full signal-path frame ~25 Hz over a WebSocket.
// A dedicated low-priority task (core 0, prio 2 — below httpd's 3, far below
// flight/radio/sensor) reads the snapshot via getDashTelemetry, formats JSON,
// and pushes to every connected WS client. The flight loop is never touched.
// ============================================================================
std::atomic<uint8_t> gWsClients{0};
std::atomic<uint16_t> gWebTelemHz{0};
TaskHandle_t gTelemTask = nullptr;

// Bounded JSON appender: vsnprintf into a fixed buffer, truncation-safe.
struct JsonAppender {
  char* buf;
  size_t cap;
  size_t len;
  void f(const char* fmt, ...) {
    if (len + 1 >= cap) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + len, cap - len, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (static_cast<size_t>(n) >= cap - len) {
      len = cap - 1;  // truncated — stop appending
    } else {
      len += static_cast<size_t>(n);
    }
  }
};
// JSON has no NaN/Inf literal — sanitize every float so a transient bad value
// can never produce a frame the browser's JSON.parse rejects.
static float jf(float v) { return isfinite(v) ? v : 0.0f; }

int formatDashJson(char* out, size_t cap, const DashTelemetry& d,
                   uint8_t wsClients, uint16_t webHz) {
  JsonAppender j{out, cap, 0};
  j.f("{\"sys\":{\"up\":%lu,\"armed\":%d,\"fs\":%d,\"fsr\":%u,\"fsb\":%d,\"fsc\":%d,\"link\":%d,\"thr\":%u,"
      "\"loop\":%u,\"bv\":%.2f,\"bp\":%d,\"blow\":%d,\"ben\":%d,\"heap\":%lu,"
      "\"minheap\":%lu,\"ovr\":%lu,\"maxus\":%lu,\"wsc\":%u,\"whz\":%u,\"mode\":%u,"
      "\"armf\":%lu,\"armlatch\":%d}",
      (unsigned long)d.uptimeMs, d.armed ? 1 : 0, d.failsafeActive ? 1 : 0, d.failsafeReason,
      d.failsafeBypass ? 1 : 0, d.failsafeBypassCompiledDefault ? 1 : 0,
      d.controlLinkUp ? 1 : 0, d.throttlePct, d.loopHz, jf(d.battVolts),
      (d.battPercent == 0xFF) ? -1 : (int)d.battPercent, d.battLow ? 1 : 0, d.battEnabled ? 1 : 0,
      (unsigned long)d.freeHeap, (unsigned long)d.minFreeHeap, (unsigned long)d.flightOverruns,
      (unsigned long)d.flightMaxUs, wsClients, webHz, d.flightMode,
      (unsigned long)d.armingFlags, d.armBlockedLatch ? 1 : 0);
  j.f(",\"att\":{\"rr\":%.2f,\"rp\":%.2f,\"ry\":%.2f,\"cr\":%.2f,\"cp\":%.2f,"
      "\"tr\":%.2f,\"tp\":%.2f,\"ty\":%.2f,\"yh\":%d,\"tyr\":%.1f,"
      "\"er\":%.2f,\"ep\":%.2f,\"at\":%d}",
      jf(d.rawRollDeg), jf(d.rawPitchDeg), jf(d.rawYawDeg), jf(d.corrRollDeg), jf(d.corrPitchDeg),
      jf(d.targetRollDeg), jf(d.targetPitchDeg), jf(d.targetYawDeg), d.yawHoldActive ? 1 : 0,
      jf(d.targetYawRateDps), jf(d.rollErrDeg), jf(d.pitchErrDeg), d.accelTrusted ? 1 : 0);
  j.f(",\"imu\":{\"a\":[%.4f,%.4f,%.4f],\"ao\":[%.4f,%.4f,%.4f],\"av\":%d,\"am\":%.4f,"
      "\"g\":[%.3f,%.3f,%.3f],\"graw\":[%.3f,%.3f,%.3f],\"gb\":[%.3f,%.3f,%.3f],\"gbv\":%d,\"rdy\":%d}",
      jf(d.accelG[0]), jf(d.accelG[1]), jf(d.accelG[2]), jf(d.accelOffG[0]), jf(d.accelOffG[1]),
      jf(d.accelOffG[2]), d.accelValid ? 1 : 0, jf(d.accelMag), jf(d.gyroDps[0]), jf(d.gyroDps[1]),
      jf(d.gyroDps[2]), jf(d.gyroRawDps[0]), jf(d.gyroRawDps[1]), jf(d.gyroRawDps[2]),
      jf(d.gyroBiasDps[0]), jf(d.gyroBiasDps[1]), jf(d.gyroBiasDps[2]),
      d.gyroBiasValid ? 1 : 0, d.imuReady ? 1 : 0);
  j.f(",\"lvl\":{\"o\":[%.3f,%.3f],\"t\":[%.3f,%.3f],\"ld\":%d,\"st\":%u,\"er\":%u,\"n\":%u,"
      "\"std\":[%.3f,%.3f]}",
      jf(d.levelOffsetDeg[0]), jf(d.levelOffsetDeg[1]), jf(d.trimDeg[0]), jf(d.trimDeg[1]),
      d.levelLoaded ? 1 : 0, d.levelCalState, d.levelCalErr, d.levelCalAccepted,
      jf(d.levelCalStdDeg[0]), jf(d.levelCalStdDeg[1]));
  j.f(",\"pid\":{\"r\":[%.2f,%.2f,%.2f,%.2f,%.2f],\"p\":[%.2f,%.2f,%.2f,%.2f,%.2f],"
      "\"y\":[%.2f,%.2f,%.2f,%.2f,%.2f],\"sp\":[%.2f,%.2f,%.2f],\"smin\":%d,\"smax\":%d,"
      "\"ssc\":%d,\"sat\":[%d,%d,%d]}",
      jf(d.pidRoll[0]), jf(d.pidRoll[1]), jf(d.pidRoll[2]), jf(d.pidRoll[3]), jf(d.pidRoll[4]),
      jf(d.pidPitch[0]), jf(d.pidPitch[1]), jf(d.pidPitch[2]), jf(d.pidPitch[3]), jf(d.pidPitch[4]),
      jf(d.pidYaw[0]), jf(d.pidYaw[1]), jf(d.pidYaw[2]), jf(d.pidYaw[3]), jf(d.pidYaw[4]),
      jf(d.rateSpDps[0]), jf(d.rateSpDps[1]), jf(d.rateSpDps[2]),
      d.satMin ? 1 : 0, d.satMax ? 1 : 0, d.satScaled ? 1 : 0,
      d.pidSat[0] ? 1 : 0, d.pidSat[1] ? 1 : 0, d.pidSat[2] ? 1 : 0);
  j.f(",\"mix\":{\"base\":%.1f,\"r\":%.2f,\"pf\":%.2f,\"pr\":%.2f,\"y\":%.2f,"
      "\"unc\":[%.1f,%.1f,%.1f,%.1f],\"m\":[%u,%u,%u,%u],\"bias\":%.3f,"
      "\"trim\":[%.3f,%.3f,%.3f,%.3f]}",
      jf(d.mixBase), jf(d.mixRoll), jf(d.mixPitchFront), jf(d.mixPitchRear), jf(d.mixYaw),
      jf(d.mixUnclamped[0]), jf(d.mixUnclamped[1]), jf(d.mixUnclamped[2]), jf(d.mixUnclamped[3]),
      d.motorRaw[0], d.motorRaw[1], d.motorRaw[2], d.motorRaw[3], jf(d.mixBias),
      jf(d.motorTrim[0]), jf(d.motorTrim[1]), jf(d.motorTrim[2]), jf(d.motorTrim[3]));
  j.f(",\"rc\":{\"comp\":%d,\"up\":%d,\"fs\":%d,\"lq\":%u,\"rssi\":%d,\"fr\":%u,\"age\":%lu,"
      "\"loss\":%u,\"pps\":%u,\"ch\":[%u,%u,%u,%u,%u,%u,%u,%u]}",
      d.crsfCompiled ? 1 : 0, d.rcLinkUp ? 1 : 0, d.rcFailsafe ? 1 : 0, d.rcLq, d.rcRssiDbm,
      d.rcFrameRateHz, (unsigned long)d.rcFrameAgeMs, d.rcLossPercent, d.rcPacketsPerSec,
      d.rcChannelsUs[0], d.rcChannelsUs[1], d.rcChannelsUs[2], d.rcChannelsUs[3],
      d.rcChannelsUs[4], d.rcChannelsUs[5], d.rcChannelsUs[6], d.rcChannelsUs[7]);
  j.f(",\"sen\":{\"baro\":{\"r\":%d,\"v\":%d,\"pa\":%.1f,\"alt\":%.2f,\"t\":%.1f,\"age\":%lu},"
      "\"tof\":{\"comp\":%d,\"r\":%d,\"rng\":%d,\"mm\":%u,\"age\":%lu},"
      "\"gps\":{\"comp\":%d,\"r\":%d,\"fix\":%d,\"sats\":%u,\"q\":%u,\"hdop\":%.1f,\"hdv\":%d,"
      "\"lat\":%ld,\"lon\":%ld,\"age\":%lu,"
      "\"gsv\":%d,\"cv\":%d,\"vv\":%d,\"sp\":%u,\"spd\":%.2f,\"cog\":%u,\"vn\":%.2f,\"ve\":%.2f,\"rmc\":%lu},"
      "\"mag\":{\"v\":%d,\"hdg\":%.1f,\"f\":%.1f,\"cal\":%d,\"src\":%u,\"gain\":%.3f,"
      "\"trim\":%.1f,\"dec\":%.1f,"
      "\"ext\":{\"comp\":%d,\"conn\":%d,\"v\":%d,\"cal\":%d,\"hdg\":%.1f,\"f\":%.1f,"
      "\"xyz\":[%.2f,%.2f,%.2f],\"rej\":%u},"
      "\"cfg\":{\"ext\":%d,\"ob\":%d,\"pref\":%d}},\"ekf\":%d,"
      "\"ekfd\":{\"av\":%d,\"pv\":%d,\"vv\":%d,\"gv\":%d,\"mv\":%d,\"if\":%d,\"yaw\":%.1f,"
      "\"v\":[%.2f,%.2f,%.2f],\"p\":[%.1f,%.1f,%.1f],\"gi\":[%.1f,%.1f,%.1f],"
      "\"mi\":%.1f,\"gps\":[%lu,%lu],\"mag\":[%lu,%lu],\"drop\":%lu}}",
      d.baroReady ? 1 : 0, d.baroValid ? 1 : 0, jf(d.baroPa), jf(d.baroAltM), jf(d.baroTempC),
      (unsigned long)d.baroAgeMs, d.tofCompiled ? 1 : 0, d.tofReady ? 1 : 0, d.tofRanging ? 1 : 0,
      d.tofMm, (unsigned long)d.tofAgeMs, d.gpsCompiled ? 1 : 0, d.gpsReady ? 1 : 0, d.gpsFix ? 1 : 0,
      d.gpsSats, d.gpsFixQual, jf(d.gpsHdop), d.gpsHdopValid ? 1 : 0,
      (long)d.gpsLatE7, (long)d.gpsLonE7, (unsigned long)d.gpsAgeMs,
      d.gpsGroundSpeedValid ? 1 : 0, d.gpsCourseValid ? 1 : 0, d.gpsVelocityValid ? 1 : 0,
      d.gpsGroundSpeedKmh10, jf(d.gpsGroundSpeedMs), d.gpsCourseCentiDeg,
      jf(d.gpsVelNorthMs), jf(d.gpsVelEastMs), (unsigned long)d.gpsRmcAgeMs,
      d.magValid ? 1 : 0, jf(d.magHeadingDeg), jf(d.magFieldUt), d.magCalValid ? 1 : 0,
      d.activeMagSource, jf(d.magYawCorrGain), jf(d.magHeadingTrimDeg), jf(d.magDeclinationDeg),
      d.extMagCompiled ? 1 : 0, d.extMagConnected ? 1 : 0, d.extMagValid ? 1 : 0,
      d.extMagCalValid ? 1 : 0, jf(d.extMagHeadingDeg), jf(d.extMagFieldUt),
      jf(d.extMagVecUt[0]), jf(d.extMagVecUt[1]), jf(d.extMagVecUt[2]),
      d.extMagRejectReason, d.magExtEnabled ? 1 : 0, d.magOnboardEnabled ? 1 : 0,
      d.magPreferExternal ? 1 : 0,
      d.ekfReady ? 1 : 0,
      d.ekfAttValid ? 1 : 0, d.ekfPosValid ? 1 : 0, d.ekfVelValid ? 1 : 0,
      d.ekfGpsValid ? 1 : 0, d.ekfMagValid ? 1 : 0, d.ekfInnovationFault ? 1 : 0,
      jf(d.ekfYawDeg), jf(d.ekfVelNed[0]), jf(d.ekfVelNed[1]), jf(d.ekfVelNed[2]),
      jf(d.ekfPosNed[0]), jf(d.ekfPosNed[1]), jf(d.ekfPosNed[2]),
      jf(d.ekfGpsInnov[0]), jf(d.ekfGpsInnov[1]), jf(d.ekfGpsInnov[2]),
      jf(d.ekfMagInnovDeg), (unsigned long)d.ekfGpsAccept, (unsigned long)d.ekfGpsReject,
      (unsigned long)d.ekfMagAccept, (unsigned long)d.ekfMagReject,
      (unsigned long)d.ekfMeasDropped);
  j.f(",\"pi\":{\"comp\":%d,\"alive\":%d,\"hb\":%lu},"
      "\"srv\":{\"att\":%d,\"pan\":%u,\"tilt\":%u,\"pant\":%u,\"tiltt\":%u}}",
      d.piCompiled ? 1 : 0, d.piLinkAlive ? 1 : 0, (unsigned long)d.piHeartbeatAgeMs,
      d.servoAttached ? 1 : 0, d.panUs, d.tiltUs, d.panTargetUs, d.tiltTargetUs);
  return static_cast<int>(j.len);
}

// Push one text frame to every connected WebSocket client. Enumerates the
// httpd client list and filters to WS sockets — no manual fd bookkeeping.
void wsBroadcast(const char* json, size_t len) {
  if (gServer == nullptr) return;
  int fds[8];
  size_t num = sizeof(fds) / sizeof(fds[0]);
  if (httpd_get_client_list(gServer, &num, fds) != ESP_OK) return;
  httpd_ws_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  frame.type = HTTPD_WS_TYPE_TEXT;
  frame.payload = reinterpret_cast<uint8_t*>(const_cast<char*>(json));
  frame.len = len;
  uint8_t wsCount = 0;
  for (size_t i = 0; i < num; ++i) {
    if (httpd_ws_get_fd_info(gServer, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
      ++wsCount;
      (void)httpd_ws_send_frame_async(gServer, fds[i], &frame);
    }
  }
  gWsClients.store(wsCount, std::memory_order_relaxed);
}

// WS endpoint. The framework completes the handshake on the GET upgrade; we
// only push telemetry, so inbound frames are drained and ignored.
esp_err_t handleWs(httpd_req_t* req) {
  if (req->method == HTTP_GET) {
    return ESP_OK;  // handshake done by the framework
  }
  httpd_ws_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  frame.type = HTTPD_WS_TYPE_TEXT;
  uint8_t buf[64];
  frame.payload = buf;
  return httpd_ws_recv_frame(req, &frame, sizeof(buf) - 1);
}

void telemetryTask(void*) {
  static char buf[4400];  // own buffer (handler uses a separate one)
  uint32_t lastRateMs = millis();
  uint16_t frames = 0;
  for (;;) {
    if (gRunning.load() && gCb.getDashTelemetry != nullptr) {
      DashTelemetry t;
      gCb.getDashTelemetry(t);
      const int n = formatDashJson(buf, sizeof(buf), t,
                                   gWsClients.load(std::memory_order_relaxed),
                                   gWebTelemHz.load(std::memory_order_relaxed));
      if (n > 0) wsBroadcast(buf, static_cast<size_t>(n));
      ++frames;
    }
    const uint32_t now = millis();
    if (now - lastRateMs >= 1000U) {
      gWebTelemHz.store(frames, std::memory_order_relaxed);
      frames = 0;
      lastRateMs = now;
    }
    vTaskDelay(pdMS_TO_TICKS(40));  // ~25 Hz dashboard push
  }
}

// ---- handlers ----
esp_err_t handleIndex(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, PIDWEB_DASHBOARD_HTML, HTTPD_RESP_USE_STRLEN);
}

// Minimal legacy tuner kept as a fallback at /legacy (also keeps the original
// PROGMEM page referenced).
esp_err_t handleLegacyIndex(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html; charset=utf-8");
  return httpd_resp_send(req, kIndexHtml, HTTPD_RESP_USE_STRLEN);
}

esp_err_t handleGetPid(httpd_req_t* req) {
  if (!gCb.getPid) return sendError(req, 500, "no_callback");
  int16_t gains[12] = {};
  gCb.getPid(gains);
  char body[256];
  int n = snprintf(body, sizeof(body),
      "{\"gains\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d],\"safe\":%s}",
      gains[0], gains[1], gains[2], gains[3], gains[4], gains[5],
      gains[6], gains[7], gains[8], gains[9], gains[10], gains[11],
      gSafeToWrite.load() ? "true" : "false");
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}

esp_err_t handlePutPid(httpd_req_t* req) {
  if (!gCb.applyPid) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[384];
  int n = recvBody(req, body, sizeof(body));
  if (n < 0) return sendError(req, 400, "bad_body");
  int16_t gains[12] = {};
  if (!parseGainsArray(body, gains)) return sendError(req, 400, "bad_json");
  if (!gCb.applyPid(gains)) return sendError(req, 409, "apply_refused");
  return sendJson(req, "{\"ok\":true}");
}

esp_err_t handleSavePid(httpd_req_t* req) {
  if (!gCb.saveAllToNvs) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!gCb.saveAllToNvs()) return sendError(req, 500, "nvs_write_failed");
  return sendJson(req, "{\"ok\":true,\"persisted\":true}");
}

esp_err_t handleRevertPid(httpd_req_t* req) {
  if (!gCb.revertFromNvs) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!gCb.revertFromNvs()) return sendError(req, 500, "revert_failed");
  return sendJson(req, "{\"ok\":true}");
}

esp_err_t handleResetPid(httpd_req_t* req) {
  if (!gCb.resetToDefaults) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!gCb.resetToDefaults()) return sendError(req, 500, "reset_failed");
  return sendJson(req, "{\"ok\":true,\"defaults\":true}");
}

esp_err_t handleCalibrateImu(httpd_req_t* req) {
  if (!gCb.calibrateImu) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!gCb.calibrateImu()) return sendError(req, 409, "calibration_refused");
  return sendJson(req, "{\"ok\":true,\"requested\":true}");
}

esp_err_t handleSpinMotor(httpd_req_t* req) {
  if (!gCb.spinMotor) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");

  char query[32] = {};
  char motorText[8] = {};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "m", motorText, sizeof(motorText)) != ESP_OK) {
    return sendError(req, 400, "missing_motor");
  }
  const long motor = strtol(motorText, nullptr, 10);
  if (motor < 1 || motor > 4) {
    return sendError(req, 400, "bad_motor");
  }
  if (!gCb.spinMotor(static_cast<uint8_t>(motor))) {
    return sendError(req, 409, "motor_spin_refused");
  }
  return sendJson(req, "{\"ok\":true,\"requested\":true}");
}

esp_err_t handleGetMix(httpd_req_t* req) {
  if (!gCb.getMixPitchFrontBias) return sendError(req, 500, "no_callback");
  float bias = 1.0f;
  float trims[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  gCb.getMixPitchFrontBias(bias);
  if (gCb.getMotorThrustTrims) gCb.getMotorThrustTrims(trims);
  char body[128];
  int n = snprintf(body, sizeof(body),
      "{\"bias\":%.3f,\"trims\":[%.3f,%.3f,%.3f,%.3f],\"safe\":%s}",
      static_cast<double>(bias),
      static_cast<double>(trims[0]), static_cast<double>(trims[1]),
      static_cast<double>(trims[2]), static_cast<double>(trims[3]),
      gSafeToWrite.load() ? "true" : "false");
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}

esp_err_t handlePutMix(httpd_req_t* req) {
  if (!gCb.setMixPitchFrontBias || !gCb.setMotorThrustTrims) {
    return sendError(req, 500, "no_callback");
  }
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[192];
  int n = recvBody(req, body, sizeof(body));
  if (n < 0) return sendError(req, 400, "bad_body");

  float bias = 0.0f;
  float trims[4] = {};
  const bool hasBias = parseJsonFloat(body, "bias", bias);
  const bool hasTrims = parseJsonFloat4(body, "trims", trims);
  if (!hasBias && !hasTrims) return sendError(req, 400, "bad_json");
  if (hasBias && !(bias >= 1.0f && bias <= 2.0f)) {
    return sendError(req, 400, "bias_out_of_range");
  }
  if (hasTrims) {
    for (float trim : trims) {
      if (!(trim >= 0.90f && trim <= 1.10f)) {
        return sendError(req, 400, "trim_out_of_range");
      }
    }
  }
  if (hasBias && !gCb.setMixPitchFrontBias(bias)) {
    return sendError(req, 409, "apply_refused");
  }
  if (hasTrims && !gCb.setMotorThrustTrims(trims)) {
    return sendError(req, 409, "apply_refused");
  }
  return sendJson(req, "{\"ok\":true}");
}

esp_err_t handleSaveMix(httpd_req_t* req) {
  if (!gCb.saveMixPitchFrontBiasToNvs || !gCb.saveMotorThrustTrimsToNvs) {
    return sendError(req, 500, "no_callback");
  }
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  const bool biasOk = gCb.saveMixPitchFrontBiasToNvs();
  const bool trimsOk = gCb.saveMotorThrustTrimsToNvs();
  if (!biasOk || !trimsOk) return sendError(req, 500, "nvs_write_failed");
  return sendJson(req, "{\"ok\":true,\"persisted\":true}");
}

esp_err_t handlePutFailsafe(httpd_req_t* req) {
  if (!gCb.setFailsafeBypass) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[96];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float bypass = 0.0f;
  if (!parseJsonFloat(body, "bypass", bypass)) return sendError(req, 400, "bad_json");
  if (!gCb.setFailsafeBypass(bypass > 0.5f)) return sendError(req, 409, "apply_refused");
  return sendJson(req, "{\"ok\":true}");
}

esp_err_t handleSaveFailsafe(httpd_req_t* req) {
  if (!gCb.saveFailsafeBypassToNvs) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!gCb.saveFailsafeBypassToNvs()) return sendError(req, 500, "nvs_write_failed");
  return sendJson(req, "{\"ok\":true,\"persisted\":true}");
}

esp_err_t handleGetState(httpd_req_t* req) {
  if (!gCb.getState) return sendError(req, 500, "no_callback");
  StateSnapshot s;
  gCb.getState(s);
  char body[224];
  int n = snprintf(body, sizeof(body),
      "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"thr\":%u,\"loopHz\":%u,"
      "\"link\":%s,\"failsafe\":%s,\"armed\":%s}",
      s.rollDeg, s.pitchDeg, s.yawDeg, (unsigned)s.throttlePct, (unsigned)s.loopHz,
      s.controlLinkUp ? "true" : "false",
      s.failsafeActive ? "true" : "false",
      s.armed ? "true" : "false");
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}

esp_err_t handleGetTune(httpd_req_t* req) {
  if (!gCb.getTune) return sendError(req, 500, "no_callback");
  TuneSnapshot t;
  gCb.getTune(t);
  // Compact-but-readable JSON. Roughly 480 bytes; fits 768-byte buffer with
  // room to spare. Order: attitude, rate setpoints, gyro, pitch terms, roll
  // terms, yaw output, motors, throttle, bias, loopHz.
  char body[768];
  int n = snprintf(body, sizeof(body),
      "{\"att\":[%.2f,%.2f,%.2f],"
      "\"rateSp\":[%.2f,%.2f,%.2f],"
      "\"gyro\":[%.2f,%.2f,%.2f],"
      "\"pitch\":{\"P\":%.2f,\"I\":%.2f,\"D\":%.2f,\"out\":%.2f},"
      "\"roll\":{\"P\":%.2f,\"I\":%.2f,\"D\":%.2f,\"out\":%.2f},"
      "\"yawOut\":%.2f,"
      "\"motors\":[%u,%u,%u,%u],"
      "\"thr\":%u,\"bias\":%.3f,\"loopHz\":%u}",
      t.rollDeg, t.pitchDeg, t.yawDeg,
      t.rollRateSpDps, t.pitchRateSpDps, t.yawRateSpDps,
      t.gxDps, t.gyDps, t.gzDps,
      t.pitchP, t.pitchI, t.pitchD, t.pitchOut,
      t.rollP, t.rollI, t.rollD, t.rollOut,
      t.yawOut,
      (unsigned)t.motors[0], (unsigned)t.motors[1],
      (unsigned)t.motors[2], (unsigned)t.motors[3],
      (unsigned)t.throttlePct, t.mixPitchFrontBias, (unsigned)t.loopHz);
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}

// GET /api/status — craft summary: armed/mode/battery/timing/sensor health +
// the arming-disable bitmask expanded into human-readable reason names.
esp_err_t handleGetStatus(httpd_req_t* req) {
  if (!gCb.getStatus) return sendError(req, 500, "no_callback");
  StatusSnapshot s;
  gCb.getStatus(s);
  static char body[1280];
  JsonAppender j{body, sizeof(body), 0};
  j.f("{\"armed\":%s,\"mode\":%u,\"failsafe\":%s,\"failsafeReason\":%u,"
      "\"battVolts\":%.2f,\"battPercent\":%d,\"battEnabled\":%s,"
      "\"loopHz\":%u,\"loopDtUs\":{\"min\":%lu,\"max\":%lu,\"avg\":%lu},"
      "\"flightOverruns\":%lu,\"flightTaskMaxUs\":%lu,"
      "\"imuOk\":%s,\"gps\":{\"compiled\":%s,\"connected\":%s,\"fix\":%s,"
      "\"fixQuality\":%u,\"sats\":%u,\"hdop\":%.1f},"
      "\"magHealthy\":%s,\"magCalValid\":%s,\"baroHealthy\":%s,"
      "\"freeHeap\":%lu,\"uptimeMs\":%lu,"
      "\"armingDisableFlags\":%lu,\"armBlockedLatch\":%s,\"armingDisableReasons\":[",
      s.armed ? "true" : "false", s.flightMode,
      s.failsafeActive ? "true" : "false", s.failsafeReason,
      jf(s.battVolts), (s.battPercent == 0xFF) ? -1 : (int)s.battPercent,
      s.battEnabled ? "true" : "false",
      s.loopHz, (unsigned long)s.loopDtMinUs, (unsigned long)s.loopDtMaxUs,
      (unsigned long)s.loopDtAvgUs,
      (unsigned long)s.flightOverruns, (unsigned long)s.flightTaskMaxUs,
      s.imuOk ? "true" : "false",
      s.gpsCompiled ? "true" : "false", s.gpsConnected ? "true" : "false",
      s.gpsFix ? "true" : "false", s.gpsFixQuality, s.gpsSats, jf(s.gpsHdop),
      s.magHealthy ? "true" : "false", s.magCalValid ? "true" : "false",
      s.baroHealthy ? "true" : "false",
      (unsigned long)s.freeHeap, (unsigned long)s.uptimeMs,
      (unsigned long)s.armingDisableFlags, s.armBlockedLatch ? "true" : "false");
  bool first = true;
  for (uint8_t bit = 0; bit < arming::kFlagCount; ++bit) {
    if ((s.armingDisableFlags & (1UL << bit)) == 0U) continue;
    j.f("%s\"%s\"", first ? "" : ",", arming::flagName(bit));
    first = false;
  }
  j.f("]}");
  return sendJson(req, body);
}

// GET /api/sensors/live — one-shot full sensor readout (raw + filtered gyro,
// calibrated accel + offsets, mag, attitude, baro, GPS, rangefinder).
esp_err_t handleGetSensorsLive(httpd_req_t* req) {
  if (!gCb.getSensorsLive) return sendError(req, 500, "no_callback");
  SensorsLiveSnapshot s;
  gCb.getSensorsLive(s);
  static char body[1280];
  JsonAppender j{body, sizeof(body), 0};
  j.f("{\"gyroRaw\":[%.3f,%.3f,%.3f],\"gyroFilt\":[%.3f,%.3f,%.3f],"
      "\"accel\":[%.4f,%.4f,%.4f],\"accelOff\":[%.4f,%.4f,%.4f],"
      "\"mag\":[%.2f,%.2f,%.2f],\"magFieldUt\":%.1f,\"magHeadingDeg\":%.1f,\"magValid\":%s,"
      "\"attitude\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},"
      "\"baro\":{\"altM\":%.2f,\"pa\":%.1f,\"tempC\":%.1f,\"valid\":%s},"
      "\"gps\":{\"lat\":%ld,\"lon\":%ld,\"altDm\":%d,\"speedMs\":%.2f,\"courseDeg\":%.1f,"
      "\"hdop\":%.1f,\"sats\":%u,\"fixQuality\":%u,\"fix\":%s},"
      "\"tof\":{\"mm\":%u,\"valid\":%s},\"loopHz\":%u}",
      jf(s.gyroRawDps[0]), jf(s.gyroRawDps[1]), jf(s.gyroRawDps[2]),
      jf(s.gyroFiltDps[0]), jf(s.gyroFiltDps[1]), jf(s.gyroFiltDps[2]),
      jf(s.accelG[0]), jf(s.accelG[1]), jf(s.accelG[2]),
      jf(s.accelOffG[0]), jf(s.accelOffG[1]), jf(s.accelOffG[2]),
      jf(s.magUt[0]), jf(s.magUt[1]), jf(s.magUt[2]),
      jf(s.magFieldUt), jf(s.magHeadingDeg), s.magValid ? "true" : "false",
      jf(s.rollDeg), jf(s.pitchDeg), jf(s.yawDeg),
      jf(s.baroAltM), jf(s.baroPa), jf(s.baroTempC), s.baroValid ? "true" : "false",
      (long)s.gpsLatE7, (long)s.gpsLonE7, (int)s.gpsAltDm, jf(s.gpsSpeedMs),
      jf(s.gpsCourseDeg), jf(s.gpsHdop), s.gpsSats, s.gpsFixQuality,
      s.gpsFix ? "true" : "false",
      s.tofMm, s.tofValid ? "true" : "false", s.loopHz);
  return sendJson(req, body);
}

esp_err_t handleGetHealth(httpd_req_t* req) {
  if (!gCb.getHealth) return sendError(req, 500, "no_callback");
  HealthSnapshot h;
  gCb.getHealth(h);
  char body[256];
  int n = snprintf(body, sizeof(body),
      "{\"uptimeMs\":%lu,\"freeHeap\":%lu,\"minFreeHeap\":%lu,"
      "\"overrunsFlight\":%lu,\"overrunsRadio\":%lu,\"overrunsSensor\":%lu,"
      "\"flightMaxUs\":%lu,\"radioMaxUs\":%lu}",
      (unsigned long)h.uptimeMs, (unsigned long)h.freeHeapBytes, (unsigned long)h.minFreeHeapBytes,
      (unsigned long)h.flightOverruns, (unsigned long)h.radioOverruns, (unsigned long)h.sensorOverruns,
      (unsigned long)h.flightMaxUs, (unsigned long)h.radioMaxUs);
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}

// One-shot dashboard frame (initial load / WS-less fallback). Uses its own
// static buffer so it can't race the telemetry task's buffer.
esp_err_t handleGetDash(httpd_req_t* req) {
  if (!gCb.getDashTelemetry) return sendError(req, 500, "no_callback");
  static char dashBuf[4400];
  DashTelemetry t;
  gCb.getDashTelemetry(t);
  const int n = formatDashJson(dashBuf, sizeof(dashBuf), t,
                               gWsClients.load(), gWebTelemHz.load());
  if (n <= 0) return sendError(req, 500, "fmt");
  return sendJson(req, dashBuf);
}

esp_err_t handleGetCal(httpd_req_t* req) {
  if (!gCb.getCalInfo) return sendError(req, 500, "no_callback");
  CalInfo c;
  gCb.getCalInfo(c);
  char body[480];
  int n = snprintf(body, sizeof(body),
      "{\"off\":[%.3f,%.3f],\"trim\":[%.3f,%.3f],\"loaded\":%s,\"calState\":%u,\"calErr\":%u,"
      "\"std\":[%.3f,%.3f],\"samples\":%u,\"accelOff\":[%.4f,%.4f,%.4f],\"accelValid\":%s,"
      "\"gyroBias\":[%.3f,%.3f,%.3f],\"gyroBiasValid\":%s,\"magCal\":%s,\"safe\":%s,"
      "\"maxOff\":%.1f,\"maxTrim\":%.1f,\"step\":%.2f}",
      (double)c.levelOffsetDeg[0], (double)c.levelOffsetDeg[1], (double)c.trimDeg[0],
      (double)c.trimDeg[1], c.levelLoaded ? "true" : "false", c.levelCalState, c.levelCalErr,
      (double)c.levelCalStdDeg[0], (double)c.levelCalStdDeg[1], c.levelCalSamples,
      (double)c.accelOffG[0], (double)c.accelOffG[1], (double)c.accelOffG[2],
      c.accelValid ? "true" : "false", (double)c.gyroBiasDps[0], (double)c.gyroBiasDps[1],
      (double)c.gyroBiasDps[2], c.gyroBiasValid ? "true" : "false", c.magCalValid ? "true" : "false",
      c.safe ? "true" : "false", (double)c.maxOffsetDeg, (double)c.maxTrimDeg, (double)c.trimStepDeg);
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}

// Generic auth+safe-gated no-body action.
esp_err_t doBoolAction(httpd_req_t* req, bool (*cb)(), const char* okBody) {
  if (cb == nullptr) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!cb()) return sendError(req, 409, "refused");
  return sendJson(req, okBody ? okBody : "{\"ok\":true}");
}

esp_err_t handleLevelCalibrate(httpd_req_t* req) {
  return doBoolAction(req, gCb.calibrateLevel, "{\"ok\":true,\"requested\":true}");
}
esp_err_t handleLevelSave(httpd_req_t* req) {
  return doBoolAction(req, gCb.saveLevelToNvs, "{\"ok\":true,\"persisted\":true}");
}
esp_err_t handleLevelReload(httpd_req_t* req) { return doBoolAction(req, gCb.reloadLevelFromNvs, nullptr); }
esp_err_t handleLevelClear(httpd_req_t* req) { return doBoolAction(req, gCb.clearLevel, nullptr); }
esp_err_t handleTrimReset(httpd_req_t* req) { return doBoolAction(req, gCb.resetTrim, nullptr); }
esp_err_t handleLevelRestore(httpd_req_t* req) { return doBoolAction(req, gCb.restoreLevelPrev, nullptr); }
esp_err_t handleAccelSave(httpd_req_t* req) {
  return doBoolAction(req, gCb.saveAccelOffset, "{\"ok\":true,\"persisted\":true}");
}
esp_err_t handleAccelClear(httpd_req_t* req) {
  return doBoolAction(req, gCb.clearAccelOffset, "{\"ok\":true,\"cleared\":true}");
}
esp_err_t handleMagStart(httpd_req_t* req) {
  return doBoolAction(req, gCb.startMagCalibration, "{\"ok\":true,\"active\":true}");
}
esp_err_t handleMagFinish(httpd_req_t* req) {
  return doBoolAction(req, gCb.finishMagCalibration, "{\"ok\":true,\"persisted\":true}");
}

// ---- Settings (magnetometer heading trim) ---------------------------------
// GET  /api/settings        -> {"magTrimDeg":<deg>,"safe":<bool>}
// POST /api/settings  body {"magTrimDeg":<deg>}  applies AND persists (the web
// page's "Save" button), so the trim survives a reboot.
esp_err_t handleGetSettings(httpd_req_t* req) {
  float trim = 0.0f;
  if (gCb.getMagTrimDeg) gCb.getMagTrimDeg(trim);
  char body[80];
  int n = snprintf(body, sizeof(body), "{\"magTrimDeg\":%.1f,\"safe\":%s}",
                   static_cast<double>(trim), gSafeToWrite.load() ? "true" : "false");
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}

esp_err_t handlePostSettings(httpd_req_t* req) {
  if (!gCb.setMagTrimDeg) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[96];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float trim = 0.0f;
  if (!parseJsonFloat(body, "magTrimDeg", trim)) return sendError(req, 400, "bad_json");
  if (!(trim >= -360.0f && trim <= 360.0f)) return sendError(req, 400, "out_of_range");
  if (!gCb.setMagTrimDeg(trim)) return sendError(req, 409, "apply_refused");
  if (gCb.saveMagTrimDegToNvs) gCb.saveMagTrimDegToNvs();  // POST == Save (persist)
  return sendJson(req, "{\"ok\":true,\"persisted\":true}");
}

// ---- External-mag yaw correction ------------------------------------------
// Source selection is fixed external-only by the FCU. PID-web exposes only the
// slow correction gain: 0 = shadow, 1 = full configured pull (still tau-limited).
esp_err_t handleGetMagConfig(httpd_req_t* req) {
  if (!gCb.getMagConfig) return sendError(req, 500, "no_callback");
  MagConfigSnapshot c;
  gCb.getMagConfig(c);
  char body[180];
  int n = snprintf(body, sizeof(body),
      "{\"extCompiled\":%s,\"extEnabled\":%s,\"onboardEnabled\":%s,"
      "\"preferExternal\":%s,\"gain\":%.3f,\"safe\":%s}",
      c.extCompiled ? "true" : "false", c.extEnabled ? "true" : "false",
      c.onboardEnabled ? "true" : "false", c.preferExternal ? "true" : "false",
      static_cast<double>(c.yawCorrGain), gSafeToWrite.load() ? "true" : "false");
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}

esp_err_t handlePutMagConfig(httpd_req_t* req) {
  if (!gCb.setMagConfig) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[80];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float gain = 0.0f;
  if (!parseJsonFloat(body, "gain", gain)) return sendError(req, 400, "bad_json");
  if (!(gain >= 0.0f && gain <= 1.0f)) return sendError(req, 400, "out_of_range");
  if (!gCb.setMagConfig(true, false, true, gain)) return sendError(req, 409, "apply_refused");
  return sendJson(req, "{\"ok\":true,\"source\":\"external\"}");
}

esp_err_t handleSaveMagConfig(httpd_req_t* req) {
  return doBoolAction(req, gCb.saveMagConfigToNvs, "{\"ok\":true,\"persisted\":true}");
}

// POST /api/calibrate?mag=1 -> {"status":"started"} (alias for the existing
// mag min/max capture; same callback as /api/mag/start).
esp_err_t handleCalibrate(httpd_req_t* req) {
  if (!gCb.startMagCalibration) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char query[32] = {};
  char magText[8] = {};
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "mag", magText, sizeof(magText)) != ESP_OK ||
      strtol(magText, nullptr, 10) != 1) {
    return sendError(req, 400, "missing_mag");
  }
  if (!gCb.startMagCalibration()) return sendError(req, 409, "calibration_refused");
  return sendJson(req, "{\"status\":\"started\"}");
}

esp_err_t handleLevelOffset(httpd_req_t* req) {
  if (!gCb.applyLevelOffset) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[96];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float r = 0.0f, p = 0.0f;
  if (!parseRollPitch(body, r, p)) return sendError(req, 400, "bad_json");
  if (!gCb.applyLevelOffset(r, p)) return sendError(req, 409, "refused");
  return sendJson(req, "{\"ok\":true}");
}

esp_err_t handleTrim(httpd_req_t* req) {
  if (!gCb.applyTrim) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[96];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float r = 0.0f, p = 0.0f;
  if (!parseRollPitch(body, r, p)) return sendError(req, 400, "bad_json");
  if (!gCb.applyTrim(r, p)) return sendError(req, 409, "refused");
  return sendJson(req, "{\"ok\":true}");
}

// ---- vibration / FFT / notch ----
esp_err_t handleGetNotch(httpd_req_t* req) {
  if (!gCb.getNotchInfo) return sendError(req, 500, "no_callback");
  NotchInfo n;
  gCb.getNotchInfo(n);
  char body[440];
  int len = snprintf(body, sizeof(body),
      "{\"comp\":%s,\"en\":%s,\"center\":%.1f,\"min\":%.1f,\"max\":%.1f,\"q\":%.2f,"
      "\"running\":%s,\"done\":%s,\"effHz\":%.1f,\"n\":%u,\"cap\":128,"
      "\"peakHz\":[%.1f,%.1f,%.1f],\"peakMag\":[%.3f,%.3f,%.3f],\"floor\":[%.3f,%.3f,%.3f],"
      "\"recCenter\":%.1f,\"recQ\":%.2f,\"conf\":%.2f}",
      n.dynamicCompiled ? "true" : "false", n.enabled ? "true" : "false", (double)n.centerHz,
      (double)n.minHz, (double)n.maxHz, (double)n.q, n.analysisRunning ? "true" : "false",
      n.analysisDone ? "true" : "false", (double)n.effectiveSampleHz, n.sampleCount,
      (double)n.peakHz[0], (double)n.peakHz[1], (double)n.peakHz[2],
      (double)n.peakMag[0], (double)n.peakMag[1], (double)n.peakMag[2],
      (double)n.noiseFloor[0], (double)n.noiseFloor[1], (double)n.noiseFloor[2],
      (double)n.recommendCenterHz, (double)n.recommendQ, (double)n.confidence);
  if (len < 0 || len >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}
esp_err_t handleGetFft(httpd_req_t* req) {
  if (!gCb.getFftSpectrum) return sendError(req, 500, "no_callback");
  char q[48] = {0}, av[8] = {0}, sv[8] = {0};
  uint8_t axis = 0, stage = 0;
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
    if (httpd_query_key_value(q, "axis", av, sizeof(av)) == ESP_OK) axis = (uint8_t)atoi(av);
    if (httpd_query_key_value(q, "stage", sv, sizeof(sv)) == ESP_OK) stage = (uint8_t)atoi(sv);
  }
  static float mags[128];
  float hpb = 0.0f;
  const uint16_t nb = gCb.getFftSpectrum(axis, stage, mags, 128, hpb);
  static char buf[2048];
  JsonAppender j{buf, sizeof(buf), 0};
  j.f("{\"hzPerBin\":%.4f,\"bins\":%u,\"mag\":[", (double)hpb, nb);
  for (uint16_t k = 0; k < nb; ++k) j.f("%s%.4f", k ? "," : "", (double)mags[k]);
  j.f("]}");
  return sendJson(req, buf);
}
esp_err_t handleNotchStart(httpd_req_t* req) { return doBoolAction(req, gCb.startNotchAnalysis, "{\"ok\":true,\"running\":true}"); }
esp_err_t handleNotchStop(httpd_req_t* req) { return doBoolAction(req, gCb.stopNotchAnalysis, nullptr); }
esp_err_t handleNotchSave(httpd_req_t* req) { return doBoolAction(req, gCb.saveNotchToNvs, "{\"ok\":true,\"persisted\":true}"); }
esp_err_t handleNotchReload(httpd_req_t* req) { return doBoolAction(req, gCb.reloadNotchFromNvs, nullptr); }
esp_err_t handleNotchApply(httpd_req_t* req) {
  if (!gCb.applyNotchTemp) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[96];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float c = 0.0f, qq = 0.0f;
  if (!parseJsonFloat(body, "center", c) || !parseJsonFloat(body, "q", qq)) return sendError(req, 400, "bad_json");
  if (!gCb.applyNotchTemp(c, qq)) return sendError(req, 409, "refused");
  return sendJson(req, "{\"ok\":true}");
}
esp_err_t handleNotchEnabled(httpd_req_t* req) {
  if (!gCb.setNotchEnabled) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[64];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float en = 0.0f;
  if (!parseJsonFloat(body, "en", en)) return sendError(req, 400, "bad_json");
  if (!gCb.setNotchEnabled(en > 0.5f)) return sendError(req, 409, "refused");
  return sendJson(req, "{\"ok\":true}");
}

// ---- pan/tilt servos ----
esp_err_t handleGetServo(httpd_req_t* req) {
  if (!gCb.getServo) return sendError(req, 500, "no_callback");
  ServoState s;
  gCb.getServo(s);
  char body[320];
  int n = snprintf(body, sizeof(body),
      "{\"att\":%s,\"pan\":%u,\"tilt\":%u,\"panT\":%u,\"tiltT\":%u,\"panMin\":%u,\"panC\":%u,"
      "\"panMax\":%u,\"tiltMin\":%u,\"tiltC\":%u,\"tiltMax\":%u,\"panInv\":%s,\"tiltInv\":%s,\"ov\":%s}",
      s.attached ? "true" : "false", s.panUs, s.tiltUs, s.panTargetUs, s.tiltTargetUs,
      s.panMinUs, s.panCenterUs, s.panMaxUs, s.tiltMinUs, s.tiltCenterUs, s.tiltMaxUs,
      s.panInverted ? "true" : "false", s.tiltInverted ? "true" : "false",
      s.webOverrideActive ? "true" : "false");
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}
esp_err_t handleServoSet(httpd_req_t* req) {
  if (!gCb.setServoMicros) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[96];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float p = 0, t = 0;
  if (!parseJsonFloat(body, "pan", p) || !parseJsonFloat(body, "tilt", t)) return sendError(req, 400, "bad_json");
  if (!gCb.setServoMicros((uint16_t)p, (uint16_t)t)) return sendError(req, 409, "refused");
  return sendJson(req, "{\"ok\":true}");
}
esp_err_t handleServoNudge(httpd_req_t* req) {
  if (!gCb.nudgeServo) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[96];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float dp = 0, dt = 0;
  parseJsonFloat(body, "dpan", dp);
  parseJsonFloat(body, "dtilt", dt);
  if (!gCb.nudgeServo((int16_t)dp, (int16_t)dt)) return sendError(req, 409, "refused");
  return sendJson(req, "{\"ok\":true}");
}
esp_err_t handleServoCenter(httpd_req_t* req) {
  if (!gCb.centerServo) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[64] = {0};
  recvBody(req, body, sizeof(body));
  float p = 1, t = 1;
  parseJsonFloat(body, "pan", p);
  parseJsonFloat(body, "tilt", t);
  if (!gCb.centerServo(p > 0.5f, t > 0.5f)) return sendError(req, 409, "refused");
  return sendJson(req, "{\"ok\":true}");
}
esp_err_t handleServoStop(httpd_req_t* req) { return doBoolAction(req, gCb.stopServo, nullptr); }
esp_err_t handleServoRelease(httpd_req_t* req) {
  if (!gCb.releaseServoOverride) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gCb.releaseServoOverride()) return sendError(req, 409, "refused");
  return sendJson(req, "{\"ok\":true}");
}
esp_err_t handleServoConfig(httpd_req_t* req) {
  if (!gCb.saveServoConfig) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[224];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float pmn = 0, pc = 0, pmx = 0, tmn = 0, tc = 0, tmx = 0, pi = 0, ti = 0;
  if (!parseJsonFloat(body, "panMin", pmn) || !parseJsonFloat(body, "panCenter", pc) ||
      !parseJsonFloat(body, "panMax", pmx) || !parseJsonFloat(body, "tiltMin", tmn) ||
      !parseJsonFloat(body, "tiltCenter", tc) || !parseJsonFloat(body, "tiltMax", tmx)) {
    return sendError(req, 400, "bad_json");
  }
  parseJsonFloat(body, "panInv", pi);
  parseJsonFloat(body, "tiltInv", ti);
  if (!gCb.saveServoConfig((uint16_t)pmn, (uint16_t)pc, (uint16_t)pmx, (uint16_t)tmn, (uint16_t)tc,
                           (uint16_t)tmx, pi > 0.5f, ti > 0.5f)) {
    return sendError(req, 409, "refused");
  }
  return sendJson(req, "{\"ok\":true,\"persisted\":true}");
}

// ---- diagnostic capture ----
esp_err_t handleGetCapture(httpd_req_t* req) {
  if (!gCb.getCaptureStatus) return sendError(req, 500, "no_callback");
  CaptureStatus c;
  gCb.getCaptureStatus(c);
  char body[256];
  const int n = snprintf(body, sizeof(body),
      "{\"active\":%s,\"waiting\":%s,\"hasData\":%s,\"overflow\":%s,"
      "\"samples\":%u,\"cap\":%u,\"effHz\":%.1f,\"dropped\":%lu}",
      c.active ? "true" : "false", c.waitingForArm ? "true" : "false",
      c.hasData ? "true" : "false", c.overflow ? "true" : "false",
      c.samples, c.capacity, (double)c.effectiveHz, (unsigned long)c.droppedSamples);
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}

esp_err_t handleCaptureStart(httpd_req_t* req) {
  return doBoolAction(req, gCb.startCapture, "{\"ok\":true,\"active\":true}");
}

esp_err_t handleCaptureArm(httpd_req_t* req) {
  return doBoolAction(req, gCb.armTriggeredCapture, "{\"ok\":true,\"waiting\":true}");
}

esp_err_t handleCaptureStop(httpd_req_t* req) {
  if (!gCb.stopCapture) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gCb.stopCapture()) return sendError(req, 409, "refused");
  return sendJson(req, "{\"ok\":true,\"active\":false}");
}

esp_err_t handleCaptureClear(httpd_req_t* req) {
  return doBoolAction(req, gCb.clearCapture, "{\"ok\":true,\"cleared\":true}");
}

esp_err_t handleCaptureCsv(httpd_req_t* req) {
  if (!gCb.captureCsvChunk) return sendError(req, 500, "no_callback");
  httpd_resp_set_type(req, "text/csv; charset=utf-8");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"fcu_diag_capture.csv\"");
  static char chunk[2048];
  uint32_t cursor = 0;
  for (uint16_t guard = 0; guard < 512; ++guard) {
    uint32_t next = 0;
    const uint32_t n = gCb.captureCsvChunk(cursor, chunk, sizeof(chunk), next);
    if (n > 0U) {
      const esp_err_t err = httpd_resp_send_chunk(req, chunk, n);
      if (err != ESP_OK) return err;
    }
    if (next == 0U) break;
    if (n == 0U && next == cursor) break;
    cursor = next;
  }
  return httpd_resp_send_chunk(req, nullptr, 0);
}

// ============================================================================
// Motors page endpoints — deadman motor test + order/direction save.
// ============================================================================

// GET /api/motors — session + config snapshot (no auth; read-only).
esp_err_t handleGetMotors(httpd_req_t* req) {
  if (!gCb.getMotorTest) return sendError(req, 500, "no_callback");
  MotorTestState s;
  gCb.getMotorTest(s);
  char body[420];
  const int n = snprintf(body, sizeof(body),
      "{\"active\":%u,\"raw\":%u,\"deadmanMs\":%lu,\"testValue\":%u,\"holdMaxMs\":%u,"
      "\"idleEnable\":%s,\"idleValue\":%u,\"map\":[%u,%u,%u,%u],\"dir\":[%u,%u,%u,%u],"
      "\"safe\":%s}",
      s.activeMotor, s.raw, (unsigned long)s.deadmanMsLeft, s.testValue, s.holdMaxMs,
      s.idleEnable ? "true" : "false", s.idleValue,
      s.map[0], s.map[1], s.map[2], s.map[3],
      s.dir[0], s.dir[1], s.dir[2], s.dir[3],
      s.safe ? "true" : "false");
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}

// POST /api/motors/test {"motor":1..4,"value":raw?,"ms":hold?,"ack":"props-removed"}
// PROPS-OFF ONLY. The ack string is required on every request — the UI wires
// it to an explicit "propellers removed" checkbox. Deadman: the browser must
// re-POST before `ms` elapses or the firmware stops the motor.
esp_err_t handleMotorsTest(httpd_req_t* req) {
  if (!gCb.motorTestRun) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[192];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  if (strstr(body, "\"ack\"") == nullptr || strstr(body, "props-removed") == nullptr) {
    return sendError(req, 400, "props_ack_required");
  }
  float motor = 0.0f, value = 0.0f, ms = 0.0f;
  if (!parseJsonFloat(body, "motor", motor)) return sendError(req, 400, "bad_json");
  parseJsonFloat(body, "value", value);  // optional; 0 = configured default
  parseJsonFloat(body, "ms", ms);        // optional; 0 = 400 ms
  if (motor < 1.0f || motor > 4.0f) return sendError(req, 400, "bad_motor");
  if (!gCb.motorTestRun(static_cast<uint8_t>(motor), static_cast<uint16_t>(value),
                        static_cast<uint16_t>(ms))) {
    return sendError(req, 409, "motor_test_refused");
  }
  return sendJson(req, "{\"ok\":true}");
}

// POST /api/motors/stop — auth only; never gated on bench state.
esp_err_t handleMotorsStop(httpd_req_t* req) {
  if (!gCb.motorTestStop) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  (void)gCb.motorTestStop();
  return sendJson(req, "{\"ok\":true,\"stopped\":true}");
}

// POST /api/motors/save-order {"map":[p1,p2,p3,p4]} — logical slot i (FR/RR/
// FL/RL) drives physical output p_i. Must be a permutation of 1..4.
esp_err_t handleMotorsSaveOrder(httpd_req_t* req) {
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[128];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float m[4] = {};
  if (!parseJsonFloat4(body, "map", m)) return sendError(req, 400, "bad_json");
  bool seen[4] = {false, false, false, false};
  for (float v : m) {
    const int iv = (int)v;
    if (v != (float)iv || iv < 1 || iv > 4 || seen[iv - 1]) {
      return sendError(req, 400, "map_not_permutation");
    }
    seen[iv - 1] = true;
  }
  static const char* kNames[4] = {"motor_map_1", "motor_map_2", "motor_map_3", "motor_map_4"};
  for (int i = 0; i < 4; ++i) {
    if (!fcu_config::setAndSave(kNames[i], m[i])) {
      return sendError(req, 500, "save_failed");
    }
  }
  return sendJson(req, "{\"ok\":true,\"persisted\":true}");
}

// POST /api/motors/save-direction {"dir":[0/1 x4]} — CW/CCW metadata per
// logical slot (documentation + future mixer-sign validation; DShot cannot
// reverse a motor without ESC config).
esp_err_t handleMotorsSaveDirection(httpd_req_t* req) {
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[128];
  if (recvBody(req, body, sizeof(body)) < 0) return sendError(req, 400, "bad_body");
  float d[4] = {};
  if (!parseJsonFloat4(body, "dir", d)) return sendError(req, 400, "bad_json");
  for (float v : d) {
    if (v != 0.0f && v != 1.0f) return sendError(req, 400, "bad_direction");
  }
  static const char* kNames[4] = {"motor_dir_1", "motor_dir_2", "motor_dir_3", "motor_dir_4"};
  for (int i = 0; i < 4; ++i) {
    if (!fcu_config::setAndSave(kNames[i], d[i])) {
      return sendError(req, 500, "save_failed");
    }
  }
  // Mixer-sign sanity: with the standard Quad-X geometry, diagonal motors
  // spin the same direction and adjacent ones opposite. Warn (not fail) so
  // unconventional props-out setups stay possible.
  const bool conventional = (d[0] == d[3]) && (d[1] == d[2]) && (d[0] != d[1]);
  char resp[96];
  snprintf(resp, sizeof(resp), "{\"ok\":true,\"persisted\":true,\"conventional\":%s}",
           conventional ? "true" : "false");
  return sendJson(req, resp);
}

// ============================================================================
// Config registry endpoints (fcu_config). The registry is a firmware module,
// not a callback — it owns its own locking and validation, and every mutation
// is additionally gated here on auth + bench-idle exactly like the callbacks.
// ============================================================================

// Stream a chunked-JSON generator (export/meta) as one HTTP response.
esp_err_t sendChunkedJson(httpd_req_t* req,
                          uint32_t (*gen)(uint32_t, char*, uint32_t, uint32_t&),
                          const char* attachmentName) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  if (attachmentName != nullptr) {
    static char disp[96];
    snprintf(disp, sizeof(disp), "attachment; filename=\"%s\"", attachmentName);
    httpd_resp_set_hdr(req, "Content-Disposition", disp);
  }
  static char chunk[2048];
  uint32_t cursor = 0;
  for (uint16_t guard = 0; guard < 512; ++guard) {
    uint32_t next = 0;
    const uint32_t n = gen(cursor, chunk, sizeof(chunk), next);
    if (n > 0U) {
      const esp_err_t err = httpd_resp_send_chunk(req, chunk, n);
      if (err != ESP_OK) return err;
    }
    if (next == 0U) break;
    if (n == 0U && next == cursor) break;  // stuck — bail rather than spin
    cursor = next;
  }
  return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t handleGetConfig(httpd_req_t* req) {
  return sendChunkedJson(req, fcu_config::metaJsonChunk, nullptr);
}

esp_err_t handleGetConfigExport(httpd_req_t* req) {
  return sendChunkedJson(req, fcu_config::exportJsonChunk, "fcu_config_backup.json");
}

esp_err_t sendImportResult(httpd_req_t* req, const fcu_config::ImportResult& r) {
  char body[224];
  const int n = snprintf(body, sizeof(body),
      "{\"ok\":%s,\"applied\":%u,\"unknown\":%u,\"rejected\":%u,"
      "\"rolledBack\":%s,\"fileVersion\":%u,\"error\":\"%s\"}",
      r.ok ? "true" : "false", r.applied, r.unknown, r.rejected,
      r.rolledBack ? "true" : "false", r.fileVersion, r.firstError);
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  if (!r.ok) {
    httpd_resp_set_status(req, "400");
  }
  return sendJson(req, body);
}

// POST /api/config — partial apply+persist of a flat {"name":value,...} object.
esp_err_t handlePostConfig(httpd_req_t* req) {
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  static char body[4096];  // static: httpd task stack is only ~6 kB
  const int n = recvBody(req, body, sizeof(body));
  if (n < 0) return sendError(req, 400, "bad_body");
  const auto res = fcu_config::applyJsonObject(body, static_cast<size_t>(n));
  return sendImportResult(req, res);
}

// POST /api/config/import — full backup restore with validate-then-apply and
// rollback-on-failure semantics (see fcu_config::importJson).
esp_err_t handlePostConfigImport(httpd_req_t* req) {
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  static char body[12288];
  const int n = recvBody(req, body, sizeof(body));
  if (n < 0) return sendError(req, 400, "bad_body");
  const auto res = fcu_config::importJson(body, static_cast<size_t>(n));
  return sendImportResult(req, res);
}

esp_err_t handlePostConfigReset(httpd_req_t* req) {
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!fcu_config::resetStoredToDefaults()) return sendError(req, 500, "nvs_write_failed");
  return sendJson(req, "{\"ok\":true,\"defaults\":true}");
}

esp_err_t handleReboot(httpd_req_t* req) {
  if (!gCb.requestReboot) return sendError(req, 500, "no_callback");
  if (!authorized(req)) return sendError(req, 401, "unauthorized");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!gCb.requestReboot()) return sendError(req, 409, "refused");
  return sendJson(req, "{\"ok\":true,\"rebooting\":true}");
}

void registerUri(httpd_handle_t srv, const char* path, httpd_method_t method,
                 esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t u = {};
  u.uri = path;
  u.method = method;
  u.handler = handler;
  u.user_ctx = nullptr;
  httpd_register_uri_handler(srv, &u);
}

void stopHttpServer(const char* reason) {
  gRunning.store(false, std::memory_order_relaxed);
  gWsClients.store(0, std::memory_order_relaxed);
  if (gServer != nullptr) {
    httpd_stop(gServer);
    gServer = nullptr;
    Serial.printf("[%s] http server stopped (%s)\n", kTag, reason ? reason : "unknown");
  }
}

// Bring up the FCU's own access point (SoftAP) at the fixed FCU_PID_AP_IP.
// Returns true once the AP is broadcasting. WPA2 requires an 8..63 char
// passphrase; a shorter/empty pass falls back to an OPEN network so the AP
// still comes up rather than silently failing.
bool startSoftAp(const char* ssid, const char* pass) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);  // disable sleep so HTTP latency is low

  const IPAddress apIp(FCU_PID_AP_IP);
  const IPAddress gateway(FCU_PID_AP_IP);
  const IPAddress subnet(255, 255, 255, 0);
  if (!WiFi.softAPConfig(apIp, gateway, subnet)) {
    Serial.printf("[%s] softAPConfig(%s) failed\n", kTag, apIp.toString().c_str());
    return false;
  }

  const bool secured = (pass != nullptr) && (strlen(pass) >= 8);
  if (!secured && pass != nullptr && pass[0] != '\0') {
    Serial.printf("[%s] WARNING: AP password is <8 chars; WPA2 needs 8+. "
                  "Starting an OPEN network instead.\n", kTag);
  }
  const bool ok = secured ? WiFi.softAP(ssid, pass) : WiFi.softAP(ssid);
  if (!ok) {
    Serial.printf("[%s] softAP('%s') failed to start\n", kTag, ssid);
    return false;
  }

  Serial.printf("[%s] SoftAP '%s' up (%s): http://%s/\n", kTag, ssid,
                secured ? "WPA2" : "OPEN", WiFi.softAPIP().toString().c_str());
  return true;
}

bool startHttpServer() {
  if (gRunning.load(std::memory_order_relaxed)) {
    return true;
  }
  if (WiFi.softAPIP() == IPAddress(static_cast<uint32_t>(0))) {
    Serial.printf("[%s] http start refused: SoftAP not up\n", kTag);
    return false;
  }

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.task_priority = 3;        // well below flight/radio/sensor
  cfg.core_id = 0;              // keep off core 1 (flight + radio)
  cfg.stack_size = FCU_PID_HTTPD_STACK_BYTES;
  cfg.max_uri_handlers = 96;    // tuner + dashboard + level/servo/notch/capture + config
  cfg.lru_purge_enable = true;

  esp_err_t err = httpd_start(&gServer, &cfg);
  if (err != ESP_OK) {
    gServer = nullptr;
    Serial.printf("[%s] httpd_start failed: %s\n", kTag, esp_err_to_name(err));
    return false;
  }

  registerUri(gServer, "/",                HTTP_GET,  handleIndex);
  registerUri(gServer, "/api/pid",         HTTP_GET,  handleGetPid);
  registerUri(gServer, "/api/pid",         HTTP_PUT,  handlePutPid);
  registerUri(gServer, "/api/pid/save",    HTTP_POST, handleSavePid);
  registerUri(gServer, "/api/pid/revert",  HTTP_POST, handleRevertPid);
  registerUri(gServer, "/api/pid/reset",   HTTP_POST, handleResetPid);
  registerUri(gServer, "/api/imu/calibrate", HTTP_POST, handleCalibrateImu);
  registerUri(gServer, "/api/motor/spin",  HTTP_POST, handleSpinMotor);
  registerUri(gServer, "/api/mix",         HTTP_GET,  handleGetMix);
  registerUri(gServer, "/api/mix",         HTTP_PUT,  handlePutMix);
  registerUri(gServer, "/api/mix/save",    HTTP_POST, handleSaveMix);
  registerUri(gServer, "/api/failsafe",    HTTP_PUT,  handlePutFailsafe);
  registerUri(gServer, "/api/failsafe/save", HTTP_POST, handleSaveFailsafe);
  registerUri(gServer, "/api/state",       HTTP_GET,  handleGetState);
  registerUri(gServer, "/api/health",      HTTP_GET,  handleGetHealth);
  registerUri(gServer, "/api/status",      HTTP_GET,  handleGetStatus);
  registerUri(gServer, "/api/sensors/live", HTTP_GET, handleGetSensorsLive);
  registerUri(gServer, "/api/tune",        HTTP_GET,  handleGetTune);

  // ---- Dashboard: legacy fallback, live telemetry, calibration/level/trim ---
  registerUri(gServer, "/legacy",              HTTP_GET,  handleLegacyIndex);
  registerUri(gServer, "/api/dash",            HTTP_GET,  handleGetDash);
  registerUri(gServer, "/api/cal",             HTTP_GET,  handleGetCal);
  registerUri(gServer, "/api/level/calibrate", HTTP_POST, handleLevelCalibrate);
  registerUri(gServer, "/api/level/offset",    HTTP_PUT,  handleLevelOffset);
  registerUri(gServer, "/api/trim",            HTTP_PUT,  handleTrim);
  registerUri(gServer, "/api/level/save",      HTTP_POST, handleLevelSave);
  registerUri(gServer, "/api/level/reload",    HTTP_POST, handleLevelReload);
  registerUri(gServer, "/api/level/clear",     HTTP_POST, handleLevelClear);
  registerUri(gServer, "/api/trim/reset",      HTTP_POST, handleTrimReset);
  registerUri(gServer, "/api/level/restore",   HTTP_POST, handleLevelRestore);
  registerUri(gServer, "/api/accel/save",      HTTP_POST, handleAccelSave);
  registerUri(gServer, "/api/accel/clear",     HTTP_POST, handleAccelClear);
  registerUri(gServer, "/api/mag/start",       HTTP_POST, handleMagStart);
  registerUri(gServer, "/api/mag/finish",      HTTP_POST, handleMagFinish);
  registerUri(gServer, "/api/settings",        HTTP_GET,  handleGetSettings);
  registerUri(gServer, "/api/settings",        HTTP_POST, handlePostSettings);
  registerUri(gServer, "/api/mag/config",      HTTP_GET,  handleGetMagConfig);
  registerUri(gServer, "/api/mag/config",      HTTP_PUT,  handlePutMagConfig);
  registerUri(gServer, "/api/mag/config/save", HTTP_POST, handleSaveMagConfig);
  registerUri(gServer, "/api/calibrate",       HTTP_POST, handleCalibrate);
  // ---- Vibration / FFT / notch ----
  registerUri(gServer, "/api/notch",           HTTP_GET,  handleGetNotch);
  registerUri(gServer, "/api/notch/fft",       HTTP_GET,  handleGetFft);
  registerUri(gServer, "/api/notch/start",     HTTP_POST, handleNotchStart);
  registerUri(gServer, "/api/notch/stop",      HTTP_POST, handleNotchStop);
  registerUri(gServer, "/api/notch/apply",     HTTP_PUT,  handleNotchApply);
  registerUri(gServer, "/api/notch/enabled",   HTTP_PUT,  handleNotchEnabled);
  registerUri(gServer, "/api/notch/save",      HTTP_POST, handleNotchSave);
  registerUri(gServer, "/api/notch/reload",    HTTP_POST, handleNotchReload);
  // ---- Pan/tilt servos ----
  registerUri(gServer, "/api/servo",           HTTP_GET,  handleGetServo);
  registerUri(gServer, "/api/servo",           HTTP_PUT,  handleServoSet);
  registerUri(gServer, "/api/servo/nudge",     HTTP_POST, handleServoNudge);
  registerUri(gServer, "/api/servo/center",    HTTP_POST, handleServoCenter);
  registerUri(gServer, "/api/servo/stop",      HTTP_POST, handleServoStop);
  registerUri(gServer, "/api/servo/release",   HTTP_POST, handleServoRelease);
  registerUri(gServer, "/api/servo/config",    HTTP_PUT,  handleServoConfig);
  // ---- Motors page (deadman test + order/direction) ----
  registerUri(gServer, "/api/motors",               HTTP_GET,  handleGetMotors);
  registerUri(gServer, "/api/motors/test",          HTTP_POST, handleMotorsTest);
  registerUri(gServer, "/api/motors/stop",          HTTP_POST, handleMotorsStop);
  registerUri(gServer, "/api/motors/save-order",    HTTP_POST, handleMotorsSaveOrder);
  registerUri(gServer, "/api/motors/save-direction", HTTP_POST, handleMotorsSaveDirection);
  // ---- Config registry (fcu_config) ----
  registerUri(gServer, "/api/config",           HTTP_GET,  handleGetConfig);
  registerUri(gServer, "/api/config",           HTTP_POST, handlePostConfig);
  registerUri(gServer, "/api/config/export",    HTTP_GET,  handleGetConfigExport);
  registerUri(gServer, "/api/config/import",    HTTP_POST, handlePostConfigImport);
  registerUri(gServer, "/api/config/reset",     HTTP_POST, handlePostConfigReset);
  registerUri(gServer, "/api/reboot",           HTTP_POST, handleReboot);
  // ---- Diagnostic capture ----
  registerUri(gServer, "/api/capture",          HTTP_GET,  handleGetCapture);
  registerUri(gServer, "/api/capture/start",    HTTP_POST, handleCaptureStart);
  registerUri(gServer, "/api/capture/arm",      HTTP_POST, handleCaptureArm);
  registerUri(gServer, "/api/capture/stop",     HTTP_POST, handleCaptureStop);
  registerUri(gServer, "/api/capture/clear",    HTTP_POST, handleCaptureClear);
  registerUri(gServer, "/api/capture.csv",      HTTP_GET,  handleCaptureCsv);

  // ---- WebSocket telemetry channel (~25 Hz push) ----
  {
    httpd_uri_t ws = {};
    ws.uri = "/ws";
    ws.method = HTTP_GET;
    ws.handler = handleWs;
    ws.is_websocket = true;
    httpd_register_uri_handler(gServer, &ws);
  }

  gRunning.store(true, std::memory_order_relaxed);
  if (gTelemTask == nullptr) {
    xTaskCreatePinnedToCore(telemetryTask, "pidweb_tlm", FCU_PID_TELEM_STACK_BYTES,
                            nullptr, 2, &gTelemTask, 0);
  }
  gLastIp = static_cast<uint32_t>(WiFi.localIP());
  Serial.printf("[%s] http server up on http://%s/\n",
                kTag, WiFi.localIP().toString().c_str());
  return true;
}

}  // namespace

void registerCallbacks(const Callbacks& cbs) {
  gCb = cbs;
}

void setAuthToken(const char* token) {
  if (token == nullptr) {
    gAuthToken[0] = '\0';
    return;
  }
  strncpy(gAuthToken, token, sizeof(gAuthToken) - 1);
  gAuthToken[sizeof(gAuthToken) - 1] = '\0';
}

void publishSafety(bool throttleZero, bool fsmArmed) {
  // Safe to write only if the FCU is bench-idle: throttle at zero AND not
  // armed by the flight FSM. Either condition flipping locks out writes.
  gSafeToWrite.store(throttleZero && !fsmArmed, std::memory_order_relaxed);
}

bool start(const char* ssid, const char* password, uint32_t connectTimeoutMs) {
  if (gRunning.load()) {
    return true;
  }
  gSsid = (ssid != nullptr) ? ssid : "";
  gPassword = (password != nullptr) ? password : "";
  (void)connectTimeoutMs;  // SoftAP comes up immediately; no connect timeout
  if (ssid == nullptr || ssid[0] == '\0') {
    Serial.printf("[%s] start refused: SSID empty\n", kTag);
    return false;
  }

  if (!startSoftAp(ssid, password)) {
    gNextReconnectMs = millis();
    return false;
  }

  return startHttpServer();
}

bool running() { return gRunning.load(); }

void service(uint32_t nowMs) {
  if (gSsid == nullptr || gSsid[0] == '\0') {
    return;
  }

  // SoftAP mode: the AP is hosted locally and does not "drop" like a station
  // link, so there is no reconnect loop. If the AP is somehow down (e.g. WiFi
  // was torn down elsewhere), bring it back up; otherwise just keep the HTTP
  // server running and emit a periodic health line with the client count.
  const uint32_t apIp = static_cast<uint32_t>(WiFi.softAPIP());
  if (apIp == 0U) {
    if (gRunning.load(std::memory_order_relaxed) || gServer != nullptr) {
      stopHttpServer("ap_down");
    }
    gLastIp = 0;
    if (static_cast<int32_t>(nowMs - gNextReconnectMs) < 0) {
      return;
    }
    gNextReconnectMs = nowMs + FCU_PID_WIFI_RECONNECT_BACKOFF_MS;
    Serial.printf("[%s] SoftAP down; restarting '%s'\n", kTag, gSsid);
    if (!startSoftAp(gSsid, gPassword)) {
      return;
    }
  }

  if (apIp != 0U && apIp != gLastIp) {
    gLastIp = apIp;
    Serial.printf("[%s] SoftAP active: http://%s/\n",
                  kTag, WiFi.softAPIP().toString().c_str());
  }
  if (!gRunning.load(std::memory_order_relaxed)) {
    (void)startHttpServer();
  }
  if ((nowMs - gLastHealthLogMs) >= FCU_PID_WIFI_HEALTH_LOG_MS) {
    gLastHealthLogMs = nowMs;
    Serial.printf("[%s] SoftAP health ip=%s clients=%d http=%u\n",
                  kTag, WiFi.softAPIP().toString().c_str(),
                  static_cast<int>(WiFi.softAPgetStationNum()),
                  static_cast<unsigned>(gRunning.load(std::memory_order_relaxed)));
  }
}

}  // namespace pid_webserver

#else  // !ENABLE_PID_WEBSERVER

// Stubs so callers can be unconditional. Linker drops them when unreferenced.
namespace pid_webserver {
void registerCallbacks(const Callbacks&) {}
void setAuthToken(const char*) {}
bool start(const char*, const char*, uint32_t) { return false; }
void service(uint32_t) {}
void publishSafety(bool, bool) {}
bool running() { return false; }
}  // namespace pid_webserver

#endif  // ENABLE_PID_WEBSERVER
