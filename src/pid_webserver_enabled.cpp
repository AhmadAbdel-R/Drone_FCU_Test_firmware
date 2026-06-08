#include "pid_webserver.h"

#if ENABLE_PID_WEBSERVER

#include <Arduino.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <esp_http_server.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace pid_webserver {

namespace {

constexpr const char* kTag = "PIDWEB";

Callbacks gCb;
httpd_handle_t gServer = nullptr;
std::atomic<bool> gSafeToWrite{false};
std::atomic<bool> gRunning{false};

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
 try{ const r=await fetch('/api/mix',{method:'PUT',headers:{'content-type':'application/json'},body:JSON.stringify({bias:v})});
  if(r.ok){poll();}else{let msg='Apply failed';try{const j=await r.json();if(j.error)msg+=': '+j.error;}catch(e){}alert(msg);}
 }catch(e){alert('Apply failed: '+e);}
}
async function saveMix(){
 await pushMix();  // ensure RAM matches UI before persisting
 await postAction('/api/mix/save','Mixer bias saved to NVS','Save failed');
}
let pushTimer=null;
function push(){ clearTimeout(pushTimer); pushTimer=setTimeout(async()=>{ try{await fetch('/api/pid',{method:'PUT',headers:{'content-type':'application/json'},body:JSON.stringify({gains})});}catch(e){} },120);}
async function save(){
 const r=await fetch('/api/pid/save',{method:'POST'});
 if(r.ok){ alert('Saved to NVS'); return; }
 let msg='Save failed';
 try{ const j=await r.json(); if(j.error) msg += ': '+j.error; }catch(e){}
 alert(msg);
}
async function postAction(url, okText, failText){
 const r=await fetch(url,{method:'POST'});
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
buildUI(); poll(); setInterval(poll,500); setInterval(pollTune,1000); pollTune();
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

// ---- handlers ----
esp_err_t handleIndex(httpd_req_t* req) {
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
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!gCb.saveAllToNvs()) return sendError(req, 500, "nvs_write_failed");
  return sendJson(req, "{\"ok\":true,\"persisted\":true}");
}

esp_err_t handleRevertPid(httpd_req_t* req) {
  if (!gCb.revertFromNvs) return sendError(req, 500, "no_callback");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!gCb.revertFromNvs()) return sendError(req, 500, "revert_failed");
  return sendJson(req, "{\"ok\":true}");
}

esp_err_t handleResetPid(httpd_req_t* req) {
  if (!gCb.resetToDefaults) return sendError(req, 500, "no_callback");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!gCb.resetToDefaults()) return sendError(req, 500, "reset_failed");
  return sendJson(req, "{\"ok\":true,\"defaults\":true}");
}

esp_err_t handleCalibrateImu(httpd_req_t* req) {
  if (!gCb.calibrateImu) return sendError(req, 500, "no_callback");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!gCb.calibrateImu()) return sendError(req, 409, "calibration_refused");
  return sendJson(req, "{\"ok\":true,\"requested\":true}");
}

esp_err_t handleSpinMotor(httpd_req_t* req) {
  if (!gCb.spinMotor) return sendError(req, 500, "no_callback");
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
  gCb.getMixPitchFrontBias(bias);
  char body[64];
  int n = snprintf(body, sizeof(body),
      "{\"bias\":%.3f,\"safe\":%s}",
      static_cast<double>(bias),
      gSafeToWrite.load() ? "true" : "false");
  if (n < 0 || n >= (int)sizeof(body)) return sendError(req, 500, "fmt");
  return sendJson(req, body);
}

esp_err_t handlePutMix(httpd_req_t* req) {
  if (!gCb.setMixPitchFrontBias) return sendError(req, 500, "no_callback");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  char body[96];
  int n = recvBody(req, body, sizeof(body));
  if (n < 0) return sendError(req, 400, "bad_body");
  float bias = 0.0f;
  if (!parseJsonFloat(body, "bias", bias)) return sendError(req, 400, "bad_json");
  if (!(bias >= 1.0f && bias <= 2.0f)) return sendError(req, 400, "out_of_range");
  if (!gCb.setMixPitchFrontBias(bias)) return sendError(req, 409, "apply_refused");
  return sendJson(req, "{\"ok\":true}");
}

esp_err_t handleSaveMix(httpd_req_t* req) {
  if (!gCb.saveMixPitchFrontBiasToNvs) return sendError(req, 500, "no_callback");
  if (!gSafeToWrite.load()) return sendError(req, 409, "armed_or_throttle_nonzero");
  if (!gCb.saveMixPitchFrontBiasToNvs()) return sendError(req, 500, "nvs_write_failed");
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

void registerUri(httpd_handle_t srv, const char* path, httpd_method_t method,
                 esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t u = {};
  u.uri = path;
  u.method = method;
  u.handler = handler;
  u.user_ctx = nullptr;
  httpd_register_uri_handler(srv, &u);
}

bool waitForWifi(const char* ssid, const char* pass, uint32_t timeoutMs) {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // disable sleep so HTTP latency is low
  WiFi.begin(ssid, pass);
  const uint32_t start = millis();
  Serial.printf("[%s] connecting to SSID '%s' (timeout %lums)\n",
                kTag, ssid, (unsigned long)timeoutMs);
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > timeoutMs) {
      Serial.printf("[%s] WiFi connect timeout\n", kTag);
      return false;
    }
    (void)esp_task_wdt_reset();
    delay(100);
  }
  Serial.printf("[%s] WiFi up: ip=%s rssi=%d dBm\n", kTag,
                WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
  return true;
}

}  // namespace

void registerCallbacks(const Callbacks& cbs) {
  gCb = cbs;
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
  if (ssid == nullptr || ssid[0] == '\0') {
    Serial.printf("[%s] start refused: SSID empty\n", kTag);
    return false;
  }

  if (!waitForWifi(ssid, password, connectTimeoutMs)) {
    return false;
  }

  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.task_priority = 3;        // well below flight/radio/sensor
  cfg.core_id = 0;              // keep off core 1 (flight + radio)
  cfg.stack_size = 6 * 1024;
  cfg.max_uri_handlers = 15;
  cfg.lru_purge_enable = true;

  esp_err_t err = httpd_start(&gServer, &cfg);
  if (err != ESP_OK) {
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
  registerUri(gServer, "/api/state",       HTTP_GET,  handleGetState);
  registerUri(gServer, "/api/health",      HTTP_GET,  handleGetHealth);
  registerUri(gServer, "/api/tune",        HTTP_GET,  handleGetTune);

  gRunning.store(true);
  Serial.printf("[%s] http server up on http://%s/\n",
                kTag, WiFi.localIP().toString().c_str());
  return true;
}

bool running() { return gRunning.load(); }

}  // namespace pid_webserver

#else  // !ENABLE_PID_WEBSERVER

// Stubs so callers can be unconditional. Linker drops them when unreferenced.
namespace pid_webserver {
void registerCallbacks(const Callbacks&) {}
bool start(const char*, const char*, uint32_t) { return false; }
void publishSafety(bool, bool) {}
bool running() { return false; }
}  // namespace pid_webserver

#endif  // ENABLE_PID_WEBSERVER
