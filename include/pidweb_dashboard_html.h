#pragma once

// =============================================================================
// Single-page flight-controller dashboard (Betaflight-configurator style).
// -----------------------------------------------------------------------------
// One PROGMEM string: HTML + CSS + vanilla JS, NO external CDN/fonts (works
// offline on the FCU's own AP/STA). Live telemetry arrives over a WebSocket
// (/ws, ~25 Hz); configuration uses the REST API with the X-Auth-Token header.
// A hand-rolled <canvas> renderer draws the 3D quad attitude model so no WebGL
// library is needed.
//
// Layout: sticky status bar -> tab nav -> one <section> per tab. Responsive
// CSS Grid (auto-fit cards) so nothing overlaps from phone to desktop.
//
// The string is split across multiple adjacent C++ string literals (the
// compiler concatenates them) only where noted; the dashboard is otherwise one
// raw literal. Keep the )DASH" delimiter out of the content.
// =============================================================================

static const char PIDWEB_DASHBOARD_HTML[] PROGMEM = R"DASH(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>FCU Dashboard</title>
<style>
:root{
 --bg:#0b0e13; --bg2:#121722; --card:#161c28; --card2:#1c2434; --line:#28303f;
 --txt:#e7ebf2; --dim:#8a94a6; --dim2:#5d6678;
 --accent:#3fa9ff; --accent2:#7c5cff;
 --ok:#33d17a; --warn:#ffb020; --err:#ff5252; --grey:#5d6678;
 --okbg:#0f2a1d; --warnbg:#2c2310; --errbg:#2e1414;
 --mono:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
 --r:10px;
}
*{box-sizing:border-box}
html,body{margin:0;background:var(--bg);color:var(--txt);
 font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;font-size:14px}
body{padding-bottom:40px}
h1,h2,h3{margin:0;font-weight:600}
a{color:var(--accent)}
.mono{font-family:var(--mono);font-variant-numeric:tabular-nums}
.dim{color:var(--dim)} .small{font-size:12px}
/* ---- status bar ---- */
#bar{position:sticky;top:0;z-index:50;background:linear-gradient(180deg,#0d1119,#0b0e13);
 border-bottom:1px solid var(--line);display:flex;flex-wrap:wrap;gap:7px;align-items:center;
 padding:8px 12px;box-shadow:0 2px 10px #0006}
#bar .brand{font-weight:700;letter-spacing:.3px;margin-right:6px;display:flex;align-items:center;gap:7px}
#bar .brand .dot{width:9px;height:9px;border-radius:50%;background:var(--err);transition:.3s}
#bar .brand .dot.on{background:var(--ok);box-shadow:0 0 8px var(--ok)}
.chip{display:inline-flex;align-items:center;gap:5px;background:var(--card);border:1px solid var(--line);
 border-radius:999px;padding:4px 10px;font-size:12px;white-space:nowrap}
.chip b{font-weight:600}
.chip .k{color:var(--dim);font-size:11px}
.chip.ok{border-color:#1e5e3e;background:var(--okbg)} .chip.ok .v{color:var(--ok)}
.chip.warn{border-color:#6b5217;background:var(--warnbg)} .chip.warn .v{color:var(--warn)}
.chip.err{border-color:#7a2b2b;background:var(--errbg)} .chip.err .v{color:var(--err)}
.chip.grey{opacity:.6}
.spacer{flex:1}
#bar button{background:var(--card);border:1px solid var(--line);color:var(--txt);border-radius:8px;
 padding:5px 10px;font-size:12px;cursor:pointer} #bar button:hover{background:var(--card2)}
/* ---- tabs ---- */
#tabs{display:flex;gap:2px;overflow-x:auto;background:var(--bg2);border-bottom:1px solid var(--line);
 padding:0 8px;position:sticky;top:46px;z-index:40}
#tabs button{background:none;border:none;border-bottom:2px solid transparent;color:var(--dim);
 padding:10px 13px;font-size:13px;cursor:pointer;white-space:nowrap}
#tabs button:hover{color:var(--txt)}
#tabs button.active{color:var(--accent);border-bottom-color:var(--accent)}
#tabs button .b{display:inline-block;min-width:16px;padding:0 4px;margin-left:4px;border-radius:8px;
 font-size:10px;background:var(--err);color:#fff}
/* ---- layout ---- */
main{padding:12px;max-width:1500px;margin:0 auto}
.tab{display:none} .tab.active{display:block}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(260px,1fr))}
.grid.wide{grid-template-columns:repeat(auto-fit,minmax(330px,1fr))}
.card{background:var(--card);border:1px solid var(--line);border-radius:var(--r);padding:12px 14px;min-width:0}
.card>h3{font-size:13px;color:var(--dim);text-transform:uppercase;letter-spacing:.5px;
 margin-bottom:10px;display:flex;align-items:center;gap:8px;justify-content:space-between}
.card.span2{grid-column:span 2} @media(max-width:680px){.card.span2{grid-column:span 1}}
.kv{display:flex;justify-content:space-between;gap:10px;padding:3px 0;border-bottom:1px dashed #ffffff0d}
.kv:last-child{border-bottom:none}
.kv .k{color:var(--dim)} .kv .v{font-family:var(--mono);font-variant-numeric:tabular-nums}
.led{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--grey);margin-right:6px;vertical-align:middle}
.led.ok{background:var(--ok)} .led.warn{background:var(--warn)} .led.err{background:var(--err)} .led.grey{background:var(--grey)}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{text-align:left;padding:6px 8px;border-bottom:1px solid var(--line)}
th{color:var(--dim);font-weight:600;font-size:11px;text-transform:uppercase;letter-spacing:.4px}
td.mono{font-family:var(--mono)}
.btn{background:var(--card2);border:1px solid var(--line);color:var(--txt);border-radius:8px;
 padding:8px 12px;font-size:13px;cursor:pointer;transition:.15s}
.btn:hover{background:#243047;border-color:#34425c}
.btn.pri{background:#13314d;border-color:#1d4f7a;color:#bfe0ff}
.btn.pri:hover{background:#184066}
.btn.danger{border-color:#5c2626;color:#ffb3b3} .btn.danger:hover{background:#3a1a1a}
.btn.sm{padding:5px 9px;font-size:12px}
.btn:disabled{opacity:.4;cursor:not-allowed}
.btns{display:flex;gap:8px;flex-wrap:wrap}
.tag{font-size:11px;padding:2px 7px;border-radius:6px;background:var(--card2);border:1px solid var(--line);color:var(--dim)}
.tag.ok{color:var(--ok);border-color:#1e5e3e} .tag.warn{color:var(--warn);border-color:#6b5217} .tag.err{color:var(--err);border-color:#7a2b2b}
input[type=number],input[type=text]{background:#0d1118;color:var(--txt);border:1px solid var(--line);
 border-radius:6px;padding:6px 8px;font-family:var(--mono);width:100%}
input[type=range]{width:100%}
.row{display:flex;align-items:center;gap:10px;margin:7px 0}
.row label{flex:0 0 130px;color:var(--dim);font-size:12px}
.row .v{flex:0 0 64px;text-align:right;font-family:var(--mono)}
canvas{display:block;width:100%;background:#0a0d12;border-radius:8px;border:1px solid var(--line)}
.note{font-size:12px;color:var(--dim);margin-top:8px;line-height:1.5}
.warnbox{background:var(--warnbg);border:1px solid #6b5217;border-radius:8px;padding:8px 10px;margin:6px 0;color:#ffd98a;font-size:13px}
.warnbox.err{background:var(--errbg);border-color:#7a2b2b;color:#ffb3b3}
.warnbox.ok{background:var(--okbg);border-color:#1e5e3e;color:#a6f0c6}
.bigwarn{background:#2a1206;border:1px solid #7a3b16;color:#ffcaa0;border-radius:8px;padding:8px 12px;
 font-size:13px;margin-bottom:12px;display:flex;gap:8px;align-items:center}
#toast{position:fixed;bottom:16px;left:50%;transform:translateX(-50%);background:#1c2434;
 border:1px solid var(--line);border-radius:8px;padding:10px 16px;font-size:13px;opacity:0;
 transition:.3s;pointer-events:none;z-index:99;max-width:90vw}
#toast.show{opacity:1}
.toolbar{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:10px}
.seg{display:inline-flex;border:1px solid var(--line);border-radius:8px;overflow:hidden}
.seg button{background:var(--card);border:none;color:var(--dim);padding:5px 10px;font-size:12px;cursor:pointer}
.seg button.active{background:var(--accent);color:#04121f}
.mbadge{font-family:var(--mono);font-size:11px}
.statgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(90px,1fr));gap:8px}
.stat{background:#0e131c;border:1px solid var(--line);border-radius:8px;padding:7px 9px}
.stat .l{font-size:10px;color:var(--dim);text-transform:uppercase}
.stat .n{font-family:var(--mono);font-size:16px;margin-top:2px}
</style></head>
<body>
<header id="bar">
 <span class="brand"><span class="dot" id="connDot"></span>FCU</span>
 <span class="chip" id="c_arm"><span class="k">STATE</span><b class="v">--</b></span>
 <span class="chip" id="c_fs"><span class="k">FS</span><b class="v">--</b></span>
 <span class="chip" id="c_batt"><span class="k">BAT</span><b class="v">--</b></span>
 <span class="chip" id="c_loop"><span class="k">LOOP</span><b class="v">--</b></span>
 <span class="chip" id="c_rc"><span class="k">RC</span><b class="v">--</b></span>
 <span class="chip" id="c_lq"><span class="k">LQ</span><b class="v">--</b></span>
 <span class="chip" id="c_tel"><span class="k">TEL</span><b class="v">--</b></span>
 <span class="chip" id="c_warn"><span class="k">WARN</span><b class="v">0</b></span>
 <span class="spacer"></span>
 <span class="chip" id="c_auth"><span class="k">AUTH</span><b class="v">?</b></span>
 <button onclick="setToken()">Token</button>
</header>
<nav id="tabs"></nav>
<main>
 <div class="bigwarn">&#9888; Bench/powered testing: <b>remove all propellers</b>. Verify the runtime failsafe setting before any powered test.</div>

 <!-- ===== OVERVIEW ===== -->
 <section class="tab active" id="t_overview">
  <div class="grid wide">
   <div class="card span2"><h3>Attitude <span class="small dim" id="ov_attmode">solid=corrected &middot; ghost=raw &middot; wire=target</span></h3>
    <canvas id="drone3d" height="300"></canvas>
    <div class="statgrid" style="margin-top:10px">
     <div class="stat"><div class="l">Roll (corr)</div><div class="n" id="ov_roll">--</div></div>
     <div class="stat"><div class="l">Pitch (corr)</div><div class="n" id="ov_pitch">--</div></div>
     <div class="stat"><div class="l">Yaw</div><div class="n" id="ov_yaw">--</div></div>
     <div class="stat"><div class="l">Throttle</div><div class="n" id="ov_thr">--</div></div>
    </div>
   </div>
   <div class="card"><h3>System health</h3><div id="ov_health"></div></div>
   <div class="card"><h3>Motors <span class="small dim">DShot raw</span></h3>
    <canvas id="ov_mix" height="180"></canvas>
    <div class="note" id="ov_mixnote">--</div>
   </div>
   <div class="card"><h3>Warnings</h3><div id="ov_warn"><div class="warnbox ok">No active warnings.</div></div></div>
   <div class="card"><h3>Radio link</h3><div id="ov_rc"></div></div>
   <div class="card"><h3>Pan / Tilt</h3><div id="ov_servo"></div></div>
  </div>
 </section>

 <!-- ===== ATTITUDE & LEVEL ===== -->
 <section class="tab" id="t_attitude">
  <div class="grid wide">
   <div class="card span2"><h3>Orientation sources</h3>
    <table><thead><tr><th></th><th>Roll&deg;</th><th>Pitch&deg;</th><th>Yaw&deg;</th></tr></thead>
     <tbody>
      <tr><td>Raw estimator</td><td class="mono" id="a_rr">--</td><td class="mono" id="a_rp">--</td><td class="mono" id="a_ry">--</td></tr>
      <tr><td>Corrected (controller)</td><td class="mono" id="a_cr">--</td><td class="mono" id="a_cp">--</td><td class="mono">--</td></tr>
      <tr><td>Target (sticks)</td><td class="mono" id="a_tr">--</td><td class="mono" id="a_tp">--</td><td class="mono" id="a_tyr">--</td></tr>
      <tr><td>Angle error</td><td class="mono" id="a_er">--</td><td class="mono" id="a_ep">--</td><td class="mono">--</td></tr>
     </tbody></table>
    <div class="note">Corrected = raw &minus; level offset &minus; trim. A physically level frame should read corrected &asymp; 0. The <b>error</b> row is what the outer loop drives to zero.</div>
   </div>
   <div class="card"><h3>IMU</h3><div id="a_imu"></div></div>
   <div class="card span2"><h3>Level calibration <span class="tag" id="a_lvltag">--</span></h3>
    <div class="note">Place the frame on a known-level surface, keep it still, then calibrate. Disarmed only. The result persists in NVS and is <b>not</b> overwritten at the next boot.</div>
    <div id="a_lvlinfo" style="margin:8px 0"></div>
    <div class="btns">
     <button class="btn pri" id="a_calbtn" onclick="act('/api/level/calibrate','Level capture started')">Calibrate Level &amp; Save</button>
     <button class="btn" onclick="act('/api/level/reload','Reloaded from NVS')">Reload from NVS</button>
     <button class="btn" onclick="act('/api/level/restore','Previous restored')">Restore previous</button>
     <button class="btn danger" onclick="act('/api/level/clear','Saved level cleared')">Clear saved</button>
    </div>
    <div class="row" style="margin-top:10px"><label>Manual offset roll&deg;</label><input type="number" step="0.1" id="a_offr"><label style="flex:0 0 auto">pitch&deg;</label><input type="number" step="0.1" id="a_offp"><button class="btn sm" onclick="applyOffset()">Apply (RAM)</button></div>
   </div>
   <div class="card span2"><h3>Manual trim</h3>
    <div class="note">Trim biases the hold point on top of the level calibration (e.g. drone drifts forward &rarr; add a little pitch). Kept separate from the mounting offset.</div>
    <div class="statgrid" style="margin:8px 0">
     <div class="stat"><div class="l">Roll trim</div><div class="n" id="a_trimr">--</div></div>
     <div class="stat"><div class="l">Pitch trim</div><div class="n" id="a_trimp">--</div></div>
    </div>
    <div class="btns">
     <button class="btn sm" onclick="trimStep('roll',1)">Roll +</button>
     <button class="btn sm" onclick="trimStep('roll',-1)">Roll &minus;</button>
     <button class="btn sm" onclick="trimStep('pitch',1)">Pitch +</button>
     <button class="btn sm" onclick="trimStep('pitch',-1)">Pitch &minus;</button>
     <button class="btn sm" onclick="act('/api/trim/reset','Trim reset')">Reset trim</button>
     <button class="btn sm pri" onclick="act('/api/level/save','Level + trim saved')">Save to NVS</button>
    </div>
   </div>
  </div>
 </section>

 <!-- ===== PID TUNING ===== -->
 <section class="tab" id="t_pid">
  <div class="grid wide" id="pidgroups"></div>
  <div class="card" style="margin-top:12px"><h3>Apply / persist</h3>
   <div class="btns">
    <button class="btn pri" onclick="savePid()">Save to NVS</button>
    <button class="btn" onclick="act('/api/pid/revert','Reverted from NVS')">Revert</button>
    <button class="btn danger" onclick="if(confirm('Reset PID to compile defaults?'))act('/api/pid/reset','Defaults restored')">Reset defaults</button>
   </div>
   <div class="note" id="pid_safe">--</div>
  </div>
 </section>

 <!-- ===== PID DIAGNOSTICS ===== -->
 <section class="tab" id="t_diag">
  <div class="grid wide">
   <div class="card span2"><h3>Rate loop terms</h3>
    <table><thead><tr><th>Axis</th><th>SP&deg;/s</th><th>Gyro&deg;/s</th><th>P</th><th>I</th><th>D</th><th>Out</th><th>Sat</th></tr></thead>
     <tbody id="diag_tbody"></tbody></table>
   </div>
   <div class="card"><h3>Gyro (filtered, &deg;/s)</h3><canvas id="diag_gyro" height="150"></canvas></div>
   <div class="card"><h3>PID output (raw)</h3><canvas id="diag_out" height="150"></canvas></div>
  </div>
 </section>

 <!-- ===== MIXER ===== -->
 <section class="tab" id="t_mixer">
  <div class="grid wide">
   <div class="card"><h3>Layout (top-down)</h3><canvas id="mix_canvas" height="320"></canvas>
    <div class="note">M1 FR (CW) &middot; M2 RR (CCW) &middot; M3 FL (CCW) &middot; M4 RL (CW). Ring fill = output above idle.</div></div>
   <div class="card span2"><h3>Mixer contributions</h3>
    <table><thead><tr><th>Motor</th><th>Base</th><th>Roll</th><th>Pitch</th><th>Yaw</th><th>Unclamped</th><th>DShot</th></tr></thead>
     <tbody id="mix_tbody"></tbody></table>
    <div id="mix_expl" class="warnbox ok" style="margin-top:10px">--</div>
   </div>
  </div>
 </section>

 <!-- ===== SENSORS ===== -->
 <section class="tab" id="t_sensors">
  <div class="card"><h3>Detected subsystems</h3>
   <table><thead><tr><th>Device</th><th>Bus</th><th>State</th><th>Value</th><th>Age</th></tr></thead>
    <tbody id="sens_tbody"></tbody></table>
   <div class="note">Green = healthy &amp; fresh &middot; Yellow = degraded/stale/uncalibrated &middot; Red = missing/invalid/timeout &middot; Grey = not compiled in. A driver being initialized is <i>not</i> the same as it returning valid recent data.</div>
  </div>
 </section>

 <!-- ===== RADIO & LINKS ===== -->
 <section class="tab" id="t_radio">
  <div class="grid wide">
   <div class="card"><h3>Control receiver (CRSF/ELRS)</h3><div id="rc_info"></div></div>
   <div class="card"><h3>Channels</h3><div id="rc_chans"></div></div>
   <div class="card"><h3>Web / WiFi telemetry</h3><div id="rc_web"></div></div>
   <div class="card"><h3>Companion (Pi)</h3><div id="rc_pi"></div></div>
  </div>
 </section>

 <!-- ===== VIBRATION & FFT ===== -->
 <section class="tab" id="t_vibe">
  <div class="grid wide">
   <div class="card span2"><h3>Final gyro entering PID <span class="small dim">post-notch &middot; &deg;/s</span></h3>
    <canvas id="vibe_strip" height="180"></canvas>
    <div class="note">Live decimated stream (~25 Hz). For full-rate raw-vs-filtered spectral content, run the FFT in <b>Filters &amp; Notch</b>.</div>
   </div>
   <div class="card"><h3>Vibration metrics <span class="small dim" id="vibe_win">~last 4 s</span></h3><div id="vibe_stats"></div></div>
  </div>
 </section>

 <!-- ===== FILTERS & NOTCH ===== -->
 <section class="tab" id="t_notch">
  <div class="grid wide">
   <div class="card span2"><h3>FFT spectrum <span class="small dim" id="nt_status">idle</span></h3>
    <div class="toolbar">
     <button class="btn pri" id="nt_run" onclick="notchRun()">Run Live Notch Analysis</button>
     <button class="btn" onclick="act('/api/notch/stop','Stopped')">Stop</button>
     <span class="seg" id="nt_axis"><button data-ax="0" class="active">Roll X</button><button data-ax="1">Pitch Y</button><button data-ax="2">Yaw Z</button></span>
     <button class="btn sm" onclick="fftExport()">Export CSV</button>
    </div>
    <canvas id="nt_fft" height="250"></canvas>
    <div class="note">Bright line = raw gyro (pre-notch) &middot; dim = final (post-notch, what the PID sees) &middot; shaded band = active notch sweep range &middot; dashed = recommended center.</div>
   </div>
   <div class="card"><h3>Current filter</h3><div id="nt_cur"></div>
    <div class="btns" style="margin-top:8px"><button class="btn sm" id="nt_en" onclick="notchToggle()">Toggle notch</button></div>
   </div>
   <div class="card"><h3>Recommendation</h3><div id="nt_rec"></div>
    <div class="btns" style="margin-top:8px">
     <button class="btn pri" onclick="notchApply()">Apply (temp)</button>
     <button class="btn" onclick="act('/api/notch/save','Notch saved to NVS')">Save NVS</button>
     <button class="btn" onclick="act('/api/notch/reload','Reloaded from NVS')">Reload</button>
    </div>
   </div>
  </div>
 </section>

 <!-- ===== PAN / TILT ===== -->
 <section class="tab" id="t_servo">
  <div class="grid wide">
   <div class="card"><h3>Pan / Tilt control <span class="tag" id="sv_ovtag">RC</span></h3>
    <div id="sv_info"></div>
    <div class="btns" style="margin:12px 0;justify-content:center">
     <button class="btn" onpointerdown="svStart(-40,0)" onpointerup="svStop()" onpointerleave="svStop()">&#9664; Pan left</button>
     <button class="btn" onpointerdown="svStart(0,40)" onpointerup="svStop()" onpointerleave="svStop()">&#9650; Tilt up</button>
     <button class="btn" onpointerdown="svStart(0,-40)" onpointerup="svStop()" onpointerleave="svStop()">&#9660; Tilt down</button>
     <button class="btn" onpointerdown="svStart(40,0)" onpointerup="svStop()" onpointerleave="svStop()">Pan right &#9654;</button>
    </div>
    <div class="row"><label>Pan µs</label><input type="range" id="sv_pan" oninput="svSetSlider()"><div class="v" id="sv_panv">--</div></div>
    <div class="row"><label>Tilt µs</label><input type="range" id="sv_tilt" oninput="svSetSlider()"><div class="v" id="sv_tiltv">--</div></div>
    <div class="btns">
     <button class="btn sm" onclick="svCenter(1,1)">Center both</button>
     <button class="btn sm" onclick="svCenter(1,0)">Center pan</button>
     <button class="btn sm" onclick="svCenter(0,1)">Center tilt</button>
     <button class="btn sm" onclick="act('/api/servo/stop','Holding position')">Stop</button>
     <button class="btn sm" onclick="act('/api/servo/release','Returned to RC')">Release to RC</button>
    </div>
   </div>
   <div class="card"><h3>Limits &amp; inversion</h3>
    <div class="row"><label>Pan min/ctr/max</label><input type="number" id="sv_pmn"><input type="number" id="sv_pc"><input type="number" id="sv_pmx"></div>
    <div class="row"><label>Tilt min/ctr/max</label><input type="number" id="sv_tmn"><input type="number" id="sv_tc"><input type="number" id="sv_tmx"></div>
    <div class="row"><label>Invert</label><label style="flex:0 0 auto"><input type="checkbox" id="sv_pi"> Pan</label><label style="flex:0 0 auto"><input type="checkbox" id="sv_ti"> Tilt</label></div>
    <div class="btns"><button class="btn pri" onclick="svSaveCfg()">Save config to NVS</button></div>
    <div class="note">Verify directions: <b>Pan left</b> should move the camera left, <b>Tilt up</b> upward. If reversed, tick Invert for that axis and re-test. Servo commands run on the control task and never block the flight loop; a 4 s command timeout hands control back to the RC sticks.</div>
   </div>
  </div>
 </section>

 <!-- ===== CAPTURE ===== -->
 <section class="tab" id="t_capture">
  <div class="grid wide">
   <div class="card"><h3>Diagnostic capture</h3>
    <div class="statgrid">
     <div class="stat"><div class="l">State</div><div class="n" id="cap_state">--</div></div>
     <div class="stat"><div class="l">Samples</div><div class="n" id="cap_samples">--</div></div>
     <div class="stat"><div class="l">Rate</div><div class="n" id="cap_rate">--</div></div>
     <div class="stat"><div class="l">Dropped</div><div class="n" id="cap_drop">--</div></div>
    </div>
    <div class="btns" style="margin-top:10px">
     <button class="btn pri" onclick="capStart()">Start Capture</button>
     <button class="btn" onclick="capArm()">Arm-Triggered Capture</button>
     <button class="btn" onclick="capStop()">Stop</button>
     <button class="btn danger" onclick="capClear()">Clear</button>
     <button class="btn" onclick="capDownload()">Download CSV</button>
    </div>
    <div class="note">Start before the props-off arming test. Arm-trigger waits while disarmed, then records the arming and first-throttle sequence into a fixed 400-row buffer at ~100 Hz.</div>
   </div>
   <div class="card"><h3>Captured fields</h3>
    <div id="cap_fields" class="small dim">Attitude, IMU, level correction, trim, PID terms, mixer contributions, DShot commands, radio state, sensor ages, battery and saturation flags are exported in CSV.</div>
   </div>
  </div>
 </section>

 <!-- ===== CONFIG ===== -->
 <section class="tab" id="t_config">
  <div class="grid wide">
   <div class="card"><h3>Mixer pitch-front bias</h3>
    <div class="note">Forward-CG compensation knob. <b>Prefer fixing level via calibration</b> rather than masking an attitude offset here.</div>
    <div class="row"><label>Front &times; pitch</label><input type="number" step="0.01" min="1" max="2" id="cfg_bias"><div class="v" id="cfg_biasnow">--</div></div>
    <div class="btns"><button class="btn" onclick="applyBias()">Apply (RAM)</button><button class="btn pri" onclick="saveBias()">Save NVS</button></div>
   </div>
   <div class="card"><h3>Accelerometer offset</h3>
    <div id="cfg_accel"></div>
    <div class="btns" style="margin-top:8px"><button class="btn" onclick="act('/api/imu/calibrate','Gyro bias calibration requested')">Calibrate gyro bias</button><button class="btn" onclick="act('/api/accel/save','Accel offset saved')">Save accel offset</button><button class="btn danger" onclick="act('/api/accel/clear','Accel offset cleared')">Clear accel offset</button></div>
   </div>
   <div class="card"><h3>Magnetometer calibration</h3>
    <div class="note">Capture hard-iron and coarse scale from a full all-axis rotation. Stored in NVS when finished.</div>
    <div class="btns" style="margin-top:8px"><button class="btn" onclick="act('/api/mag/start','Mag capture started')">Start mag capture</button><button class="btn pri" onclick="act('/api/mag/finish','Mag calibration saved')">Finish &amp; save</button></div>
   </div>
   <div class="card"><h3>Compass &amp; heading trim</h3>
    <div class="note">Live heading. Trim corrects the small residual after a good mag cal (and applies declination). <b>Save</b> persists to NVS; writes need disarmed.</div>
    <div style="display:flex;gap:14px;align-items:center;flex-wrap:wrap;margin-top:6px">
     <svg id="mag_dial" width="160" height="160" viewBox="0 0 160 160">
      <circle id="mag_ring" cx="80" cy="80" r="72" fill="none" stroke="#5d6678" stroke-width="6"></circle>
      <g fill="#8a94a6" font-size="12" text-anchor="middle">
       <text x="80" y="20">N</text><text x="80" y="151">S</text><text x="149" y="84">E</text><text x="11" y="84">W</text>
      </g>
      <g id="mag_needle" style="transform-box:fill-box;transform-origin:center;transition:transform .12s linear,opacity .2s">
       <polygon points="80,18 87,84 80,98 73,84" fill="#ff5252"></polygon>
       <polygon points="80,142 87,84 80,70 73,84" fill="#8a94a6"></polygon>
      </g>
      <circle cx="80" cy="80" r="4" fill="#e7ebf2"></circle>
     </svg>
     <div style="min-width:150px">
      <div id="mag_hdg" style="font-size:22px">--&deg;</div>
      <div id="mag_state" class="dim small">no fix</div>
      <div class="dim small" style="margin-top:8px">Field (µT)</div>
      <div style="height:12px;border-radius:6px;background:#0b0e13;overflow:hidden;margin-top:2px">
       <div id="mag_bar" style="height:100%;width:0;background:#33d17a;transition:width .2s,background .2s"></div>
      </div>
      <div id="mag_field" class="dim small">-- µT</div>
     </div>
    </div>
    <div style="display:flex;gap:10px;align-items:center;margin-top:10px">
     <label class="dim small">Trim &deg;</label>
     <input type="range" id="mag_trim" min="-360" max="360" step="1" value="0" style="flex:1" oninput="document.getElementById('mag_trimn').value=this.value">
     <input type="number" id="mag_trimn" min="-360" max="360" step="1" value="0" style="width:64px" oninput="document.getElementById('mag_trim').value=this.value">
    </div>
    <div class="btns" style="margin-top:8px"><button class="btn pri" onclick="saveMagTrim()">Save trim NVS</button><button class="btn" onclick="act('/api/calibrate?mag=1','Spin craft on all axes for 30 s...')">Start mag cal</button></div>
   </div>
   <div class="card"><h3>Failsafes</h3>
    <div id="cfg_failsafe"></div>
    <div class="btns" style="margin-top:8px"><button class="btn pri" onclick="setFailsafeBypass(0)">Enable failsafes</button><button class="btn danger" onclick="setFailsafeBypass(1)">Disable failsafes</button><button class="btn" onclick="saveFailsafeBypass()">Save to NVS</button></div>
   </div>
   <div class="card"><h3>Links</h3>
    <div class="btns"><a class="btn" href="/legacy" target="_blank">Legacy tuner</a><a class="btn" href="/api/dash" target="_blank">Raw telemetry JSON</a></div>
    <div class="note" id="cfg_build">--</div>
   </div>
  </div>
 </section>
</main>
<div id="toast"></div>
<script>
)DASH"  // ---- split: JS follows in a second literal so this header stays scannable ----
R"DASH(
"use strict";
// ---------- auth token ----------
let TOKEN=new URLSearchParams(location.search).get('token')||localStorage.getItem('fcuToken')||'';
if(TOKEN){try{localStorage.setItem('fcuToken',TOKEN);}catch(e){}}
function authHdrs(x){const h=Object.assign({},x||{});if(TOKEN)h['X-Auth-Token']=TOKEN;return h;}
function setToken(){const t=prompt('Paste FCU_PID_AUTH_TOKEN:',TOKEN);if(t===null)return;TOKEN=t.trim();
 try{TOKEN?localStorage.setItem('fcuToken',TOKEN):localStorage.removeItem('fcuToken');}catch(e){}updAuth();}
function updAuth(){const c=document.querySelector('#c_auth');c.className='chip '+(TOKEN?'ok':'err');
 c.querySelector('.v').textContent=TOKEN?'set':'none';}
let toastT=null;
function toast(m,err){const t=document.getElementById('toast');t.textContent=m;t.style.borderColor=err?'#7a2b2b':'#28303f';
 t.classList.add('show');clearTimeout(toastT);toastT=setTimeout(()=>t.classList.remove('show'),2600);}
// ---------- REST helpers ----------
async function act(url,okMsg,body){
 try{const o={method:'POST',headers:authHdrs(body?{'content-type':'application/json'}:{})};
  if(body)o.body=JSON.stringify(body);
  const r=await fetch(url,o);
  if(r.ok){toast(okMsg||'OK');return true;}
  if(r.status===401){toast('Auth required — set token',true);return false;}
  let m='Failed ('+r.status+')';try{const j=await r.json();if(j.error)m+=': '+j.error;}catch(e){}
  toast(m,true);return false;
 }catch(e){toast('Network error',true);return false;}
}
async function put(url,okMsg,body){
 try{const r=await fetch(url,{method:'PUT',headers:authHdrs({'content-type':'application/json'}),body:JSON.stringify(body)});
  if(r.ok){if(okMsg)toast(okMsg);return true;}
  if(r.status===401){toast('Auth required — set token',true);return false;}
  let m='Failed ('+r.status+')';try{const j=await r.json();if(j.error)m+=': '+j.error;}catch(e){}toast(m,true);return false;
 }catch(e){toast('Network error',true);return false;}
}
// ---------- tabs ----------
const TABS=[['overview','Overview'],['attitude','Attitude & Level'],['pid','PID Tuning'],
 ['diag','PID Diagnostics'],['mixer','Motor Mixer'],['sensors','Sensors'],['radio','Radio & Links'],
 ['vibe','Vibration & FFT'],['notch','Filters & Notch'],['servo','Pan/Tilt'],['capture','Capture'],['config','Config']];
let activeTab='overview';
function buildTabs(){const n=document.getElementById('tabs');TABS.forEach(([id,label])=>{
 const b=document.createElement('button');b.id='tab_'+id;b.textContent=label;b.onclick=()=>showTab(id);
 if(id==='overview')b.classList.add('active');n.appendChild(b);});}
function showTab(id){activeTab=id;document.querySelectorAll('#tabs button').forEach(b=>b.classList.remove('active'));
 document.getElementById('tab_'+id).classList.add('active');
 document.querySelectorAll('.tab').forEach(s=>s.classList.remove('active'));
 document.getElementById('t_'+id).classList.add('active');
 if(window.onTabShown)window.onTabShown(id);}
// ---------- format helpers ----------
const f=(v,d)=>(v==null||isNaN(v))?'--':(+v).toFixed(d==null?2:d);
const ageTxt=ms=>ms>=4294967295||ms==null?'never':(ms<2000?ms+'ms':(ms/1000).toFixed(1)+'s');
function led(state){return '<span class="led '+state+'"></span>';}
function kv(k,v){return '<div class="kv"><span class="k">'+k+'</span><span class="v">'+v+'</span></div>';}
// ---------- WebSocket ----------
let LAST=null,wsBad=0,lastMsgT=0;
function connectWs(){
 let url=(location.protocol==='https:'?'wss://':'ws://')+location.host+'/ws';
 let ws;try{ws=new WebSocket(url);}catch(e){setTimeout(connectWs,1500);return;}
 ws.onopen=()=>{document.getElementById('connDot').classList.add('on');};
 ws.onclose=()=>{document.getElementById('connDot').classList.remove('on');setTimeout(connectWs,1500);};
 ws.onerror=()=>{try{ws.close();}catch(e){}};
 ws.onmessage=ev=>{try{LAST=JSON.parse(ev.data);lastMsgT=Date.now();render(LAST);}catch(e){}};
}
// fallback: if WS quiet >1.5s, poll /api/dash
setInterval(async()=>{if(Date.now()-lastMsgT>1500){try{const r=await fetch('/api/dash');if(r.ok){LAST=await r.json();render(LAST);}}catch(e){}}},1000);
)DASH"
R"DASH(
// ---------- warnings engine ----------
function computeWarnings(m){
 const w=[];const st=m.sys,a=m.att,im=m.imu,lv=m.lvl,pid=m.pid,sen=m.sen,rc=m.rc;
 const stationary=!st.armed && st.thr===0;
 if(st.fs)w.push(['err','Failsafe ACTIVE (reason '+st.fsr+')']);
 if(!st.link && rc.comp)w.push(['err','Control link DOWN']);
 if(rc.comp && rc.up && rc.lq<50 && rc.lq>0)w.push(['warn','Low RC link quality: '+rc.lq+'%']);
 if(rc.comp && rc.fs)w.push(['warn','Receiver reports failsafe (LQ 0)']);
 if(st.loop>0 && st.loop<400)w.push(['warn','Loop rate low: '+st.loop+' Hz']);
 if(st.ovr>0)w.push(['warn','Flight loop overruns: '+st.ovr]);
 if(!im.rdy)w.push(['err','IMU not ready']);
 if(im.rdy && (im.am<0.85||im.am>1.15))w.push(['warn','Accel magnitude off: '+f(im.am,3)+' g']);
 if(im.gbv){const gb=Math.max(Math.abs(im.gb[0]),Math.abs(im.gb[1]),Math.abs(im.gb[2]));
  if(gb>5)w.push(['warn','Large gyro bias: '+f(gb,2)+' °/s']);}
 if(!lv.ld)w.push(['warn','No saved level calibration loaded — run Calibrate Level']);
 if(stationary && (Math.abs(a.cr)>2||Math.abs(a.cp)>2))
  w.push(['err','Corrected attitude not level while still: roll '+f(a.cr,1)+'° pitch '+f(a.cp,1)+'° — calibrate level']);
 // motor spread before throttle
 if(st.thr===0){const mm=m.mix.m;const sp=Math.max(...mm)-Math.min(...mm);
  if(sp>4 && Math.max(...mm)>0)w.push(['warn','Motors unequal at zero throttle (spread '+sp+')']);}
 if(pid.smax)w.push(['warn','Mixer upper saturation']);
 if(pid.ssc)w.push(['warn','Mixer correction scaled (severe)']);
 if(sen.baro.r&&sen.baro.age>2000)w.push(['warn','Baro stale ('+ageTxt(sen.baro.age)+')']);
 if(sen.gps.comp&&sen.gps.r&&!sen.gps.fix)w.push(['warn','GPS connected, no fix']);
 if(sen.tof.comp&&sen.tof.r&&sen.tof.age>1500)w.push(['warn','ToF stale']);
 if(st.blow)w.push(['warn','Battery low: '+f(st.bv,2)+' V']);
 if(st.fsb)w.push(['err','Failsafe bypass is active']);
 return w;
}
// ---------- render ----------
function setChip(id,cls,txt){const c=document.getElementById(id);c.className='chip '+cls;c.querySelector('.v').textContent=txt;}
function render(m){
 const st=m.sys;
 // status bar
 setChip('c_arm',st.armed?'err':'ok',st.armed?'ARMED':'DISARMED');
 setChip('c_fs',st.fs?'err':'ok',st.fs?'FAILSAFE':'clear');
 if(st.ben)setChip('c_batt',st.blow?'err':(st.bp>=0&&st.bp<25?'warn':'ok'),f(st.bv,2)+'V'+(st.bp>=0?' '+st.bp+'%':''));
 else setChip('c_batt','grey','off');
 setChip('c_loop',st.loop>=400?'ok':(st.loop>0?'warn':'err'),st.loop+'Hz');
 const rc=m.rc;
 if(!rc.comp)setChip('c_rc','grey','n/a');else setChip('c_rc',rc.up?'ok':'err',rc.up?'up':'down');
 if(!rc.comp)setChip('c_lq','grey','n/a');else setChip('c_lq',rc.lq>=70?'ok':(rc.lq>0?'warn':'err'),rc.lq+'%');
 setChip('c_tel','ok',(st.whz||0)+'Hz/'+(st.wsc||0));
 const warns=computeWarnings(m);
 const wc=document.getElementById('c_warn');wc.className='chip '+(warns.some(x=>x[0]==='err')?'err':(warns.length?'warn':'ok'));
 wc.querySelector('.v').textContent=warns.length;
 // overview
 document.getElementById('ov_roll').textContent=f(m.att.cr,1)+'°';
 document.getElementById('ov_pitch').textContent=f(m.att.cp,1)+'°';
 document.getElementById('ov_yaw').textContent=f(m.att.ry,1)+'°';
 document.getElementById('ov_thr').textContent=st.thr+'%';
 renderHealth(m);renderWarn(warns);renderOvRc(m);renderOvServo(m);
 drawDrone(m);drawMix(document.getElementById('ov_mix'),m,true);
 // active tab content
 if(activeTab==='attitude')renderAttitude(m);
 else if(activeTab==='diag')renderDiag(m);
 else if(activeTab==='mixer')renderMixer(m);
 else if(activeTab==='sensors')renderSensors(m);
 else if(activeTab==='radio')renderRadio(m);
 else if(activeTab==='config')renderConfig(m);
 if(window.onRender)window.onRender(m);
}
function renderHealth(m){const st=m.sys,el=document.getElementById('ov_health');
 el.innerHTML=kv('Uptime',(st.up/1000|0)+' s')+kv('Free heap',(st.heap/1024|0)+' KB (min '+(st.minheap/1024|0)+')')
 +kv('Loop / max',st.loop+' Hz / '+st.maxus+' µs')+kv('Overruns',st.ovr)
 +kv('Estimator',(m.sen.ekf?'EKF+':'')+'complementary')+kv('Telemetry',(st.whz||0)+' Hz, '+(st.wsc||0)+' client(s)');}
function renderWarn(warns){const el=document.getElementById('ov_warn');
 if(!warns.length){el.innerHTML='<div class="warnbox ok">No active warnings.</div>';return;}
 el.innerHTML=warns.map(w=>'<div class="warnbox '+(w[0]==='err'?'err':'')+'">'+w[1]+'</div>').join('');}
function renderOvRc(m){const rc=m.rc,el=document.getElementById('ov_rc');
 if(!rc.comp){el.innerHTML='<div class="dim small">No CRSF receiver compiled in.</div>';return;}
 el.innerHTML=kv('Link',(rc.up?'<span class="tag ok">UP</span>':'<span class="tag err">DOWN</span>'))
 +kv('LQ / RSSI',rc.lq+'% / '+rc.rssi+' dBm')+kv('Frame rate',rc.fr+' Hz')
 +kv('Last packet',ageTxt(rc.age))+kv('Loss',rc.loss+'%');}
function renderOvServo(m){const s=m.srv,el=document.getElementById('ov_servo');
 if(!s.att){el.innerHTML='<div class="dim small">Pan/tilt not attached.</div>';return;}
 el.innerHTML=kv('Pan',s.pan+' µs')+kv('Tilt',s.tilt+' µs');}
)DASH"
R"DASH(
// ---------- 3D drone (hand-rolled canvas) ----------
// Body frame FRD: x=forward, y=right, z=down. Euler: +roll=right-wing-down,
// +pitch=nose-up, +yaw=nose-right (firmware convention). We rotate model points
// body->world (NED), convert to a graphics frame (x=right,y=up,z=fwd), apply a
// fixed 3/4 camera, and project orthographically.
let show={solid:true,ghost:true,wire:true};
function matMul(a,b){const r=[0,0,0,0,0,0,0,0,0];for(let i=0;i<3;i++)for(let j=0;j<3;j++){let s=0;
 for(let k=0;k<3;k++)s+=a[i*3+k]*b[k*3+j];r[i*3+j]=s;}return r;}
function mv(a,v){return [a[0]*v[0]+a[1]*v[1]+a[2]*v[2],a[3]*v[0]+a[4]*v[1]+a[5]*v[2],a[6]*v[0]+a[7]*v[1]+a[8]*v[2]];}
function Rx(t){const c=Math.cos(t),s=Math.sin(t);return [1,0,0,0,c,-s,0,s,c];}
function Ry(t){const c=Math.cos(t),s=Math.sin(t);return [c,0,s,0,1,0,-s,0,c];}
function Rz(t){const c=Math.cos(t),s=Math.sin(t);return [c,-s,0,s,c,0,0,0,1];}
const D2R=Math.PI/180;
// fixed camera: look from front-above-right
const CAM=matMul(Rx(-62*D2R),Ry(28*D2R));
// body->NED, 3-2-1 (yaw·pitch·roll). Verified per-axis: +roll right-wing-down,
// +pitch nose-up, +yaw nose-right all map to the visually-correct screen motion.
function attMat(roll,pitch,yaw){return matMul(Rz(yaw*D2R),matMul(Ry(pitch*D2R),Rx(roll*D2R)));}
function proj(p,cx,cy,sc){
 // p in NED (x=N,y=E,z=D). graphics: gx=E=y, gy=-D=-z(up), gz=N=x
 const g=[p[1],-p[2],p[0]];const c=mv(CAM,g);
 return [cx+c[0]*sc, cy-c[1]*sc, c[2]];
}
function droneModel(){
 // arms to motors (FRD): FR(+x,+y) FL(+x,-y) RR(-x,+y) RL(-x,-y)
 const L=1.0;const m={
  M1:[ L, L,0],M3:[ L,-L,0],M2:[-L, L,0],M4:[-L,-L,0]
 };return m;
}
function drawFrame(ctx,R,cx,cy,sc,style){
 const m=droneModel();const motors=[['M1',m.M1,'#ff5252'],['M2',m.M2,'#ffb020'],['M3',m.M3,'#33d17a'],['M4',m.M4,'#3fa9ff']];
 // arms from center
 ctx.lineWidth=style.lw;ctx.strokeStyle=style.arm;
 for(const[,pos]of motors){const a=proj(mv(R,[0,0,0]),cx,cy,sc);const b=proj(mv(R,pos),cx,cy,sc);
  ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();}
 // body
 const bc=proj(mv(R,[0,0,0]),cx,cy,sc);
 ctx.fillStyle=style.body;ctx.beginPath();ctx.arc(bc[0],bc[1],sc*0.22,0,7);ctx.fill();
 // front marker (nose +x)
 const nose=proj(mv(R,[1.5,0,0]),cx,cy,sc);
 ctx.strokeStyle=style.nose;ctx.lineWidth=style.lw+1;ctx.beginPath();ctx.moveTo(bc[0],bc[1]);ctx.lineTo(nose[0],nose[1]);ctx.stroke();
 // motors + prop discs + labels
 for(const[name,pos,col]of motors){const p=proj(mv(R,pos),cx,cy,sc);
  ctx.strokeStyle=style.solid?col:style.arm;ctx.lineWidth=style.lw;
  ctx.beginPath();ctx.arc(p[0],p[1],sc*0.42,0,7);ctx.stroke();
  if(style.solid){ctx.fillStyle=col+'22';ctx.fill();ctx.fillStyle=col;ctx.font='bold 11px sans-serif';ctx.fillText(name,p[0]-7,p[1]+4);}
 }
}
function drawAxes(ctx,R,cx,cy,sc){
 const o=proj(mv(R,[0,0,0]),cx,cy,sc);
 const ax=[[[2.2,0,0],'#ff5252','X'],[[0,2.2,0],'#33d17a','Y'],[[0,0,1.6],'#3fa9ff','Z']];
 ctx.font='10px sans-serif';
 for(const[v,c,n]of ax){const p=proj(mv(R,v),cx,cy,sc);
  ctx.strokeStyle=c;ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(o[0],o[1]);ctx.lineTo(p[0],p[1]);ctx.stroke();
  ctx.fillStyle=c;ctx.fillText(n,p[0]+2,p[1]);}
}
function drawDrone(m){
 const cv=document.getElementById('drone3d');if(!cv)return;const ctx=cv.getContext('2d');
 const W=cv.width=cv.clientWidth,H=cv.height;ctx.clearRect(0,0,W,H);
 const cx=W/2,cy=H/2+10,sc=Math.min(W,H)/5.2;
 // target (wireframe), raw (ghost), corrected (solid) drawn back-to-front
 if(show.wire){const Rt=attMat(m.att.tr,m.att.tp,m.att.ry);drawFrame(ctx,Rt,cx,cy,sc,{lw:1,arm:'#7c5cff66',body:'#7c5cff22',nose:'#7c5cff88',solid:false});}
 if(show.ghost){const Rr=attMat(m.att.rr,m.att.rp,m.att.ry);drawFrame(ctx,Rr,cx,cy,sc,{lw:1.5,arm:'#8a94a655',body:'#8a94a633',nose:'#8a94a6aa',solid:false});}
 if(show.solid){const Rc=attMat(m.att.cr,m.att.cp,m.att.ry);drawFrame(ctx,Rc,cx,cy,sc,{lw:3,arm:'#cdd6e6',body:'#1c2434',nose:'#ffffff',solid:true});drawAxes(ctx,Rc,cx,cy,sc);}
 // legend
 ctx.font='11px sans-serif';ctx.fillStyle='#8a94a6';ctx.fillText('X fwd (red) · Y right (grn) · Z down (blu)',8,H-8);
}
)DASH"
R"DASH(
// ---------- mixer drawing ----------
function motorColor(v){ // v normalized 0..1
 if(v>0.66)return '#ff5252';if(v>0.33)return '#ffb020';return '#33d17a';}
function drawMix(cv,m,compact){
 if(!cv)return;const ctx=cv.getContext('2d');const W=cv.width=cv.clientWidth,H=cv.height;ctx.clearRect(0,0,W,H);
 const raw=m.mix.m;const lo=48,hi=2047;
 const pos={0:[0.74,0.28],1:[0.74,0.72],2:[0.26,0.28],3:[0.26,0.72]}; // M1 FR,M2 RR,M3 FL,M4 RL (screen: x right, y down)
 const names=['M1','M2','M3','M4'];
 // frame cross
 ctx.strokeStyle='#28303f';ctx.lineWidth=2;
 ctx.beginPath();ctx.moveTo(W*0.26,H*0.28);ctx.lineTo(W*0.74,H*0.72);ctx.moveTo(W*0.74,H*0.28);ctx.lineTo(W*0.26,H*0.72);ctx.stroke();
 ctx.fillStyle='#5d6678';ctx.font='10px sans-serif';ctx.fillText('FRONT',W/2-16,14);
 const mx=Math.max(...raw,lo+1);
 for(let i=0;i<4;i++){const x=W*pos[i][0],y=H*pos[i][1];
  const norm=Math.max(0,(raw[i]-lo)/(hi-lo));const rNorm=Math.max(0,(raw[i]-lo)/(Math.max(1,mx-lo)));
  const R=Math.min(W,H)*0.17;
  ctx.beginPath();ctx.arc(x,y,R,0,7);ctx.strokeStyle='#28303f';ctx.lineWidth=3;ctx.stroke();
  ctx.beginPath();ctx.arc(x,y,R,-Math.PI/2,-Math.PI/2+rNorm*2*Math.PI);ctx.strokeStyle=motorColor(norm);ctx.lineWidth=5;ctx.stroke();
  ctx.fillStyle='#e7ebf2';ctx.font='bold 13px sans-serif';ctx.textAlign='center';ctx.fillText(names[i],x,y-4);
  ctx.font='11px var(--mono)';ctx.fillStyle='#8a94a6';ctx.fillText(raw[i],x,y+12);ctx.textAlign='left';
 }
}
function mixExplain(m){
 const raw=m.mix.m;const sp=Math.max(...raw)-Math.min(...raw);
 if(Math.max(...raw)<=48)return['ok','Motors idle (zero throttle).'];
 if(sp<=4)return['ok','Motors balanced (spread '+sp+').'];
 // which axis dominates
 const pf=m.mix.pf,pr=m.mix.pr,r=m.mix.r,y=m.mix.y;
 const hi=raw.indexOf(Math.max(...raw)),names=['M1 (FR)','M2 (RR)','M3 (FL)','M4 (RL)'];
 let axis='',mag=Math.max(Math.abs(r),Math.abs((pf+pr)/2),Math.abs(y));
 if(mag===Math.abs((pf+pr)/2))axis=(pf+pr)>0?'pitch (nose-down correction → front motors up)':'pitch (nose-up correction → rear motors up)';
 else if(mag===Math.abs(r))axis='roll';else axis='yaw';
 const front=raw[0]+raw[2],rear=raw[1]+raw[3];
 const side = front>rear?'FRONT (M1+M3)':'REAR (M2+M4)';
 return[(sp>40?'':'ok'),'Spread '+sp+' DShot. Dominant axis: '+axis+'. Higher pair: '+side+'. P/I/D pitch out='+f(m.pid.p[3],1)+', roll out='+f(m.pid.r[3],1)+'.'];
}
function renderMixer(m){
 drawMix(document.getElementById('mix_canvas'),m,false);
 const mb=m.mix,raw=mb.m;const names=['M1 FR (CW)','M2 RR (CCW)','M3 FL (CCW)','M4 RL (CW)'];
 const pitch=[mb.pf,mb.pr,mb.pf,mb.pr];const roll=[-mb.r,-mb.r,mb.r,mb.r];const yaw=[-mb.y,mb.y,mb.y,-mb.y];
 let h='';for(let i=0;i<4;i++)h+='<tr><td>'+names[i]+'</td><td class="mono">'+f(mb.base,0)+'</td><td class="mono">'+f(roll[i],1)
  +'</td><td class="mono">'+f(pitch[i],1)+'</td><td class="mono">'+f(yaw[i],1)+'</td><td class="mono">'+f(mb.unc[i],0)+'</td><td class="mono">'+raw[i]+'</td></tr>';
 document.getElementById('mix_tbody').innerHTML=h;
 const ex=mixExplain(m);const eb=document.getElementById('mix_expl');eb.className='warnbox '+(ex[0]||'');eb.textContent=ex[1];
 // overview note
 document.getElementById('ov_mixnote').textContent=ex[1];
}
)DASH"
R"DASH(
// ---------- attitude tab ----------
function renderAttitude(m){
 const a=m.att,im=m.imu,lv=m.lvl;
 const S=(id,v,d)=>{const e=document.getElementById(id);if(e)e.textContent=f(v,d);};
 S('a_rr',a.rr,2);S('a_rp',a.rp,2);S('a_ry',a.ry,1);S('a_cr',a.cr,2);S('a_cp',a.cp,2);
 S('a_tr',a.tr,2);S('a_tp',a.tp,2);document.getElementById('a_tyr').textContent=f(a.tyr,1)+' /s';
 S('a_er',a.er,2);S('a_ep',a.ep,2);
 document.getElementById('a_imu').innerHTML=
  kv('Accel (g)',a.at?'':'<span class="tag warn">untrusted</span>'+'&nbsp;['+f(im.a[0],3)+', '+f(im.a[1],3)+', '+f(im.a[2],3)+']')
  +kv('|Accel|',f(im.am,3)+' g')
  +kv('Gyro (°/s)','['+f(im.g[0],1)+', '+f(im.g[1],1)+', '+f(im.g[2],1)+']')
  +kv('Gyro bias',im.gbv?'['+f(im.gb[0],2)+', '+f(im.gb[1],2)+', '+f(im.gb[2],2)+']':'<span class="tag warn">uncal</span>')
  +kv('Accel offset',im.av?'['+f(im.ao[0],3)+', '+f(im.ao[1],3)+', '+f(im.ao[2],3)+']':'<span class="tag warn">none</span>');
 const states=['idle','capturing…','OK','FAILED'];const errs=['','motion','accel','range','NVS','not idle'];
 const tag=document.getElementById('a_lvltag');
 tag.className='tag '+(lv.st===1?'warn':(lv.st===2?'ok':(lv.st===3?'err':'')));
 tag.textContent=states[lv.st]+(lv.st===3&&lv.er?' ('+errs[lv.er]+')':'');
 document.getElementById('a_lvlinfo').innerHTML=
  kv('Saved level',lv.ld?'<span class="tag ok">loaded</span>':'<span class="tag warn">none</span>')
  +kv('Offset (roll/pitch)',f(lv.o[0],3)+'° / '+f(lv.o[1],3)+'°')
  +kv('Capture quality (std)',lv.st?f(lv.std[0],3)+'° / '+f(lv.std[1],3)+'° ('+lv.n+' samp)':'—');
 const tr=document.getElementById('a_trimr'),tp=document.getElementById('a_trimp');
 if(tr)tr.textContent=f(lv.t[0],2)+'°';if(tp)tp.textContent=f(lv.t[1],2)+'°';
}
function applyOffset(){const r=parseFloat(document.getElementById('a_offr').value||'0'),p=parseFloat(document.getElementById('a_offp').value||'0');
 put('/api/level/offset','Offset applied (RAM)',{roll:r,pitch:p});}
function trimStep(axis,dir){const lv=LAST&&LAST.lvl;if(!lv)return;const step=(window.TRIMSTEP||0.5)*dir;
 let r=lv.t[0],p=lv.t[1];if(axis==='roll')r+=step;else p+=step;put('/api/trim','Trim '+axis+' '+(dir>0?'+':'−'),{roll:r,pitch:p});}
// ---------- diagnostics tab ----------
const stripG=mkStrip('diag_gyro',[['x','#ff5252'],['y','#33d17a'],['z','#3fa9ff']],200);
const stripO=mkStrip('diag_out',[['roll','#ff5252'],['pitch','#33d17a'],['yaw','#3fa9ff']],260);
function renderDiag(m){
 const ax=[['Roll',m.pid.r,m.pid.sp[0],m.imu.g[0],m.pid.sat[0]],['Pitch',m.pid.p,m.pid.sp[1],m.imu.g[1],m.pid.sat[1]],['Yaw',m.pid.y,m.pid.sp[2],m.imu.g[2],m.pid.sat[2]]];
 let h='';for(const[n,t,sp,gy,sat]of ax)h+='<tr><td>'+n+'</td><td class="mono">'+f(sp,1)+'</td><td class="mono">'+f(gy,1)
  +'</td><td class="mono">'+f(t[0],1)+'</td><td class="mono">'+f(t[1],1)+'</td><td class="mono">'+f(t[2],1)+'</td><td class="mono">'+f(t[3],1)
  +'</td><td>'+(sat?'<span class="tag err">SAT</span>':'')+'</td></tr>';
 document.getElementById('diag_tbody').innerHTML=h;
 stripG.push([m.imu.g[0],m.imu.g[1],m.imu.g[2]]);stripO.push([m.pid.r[3],m.pid.p[3],m.pid.y[3]]);
 stripG.draw();stripO.draw();
}
// ---------- strip chart ----------
function mkStrip(canvasId,series,range){
 const N=180;const data=series.map(()=>new Array(N).fill(0));let head=0;
 return{push(vals){for(let i=0;i<series.length;i++)data[i][head]=vals[i];head=(head+1)%N;},
  draw(){const cv=document.getElementById(canvasId);if(!cv)return;const ctx=cv.getContext('2d');
   const W=cv.width=cv.clientWidth,H=cv.height;ctx.clearRect(0,0,W,H);
   ctx.strokeStyle='#1c2434';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(0,H/2);ctx.lineTo(W,H/2);ctx.stroke();
   const sc=(H/2-6)/range;
   for(let s=0;s<series.length;s++){ctx.strokeStyle=series[s][1];ctx.lineWidth=1.4;ctx.beginPath();
    for(let i=0;i<N;i++){const idx=(head+i)%N;const x=i/(N-1)*W;const y=H/2-data[s][idx]*sc;
     if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}ctx.stroke();}
   ctx.font='10px sans-serif';let lx=6;for(const[n,c]of series){ctx.fillStyle=c;ctx.fillText(n,lx,12);lx+=34;}
  }};
}
)DASH"
R"DASH(
// ---------- sensors tab ----------
function sensRow(name,bus,state,stateCls,val,age){
 return '<tr><td>'+name+'</td><td class="dim small">'+bus+'</td><td>'+led(stateCls)+state+'</td><td class="mono">'+val+'</td><td class="mono dim">'+age+'</td></tr>';}
function renderSensors(m){
 const s=m.sen,im=m.imu;let h='';
 const gpsSp=s.gps.gsv?f(s.gps.spd,2)+' m/s ('+f(s.gps.sp/10,1)+' km/h)':'--';
 const gpsCog=s.gps.cv?f(s.gps.cog/100,1)+' deg':'--';
 const ek=s.ekfd||{};
 // IMU accel/gyro
 h+=sensRow('Gyroscope (ICM-20948)','SPI',im.rdy?'healthy':'fail',im.rdy?'ok':'err','['+f(im.g[0],0)+','+f(im.g[1],0)+','+f(im.g[2],0)+'] °/s','live');
 h+=sensRow('Accelerometer','SPI',im.rdy?(im.av?'healthy':'uncalibrated'):'fail',im.rdy?(im.av?'ok':'warn'):'err',f(im.am,3)+' g','live');
 h+=sensRow('Magnetometer (AK09916)','SPI(aux)',s.mag.v?(s.mag.cal?'healthy':'uncalibrated'):'invalid',s.mag.v?(s.mag.cal?'ok':'warn'):'err',f(s.mag.hdg,0)+'° '+f(s.mag.f,0)+'µT','live');
 h+=sensRow('Barometer (BMP280)','I2C',!s.baro.r?'missing':(s.baro.v&&s.baro.age<2000?'healthy':'stale'),!s.baro.r?'red':(s.baro.v&&s.baro.age<2000?'ok':'warn'),f(s.baro.alt,2)+' m '+f(s.baro.t,1)+'°C',ageTxt(s.baro.age));
 h+=sensRow('ToF (VL53L1X)',s.tof.comp?'I2C':'—',!s.tof.comp?'not compiled':(!s.tof.r?'missing':(s.tof.rng&&s.tof.age<1500?'healthy':'stale')),!s.tof.comp?'grey':(!s.tof.r?'red':(s.tof.rng&&s.tof.age<1500?'ok':'warn')),s.tof.mm+' mm',ageTxt(s.tof.age));
 h+=sensRow('GPS (NMEA)',s.gps.comp?'UART1':'—',!s.gps.comp?'not compiled':(!s.gps.r?'missing':(s.gps.fix?'fix '+s.gps.sats+' sats':'no fix')),!s.gps.comp?'grey':(!s.gps.r?'red':(s.gps.fix?'ok':'warn')),(s.gps.lat/1e7).toFixed(5)+', '+(s.gps.lon/1e7).toFixed(5),ageTxt(s.gps.age));
 h+=sensRow('GPS RMC velocity','UART1',s.gps.vv?'valid':(s.gps.gsv?'speed only':'waiting'),s.gps.vv?'ok':(s.gps.gsv?'warn':'grey'),gpsSp+' cog '+gpsCog+' vNE ['+f(s.gps.vn,2)+','+f(s.gps.ve,2)+']',ageTxt(s.gps.rmc));
 h+=sensRow('Battery ADC',m.sys.ben?'ADC':'—',m.sys.ben?(m.sys.blow?'low':'healthy'):'off',m.sys.ben?(m.sys.blow?'warn':'ok'):'grey',f(m.sys.bv,2)+' V '+(m.sys.bp>=0?m.sys.bp+'%':''),'live');
 h+=sensRow('EKF (shadow)','—',s.ekf?(ek.if?'rejecting':'running'):'idle',s.ekf?(ek.if?'warn':'ok'):'grey',s.ekf?('yaw '+f(ek.yaw,1)+' deg vNED ['+f(ek.v?.[0],2)+','+f(ek.v?.[1],2)+','+f(ek.v?.[2],2)+']'):'—','—');
 h+=sensRow('EKF GPS/Mag aid','—',s.ekf?('gps '+(ek.gv?'valid':'idle')+' mag '+(ek.mv?'valid':'idle')):'idle',s.ekf?((ek.gv&&ek.mv)?'ok':'warn'):'grey',s.ekf?('gps '+(ek.gps?.[0]||0)+'/'+(ek.gps?.[1]||0)+' mag '+(ek.mag?.[0]||0)+'/'+(ek.mag?.[1]||0)+' mi '+f(ek.mi,1)+' deg'):'—','drops '+(ek.drop||0));
 h+=sensRow('Companion (Pi)',m.pi.comp?'UART2':'—',!m.pi.comp?'not compiled':(m.pi.alive?'alive':'no link'),!m.pi.comp?'grey':(m.pi.alive?'ok':'warn'),'hb '+ageTxt(m.pi.hb),ageTxt(m.pi.hb));
 document.getElementById('sens_tbody').innerHTML=h;
}
// ---------- radio tab ----------
function renderRadio(m){
 const rc=m.rc;
 document.getElementById('rc_info').innerHTML=rc.comp?(
  kv('Protocol','CRSF / ELRS')+kv('Link',rc.up?'<span class="tag ok">UP</span>':'<span class="tag err">DOWN</span>')
  +kv('Failsafe',rc.fs?'<span class="tag err">yes</span>':'<span class="tag ok">no</span>')
  +kv('Link quality',rc.lq+' %')+kv('RSSI',rc.rssi+' dBm')+kv('Frame rate',rc.fr+' Hz')
  +kv('Last packet',ageTxt(rc.age))+kv('Loss',rc.loss+' %')+kv('Accepted/s',rc.pps)
 ):'<div class="dim">No CRSF control compiled in this build.</div>';
 const names=['Roll','Pitch','Throttle','Yaw','Arm','Pan','Tilt','Aux8'];
 let h='';for(let i=0;i<8;i++){const us=rc.ch[i]||0;const p=Math.max(0,Math.min(100,(us-1000)/10));
  h+='<div class="row" style="margin:4px 0"><label style="flex:0 0 70px">'+names[i]+'</label><div style="flex:1;background:#0d1118;border:1px solid #28303f;border-radius:5px;height:14px;position:relative"><div style="position:absolute;left:0;top:0;bottom:0;width:'+p+'%;background:#3fa9ff44;border-radius:5px"></div></div><div class="v">'+us+'</div></div>';}
 document.getElementById('rc_chans').innerHTML=h;
 document.getElementById('rc_web').innerHTML=kv('WebSocket',document.getElementById('connDot').classList.contains('on')?'<span class="tag ok">connected</span>':'<span class="tag err">down</span>')
  +kv('Push rate',(m.sys.whz||0)+' Hz')+kv('Clients',m.sys.wsc||0)+kv('Free heap',(m.sys.heap/1024|0)+' KB');
 document.getElementById('rc_pi').innerHTML=m.pi.comp?(kv('Link',m.pi.alive?'<span class="tag ok">alive</span>':'<span class="tag warn">no link</span>')+kv('Heartbeat age',ageTxt(m.pi.hb))):'<div class="dim">Autonomy UART not compiled in.</div>';
}
// ---------- config tab ----------
function renderConfig(m){
 document.getElementById('cfg_biasnow').textContent=f(m.mix.bias,3);
 const bi=document.getElementById('cfg_bias');if(bi&&document.activeElement!==bi)bi.value=m.mix.bias.toFixed(2);
 document.getElementById('cfg_accel').innerHTML=kv('Offset (g)',m.imu.av?'['+f(m.imu.ao[0],4)+', '+f(m.imu.ao[1],4)+', '+f(m.imu.ao[2],4)+']':'<span class="tag warn">none</span>')
  +kv('Gyro bias',m.imu.gbv?'['+f(m.imu.gb[0],3)+', '+f(m.imu.gb[1],3)+', '+f(m.imu.gb[2],3)+']':'<span class="tag warn">uncal</span>');
 document.getElementById('cfg_failsafe').innerHTML=kv('Runtime',m.sys.fsb?'<span class="tag err">BYPASSED</span>':'<span class="tag ok">ENABLED</span>')
  +kv('Compiled default',m.sys.fsc?'bypass':'enabled')+kv('Active latch',m.sys.fs?'latched':'clear');
 document.getElementById('cfg_build').textContent='Loop '+m.sys.loop+' Hz · heap '+(m.sys.heap/1024|0)+' KB · telem '+(m.sys.whz||0)+' Hz';
 renderMag(m);
 if(!magTrimLoaded)loadMagTrim();
}
function applyBias(){const v=parseFloat(document.getElementById('cfg_bias').value);if(!(v>=1&&v<=2)){toast('Bias 1.00–2.00',true);return;}put('/api/mix','Bias applied',{bias:v});}
async function saveBias(){await applyBias();act('/api/mix/save','Bias saved to NVS');}
function setFailsafeBypass(b){put('/api/failsafe',b?'Failsafes disabled':'Failsafes enabled',{bypass:b?1:0});}
function saveFailsafeBypass(){act('/api/failsafe/save','Failsafe setting saved to NVS');}
// ---------- compass + mag heading trim ----------
let magTrimLoaded=false,magAcc=0;
function magColour(fld){return fld>90?'#ff5252':(fld>=65?'#ffb020':(fld>=40?'#33d17a':'#5d6678'));}
function renderMag(m){
 const g=m.sen&&m.sen.mag;if(!g)return;
 const ok=g.v===1,hdg=+g.hdg||0,fld=+g.f||0,c=magColour(fld);
 const nd=document.getElementById('mag_needle');if(nd){magAcc+=((hdg-magAcc)%360+540)%360-180;nd.style.transform='rotate('+magAcc+'deg)';nd.style.opacity=ok?'1':'0.22';}
 const ht=document.getElementById('mag_hdg');if(ht)ht.textContent=ok?hdg.toFixed(0)+'°':'--°';
 const ms=document.getElementById('mag_state');if(ms){ms.textContent=ok?'valid':'INVALID (15–95 µT gate)';ms.style.color=ok?'':'#ffb020';}
 const rg=document.getElementById('mag_ring');if(rg)rg.setAttribute('stroke',ok?c:'#5d6678');
 const bar=document.getElementById('mag_bar');if(bar){bar.style.width=Math.max(0,Math.min(100,fld/120*100))+'%';bar.style.background=c;}
 const ft=document.getElementById('mag_field');if(ft)ft.textContent=f(fld,1)+' µT';
}
function loadMagTrim(){magTrimLoaded=true;fetch('/api/settings').then(r=>r.json()).then(j=>{
 if(typeof j.magTrimDeg!=='number')return;const s=document.getElementById('mag_trim'),n=document.getElementById('mag_trimn');
 if(s&&document.activeElement!==s)s.value=j.magTrimDeg;if(n&&document.activeElement!==n)n.value=j.magTrimDeg;}).catch(()=>{});}
function saveMagTrim(){const v=Math.max(-360,Math.min(360,parseInt(document.getElementById('mag_trimn').value,10)||0));
 act('/api/settings','Heading trim saved: '+v+'°',{magTrimDeg:v});}
)DASH"
R"DASH(
// ---------- PID tuning ----------
const PIDF=[['rrP','Rate Roll P',0,5000,25,'Roll rate'],['rrI','Rate Roll I',0,3000,25,'Roll rate'],['rrD','Rate Roll D',0,1000,5,'Roll rate'],
 ['rpP','Rate Pitch P',0,5000,25,'Pitch rate'],['rpI','Rate Pitch I',0,3000,25,'Pitch rate'],['rpD','Rate Pitch D',0,1000,5,'Pitch rate'],
 ['ryP','Rate Yaw P',0,5000,25,'Yaw rate'],['ryI','Rate Yaw I',0,3000,25,'Yaw rate'],['ryD','Rate Yaw D',0,1000,5,'Yaw rate'],
 ['aR','Angle Roll P',0,10000,100,'Angle'],['aP','Angle Pitch P',0,10000,100,'Angle'],['aY','Angle Yaw P',0,10000,100,'Angle']];
let gains=new Array(12).fill(0),pushT=null;
function buildPid(){const groups={};PIDF.forEach((ff,i)=>{(groups[ff[5]]=groups[ff[5]]||[]).push(i);});
 const root=document.getElementById('pidgroups');root.innerHTML='';
 for(const g in groups){const c=document.createElement('div');c.className='card';c.innerHTML='<h3>'+g+'</h3>';
  groups[g].forEach(i=>{const ff=PIDF[i];const r=document.createElement('div');r.className='row';
   r.innerHTML='<label>'+ff[1]+'</label><input type="range" min="'+ff[2]+'" max="'+ff[3]+'" step="'+ff[4]+'" id="pr'+i+'"><div class="v" id="pv'+i+'">--</div>';
   c.appendChild(r);const inp=r.querySelector('input');
   inp.addEventListener('input',e=>{gains[i]=+e.target.value;document.getElementById('pv'+i).textContent=(gains[i]/1000).toFixed(3);pushPid();});});
  root.appendChild(c);}}
function setPidUI(g){PIDF.forEach((ff,i)=>{const r=document.getElementById('pr'+i);if(r&&document.activeElement!==r){r.value=g[i];document.getElementById('pv'+i).textContent=(g[i]/1000).toFixed(3);}});gains=g.slice();}
function pushPid(){clearTimeout(pushT);pushT=setTimeout(()=>put('/api/pid',null,{gains}),140);}
async function loadPid(){try{const r=await fetch('/api/pid');const j=await r.json();if(j.gains&&j.gains.length===12)setPidUI(j.gains);
 const s=document.getElementById('pid_safe');s.textContent=j.safe?'Bench-idle: edits allowed.':'ARMED / throttle ≠ 0 — edits refused.';s.style.color=j.safe?'var(--ok)':'var(--err)';}catch(e){}}
async function savePid(){const r=await fetch('/api/pid/save',{method:'POST',headers:authHdrs()});toast(r.ok?'PID saved to NVS':'Save failed',!r.ok);}
// periodic PID refresh (safety flag + external changes)
setInterval(()=>{if(activeTab==='pid')loadPid();},1500);
// ---------- boot ----------
buildTabs();buildPid();updAuth();loadPid();connectWs();
if(!TOKEN)setTimeout(()=>{if(!TOKEN)setToken();},500);
window.onRender=null;
// fetch cal info once for trim step + initial fields
(async()=>{try{const r=await fetch('/api/cal');if(r.ok){const c=await r.json();window.TRIMSTEP=c.step||0.5;
 const or=document.getElementById('a_offr'),op=document.getElementById('a_offp');if(or&&!or.value)or.value=c.off[0].toFixed(2);if(op&&!op.value)op.value=c.off[1].toFixed(2);}}catch(e){}})();
// stage hooks (vibration/notch/servo/capture wire themselves in if present)
window.onTabShown=function(id){if(window.STAGE&&window.STAGE[id])window.STAGE[id]();};
window.STAGE={};
// ===== Vibration tab =====
const vibeStrip=mkStrip('vibe_strip',[['gx','#ff5252'],['gy','#33d17a'],['gz','#3fa9ff']],300);
const vibeBuf=[[],[],[]];const VIBE_N=100;
function renderVibe(m){
 vibeStrip.push([m.imu.g[0],m.imu.g[1],m.imu.g[2]]);vibeStrip.draw();
 for(let i=0;i<3;i++){vibeBuf[i].push(m.imu.g[i]);if(vibeBuf[i].length>VIBE_N)vibeBuf[i].shift();}
 const stat=arr=>{const n=arr.length;if(!n)return{rms:0,sd:0,pp:0,mean:0};let s=0,s2=0,mn=1e9,mx=-1e9;
  for(const v of arr){s+=v;s2+=v*v;if(v<mn)mn=v;if(v>mx)mx=v;}const mean=s/n;return{rms:Math.sqrt(s2/n),sd:Math.sqrt(Math.max(0,s2/n-mean*mean)),pp:mx-mn,mean};};
 const ax=['Roll X','Pitch Y','Yaw Z'];let h='<table><thead><tr><th>Axis</th><th>RMS</th><th>SD</th><th>P-P</th><th>Mean</th></tr></thead><tbody>';
 for(let i=0;i<3;i++){const s=stat(vibeBuf[i]);h+='<tr><td>'+ax[i]+'</td><td class="mono">'+f(s.rms,2)+'</td><td class="mono">'+f(s.sd,2)+'</td><td class="mono">'+f(s.pp,2)+'</td><td class="mono">'+f(s.mean,2)+'</td></tr>';}
 h+='</tbody></table>';const nt=window.NT&&window.NT.info;
 h+='<div class="statgrid" style="margin-top:8px"><div class="stat"><div class="l">Dom freq</div><div class="n">'+(nt&&nt.done?f(nt.recCenter,0)+' Hz':'—')+'</div></div>'
  +'<div class="stat"><div class="l">Eff rate</div><div class="n">'+(nt&&nt.effHz?f(nt.effHz,0):'—')+'</div></div>'
  +'<div class="stat"><div class="l">Nyquist</div><div class="n">'+(nt&&nt.effHz?f(nt.effHz/2,0):'—')+'</div></div></div>';
 document.getElementById('vibe_stats').innerHTML=h;
}
// ===== Notch / FFT tab =====
window.NT={info:null,raw:null,fin:null,hpb:0,axis:0};
document.querySelectorAll('#nt_axis button').forEach(b=>b.onclick=()=>{
 document.querySelectorAll('#nt_axis button').forEach(x=>x.classList.remove('active'));b.classList.add('active');
 window.NT.axis=+b.dataset.ax;fftFetch();});
async function notchRun(){if(await act('/api/notch/start','Capturing 256 samples…'))window.NT.raw=window.NT.fin=null;}
async function notchPoll(){
 try{const r=await fetch('/api/notch');if(!r.ok)return;const n=await r.json();window.NT.info=n;
  document.getElementById('nt_status').textContent=n.running?('capturing '+n.n+'/256…'):(n.done?('done · '+f(n.effHz,0)+' Hz eff'):'idle');
  document.getElementById('nt_cur').innerHTML=kv('Compiled',n.comp?'<span class="tag ok">yes</span>':'<span class="tag warn">no</span>')
   +kv('Enabled',n.en?'<span class="tag ok">on</span>':'<span class="tag warn">bypassed</span>')
   +kv('Center (live)',f(n.center,1)+' Hz')+kv('Sweep band',f(n.min,0)+'–'+f(n.max,0)+' Hz')+kv('Q',f(n.q,2));
  const eb=document.getElementById('nt_en');if(eb)eb.textContent=n.en?'Disable notch':'Enable notch';
  const conf=n.conf>0.66?'high':(n.conf>0.33?'medium':'low');
  document.getElementById('nt_rec').innerHTML=kv('Center',f(n.recCenter,1)+' Hz')+kv('Q',f(n.recQ,2))
   +kv('Confidence','<span class="tag '+(n.conf>0.5?'ok':'warn')+'">'+conf+' ('+f(n.conf*100,0)+'%)</span>')
   +kv('Peak R/P/Y',f(n.peakHz[0],0)+'/'+f(n.peakHz[1],0)+'/'+f(n.peakHz[2],0)+' Hz')
   +kv('SNR (peak÷floor)',(n.floor[0]>0?f(n.peakMag[0]/n.floor[0],1):'–')+'×');
  if(n.done&&!window.NT.raw)fftFetch();
 }catch(e){}
}
async function fftFetch(){const ax=window.NT.axis;
 try{const[r0,r1]=await Promise.all([fetch('/api/notch/fft?stage=0&axis='+ax),fetch('/api/notch/fft?stage=1&axis='+ax)]);
  if(r0.ok&&r1.ok){const a=await r0.json(),b=await r1.json();window.NT.raw=a.mag;window.NT.fin=b.mag;window.NT.hpb=a.hzPerBin;drawFFT();}}catch(e){}}
function drawFFT(){const cv=document.getElementById('nt_fft');if(!cv)return;const ctx=cv.getContext('2d');
 const W=cv.width=cv.clientWidth,H=cv.height;ctx.clearRect(0,0,W,H);
 const raw=window.NT.raw,fin=window.NT.fin,hpb=window.NT.hpb||1,info=window.NT.info;const pad=30,gw=W-pad-6,gh=H-pad-8;
 if(!raw){ctx.fillStyle='#5d6678';ctx.font='12px sans-serif';ctx.fillText('Run analysis to capture a spectrum.',pad,H/2);return;}
 const nb=raw.length;let mx=1e-6;for(let i=1;i<nb;i++){if(raw[i]>mx)mx=raw[i];if(fin&&fin[i]>mx)mx=fin[i];}
 const fmax=nb*hpb;const X=k=>pad+(k*hpb/fmax)*gw,Y=v=>4+gh-(v/mx)*gh;
 if(info){ctx.fillStyle='#3fa9ff18';const x1=pad+(info.min/fmax)*gw,x2=pad+(info.max/fmax)*gw;ctx.fillRect(x1,4,x2-x1,gh);}
 ctx.strokeStyle='#28303f';ctx.beginPath();ctx.moveTo(pad,4+gh);ctx.lineTo(W,4+gh);ctx.stroke();
 ctx.fillStyle='#5d6678';ctx.font='10px sans-serif';
 for(let hz=0;hz<=fmax;hz+=50){const x=pad+(hz/fmax)*gw;ctx.fillText(hz,x-6,H-3);ctx.strokeStyle='#161c28';ctx.beginPath();ctx.moveTo(x,4);ctx.lineTo(x,4+gh);ctx.stroke();}
 if(fin){ctx.beginPath();ctx.moveTo(X(0),Y(0));for(let k=1;k<nb;k++)ctx.lineTo(X(k),Y(fin[k]));ctx.strokeStyle='#5d6678';ctx.lineWidth=1.2;ctx.stroke();}
 ctx.beginPath();ctx.moveTo(X(0),Y(0));for(let k=1;k<nb;k++)ctx.lineTo(X(k),Y(raw[k]));ctx.strokeStyle='#3fa9ff';ctx.lineWidth=1.6;ctx.stroke();
 if(info&&info.recCenter>0){const x=pad+(info.recCenter/fmax)*gw;ctx.strokeStyle='#ffb020';ctx.setLineDash([4,3]);ctx.beginPath();ctx.moveTo(x,4);ctx.lineTo(x,4+gh);ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#ffb020';ctx.fillText(f(info.recCenter,0)+'Hz',x+2,14);}
}
async function notchToggle(){const n=window.NT.info;if(!n)return;await put('/api/notch/enabled',n.en?'Notch disabled':'Notch enabled',{en:n.en?0:1});}
async function notchApply(){const n=window.NT.info;if(!n||!n.done){toast('Run analysis first',true);return;}await put('/api/notch/apply','Notch applied (temp)',{center:n.recCenter,q:n.recQ});}
function fftExport(){const raw=window.NT.raw,fin=window.NT.fin,hpb=window.NT.hpb||1;if(!raw){toast('No FFT captured',true);return;}
 let csv='hz,raw,final\n';for(let k=0;k<raw.length;k++)csv+=(k*hpb).toFixed(2)+','+raw[k].toFixed(5)+','+((fin&&fin[k])||0).toFixed(5)+'\n';dl(csv,'fft_axis'+window.NT.axis+'.csv');}
function dl(text,name){const b=new Blob([text],{type:'text/csv'});const u=URL.createObjectURL(b);const a=document.createElement('a');a.href=u;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(u),1000);}
let notchTimer=null;
STAGE.notch=()=>{notchPoll();if(!notchTimer)notchTimer=setInterval(()=>{if(activeTab==='notch')notchPoll();else{clearInterval(notchTimer);notchTimer=null;}},1000);};
// ===== Pan/Tilt servo tab =====
let svHoldTimer=null,svTimer=null;
function setIfEmpty(id,v){const e=document.getElementById(id);if(e&&!e.value&&document.activeElement!==e)e.value=v;}
async function svPoll(){try{const r=await fetch('/api/servo');if(!r.ok)return;const s=await r.json();
 const t=document.getElementById('sv_ovtag');t.textContent=s.ov?'WEB':'RC';t.className='tag '+(s.ov?'ok':'');
 document.getElementById('sv_info').innerHTML=kv('Attached',s.att?'<span class="tag ok">yes</span>':'<span class="tag err">no</span>')
  +kv('Pan cur→target',s.pan+' → '+s.panT+' µs')+kv('Tilt cur→target',s.tilt+' → '+s.tiltT+' µs');
 const pr=document.getElementById('sv_pan'),tr=document.getElementById('sv_tilt');
 if(pr){pr.min=Math.min(s.panMin,s.panMax);pr.max=Math.max(s.panMin,s.panMax);if(document.activeElement!==pr)pr.value=s.panT;}
 if(tr){tr.min=Math.min(s.tiltMin,s.tiltMax);tr.max=Math.max(s.tiltMin,s.tiltMax);if(document.activeElement!==tr)tr.value=s.tiltT;}
 document.getElementById('sv_panv').textContent=s.pan;document.getElementById('sv_tiltv').textContent=s.tilt;
 setIfEmpty('sv_pmn',s.panMin);setIfEmpty('sv_pc',s.panC);setIfEmpty('sv_pmx',s.panMax);
 setIfEmpty('sv_tmn',s.tiltMin);setIfEmpty('sv_tc',s.tiltC);setIfEmpty('sv_tmx',s.tiltMax);
 const pi=document.getElementById('sv_pi'),ti=document.getElementById('sv_ti');
 if(pi&&document.activeElement!==pi)pi.checked=s.panInv;if(ti&&document.activeElement!==ti)ti.checked=s.tiltInv;
}catch(e){}}
function svNudge(dp,dt){act('/api/servo/nudge',null,{dpan:dp,dtilt:dt});}
function svStart(dp,dt){svNudge(dp,dt);clearInterval(svHoldTimer);svHoldTimer=setInterval(()=>svNudge(dp,dt),150);}
function svStop(){clearInterval(svHoldTimer);svHoldTimer=null;svPoll();}
function svCenter(p,t){act('/api/servo/center',null,{pan:p,tilt:t});}
let svSlideT=null;
function svSetSlider(){clearTimeout(svSlideT);svSlideT=setTimeout(()=>{const p=+document.getElementById('sv_pan').value,t=+document.getElementById('sv_tilt').value;put('/api/servo',null,{pan:p,tilt:t});},80);}
function svSaveCfg(){const g=id=>+document.getElementById(id).value;
 put('/api/servo/config','Servo config saved to NVS',{panMin:g('sv_pmn'),panCenter:g('sv_pc'),panMax:g('sv_pmx'),tiltMin:g('sv_tmn'),tiltCenter:g('sv_tc'),tiltMax:g('sv_tmx'),panInv:document.getElementById('sv_pi').checked?1:0,tiltInv:document.getElementById('sv_ti').checked?1:0});}
STAGE.servo=()=>{svPoll();if(!svTimer)svTimer=setInterval(()=>{if(activeTab==='servo')svPoll();else{clearInterval(svTimer);svTimer=null;}},700);};
// ===== Diagnostic capture tab =====
let capTimer=null;
async function capPoll(){try{const r=await fetch('/api/capture');if(!r.ok)return;const c=await r.json();
 const state=c.waiting?'waiting':(c.active?'recording':(c.hasData?'stopped':'empty'));
 document.getElementById('cap_state').textContent=state+(c.overflow?' full':'');
 document.getElementById('cap_samples').textContent=(c.samples||0)+' / '+(c.cap||0);
 document.getElementById('cap_rate').textContent=c.effHz?f(c.effHz,1)+' Hz':'--';
 document.getElementById('cap_drop').textContent=c.dropped||0;
 const fld=document.getElementById('cap_fields');if(fld)fld.textContent='CSV schema v2, '+(c.samples||0)+' rows. Buffer '+Math.round(((c.samples||0)/(c.cap||1))*100)+'% used.';
}catch(e){}}
async function capStart(){if(await act('/api/capture/start','Capture started'))capPoll();}
async function capArm(){if(await act('/api/capture/arm','Waiting for arm transition'))capPoll();}
async function capStop(){if(await act('/api/capture/stop','Capture stopped'))capPoll();}
async function capClear(){if(confirm('Clear diagnostic capture?')&&await act('/api/capture/clear','Capture cleared'))capPoll();}
function capDownload(){window.location.href='/api/capture.csv';}
STAGE.capture=()=>{capPoll();if(!capTimer)capTimer=setInterval(()=>{if(activeTab==='capture')capPoll();else{clearInterval(capTimer);capTimer=null;}},500);};
(function(){const prev=window.onRender;window.onRender=m=>{if(prev)prev(m);if(activeTab==='vibe')renderVibe(m);};})();
</script>
</body></html>)DASH";
