#pragma once

// =============================================================================
// esp_larp — presentation dashboard v3 (single PROGMEM page, served at "/").
// -----------------------------------------------------------------------------
// "Engineering console" theme: charcoal header/nav band + light warm-gray work
// surface + WHITE panels carrying DARK instrument faces (avionics-style gauges
// mounted in a light test console). One raw string: HTML + CSS + vanilla JS,
// ZERO external requests. Six hash-routed pages; Overview is a full
// instrument wall (horizon, heading, ToF altitude tape, stick, throttle,
// hero values, compact GPS/mission map, summaries).
//
// DATA-SOURCE MODEL (unchanged): LIVE SENSOR teal / LIVE REMOTE blue /
// PRESENTATION amber / RECORDED violet — top accent strip + badge per panel,
// one legend dialog. ToF (VL53L1X) is a LIVE SENSOR since v3; ToF was removed
// from the presentation provider entirely (disconnected -> shown offline,
// never mock-substituted).
//
// DESIGN SYSTEM v3 (ui-ux-pro-max "Data-Dense", adapted hybrid)
//   band #20272f (header/nav) text #e8edf3 · page #e7e9e4 · panel #fff
//   inset #f4f5f2 · border #d3d7cf · divider #e4e6e0
//   ink #1d242c / #57616c / #6b7480 · accent #0c6b73 (bright face var #31b3c2)
//   ok #27703c · warn #a86f14 · err #b3372f · off #7c848d
//   instrument faces #1d242c with #aeb7c2 ticks
//   panels: 3px top accent in source colour, equal-height (flex), 16-18px pads
//   type: hero 26px mono · values 14px mono · labels 11px caps
//   spacing 4/8/12/16/24 · radius 6/4 · buttons 34px · bp 720/1080 · max 1560px
// =============================================================================

static const char LARP_DASHBOARD_HTML[] PROGMEM = R"LARP(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Team 47 — FCU Telemetry Demonstrator</title>
<style>
:root{--page:#11161c;--panel:#1a2129;--inset:#141a21;--line:#2b3540;--div:#232d37;
--band:#1d242c;--band2:#2a323c;--bandtxt:#e8edf3;--banddim:#9aa5b1;
--ink:#e8edf3;--dim:#a9b3be;--mut:#7f8a95;
--acc:#0f7c86;--accbr:#31b3c2;--accbg:#16333a;
--ok:#3dbd66;--okbr:#43c56d;--warn:#e0a33a;--warnbr:#e0a33a;--warnbg:#33290f;
--err:#ef6a5f;--errbr:#ef6a5f;--off:#7a828c;
--face:#1d242c;--face2:#28313c;--tick:#aeb7c2;
--ssen:#2fb3c2;--srem:#6ea8ff;--spre:#e0a33a;--srec:#a98fd6;--spi:#4fbf8b;
--mono:ui-monospace,'Cascadia Mono',Consolas,Menlo,monospace;
--sans:system-ui,-apple-system,'Segoe UI',Roboto,sans-serif}
*{box-sizing:border-box;margin:0;padding:0}
html{-webkit-text-size-adjust:100%}
body{background:var(--page);color:var(--ink);font:13.5px/1.5 var(--sans)}
button{cursor:pointer;font:inherit;color:inherit}
.pmodes{position:fixed;right:5px;bottom:5px;z-index:99;display:flex;gap:3px;opacity:.22}
.pmodes:hover,.pmodes:focus-within{opacity:.82}
.pmodes button{width:18px;height:18px;padding:0;border:1px solid #56616d;border-radius:2px;
background:#151b22;color:#89939e;font:700 9px/1 var(--mono)}
.pmodes button:hover,.pmodes button:focus{color:#fff;border-color:#aeb7c2}
.mtest-grid{display:grid;grid-template-columns:repeat(4,minmax(110px,1fr));gap:12px}
.mtest-grid button{min-height:76px;border:2px solid #7a2a26;border-radius:6px;
background:#331312;color:#ef6a5f;font:700 18px var(--mono);touch-action:none}
.mtest-grid button:active{background:#6b1e1a;color:#fff}
.mono{font-family:var(--mono);font-variant-numeric:tabular-nums}
/* ---------- header (charcoal band) ---------- */
header{background:var(--band);color:var(--bandtxt);padding:12px 22px;
display:flex;flex-wrap:wrap;align-items:center;gap:8px 22px}
.brand h1{font-size:16px;font-weight:650;letter-spacing:.02em}
.brand .sub{font-size:11.5px;color:var(--banddim)}
.hstat{display:flex;align-items:center;gap:14px;margin-left:auto;flex-wrap:wrap}
.hs{display:flex;align-items:center;gap:6px;font-size:11px;color:var(--banddim);
letter-spacing:.05em;white-space:nowrap}
.dot{width:9px;height:9px;border-radius:50%;background:#4a545f;flex:0 0 9px}
.dot.on{background:var(--okbr)}.dot.warn{background:var(--warnbr)}.dot.err{background:var(--errbr)}
.inhibit{font:650 10.5px/1 var(--sans);letter-spacing:.06em;color:var(--accbr);
border:1px solid #2c5a61;border-radius:4px;padding:6px 9px;white-space:nowrap;background:#1a2e33}
.lgbtn{background:transparent;border:1px solid #3a444f;border-radius:4px;
padding:6px 11px;font-size:11.5px;color:var(--banddim);min-height:30px}
.lgbtn:hover{border-color:var(--accbr);color:var(--accbr)}
main .lgbtn{color:var(--dim);border-color:var(--line)}
main .lgbtn:hover{color:var(--acc);border-color:var(--acc)}
/* ---------- nav (same band) ---------- */
nav{background:var(--band);border-bottom:3px solid var(--band2);padding:0 14px;
display:flex;gap:2px;overflow-x:auto;position:sticky;top:0;z-index:20}
nav button{background:none;border:none;border-bottom:3px solid transparent;margin-bottom:-3px;
padding:12px 16px;font-size:13.5px;color:var(--banddim);white-space:nowrap;min-height:46px}
nav button:hover{color:var(--bandtxt)}
nav button.act{color:#fff;border-bottom-color:var(--accbr);font-weight:600}
nav button:focus-visible,button:focus-visible{outline:2px solid var(--accbr);outline-offset:-2px}
/* ---------- arm banner ---------- */
#armbn{display:none;background:var(--warnbg);border-bottom:1px solid #5a4416;color:var(--warn);
padding:9px 20px;font:650 12.5px/1.4 var(--sans);text-align:center}
#armbn.show{display:block}
/* ---------- layout ---------- */
main{max-width:1560px;margin:0 auto;padding:18px 22px 44px}
.page{display:none}.page.act{display:block}
.grid{display:grid;gap:16px;align-items:stretch}
.g2{grid-template-columns:1fr 1fr}
.g3{grid-template-columns:1fr 1fr 1fr}
.g32{grid-template-columns:3fr 2fr}
.stack{display:flex;flex-direction:column;gap:16px}
.stack>.panel{flex:1;margin-bottom:0}
.panel{background:var(--panel);border:1px solid var(--line);border-top:3px solid var(--off);
border-radius:6px;margin-bottom:16px;box-shadow:0 2px 6px rgba(0,0,0,.28);
display:flex;flex-direction:column}
.grid>.panel{margin-bottom:0}
.panel.p-sen{border-top-color:var(--ssen)}.panel.p-rem{border-top-color:var(--srem)}
.panel.p-pre{border-top-color:var(--spre)}.panel.p-rec{border-top-color:var(--srec)}
.ph{display:flex;align-items:center;gap:10px;padding:10px 16px;border-bottom:1px solid var(--div)}
.ph h2{font-size:12px;font-weight:700;letter-spacing:.06em;text-transform:uppercase;color:var(--dim)}
.ph .sp{margin-left:auto;display:flex;gap:10px;align-items:center;flex-wrap:wrap;justify-content:flex-end}
.pb{padding:16px 18px;flex:1}
/* source badges */
.src{display:inline-flex;align-items:center;gap:5px;font-size:10.5px;font-weight:600;
letter-spacing:.03em;color:var(--mut);white-space:nowrap}
.src i{width:8px;height:8px;border-radius:50%;display:inline-block}
.src.sen i{background:var(--ssen)}.src.rem i{background:var(--srem)}
.src.pre i{background:var(--spre)}.src.rec i{background:var(--srec)}.src.pi i{background:var(--spi)}
.panel.p-pi{border-top-color:var(--spi)}
/* plant scan grid */
.pgrid{display:grid;grid-template-columns:repeat(6,1fr);gap:4px;max-width:420px}
.pcell{aspect-ratio:1;border-radius:3px;display:flex;flex-direction:column;
align-items:center;justify-content:center;gap:1px;font:700 12px/1 var(--mono);
color:#0d1524;background:var(--inset);border:1px solid var(--div)}
.pcell .pcf{font:500 9.5px/1 var(--mono);opacity:.85}
.pcell.h{background:#3dbd66}.pcell.p{background:#e0a33a}.pcell.r{background:#d9614a;color:#fff}
.pcell.off{background:var(--inset);color:var(--off);border-style:dashed}
.pgrid.mini{gap:3px;width:138px;max-width:138px}
.pgrid.mini .pcell{border-radius:2px}
.pgrid.mini .pcl,.pgrid.mini .pcf{display:none}
.povr{display:inline-block;font:700 14px/1 var(--mono);padding:6px 12px;border-radius:4px;
background:var(--inset);color:var(--off);border:1px solid var(--line)}
.povr.h{background:#16351f;color:#43c56d;border-color:#2e7d43}
.povr.p{background:#33290f;color:#e0a33a;border-color:#7a5a1a}
.povr.r{background:#331312;color:#ef6a5f;border-color:#7a2a26}
.povr.u{background:#2f2c12;color:#e0d33a;border-color:#6a6320}
.pleg{display:flex;flex-wrap:wrap;gap:10px;margin-top:10px;font-size:11.5px;color:var(--dim)}
.pleg span{display:inline-flex;align-items:center;gap:6px}
.pleg i{width:11px;height:11px;border-radius:2px;display:inline-block}
/* key-value rows */
.kv{display:flex;justify-content:space-between;align-items:baseline;gap:10px;padding:6px 0}
.kv+.kv{border-top:1px solid var(--div)}
.kv .k{color:var(--dim)}
.kv .v{font-family:var(--mono);font-variant-numeric:tabular-nums;font-size:14px;text-align:right}
.kv .v .u{color:var(--mut);font-size:11px;margin-left:2px}
.good{color:var(--ok)}.warnc{color:var(--warn)}.bad{color:var(--err)}.offc{color:var(--off)}
/* hero numbers */
.heros{display:grid;grid-template-columns:repeat(auto-fit,minmax(118px,1fr));gap:12px}
.hero{background:var(--inset);border:1px solid var(--div);border-radius:5px;padding:10px 12px}
.hero .l{font-size:10.5px;letter-spacing:.06em;text-transform:uppercase;color:var(--mut);font-weight:600}
.hero .n{font:650 26px/1.25 var(--mono);font-variant-numeric:tabular-nums}
.hero .n .u{font-size:13px;color:var(--mut);font-weight:400}
/* instrument wall */
.instwall{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:20px;align-items:center}
.instwall .icell{max-width:330px;margin:0 auto;width:100%}
.inst{display:grid;grid-template-columns:300px 300px 140px 1fr;gap:22px;align-items:start}
.inst svg,.instwall svg{width:100%;height:auto;display:block}
.offline{filter:grayscale(1);opacity:.45}
.corner{font:650 10px/1 var(--sans);letter-spacing:.05em;color:var(--off);
border:1px solid var(--line);border-radius:3px;padding:4px 7px;background:var(--inset)}
.ilbl{text-align:center;font:600 10.5px/1 var(--sans);letter-spacing:.06em;color:var(--mut);
text-transform:uppercase;margin-top:6px}
.ival{text-align:center;font:600 13px var(--mono);margin-top:3px}
/* buttons */
.btn{background:var(--inset);border:1px solid var(--line);border-radius:4px;color:var(--dim);
padding:0 15px;height:34px;font-size:12.5px;transition:border-color .15s,background .15s}
.btn:hover{border-color:var(--accbr);color:var(--accbr)}
.btn.pri{background:var(--acc);border-color:var(--acc);color:#fff}
.btn.pri:hover{background:#0d6b74;color:#fff}
.btn.warnb{color:var(--warn)}.btn.warnb:hover{border-color:var(--warn)}
.btn:disabled{color:var(--mut);border-color:var(--div);background:var(--inset);cursor:not-allowed}
.btnrow{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
/* meters */
.meter{display:flex;align-items:center;gap:8px;margin:6px 0}
.meter .ml{width:7ch;font:11px var(--mono);color:var(--dim)}
.meter .tr{flex:1;height:9px;background:var(--inset);border:1px solid var(--div);border-radius:4px;overflow:hidden}
.meter .tr i{display:block;height:100%;background:var(--acc);width:0;transition:width .25s linear}
.meter .mv{width:5ch;font:11px var(--mono);color:var(--dim);text-align:right}
.note{color:var(--dim);font-size:12.5px;margin:7px 0}
.statline{font:12.5px var(--mono);margin-top:10px;min-height:1.3em}
canvas.ch{width:100%;height:170px;display:block}
canvas.chbig{width:100%;height:230px;display:block}
/* remote widgets */
.rcwrap{display:grid;grid-template-columns:110px 230px 1fr;gap:24px;align-items:start}
.thrwrap{text-align:center}
.sw{display:inline-flex;align-items:center;gap:6px;border:1px solid var(--line);border-radius:4px;
padding:6px 10px;font-size:11.5px;font-weight:600;color:var(--mut);background:var(--inset)}
.sw.on{color:#0d1524;border-color:var(--srem);background:var(--srem)}
.cbar{display:flex;align-items:center;gap:8px;margin:5px 0}
.cbar .cl{width:9ch;font-size:11.5px;color:var(--dim)}
.cbar .tr{flex:1;height:11px;background:var(--inset);border:1px solid var(--div);border-radius:4px;position:relative}
.cbar .tr i{position:absolute;top:0;bottom:0;background:var(--srem)}
.cbar .cv{width:9ch;font:12px var(--mono);text-align:right}
/* tables */
table{width:100%;border-collapse:collapse;font-size:12.5px}
th{font-size:10.5px;letter-spacing:.05em;text-transform:uppercase;color:var(--mut);
text-align:left;padding:7px 8px;border-bottom:1px solid var(--line)}
td{padding:7px 8px;border-bottom:1px solid var(--div);font-family:var(--mono);font-variant-numeric:tabular-nums}
td .btn{height:27px;padding:0 8px;font-size:11px}
/* maps */
.mapframe{background:var(--inset);border:1px solid var(--line);border-radius:5px;overflow:hidden}
#map{width:100%;height:auto;display:block;cursor:crosshair;touch-action:none}
#omap{width:100%;height:auto;display:block}
.maptag{font:650 10.5px/1 var(--sans);letter-spacing:.05em;color:var(--warn)}
/* dialog */
#ovl{display:none;position:fixed;inset:0;background:rgba(29,36,44,.55);z-index:50;
align-items:flex-start;justify-content:center;padding:40px 16px}
#ovl.show{display:flex}
#dlg{background:var(--panel);border:1px solid var(--line);border-radius:6px;max-width:660px;
width:100%;max-height:82vh;overflow:auto;padding:22px}
#dlg h3{font-size:15px;margin-bottom:10px}
#dlg h4{font-size:12px;text-transform:uppercase;letter-spacing:.05em;color:var(--dim);margin:15px 0 5px}
#dlg p,#dlg li{font-size:12.5px;color:var(--dim)}
#dlg ul{padding-left:18px}
/* responsive */
@media(max-width:1080px){.g3{grid-template-columns:1fr 1fr}.g32{grid-template-columns:1fr}
.inst{grid-template-columns:1fr 1fr}.inst>div:nth-child(3){max-width:140px}
.inst>div:last-child{grid-column:1/-1}}
@media(max-width:720px){main{padding:12px 10px 34px}.g2,.g3{grid-template-columns:1fr}
.inst{grid-template-columns:1fr}.inst>div{max-width:280px;margin:0 auto;width:100%}
.rcwrap{grid-template-columns:1fr;justify-items:center}.rcwrap>div:last-child{width:100%}
.hstat{gap:9px}header{padding:10px 12px}.hero .n{font-size:21px}
.instwall{grid-template-columns:repeat(auto-fit,minmax(140px,1fr))}}
@media(prefers-reduced-motion:reduce){*{transition:none!important}}
</style></head><body>
<header>
 <div class="brand">
  <h1>CAPSTONE TEAM 47 · FCU TELEMETRY DEMONSTRATOR</h1>
  <div class="sub">Ed Lumley Centre for Engineering Innovation · ESP_LARP presentation build</div>
 </div>
 <div class="hstat">
  <span class="hs"><i class="dot" id="d_imu"></i>IMU</span>
  <span class="hs"><i class="dot" id="d_mag"></i>MAG</span>
  <span class="hs"><i class="dot" id="d_tof"></i>TOF</span>
  <span class="hs"><i class="dot" id="d_rc"></i>REMOTE</span>
  <span class="hs"><i class="dot" id="d_ws"></i><span id="h_cli">LINK</span></span>
  <span class="inhibit">FLIGHT PROPULSION INHIBITED</span>
  <button class="lgbtn" onclick="ovl(1)">Data sources</button>
 </div>
</header>
<div id="armbn">ARM REQUEST RECEIVED — FLIGHT PROPULSION INHIBITED</div>
<nav id="nav"></nav>
<main>

<!-- ================= OVERVIEW ================= -->
<section class="page" id="pg-overview">
 <div class="panel p-sen">
  <div class="ph"><h2>Flight instruments</h2>
   <span class="sp"><span class="src sen"><i></i>Live Sensor</span><span class="src rem"><i></i>Live Remote</span></span></div>
  <div class="pb">
   <div class="instwall">
    <div class="icell"><div data-horizon="o_"></div><div class="ilbl">Attitude</div></div>
    <div class="icell"><div data-heading="o_"></div><div class="ilbl">Heading</div></div>
    <div class="icell" style="max-width:165px"><div data-alt="o_"></div><div class="ilbl">ToF altitude</div><div class="ival" id="o_altnum2">--</div></div>
    <div class="icell" style="max-width:260px"><div data-stick="o_"></div><div class="ilbl">Stick</div><div class="ival" id="o_stickval">--</div></div>
    <div class="icell" style="max-width:140px"><div data-thr="o_"></div><div class="ilbl">Throttle</div><div class="ival" id="o_thrval">--</div></div>
   </div>
   <div class="heros" style="margin-top:16px">
    <div class="hero"><div class="l">Roll</div><div class="n"><span id="o_roll">--</span><span class="u">&#176;</span></div></div>
    <div class="hero"><div class="l">Pitch</div><div class="n"><span id="o_pitch">--</span><span class="u">&#176;</span></div></div>
    <div class="hero"><div class="l">Heading</div><div class="n"><span id="o_hdg">--</span><span class="u">&#176;</span></div></div>
    <div class="hero"><div class="l">Rel yaw</div><div class="n"><span id="o_ryaw">--</span><span class="u">&#176;</span></div></div>
    <div class="hero"><div class="l">ToF alt</div><div class="n"><span id="o_tofa">--</span><span class="u">m</span></div></div>
    <div class="hero"><div class="l">|Gyro|</div><div class="n"><span id="o_gmag">--</span><span class="u">dps</span></div></div>
   </div>
  </div>
 </div>
 <div class="grid g32">
  <div class="panel p-pre">
   <div class="ph"><h2>Position &amp; mission</h2>
    <span class="sp"><span class="maptag">PREVIEW ONLY</span><span class="src pre"><i></i>Presentation GPS</span></span></div>
   <div class="pb">
    <div class="mapframe"><svg id="omap" viewBox="0 0 800 600" aria-label="Venue map overview"></svg></div>
    <div class="kv" style="margin-top:10px"><span class="k">GPS (model) / waypoints</span><span class="v" id="o_gps">--</span></div>
   </div>
  </div>
  <div class="stack">
   <div class="panel p-rem">
    <div class="ph"><h2>Remote link</h2><span class="sp"><span class="src rem"><i></i>Live Remote</span></span></div>
    <div class="pb">
     <div class="kv"><span class="k">nRF24 link</span><span class="v" id="o_rc">--</span></div>
     <div class="kv"><span class="k">Arm switch</span><span class="v" id="o_arm">--</span></div>
     <div class="kv"><span class="k">Packet age / rate</span><span class="v" id="o_page">--</span></div>
    </div>
   </div>
   <div class="panel">
    <div class="ph"><h2>System</h2><span class="sp"><span class="src sen"><i></i>Measured</span></span></div>
    <div class="pb">
     <div class="kv"><span class="k">Status</span><span class="v" id="o_ready">CONNECTING&#8230;</span></div>
     <div class="kv"><span class="k">Flight propulsion</span><span class="v good">INHIBITED · TEST PAGE SEPARATE</span></div>
     <div class="kv"><span class="k">Uptime / clients</span><span class="v" id="o_up">--</span></div>
     <div class="kv"><span class="k">Sensors / IMU+MAG</span><span class="v" id="o_sens">--</span></div>
    </div>
   </div>
   <div class="panel p-pre">
    <div class="ph"><h2>Aux telemetry</h2><span class="sp"><span class="src pre"><i></i>Presentation</span></span></div>
    <div class="pb">
     <div class="kv"><span class="k">Battery</span><span class="v" id="o_bat">--</span></div>
     <div class="kv"><span class="k">Baro altitude</span><span class="v" id="o_alt">--</span></div>
     <div class="kv"><span class="k">RC link quality (model)</span><span class="v" id="o_lq">--</span></div>
    </div>
   </div>
  </div>
 </div>
 <div class="panel p-pi">
  <div class="ph"><h2>Plant scan — Raspberry Pi</h2>
   <span class="sp"><button class="lgbtn" onclick="nav('plant')">Open Plant page</button><span class="src pi"><i></i>Live Pi</span></span></div>
  <div class="pb" style="display:grid;grid-template-columns:auto 1fr;gap:24px;align-items:center">
   <div class="pgrid mini" id="o_pgrid"></div>
   <div>
    <div class="kv"><span class="k">Link</span><span class="v" id="o_pi_link">--</span></div>
    <div class="kv"><span class="k">Overall</span><span class="v"><span class="povr off" id="o_pi_ov">--</span></span></div>
    <div class="kv"><span class="k">Patches G/Y/R</span><span class="v" id="o_pi_gyr">--</span></div>
    <div class="kv"><span class="k">Scan # / age</span><span class="v" id="o_pi_scan">--</span></div>
   </div>
  </div>
 </div>
</section>

<!-- ================= PLANT ================= -->
<section class="page" id="pg-plant">
 <div class="panel p-pi"><div class="ph"><h2>Plant classification — Raspberry Pi</h2>
  <span class="sp"><span class="corner" id="pl_off" style="display:none">PI LINK DOWN</span><span class="src pi"><i></i>Live Pi</span></span></div>
  <div class="pb grid g32">
   <div>
    <div class="pgrid" id="pl_grid"></div>
    <div class="pleg">
     <span><i style="background:#3dbd66"></i>healthy</span>
     <span><i style="background:#e0a33a"></i>pest attack</span>
     <span><i style="background:#d9614a"></i>rust</span>
     <span><i style="background:var(--inset);border:1px dashed var(--line)"></i>no data</span>
    </div>
   </div>
   <div>
    <div class="kv"><span class="k">Link</span><span class="v" id="pl_link">--</span></div>
    <div class="kv"><span class="k">Overall state</span><span class="v"><span class="povr off" id="pl_ov">--</span></span></div>
    <div class="kv"><span class="k">Patches G/Y/R</span><span class="v" id="pl_gyr">--</span></div>
    <div class="kv"><span class="k">Scan seq / total</span><span class="v" id="pl_seq">--</span></div>
    <div class="kv"><span class="k">Last update / bad lines</span><span class="v" id="pl_age">--</span></div>
    <div class="meter" style="margin-top:10px"><span class="ml">HEALTHY</span><div class="tr"><i id="pl_ah" style="background:#3dbd66"></i></div><span class="mv" id="pl_ah_t">--</span></div>
    <div class="meter"><span class="ml">PEST</span><div class="tr"><i id="pl_ap" style="background:#e0a33a"></i></div><span class="mv" id="pl_ap_t">--</span></div>
    <div class="meter"><span class="ml">RUST</span><div class="tr"><i id="pl_ar" style="background:#d9614a"></i></div><span class="mv" id="pl_ar_t">--</span></div>
    <p class="note">Live from the Raspberry Pi ONNX leaf classifier over UART (Serial2, GPIO16).
    Each cell is one of the 36 image patches — colour is the predicted class, the number is the
    raw model confidence (uncapped). Overall state is the Pi's own average-probability decision
    (average rust &#8805; 70% or average pest &#8805; 80% &#8594; unhealthy). Classification and the
    overall call are made entirely on the Pi; the FCU only displays them.</p>
   </div>
  </div>
 </div>
</section>

<!-- ================= MOTORS ================= -->
<section class="page" id="pg-motors">
 <div class="panel"><div class="ph"><h2>Individual motor test</h2>
  <span class="sp"><span class="corner">DSHOT300 · ONE MOTOR ONLY · 50% HARD CAP</span></span></div>
  <div class="pb">
   <p class="note bad">Secure the airframe, clear the area, and use eye protection.
   First commissioning should be performed with propellers removed.</p>
   <div class="kv"><span class="k">Controller</span><span class="v" id="mt_ready">--</span></div>
   <div class="kv"><span class="k">Active output</span><span class="v" id="mt_active">STOPPED</span></div>
   <div class="kv"><span class="k">Signal pins</span><span class="v">M1 25 · M2 26 · M3 27 · M4 32</span></div>
   <div style="margin:14px 0">
    <label><input type="checkbox" id="mt_ack"> Airframe restrained, area clear, remote arm switch off</label>
    <button class="btn warnb" style="margin-left:12px" onclick="motorEnable(this)">Enable for 30 seconds</button>
   </div>
   <label for="mt_thr">Throttle <strong id="mt_thr_v">10%</strong></label>
   <input id="mt_thr" type="range" min="1" max="50" value="10" style="width:100%;margin:8px 0 18px"
    oninput="$('mt_thr_v').textContent=this.value+'%'">
   <div class="mtest-grid">
    <button onpointerdown="motorHoldStart(event,1)">HOLD M1</button>
    <button onpointerdown="motorHoldStart(event,2)">HOLD M2</button>
    <button onpointerdown="motorHoldStart(event,3)">HOLD M3</button>
    <button onpointerdown="motorHoldStart(event,4)">HOLD M4</button>
   </div>
   <button class="btn warnb" style="width:100%;margin-top:16px" onclick="motorStop()">STOP ALL</button>
  </div>
 </div>
</section>

<!-- ================= ATTITUDE ================= -->
<section class="page" id="pg-attitude">
 <div class="panel p-sen"><div class="ph"><h2>Attitude — physical airframe</h2>
  <span class="sp"><span class="corner" id="a_off" style="display:none">IMU OFFLINE</span><span class="src sen"><i></i>Live Sensor</span></span></div>
  <div class="pb">
   <div class="inst">
    <div><div data-horizon="a_"></div></div>
    <div><div data-heading="a_"></div></div>
    <div><div data-alt="a_"></div><div class="ival" id="a_altnum2">--</div></div>
    <div>
     <div class="heros">
      <div class="hero"><div class="l">Roll</div><div class="n"><span id="a_roll">--</span><span class="u">&#176;</span></div></div>
      <div class="hero"><div class="l">Pitch</div><div class="n"><span id="a_pitch">--</span><span class="u">&#176;</span></div></div>
      <div class="hero"><div class="l">Heading</div><div class="n"><span id="a_hdg">--</span><span class="u">&#176;</span></div></div>
      <div class="hero"><div class="l">Rel yaw</div><div class="n"><span id="a_ryaw">--</span><span class="u">&#176;</span></div></div>
      <div class="hero"><div class="l">|Accel|</div><div class="n"><span id="a_amag">--</span><span class="u">g</span></div></div>
      <div class="hero"><div class="l">ToF alt</div><div class="n"><span id="a_tofa">--</span><span class="u">m</span></div></div>
     </div>
     <div class="kv" style="margin-top:12px"><span class="k">Fusion</span><span class="v" id="a_fuse">--</span></div>
     <div class="kv"><span class="k">Sample age / loop rate</span><span class="v" id="a_age">--</span></div>
     <div class="btnrow" style="margin-top:12px">
      <button class="btn pri" onclick="api('/api/orientation/zero',this)">Set current orientation as zero</button>
      <span class="src sen" id="a_zero"><i></i>zero not set</span>
     </div>
    </div>
   </div>
  </div>
 </div>
 <div class="grid g2">
  <div class="panel p-sen"><div class="ph"><h2>IMU calibration — ICM-20948</h2><span class="sp"><span class="src sen"><i></i>Live</span></span></div>
   <div class="pb">
    <div class="kv"><span class="k">Sensor</span><span class="v" id="c_imu_st">--</span></div>
    <div class="kv"><span class="k">State</span><span class="v" id="c_imu_cal">--</span></div>
    <div class="meter"><span class="ml">PROG</span><div class="tr"><i id="p_imu"></i></div><span class="mv" id="p_imu_t">0%</span></div>
    <p class="note">Place the airframe still and level, then start. Motion aborts the capture.</p>
    <div class="btnrow">
     <button class="btn pri" onclick="api('/api/calibrate/imu',this)">Calibrate IMU</button>
     <button class="btn warnb" onclick="api('/api/calibration/clear?t=imu',this)">Clear stored</button>
    </div>
   </div>
  </div>
  <div class="panel p-sen"><div class="ph"><h2>Magnetometer calibration — MMC5603</h2><span class="sp"><span class="src sen"><i></i>Live</span></span></div>
   <div class="pb">
    <div class="kv"><span class="k">Sensor</span><span class="v" id="c_mag_st">--</span></div>
    <div class="kv"><span class="k">State</span><span class="v" id="c_mag_cal">--</span></div>
    <div class="meter"><span class="ml">X</span><div class="tr"><i id="p_mx"></i></div><span class="mv" id="p_mx_t">0%</span></div>
    <div class="meter"><span class="ml">Y</span><div class="tr"><i id="p_my"></i></div><span class="mv" id="p_my_t">0%</span></div>
    <div class="meter"><span class="ml">Z</span><div class="tr"><i id="p_mz"></i></div><span class="mv" id="p_mz_t">0%</span></div>
    <div class="meter"><span class="ml">SAMPLES</span><div class="tr"><i id="p_ms"></i></div><span class="mv" id="p_ms_t">0%</span></div>
    <p class="note">Start, then rotate the airframe slowly through roll, pitch and yaw until all meters fill, then finish.</p>
    <div class="btnrow">
     <button class="btn pri" onclick="api('/api/calibrate/mag/start',this)">Start</button>
     <button class="btn" onclick="api('/api/calibrate/mag/stop',this)">Finish</button>
     <button class="btn warnb" onclick="api('/api/calibration/clear?t=mag',this)">Clear stored</button>
    </div>
   </div>
  </div>
 </div>
 <div class="statline" id="calmsg"></div>
</section>

<!-- ================= REMOTE ================= -->
<section class="page" id="pg-remote">
 <div class="panel p-rem"><div class="ph"><h2>nRF24L01 link</h2><span class="sp"><span class="src rem"><i></i>Live Remote</span></span></div>
  <div class="pb grid g3" style="gap:4px 26px">
   <div class="kv"><span class="k">Module</span><span class="v" id="r_mod">--</span></div>
   <div class="kv"><span class="k">Remote link</span><span class="v" id="r_link">--</span></div>
   <div class="kv"><span class="k">Packet age</span><span class="v" id="r_age">--</span></div>
   <div class="kv"><span class="k">Packets / s</span><span class="v" id="r_pps">--</span></div>
   <div class="kv"><span class="k">Total / dropped (est.)</span><span class="v" id="r_tot">--</span></div>
   <div class="kv"><span class="k">Sequence</span><span class="v" id="r_seq">--</span></div>
   <div class="kv"><span class="k">Carrier detect (RPD)</span><span class="v" id="r_rpd">--</span></div>
   <div class="kv"><span class="k">Channel / rate</span><span class="v">76 · 250 kbps</span></div>
   <div class="kv"><span class="k">RSSI</span><span class="v offc">n/a on nRF24L01</span></div>
  </div>
 </div>
 <div class="panel p-rem"><div class="ph"><h2>Control inputs — display only</h2>
  <span class="sp"><span class="corner" id="r_stale" style="display:none">STALE — VALUES NEUTRALIZED</span><span class="src rem"><i></i>Live Remote</span></span></div>
  <div class="pb rcwrap">
   <div class="thrwrap"><div data-thr="r_"></div><div class="ival" id="r_thrval">THR --%</div></div>
   <div><div data-stick="r_"></div><div class="ival" id="r_stickval">X -- · Y --</div></div>
   <div>
    <div class="cbar"><span class="cl">Throttle</span><div class="tr"><i id="cb_thr"></i></div><span class="cv" id="cv_thr">--</span></div>
    <div class="cbar"><span class="cl">Stick X</span><div class="tr"><i id="cb_sx"></i></div><span class="cv" id="cv_sx">--</span></div>
    <div class="cbar"><span class="cl">Stick Y</span><div class="tr"><i id="cb_sy"></i></div><span class="cv" id="cv_sy">--</span></div>
    <div class="btnrow" style="margin:14px 0 6px">
     <span class="sw" id="sw_arm">ARM REQ</span>
     <span class="sw" id="sw_pid">PID MODE</span>
     <span class="sw" id="sw_btn">BUTTON</span>
     <span class="sw" id="sw_hold">THR HOLD</span>
     <span class="sw" id="sw_atk">AUTO-TKF</span>
     <span class="sw" id="sw_aut">AUTONOMY</span>
    </div>
    <div class="kv"><span class="k">Mode byte / takeoff alt</span><span class="v" id="r_mode">--</span></div>
    <p class="note">The transmitter protocol carries throttle, one X/Y stick pair and switch
    flags (no separate yaw channel). The arm switch is informational only in this build:
    an arm request displays a banner and nothing else — no arming path exists in the binary.</p>
   </div>
  </div>
 </div>
</section>

<!-- ================= SENSORS & FFT ================= -->
<section class="page" id="pg-sensors">
 <div class="panel p-sen"><div class="ph"><h2>Sensor mounting orientation</h2>
  <span class="sp"><span class="src sen"><i></i>Saved on ESP32</span></span></div>
  <div class="pb grid g2">
   <div>
    <div class="note">IMU — rotates accelerometer and gyroscope together</div>
    <div class="btnrow" style="margin-top:8px">
     <label>X <select class="btn mrot" id="m_ix"></select></label>
     <label>Y <select class="btn mrot" id="m_iy"></select></label>
     <label>Z <select class="btn mrot" id="m_iz"></select></label>
     <button class="btn pri" onclick="mountSave('imu',this)">Apply IMU</button>
    </div>
   </div>
   <div>
    <div class="note">Magnetometer — rotates the calibrated magnetic vector</div>
    <div class="btnrow" style="margin-top:8px">
     <label>X <select class="btn mrot" id="m_mx"></select></label>
     <label>Y <select class="btn mrot" id="m_my"></select></label>
     <label>Z <select class="btn mrot" id="m_mz"></select></label>
     <button class="btn pri" onclick="mountSave('mag',this)">Apply MAG</button>
    </div>
   </div>
   <p class="note" style="grid-column:1/-1">Rotations are applied in X, then Y, then Z order.
   After changing IMU orientation, set the current orientation as zero. Recalibrate if offsets no longer look correct.</p>
   <div class="statline" id="mountmsg" style="grid-column:1/-1">Loading saved orientation…</div>
  </div>
 </div>
 <div class="panel p-sen"><div class="ph"><h2>IMU &amp; ToF time series</h2>
  <span class="sp">
   <select id="chwin" class="btn" style="height:30px;padding:0 6px"><option value="10">10 s</option><option value="20" selected>20 s</option><option value="30">30 s</option></select>
   <button class="btn" id="chpause" onclick="togglePause()">Pause</button>
   <button class="btn" onclick="clearHist()">Clear</button>
   <span class="src sen"><i></i>Live Sensor</span></span></div>
  <div class="pb grid g2">
   <div><div class="note">Attitude (roll / pitch / rel-yaw, &#176;)</div><canvas class="ch" id="ch_att"></canvas></div>
   <div><div class="note">Gyro (X / Y / Z, dps)</div><canvas class="ch" id="ch_gyr"></canvas></div>
   <div><div class="note">Accel (X / Y / Z, g) + |a|</div><canvas class="ch" id="ch_acc"></canvas></div>
   <div><div class="note">ToF altitude (m)</div><canvas class="ch" id="ch_alt"></canvas></div>
   <div><div class="note">Die temperature (&#176;C)</div><canvas class="ch" id="ch_tmp"></canvas></div>
  </div>
 </div>
 <div class="panel p-sen"><div class="ph"><h2>Live IMU vibration spectrum (accelerometer)</h2>
  <span class="sp">
   <select id="fftaxis" class="btn" style="height:30px;padding:0 6px"><option value="x">Accel X</option><option value="y">Accel Y</option><option value="z" selected>Accel Z</option></select>
   <span class="src sen"><i></i>Live Sensor</span></span></div>
  <div class="pb">
   <canvas class="chbig" id="ch_fft"></canvas>
   <div class="statline" id="fftinfo">waiting for samples…</div>
  </div>
 </div>
 <div class="panel p-rec"><div class="ph"><h2>Motor vibration spectra</h2>
  <span class="sp"><span class="src rec"><i></i>Recorded Result</span></span></div>
  <div class="pb">
   <div class="btnrow" id="mtrsel" style="margin-bottom:10px"></div>
   <canvas class="chbig" id="ch_mfft"></canvas>
   <div class="grid g2" style="margin-top:12px">
    <div><table id="mpk"><thead><tr><th>Peak</th><th>Freq</th><th>Magnitude</th></tr></thead><tbody></tbody></table></div>
    <div><div class="note">RMS comparison (relative)</div><div id="mrms"></div></div>
   </div>
   <p class="note" id="msrc"></p>
  </div>
 </div>
</section>

<!-- ================= MISSION ================= -->
<section class="page" id="pg-mission">
 <div class="panel p-pre"><div class="ph"><h2>Mission preview</h2>
  <span class="sp"><span class="maptag">NAVIGATION EXECUTION DISABLED</span><span class="src pre"><i></i>Presentation</span></span></div>
  <div class="pb grid g32">
   <div>
    <div class="mapframe"><svg id="map" viewBox="0 0 800 600" aria-label="Offline schematic venue map"></svg></div>
    <div class="btnrow" style="margin-top:10px">
     <button class="btn" onclick="mapZoom(1.5)">Zoom +</button>
     <button class="btn" onclick="mapZoom(1/1.5)">Zoom &#8722;</button>
     <button class="btn" onclick="mapCenter()">Centre map</button>
     <span id="lyrsel"><button class="btn pri" data-l="venue" onclick="setLayer('venue')">Venue</button>
     <button class="btn" data-l="city" onclick="setLayer('city')">Windsor city</button></span>
     <span class="note">Click the map to add a waypoint · drag to pan</span>
    </div>
   </div>
   <div>
    <div class="btnrow" style="margin-bottom:10px">
     <button class="btn pri" onclick="wpAdd()">Add waypoint</button>
     <button class="btn" onclick="wpDefault()">Load demo mission</button>
     <button class="btn" onclick="wpExport(this)">Copy coordinates</button>
     <button class="btn warnb" onclick="wpClear()">Clear</button>
    </div>
    <table id="wpt"><thead><tr><th>#</th><th>Lat</th><th>Lon</th><th>Dist</th><th>Brg</th><th>Alt m</th><th></th></tr></thead><tbody></tbody></table>
    <div class="kv" style="margin-top:10px"><span class="k">Waypoints / route length</span><span class="v" id="wp_tot">0 / 0 m</span></div>
    <p class="note">Interface preview only: this build contains no navigation controller,
    no mission executor and no motor outputs. The vehicle marker is the presentation GPS
    source near the configured venue centre.</p>
   </div>
  </div>
 </div>
</section>

<!-- ================= SYSTEM ================= -->
<section class="page" id="pg-system">
 <div class="grid g2">
  <div class="panel"><div class="ph"><h2>Firmware</h2></div><div class="pb">
   <div class="kv"><span class="k">Environment</span><span class="v">esp_larp</span></div>
   <div class="kv"><span class="k">Board target</span><span class="v">esp32dev (classic ESP32)</span></div>
   <div class="kv"><span class="k">Build</span><span class="v" id="y_build">--</span></div>
   <div class="kv"><span class="k">ESP-IDF</span><span class="v" id="y_sdk">--</span></div>
   <div class="kv"><span class="k">Reset reason</span><span class="v" id="y_rst">--</span></div>
  </div></div>
  <div class="panel"><div class="ph"><h2>Network</h2></div><div class="pb">
   <div class="kv"><span class="k">SSID</span><span class="v">capstone_team_47</span></div>
   <div class="kv"><span class="k">AP address / channel</span><span class="v" id="y_ap">--</span></div>
   <div class="kv"><span class="k">Clients (AP / WS)</span><span class="v" id="y_cli">--</span></div>
   <div class="kv"><span class="k">Telemetry sent</span><span class="v" id="y_tx">--</span></div>
   <div class="kv"><span class="k">WS connects since boot</span><span class="v" id="y_wsc">--</span></div>
  </div></div>
  <div class="panel p-sen"><div class="ph"><h2>Sensors</h2><span class="sp"><span class="src sen"><i></i>Live Sensor</span></span></div><div class="pb">
   <div class="kv"><span class="k">IMU ICM-20948</span><span class="v" id="y_imu">--</span></div>
   <div class="kv"><span class="k">IMU calibration</span><span class="v" id="y_imucal">--</span></div>
   <div class="kv"><span class="k">Mag MMC5603</span><span class="v" id="y_mag">--</span></div>
   <div class="kv"><span class="k">Mag calibration</span><span class="v" id="y_magcal">--</span></div>
   <div class="kv"><span class="k">ToF VL53L1X</span><span class="v" id="y_tof">--</span></div>
   <div class="kv"><span class="k">Sample ages (IMU/mag/ToF)</span><span class="v" id="y_ages">--</span></div>
   <div class="kv"><span class="k">Gyro / accel / temp</span><span class="v" id="y_raw" style="min-width:0">--</span></div>
  </div></div>
  <div class="panel p-rem"><div class="ph"><h2>Remote</h2><span class="sp"><span class="src rem"><i></i>Live Remote</span></span></div><div class="pb">
   <div class="kv"><span class="k">nRF24 module</span><span class="v" id="y_nrf">--</span></div>
   <div class="kv"><span class="k">Packet age / rate</span><span class="v" id="y_rcage">--</span></div>
   <div class="kv"><span class="k">Loss estimate</span><span class="v" id="y_rcloss">--</span></div>
   <div class="kv"><span class="k">Last values</span><span class="v" id="y_rcv" style="min-width:0">--</span></div>
  </div></div>
  <div class="panel"><div class="ph"><h2>Resources</h2></div><div class="pb">
   <div class="kv"><span class="k">Uptime</span><span class="v" id="y_up">--</span></div>
   <div class="kv"><span class="k">Free heap / minimum</span><span class="v" id="y_heap">--</span></div>
   <div class="kv"><span class="k">Sensor loop / web push</span><span class="v" id="y_hz">--</span></div>
  </div></div>
  <div class="panel"><div class="ph"><h2>Safety &amp; data sources</h2></div><div class="pb">
   <div class="kv"><span class="k">Motor / ESC / DShot code</span><span class="v good">NOT IN BINARY</span></div>
   <div class="kv"><span class="k">Arming</span><span class="v good">INHIBITED (display-only input)</span></div>
   <div class="kv"><span class="k">Navigation execution</span><span class="v good">DISABLED (preview only)</span></div>
   <div class="kv"><span class="k">Motor GPIO</span><span class="v good">NEVER CONFIGURED</span></div>
   <p class="note" style="margin-top:10px">Sources: attitude, IMU/mag/ToF values and calibration are
   measured on physically connected sensors (Live Sensor). Remote inputs are received from the
   physical nRF24 handheld (Live Remote). GPS, barometric altitude, battery and RC-link-quality
   figures are a deterministic presentation source — that hardware is not attached to this board.
   Motor spectra are recorded bench results (2026-05-16). Uptime, heap, client counts and loop
   rates are always real measurements. Full legend: <button class="lgbtn" onclick="ovl(1)">Data sources</button></p>
  </div></div>
 </div>
</section>
</main>

<!-- data-sources dialog -->
<div id="ovl" onclick="if(event.target===this)ovl(0)">
 <div id="dlg" role="dialog" aria-label="Data sources">
  <h3>Data sources in this demonstrator</h3>
  <p>Every panel carries one of four source marks (also its top border colour):</p>
  <h4><span class="src sen"><i></i>Live Sensor</span></h4>
  <ul><li>ICM-20948 IMU, MMC5603 magnetometer and VL53L1X time-of-flight rangefinder, physically connected over I2C.</li>
  <li>Roll, pitch, heading, relative yaw, ToF altitude, vibration spectrum: computed on-board from those sensors only. Never substituted.</li></ul>
  <h4><span class="src rem"><i></i>Live Remote</span></h4>
  <ul><li>nRF24L01 receiver decoding the project's original handheld transmitter packets (throttle, stick X/Y, switches, sequence).</li>
  <li>Display-only: the arm switch shows a banner; no arming or motor path exists in this build.</li></ul>
  <h4><span class="src pre"><i></i>Presentation Data</span></h4>
  <ul><li>GPS position/satellites/HDOP, barometric altitude, battery voltage/current/percent, RC link-quality model, mission vehicle marker.</li>
  <li>Deterministic smooth generator for hardware not attached to this board. Not sensor readings.</li></ul>
  <h4><span class="src pi"><i></i>Live Pi</span></h4>
  <ul><li>Raspberry Pi plant classifier over UART: per-patch class (healthy / pest / rust) and raw
  model confidence for a 6&#215;6 grid, plus the Pi's overall average-probability decision.</li>
  <li>Classification runs entirely on the Pi (ONNX); the FCU validates the line and displays it.</li></ul>
  <h4><span class="src rec"><i></i>Recorded Result</span></h4>
  <ul><li>Motor vibration spectra: real bench sweep captured 2026-05-16 on the original airframe
  (tools/motor_fft, gyro-Y axis, motors at DShot 1100). Peak tables are from the original test reports.</li></ul>
  <p style="margin-top:10px">System metrics (uptime, heap, clients, rates) are always real. Propulsion,
  arming and navigation execution are excluded from this firmware image entirely.</p>
  <div class="btnrow" style="margin-top:12px"><button class="btn pri" onclick="ovl(0)">Close</button></div>
 </div>
</div>

<script>
'use strict';
/* ================= embedded recorded dataset (REAL bench data) ============= */
var MFFT={source:"tools/motor_fft sweep_motor*_20260516 bench sweep",axis:"gyro-Y",dshot:1100,
rate:1000,fftSize:4096,f0:1.953,df:3.90625,
motors:[
{m:1,mags:[6.4,7.4,8.8,9.8,8.0,10.0,9.0,6.8,7.0,8.3,6.9,6.2,9.7,6.4,11.8,4.9,7.5,14.7,5.5,7.5,13.5,6.6,5.6,6.3,8.1,9.5,7.9,9.7,6.8,7.9,9.4,8.3,229.4,8.0,6.7,8.9,5.1,4.2,5.8,8.1,6.7,8.0,18.7,6.1,6.2,5.7,3.7,14.8,6.5,4.0,7.2,3.5,5.0,5.2,4.9,8.2,4.4,24.2,6.1,5.8,4.9,6.1,7.5,7.8,7.9,15.6,4.2,4.7,7.8,5.6,13.0,4.7,5.9,6.4,3.4,5.1,3.4,5.1,3.3,5.1,6.3,4.6,4.3,3.1,5.5,5.9,2.7,3.6,5.9,3.2,3.2,3.5,6.5,4.6,5.4,47.8,4.4,6.4,4.3,4.0,9.0,4.1,5.6,2.7,3.5,3.2,3.2,13.3,3.2,3.0,8.3,3.5,3.6,3.6,3.3,6.3,3.0,5.1,7.5,3.4,4.2,4.4,31.8,5.5,3.7,7.5,3.9,4.5],peaks:[[124.5,1835.5],[365.6,328.0],[470.3,201.4]]},
{m:2,mags:[8.6,6.2,4.5,6.9,7.5,7.4,7.0,8.0,7.2,8.9,8.5,6.9,7.9,7.5,7.6,8.6,8.9,9.1,13.0,15.6,5.6,8.2,9.5,7.1,7.6,10.0,10.1,7.2,8.1,9.5,7.5,9.0,6.8,170.1,9.4,7.1,4.6,6.2,7.6,6.0,5.3,5.3,13.2,6.8,7.5,6.7,5.1,5.0,14.1,6.9,7.3,6.9,4.4,3.5,5.4,6.5,6.4,21.7,7.2,7.0,4.8,7.3,4.6,5.2,5.2,5.9,11.1,4.7,3.3,8.5,18.0,3.5,8.8,5.3,3.7,4.1,4.3,5.1,5.6,4.5,5.0,4.4,4.9,4.5,5.1,3.9,4.0,2.8,4.8,3.9,3.8,3.8,5.5,8.4,46.5,15.5,6.0,4.0,3.3,5.7,5.1,4.7,6.5,4.6,4.5,4.1,4.1,3.9,9.5,6.5,4.1,4.5,3.2,3.0,4.6,3.3,4.4,7.0,2.8,3.2,3.9,2.7,5.7,38.4,10.3,4.8,3.6,3.7],peaks:[[128.5,1550.2],[367.5,157.1]]},
{m:3,mags:[9.3,12.1,17.1,14.8,24.6,32.7,12.6,8.3,8.9,15.5,13.2,10.0,11.2,7.3,10.5,5.5,13.0,21.0,9.3,11.9,18.3,11.3,10.6,7.5,11.6,12.0,9.5,19.1,6.8,10.5,13.4,25.7,455.3,23.9,14.7,15.7,8.5,10.8,7.6,14.0,7.3,9.5,36.8,12.1,6.1,8.7,16.1,25.3,17.6,8.5,13.8,8.3,6.2,10.8,12.0,13.2,12.0,54.9,27.9,10.5,6.0,8.1,11.2,11.6,10.0,15.1,11.9,10.3,10.6,18.4,33.2,12.7,8.9,13.3,7.6,7.6,6.0,8.7,8.6,6.6,9.5,6.8,7.5,7.7,9.5,12.0,7.4,8.9,11.1,8.1,10.0,12.6,14.1,11.9,40.3,110.2,39.7,14.4,12.1,8.2,13.1,7.4,14.3,10.9,8.8,6.2,7.8,28.6,8.6,6.5,15.6,7.5,5.9,5.1,7.4,12.3,6.7,9.3,19.1,8.2,8.0,14.0,77.8,32.4,13.0,15.1,9.9,9.4],peaks:[[126.7,3347.2],[372.3,365.5]]},
{m:4,mags:[7.5,7.6,7.0,6.6,10.1,6.7,7.5,7.8,10.0,5.6,7.0,7.3,7.7,8.0,7.4,7.3,8.5,7.0,8.2,8.8,7.3,9.2,5.7,8.4,6.6,7.1,6.3,6.4,6.2,5.1,6.1,6.6,6.1,122.8,7.2,6.5,6.2,6.6,6.9,5.8,5.1,8.0,11.9,6.1,5.2,6.0,5.3,6.3,7.9,4.8,3.6,5.8,5.7,4.8,5.6,6.7,7.6,20.0,5.5,5.3,3.8,5.9,7.1,5.2,5.4,5.2,7.0,3.6,6.0,3.4,6.7,5.6,6.7,3.6,5.0,6.3,3.2,4.6,3.7,4.3,4.1,3.5,4.3,4.1,4.0,5.7,3.8,5.1,3.1,3.3,3.9,4.2,3.8,6.1,35.4,9.7,4.5,3.0,2.2,5.2,4.8,3.1,4.2,3.0,2.7,3.4,2.8,3.6,8.5,4.3,3.5,3.3,2.3,2.6,2.8,4.6,3.1,5.5,2.7,2.4,2.1,4.7,2.4,22.4,5.1,4.3,3.8,3.5],peaks:[[127.8,1055.6],[364.1,142.8]]}]};
/* Map configuration — REAL offline OpenStreetMap imagery embedded in flash
   (see include/esp_larp_map_assets.h; (c) OpenStreetMap contributors, ODbL —
   attribution rendered on every map). Bounds are exact stitched-tile edges.
   HOME = approximate Ed Lumley Centre coordinates (single change point). */
var HOME={lat:42.304924,lon:-83.062016};
var MAPS={
 venue:{img:'/map/venue.jpg',latN:42.309815,latS:42.297627,lonW:-83.078613,lonE:-83.056641,label:'Venue (CEI area)'},
 city:{img:'/map/city.jpg',latN:42.358544,latS:42.261049,lonW:-83.188477,lonE:-83.012695,label:'Windsor city'}};
var MAP_LAYER='venue';
var MAP_W=800,MAP_H=600;
/* Mission state must exist before the router's first show() (the overview
   and mission pages draw the map during cold load). */
var MISSION=[];
try{MISSION=JSON.parse(localStorage.getItem('larp.mission.v1')||'[]')}catch(e){MISSION=[]}
var VB={x:0,y:0,w:MAP_W,h:MAP_H};

var $=function(i){return document.getElementById(i)};
var PAGES=[['overview','Overview'],['attitude','Attitude'],['remote','Remote'],
['sensors','Sensors & FFT'],['plant','Plant'],['motors','Motors'],
['mission','Mission'],['system','System']];
var T={d:null,paused:false,lastMsg:0};
var H={t:[],rr:[],rp:[],ry:[],gx:[],gy:[],gz:[],ax:[],ay:[],az:[],am:[],tp:[],alt:[]};
var HMAX=400, MCOL=['#2fb3c2','#6ea8ff','#e0a33a','#a98fd6'];

/* ================= instrument factory (dark faces, shared markup) ========= */
function instHorizon(p){return '<svg viewBox="-100 -100 200 200" role="img" aria-label="Artificial horizon">'+
'<defs><clipPath id="'+p+'hc"><circle r="84"/></clipPath></defs>'+
'<circle r="92" fill="#28313c"/>'+
'<g clip-path="url(#'+p+'hc)"><g id="'+p+'horz">'+
'<rect x="-300" y="-500" width="600" height="500" fill="#3d6d99"/>'+
'<rect x="-300" y="0" width="600" height="500" fill="#7a5a33"/>'+
'<line x1="-300" x2="300" y1="0" y2="0" stroke="#fff" stroke-width="1.6"/>'+
'<g stroke="#eef2f6" stroke-width=".9" font-size="8.5" fill="#eef2f6" text-anchor="middle">'+
'<line x1="-22" x2="22" y1="-28" y2="-28"/><text x="31" y="-25.5">10</text><text x="-31" y="-25.5">10</text>'+
'<line x1="-12" x2="12" y1="-14" y2="-14"/>'+
'<line x1="-22" x2="22" y1="28" y2="28"/><text x="31" y="30.5">10</text><text x="-31" y="30.5">10</text>'+
'<line x1="-12" x2="12" y1="14" y2="14"/>'+
'<line x1="-28" x2="28" y1="-56" y2="-56"/><text x="37" y="-53.5">20</text><text x="-37" y="-53.5">20</text>'+
'<line x1="-28" x2="28" y1="56" y2="56"/><text x="37" y="58.5">20</text><text x="-37" y="58.5">20</text>'+
'</g></g></g>'+
'<circle r="84" fill="none" stroke="#39434f" stroke-width="2.5"/>'+
'<g id="'+p+'rollarc"><path d="M0,-84 l-4.5,9 h9 z" fill="#aeb7c2"/></g>'+
'<g stroke="#31b3c2" stroke-width="2.8" fill="none">'+
'<path d="M-42,0 h26 l7,7"/><path d="M42,0 h-26 l-7,7"/><circle r="2.1" fill="#31b3c2" stroke="none"/></g></svg>'}
function instHeading(p){
 var ticks='';for(var d=0;d<360;d+=30){ticks+='<line x1="0" y1="-84" x2="0" y2="-75" stroke="#aeb7c2" transform="rotate('+d+')"/>'}
 return '<svg viewBox="-100 -100 200 200" role="img" aria-label="Heading indicator">'+
'<circle r="92" fill="#28313c"/><circle r="84" fill="#1d242c" stroke="#39434f"/>'+
'<g id="'+p+'hdgring" fill="#aeb7c2" font-size="11" text-anchor="middle" font-family="ui-monospace,monospace">'+ticks+
'<text y="-64">N</text><text x="66" y="4">E</text><text y="72">S</text><text x="-66" y="4">W</text></g>'+
'<path d="M0,-84 l-5,-9 h10 z" fill="#31b3c2"/>'+
'<g stroke="#31b3c2" stroke-width="2.4" fill="none"><path d="M0,-26 V26 M-10,8 L0,26 L10,8"/></g>'+
'<text id="'+p+'hdgnum" text-anchor="middle" y="-36" fill="#fff" font-size="16" font-family="ui-monospace,monospace">---</text>'+
'<text id="'+p+'hdglbl" text-anchor="middle" y="48" fill="#8d99a6" font-size="9.5">HEADING (MAG)</text></svg>'}
function instAlt(p){
 var s='<svg viewBox="0 0 96 208" role="img" aria-label="ToF altitude tape">'+
'<rect x="2" y="2" width="92" height="204" rx="5" fill="#28313c"/>'+
'<rect x="8" y="12" width="56" height="184" rx="3" fill="#1d242c" stroke="#39434f"/>';
 for(var m=0;m<=4;m++){var y=188-m*42;
  s+='<line x1="56" x2="64" y1="'+y+'" y2="'+y+'" stroke="#aeb7c2"/>'+
     '<text x="70" y="'+(y+3)+'" font-size="9.5" fill="#aeb7c2" font-family="ui-monospace,monospace">'+m+'</text>';
  if(m<4){var ym=y-21;s+='<line x1="59" x2="64" y1="'+ym+'" y2="'+ym+'" stroke="#5c6874"/>'}}
 s+='<g id="'+p+'altptr" transform="translate(0,188)">'+
'<path d="M12,0 h38" stroke="#31b3c2" stroke-width="2.6"/>'+
'<path d="M50,0 l9,-5.5 v11 z" fill="#31b3c2"/></g>'+
'<text x="36" y="9.5" text-anchor="middle" font-size="8.5" fill="#8d99a6">TOF ALT m</text></svg>';
 return s}
function instStick(p){return '<svg viewBox="-110 -110 220 220" role="img" aria-label="Stick position">'+
'<rect x="-104" y="-104" width="208" height="208" rx="10" fill="#28313c"/>'+
'<rect x="-96" y="-96" width="192" height="192" rx="7" fill="#1d242c" stroke="#39434f"/>'+
'<line x1="-96" x2="96" y1="0" y2="0" stroke="#333d48"/>'+
'<line x1="0" x2="0" y1="-96" y2="96" stroke="#333d48"/>'+
'<circle r="38" fill="none" stroke="#333d48"/><circle r="76" fill="none" stroke="#333d48"/>'+
'<circle id="'+p+'stickdot" r="9" fill="#5b8fe8"/></svg>'}
function instThr(p){return '<svg viewBox="0 0 64 208" role="img" aria-label="Throttle">'+
'<rect x="2" y="2" width="60" height="204" rx="5" fill="#28313c"/>'+
'<rect x="10" y="12" width="30" height="184" rx="3" fill="#1d242c" stroke="#39434f"/>'+
'<rect id="'+p+'thrfill" x="11" y="195" width="28" height="0" fill="#5b8fe8"/>'+
'<g stroke="#5c6874">'+
'<line x1="42" x2="48" y1="196" y2="196"/><line x1="42" x2="48" y1="150" y2="150"/>'+
'<line x1="42" x2="48" y1="104" y2="104"/><line x1="42" x2="48" y1="58" y2="58"/>'+
'<line x1="42" x2="48" y1="12" y2="12"/></g>'+
'<text x="52" y="199" font-size="9" fill="#aeb7c2" font-family="ui-monospace,monospace">0</text>'+
'<text x="50" y="16" font-size="9" fill="#aeb7c2" font-family="ui-monospace,monospace">100</text></svg>'}
(function buildInstruments(){
 document.querySelectorAll('[data-horizon]').forEach(function(e){e.innerHTML=instHorizon(e.getAttribute('data-horizon'))});
 document.querySelectorAll('[data-heading]').forEach(function(e){e.innerHTML=instHeading(e.getAttribute('data-heading'))});
 document.querySelectorAll('[data-alt]').forEach(function(e){e.innerHTML=instAlt(e.getAttribute('data-alt'))});
 document.querySelectorAll('[data-stick]').forEach(function(e){e.innerHTML=instStick(e.getAttribute('data-stick'))});
 document.querySelectorAll('[data-thr]').forEach(function(e){e.innerHTML=instThr(e.getAttribute('data-thr'))});
})();

/* ---- plant-scan 6x6 grids (built once, cells reused) ---- */
function buildPGrid(id){
 var g=$(id);if(!g||g.childElementCount)return;
 var s='';for(var i=0;i<36;i++){s+='<div class="pcell off"><span class="pcl">--</span><span class="pcf"></span></div>'}
 g.innerHTML=s;
}
buildPGrid('o_pgrid');buildPGrid('pl_grid');
var PCLS={'0':['h','H'],'1':['p','P'],'2':['r','R']};
var POVR={'H':['h','HEALTHY'],'P':['p','UNHEALTHY · PEST'],'R':['r','UNHEALTHY · RUST'],'U':['u','UNCERTAIN']};
// Fill a grid from a plant block. up=false -> all cells shown as "no data".
function fillPGrid(id,P,up,withText){
 var g=$(id);if(!g)return;
 var cells=g.children;
 for(var i=0;i<36;i++){
  var cell=cells[i];
  if(!up||!P.cls||P.cls.length<36){cell.className='pcell off';
   if(withText){cell.children[0].textContent='--';cell.children[1].textContent=''}continue}
  var m=PCLS[P.cls.charAt(i)]||['off','?'];
  cell.className='pcell '+m[0];
  if(withText){
   var cf=parseInt(P.cf.substr(i*2,2),16);if(!isFinite(cf))cf=0;
   cell.children[0].textContent=m[1];
   cell.children[1].textContent=cf+'%';
  }
 }
}

/* ================= router ================= */
function curPage(){var h=location.hash.replace('#/','');return PAGES.some(function(p){return p[0]===h})?h:null}
function nav(p){location.hash='#/'+p}
function show(p){
 PAGES.forEach(function(pp){
  $('pg-'+pp[0]).classList.toggle('act',pp[0]===p);
  $('nv-'+pp[0]).classList.toggle('act',pp[0]===p);
 });
 try{localStorage.setItem('larp.page',p)}catch(e){}
 sizeCanvases(); renderAll();
 if(p==='mission')missionDraw();
 if(p==='overview')overviewMapDraw();
}
window.addEventListener('hashchange',function(){var p=curPage();if(p)show(p)});
(function initNav(){
 var n=$('nav');
 PAGES.forEach(function(p){
  var b=document.createElement('button');b.id='nv-'+p[0];b.textContent=p[1];
  b.onclick=function(){nav(p[0])};n.appendChild(b);
 });
 var start=curPage();
 if(!start){try{start=localStorage.getItem('larp.page')}catch(e){}}
 if(!PAGES.some(function(p){return p[0]===start}))start='overview';
 if(curPage()!==start){history.replaceState(null,'','#/'+start)}
 show(start);
})();
function ovl(on){$('ovl').classList.toggle('show',!!on)}

/* ================= websocket ================= */
var ws=null;
function connect(){
 ws=new WebSocket('ws://'+location.host+'/ws');
 ws.onopen=function(){setDot('d_ws','on');$('h_cli').textContent='LINK'};
 ws.onclose=function(){setDot('d_ws','err');setTimeout(connect,1500)};
 ws.onerror=function(){try{ws.close()}catch(e){}};
 ws.onmessage=function(ev){
  var d;try{d=JSON.parse(ev.data)}catch(e){return}
  T.d=d;T.lastMsg=Date.now();
  if(!T.paused)pushHist(d);
  dirty=true;
 };
}
function setDot(id,cls){$(id).className='dot '+(cls||'')}
function pushHist(d){
 var a=d.real.att,im=d.real.imu,tf=d.real.tof,now=performance.now()/1000;
 H.t.push(now);H.rr.push(a.rr);H.rp.push(a.rp);H.ry.push(a.ry);
 H.gx.push(im.gx);H.gy.push(im.gy);H.gz.push(im.gz);
 H.ax.push(im.ax);H.ay.push(im.ay);H.az.push(im.az);
 H.am.push(Math.sqrt(im.ax*im.ax+im.ay*im.ay+im.az*im.az));H.tp.push(im.t);
 H.alt.push((tf&&tf.ok&&tf.valid)?tf.mm/1000:null);
 if(H.t.length>HMAX){for(var k in H)H[k].shift()}
}
function clearHist(){for(var k in H)H[k].length=0;dirty=true}
function togglePause(){T.paused=!T.paused;$('chpause').textContent=T.paused?'Resume':'Pause'}

/* ================= formatting ================= */
function f(x,d){return(x===undefined||x===null||!isFinite(x))?'--':x.toFixed(d===undefined?1:d)}
function upfmt(s){var h=Math.floor(s/3600),m=Math.floor(s%3600/60),ss=s%60;
 return(h<10?'0':'')+h+':'+(m<10?'0':'')+m+':'+(ss<10?'0':'')+ss}
function setv(id,txt,cls){var e=$(id);e.textContent=txt;if(cls!==undefined)e.className='v '+cls}

/* ================= shared instrument updaters ================= */
var dirty=false, lastMapMs=0, tgt={r:0,p:0,h:0},cur={r:0,p:0,h:0};
function updHdgText(p,at){
 $(p+'hdgnum').textContent=(at.hv?f(at.hdg,0):f((at.ry+360)%360,0))+'°';
 $(p+'hdglbl').textContent=at.hv?'HEADING (MAG)':'REL YAW (GYRO)';
}
function updAlt(p,tof){
 var ok=tof&&tof.ok, val=ok&&tof.valid;
 var m=val?tof.mm/1000:0;
 var y=188-Math.max(0,Math.min(4,m))*42;
 $(p+'altptr').setAttribute('transform','translate(0,'+y.toFixed(1)+')');
}
function updThrStick(p,live,thr,sx,sy){
 var fh=Math.max(0,Math.min(100,thr))*1.84;
 var e=$(p+'thrfill');e.setAttribute('height',fh.toFixed(1));e.setAttribute('y',(196-fh).toFixed(1));
 $(p+'stickdot').setAttribute('cx',sx*0.76);$(p+'stickdot').setAttribute('cy',-sy*0.76);
}
function tofText(tof){
 if(!tof||!tof.ok)return['OFFLINE','offc'];
 if(!tof.valid)return['OUT OF RANGE','warnc'];
 return[(tof.mm/1000).toFixed(2),''];
}

/* ================= main render ================= */
function renderAll(){
 if(!T.d)return;
 var d=T.d,R=d.real,S=d.sim,Y=d.sys,im=R.imu,mg=R.mag,at=R.att,rc=R.rc,tf=R.tof;
 setDot('d_imu',im.ok?'on':'err');setDot('d_mag',mg.ok?'on':'err');
 setDot('d_tof',tf.ok?'on':'err');
 setDot('d_rc',rc.det?(rc.link?'on':'warn'):'err');
 $('h_cli').textContent=Y.cli+' CLIENT'+(Y.cli===1?'':'S');
 $('armbn').classList.toggle('show',!!(rc.link&&rc.arm));
 if(at.valid&&!at.stale){tgt.r=at.rr;tgt.p=at.rp}
 tgt.h=at.hv?at.hdg:((at.ry+360)%360);
 var pg=curPage();
 var ttx=tofText(tf);
 var live=rc.link, sx=live?rc.sx:0, sy=live?rc.sy:0, thr=live?rc.thr:0;
 if(pg==='overview'){
  updHdgText('o_',at);updAlt('o_',tf);updThrStick('o_',live,thr,sx,sy);
  $('o_altnum2').textContent=ttx[0];
  $('o_stickval').textContent=live?('X '+sx+' · Y '+sy):'X -- · Y --';
  $('o_thrval').textContent=live?thr+' %':'--';
  $('o_roll').textContent=f(at.rr);$('o_pitch').textContent=f(at.rp);
  $('o_hdg').textContent=at.hv?f(at.hdg,0):'N/A';$('o_ryaw').textContent=f(at.ry);
  $('o_tofa').textContent=(tf.ok&&tf.valid)?(tf.mm/1000).toFixed(2):'--';
  $('o_gmag').textContent=f(at.mot,0);
  setv('o_ready',im.ok?'READY':'DEGRADED — IMU OFFLINE',im.ok?'good':'warnc');
  setv('o_up',upfmt(Y.up)+' · '+Y.cli+' cli');
  setv('o_sens',(im.ok?'IMU OK':'IMU OFF')+' · '+(mg.ok?'MAG OK':'MAG OFF'),
       im.ok&&mg.ok?'good':(im.ok?'warnc':'bad'));
  setv('o_rc',rc.det?(rc.link?'CONNECTED':'NO PACKETS'):'MODULE ABSENT',
       rc.link?'good':(rc.det?'warnc':'offc'));
  setv('o_arm',rc.link?(rc.arm?'ARM REQUEST (INHIBITED)':'OFF'):'--',rc.arm&&rc.link?'warnc':'');
  setv('o_page',rc.link?(rc.age+' ms · '+rc.pps+'/s'):'--');
  setv('o_gps',(S.fix===3?'3D':'--')+' · '+S.sats+' sats · '+MISSION.length+' wp');
  setv('o_bat',f(S.v,2)+' V · '+S.pct+' %');
  setv('o_alt',f(S.baro,2)+' m');
  setv('o_lq',S.lq+' %');
  // Plant scan (Raspberry Pi) mini summary.
  var P=R.plant,pu=P.up;
  fillPGrid('o_pgrid',P,pu,false);
  setv('o_pi_link',pu?'CONNECTED':(P.n>0?'STALE':'WAITING'),pu?'good':(P.n>0?'warnc':'offc'));
  var ov=POVR[P.ov]||['off','--'];
  var oe=$('o_pi_ov');oe.textContent=pu?ov[1]:'--';oe.className='povr '+(pu?ov[0]:'off');
  setv('o_pi_gyr',pu?(P.g+' / '+P.y+' / '+P.r):'--');
  setv('o_pi_scan',pu?('#'+P.seq+' · '+(P.age>=999999?'--':P.age+' ms')):'--');
  // Map redraw throttled to ~1 Hz: the base <image> is a large raster and
  // the only moving overlay (presentation-GPS marker) drifts slowly.
  if(Date.now()-lastMapMs>1000){lastMapMs=Date.now();overviewMapDraw()}
 }
 else if(pg==='attitude'){
  var offl=!at.valid||at.stale;
  document.querySelectorAll('#pg-attitude [data-horizon],#pg-attitude [data-heading]').forEach(function(e){
   e.classList.toggle('offline',offl)});
  var tofOff=!tf.ok;
  document.querySelectorAll('#pg-attitude [data-alt]').forEach(function(e){
   e.classList.toggle('offline',tofOff)});
  $('a_off').style.display=offl?'':'none';
  updHdgText('a_',at);updAlt('a_',tf);
  $('a_altnum2').textContent=ttx[0];
  $('a_roll').textContent=f(at.rr);$('a_pitch').textContent=f(at.rp);
  $('a_hdg').textContent=at.hv?f(at.hdg,0):'N/A';$('a_ryaw').textContent=f(at.ry);
  $('a_amag').textContent=H.am.length?f(H.am[H.am.length-1],2):'--';
  $('a_tofa').textContent=(tf.ok&&tf.valid)?(tf.mm/1000).toFixed(2):'--';
  setv('a_fuse',!at.valid?'NO IMU':at.stale?'STALE':(at.magT?'IMU + MAG':'IMU ONLY (heading unavailable)'),
       !at.valid||at.stale?'bad':(at.magT?'good':'warnc'));
  setv('a_age',(im.ok?im.age+' ms':'--')+' · '+R.hz+' Hz');
  $('a_zero').innerHTML='<i></i>'+(at.zero?'zero set':'zero not set');
  setv('c_imu_st',im.ok?('OK @ 0x'+im.addr.toString(16).toUpperCase()):'DISCONNECTED',im.ok?'good':'bad');
  setv('c_imu_cal',im.cal+(im.nvs?' (stored)':''),im.cal==='CALIBRATED'?'good':'warnc');
  $('p_imu').style.width=im.calPct+'%';$('p_imu_t').textContent=im.calPct+'%';
  setv('c_mag_st',mg.ok?('OK @ 0x'+mg.addr.toString(16).toUpperCase()):'DISCONNECTED',mg.ok?'good':'bad');
  setv('c_mag_cal',mg.cal+(mg.nvs?' (stored)':''),mg.cal==='CALIBRATED'?'good':'warnc');
  $('p_mx').style.width=mg.cx+'%';$('p_mx_t').textContent=mg.cx+'%';
  $('p_my').style.width=mg.cy+'%';$('p_my_t').textContent=mg.cy+'%';
  $('p_mz').style.width=mg.cz+'%';$('p_mz_t').textContent=mg.cz+'%';
  $('p_ms').style.width=mg.sp+'%';$('p_ms_t').textContent=mg.sp+'%';
  var cm=$('calmsg');
  if(im.cal==='KEEP STILL'){cm.textContent='IMU calibrating — keep the airframe perfectly still…';cm.className='statline warnc'}
  else if(im.cal==='MOTION DETECTED'){cm.textContent='IMU calibration rejected: motion detected. Retry when still.';cm.className='statline bad'}
  else if(mg.cal==='ROTATE DRONE'){cm.textContent='Magnetometer capturing — rotate slowly through roll, pitch and yaw.';cm.className='statline warnc'}
  else if(mg.cal==='CAL REJECTED'){cm.textContent='Magnetometer calibration rejected: insufficient rotation coverage.';cm.className='statline bad'}
 }
 else if(pg==='remote'){
  setv('r_mod',rc.det?'DETECTED':'NOT DETECTED',rc.det?'good':'bad');
  setv('r_link',rc.det?(rc.link?'CONNECTED':'NO PACKETS'):'--',rc.link?'good':'warnc');
  setv('r_age',rc.tot>0?(rc.age>=99999?'>99 s':rc.age+' ms'):'--',rc.link?'':'warnc');
  setv('r_pps',rc.pps+' /s');
  setv('r_tot',rc.tot+' / '+rc.drop);
  setv('r_seq',''+rc.seq);
  setv('r_rpd',rc.rpd?'CARRIER':'quiet',rc.rpd?'good':'offc');
  $('r_stale').style.display=live?'none':'';
  updThrStick('r_',live,thr,sx,sy);
  $('r_thrval').textContent='THR '+(live?thr+'%':'--');
  $('r_stickval').textContent=live?('X '+sx+' · Y '+sy):'X -- · Y --';
  cbar('cb_thr','cv_thr',thr,0,100,live?thr+' %':'--');
  cbar('cb_sx','cv_sx',sx,-100,100,live?sx+' %':'--');
  cbar('cb_sy','cv_sy',sy,-100,100,live?sy+' %':'--');
  swst('sw_arm',live&&rc.arm);swst('sw_pid',live&&rc.pidm);swst('sw_btn',live&&rc.btn);
  swst('sw_hold',live&&rc.hold);swst('sw_atk',live&&rc.atk);swst('sw_aut',live&&rc.aut);
  $('sw_arm').textContent=live&&rc.arm?'ARM REQ — INHIBITED':'ARM REQ';
  setv('r_mode',live?(rc.mode+' · '+(rc.alt/10).toFixed(1)+' m'):'--');
 }
 else if(pg==='system'){
  setv('y_build',Y.build);setv('y_sdk',Y.sdk);setv('y_rst',Y.rst);
  setv('y_ap',Y.ip+' · ch '+Y.ch);setv('y_cli',Y.cli+' / '+Y.ws);
  setv('y_tx',Y.txkb<1024?Y.txkb+' kB':(Y.txkb/1024).toFixed(1)+' MB');setv('y_wsc',''+Y.wsc);
  setv('y_imu',im.ok?('OK @ 0x'+im.addr.toString(16).toUpperCase()):'DISCONNECTED',im.ok?'good':'bad');
  setv('y_imucal',im.cal+(im.nvs?' (stored)':''));
  setv('y_mag',mg.ok?('OK @ 0x'+mg.addr.toString(16).toUpperCase()):'DISCONNECTED',mg.ok?'good':'bad');
  setv('y_magcal',mg.cal+(mg.nvs?' (stored)':''));
  setv('y_tof',tf.ok?('OK @ 0x29'+(tf.valid?'':' — OUT OF RANGE')):'DISCONNECTED',tf.ok?(tf.valid?'good':'warnc'):'bad');
  setv('y_ages',(im.ok?im.age:'--')+' / '+(mg.ok?mg.age:'--')+' / '+(tf.ok?tf.age:'--')+' ms');
  setv('y_raw',f(im.gx,0)+'/'+f(im.gy,0)+'/'+f(im.gz,0)+' dps · '+
       f(im.ax,2)+'/'+f(im.ay,2)+'/'+f(im.az,2)+' g · '+f(im.t,1)+' °C');
  setv('y_nrf',rc.det?'DETECTED':'NOT DETECTED',rc.det?'good':'bad');
  setv('y_rcage',(rc.tot?rc.age+' ms':'--')+' · '+rc.pps+' /s');
  setv('y_rcloss',rc.tot?((100*rc.drop/(rc.tot+rc.drop)).toFixed(1)+' % ('+rc.drop+'/'+(rc.tot+rc.drop)+')'):'--');
  setv('y_rcv','thr '+rc.thr+' · X '+rc.sx+' · Y '+rc.sy+' · flags '+rc.mode);
  setv('y_up',upfmt(Y.up));
  setv('y_heap',Math.round(Y.heap/1024)+' / '+Math.round(Y.minheap/1024)+' kB');
  setv('y_hz',R.hz+' / '+Y.pushHz+' Hz');
 }
 else if(pg==='plant'){
  var P=R.plant,pu=P.up;
  $('pl_off').style.display=pu?'none':'';
  fillPGrid('pl_grid',P,pu,true);
  setv('pl_link',pu?'CONNECTED':(P.n>0?'STALE — no recent scan':'WAITING for first scan'),
       pu?'good':(P.n>0?'warnc':'offc'));
  var ov=POVR[P.ov]||['off','--'];
  var pe=$('pl_ov');pe.textContent=pu?ov[1]:'--';pe.className='povr '+(pu?ov[0]:'off');
  setv('pl_gyr',pu?(P.g+' / '+P.y+' / '+P.r):'--');
  setv('pl_seq',pu?('#'+P.seq+' / '+P.n):'--');
  setv('pl_age',pu?((P.age>=999999?'--':P.age+' ms')+' · '+P.bad+' bad'):'--');
  $('pl_ah').style.width=(pu?P.ah:0)+'%';$('pl_ah_t').textContent=pu?P.ah+'%':'--';
  $('pl_ap').style.width=(pu?P.ap:0)+'%';$('pl_ap_t').textContent=pu?P.ap+'%':'--';
  $('pl_ar').style.width=(pu?P.ar:0)+'%';$('pl_ar_t').textContent=pu?P.ar+'%':'--';
 }
 else if(pg==='mission'){
  if(Date.now()-lastMapMs>1000){lastMapMs=Date.now();missionDraw()}
 }
}
function cbar(bid,vid,val,min,max,txt){
 var e=$(bid),pct=(val-min)/(max-min)*100;
 if(min<0){var z=(0-min)/(max-min)*100;
  e.style.left=Math.min(pct,z)+'%';e.style.width=Math.abs(pct-z)+'%'}
 else{e.style.left='0';e.style.width=pct+'%'}
 $(vid).textContent=txt;
}
function swst(id,on){$(id).classList.toggle('on',!!on)}

/* ================= canvas charts (null = gap) ================= */
function sizeCanvases(){
 document.querySelectorAll('canvas').forEach(function(c){
  var r=c.getBoundingClientRect();if(r.width<10)return;
  var dpr=window.devicePixelRatio||1;
  c.width=Math.round(r.width*dpr);c.height=Math.round(r.height*dpr);
 });
 dirty=true;
}
window.addEventListener('resize',sizeCanvases);
function drawChart(cid,keys,cols,fixed){
 var c=$(cid);if(!c.width)return;
 var g=c.getContext('2d'),W=c.width,Hh=c.height,dpr=window.devicePixelRatio||1;
 g.clearRect(0,0,W,Hh);
 var win=+($('chwin').value||20),now=performance.now()/1000;
 var i0=0;while(i0<H.t.length&&H.t[i0]<now-win)i0++;
 var n=H.t.length-i0;
 if(n<2){g.fillStyle='#8a95a1';g.font=(11*dpr)+'px sans-serif';
  g.fillText('collecting…',8*dpr,16*dpr);return}
 var mn=1e9,mx=-1e9,any=false;
 keys.forEach(function(k){for(var i=i0;i<H[k].length;i++){var v=H[k][i];
  if(v==null)continue;any=true;if(v<mn)mn=v;if(v>mx)mx=v}});
 if(!any){g.fillStyle='#8a95a1';g.font=(11*dpr)+'px sans-serif';
  g.fillText('no valid samples in window',8*dpr,16*dpr);return}
 if(fixed){mn=Math.min(mn,fixed[0]);mx=Math.max(mx,fixed[1])}
 if(mx-mn<1e-3){mx+=0.5;mn-=0.5}
 var pad=(mx-mn)*0.1;mn-=pad;mx+=pad;
 g.strokeStyle='#2a333d';g.lineWidth=1;g.beginPath();
 for(var r=1;r<4;r++){var y=Hh*r/4;g.moveTo(0,y);g.lineTo(W,y)}g.stroke();
 g.fillStyle='#8a95a1';g.font=(10*dpr)+'px ui-monospace,monospace';
 g.fillText(mx.toFixed(1),4*dpr,11*dpr);g.fillText(mn.toFixed(1),4*dpr,Hh-4*dpr);
 keys.forEach(function(k,ki){
  g.strokeStyle=cols[ki];g.lineWidth=1.5*dpr;g.beginPath();
  var pen=false,last=null;
  for(var i=i0;i<H.t.length;i++){
   var v=H[k][i];
   if(v==null){pen=false;continue}
   var x=(H.t[i]-(now-win))/win*W;
   var y=Hh-(v-mn)/(mx-mn)*Hh;
   if(!pen){g.moveTo(x,y);pen=true}else{g.lineTo(x,y)}
   last=v;
  }
  g.stroke();
  g.fillStyle=cols[ki];
  g.fillText(last==null?'--':last.toFixed(1),W-52*dpr,(12+11*ki)*dpr);
 });
}

/* ================= IMU FFT (browser-side) ================= */
function fftMag(x){
 var N=256,re=new Array(N),im=new Array(N),i,j;
 var mean=0;for(i=0;i<x.length;i++)mean+=x[i];mean/=x.length||1;
 for(i=0;i<N;i++){var v=i<x.length?x[i]-mean:0;
  re[i]=v*0.5*(1-Math.cos(2*Math.PI*i/(N-1)));im[i]=0}
 for(i=1,j=0;i<N;i++){var bit=N>>1;
  for(;j&bit;bit>>=1)j^=bit;j^=bit;
  if(i<j){var t=re[i];re[i]=re[j];re[j]=t;t=im[i];im[i]=im[j];im[j]=t}}
 for(var len=2;len<=N;len<<=1){
  var ang=-2*Math.PI/len,wr=Math.cos(ang),wi=Math.sin(ang);
  for(var s=0;s<N;s+=len){
   var cr=1,ci=0;
   for(var k=0;k<len/2;k++){
    var ur=re[s+k],ui=im[s+k];
    var vr=re[s+k+len/2]*cr-im[s+k+len/2]*ci;
    var vi=re[s+k+len/2]*ci+im[s+k+len/2]*cr;
    re[s+k]=ur+vr;im[s+k]=ui+vi;
    re[s+k+len/2]=ur-vr;im[s+k+len/2]=ui-vi;
    var ncr=cr*wr-ci*wi;ci=cr*wi+ci*wr;cr=ncr;
   }
  }
 }
 var m=new Array(N/2);
 for(i=0;i<N/2;i++)m[i]=Math.sqrt(re[i]*re[i]+im[i]*im[i])/N;
 return m;
}
var fftBusy=false;
function fftPoll(){
 if(curPage()!=='sensors'||fftBusy){return}
 fftBusy=true;
 fetch('/api/fft/imu?axis='+$('fftaxis').value).then(function(r){return r.json()})
 .then(function(j){
  fftBusy=false;
  if(!j.d||j.d.length<32){$('fftinfo').textContent='waiting for IMU samples…';return}
  var m=fftMag(j.d),rate=j.rate||100,df=rate/256;
  var mean=0;for(var i=0;i<j.d.length;i++)mean+=j.d[i];mean/=j.d.length;
  var rms=0;for(i=0;i<j.d.length;i++)rms+=(j.d[i]-mean)*(j.d[i]-mean);
  rms=Math.sqrt(rms/j.d.length);
  var p1=1,p2=1;
  for(i=2;i<m.length;i++){if(m[i]>m[p1])p1=i}
  for(i=2;i<m.length;i++){if(Math.abs(i-p1)>3&&m[i]>m[p2]&&i!==p1)p2=i}
  drawSpectrum('ch_fft',m.slice(1),df,df,'#2fb3c2',[p1-1],null);
  $('fftinfo').textContent='axis '+j.axis.toUpperCase()+' · '+j.n+' samples @ '+rate+
   ' Hz · Δf '+df.toFixed(2)+' Hz · dominant '+(p1*df).toFixed(1)+' Hz · secondary '+
   (p2*df).toFixed(1)+' Hz · RMS '+rms.toFixed(3)+' g';
 }).catch(function(){fftBusy=false;$('fftinfo').textContent='FFT fetch failed — link down?'});
}
setInterval(fftPoll,1000);

function drawSpectrum(cid,mags,f0,df,color,marks,overlay){
 var c=$(cid);if(!c.width)return;
 var g=c.getContext('2d'),W=c.width,Hh=c.height,dpr=window.devicePixelRatio||1;
 g.clearRect(0,0,W,Hh);
 var n=mags.length,fmax=f0+df*(n-1);
 var mx=0;var sets=overlay||[{mags:mags,color:color}];
 sets.forEach(function(s){for(var i=0;i<s.mags.length;i++)if(s.mags[i]>mx)mx=s.mags[i]});
 if(mx<=0)mx=1;
 g.strokeStyle='#2a333d';g.beginPath();
 for(var r=1;r<4;r++){var y=Hh*r/4;g.moveTo(0,y);g.lineTo(W,y)}g.stroke();
 g.fillStyle='#8a95a1';g.font=(10*dpr)+'px ui-monospace,monospace';
 var step=fmax<120?10:100;
 for(var fx=Math.ceil(f0/step)*step;fx<=fmax;fx+=step){var x=(fx-f0)/(fmax-f0)*W;
  if(x>0&&x<W-30*dpr){g.fillText(fx+'Hz',x,Hh-4*dpr);
   g.strokeStyle='#2a333d';g.beginPath();g.moveTo(x,0);g.lineTo(x,Hh-14*dpr);g.stroke()}}
 if(marks)marks.forEach(function(mi){
  var x=(mi)/(n-1)*W;g.strokeStyle='#ef6a5f';g.setLineDash([4*dpr,4*dpr]);
  g.beginPath();g.moveTo(x,0);g.lineTo(x,Hh-14*dpr);g.stroke();g.setLineDash([])});
 sets.forEach(function(s){
  g.strokeStyle=s.color;g.lineWidth=1.5*dpr;g.beginPath();
  for(var i=0;i<s.mags.length;i++){
   var x=i/(s.mags.length-1)*W,y=Hh-14*dpr-(s.mags[i]/mx)*(Hh-24*dpr);
   i?g.lineTo(x,y):g.moveTo(x,y);
  }
  g.stroke();
 });
}

/* ================= recorded motor spectra ================= */
var mSel=0;
(function(){
 var row=$('mtrsel');
 ['Motor 1','Motor 2','Motor 3','Motor 4','Compare all'].forEach(function(lb,i){
  var b=document.createElement('button');b.className='btn';b.textContent=lb;
  b.onclick=function(){mSel=i;row.querySelectorAll('.btn').forEach(function(x,xi){
    x.classList.toggle('pri',xi===i)});drawMotor()};
  if(i===0)b.classList.add('pri');
  row.appendChild(b);
 });
 $('msrc').textContent='Source: Recorded Motor Test — bench sweep 2026-05-16, '+
  MFFT.axis+' spectrum, motors at DShot '+MFFT.dshot+' ('+MFFT.source+', FFT '+
  MFFT.fftSize+' pts @ '+MFFT.rate+' Hz, reduced to 128 bins). Peak table from the original test reports.';
})();
function drawMotor(){
 if(curPage()!=='sensors')return;
 var tb=$('mpk').querySelector('tbody'),html='';
 if(mSel<4){
  var mo=MFFT.motors[mSel],fund=mo.peaks[0][0];
  var marks=[];for(var h=1;h<=3;h++){var bin=(fund*h-MFFT.f0)/MFFT.df;
   if(bin<128)marks.push(bin)}
  drawSpectrum('ch_mfft',mo.mags,MFFT.f0,MFFT.df,MCOL[mSel],marks,null);
  mo.peaks.forEach(function(p,i){
   html+='<tr><td>'+(i===0?'fundamental':'harmonic '+(i+1))+'</td><td>'+p[0].toFixed(1)+
    ' Hz</td><td>'+p[1].toFixed(0)+'</td></tr>'});
 }else{
  drawSpectrum('ch_mfft',MFFT.motors[0].mags,MFFT.f0,MFFT.df,MCOL[0],null,
   MFFT.motors.map(function(mo,i){return{mags:mo.mags,color:MCOL[i]}}));
  MFFT.motors.forEach(function(mo,i){
   html+='<tr><td style="color:'+MCOL[i]+'">Motor '+mo.m+'</td><td>'+
    mo.peaks[0][0].toFixed(1)+' Hz</td><td>'+mo.peaks[0][1].toFixed(0)+'</td></tr>'});
 }
 tb.innerHTML=html;
 var rms=MFFT.motors.map(function(mo){var s=0;mo.mags.forEach(function(v){s+=v*v});return Math.sqrt(s)});
 var mmax=Math.max.apply(null,rms),rh='';
 MFFT.motors.forEach(function(mo,i){
  rh+='<div class="meter"><span class="ml">M'+mo.m+'</span><div class="tr">'+
   '<i style="width:'+(rms[i]/mmax*100).toFixed(0)+'%;background:'+MCOL[i]+'"></i></div>'+
   '<span class="mv">'+(rms[i]/mmax*100).toFixed(0)+'%</span></div>'});
 $('mrms').innerHTML=rh;
}

/* ================= mission + overview maps (shared renderer) ============== */
function mapSave(){try{localStorage.setItem('larp.mission.v1',JSON.stringify(MISSION))}catch(e){}}
function mapCfg(){return MAPS[MAP_LAYER]}
function prj(lat,lon){var c=mapCfg();
 return{x:(lon-c.lonW)/(c.lonE-c.lonW)*MAP_W,
        y:(c.latN-lat)/(c.latN-c.latS)*MAP_H}}
function unprj(x,y){var c=mapCfg();
 return{lat:c.latN-y/MAP_H*(c.latN-c.latS),
        lon:c.lonW+x/MAP_W*(c.lonE-c.lonW)}}
function setLayer(l){MAP_LAYER=l;VB={x:0,y:0,w:MAP_W,h:MAP_H};
 document.querySelectorAll('#lyrsel .btn').forEach(function(b){
  b.classList.toggle('pri',b.getAttribute('data-l')===l)});
 missionDraw()}
function distM(a,b){
 var dy=(b.lat-a.lat)*111320,dx=(b.lon-a.lon)*111320*Math.cos(a.lat*Math.PI/180);
 return Math.sqrt(dx*dx+dy*dy)}
function brg(a,b){
 var dy=(b.lat-a.lat),dx=(b.lon-a.lon)*Math.cos(a.lat*Math.PI/180);
 return((Math.atan2(dx,dy)*180/Math.PI)+360)%360}
function mapMarkup(){
 var c=mapCfg();
 var scaleM=Math.round((c.lonE-c.lonW)*111320*Math.cos(c.latN*Math.PI/180)*(120/MAP_W));
 var s='<image href="'+c.img+'" x="0" y="0" width="'+MAP_W+'" height="'+MAP_H+'" preserveAspectRatio="none"/>';
 var home=prj(HOME.lat,HOME.lon);
 if(MISSION.length){
  var pts=[home].concat(MISSION.map(function(w){return prj(w.lat,w.lon)}));
  s+='<polyline fill="none" stroke="#0d7a86" stroke-width="2.5" stroke-dasharray="7 4" points="'+
   pts.map(function(p){return p.x.toFixed(1)+','+p.y.toFixed(1)}).join(' ')+'"/>';
 }
 s+='<g transform="translate('+home.x.toFixed(1)+','+home.y.toFixed(1)+')">'+
  '<rect x="-7" y="-7" width="14" height="14" fill="#2456a6" rx="2" stroke="#fff"/>'+
  '<text x="11" y="4" font-size="12" font-weight="600" fill="#1d3f7a" stroke="#fff" stroke-width="2.5" paint-order="stroke">HOME</text></g>';
 if(T.d){var v=prj(T.d.sim.lat,T.d.sim.lon);
  s+='<g transform="translate('+v.x.toFixed(1)+','+v.y.toFixed(1)+')"><circle r="7" fill="#c07f10" stroke="#fff" stroke-width="1.5"/>'+
   '<circle r="11" fill="none" stroke="#c07f10"/><title>vehicle (presentation GPS)</title></g>'}
 MISSION.forEach(function(w,i){var p=prj(w.lat,w.lon);
  s+='<g transform="translate('+p.x.toFixed(1)+','+p.y.toFixed(1)+')">'+
   '<circle r="10" fill="#0d7a86" stroke="#fff" stroke-width="1.5"/>'+
   '<text y="4" text-anchor="middle" font-size="11" fill="#fff">'+(i+1)+'</text></g>'});
 /* scale bar + REQUIRED OSM attribution */
 s+='<g transform="translate(24,'+(MAP_H-18)+')"><rect x="-8" y="-16" width="150" height="26" rx="3" fill="rgba(255,255,255,.75)"/>'+
  '<line x1="0" y1="0" x2="120" y2="0" stroke="#333" stroke-width="2"/>'+
  '<text x="36" y="-4" font-size="10" fill="#333">'+scaleM+' m</text></g>';
 s+='<g><rect x="'+(MAP_W-198)+'" y="'+(MAP_H-20)+'" width="198" height="20" fill="rgba(255,255,255,.75)"/>'+
  '<text x="'+(MAP_W-6)+'" y="'+(MAP_H-6)+'" text-anchor="end" font-size="10" fill="#333">&#169; OpenStreetMap contributors</text></g>';
 return s;
}
function overviewMapDraw(){
 var svg=$('omap');if(!svg)return;
 svg.setAttribute('viewBox','0 0 '+MAP_W+' '+MAP_H);
 svg.innerHTML=mapMarkup();
}
function missionDraw(){
 var svg=$('map');
 svg.setAttribute('viewBox',VB.x+' '+VB.y+' '+VB.w+' '+VB.h);
 svg.innerHTML=mapMarkup();
 var tb=$('wpt').querySelector('tbody'),html='',prev={lat:HOME.lat,lon:HOME.lon},tot=0;
 MISSION.forEach(function(w,i){
  var d=distM(prev,w),b=brg(prev,w);tot+=d;
  html+='<tr><td>'+(i+1)+'</td><td>'+w.lat.toFixed(5)+'</td><td>'+w.lon.toFixed(5)+
   '</td><td>'+d.toFixed(0)+' m</td><td>'+b.toFixed(0)+'°</td><td>'+w.alt.toFixed(1)+'</td>'+
   '<td><button class="btn" onclick="wpMove('+i+',-1)">&#8593;</button> '+
   '<button class="btn" onclick="wpMove('+i+',1)">&#8595;</button> '+
   '<button class="btn warnb" onclick="wpDel('+i+')">&#215;</button></td></tr>';
  prev=w;
 });
 tb.innerHTML=html;
 $('wp_tot').textContent=MISSION.length+' / '+tot.toFixed(0)+' m';
}
function wpAdd(lat,lon){
 var c=unprj(VB.x+VB.w/2,VB.y+VB.h/2);
 MISSION.push({lat:lat!==undefined?lat:c.lat,lon:lon!==undefined?lon:c.lon,alt:2.0});
 mapSave();missionDraw();
}
function wpDel(i){MISSION.splice(i,1);mapSave();missionDraw()}
function wpMove(i,dir){var j=i+dir;if(j<0||j>=MISSION.length)return;
 var t=MISSION[i];MISSION[i]=MISSION[j];MISSION[j]=t;mapSave();missionDraw()}
function wpClear(){MISSION=[];mapSave();missionDraw()}
function wpDefault(){
 MISSION=[];
 var r=0.00035,c=HOME;
 [[0,1.6],[1.2,0.8],[1.2,-0.8],[0,-1.6],[-1.2,-0.8],[-1.2,0.8]].forEach(function(p,i){
  MISSION.push({lat:c.lat+p[0]*r,lon:c.lon+p[1]*r*1.35,alt:2.0+i*0.5})});
 mapSave();missionDraw();
}
function wpExport(btn){
 var txt=JSON.stringify(MISSION.map(function(w,i){return{n:i+1,lat:+w.lat.toFixed(6),
  lon:+w.lon.toFixed(6),alt:w.alt}}),null,1);
 if(navigator.clipboard&&navigator.clipboard.writeText){
  navigator.clipboard.writeText(txt).then(function(){btn.textContent='Copied';
   setTimeout(function(){btn.textContent='Copy coordinates'},1500)})}
 else{window.prompt('Waypoint JSON:',txt)}
}
function mapZoom(k){
 var cx=VB.x+VB.w/2,cy=VB.y+VB.h/2;
 VB.w=Math.min(MAP_W,Math.max(100,VB.w/k));VB.h=VB.w*MAP_H/MAP_W;
 VB.x=cx-VB.w/2;VB.y=cy-VB.h/2;missionDraw();
}
function mapCenter(){VB={x:0,y:0,w:MAP_W,h:MAP_H};missionDraw()}
(function(){
 var svg=$('map'),drag=null;
 function evPt(ev){var r=svg.getBoundingClientRect();
  var t=ev.touches?ev.touches[0]:ev;
  return{x:VB.x+(t.clientX-r.left)/r.width*VB.w,y:VB.y+(t.clientY-r.top)/r.height*VB.h}}
 svg.addEventListener('pointerdown',function(ev){drag={p:evPt(ev),moved:false};svg.setPointerCapture(ev.pointerId)});
 svg.addEventListener('pointermove',function(ev){
  if(!drag)return;var p=evPt(ev);
  var dx=p.x-drag.p.x,dy=p.y-drag.p.y;
  if(Math.abs(dx)>3||Math.abs(dy)>3){drag.moved=true;VB.x-=dx;VB.y-=dy;missionDraw()}});
 svg.addEventListener('pointerup',function(ev){
  if(drag&&!drag.moved){var p=evPt(ev),ll=unprj(p.x,p.y);wpAdd(ll.lat,ll.lon)}
  drag=null});
})();

/* ================= calibration API ================= */
function api(path,btn){
 if(btn)btn.disabled=true;
 fetch(path,{method:'POST'}).then(function(r){return r.json()}).then(function(j){
  var cm=$('calmsg');
  if(j.ok){cm.textContent='OK: '+(j.msg||path);cm.className='statline good'}
  else{cm.textContent='REFUSED: '+(j.err||'error');cm.className='statline bad'}
 }).catch(function(){var cm=$('calmsg');cm.textContent='Request failed — link down?';
  cm.className='statline bad'})
 .then(function(){if(btn)btn.disabled=false});
}
function plantMode(mode){
 fetch('/api/plant/mode?m='+mode,{method:'POST'}).catch(function(){});
}
document.querySelectorAll('.mrot').forEach(function(s){
 ['0°','90°','180°','270°'].forEach(function(t,i){
  var o=document.createElement('option');o.value=i;o.textContent=t;s.appendChild(o)})
});
function mountLoad(){
 fetch('/api/mount',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){
  ['x','y','z'].forEach(function(a,i){$('m_i'+a).value=j.imu[i];$('m_m'+a).value=j.mag[i]});
  $('mountmsg').textContent='Orientation loaded';$('mountmsg').className='statline good';
 }).catch(function(){$('mountmsg').textContent='Could not load orientation';$('mountmsg').className='statline bad'});
}
function mountSave(sensor,btn){
 var p=sensor==='imu'?'i':'m';
 var q='?s='+sensor+'&x='+$('m_'+p+'x').value+'&y='+$('m_'+p+'y').value+
       '&z='+$('m_'+p+'z').value;
 btn.disabled=true;
 fetch('/api/mount'+q,{method:'POST'}).then(function(r){return r.json()}).then(function(j){
  $('mountmsg').textContent=j.ok?(sensor.toUpperCase()+' orientation saved'):(j.err||'Refused');
  $('mountmsg').className='statline '+(j.ok?'good':'bad');
  clearHist();
 }).catch(function(){$('mountmsg').textContent='Save failed';$('mountmsg').className='statline bad'})
 .then(function(){btn.disabled=false});
}
var mtTimer=0,mtMotor=0;
function motorEnable(btn){
 if(!$('mt_ack').checked){$('mt_ready').textContent='CHECK SAFETY ACKNOWLEDGEMENT';return}
 btn.disabled=true;
 fetch('/api/motor/enable',{method:'POST'}).then(function(r){return r.json()}).then(function(j){
  $('mt_ready').textContent=j.ok?'ENABLED FOR 30 SECONDS':('LOCKED · '+(j.err||'REFUSED'));
 }).catch(function(){$('mt_ready').textContent='REQUEST FAILED'})
 .then(function(){btn.disabled=false});
}
function motorRefresh(){
 if(!mtMotor)return;
 var t=Math.min(50,Math.max(1,+$('mt_thr').value||1));
 fetch('/api/motor/run?m='+mtMotor+'&t='+t,{method:'POST'}).catch(motorStop);
}
function motorHoldStart(ev,motor){
 ev.preventDefault();
 if(mtMotor)motorStop();
 mtMotor=motor;motorRefresh();
 mtTimer=setInterval(motorRefresh,180);
}
function motorStop(){
 mtMotor=0;if(mtTimer){clearInterval(mtTimer);mtTimer=0}
 fetch('/api/motor/stop',{method:'POST',keepalive:true}).catch(function(){});
}
function motorStatus(){
 if(curPage()!=='motors')return;
 fetch('/api/motor/status',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){
  $('mt_ready').textContent=!j.ready?'LOCKED':(j.enabled?'ENABLED':'DISABLED');
  $('mt_active').textContent=j.motor?('M'+j.motor+' · '+j.throttle+'% · '+j.deadman+' ms'):'STOPPED';
 }).catch(function(){$('mt_ready').textContent='OFFLINE'});
}
window.addEventListener('pointerup',motorStop);
window.addEventListener('pointercancel',motorStop);
document.addEventListener('visibilitychange',function(){if(document.hidden)motorStop()});
window.addEventListener('beforeunload',motorStop);
setInterval(motorStatus,500);

/* ================= render loop ================= */
setInterval(function(){
 if(Date.now()-T.lastMsg>4000&&T.lastMsg){setDot('d_ws','err');$('h_cli').textContent='NO DATA'}
},2000);
var lastRafMs=0;
function stepInstr(k){
 cur.r+=(tgt.r-cur.r)*k;cur.p+=(tgt.p-cur.p)*k;
 var dh=((tgt.h-cur.h+540)%360)-180;cur.h+=dh*k;cur.h=(cur.h+360)%360;
 var p=curPage()==='attitude'?'a_':(curPage()==='overview'?'o_':null);
 if(p){
  $(p+'horz').setAttribute('transform','rotate('+(-cur.r)+') translate(0,'+(cur.p*2.8)+')');
  $(p+'rollarc').setAttribute('transform','rotate('+(-cur.r)+')');
  $(p+'hdgring').setAttribute('transform','rotate('+(-cur.h)+')');
 }
}
(function loop(){
 lastRafMs=Date.now();
 stepInstr(0.2);
 requestAnimationFrame(loop);
})();
setInterval(function(){
 // rAF is throttled in hidden/battery-saver contexts — keep instruments
 // moving from this timer whenever the rAF loop has stalled.
 if(Date.now()-lastRafMs>250)stepInstr(0.5);
 if(!dirty)return;dirty=false;
 renderAll();
 if(curPage()==='sensors'){
  drawChart('ch_att',['rr','rp','ry'],['#2fb3c2','#6ea8ff','#e0a33a']);
  drawChart('ch_gyr',['gx','gy','gz'],['#2fb3c2','#6ea8ff','#e0a33a']);
  drawChart('ch_acc',['ax','ay','az','am'],['#2fb3c2','#6ea8ff','#e0a33a','#a98fd6']);
  drawChart('ch_alt',['alt'],['#2fb3c2']);
  drawChart('ch_tmp',['tp'],['#ef6a5f']);
  drawMotor();
 }
},80);
sizeCanvases();
mountLoad();
connect();
</script>
<div class="pmodes" aria-label="Plant state">
 <button title="Healthy: usually 80–88%, occasional outliers, maximum 90%" onclick="plantMode('H')">H</button>
 <button title="Rust: usually 80–88%, occasional outliers, maximum 90%" onclick="plantMode('R')">R</button>
 <button title="Pest: usually 80–88%, occasional outliers, maximum 90%" onclick="plantMode('P')">P</button>
</div>
</body></html>
)LARP";
