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
/* ---- mission planner ---- */
#ms_wrap{display:flex;gap:12px;align-items:stretch}
@media(max-width:980px){#ms_wrap{flex-direction:column}}
#ms_mapcard{flex:1;min-width:0;position:relative;border:1px solid var(--line);border-radius:var(--r);overflow:hidden;background:#0a0d12}
#ms_map{height:min(70vh,660px);min-height:430px;background:#0a0d12;cursor:crosshair;z-index:0}
#ms_side{flex:0 0 332px;display:flex;flex-direction:column;gap:12px;max-height:min(70vh,660px)}
@media(max-width:980px){#ms_side{flex:1;max-height:560px}}
#ms_boot{position:absolute;inset:0;z-index:1200;display:flex;flex-direction:column;gap:14px;align-items:center;justify-content:center;background:#0a0d12;text-align:center;padding:16px}
.ms-spin{width:34px;height:34px;border-radius:50%;border:3px solid #28303f;border-top-color:var(--accent);animation:msspin 1s linear infinite}
@keyframes msspin{to{transform:rotate(360deg)}}
.ms-glass{position:absolute;z-index:1100;display:flex;gap:8px;align-items:center;flex-wrap:wrap;
 background:#0d121cd9;border:1px solid #ffffff22;border-radius:10px;padding:6px 8px;
 backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);box-shadow:0 4px 20px #000a}
#ms_tools{top:10px;left:10px}
#ms_stats{bottom:24px;left:10px}
.ms-chip{display:inline-flex;gap:6px;align-items:center;font-size:12px;padding:0 3px}
.ms-chip .k{color:var(--dim);font-size:10px;letter-spacing:.4px}
.ms-chip b{font-family:var(--mono);font-weight:600}
#ms_hint{position:absolute;left:50%;top:58px;transform:translateX(-50%);z-index:1050;background:#0d121cd9;
 border:1px solid #3fa9ff55;color:#bfe0ff;padding:7px 14px;border-radius:999px;font-size:12px;white-space:nowrap;
 backdrop-filter:blur(8px);pointer-events:none;box-shadow:0 4px 16px #000a}
.ms-wp{width:30px;height:30px;border-radius:50%;display:flex;align-items:center;justify-content:center;
 color:#fff;font:700 12px system-ui;background:radial-gradient(circle at 32% 30%,#6ec1ff,#1861c9 72%);
 border:2px solid #ffffffe0;box-shadow:0 0 12px #3fa9ffb0,0 2px 8px #000b;transition:transform .12s;cursor:grab}
.ms-wp.hold{background:radial-gradient(circle at 32% 30%,#a78dff,#5b2fd6 72%);box-shadow:0 0 12px #7c5cffb0,0 2px 8px #000b}
.ms-wp.land{background:radial-gradient(circle at 32% 30%,#ffd06e,#cf7a10 72%);box-shadow:0 0 12px #ffb020a8,0 2px 8px #000b;color:#231500}
.ms-wp.sel{transform:scale(1.25);outline:2px solid #fff;outline-offset:2px}
.ms-nul{background:none!important;border:none!important}
.ms-tip{background:#0d121cf0!important;border:1px solid #3fa9ff44!important;color:var(--txt)!important;
 font-family:var(--mono);font-size:11px;border-radius:7px;box-shadow:0 4px 14px #000a!important}
.ms-tip:before{border:none!important}
.ms-route-dash{stroke-dasharray:2 12;stroke-linecap:round;animation:msdash 1s linear infinite}
@keyframes msdash{to{stroke-dashoffset:-14}}
.ms-home{width:28px;height:28px;border-radius:50%;display:flex;align-items:center;justify-content:center;
 background:radial-gradient(circle at 30% 30%,#4de394,#128a4c 75%);border:2px solid #fff;box-shadow:0 0 10px #33d17a90,0 2px 5px #0009}
.ms-dwrap{position:relative;width:34px;height:34px}
.ms-pulse{position:absolute;inset:0;border-radius:50%;border:2px solid #25e0ff;animation:mspulse 1.6s ease-out infinite}
@keyframes mspulse{0%{transform:scale(.4);opacity:.9}100%{transform:scale(1.5);opacity:0}}
.ms-drone-rot{position:absolute;inset:7px;display:flex;align-items:center;justify-content:center;
 filter:drop-shadow(0 0 6px #25e0ffc0);transition:transform .25s linear}
.ms-ghost{width:18px;height:18px;border-radius:50%;background:radial-gradient(circle,#fff,#3fa9ff);box-shadow:0 0 14px #3fa9ff,0 0 30px #3fa9ff80}
.ms-item{background:var(--card2);border:1px solid var(--line);border-radius:9px;padding:8px 10px;cursor:pointer;flex:0 0 auto;transition:border-color .15s,box-shadow .15s}
.ms-item:hover{border-color:#34425c}
.ms-item.sel{border-color:var(--accent);box-shadow:0 0 0 1px var(--accent) inset,0 0 14px #3fa9ff28}
.ms-item .hd{display:flex;align-items:center;gap:7px}
.ms-num{flex:0 0 auto;width:22px;height:22px;border-radius:50%;display:flex;align-items:center;justify-content:center;
 color:#fff;font:700 11px system-ui;background:radial-gradient(circle at 32% 30%,#6ec1ff,#1861c9 72%)}
.ms-num.hold{background:radial-gradient(circle at 32% 30%,#a78dff,#5b2fd6 72%)}
.ms-num.land{background:radial-gradient(circle at 32% 30%,#ffd06e,#cf7a10 72%);color:#231500}
.ms-act{background:#0d1118;color:var(--txt);border:1px solid var(--line);border-radius:6px;padding:3px 6px;font-size:12px}
.ms-ib{background:none;border:1px solid transparent;border-radius:6px;color:var(--dim);cursor:pointer;padding:3px 6px;font-size:11px}
.ms-ib:hover{color:var(--txt);border-color:var(--line);background:#243047}
.ms-ib.del:hover{color:#ff9c9c;border-color:#5c2626;background:#3a1a1a}
.ms-coord{color:var(--dim);font-size:11px;margin:5px 0 0 2px}
.ms-fields{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:7px}
.ms-fields label{display:block;font-size:9px;color:var(--dim);text-transform:uppercase;letter-spacing:.4px;margin-bottom:2px}
.ms-fields input{padding:4px 6px;font-size:12px}
.ms-none{border:1px dashed #34425c;border-radius:9px;color:var(--dim);text-align:center;padding:26px 10px;font-size:13px;line-height:1.6}
#ms_map .leaflet-bar a{background:#12182a;color:#cfd6e4;border-bottom-color:#28303f}
#ms_map .leaflet-bar a:hover{background:#1d2534;color:#fff}
#ms_map .leaflet-bar{border:1px solid #ffffff22;box-shadow:0 4px 14px #000a;border-radius:8px;overflow:hidden}
#ms_map .leaflet-control-attribution{background:#0b0e13b8;color:#5d6678;font-size:10px}
#ms_map .leaflet-control-attribution a{color:#8a94a6}
#ms_map .leaflet-control-scale-line{background:#0b0e13b0;color:#aab4c6;border:2px solid #aab4c680;border-top:none}
/* ---- mission planner: component skin (modern app look) ---- */
#t_mission .card{background:linear-gradient(180deg,#161d2b,#121826);border:1px solid #ffffff14;border-radius:14px;
 box-shadow:0 1px 2px #0006,0 10px 30px #00000038;padding:14px 15px}
.ms-ph{display:flex;align-items:center;justify-content:space-between;margin-bottom:11px}
.ms-ph span:first-child{font-size:14px;font-weight:650;letter-spacing:.2px}
.ms-badge{min-width:22px;text-align:center;padding:2px 8px;border-radius:999px;font:600 11px var(--mono);
 background:linear-gradient(135deg,#3fa9ff33,#7c5cff33);border:1px solid #3fa9ff55;color:#9fd4ff}
.msb{display:inline-flex;align-items:center;justify-content:center;gap:6px;height:34px;padding:0 14px;
 border-radius:10px;font:600 13px system-ui;color:var(--txt);background:#1d2536;border:1px solid #ffffff1a;
 cursor:pointer;transition:transform .13s,box-shadow .13s,background .13s,border-color .13s,filter .13s}
.msb:hover{background:#242e44;border-color:#ffffff2e;transform:translateY(-1px)}
.msb:active{transform:translateY(0) scale(.98)}
.msb.sm{height:28px;padding:0 10px;font-size:12px;border-radius:8px}
.msb.pri{background:linear-gradient(135deg,#2f8fe0,#6d4de6);border-color:#7c9dff55;color:#fff;
 box-shadow:0 4px 14px #3f6dff3d}
.msb.pri:hover{filter:brightness(1.12);box-shadow:0 6px 20px #3f6dff55}
.msb.danger{color:#ff9c9c}
.msb.danger:hover{background:#3a1a1a;border-color:#7a2b2b}
.ms-seg{display:flex;background:#0b0f17e6;border:1px solid #ffffff14;border-radius:10px;padding:3px;gap:2px}
.ms-seg button{border:none;background:none;color:var(--dim);font:600 12px system-ui;padding:5px 11px;
 border-radius:7px;cursor:pointer;transition:all .15s}
.ms-seg button:hover{color:var(--txt)}
.ms-seg button.active{background:linear-gradient(135deg,#2f8fe0,#6d4de6);color:#fff;box-shadow:0 2px 10px #3f6dff4d}
.ms-vr{width:1px;height:20px;background:#ffffff1c}
.ms-opt{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:10px 12px;
 background:#0d1220;border:1px solid #ffffff10;border-radius:11px}
.ms-optt{font-size:13px;font-weight:600}
.ms-sw{position:relative;flex:0 0 40px;width:40px;height:22px;cursor:pointer}
.ms-sw input{position:absolute;opacity:0;width:100%;height:100%;margin:0;cursor:pointer}
.ms-sw span{position:absolute;inset:0;border-radius:999px;background:#232c3f;border:1px solid #ffffff1f;transition:.2s;pointer-events:none}
.ms-sw span:after{content:'';position:absolute;top:2px;left:2px;width:16px;height:16px;border-radius:50%;
 background:#8a94a6;transition:.2s}
.ms-sw input:checked+span{background:linear-gradient(135deg,#2f8fe0,#6d4de6);border-color:#7c9dff66}
.ms-sw input:checked+span:after{left:20px;background:#fff;box-shadow:0 1px 4px #0008}
.ms-actions{display:flex;flex-direction:column;gap:8px;margin-top:10px}
.ms-actions .pri{width:100%}
.ms-actrow{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
.ms-actrow .msb{width:100%;padding:0}
#t_mission .ms-item{border-radius:12px;background:#151c2b;transition:border-color .15s,box-shadow .15s,transform .15s}
#t_mission .ms-item:hover{transform:translateY(-1px);box-shadow:0 4px 14px #00000040}
#t_mission input[type=number],#t_mission .ms-act{transition:border-color .15s,box-shadow .15s;border-radius:8px}
#t_mission input[type=number]:focus,#t_mission .ms-act:focus{outline:none;border-color:#3fa9ff;box-shadow:0 0 0 3px #3fa9ff2b}
#ms_stats{background:transparent;border:none;box-shadow:none;backdrop-filter:none;-webkit-backdrop-filter:none;padding:0;gap:6px}
#ms_stats .ms-chip{background:#0d121cd9;border:1px solid #ffffff22;border-radius:10px;padding:6px 10px;
 backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);box-shadow:0 4px 14px #0009}
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
   <div class="card span2"><h3>Attitude <span class="tag" id="ov_leveltag">--</span> <span class="small dim" id="ov_attmode">solid=corrected &middot; ghost=raw &middot; wire=target</span></h3>
    <canvas id="drone3d" height="350"></canvas>
    <div class="btns" style="margin-top:8px;align-items:center">
     <button class="btn sm pri" id="view_iso" onclick="setAttView('iso')">Pilot 3D</button>
     <button class="btn sm" id="view_top" onclick="setAttView('top')">Top</button>
     <button class="btn sm" id="view_front" onclick="setAttView('front')">Front</button>
     <button class="btn sm" id="view_right" onclick="setAttView('right')">Right</button>
     <button class="btn sm" onclick="resetYawDriftRef()">Zero yaw drift</button>
     <span class="small dim" id="ov_viewnote">world grid is fixed; aircraft moves</span>
    </div>
    <div class="statgrid" style="margin-top:10px">
     <div class="stat"><div class="l">Roll (corr)</div><div class="n" id="ov_roll">--</div></div>
     <div class="stat"><div class="l">Pitch (corr)</div><div class="n" id="ov_pitch">--</div></div>
     <div class="stat"><div class="l">Gyro yaw</div><div class="n" id="ov_yaw">--</div></div>
     <div class="stat"><div class="l">Mag heading</div><div class="n" id="ov_magyaw">--</div></div>
     <div class="stat"><div class="l">Yaw drift / ref</div><div class="n" id="ov_drift">--</div></div>
     <div class="stat"><div class="l">Throttle</div><div class="n" id="ov_thr">--</div></div>
    </div>
   </div>
   <div class="card"><h3>Heading <span class="small dim">external mag</span></h3>
    <div style="display:flex;gap:14px;align-items:center;flex-wrap:wrap">
     <svg width="132" height="132" viewBox="0 0 160 160">
      <circle id="ov_ring" cx="80" cy="80" r="72" fill="none" stroke="#5d6678" stroke-width="6"></circle>
      <g fill="#8a94a6" font-size="12" text-anchor="middle">
       <text x="80" y="20">N</text><text x="80" y="151">S</text><text x="149" y="84">E</text><text x="11" y="84">W</text>
      </g>
      <g id="ov_needle" style="transform-box:fill-box;transform-origin:center;transition:transform .12s linear,opacity .2s">
       <polygon points="80,18 87,84 80,98 73,84" fill="#ff5252"></polygon>
       <polygon points="80,142 87,84 80,70 73,84" fill="#8a94a6"></polygon>
      </g>
      <circle cx="80" cy="80" r="4" fill="#e7ebf2"></circle>
     </svg>
     <div style="min-width:120px">
      <div id="ov_hdg" style="font-size:24px">--&deg;</div>
      <div id="ov_hdgcard" class="dim small">no fix</div>
      <div class="dim small" style="margin-top:8px">Field (µT)</div>
      <div style="height:10px;border-radius:6px;background:#0b0e13;overflow:hidden;margin-top:2px">
       <div id="ov_fieldbar" style="height:100%;width:0;background:#33d17a;transition:width .2s,background .2s"></div>
      </div>
      <div id="ov_field" class="dim small">-- µT</div>
      <div id="ov_magxyz" class="mono small" style="margin-top:7px;line-height:1.5">X --<br>Y --<br>Z --</div>
     </div>
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

 <!-- ===== SETUP / INSTRUMENTS ===== -->
 <section class="tab" id="t_setup">
  <div class="grid wide">
   <div class="card span2"><h3>Instruments <span class="small dim">estimator attitude &middot; a wrong board orientation shows here immediately</span></h3>
    <canvas id="su_tape" height="44"></canvas>
    <div style="display:flex;gap:16px;flex-wrap:wrap;align-items:flex-start;margin-top:10px">
     <canvas id="su_ah" height="330" style="width:330px;max-width:100%;flex:0 0 auto"></canvas>
     <div style="flex:0 0 auto;display:flex;flex-direction:column;align-items:center;gap:4px;padding-top:22px">
      <svg width="190" height="190" viewBox="0 0 160 160">
       <circle id="su_ring" cx="80" cy="80" r="72" fill="none" stroke="#5d6678" stroke-width="6"></circle>
       <g stroke="#5d667855" stroke-width="2">
        <line x1="80" y1="10" x2="80" y2="150"></line><line x1="10" y1="80" x2="150" y2="80"></line>
       </g>
       <circle cx="80" cy="80" r="58" fill="#0a0d12"></circle>
       <g fill="#8a94a6" font-size="12" text-anchor="middle">
        <text x="80" y="20">N</text><text x="80" y="151">S</text><text x="149" y="84">E</text><text x="11" y="84">W</text>
       </g>
       <g id="su_needle" style="transform-box:fill-box;transform-origin:center;transition:transform .12s linear">
        <polygon points="80,26 87,84 80,96 73,84" fill="#ff5252"></polygon>
        <polygon points="80,134 87,84 80,72 73,84" fill="#8a94a6"></polygon>
       </g>
       <circle cx="80" cy="80" r="4" fill="#e7ebf2"></circle>
      </svg>
      <div id="su_hdg" class="mono" style="font-size:22px">--&deg;</div>
      <div id="su_hdgsrc" class="dim small">--</div>
     </div>
     <div style="flex:1;min-width:210px">
      <div id="su_nums"></div>
      <div class="note" style="margin-top:8px">The horizon uses the <b>corrected estimator attitude</b> (same signal the angle controller flies), not raw gyro integration. Tilt the craft by hand: the horizon must move opposite the craft, like a real ADI. The compass follows the active heading source (external mag when valid, else gyro).</div>
     </div>
    </div>
   </div>
   <div class="card"><h3>Arming <span class="tag" id="su_armtag">--</span></h3><div id="su_arm"></div></div>
   <div class="card"><h3>Sensor health</h3><div id="su_sens"></div></div>
   <div class="card span2"><h3>Board alignment test <span class="tag" id="su_altag">idle</span></h3>
    <div class="note">Props off. This guided check confirms the IMU axes match the airframe: tilt the craft and verify the instruments answer in the correct direction. A failed step almost always means a wrong board mounting/rotation.</div>
    <div id="su_alsteps" style="margin:10px 0"></div>
    <div class="btns">
     <button class="btn pri" id="su_albtn" onclick="alignStart()">Start test</button>
     <button class="btn sm" onclick="alignSkip()" id="su_alskip" disabled>Skip step</button>
    </div>
   </div>
  </div>
 </section>

 <!-- ===== MOTORS (wizard + deadman test) ===== -->
 <section class="tab" id="t_motors">
  <div class="warnbox err" style="font-size:14px">&#9888; <b>REMOVE ALL PROPELLERS before any motor test.</b> Motors only spin while a button is physically held; releasing, closing the page or losing WiFi stops them within half a second.</div>
  <div class="grid wide">
   <div class="card span2"><h3>Motor test <span class="tag" id="mo_state">idle</span></h3>
    <div class="row" style="margin-bottom:2px">
     <label style="flex:0 0 auto;display:flex;align-items:center;gap:8px;color:var(--txt)">
      <input type="checkbox" id="mo_ack" style="width:18px;height:18px"> <b>Propellers are removed</b>
     </label>
     <span class="dim small">required before any spin button works</span>
    </div>
    <div class="row"><label>Test value (DShot)</label>
     <input type="number" id="mo_val" min="48" max="800" step="1" style="max-width:110px">
     <button class="btn sm" onclick="moSaveTestVal()">Save default</button>
     <span class="dim small">48&ndash;800 (never 1&ndash;47: DShot special commands)</span>
    </div>
    <div style="display:flex;gap:18px;flex-wrap:wrap;align-items:center;margin-top:8px">
     <canvas id="mo_quad" height="240" style="width:240px;max-width:100%;flex:0 0 auto"></canvas>
     <div style="flex:1;min-width:230px">
      <div class="dim small" style="margin-bottom:6px">Hold a button to spin that OUTPUT (pad). Release = stop.</div>
      <div class="btns" style="gap:10px">
       <button class="btn" id="mo_b1" style="min-width:100px">&#9210; OUT 1</button>
       <button class="btn" id="mo_b2" style="min-width:100px">&#9210; OUT 2</button>
       <button class="btn" id="mo_b3" style="min-width:100px">&#9210; OUT 3</button>
       <button class="btn" id="mo_b4" style="min-width:100px">&#9210; OUT 4</button>
      </div>
      <button class="btn danger" style="width:100%;margin-top:12px;font-size:15px;padding:12px"
       onclick="moStop()">&#9632; STOP ALL MOTORS</button>
      <div class="note" id="mo_note">--</div>
     </div>
    </div>
   </div>
   <div class="card span2"><h3>Motor order wizard <span class="tag" id="mw_tag">idle</span></h3>
    <div class="note">Spins each OUTPUT one at a time; you tell it which corner moved and which way it spins (seen from above). Finish writes the motor map + direction metadata to the config. Props off, craft held or strapped down.</div>
    <div id="mw_body" style="margin-top:8px"><div class="dim small">Not started.</div></div>
    <div class="btns" style="margin-top:8px">
     <button class="btn pri" id="mw_start" onclick="mwStart()">Start wizard</button>
    </div>
   </div>
   <div class="card"><h3>Saved order &amp; direction</h3>
    <table><thead><tr><th>Mixer slot</th><th>Output</th><th>Direction</th></tr></thead>
    <tbody id="mo_maptable"></tbody></table>
    <div class="note">Mixer slots keep the Quad-X geometry (M1 FR, M2 RR, M3 FL, M4 RL); the map re-routes which pad each slot drives. Direction is metadata — reversing a motor requires ESC configuration (AM32/BLHeli), not the FCU.</div>
   </div>
   <div class="card"><h3>Armed idle <span class="tag warn">spins props on arm</span></h3>
    <div class="warnbox">When enabled, arming immediately spins all motors at the idle value — like every commercial FCU. Leave OFF until the airframe passes the props-off checklist.</div>
    <div class="row"><label>Idle on arm</label><input type="checkbox" id="mo_idleen" style="width:18px;height:18px"></div>
    <div class="row"><label>Idle value (DShot)</label><input type="number" id="mo_idleval" min="48" max="300" step="1" style="max-width:110px"></div>
    <div class="btns"><button class="btn pri" onclick="moSaveIdle()">Save idle config</button></div>
    <div class="note">Idle stops on disarm, failsafe and kill switch. Separate from the test value above.</div>
   </div>
  </div>
 </section>

 <!-- ===== MODES (aux-range assignment) ===== -->
 <section class="tab" id="t_modes">
  <div class="grid wide">
   <div class="card span2"><h3>Receiver channels <span class="small dim">live µs</span></h3>
    <div id="md_channels"></div>
   </div>
   <div class="card span2"><h3>Mode slots <span class="tag" id="md_tag">--</span></h3>
    <div class="note">Each slot binds a function to a channel range. A mode is ACTIVE while the
     channel sits inside [min, max]. <b>ARM</b> replaces the fixed CH5 switch (default slot 1
     reproduces it); removing every ARM slot makes arming impossible. <b>KILL</b> stops motors
     instantly and blocks arming while held. ANGLE/ACRO/BEEPER are evaluated and displayed but
     carry no authority yet (this FCU always flies angle-stabilized; no ESC beacon support).</div>
    <table style="margin-top:8px"><thead><tr><th>#</th><th>Function</th><th>Channel</th><th>Min µs</th><th>Max µs</th><th>Live</th><th>State</th></tr></thead>
     <tbody id="md_slots"></tbody></table>
    <div id="md_conflicts" style="margin-top:8px"></div>
    <div class="btns" style="margin-top:8px">
     <button class="btn pri" onclick="mdSave()">Save modes</button>
     <button class="btn" onclick="mdLoad(true)">Reload saved</button>
    </div>
   </div>
   <div class="card"><h3>Active modes</h3><div id="md_active"></div></div>
   <div class="card"><h3>Arming status</h3><div id="md_arm"></div></div>
  </div>
 </section>

 <!-- ===== GPS ===== -->
 <section class="tab" id="t_gps">
  <div class="grid wide">
   <div class="card"><h3>Fix status</h3><div id="gp_status"></div></div>
   <div class="card"><h3>Home <span class="tag" id="gp_hometag">--</span></h3>
    <div id="gp_home"></div>
    <div class="btns" style="margin-top:8px">
     <button class="btn pri" onclick="act('/api/gps/sethome','Home re-captured')">Set home here</button>
    </div>
    <div class="note">Refused unless fix + minimum satellites + HDOP pass the nav quality gates below. Home also auto-captures at boot after the origin debouncer locks.</div>
   </div>
   <div class="card span2"><h3>Receiver configuration <span class="tag" id="gp_cfgtag">--</span></h3>
    <div class="note">UBLOX provider pushes UBX config frames to the module (disarmed only). Generic NMEA is listen-only. Baud changes take effect immediately on Apply; the FCU re-baubs its UART in the same step.</div>
    <div class="row"><label>Provider</label><select id="gp_prov"><option value="0">Generic NMEA (listen only)</option><option value="1">UBLOX</option></select></div>
    <div class="row"><label>Baud</label><select id="gp_baud"><option value="0">9600</option><option value="1">19200</option><option value="2">38400</option><option value="3">57600</option><option value="4">115200</option></select></div>
    <div class="row"><label>Update rate (Hz)</label><input type="number" id="gp_rate" min="1" max="10" step="1" style="max-width:90px"></div>
    <div class="row"><label>Dynamic model</label><select id="gp_dyn"><option value="0">Portable</option><option value="1">Pedestrian</option><option value="2">Automotive</option><option value="3">Airborne &lt;1g</option><option value="4">Airborne &lt;2g</option><option value="5">Airborne &lt;4g</option></select></div>
    <div class="row"><label>SBAS</label><input type="checkbox" id="gp_sbas" style="width:18px;height:18px"></div>
    <div class="row"><label>Auto-config at boot</label><input type="checkbox" id="gp_auto" style="width:18px;height:18px"></div>
    <div class="btns" style="margin-top:8px">
     <button class="btn pri" onclick="gpSave()">Save settings</button>
     <button class="btn" onclick="gpApply()">Apply to receiver now</button>
    </div>
   </div>
   <div class="card span2"><h3>Navigation scaffold <span class="tag" id="nv_tag">--</span></h3>
    <div class="note">The POSHOLD cascade (position error &rarr; velocity setpoint &rarr; tilt command) runs in
     <b>SHADOW</b>: with the POS_HOLD mode switch active and all gates green it holds the engage position and
     computes the tilt it <i>would</i> command — with <b>no motor authority</b>. Validate the shadow output on
     the bench/in flight before authority is wired in. Limits below apply live. Nav PID gains are on the Config tab
     (nav_pos_p, nav_vel_*).</div>
    <div style="display:flex;gap:16px;flex-wrap:wrap;margin-top:8px">
     <div style="flex:1;min-width:200px" id="nv_status"></div>
     <div style="flex:1;min-width:200px" id="nv_shadow"></div>
    </div>
    <div class="row" style="margin-top:6px"><label>Max nav tilt (°)</label><input type="number" id="nv_tilt" min="5" max="45" step="1" style="max-width:90px"></div>
    <div class="row"><label>Max velocity (m/s)</label><input type="number" id="nv_vel" min="0.5" max="15" step="0.5" style="max-width:90px"></div>
    <div class="row"><label>Max acceleration (m/s²)</label><input type="number" id="nv_acc" min="0.5" max="10" step="0.5" style="max-width:90px"></div>
    <div class="row"><label>Stick override (%)</label><input type="number" id="nv_ovr" min="5" max="80" step="5" style="max-width:90px"></div>
    <div class="row"><label>On GPS-quality loss</label><select id="nv_loss" style="max-width:220px"><option value="0">Fall back to ALT HOLD</option><option value="1">Land</option><option value="2">Failsafe</option></select></div>
    <div class="btns"><button class="btn pri" onclick="nvSave()">Save nav limits</button></div>
   </div>
   <div class="card span2"><h3>Nav / arming quality gates</h3>
    <div class="note">POSHOLD/RTH selection while these are unmet blocks arming (NAV_MODE_UNSAFE) and gates Set-home. Arming-time GPS requirements live under <b>arm_require_gps</b> on the Config tab.</div>
    <div class="row"><label>Nav min satellites</label><input type="number" id="gp_navsat" min="5" max="20" step="1" style="max-width:90px"></div>
    <div class="row"><label>Nav max HDOP</label><input type="number" id="gp_navhdop" min="1" max="5" step="0.1" style="max-width:90px"></div>
    <div class="row"><label>Allow arming into unsafe nav mode</label><input type="checkbox" id="gp_navaau" style="width:18px;height:18px"></div>
    <div class="btns"><button class="btn pri" onclick="gpSaveGates()">Save gates</button></div>
   </div>
  </div>
 </section>

 <!-- ===== MISSION PLANNER ===== -->
 <section class="tab" id="t_mission">
  <div id="ms_wrap">
   <div id="ms_mapcard">
    <div id="ms_map"></div>
    <div id="ms_boot"><div class="ms-spin"></div><div class="dim">Preparing map&hellip;</div></div>
    <div id="ms_tools" class="ms-glass">
     <div class="ms-seg" id="ms_layers">
      <button data-l="sat" onclick="msSetLayer('sat')">Satellite</button>
      <button data-l="dark" onclick="msSetLayer('dark')">Dark</button>
      <button data-l="osm" onclick="msSetLayer('osm')">Streets</button>
     </div>
     <span class="ms-vr"></span>
     <button class="msb sm" onclick="msFit()" title="Fit mission in view">&#8982; Fit</button>
     <button class="msb sm" onclick="msUndo()" title="Remove last waypoint">&#8630; Undo</button>
     <button class="msb sm danger" onclick="msClear()">Clear</button>
    </div>
    <div id="ms_stats" class="ms-glass">
     <span class="ms-chip"><span class="k">WPS</span><b id="ms_s_n">0</b></span>
     <span class="ms-chip"><span class="k">DIST</span><b id="ms_s_d">--</b></span>
     <span class="ms-chip"><span class="k">TIME</span><b id="ms_s_t">--</b></span>
     <span class="ms-chip" id="ms_fix"><span class="k">GPS</span><b>--</b></span>
     <span class="ms-chip mono small" id="ms_coords">--</span>
    </div>
    <div id="ms_hint">Click the map to drop WP1 &mdash; drag markers to adjust</div>
   </div>
   <div id="ms_side">
    <div class="card" style="flex:1;display:flex;flex-direction:column;min-height:0">
     <div class="ms-ph"><span>Waypoints</span><span class="ms-badge" id="ms_cnt">0</span></div>
     <div id="ms_list" style="flex:1;overflow:auto;display:flex;flex-direction:column;gap:8px;padding-right:2px"></div>
    </div>
    <div class="card">
     <div class="ms-ph"><span>Mission</span></div>
     <div class="ms-opt">
      <div><div class="ms-optt">Return to home</div><div class="dim small">Fly back after the last waypoint</div></div>
      <label class="ms-sw"><input type="checkbox" id="ms_rth" checked onchange="msRthTgl(this.checked)"><span></span></label>
     </div>
     <div class="ms-actions">
      <button class="msb pri" id="ms_pvbtn" onclick="msPreview()">&#9654;&nbsp; Preview flight</button>
      <div class="ms-actrow">
       <button class="msb" onclick="msExport()">Export</button>
       <button class="msb" onclick="document.getElementById('ms_file').click()">Import</button>
       <button class="msb" onclick="msUpload()">Upload</button>
      </div>
      <input type="file" id="ms_file" accept=".json,application/json" style="display:none" onchange="msImport(this)">
     </div>
     <div class="note" style="margin-top:10px">Preview flies the route at 8&times; real time using each waypoint&rsquo;s
      speed and hold. Missions auto-save in this browser; Export writes an INAV-style JSON you can re-import.
      Home &amp; live position come from FC telemetry when GPS has a fix.</div>
    </div>
   </div>
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
   <div class="card span2"><h3>Six-position accel calibration <span class="tag" id="ac_tag">idle</span></h3>
    <div class="note">Solves accelerometer <b>zero AND gain</b> per axis from six poses (each face of the craft
     pointing up: level, inverted, nose up, nose down, left side, right side — any order, the face is
     auto-detected). Better than the boot-time one-pose offset and supersedes it once saved.
     Keep the craft perfectly still on a firm surface for each capture.</div>
    <div id="ac_faces" style="display:flex;gap:8px;flex-wrap:wrap;margin:10px 0"></div>
    <div id="ac_state" class="note">--</div>
    <div class="btns" style="margin-top:8px">
     <button class="btn pri" onclick="acStart()">Start session</button>
     <button class="btn" id="ac_capbtn" onclick="acCapture()">Capture this pose</button>
     <button class="btn pri" id="ac_finbtn" onclick="acFinish()">Finish &amp; save</button>
     <button class="btn danger" onclick="acCancel()">Cancel</button>
    </div>
    <div class="note mono" id="ac_result">--</div>
   </div>
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
  <div class="warnbox ok" style="margin-bottom:12px">&#128190; All tuning on this page (PID gains, yaw/heading settings, mixer bias, motor trims, failsafe bypass) <b>auto-saves to flash 3&nbsp;s after your last edit</b> — what you tune is what boots. The Save buttons force it immediately; Revert reloads what is in flash.</div>
  <div class="card span2" id="yawcard" style="margin-bottom:12px">
   <h3>Yaw &amp; Heading stability <span class="tag" id="yh_status">--</span></h3>
   <div class="note">Everything that locks your heading, in one place. Heading-hold engages only when all four gates are green <b>and</b> the yaw stick is centered. Edits apply live while disarmed and auto-save to flash 3&nbsp;s after the last change.</div>
   <div class="statgrid" id="yh_gates" style="margin:8px 0"></div>
   <div id="yawknobs"></div>
   <div class="btns" style="margin-top:8px">
    <button class="btn pri" onclick="saveYaw()">Save yaw tuning &rarr; NVS</button>
    <button class="btn" onclick="loadPid();loadMagGainCard()">Reload</button>
   </div>
   <div class="note">Rate Yaw P/I = how hard it resists rotation (the foundation). Angle Yaw P = how firmly it returns to the locked heading. Mag gain = slow compass drift-correction (0 = gyro only). If the compass-cal gate is red, run the mag calibration first.</div>
  </div>
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
    <table><thead><tr><th>Motor</th><th>Base</th><th>Roll</th><th>Pitch</th><th>Yaw</th><th>Trim</th><th>Unclamped</th><th>DShot</th></tr></thead>
     <tbody id="mix_tbody"></tbody></table>
    <div id="mix_expl" class="warnbox ok" style="margin-top:10px">--</div>
   </div>
   <div class="card span2"><h3>Custom mixer table <span class="tag warn">advanced</span></h3>
    <div class="note">Per-motor coefficients the flight mixer applies:
     <span class="mono">out = base·T + roll·R + pitch·P + yaw·Y</span>.
     Defaults are the flown Quad-X geometry. The pitch-front bias (below in PID/Mixer
     settings) multiplies the pitch term of motors with a <b>negative</b> P coefficient
     (the front motors). Values apply atomically between mixer ticks and persist in config.</div>
    <table style="margin-top:8px"><thead><tr><th>Motor</th><th>Throttle (0..2)</th><th>Roll (−2..2)</th><th>Pitch (−2..2)</th><th>Yaw (−2..2)</th></tr></thead>
     <tbody id="mixt_body"></tbody></table>
    <div class="btns" style="margin-top:10px">
     <button class="btn pri" onclick="mixtSave()">Validate &amp; Save</button>
     <button class="btn" onclick="mixtLoad(true)">Reload saved</button>
     <button class="btn danger" onclick="mixtReset()">Reset to Quad-X</button>
    </div>
    <div class="note" id="mixt_note">--</div>
   </div>
   <div class="card span2"><h3>Per-motor thrust trim <span class="tag warn">advanced</span></h3>
    <div class="note">Multiplicative gain on each motor's command <b>above idle</b>. Range 0.900–1.100; 1.000 is off. Armed-idle remains equal. Use only after motor order/direction and level calibration are correct.</div>
    <div class="grid" style="margin-top:10px">
     <div class="row"><label>M1 front-right</label><input id="mix_trim_0" type="number" min="0.9" max="1.1" step="0.005" oninput="motorTrimEdit(0,this.value)"><div class="v" id="mix_trim_now_0">--</div></div>
     <div class="row"><label>M2 rear-right</label><input id="mix_trim_1" type="number" min="0.9" max="1.1" step="0.005" oninput="motorTrimEdit(1,this.value)"><div class="v" id="mix_trim_now_1">--</div></div>
     <div class="row"><label>M3 front-left</label><input id="mix_trim_2" type="number" min="0.9" max="1.1" step="0.005" oninput="motorTrimEdit(2,this.value)"><div class="v" id="mix_trim_now_2">--</div></div>
     <div class="row"><label>M4 rear-left</label><input id="mix_trim_3" type="number" min="0.9" max="1.1" step="0.005" oninput="motorTrimEdit(3,this.value)"><div class="v" id="mix_trim_now_3">--</div></div>
    </div>
    <div class="btns"><button class="btn" onclick="applyMotorTrims()">Apply (RAM)</button><button class="btn pri" onclick="saveMotorTrims()">Save NVS</button><button class="btn danger" onclick="resetMotorTrims()">Reset 1.000</button></div>
    <div id="mix_trim_status" class="note">Right-hand values are the live values confirmed by the FCU.</div>
    <div class="note">Change one side by 0.010–0.020 at a time. Values beyond ±5% indicate a mechanical, CG, prop, ESC, or calibration problem that should be fixed physically.</div>
   </div>
  </div>
 </section>

 <!-- ===== SENSORS ===== -->
 <section class="tab" id="t_sensors">
  <div class="card span2" style="margin-bottom:12px"><h3>Compass calibration <span class="tag" id="mc_tag">idle</span></h3>
   <div class="note">Timed hard-iron + scale capture on the <b>external MMC5603</b>. Start, then rotate the craft
    through every orientation — yaw circles while level, nose-up, nose-down and on both sides. Finishing is refused
    below 6/8 orientation coverage. Keep away from steel benches, chargers and powered ESCs; the battery should be
    mounted in its flight position.</div>
   <div style="display:flex;gap:16px;flex-wrap:wrap;align-items:flex-start;margin-top:10px">
    <canvas id="mc_sphere" height="180" style="width:420px;max-width:100%;flex:0 0 auto"></canvas>
    <div style="flex:1;min-width:220px" id="mc_stats"></div>
   </div>
   <div style="height:10px;border-radius:6px;background:#0b0e13;overflow:hidden;margin-top:8px">
    <div id="mc_prog" style="height:100%;width:0;background:#3fa9ff;transition:width .3s"></div>
   </div>
   <div class="btns" style="margin-top:10px">
    <button class="btn pri" onclick="mcStart()">Start calibration</button>
    <button class="btn" id="mc_finbtn" onclick="mcFinish()">Finish &amp; save</button>
    <button class="btn danger" onclick="mcCancel()">Cancel</button>
    <label class="dim small" style="display:flex;align-items:center;gap:6px;margin-left:8px">Window
     <input type="number" id="mc_window" min="20" max="120" step="5" style="max-width:70px"> s
     <button class="btn sm" onclick="mcSaveWindow()">Set</button></label>
    <label class="dim small" style="display:flex;align-items:center;gap:6px;margin-left:8px">Alignment
     <select id="mc_align" onchange="mcSaveAlign()"><option value="0">None</option><option value="1">CW 90°</option><option value="2">CW 180°</option><option value="3">CW 270°</option><option value="4">Flip</option><option value="5">Flip + CW 90°</option><option value="6">Flip + CW 180°</option><option value="7">Flip + CW 270°</option></select></label>
   </div>
   <div id="mc_warn"></div>
  </div>
  <div class="card span2" style="margin-bottom:12px"><h3>Altitude sources <span class="tag" id="al_tag">--</span></h3>
   <div style="display:flex;gap:16px;flex-wrap:wrap">
    <div style="flex:1;min-width:220px" id="al_baro"></div>
    <div style="flex:1;min-width:220px" id="al_tof"></div>
   </div>
   <div class="btns" style="margin-top:10px">
    <button class="btn pri" onclick="act('/api/baro/zero','Zeroing altitude (averaging 10 samples)…')">Zero altitude here</button>
    <label class="dim small" style="display:flex;align-items:center;gap:6px">Priority
     <select id="al_src" onchange="alSaveCfg()"><option value="0">Auto (ToF &gt; baro &gt; GPS)</option><option value="1">Baro only</option><option value="2">Rangefinder only</option></select></label>
    <label class="dim small" style="display:flex;align-items:center;gap:6px">
     <input type="checkbox" id="al_med" onchange="alSaveCfg()" style="width:16px;height:16px"> ToF median filter</label>
   </div>
   <div class="note">Unified altitude feeds telemetry and the navigation scaffold; the flight altitude-hold
    controller keeps its proven direct-ToF path. GPS altitude is absolute MSL and used as a reference only.
    Baro temperature-drift compensation is not implemented yet — re-zero after large temperature changes.</div>
  </div>
  <div class="card"><h3>Detected subsystems</h3>
   <table><thead><tr><th>Device</th><th>Bus</th><th>State</th><th>Value</th><th>Age</th></tr></thead>
    <tbody id="sens_tbody"></tbody></table>
   <div class="note">Green = healthy &amp; fresh &middot; Yellow = degraded/stale/uncalibrated &middot; Red = missing/invalid/timeout &middot; Grey = not compiled in. A driver being initialized is <i>not</i> the same as it returning valid recent data.</div>
  </div>
 </section>

 <!-- ===== RADIO & LINKS ===== -->
 <section class="tab" id="t_radio">
  <div class="grid wide">
   <div class="card span2"><h3>Channel map <span class="small dim">live µs &middot; 900–2100 &middot; tick = 1500 center</span></h3>
    <div id="rc_chans"></div>
    <div class="note">Function labels come from the fixed stick map (CH1–4), the gimbal channels, and your Modes-page assignments (shown with their range; the bar segment lights green while the mode is active). Assign or change ranges on the <a href="#" onclick="showTab('modes');return false">Modes</a> tab.</div>
   </div>
   <div class="card"><h3>Control receiver (CRSF/ELRS)</h3><div id="rc_info"></div></div>
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
   <div class="card span2"><h3>Gyro / Accel / Setpoint filters <span class="tag" id="ft_tag">--</span></h3>
    <div class="note">The full chain the rate PIDs see: dynamic notch (below) &rarr; <b>gyro LPF1</b> &rarr; <b>gyro LPF2</b>.
     Changes apply atomically between flight ticks and persist immediately. Loop <b id="ft_loop">--</b> Hz —
     filters derive from the measured dt, but <b>PID behavior still changes if the loop rate changes</b>; retune after altering loop timing.</div>
    <div class="grid" style="margin-top:8px">
     <div>
      <div class="row"><label>Gyro LPF1 type</label><select id="ft_g1t"><option value="0">Off</option><option value="1">PT1</option><option value="2">Biquad</option></select></div>
      <div class="row"><label>Gyro LPF1 Hz</label><input type="number" id="ft_g1h" min="10" max="500" step="5" style="max-width:90px"></div>
      <div class="row"><label>Dyn min Hz (0=static)</label><input type="number" id="ft_gdmin" min="0" max="500" step="5" style="max-width:90px"></div>
      <div class="row"><label>Dyn max Hz</label><input type="number" id="ft_gdmax" min="0" max="750" step="5" style="max-width:90px"></div>
      <div class="row"><label>Dyn expo (0-10)</label><input type="number" id="ft_gdexp" min="0" max="10" step="1" style="max-width:90px"></div>
      <div class="row"><label>Gyro LPF2 type</label><select id="ft_g2t"><option value="0">Off</option><option value="1">PT1</option><option value="2">Biquad</option></select></div>
      <div class="row"><label>Gyro LPF2 Hz</label><input type="number" id="ft_g2h" min="10" max="500" step="5" style="max-width:90px"></div>
     </div>
     <div>
      <div class="row"><label>Accel LPF type</label><select id="ft_at"><option value="1">PT1</option><option value="2">Biquad</option></select></div>
      <div class="row"><label>Accel LPF Hz</label><input type="number" id="ft_ah" min="1" max="200" step="1" style="max-width:90px"></div>
      <div class="row"><label>Accel notch Hz (0=off)</label><input type="number" id="ft_anh" min="0" max="400" step="5" style="max-width:90px"></div>
      <div class="row"><label>Accel notch Q</label><input type="number" id="ft_anq" min="0.5" max="10" step="0.5" style="max-width:90px"></div>
      <div class="row"><label>D-term LPF Hz (0=off)</label><input type="number" id="ft_dh" min="0" max="400" step="5" style="max-width:90px"></div>
      <div class="row"><label>Setpoint LPF Hz (0=off)</label><input type="number" id="ft_sh" min="0" max="200" step="5" style="max-width:90px"></div>
     </div>
    </div>
    <div class="warnbox" id="ft_dynnote" style="display:none">Dynamic cutoff requires the PT1 type — a biquad here runs static at the base cutoff.</div>
    <div class="btns" style="margin-top:8px">
     <button class="btn pri" onclick="ftApply()">Apply &amp; Save filters</button>
     <button class="btn" onclick="ftLoad(true)">Reload saved</button>
    </div>
    <div class="note mono" id="ft_live">raw vs filtered: --</div>
   </div>
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
  <div class="card span2" style="margin-bottom:12px"><h3>Blackbox flight log <span class="tag" id="bb_tag">--</span></h3>
   <div class="note">Compact ring log of the whole control path (raw/filtered gyro, accel, attitude, PID outputs,
    motors, mode, arming flags, GPS, mag field, unified altitude) — always keeps the <b>most recent</b>
    ~15 s at 50 Hz, so the tail survives an incident. Records while motors run when "log in flight" is on,
    while a BLACKBOX mode switch is active, or during a manual bench recording. Download stops recording first.</div>
   <div id="bb_stats" style="margin:8px 0"></div>
   <div class="btns">
    <button class="btn pri" onclick="bbAct('start','Recording')">Start (bench)</button>
    <button class="btn" onclick="bbAct('stop','Stopped')">Stop</button>
    <button class="btn danger" onclick="bbClear()">Clear</button>
    <button class="btn" onclick="window.location.href='/api/blackbox.csv'">Download CSV</button>
    <label class="dim small" style="display:flex;align-items:center;gap:6px;margin-left:8px">
     <input type="checkbox" id="bb_en" onchange="bbSaveCfg()" style="width:16px;height:16px"> log in flight</label>
    <label class="dim small" style="display:flex;align-items:center;gap:6px">rate
     <input type="number" id="bb_rate" min="10" max="250" step="10" style="max-width:70px" onchange="bbSaveCfg()"> Hz</label>
   </div>
  </div>
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
   <div class="card"><h3>External magnetometer calibration</h3>
    <div class="note">Capture hard-iron and coarse scale for the I2C MMC5603 from a full all-axis rotation. Stored in NVS when finished.</div>
    <div class="btns" style="margin-top:8px"><button class="btn" onclick="act('/api/mag/start','Mag capture started')">Start mag capture</button><button class="btn pri" onclick="act('/api/mag/finish','Mag calibration saved')">Finish &amp; save</button></div>
   </div>
   <div class="card"><h3>Compass &amp; heading trim</h3>
    <div class="note">Live heading from the <b>external</b> magnetometer (the onboard IMU mag is never used for heading/yaw). Trim corrects the small residual after a good mag cal (and applies declination). <b>Save</b> persists to NVS; writes need disarmed.</div>
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
    <div class="btns" style="margin-top:8px"><button class="btn pri" onclick="saveMagTrim()">Save trim NVS</button><button class="btn danger" onclick="resetMagTrim()">Reset trim to 0&deg;</button><button class="btn" onclick="act('/api/calibrate?mag=1','Spin craft on all axes for 30 s...')">Start mag cal</button></div>
   </div>
   <div class="card"><h3>External-mag yaw correction</h3>
    <div class="note">Corrects long-term <b>yaw estimate drift</b> using the MMC5603. The gyro remains the fast yaw-rate PID sensor. Heading hold activates only when external-mag calibration is valid, correction gain is above zero, Angle Yaw P is above zero, and the yaw stick is centered.</div>
    <div id="cfg_magfusion" style="margin:8px 0"></div>
    <div class="row"><label>Correction gain</label><input type="range" id="mag_gain" min="0" max="1" step="0.05" value="0" oninput="document.getElementById('mag_gainn').value=this.value"><input type="number" id="mag_gainn" min="0" max="1" step="0.05" value="0" style="width:70px" oninput="document.getElementById('mag_gain').value=this.value"></div>
    <div class="btns" style="margin-top:8px">
     <button class="btn" onclick="setMagGain(0)">Shadow 0.00</button>
     <button class="btn" onclick="setMagGain(0.10)">Low 0.10</button>
     <button class="btn" onclick="setMagGain(0.25)">Medium 0.25</button>
     <button class="btn" onclick="applyMagGain()">Apply (RAM)</button>
     <button class="btn pri" onclick="saveMagGain()">Save NVS</button>
    </div>
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
const TABS=[['overview','Overview'],['setup','Setup'],['motors','Motors'],['modes','Modes'],['gps','GPS'],['mission','Mission'],['attitude','Attitude & Level'],['pid','PID Tuning'],
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
let accelOffSince=0;
function computeWarnings(m){
 const w=[];const st=m.sys,a=m.att,im=m.imu,lv=m.lvl,pid=m.pid,sen=m.sen,rc=m.rc;
 const stationary=!st.armed && st.thr===0;
 if(st.fs)w.push(['err','Failsafe ACTIVE (reason '+st.fsr+')']);
 if(!st.link && rc.comp)w.push(['err','Control link DOWN']);
 if(rc.comp && rc.up && rc.lq<50 && rc.lq>0)w.push(['warn','Low RC link quality: '+rc.lq+'%']);
 if(rc.comp && rc.fs)w.push(['warn','Receiver reports failsafe (LQ 0)']);
 if(st.loop>0 && st.loop<400)w.push(['warn','Loop rate low: '+st.loop+' Hz']);
 if(st.loop>0 && st.loop<400 && st.ovr>0)w.push(['warn','Flight loop overruns since boot: '+st.ovr]);
 if(!im.rdy)w.push(['err','IMU not ready']);
 const accelOff=im.rdy&&(im.am<0.85||im.am>1.15);
 const gyroQuiet=im.rdy&&Math.max(Math.abs(im.g[0]),Math.abs(im.g[1]),Math.abs(im.g[2]))<3;
 if(!st.armed&&gyroQuiet&&accelOff){
  if(!accelOffSince)accelOffSince=Date.now();
  if(Date.now()-accelOffSince>750)w.push(['warn','Accel magnitude persistently off while stationary: '+f(im.am,3)+' g']);
 }else accelOffSince=0;
 if(im.gbv){const gb=Math.max(Math.abs(im.gb[0]),Math.abs(im.gb[1]),Math.abs(im.gb[2]));
  if(gb>5)w.push(['warn','Large gyro bias: '+f(gb,2)+' °/s']);}
 if(!lv.ld)w.push(['warn','No saved level calibration loaded — run Calibrate Level']);
 if(st.armed&&!a.at)w.push(['warn','Accel correction temporarily gated by flight acceleration/vibration']);
 if(Math.abs(pid.r[1])>100)w.push(['err','Roll integrator wound up: I='+f(pid.r[1],1)+' - disarm to reset']);
 if(Math.abs(pid.p[1])>100)w.push(['err','Pitch integrator wound up: I='+f(pid.p[1],1)+' - disarm to reset']);
 if(stationary && (Math.abs(a.cr)>2||Math.abs(a.cp)>2))
  w.push(['err','Corrected attitude not level while still: roll '+f(a.cr,1)+'° pitch '+f(a.cp,1)+'° — calibrate level']);
 // motor spread before throttle
 if(st.thr===0){const mm=m.mix.m;const sp=Math.max(...mm)-Math.min(...mm);
  if(sp>4 && Math.max(...mm)>0)w.push(['warn','Motors unequal at zero throttle (spread '+sp+')']);}
 if(pid.smax)w.push(['warn','Mixer upper saturation']);
 if(pid.ssc)w.push(['warn','Mixer correction scaled (severe)']);
 if(m.mix.trim&&m.mix.trim.some(v=>Math.abs(v-1)>0.0501))w.push(['warn','Motor thrust trim exceeds 5% — inspect mechanics/CG']);
 if(sen.baro.r&&sen.baro.age>2000)w.push(['warn','Baro stale ('+ageTxt(sen.baro.age)+')']);
 if(sen.gps.comp&&sen.gps.r&&!sen.gps.fix)w.push(['warn','GPS connected, no fix']);
 if(sen.tof.comp&&sen.tof.r&&sen.tof.age>1500)w.push(['warn','ToF stale']);
 if(sen.mag&&sen.mag.gain>0&&sen.mag.v!==1)w.push(['warn','External-mag yaw correction enabled but compass is invalid']);
 if(sen.mag&&sen.mag.ext&&sen.mag.ext.v===1&&!sen.mag.ext.cal)w.push(['warn','External magnetometer is not calibrated — compass will not match reliably']);
 if(sen.mag&&Math.abs(+sen.mag.trim||0)>45)w.push(['err','Large saved compass trim: '+f(sen.mag.trim,1)+'° — reset trim and calibrate the external mag']);
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
 document.getElementById('ov_yaw').textContent=f(m.att.ry,1)+'°'+(m.att.yh?' HOLD':'');
 updateYawDrift(m);
 document.getElementById('ov_thr').textContent=st.thr+'%';
 renderHealth(m);renderWarn(warns);renderOvRc(m);renderOvServo(m);
 setDroneTarget(m.att,m.sen&&m.sen.mag);renderOvHeading(m);drawMix(document.getElementById('ov_mix'),m,true);
 // active tab content
 if(activeTab==='setup')renderSetup(m);
 else if(activeTab==='modes')renderModes(m);
 else if(activeTab==='gps')renderGps(m);
 else if(activeTab==='attitude')renderAttitude(m);
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
// overview compass — external mag heading (mirrors the Config-tab dial; the
// onboard IMU mag is never selected, see firmware selectActiveMagSource).
let ovMagAcc=0;
function renderOvHeading(m){
 const g=m.sen&&m.sen.mag;if(!g)return;
 const e=g.ext||{},ok=g.v===1&&g.src===2,hdg=+(e.hdg??g.hdg)||0,fld=+(e.f??g.f)||0,c=magColour(fld);
 const xyz=Array.isArray(e.xyz)?e.xyz:[0,0,0];
 ovMagAcc+=((hdg-ovMagAcc)%360+540)%360-180;
 const nd=document.getElementById('ov_needle');if(nd){nd.style.transform='rotate('+ovMagAcc+'deg)';nd.style.opacity=ok?'1':'0.22';}
 const ht=document.getElementById('ov_hdg');if(ht)ht.textContent=ok?hdg.toFixed(0)+'° '+compassPoint(hdg):'--°';
 const ms=document.getElementById('ov_hdgcard');if(ms){ms.textContent=ok?'external · valid':('external · '+magRejectName(e.rej));ms.style.color=ok?'':'#ffb020';}
 const rg=document.getElementById('ov_ring');if(rg)rg.setAttribute('stroke',ok?c:'#5d6678');
 const bar=document.getElementById('ov_fieldbar');if(bar){bar.style.width=Math.max(0,Math.min(100,fld/120*100))+'%';bar.style.background=c;}
 const ft=document.getElementById('ov_field');if(ft)ft.textContent=f(fld,1)+' µT';
 const xt=document.getElementById('ov_magxyz');if(xt)xt.innerHTML=
  '<span style="color:#ff5252">X/FWD '+f(xyz[0],1)+'</span><br><span style="color:#33d17a">Y/RIGHT '+f(xyz[1],1)+'</span><br><span style="color:#3fa9ff">Z/DOWN '+f(xyz[2],1)+'</span>';
}
)DASH"
R"DASH(
// ---------- 3D attitude viewer ----------
// Fixed world frame: N/E horizontal, D down. The grid and cyan level ring never
// rotate; only the aircraft does. This makes a level error visually obvious.
let show={solid:true,ghost:true,wire:true},ATT_VIEW='iso';
let yawDriftRef=null,yawDriftNow=null,lastMagHeading=null,lastYawHeading=null;
let _tiltTrail=[],_trailMs=0;
function matMul(a,b){const r=[0,0,0,0,0,0,0,0,0];for(let i=0;i<3;i++)for(let j=0;j<3;j++){let s=0;
 for(let k=0;k<3;k++)s+=a[i*3+k]*b[k*3+j];r[i*3+j]=s;}return r;}
function mv(a,v){return [a[0]*v[0]+a[1]*v[1]+a[2]*v[2],a[3]*v[0]+a[4]*v[1]+a[5]*v[2],a[6]*v[0]+a[7]*v[1]+a[8]*v[2]];}
function add3(a,b){return [a[0]+b[0],a[1]+b[1],a[2]+b[2]];}
function Rx(t){const c=Math.cos(t),s=Math.sin(t);return [1,0,0,0,c,-s,0,s,c];}
function Ry(t){const c=Math.cos(t),s=Math.sin(t);return [c,0,s,0,1,0,-s,0,c];}
function Rz(t){const c=Math.cos(t),s=Math.sin(t);return [c,-s,0,s,c,0,0,0,1];}
const D2R=Math.PI/180,CAM_ISO=matMul(Rx(-62*D2R),Ry(28*D2R)),MODEL_O=[0,0,-0.62];
function wrap180d(v){return ((v+180)%360+360)%360-180;}
function attMat(roll,pitch,yaw){return matMul(Rz(yaw*D2R),matMul(Ry(pitch*D2R),Rx(roll*D2R)));}
function setAttView(v){
 ATT_VIEW=v;
 ['iso','top','front','right'].forEach(x=>{const b=document.getElementById('view_'+x);if(b)b.classList.toggle('pri',x===v);});
 const notes={iso:'fixed world grid · chase/isometric camera',top:'top view · yaw and north/east',front:'front view · roll against horizon',right:'right view · pitch against horizon'};
 const n=document.getElementById('ov_viewnote');if(n)n.textContent=notes[v]||'';
 if(_attDisp)drawDrone({att:_attDisp});
}
function proj(p,cx,cy,sc){
 if(ATT_VIEW==='top')return [cx+p[1]*sc,cy-p[0]*sc,-p[2]];
 if(ATT_VIEW==='front')return [cx+p[1]*sc,cy+p[2]*sc,p[0]];
 if(ATT_VIEW==='right')return [cx+p[0]*sc,cy+p[2]*sc,-p[1]];
 const g=[p[1],-p[2],p[0]],c=mv(CAM_ISO,g);return [cx+c[0]*sc,cy-c[1]*sc,c[2]];
}
function bodyWorld(R,v){return add3(MODEL_O,mv(R,v));}
function canvasCtx(cv){
 const W=Math.max(320,cv.clientWidth|0),H=+(cv.getAttribute('height')||350),d=Math.min(window.devicePixelRatio||1,2);
 if(cv.width!==Math.round(W*d)||cv.height!==Math.round(H*d)){cv.width=Math.round(W*d);cv.height=Math.round(H*d);}
 const ctx=cv.getContext('2d');ctx.setTransform(d,0,0,d,0,0);ctx.clearRect(0,0,W,H);return {ctx,W,H};
}
function path3(ctx,pts,cx,cy,sc,close){
 if(!pts.length)return;const a=proj(pts[0],cx,cy,sc);ctx.beginPath();ctx.moveTo(a[0],a[1]);
 for(let i=1;i<pts.length;i++){const p=proj(pts[i],cx,cy,sc);ctx.lineTo(p[0],p[1]);}if(close)ctx.closePath();
}
function circle3(ctx,center,r,R,cx,cy,sc){
 const q=[];for(let i=0;i<=40;i++){const t=i/40*Math.PI*2,local=[center[0]+Math.cos(t)*r,center[1]+Math.sin(t)*r,center[2]];
  q.push(R?bodyWorld(R,local):add3(MODEL_O,local));}path3(ctx,q,cx,cy,sc,false);
}
function drawWorld(ctx,cx,cy,sc,tilt){
 // Ground plane below the model.
 const z=.68,s=3.2,step=.8,corners=[[-s,-s,z],[s,-s,z],[s,s,z],[-s,s,z]];
 path3(ctx,corners,cx,cy,sc,true);ctx.fillStyle='#0c1620';ctx.fill();ctx.strokeStyle='#1c3344';ctx.lineWidth=1;ctx.stroke();
 ctx.strokeStyle='#173044';ctx.lineWidth=.8;
 for(let v=-s;v<=s+.01;v+=step){path3(ctx,[[-s,v,z],[s,v,z]],cx,cy,sc);ctx.stroke();path3(ctx,[[v,-s,z],[v,s,z]],cx,cy,sc);ctx.stroke();}
 // World-level reference ring through the aircraft centre.
 const ring=[];for(let i=0;i<=64;i++){const t=i/64*Math.PI*2;ring.push([MODEL_O[0]+Math.cos(t)*1.72,MODEL_O[1]+Math.sin(t)*1.72,MODEL_O[2]]);}
 path3(ctx,ring,cx,cy,sc);ctx.setLineDash([6,4]);ctx.strokeStyle=tilt<.5?'#33d17a':(tilt<2?'#ffb020':'#ff5252');ctx.lineWidth=1.6;ctx.stroke();ctx.setLineDash([]);
 const axes=[[[0,0,z],[2.8,0,z],'#66d9ef','N'],[[0,0,z],[0,2.8,z],'#ffd166','E']];
 ctx.font='bold 11px sans-serif';
 for(const[a,b,col,n]of axes){path3(ctx,[a,b],cx,cy,sc);ctx.strokeStyle=col;ctx.lineWidth=2;ctx.stroke();const p=proj(b,cx,cy,sc);ctx.fillStyle=col;ctx.fillText(n,p[0]+4,p[1]-3);}
}
function droneModel(){const L=1.0;return {M1:[L,L,0],M3:[L,-L,0],M2:[-L,L,0],M4:[-L,-L,0]};}
function drawFrame(ctx,R,cx,cy,sc,style){
 const m=droneModel(),motors=[['M1',m.M1,'#ff5252'],['M2',m.M2,'#ffb020'],['M3',m.M3,'#33d17a'],['M4',m.M4,'#3fa9ff']];
 const bc=proj(bodyWorld(R,[0,0,0]),cx,cy,sc);
 ctx.lineWidth=style.lw;ctx.strokeStyle=style.arm;
 for(const[,pos]of motors){const p=proj(bodyWorld(R,pos),cx,cy,sc);ctx.beginPath();ctx.moveTo(bc[0],bc[1]);ctx.lineTo(p[0],p[1]);ctx.stroke();}
 // Directional body plate: pointed nose makes yaw readable without relying on axes.
 const plate=[[.70,0,0],[.05,.38,0],[-.48,.30,0],[-.48,-.30,0],[.05,-.38,0]].map(v=>bodyWorld(R,v));
 path3(ctx,plate,cx,cy,sc,true);ctx.fillStyle=style.body;ctx.fill();ctx.strokeStyle=style.nose;ctx.lineWidth=style.lw;ctx.stroke();
 for(const[name,pos,col]of motors){circle3(ctx,pos,.40,R,cx,cy,sc);ctx.strokeStyle=style.solid?col:style.arm;ctx.lineWidth=style.lw;ctx.stroke();
  if(style.solid){const p=proj(bodyWorld(R,pos),cx,cy,sc);ctx.fillStyle=col;ctx.font='bold 11px sans-serif';ctx.fillText(name,p[0]-7,p[1]+4);}}
}
function drawAxes(ctx,R,cx,cy,sc){
 const o=proj(bodyWorld(R,[0,0,0]),cx,cy,sc),ax=[[[1.85,0,0],'#ff5252','FWD'],[[0,1.85,0],'#33d17a','RIGHT'],[[0,0,1.35],'#3fa9ff','DOWN']];
 ctx.font='bold 10px sans-serif';
 for(const[v,c,n]of ax){const p=proj(bodyWorld(R,v),cx,cy,sc);ctx.strokeStyle=c;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(o[0],o[1]);ctx.lineTo(p[0],p[1]);ctx.stroke();ctx.fillStyle=c;ctx.fillText(n,p[0]+4,p[1]-2);}
}
function drawArrow2(ctx,a,b,col,label,dashed){
 ctx.save();ctx.strokeStyle=col;ctx.fillStyle=col;ctx.lineWidth=2.5;if(dashed)ctx.setLineDash([5,4]);
 ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();ctx.setLineDash([]);
 const ang=Math.atan2(b[1]-a[1],b[0]-a[0]),sz=8;ctx.beginPath();ctx.moveTo(b[0],b[1]);
 ctx.lineTo(b[0]-sz*Math.cos(ang-.45),b[1]-sz*Math.sin(ang-.45));ctx.lineTo(b[0]-sz*Math.cos(ang+.45),b[1]-sz*Math.sin(ang+.45));ctx.closePath();ctx.fill();
 if(label){ctx.font='bold 10px sans-serif';ctx.fillText(label,b[0]+5,b[1]-5);}ctx.restore();
}
function drawMagVector(ctx,R,mag,cx,cy,sc){
 if(!mag||!mag.valid||!Array.isArray(mag.xyz))return;
 const v=mag.xyz.map(Number),norm=Math.hypot(v[0],v[1],v[2]);if(!(norm>1))return;
 const unit=[v[0]/norm,v[1]/norm,v[2]/norm],world=mv(R,unit),o3=MODEL_O,end3=add3(o3,[world[0]*2.05,world[1]*2.05,world[2]*2.05]);
 const o=proj(o3,cx,cy,sc),end=proj(end3,cx,cy,sc);drawArrow2(ctx,o,end,'#e879f9','MAG XYZ',false);
 const h=Math.hypot(world[0],world[1]);if(h>.05){const hp=add3(o3,[world[0]/h*1.72,world[1]/h*1.72,0]);drawArrow2(ctx,o,proj(hp,cx,cy,sc),'#f0abfc','HORIZONTAL',true);}
}
function drawLevelInset(ctx,W,a){
 const x=W-58,y=57,r=38,max=10;ctx.fillStyle='#0b111aDD';ctx.beginPath();ctx.arc(x,y,r+8,0,7);ctx.fill();
 ctx.strokeStyle='#445267';ctx.lineWidth=1;ctx.beginPath();ctx.arc(x,y,r,0,7);ctx.moveTo(x-r,y);ctx.lineTo(x+r,y);ctx.moveTo(x,y-r);ctx.lineTo(x,y+r);ctx.stroke();
 ctx.strokeStyle='#24354a';for(const q of [r*.5]){ctx.beginPath();ctx.arc(x,y,q,0,7);ctx.stroke();}
 if(_tiltTrail.length>1){ctx.strokeStyle='#5d8fbf88';ctx.lineWidth=1;ctx.beginPath();_tiltTrail.forEach((p,i)=>{const px=x+Math.max(-1,Math.min(1,p[0]/max))*r,py=y-Math.max(-1,Math.min(1,p[1]/max))*r;i?ctx.lineTo(px,py):ctx.moveTo(px,py);});ctx.stroke();}
 const dx=Math.max(-1,Math.min(1,a.cr/max))*r,dy=-Math.max(-1,Math.min(1,a.cp/max))*r,tilt=Math.hypot(a.cr,a.cp);
 ctx.fillStyle=tilt<.5?'#33d17a':(tilt<2?'#ffb020':'#ff5252');ctx.beginPath();ctx.arc(x+dx,y+dy,5,0,7);ctx.fill();
 ctx.fillStyle='#aeb8c8';ctx.font='10px sans-serif';ctx.fillText('LEVEL ±10°',x-30,y+r+16);
}
function drawDrone(m){
 const cv=document.getElementById('drone3d');if(!cv)return;const z=canvasCtx(cv),ctx=z.ctx,W=z.W,H=z.H;
 const cx=W/2,cy=H/2-2,sc=Math.min(W,H)/(ATT_VIEW==='iso'?6.1:6.5),a=m.att,tilt=Math.hypot(a.cr,a.cp);
 drawWorld(ctx,cx,cy,sc,tilt);
 if(show.wire){const Rt=attMat(a.tr,a.tp,a.yh?a.ty:a.ry);drawFrame(ctx,Rt,cx,cy,sc,{lw:1,arm:'#a06cff66',body:'#7c5cff18',nose:'#a06cff99',solid:false});}
 if(show.ghost){const Rr=attMat(a.rr,a.rp,a.ry);drawFrame(ctx,Rr,cx,cy,sc,{lw:1.3,arm:'#8a94a655',body:'#8a94a622',nose:'#8a94a6aa',solid:false});}
 if(show.solid){const Rc=attMat(a.cr,a.cp,a.ry);drawFrame(ctx,Rc,cx,cy,sc,{lw:2.8,arm:'#d6deea',body:'#26344a',nose:'#ffffff',solid:true});drawAxes(ctx,Rc,cx,cy,sc);drawMagVector(ctx,Rc,_magDisp,cx,cy,sc);}
 drawLevelInset(ctx,W,a);
 ctx.font='11px sans-serif';ctx.fillStyle='#8a94a6';ctx.fillText('WORLD: N cyan · E amber · level ring fixed',8,H-24);
 ctx.fillText('BODY: FWD red · RIGHT green · DOWN blue',8,H-8);
}
function updateYawDrift(m){
 const g=m.sen&&m.sen.mag,e=g&&g.ext||{},ok=g&&g.v===1&&g.src===2&&e.v===1;
 const mh=ok?+(e.hdg??g.hdg):null,y=+m.att.ry||0;lastMagHeading=mh;lastYawHeading=y;
 if(ok){const delta=wrap180d(mh-y);if(yawDriftRef===null)yawDriftRef=delta;yawDriftNow=wrap180d(delta-yawDriftRef);}
 else yawDriftNow=null;
 const me=document.getElementById('ov_magyaw'),de=document.getElementById('ov_drift');
 if(me)me.textContent=ok?f(mh,1)+'° '+compassPoint(mh):'--';if(de)de.textContent=yawDriftNow==null?'--':((yawDriftNow>=0?'+':'')+f(yawDriftNow,1)+'°');
 const tilt=Math.hypot(+m.att.cr||0,+m.att.cp||0),tag=document.getElementById('ov_leveltag');
 if(tag){tag.className='tag '+(tilt<.5?'ok':(tilt<2?'warn':'err'));tag.textContent=tilt<.5?'LEVEL':('TILT '+f(tilt,1)+'°');}
}
function resetYawDriftRef(){
 if(lastMagHeading==null||lastYawHeading==null){toast('External mag is not valid',true);return;}
 yawDriftRef=wrap180d(lastMagHeading-lastYawHeading);yawDriftNow=0;toast('Yaw drift reference zeroed');
}
// Smooth telemetry at display refresh rate and retain a short roll/pitch trail.
let _attTgt=null,_attDisp=null,_attRAF=0,_magDisp=null;
function setDroneTarget(a,mag){
 if(!a)return;_attTgt={cr:+a.cr||0,cp:+a.cp||0,ry:+a.ry||0,rr:+a.rr||0,rp:+a.rp||0,tr:+a.tr||0,tp:+a.tp||0,ty:+a.ty||0,yh:a.yh===1};
 const e=mag&&mag.ext||{};_magDisp={valid:mag&&mag.v===1&&mag.src===2&&e.v===1,xyz:Array.isArray(e.xyz)?e.xyz.slice(0,3):null};
 const now=Date.now();if(now-_trailMs>180){_trailMs=now;_tiltTrail.push([_attTgt.cr,_attTgt.cp]);if(_tiltTrail.length>70)_tiltTrail.shift();}
 if(!_attDisp)_attDisp=Object.assign({},_attTgt);if(!_attRAF)_attRAF=requestAnimationFrame(_attStep);
}
function _attStep(){
 _attRAF=0;if(!_attTgt||!_attDisp)return;const K=.26;let moving=false;
 for(const k of ['cr','cp','rr','rp','tr','tp']){const d=_attTgt[k]-_attDisp[k];_attDisp[k]+=d*K;if(Math.abs(d)>.03)moving=true;}
 const dy=wrap180d(_attTgt.ry-_attDisp.ry);_attDisp.ry+=dy*K;if(Math.abs(dy)>.03)moving=true;
 const vis=activeTab==='overview'&&!document.hidden;if(vis)drawDrone({att:_attDisp});if(moving||vis)_attRAF=requestAnimationFrame(_attStep);
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
 const tr=m.mix.trim||[1,1,1,1],trTxt=tr.some(v=>Math.abs(v-1)>.0005)?' Trims: '+tr.map((v,i)=>'M'+(i+1)+'='+f(v,3)).join(' ')+'.':'';
 const ax=Math.abs(m.pid.r[3])>=Math.abs(m.pid.p[3])?'roll':'pitch';
 const t=ax==='roll'?m.pid.r:m.pid.p,ai=ax==='roll'?0:1;
 const src=' '+ax.toUpperCase()+': target '+f(ai?m.att.tp:m.att.tr,2)+' deg, corrected '+f(ai?m.att.cp:m.att.cr,2)+
  ' deg, rate SP '+f(m.pid.sp[ai],1)+' deg/s, gyro '+f(m.imu.g[ai],1)+' deg/s, P/I/D '+
  f(t[0],1)+'/'+f(t[1],1)+'/'+f(t[2],1)+'.';
 return[(sp>40?'':'ok'),'Spread '+sp+' DShot. Dominant PID axis: '+axis+'. Higher pair: '+side+'.'+src+trTxt];
}
let motorTrimDraft=[1,1,1,1],motorTrimDirty=[false,false,false,false],motorTrimPending=null;
function setMotorTrimStatus(msg,err){
 const e=document.getElementById('mix_trim_status');if(!e)return;e.textContent=msg;e.style.color=err?'#ffb020':'';
}
function motorTrimEdit(i,value){
 const v=parseFloat(value);if(Number.isFinite(v))motorTrimDraft[i]=v;
 motorTrimDirty[i]=true;motorTrimPending=null;setMotorTrimStatus('Edited values are not yet applied.');
}
function motorTrimMatches(a,b){
 return Array.isArray(a)&&Array.isArray(b)&&a.length===4&&b.length===4&&a.every((v,i)=>Math.abs((+v)-(+b[i]))<0.0006);
}
function renderMixer(m){
 drawMix(document.getElementById('mix_canvas'),m,false);
 const mb=m.mix,raw=mb.m;const names=['M1 FR (CW)','M2 RR (CCW)','M3 FL (CCW)','M4 RL (CW)'];
 const trim=mb.trim||[1,1,1,1];const pitch=[mb.pf,mb.pr,mb.pf,mb.pr];const roll=[-mb.r,-mb.r,mb.r,mb.r];const yaw=[-mb.y,mb.y,mb.y,-mb.y];
 let h='';for(let i=0;i<4;i++)h+='<tr><td>'+names[i]+'</td><td class="mono">'+f(mb.base,0)+'</td><td class="mono">'+f(roll[i],1)
  +'</td><td class="mono">'+f(pitch[i],1)+'</td><td class="mono">'+f(yaw[i],1)+'</td><td class="mono">'+f(trim[i],3)+'</td><td class="mono">'+f(mb.unc[i],0)+'</td><td class="mono">'+raw[i]+'</td></tr>';
 document.getElementById('mix_tbody').innerHTML=h;
 if(motorTrimPending&&motorTrimMatches(trim,motorTrimPending)){
  motorTrimDraft=trim.map(Number);motorTrimDirty=[false,false,false,false];motorTrimPending=null;
  setMotorTrimStatus('FCU live values confirmed.');
 }
 for(let i=0;i<4;i++){const now=document.getElementById('mix_trim_now_'+i),inp=document.getElementById('mix_trim_'+i);
  if(now)now.textContent=f(trim[i],3);
  if(!motorTrimDirty[i])motorTrimDraft[i]=+trim[i];
  if(inp&&!motorTrimDirty[i]&&document.activeElement!==inp)inp.value=(+trim[i]).toFixed(3);}
 const ex=mixExplain(m);const eb=document.getElementById('mix_expl');eb.className='warnbox '+(ex[0]||'');eb.textContent=ex[1];
 // overview note
 document.getElementById('ov_mixnote').textContent=ex[1];
}
function readMotorTrims(){
 const v=[];for(let i=0;i<4;i++){const inp=document.getElementById('mix_trim_'+i),n=parseFloat(inp.value);
  v.push(motorTrimDirty[i]?motorTrimDraft[i]:n);}
 if(v.some(x=>!(x>=0.9&&x<=1.1))){toast('Each motor trim must be 0.900–1.100',true);return null;}return v;
}
async function refreshMotorTrims(expected){
 try{const r=await fetch('/api/mix');if(!r.ok)return false;const j=await r.json(),tr=j.trims;
  if(!Array.isArray(tr)||tr.length!==4)return false;
  if(expected&&!motorTrimMatches(tr,expected))return false;
  motorTrimDraft=tr.map(Number);motorTrimDirty=[false,false,false,false];motorTrimPending=null;
  for(let i=0;i<4;i++){const inp=document.getElementById('mix_trim_'+i);if(inp)inp.value=(+tr[i]).toFixed(3);}
  return true;
 }catch(e){return false;}
}
async function applyMotorTrims(silent){
 const trims=readMotorTrims();if(!trims)return false;
 motorTrimPending=trims.slice();motorTrimDirty=[true,true,true,true];
 setMotorTrimStatus('Applying; waiting for FCU confirmation...');
 const ok=await put('/api/mix',silent?'':'Motor trims applied',{trims});
 if(!ok){motorTrimPending=null;setMotorTrimStatus('Apply failed; edited values were not changed.',true);return false;}
 if(await refreshMotorTrims(trims))setMotorTrimStatus('Applied and verified in FCU RAM.');
 else setMotorTrimStatus('Applied; waiting for live telemetry confirmation.',true);
 return true;
}
async function saveMotorTrims(){
 const expected=readMotorTrims();if(!expected)return;
 if(!await applyMotorTrims(true))return;
 if(!await act('/api/mix/save','Mixer settings saved to NVS')){setMotorTrimStatus('NVS save failed.',true);return;}
 if(await refreshMotorTrims(expected))setMotorTrimStatus('Saved to NVS and verified against FCU live values.');
 else setMotorTrimStatus('NVS reported saved, but live readback did not match.',true);
}
async function resetMotorTrims(){
 motorTrimDraft=[1,1,1,1];motorTrimDirty=[true,true,true,true];
 for(let i=0;i<4;i++)document.getElementById('mix_trim_'+i).value='1.000';
 await applyMotorTrims();
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
 const mg=s.mag||{},ex=mg.ext||{};
 const mxyz=Array.isArray(ex.xyz)?ex.xyz:[0,0,0],mh=Math.hypot(mxyz[0],mxyz[1]);
 const exState=!ex.comp?'not compiled':(!ex.conn?'missing':(ex.v?(ex.cal?'healthy':'uncalibrated'):magRejectName(ex.rej)));
 const exCls=!ex.comp?'grey':(!ex.conn?'err':(ex.v?(ex.cal?'ok':'warn'):'err'));
 h+=sensRow('External magnetometer (MMC5603)','I2C 0x30',exState,exCls,
  'XYZ ['+f(mxyz[0],1)+', '+f(mxyz[1],1)+', '+f(mxyz[2],1)+'] µT · H '+f(mh,1)+' · '+f(ex.hdg,0)+'° '+compassPoint(ex.hdg),'live');
 h+=sensRow('Onboard magnetometer (AK09916)','SPI(aux)','ignored by policy','grey','not used for heading/yaw','—');
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
// ---- Channel map (INAV receiver-tab style) ---------------------------------
// Fixed roles for CH1-4 (CRSF bridge) + gimbal channels, overlaid with the
// live Modes-page slot assignments (function name + range, fetched once from
// /api/config and refreshed when the Modes tab saves).
let RC_SLOTS=null;
async function rcLoadSlots(){
 try{const r=await fetch('/api/config');if(!r.ok)return;const j=await r.json();
  const map={};(j.params||[]).forEach(p=>map[p.n]=p.v);
  RC_SLOTS=[];
  for(let n=1;n<=8;n++){const f=+map['mode'+n+'_func']||0,c=+map['mode'+n+'_channel']||0;
   if(f&&c)RC_SLOTS.push({f:f,c:c,l:+map['mode'+n+'_min_us']||1300,h:+map['mode'+n+'_max_us']||1700});}
 }catch(e){}
}
function rcChannelRows(rc){
 const base={1:'Roll',2:'Pitch',3:'Throttle',4:'Yaw',6:'Pan',7:'Tilt'};
 let h='';
 for(let i=0;i<16;i++){
  const ch=i+1,us=rc.ch[i]||0;
  const pct=us?Math.max(0,Math.min(100,(us-900)/12)):0;   // 900..2100 span
  const slots=(RC_SLOTS||[]).filter(s=>s.c===ch);
  let label=base[ch]||'';
  let seg='';
  slots.forEach(s=>{
   const active=us>0&&us>=s.l&&us<=s.h;
   const l=Math.max(0,Math.min(100,(s.l-900)/12)),w=Math.max(1,Math.min(100,(s.h-900)/12)-l);
   seg+='<div style="position:absolute;top:0;bottom:0;left:'+l+'%;width:'+w+'%;border-radius:4px;'
    +'background:'+(active?'#33d17a55':'#5d667833')+';border:1px solid '+(active?'#33d17a':'#5d667855')+'"></div>';
   label+=(label?' · ':'')+MD_FUNCS[s.f]+' '+s.l+'–'+s.h+(active?' ●':'');
  });
  h+='<div class="row" style="margin:3px 0">'
   +'<label style="flex:0 0 44px">CH'+ch+'</label>'
   +'<div style="flex:1;background:#0d1118;border:1px solid #28303f;border-radius:5px;height:15px;position:relative">'
   +seg
   +'<div style="position:absolute;left:50%;top:0;bottom:0;width:1px;background:#5d667888"></div>'
   +(us?'<div style="position:absolute;left:0;top:0;bottom:0;width:'+pct+'%;background:#3fa9ff44;border-radius:4px"></div>'
       +'<div style="position:absolute;top:-1px;bottom:-1px;left:calc('+pct+'% - 1px);width:3px;background:#3fa9ff;border-radius:2px"></div>':'')
   +'</div>'
   +'<div class="v" style="flex:0 0 46px">'+(us||'--')+'</div>'
   +'<span class="dim small" style="flex:0 0 210px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap">'+(label||'—')+'</span>'
   +'</div>';
 }
 return h;
}
function renderRadio(m){
 const rc=m.rc;
 document.getElementById('rc_info').innerHTML=rc.comp?(
  kv('Protocol','CRSF / ELRS')+kv('Link',rc.up?'<span class="tag ok">UP</span>':'<span class="tag err">DOWN</span>')
  +kv('Failsafe',rc.fs?'<span class="tag err">yes</span>':'<span class="tag ok">no</span>')
  +kv('Link quality',rc.lq+' %')+kv('RSSI',rc.rssi+' dBm')+kv('Frame rate',rc.fr+' Hz')
  +kv('Last packet',ageTxt(rc.age))+kv('Loss',rc.loss+' %')+kv('Accepted/s',rc.pps)
 ):'<div class="dim">No CRSF control compiled in this build.</div>';
 document.getElementById('rc_chans').innerHTML=rcChannelRows(rc);
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
function magRejectName(r){return ['NO_SOURCE','NOT_PRESENT','DISABLED','FIELD_LOW','FIELD_HIGH','STALE','HEADING_JUMP','READ_FAIL'][r]||'INVALID';}
function wrap360d(v){return ((+v%360)+360)%360;}
function compassPoint(deg){
 const p=['N','NNE','NE','ENE','E','ESE','SE','SSE','S','SSW','SW','WSW','W','WNW','NW','NNW'];
 return Number.isFinite(+deg)?p[Math.round(wrap360d(deg)/22.5)%16]:'--';
}
function renderMag(m){
 const g=m.sen&&m.sen.mag;if(!g)return;
 const e=g.ext||{},ok=g.v===1&&g.src===2,hdg=+(e.hdg??g.hdg)||0,fld=+(e.f??g.f)||0,c=magColour(fld);
 const trim=+g.trim||0,declination=+g.dec||0,untrimmed=wrap360d(hdg-trim-declination);
 const xyz=Array.isArray(e.xyz)?e.xyz:[0,0,0],horiz=Math.hypot(xyz[0],xyz[1]),vertical=Math.abs(xyz[2]);
 const nd=document.getElementById('mag_needle');if(nd){magAcc+=((hdg-magAcc)%360+540)%360-180;nd.style.transform='rotate('+magAcc+'deg)';nd.style.opacity=ok?'1':'0.22';}
 const ht=document.getElementById('mag_hdg');if(ht)ht.textContent=ok?hdg.toFixed(0)+'° '+compassPoint(hdg):'--°';
 const magWarn=ok&&(!e.cal||Math.abs(trim)>45);
 const ms=document.getElementById('mag_state');if(ms){ms.textContent=ok?('EXTERNAL valid · '+compassPoint(hdg)+' · cal '+(e.cal?'yes':'NO')+' · trim '+(trim>=0?'+':'')+f(trim,1)+'°'):('EXTERNAL rejected · '+magRejectName(e.rej));ms.style.color=(!ok||magWarn)?'#ffb020':'';}
 const rg=document.getElementById('mag_ring');if(rg)rg.setAttribute('stroke',ok?c:'#5d6678');
 const bar=document.getElementById('mag_bar');if(bar){bar.style.width=Math.max(0,Math.min(100,fld/120*100))+'%';bar.style.background=c;}
 const ft=document.getElementById('mag_field');if(ft)ft.textContent=f(fld,1)+' µT';
 const gain=Math.max(0,Math.min(1,+g.gain||0)),gs=document.getElementById('mag_gain'),gn=document.getElementById('mag_gainn');
 if(gs&&document.activeElement!==gs)gs.value=gain.toFixed(2);if(gn&&document.activeElement!==gn)gn.value=gain.toFixed(2);
 const fs=document.getElementById('cfg_magfusion');if(fs){const tau=gain>0?(8/gain):0;
  fs.innerHTML=kv('Source',ok?'<span class="tag ok">MMC5603 valid</span>':'<span class="tag err">external invalid</span>')
   +kv('Displayed heading',ok?(f(hdg,1)+'° '+compassPoint(hdg)):'--')
   +kv('Untrimmed magnetic',ok?(f(untrimmed,1)+'° '+compassPoint(untrimmed)):'--')
   +kv('Saved trim',Math.abs(trim)>45?'<span class="tag err">'+(trim>=0?'+':'')+f(trim,1)+'° CHECK</span>':((trim>=0?'+':'')+f(trim,1)+'°'))
   +kv('Declination',f(declination,1)+'°')
   +kv('Calibration',e.cal?'<span class="tag ok">loaded</span>':'<span class="tag warn">NOT CALIBRATED</span>')
   +kv('Heading hold',m.att.yh?('<span class="tag ok">ACTIVE · target '+f(m.att.ty,1)+'°</span>'):'<span class="tag">inactive</span>')
   +kv('Body X / Y / Z',f(xyz[0],2)+' / '+f(xyz[1],2)+' / '+f(xyz[2],2)+' µT')
   +kv('Horizontal / vertical',f(horiz,2)+' / '+f(vertical,2)+' µT')
   +kv('Mode',gain===0?'<span class="tag">SHADOW</span>':('<span class="tag warn">ACTIVE '+f(gain,2)+'</span>'))
   +kv('Approx. yaw pull',gain===0?'none':f(tau,0)+' s time constant')
   +kv('Onboard AK09916','disabled');}
}
function loadMagTrim(){magTrimLoaded=true;fetch('/api/settings').then(r=>r.json()).then(j=>{
 if(typeof j.magTrimDeg!=='number')return;const s=document.getElementById('mag_trim'),n=document.getElementById('mag_trimn');
 if(s&&document.activeElement!==s)s.value=j.magTrimDeg;if(n&&document.activeElement!==n)n.value=j.magTrimDeg;}).catch(()=>{});}
async function saveMagTrim(){const v=Math.max(-360,Math.min(360,parseFloat(document.getElementById('mag_trimn').value)||0));
 if(await act('/api/settings','Heading trim saved: '+v+'°',{magTrimDeg:v})){magTrimLoaded=false;loadMagTrim();}}
async function resetMagTrim(){
 document.getElementById('mag_trim').value=0;document.getElementById('mag_trimn').value=0;
 await saveMagTrim();
}
async function applyMagGain(){
 const v=parseFloat(document.getElementById('mag_gainn').value);
 if(!(v>=0&&v<=1)){toast('Mag gain must be 0.00 to 1.00',true);return false;}
 return put('/api/mag/config',v===0?'Mag correction in shadow mode':'Mag yaw correction applied in RAM',{gain:v});
}
async function saveMagGain(){if(await applyMagGain())await act('/api/mag/config/save','Mag yaw correction saved to NVS');}
async function setMagGain(v){document.getElementById('mag_gain').value=v;document.getElementById('mag_gainn').value=v;await applyMagGain();}
)DASH"
R"DASH(
// ---------- PID tuning ----------
const PIDF=[['rrP','Rate Roll P',0,5000,25,'Roll rate'],['rrI','Rate Roll I',0,3000,25,'Roll rate'],['rrD','Rate Roll D',0,1000,5,'Roll rate'],
 ['rpP','Rate Pitch P',0,5000,25,'Pitch rate'],['rpI','Rate Pitch I',0,3000,25,'Pitch rate'],['rpD','Rate Pitch D',0,1000,5,'Pitch rate'],
 ['ryP','Rate Yaw P',0,5000,25,'Yaw rate'],['ryI','Rate Yaw I',0,3000,25,'Yaw rate'],['ryD','Rate Yaw D',0,1000,5,'Yaw rate'],
 ['aR','Angle Roll P',0,10000,100,'Angle'],['aP','Angle Pitch P',0,10000,100,'Angle'],['aY','Angle Yaw P',0,10000,100,'Angle']];
let gains=new Array(12).fill(0),pushT=null,MAGGAIN=0,magT=null;
// Refresh every widget bound to gains[i] (main slider+number, value label, and
// the yaw-card duplicate), skipping whichever the user is actively editing.
function gainUI(i){const m=gains[i],a=document.activeElement;
 const sl=document.getElementById('pr'+i),nu=document.getElementById('pn'+i),lv=document.getElementById('pv'+i),yk=document.getElementById('yk'+i),yn=document.getElementById('yk'+i+'n');
 if(sl&&a!==sl)sl.value=m; if(nu&&a!==nu)nu.value=m; if(lv)lv.textContent=(m/1000).toFixed(3);
 if(yk&&a!==yk)yk.value=m; if(yn&&a!==yn)yn.value=m;}
function editGain(i,v){v=Math.max(PIDF[i][2],Math.min(PIDF[i][3],Math.round(v)));gains[i]=v;gainUI(i);pushPid();}
function pushPid(){clearTimeout(pushT);pushT=setTimeout(()=>put('/api/pid',null,{gains}),140);}
function buildPid(){const groups={};PIDF.forEach((ff,i)=>{(groups[ff[5]]=groups[ff[5]]||[]).push(i);});
 const root=document.getElementById('pidgroups');root.innerHTML='';
 for(const g in groups){const c=document.createElement('div');c.className='card';c.innerHTML='<h3>'+g+'</h3>';
  groups[g].forEach(i=>{const ff=PIDF[i];const r=document.createElement('div');r.className='row';
   r.innerHTML='<label>'+ff[1]+'</label><input type="range" min="'+ff[2]+'" max="'+ff[3]+'" step="'+ff[4]+'" id="pr'+i+'"><input type="number" style="width:74px" min="'+ff[2]+'" max="'+ff[3]+'" step="'+ff[4]+'" id="pn'+i+'"><div class="v" id="pv'+i+'">--</div>';
   c.appendChild(r);
   // c is not in the document yet - lookups must stay within the row subtree
   r.querySelector('#pr'+i).addEventListener('input',e=>editGain(i,+e.target.value));
   r.querySelector('#pn'+i).addEventListener('change',e=>editGain(i,+e.target.value));});
  root.appendChild(c);}}
function setPidUI(g){gains=g.slice();PIDF.forEach((ff,i)=>gainUI(i));}
async function loadPid(){try{const r=await fetch('/api/pid');const j=await r.json();if(j.gains&&j.gains.length===12)setPidUI(j.gains);
 const s=document.getElementById('pid_safe');if(s){s.textContent=j.safe?'Bench-idle: edits allowed.':'ARMED / throttle ≠ 0 — edits refused.';s.style.color=j.safe?'var(--ok)':'var(--err)';}}catch(e){}}
async function savePid(){const r=await fetch('/api/pid/save',{method:'POST',headers:authHdrs()});toast(r.ok?'PID saved to NVS':'Save failed',!r.ok);}
// ---------- Yaw & heading stability card ----------
const YAWPID=[[6,'Rate Yaw P · resist rotation'],[7,'Rate Yaw I · hold against bias'],[11,'Angle Yaw P · heading-hold strength']];
function buildYawKnobs(){const root=document.getElementById('yawknobs');if(!root)return;root.innerHTML='';
 YAWPID.forEach(function(k){const i=k[0],ff=PIDF[i],r=document.createElement('div');r.className='row';
  r.innerHTML='<label>'+k[1]+'</label><input type="range" min="'+ff[2]+'" max="'+ff[3]+'" step="'+ff[4]+'" id="yk'+i+'"><input type="number" style="width:74px" min="'+ff[2]+'" max="'+ff[3]+'" step="'+ff[4]+'" id="yk'+i+'n">';
  root.appendChild(r);
  document.getElementById('yk'+i).addEventListener('input',e=>editGain(i,+e.target.value));
  document.getElementById('yk'+i+'n').addEventListener('change',e=>editGain(i,+e.target.value));});
 const r=document.createElement('div');r.className='row';
 r.innerHTML='<label>Mag yaw correction gain · 0=off</label><input type="range" min="0" max="1" step="0.05" id="ykmag"><input type="number" style="width:74px" min="0" max="1" step="0.05" id="ykmagn">';
 root.appendChild(r);
 document.getElementById('ykmag').addEventListener('input',e=>setMagGainCard(+e.target.value));
 document.getElementById('ykmagn').addEventListener('change',e=>setMagGainCard(+e.target.value));}
function setMagGainCard(v){v=Math.max(0,Math.min(1,v||0));MAGGAIN=v;const a=document.activeElement;
 const s=document.getElementById('ykmag'),n=document.getElementById('ykmagn');
 if(s&&a!==s)s.value=v; if(n&&a!==n)n.value=v.toFixed(2);
 clearTimeout(magT);magT=setTimeout(()=>put('/api/mag/config',null,{gain:v}),200);}
async function loadMagGainCard(){try{const r=await fetch('/api/mag/config');const j=await r.json();if(typeof j.gain==='number'){MAGGAIN=j.gain;const a=document.activeElement,s=document.getElementById('ykmag'),n=document.getElementById('ykmagn');if(s&&a!==s)s.value=j.gain;if(n&&a!==n)n.value=j.gain.toFixed(2);}}catch(e){}}
function gateBox(lbl,ok,sub){return '<div class="stat"><div class="l">'+lbl+'</div><div class="n" style="color:'+(ok?'var(--ok)':'var(--err)')+'">'+(ok?'✓':'✗')+'</div><div class="l">'+sub+'</div></div>';}
function renderYawCard(m){const st=document.getElementById('yh_status');if(!st)return;
 const a=(m&&m.att)||{},hold=a.yh===1;
 st.textContent=hold?('HOLD ACTIVE · target '+f(a.ty,1)+'°'):'hold inactive';st.className='tag '+(hold?'ok':'');
 const ex=(m&&m.sen&&m.sen.mag&&m.sen.mag.ext)||{};
 const calOk=ex.cal===1||ex.cal===true,gainOk=MAGGAIN>0,ayOk=gains[11]>0;
 const g=document.getElementById('yh_gates');if(g)g.innerHTML=
  gateBox('Compass cal',calOk,calOk?'valid':'run mag cal')
 +gateBox('Corr gain &gt;0',gainOk,gainOk?f(MAGGAIN,2):'raise above 0')
 +gateBox('Angle Yaw P &gt;0',ayOk,ayOk?(gains[11]/1000).toFixed(2):'raise above 0')
 +gateBox('Heading hold',hold,hold?('target '+f(a.ty,1)+'°'):'center yaw stick');}
async function saveYaw(){await put('/api/mag/config',null,{gain:MAGGAIN});
 const a=await fetch('/api/pid/save',{method:'POST',headers:authHdrs()});
 const b=await fetch('/api/mag/config/save',{method:'POST',headers:authHdrs()});
 const ok=a.ok&&b.ok;toast(ok?'Yaw tuning saved to NVS (PID + mag gain)':('Save failed'+(a.ok?'':' pid')+(b.ok?'':' mag')),!ok);}
// periodic PID refresh (safety flag + external changes)
setInterval(()=>{if(activeTab==='pid'){loadPid();loadMagGainCard();}},1500);
// ---------- boot ----------
buildTabs();buildPid();buildYawKnobs();updAuth();loadPid();loadMagGainCard();connectWs();
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
(function(){const prev=window.onRender;window.onRender=m=>{if(prev)prev(m);if(activeTab==='vibe')renderVibe(m);if(activeTab==='pid')renderYawCard(m);};})();
// ===== Blackbox card (Capture tab) ===========================================
let bbTimer=null,bbCfgLoaded=false;
async function bbPoll(){
 try{const r=await fetch('/api/blackbox');if(!r.ok)return;const s=await r.json();
  const tag=document.getElementById('bb_tag');
  tag.textContent=s.recording?'RECORDING':(s.records?'holds '+s.records+' records':'empty');
  tag.className='tag '+(s.recording?'warn':(s.records?'ok':''));
  document.getElementById('bb_stats').innerHTML=
   kv('Buffer',s.records+' / '+s.capacity+' records'+(s.allocated?'':' (not allocated)'))
  +kv('Rate',s.rateHz+' Hz → ~'+f(s.capacity/s.rateHz,0)+' s of history')
  +kv('Trigger',(s.manual?'manual':'')+(s.manual&&s.enabled?' + ':'')+(s.enabled?'motors-active':(s.manual?'':'none')));
  if(!bbCfgLoaded){bbCfgLoaded=true;
   document.getElementById('bb_en').checked=s.enabled;
   document.getElementById('bb_rate').value=s.rateHz;}
 }catch(e){}
}
async function bbAct(a,msg){if(await act('/api/blackbox/'+a,msg))bbPoll();}
async function bbClear(){if(confirm('Clear the blackbox buffer?')&&await act('/api/blackbox/clear','Cleared'))bbPoll();}
function bbSaveCfg(){
 act('/api/config','Blackbox config saved',{bb_enable:document.getElementById('bb_en').checked?1:0,
  bb_rate_hz:Math.max(10,Math.min(250,+document.getElementById('bb_rate').value||50))});
}
(function(){const prev=STAGE.capture;STAGE.capture=()=>{if(prev)prev();bbPoll();
 if(!bbTimer)bbTimer=setInterval(()=>{if(activeTab==='capture')bbPoll();else{clearInterval(bbTimer);bbTimer=null;}},700);};})();
// ===== Altitude sources card (Sensors tab) ===================================
let alCfgLoaded=false;
async function alLoadCfg(){
 try{const r=await fetch('/api/config');if(!r.ok)return;const j=await r.json();
  const map={};(j.params||[]).forEach(p=>map[p.n]=p.v);
  document.getElementById('al_src').value=map.alt_source??0;
  document.getElementById('al_med').checked=(map.tof_median_filter??1)>0.5;
  alCfgLoaded=true;
 }catch(e){}
}
function alSaveCfg(){
 act('/api/config','Altitude source config saved',{alt_source:+document.getElementById('al_src').value,
  tof_median_filter:document.getElementById('al_med').checked?1:0});
}
function renderAltCard(m){
 const sen=m.sen,alt=sen.alt||{m:0,src:0};
 const srcNames=['none','rangefinder','baro','GPS (MSL ref)'];
 const tag=document.getElementById('al_tag');if(!tag)return;
 tag.textContent=f(alt.m,2)+' m · '+srcNames[alt.src];
 tag.className='tag '+(alt.src?'ok':'err');
 document.getElementById('al_baro').innerHTML='<div class="dim small" style="margin-bottom:4px">BAROMETER</div>'
  +kv('Status',(sen.baro.v?led('ok'):led('err'))+(sen.baro.v?'healthy':'invalid'))
  +kv('Altitude',f(sen.baro.alt,2)+' m')+kv('Pressure',f(sen.baro.pa,0)+' Pa')
  +kv('Temperature',f(sen.baro.t,1)+' °C')+kv('Age',ageTxt(sen.baro.age));
 document.getElementById('al_tof').innerHTML='<div class="dim small" style="margin-bottom:4px">RANGEFINDER (ToF)</div>'
  +kv('Status',(sen.tof.comp?(sen.tof.rng?led('ok'):led('warn')):led('grey'))+(sen.tof.comp?(sen.tof.rng?'ranging':'not ranging'):'not compiled'))
  +kv('Distance',sen.tof.rng?sen.tof.mm+' mm':'--')+kv('Age',ageTxt(sen.tof.age))
  +kv('Usable band','0.03 – 3.5 m');
}
(function(){const prev=STAGE.sensors;STAGE.sensors=()=>{if(prev)prev();if(!alCfgLoaded)alLoadCfg();};})();
(function(){const prev=window.onRender;window.onRender=m=>{if(prev)prev(m);if(activeTab==='sensors')renderAltCard(m);};})();
// ===== Compass calibration wizard (Sensors tab) ==============================
let mcTimer=null,mcAlignTouched=false;
async function mcPoll(){
 try{const r=await fetch('/api/calibration/mag');if(!r.ok)return;const s=await r.json();
  const tag=document.getElementById('mc_tag');
  tag.textContent=s.active?'CAPTURING':(s.calValid?'calibrated':'idle');
  tag.className='tag '+(s.active?'warn':(s.calValid?'ok':''));
  const cov=(s.octants||0),covN=cov.toString(2).split('1').length-1;
  const remain=Math.max(0,((s.windowMs-(s.elapsedMs||0))/1000));
  document.getElementById('mc_stats').innerHTML=
   kv('Target',s.external?'external MMC5603':'onboard')
  +kv('Samples',s.samples)
  +kv('Coverage',covN+'/8 octants '+(covN>=6?'<span class="tag ok">OK</span>':'<span class="tag warn">rotate more</span>'))
  +kv('Axis span (µT)',s.range.map(v=>f(v,0)).join(' / ')+' <span class="dim small">need ≥30 each</span>')
  +kv('Field now',f(s.fieldUt,1)+' µT')
  +(s.active?kv('Time left',f(remain,0)+' s'):'');
  document.getElementById('mc_prog').style.width=
   s.active?Math.min(100,(s.elapsedMs||0)/s.windowMs*100)+'%':'0';
  const w=document.getElementById('mc_window');
  if(document.activeElement!==w&&!w.value)w.value=Math.round(s.windowMs/1000);
  const al=document.getElementById('mc_align');
  if(!mcAlignTouched&&document.activeElement!==al)al.value=s.align;
  document.getElementById('mc_finbtn').disabled=!s.active;
  // warnings: unrealistic field / poor coverage
  let warn='';
  if(s.fieldUt>0&&(s.fieldUt<25||s.fieldUt>65))
   warn+='<div class="warnbox">Field '+f(s.fieldUt,1)+' µT is outside the 25–65 µT earth range — check for nearby current wires, magnets, or a bad scale calibration.</div>';
  if(s.active&&remain<5&&covN<6)
   warn+='<div class="warnbox err">Coverage still '+covN+'/8 — keep rotating (nose up/down, knife edge) or Finish will be refused.</div>';
  document.getElementById('mc_warn').innerHTML=warn;
  mcDrawSphere(s.pts||[]);
  if(s.active&&remain<=0){mcFinish();}  // window elapsed -> auto-finish
 }catch(e){}
}
function mcDrawSphere(pts){
 const cv=document.getElementById('mc_sphere');if(!cv)return;
 const{ctx:x,W,H}=canvasCtx(cv);
 // three orthographic projections: XY (top), XZ (front), YZ (side)
 const panes=[['XY',0,1],['XZ',0,2],['YZ',1,2]];
 const pw=W/3;
 panes.forEach((p,i)=>{
  const cx=pw*i+pw/2,cy=H/2,r=Math.min(pw,H)/2-14;
  x.strokeStyle='#28303f';x.beginPath();x.arc(cx,cy,r,0,7);x.stroke();
  x.fillStyle='#8a94a6';x.font='10px system-ui';x.textAlign='center';x.fillText(p[0],cx,12);
  x.fillStyle='#3fa9ff';
  pts.forEach(q=>{const a=q[p[1]]/100,b=q[p[2]]/100;
   x.fillRect(cx+a*r-1.5,cy-b*r-1.5,3,3);});
 });
}
async function mcStart(){if(await act('/api/calibration/mag/start','Rotate the craft through ALL orientations'))mcPoll();}
async function mcFinish(){if(await act('/api/calibration/mag/finish','Compass calibration saved'))mcPoll();else mcPoll();}
async function mcCancel(){if(await act('/api/calibration/mag/cancel','Cancelled'))mcPoll();}
function mcSaveWindow(){const v=Math.max(20,Math.min(120,+document.getElementById('mc_window').value||30));
 act('/api/config','Calibration window saved',{mag_cal_time_s:v});}
function mcSaveAlign(){mcAlignTouched=true;
 act('/api/config','Mag alignment saved — verify with the Setup compass test',{mag_align:+document.getElementById('mc_align').value});}
(function(){const prev=STAGE.sensors;STAGE.sensors=()=>{if(prev)prev();mcPoll();
 if(!mcTimer)mcTimer=setInterval(()=>{if(activeTab==='sensors')mcPoll();else{clearInterval(mcTimer);mcTimer=null;}},500);};})();
// ===== Six-position accel calibration (Attitude & Level tab) =================
const AC_FACES=['+X','-X','+Y','-Y','+Z','-Z'];
const AC_STATES=['idle','waiting for stillness…','sampling…','face captured','face FAILED'];
let acTimer=null;
async function acPoll(){
 try{const r=await fetch('/api/calibration/accel');if(!r.ok)return;const s=await r.json();
  const tag=document.getElementById('ac_tag');
  tag.textContent=s.active?(AC_STATES[s.state]||s.state):(s.cal6Valid?'calibrated':'idle');
  tag.className='tag '+(s.active?'warn':(s.cal6Valid?'ok':''));
  document.getElementById('ac_faces').innerHTML=AC_FACES.map((n,i)=>{
   const done=s.facesDone&(1<<i),act=s.activeFace===i&&s.active;
   return '<span class="tag '+(done?'ok':(act?'warn':''))+'" style="padding:6px 12px">'+n+(done?' ✓':'')+'</span>';
  }).join('');
  let st=AC_STATES[s.state]||'';
  if(s.state===2)st+=' '+s.samples+'/'+s.samplesPerFace;
  if(s.state===4&&s.error)st+=' — '+s.error;
  document.getElementById('ac_state').textContent=(s.active?st:'No session.')+
   '  ·  |accel| = '+f(s.accelMag,3)+' g (should read ≈1.000 when still)';
  document.getElementById('ac_result').textContent=
   'applied  zero '+s.zero.map(v=>f(v,3)).join('/')+'  gain '+s.gain.map(v=>f(v,3)).join('/')
   +(s.cal6Valid?'  (6-pos)':'  (one-pose or none)');
  const allDone=(s.facesDone&63)===63;
  document.getElementById('ac_finbtn').disabled=!(s.active&&allDone);
  document.getElementById('ac_capbtn').disabled=!s.active||s.state===1||s.state===2;
 }catch(e){}
}
async function acStart(){if(await act('/api/calibration/accel/start','Session started — set the craft level'))acPoll();}
async function acCapture(){if(await act('/api/calibration/accel/capture-face','Hold still…'))acPoll();}
async function acFinish(){if(await act('/api/calibration/accel/finish','Accel calibration saved'))acPoll();}
async function acCancel(){if(await act('/api/calibration/accel/cancel','Cancelled'))acPoll();}
(function(){const prev=STAGE.attitude;STAGE.attitude=()=>{if(prev)prev();acPoll();
 if(!acTimer)acTimer=setInterval(()=>{if(activeTab==='attitude')acPoll();else{clearInterval(acTimer);acTimer=null;}},400);};})();
// ===== Filter chain card (Filters & Notch tab) ===============================
let FT_LOADED=false;
async function ftLoad(showToast){
 try{const r=await fetch('/api/config');if(!r.ok)return;const j=await r.json();
  const map={};(j.params||[]).forEach(p=>map[p.n]=p.v);
  const set=(id,v)=>{const el=document.getElementById(id);if(el&&v!=null)el.value=v;};
  set('ft_g1t',map.gyro_lpf_type);set('ft_g1h',map.gyro_lpf_hz);
  set('ft_gdmin',map.gyro_lpf_dyn_min_hz);set('ft_gdmax',map.gyro_lpf_dyn_max_hz);
  set('ft_gdexp',map.gyro_lpf_dyn_expo);set('ft_g2t',map.gyro_lpf2_type);set('ft_g2h',map.gyro_lpf2_hz);
  set('ft_at',map.accel_lpf_type);set('ft_ah',map.accel_lpf_hz);
  set('ft_anh',map.accel_notch_hz);set('ft_anq',map.accel_notch_q);
  set('ft_dh',map.dterm_lpf_hz);set('ft_sh',map.setpoint_lpf_hz);
  FT_LOADED=true;const t=document.getElementById('ft_tag');t.className='tag ok';t.textContent='loaded';
  if(showToast)toast('Filter settings reloaded');
 }catch(e){}
}
async function ftApply(){
 const g=id=>+document.getElementById(id).value||0;
 const body={gyro_lpf_type:g('ft_g1t'),gyro_lpf_hz:g('ft_g1h')||90,
  gyro_lpf_dyn_min_hz:g('ft_gdmin'),gyro_lpf_dyn_max_hz:g('ft_gdmax'),gyro_lpf_dyn_expo:g('ft_gdexp'),
  gyro_lpf2_type:g('ft_g2t'),gyro_lpf2_hz:g('ft_g2h')||150,
  accel_lpf_type:g('ft_at')||2,accel_lpf_hz:g('ft_ah')||15,
  accel_notch_hz:g('ft_anh'),accel_notch_q:g('ft_anq')||3,
  dterm_lpf_hz:g('ft_dh'),setpoint_lpf_hz:g('ft_sh')};
 if(body.gyro_lpf_dyn_min_hz>0&&body.gyro_lpf_dyn_max_hz<=body.gyro_lpf_dyn_min_hz){
  toast('Dyn max must exceed dyn min',true);return;}
 await act('/api/filters/apply','Filters applied & saved',body);
}
function renderFilterCard(m){
 const el=document.getElementById('ft_live');if(!el)return;
 const g=m.imu.g||[0,0,0],gr=m.imu.graw||[0,0,0];
 el.textContent='raw vs filtered (dps)  X '+f(gr[0],1)+' → '+f(g[0],1)
  +'   Y '+f(gr[1],1)+' → '+f(g[1],1)+'   Z '+f(gr[2],1)+' → '+f(g[2],1);
 const lp=document.getElementById('ft_loop');if(lp)lp.textContent=m.sys.loop;
 const dyn=document.getElementById('ft_dynnote');
 if(dyn)dyn.style.display=(+document.getElementById('ft_gdmin').value>0&&+document.getElementById('ft_g1t').value===2)?'block':'none';
}
(function(){const prev=STAGE.notch;STAGE.notch=()=>{if(prev)prev();if(!FT_LOADED)ftLoad();};})();
(function(){const prev=window.onRender;window.onRender=m=>{if(prev)prev(m);if(activeTab==='notch')renderFilterCard(m);};})();
// ===== GPS tab ===============================================================
// Status rides the WS frame; config loads once from /api/config and saves via
// /api/gps/save; "Apply to receiver" schedules the disarmed-only UBX push.
let GP_LOADED=false;
const NAVERR=['','no GPS fix','too few satellites','HDOP too high','no heading source',
 'compass not calibrated','home not set','no ToF/baro altitude source','stick override'];
async function gpLoad(){
 try{const r=await fetch('/api/config');if(!r.ok)return;const j=await r.json();
  const map={};(j.params||[]).forEach(p=>map[p.n]=p.v);
  document.getElementById('nv_tilt').value=map.nav_max_tilt_deg??20;
  document.getElementById('nv_vel').value=map.nav_max_vel_ms??5;
  document.getElementById('nv_acc').value=map.nav_max_accel_ms2??2.5;
  document.getElementById('nv_ovr').value=map.nav_stick_override_pct??20;
  document.getElementById('nv_loss').value=map.nav_gps_loss_action??0;
  document.getElementById('gp_prov').value=map.gps_provider??1;
  document.getElementById('gp_baud').value=map.gps_baud??0;
  document.getElementById('gp_rate').value=map.gps_rate_hz??5;
  document.getElementById('gp_dyn').value=map.gps_dyn_model??3;
  document.getElementById('gp_sbas').checked=(map.gps_sbas??1)>0.5;
  document.getElementById('gp_auto').checked=(map.gps_auto_config??0)>0.5;
  document.getElementById('gp_navsat').value=map.nav_min_sats??8;
  document.getElementById('gp_navhdop').value=map.nav_max_hdop??2;
  document.getElementById('gp_navaau').checked=(map.nav_allow_arm_unsafe??0)>0.5;
  GP_LOADED=true;
 }catch(e){}
}
function gpBody(){
 return{gps_provider:+document.getElementById('gp_prov').value,
  gps_baud:+document.getElementById('gp_baud').value,
  gps_rate_hz:+document.getElementById('gp_rate').value||5,
  gps_dyn_model:+document.getElementById('gp_dyn').value,
  gps_sbas:document.getElementById('gp_sbas').checked?1:0,
  gps_auto_config:document.getElementById('gp_auto').checked?1:0};
}
async function gpSave(){await act('/api/gps/save','GPS settings saved',gpBody());}
async function gpApply(){
 if(!await act('/api/gps/save','Settings saved',gpBody()))return;
 await act('/api/gps/configure','UBX config scheduled (watch fix status)');
}
async function gpSaveGates(){
 await act('/api/gps/save','Nav gates saved',{nav_min_sats:+document.getElementById('gp_navsat').value||8,
  nav_max_hdop:+document.getElementById('gp_navhdop').value||2,
  nav_allow_arm_unsafe:document.getElementById('gp_navaau').checked?1:0});
}
function renderGps(m){
 const g=m.sen.gps,el=document.getElementById('gp_status');
 if(!g.comp){el.innerHTML='<div class="dim small">GPS not compiled in this build.</div>';return;}
 const fixNames={0:'no fix',1:'GPS',2:'DGPS'};
 el.innerHTML=
  kv('Fix',(g.fix?led('ok'):led('err'))+(fixNames[g.q]||('q'+g.q)))
 +kv('Satellites',g.sats)
 +kv('HDOP',g.hdv?f(g.hdop,1):'--')
 +kv('Position',g.fix?(g.lat/1e7).toFixed(6)+', '+(g.lon/1e7).toFixed(6):'--')
 +kv('Ground speed',g.gsv?f(g.spd,1)+' m/s':'--')
 +kv('Course',g.cv?f(g.cog/100,0)+'°':'--')
 +kv('Sentence age',ageTxt(g.age));
 const h=g.home||{},ht=document.getElementById('gp_hometag');
 ht.className='tag '+(h.set?'ok':'warn');ht.textContent=h.set?'SET':'not set';
 document.getElementById('gp_home').innerHTML=h.set?
  kv('Home',(h.lat/1e7).toFixed(6)+', '+(h.lon/1e7).toFixed(6))
  +kv('Distance',g.fix?f(h.dist,1)+' m':'--')
  +kv('Bearing to home',g.fix?f(h.brg,0)+'°':'--')
  :'<div class="dim small">Waiting for the origin debouncer (stable fix, ≥6 sats, 10 s) or a manual capture.</div>';
 const ct=document.getElementById('gp_cfgtag');
 if(GP_LOADED){ct.className='tag ok';ct.textContent='loaded';}
 // nav scaffold card
 const nv=m.nav||{ph:1,rth:1,err:0,eng:0};
 const nt=document.getElementById('nv_tag');
 nt.textContent=nv.eng?'SHADOW ENGAGED':(nv.ph===0?'POSHOLD available':'blocked');
 nt.className='tag '+(nv.eng?'warn':(nv.ph===0?'ok':''));
 document.getElementById('nv_status').innerHTML=
  kv('POS HOLD',nv.ph===0?'<span class="tag ok">available</span>':'<span class="tag warn">'+(NAVERR[nv.ph]||nv.ph)+'</span>')
 +kv('RTH',nv.rth===0?'<span class="tag ok">available</span>':'<span class="tag warn">'+(NAVERR[nv.rth]||nv.rth)+'</span>')
 +kv('Live error',nv.err?NAVERR[nv.err]||nv.err:'none');
 document.getElementById('nv_shadow').innerHTML=
  kv('Shadow state',nv.eng?'holding position':'idle')
 +kv('Cmd roll / pitch',f(nv.cr,1)+'° / '+f(nv.cp,1)+'°')
 +kv('Distance to hold point',f(nv.dist,1)+' m');
}
function nvSave(){
 act('/api/gps/save','Nav limits saved',{nav_max_tilt_deg:+document.getElementById('nv_tilt').value||20,
  nav_max_vel_ms:+document.getElementById('nv_vel').value||5,
  nav_max_accel_ms2:+document.getElementById('nv_acc').value||2.5,
  nav_stick_override_pct:+document.getElementById('nv_ovr').value||20,
  nav_gps_loss_action:+document.getElementById('nv_loss').value});
}
STAGE.gps=()=>{if(!GP_LOADED)gpLoad();};
// ===== Modes tab (aux-range assignment) ======================================
// Slot config loads/saves through /api/modes (flat param object); live channel
// values + the active/assigned masks ride the WS frame at 25 Hz.
const MD_FUNCS=['NONE','ARM','ANGLE','ACRO','ALT HOLD','POS HOLD','RTH','KILL SWITCH','BEEPER','BLACKBOX'];
let MD_LOADED=false,MD_SLOTS=[];
function mdBuild(){
 const tb=document.getElementById('md_slots');if(!tb||tb.childElementCount)return;
 let h='';
 for(let n=0;n<8;n++){
  h+='<tr><td>'+(n+1)+'</td>'
   +'<td><select id="md_f'+n+'" onchange="mdEdited()">'+MD_FUNCS.map((f,i)=>'<option value="'+i+'">'+f+'</option>').join('')+'</select></td>'
   +'<td><select id="md_c'+n+'" onchange="mdEdited()"><option value="0">--</option>'+Array.from({length:16},(_,i)=>'<option value="'+(i+1)+'">CH'+(i+1)+'</option>').join('')+'</select></td>'
   +'<td><input type="number" id="md_l'+n+'" min="900" max="2100" step="25" style="max-width:80px" oninput="mdEdited()"></td>'
   +'<td><input type="number" id="md_h'+n+'" min="900" max="2100" step="25" style="max-width:80px" oninput="mdEdited()"></td>'
   +'<td class="mono" id="md_us'+n+'">--</td>'
   +'<td id="md_st'+n+'"><span class="tag">--</span></td></tr>';
 }
 tb.innerHTML=h;
}
async function mdLoad(showToast){
 mdBuild();
 try{const r=await fetch('/api/config');if(!r.ok)return;const j=await r.json();
  const map={};(j.params||[]).forEach(p=>map[p.n]=p.v);
  for(let n=0;n<8;n++){
   document.getElementById('md_f'+n).value=map['mode'+(n+1)+'_func']??0;
   document.getElementById('md_c'+n).value=map['mode'+(n+1)+'_channel']??0;
   document.getElementById('md_l'+n).value=map['mode'+(n+1)+'_min_us']??1300;
   document.getElementById('md_h'+n).value=map['mode'+(n+1)+'_max_us']??1700;
  }
  MD_LOADED=true;mdConflicts();
  if(showToast)toast('Modes reloaded');
 }catch(e){}
}
function mdEdited(){mdConflicts();document.getElementById('md_tag').textContent='unsaved edits';document.getElementById('md_tag').className='tag warn';}
function mdCollect(){
 const s=[];
 for(let n=0;n<8;n++)s.push({f:+document.getElementById('md_f'+n).value,c:+document.getElementById('md_c'+n).value,
  l:+document.getElementById('md_l'+n).value||1300,h:+document.getElementById('md_h'+n).value||1700});
 return s;
}
function mdConflicts(){
 const el=document.getElementById('md_conflicts');if(!el)return;
 const s=mdCollect(),w=[];
 const armSlots=s.filter(x=>x.f===1&&x.c>0);
 if(!armSlots.length)w.push(['err','No ARM slot assigned — arming will be IMPOSSIBLE.']);
 const seen={};
 s.forEach((x,i)=>{if(x.f&&x.c){if(seen[x.f]!=null)w.push(['warn',MD_FUNCS[x.f]+' assigned in slots '+(seen[x.f]+1)+' and '+(i+1)+' (either activates it)']);else seen[x.f]=i;}});
 s.forEach((x,i)=>{if(x.f&&x.c&&x.l>=x.h)w.push(['err','Slot '+(i+1)+': min ≥ max']);});
 const arm=s.find(x=>x.f===1&&x.c),kill=s.find(x=>x.f===7&&x.c);
 if(arm&&kill&&arm.c===kill.c&&arm.l<kill.h&&kill.l<arm.h)w.push(['err','ARM and KILL ranges OVERLAP on CH'+arm.c]);
 s.forEach((x,i)=>{if(x.f&&x.c>=1&&x.c<=4)w.push(['warn','Slot '+(i+1)+' uses CH'+x.c+' (a stick channel)']);});
 el.innerHTML=w.length?w.map(x=>'<div class="warnbox '+(x[0]==='err'?'err':'')+'">'+x[1]+'</div>').join('')
  :'<div class="warnbox ok">No conflicts.</div>';
 return w;
}
async function mdSave(){
 const s=mdCollect(),w=mdConflicts()||[];
 if(w.some(x=>x[0]==='err')&&!confirm('There are ERROR-level conflicts (see list). Save anyway?'))return;
 const body={};
 s.forEach((x,n)=>{body['mode'+(n+1)+'_func']=x.f;body['mode'+(n+1)+'_channel']=x.c;
  body['mode'+(n+1)+'_min_us']=x.l;body['mode'+(n+1)+'_max_us']=x.h;});
 if(await act('/api/modes','Modes saved & applied',body)){
  document.getElementById('md_tag').textContent='saved';document.getElementById('md_tag').className='tag ok';
  rcLoadSlots();  // keep the Radio-tab channel map labels in sync
 }
}
function renderModes(m){
 if(!MD_LOADED)return;
 const rc=m.rc,ch=rc.ch||[],md=rc.modes||{act:0,asg:0};
 // channel bars
 const el=document.getElementById('md_channels');
 let h='';
 for(let i=0;i<16;i++){
  const us=ch[i]||0,pct=Math.max(0,Math.min(100,(us-900)/1200*100));
  h+='<div style="display:flex;align-items:center;gap:8px;margin:2px 0"><span class="dim small" style="flex:0 0 38px">CH'+(i+1)+'</span>'
   +'<div style="flex:1;height:12px;background:#0b0e13;border-radius:6px;overflow:hidden"><div style="height:100%;width:'+pct+'%;background:'+(i<4?'#3fa9ff':'#7c5cff')+'"></div></div>'
   +'<span class="mono small" style="flex:0 0 46px;text-align:right">'+(us||'--')+'</span></div>';
 }
 el.innerHTML=h;
 // per-slot live state
 const s=mdCollect();
 s.forEach((x,n)=>{
  const us=x.c>=1?ch[x.c-1]||0:0;
  document.getElementById('md_us'+n).textContent=x.c?us:'--';
  const active=x.f&&x.c&&us>=x.l&&us<=x.h&&us>0;
  document.getElementById('md_st'+n).innerHTML=x.f&&x.c?('<span class="tag '+(active?'ok':'')+'">'+(active?'ACTIVE':'inactive')+'</span>'):'<span class="tag">unused</span>';
 });
 // active modes card (firmware-evaluated mask, not the local check)
 let ah='';
 for(let f=1;f<MD_FUNCS.length;f++){
  const assigned=md.asg&(1<<f),active=md.act&(1<<f);
  if(!assigned)continue;
  ah+=kv(MD_FUNCS[f],active?'<span class="tag ok">ACTIVE</span>':'<span class="tag">inactive</span>');
 }
 if(md.kill)ah='<div class="warnbox err">KILL SWITCH ACTIVE — motors stopped, arming blocked.</div>'+ah;
 document.getElementById('md_active').innerHTML=ah||'<div class="dim small">No modes assigned.</div>';
 // arming card (reuse setup-tab reasons list)
 const st=m.sys,flags=+st.armf||0;
 let arm='';
 if(st.armed)arm='<div class="warnbox err">ARMED</div>';
 else if(flags===0)arm='<div class="warnbox ok">Ready to arm.</div>';
 else{arm='<div class="dim small" style="margin-bottom:6px">Arming disabled because:</div>';
  for(let b=0;b<AF_NAMES.length;b++)if(flags&(1<<b))arm+='<div class="warnbox">'+AF_NAMES[b]+'</div>';}
 document.getElementById('md_arm').innerHTML=arm;
}
STAGE.modes=()=>{if(!MD_LOADED)mdLoad();};
(function(){const prev=STAGE.radio;STAGE.radio=()=>{if(prev)prev();if(!RC_SLOTS)rcLoadSlots();};})();
// ===== Mixer table editor ====================================================
// Reads/writes the 16 mix_m*_{throttle,roll,pitch,yaw} params through
// /api/config (validated + persisted firmware-side; staged atomically into
// the flight mixer). Live outputs are already on this tab via the WS frame.
const MIXT_DEF=[1,-1,-1,-1, 1,-1,1,1, 1,1,-1,1, 1,1,1,-1];
const MIXT_NAMES=['M1 front-right','M2 rear-right','M3 front-left','M4 rear-left'];
let MIXT_LOADED=false;
function mixtBuild(){
 const tb=document.getElementById('mixt_body');if(!tb||tb.childElementCount)return;
 tb.innerHTML=MIXT_NAMES.map((n,m)=>'<tr><td>'+n+'</td>'+[0,1,2,3].map(a=>
  '<td><input type="number" id="mixt_'+(m*4+a)+'" step="0.05" min="'+(a===0?0:-2)+'" max="2" style="max-width:90px"></td>').join('')+'</tr>').join('');
}
async function mixtLoad(showToast){
 mixtBuild();
 try{const r=await fetch('/api/config');if(!r.ok)return;const j=await r.json();
  const map={};(j.params||[]).forEach(p=>map[p.n]=p.v);
  const ax=['throttle','roll','pitch','yaw'];
  for(let m=0;m<4;m++)for(let a=0;a<4;a++){
   const v=map['mix_m'+(m+1)+'_'+ax[a]];
   const el=document.getElementById('mixt_'+(m*4+a));
   if(el&&v!=null)el.value=v;}
  MIXT_LOADED=true;
  if(showToast)toast('Mixer table reloaded');
 }catch(e){}
}
function mixtCollect(){
 const t=[];
 for(let i=0;i<16;i++){const v=parseFloat(document.getElementById('mixt_'+i).value);
  if(isNaN(v))return null;t.push(v);}
 return t;
}
function mixtWarnings(t){
 const w=[];
 for(let i=0;i<16;i+=4)if(t[i]<0||t[i]>2)return{err:'Throttle coefficients must be 0..2'};
 for(let i=0;i<16;i++)if(t[i]<-2||t[i]>2)return{err:'Coefficients must be within ±2'};
 const ax=['roll','pitch','yaw'];
 for(let a=1;a<4;a++){
  const col=[t[a],t[4+a],t[8+a],t[12+a]];
  if(!col.some(v=>v>0.05)||!col.some(v=>v<-0.05))w.push('No differential '+ax[a-1]+' authority (need + and − entries)');
  const sum=col.reduce((s,v)=>s+v,0);
  if(Math.abs(sum)>0.5)w.push(ax[a-1]+' column sum '+sum.toFixed(2)+' ≠ 0 (couples into collective)');
 }
 return{warn:w};
}
async function mixtSave(){
 const t=mixtCollect();
 if(!t){toast('Fill every coefficient first',true);return;}
 const v=mixtWarnings(t);
 if(v.err){toast(v.err,true);return;}
 if(v.warn.length&&!confirm('Mixer sanity warnings:\n\n- '+v.warn.join('\n- ')+'\n\nSave anyway?'))return;
 const ax=['throttle','roll','pitch','yaw'];const body={};
 for(let m=0;m<4;m++)for(let a=0;a<4;a++)body['mix_m'+(m+1)+'_'+ax[a]]=t[m*4+a];
 if(await act('/api/config','Mixer table saved & applied',body))
  document.getElementById('mixt_note').textContent=v.warn.length?('Saved with warnings: '+v.warn.join(' · ')):'Saved. Default Quad-X geometry = throttle 1, roll/pitch/yaw ±1.';
}
function mixtReset(){
 if(!confirm('Reset the mixer table to the default Quad-X geometry?'))return;
 for(let i=0;i<16;i++)document.getElementById('mixt_'+i).value=MIXT_DEF[i];
 mixtSave();
}
STAGE.mixer=()=>{if(!MIXT_LOADED)mixtLoad();};
// ===== Motors tab (deadman test + order wizard) =============================
// SAFETY MODEL: a motor spins only while a hold-button is down. The browser
// re-POSTs /api/motors/test every 150 ms with a 400 ms deadman; the firmware
// stops the motor when refreshes lapse for ANY reason (release, tab close,
// WiFi drop). The STOP button and props-removed checkbox are always visible.
let MO={cfg:null,holdTimer:null,holdOut:0,pollTimer:null};
async function moPoll(){try{const r=await fetch('/api/motors');if(!r.ok)return;MO.cfg=await r.json();moRender();}catch(e){}}
function moAck(){return document.getElementById('mo_ack').checked;}
function moVal(){const v=+document.getElementById('mo_val').value||0;return Math.max(48,Math.min(800,v));}
async function moSendTest(out){
 try{const r=await fetch('/api/motors/test',{method:'POST',headers:authHdrs({'content-type':'application/json'}),
  body:JSON.stringify({motor:out,value:moVal(),ms:400,ack:'props-removed'})});
  if(!r.ok){moHoldEnd(false);let m='refused';try{const j=await r.json();m=j.error||m;}catch(e){}
   toast('Motor test '+m,true);}
 }catch(e){moHoldEnd(false);}
}
function moHoldStart(out){
 if(!moAck()){toast('Check "Propellers are removed" first',true);return;}
 if(MO.holdOut)return;
 MO.holdOut=out;moSendTest(out);
 MO.holdTimer=setInterval(()=>moSendTest(out),150);
 moRender();
}
function moHoldEnd(sendStop){
 if(MO.holdTimer){clearInterval(MO.holdTimer);MO.holdTimer=null;}
 if(MO.holdOut){MO.holdOut=0;if(sendStop!==false)moStop(true);}
 moRender();
}
async function moStop(quiet){
 try{await fetch('/api/motors/stop',{method:'POST',headers:authHdrs()});}catch(e){}
 if(!quiet)toast('Motors stopped');
 moPoll();
}
function moBindHold(){
 const ie=document.getElementById('mo_idleen');
 if(ie&&!ie._bound){ie._bound=1;ie.addEventListener('change',()=>ie._touched=1);}
 for(let n=1;n<=4;n++){const b=document.getElementById('mo_b'+n);if(!b||b._bound)continue;b._bound=1;
  b.addEventListener('pointerdown',e=>{e.preventDefault();b.setPointerCapture(e.pointerId);moHoldStart(n);});
  ['pointerup','pointercancel','pointerleave','lostpointercapture'].forEach(ev=>
   b.addEventListener(ev,()=>{if(MO.holdOut===n)moHoldEnd();}));
  b.addEventListener('contextmenu',e=>e.preventDefault());}
}
// stop on tab hide / navigation as extra belt-and-braces (deadman covers it anyway)
document.addEventListener('visibilitychange',()=>{if(document.hidden&&MO.holdOut)moHoldEnd();});
function moDrawQuad(){
 const cv=document.getElementById('mo_quad');if(!cv)return;
 const d=Math.min(window.devicePixelRatio||1,2),S=240;
 if(cv.width!==S*d){cv.width=S*d;cv.height=S*d;}
 const x=cv.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,S,S);
 const c=S/2,arm=78,r=30;
 const cfg=MO.cfg||{map:[1,2,3,4],dir:[0,1,1,0],active:0};
 // logical slots: 0=FR 1=RR 2=FL 3=RL — canvas: nose up
 const pos=[[c+arm,c-arm],[c+arm,c+arm],[c-arm,c-arm],[c-arm,c+arm]];
 x.strokeStyle='#28303f';x.lineWidth=8;
 x.beginPath();x.moveTo(pos[2][0],pos[2][1]);x.lineTo(pos[1][0],pos[1][1]);
 x.moveTo(pos[0][0],pos[0][1]);x.lineTo(pos[3][0],pos[3][1]);x.stroke();
 // nose arrow
 x.fillStyle='#3fa9ff';x.beginPath();x.moveTo(c,c-30);x.lineTo(c-10,c-8);x.lineTo(c+10,c-8);x.closePath();x.fill();
 x.font='11px system-ui';x.textAlign='center';
 const names=['FR','RR','FL','RL'];
 for(let i=0;i<4;i++){
  const out=cfg.map[i],act=cfg.active===out&&out>0;
  x.beginPath();x.arc(pos[i][0],pos[i][1],r,0,7);
  x.fillStyle=act?'#13314d':'#161c28';x.fill();
  x.strokeStyle=act?'#3fa9ff':'#28303f';x.lineWidth=2;x.stroke();
  // direction arc (from above): CW or CCW
  x.strokeStyle=cfg.dir[i]?'#33d17a':'#ffb020';x.lineWidth=2;
  x.beginPath();
  if(cfg.dir[i])x.arc(pos[i][0],pos[i][1],r-6,0.6,2.5);else x.arc(pos[i][0],pos[i][1],r-6,0.6,2.5,false);
  x.stroke();
  const ang=cfg.dir[i]?2.5:0.6,ax=pos[i][0]+(r-6)*Math.cos(ang),ay=pos[i][1]+(r-6)*Math.sin(ang);
  x.fillStyle=cfg.dir[i]?'#33d17a':'#ffb020';
  x.beginPath();x.arc(ax,ay,3,0,7);x.fill();
  x.fillStyle='#e7ebf2';x.fillText('OUT '+out,pos[i][0],pos[i][1]-2);
  x.fillStyle='#8a94a6';x.fillText(names[i]+' · '+(cfg.dir[i]?'CCW':'CW'),pos[i][0],pos[i][1]+12);
 }
}
function moRender(){
 const cfg=MO.cfg;if(!cfg)return;
 const st=document.getElementById('mo_state');
 if(cfg.active){st.className='tag warn';st.textContent='SPINNING OUT '+cfg.active+' @ '+cfg.raw;}
 else if(!cfg.safe){st.className='tag err';st.textContent='NOT BENCH-IDLE';}
 else{st.className='tag ok';st.textContent='ready';}
 const val=document.getElementById('mo_val');
 if(document.activeElement!==val&&!val.value)val.value=cfg.testValue;
 const ie=document.getElementById('mo_idleen'),iv=document.getElementById('mo_idleval');
 if(document.activeElement!==ie&&!ie._touched)ie.checked=!!cfg.idleEnable;
 if(document.activeElement!==iv&&!iv.value)iv.value=cfg.idleValue;
 document.getElementById('mo_note').textContent=
  'deadman '+(cfg.deadmanMs||0)+' ms · hold ceiling '+cfg.holdMaxMs+' ms · session cap 120 s';
 const names=['M1 · front-right','M2 · rear-right','M3 · front-left','M4 · rear-left'];
 document.getElementById('mo_maptable').innerHTML=cfg.map.map((o,i)=>
  '<tr><td>'+names[i]+'</td><td class="mono">OUT '+o+'</td><td>'+(cfg.dir[i]?'CCW':'CW')+'</td></tr>').join('');
 moDrawQuad();
}
function moSaveTestVal(){act('/api/config','Test value saved',{motor_test_value:moVal()});}
function moSaveIdle(){
 const en=document.getElementById('mo_idleen').checked?1:0;
 const v=Math.max(48,Math.min(300,+document.getElementById('mo_idleval').value||48));
 if(en&&!confirm('Enable ARMED IDLE?\n\nArming will spin ALL motors at '+v+' immediately, on the ground. Only proceed if the props-off checklist passed.'))return;
 act('/api/config','Armed-idle config saved',{motor_idle_enable:en,motor_idle_value:v});
}
// ---- order wizard ----
let MW={run:false,out:1,ans:{}};
function mwStart(){MW={run:true,out:1,ans:{}};document.getElementById('mw_tag').textContent='output 1 of 4';mwDraw();}
function mwPick(kind,v){
 const a=MW.ans[MW.out]=MW.ans[MW.out]||{};a[kind]=v;mwDraw();
}
function mwNext(){
 const a=MW.ans[MW.out];
 if(!a||a.pos==null||a.dir==null){toast('Pick the corner and direction first',true);return;}
 if(MW.out<4){MW.out++;document.getElementById('mw_tag').textContent='output '+MW.out+' of 4';mwDraw();}
 else mwFinish();
}
async function mwFinish(){
 MW.run=false;
 const posOf={};let dup=false;
 for(let o=1;o<=4;o++){const p=MW.ans[o].pos;if(posOf[p]!=null)dup=true;posOf[p]=o;}
 if(dup){toast('Two outputs were assigned the same corner — restart',true);
  document.getElementById('mw_tag').textContent='conflict';mwDraw();return;}
 // logical order: 0=FR 1=RR 2=FL 3=RL; map[i]=output at that corner
 const map=[posOf.FR,posOf.RR,posOf.FL,posOf.RL];
 const dirOf={};for(let o=1;o<=4;o++)dirOf[o]=MW.ans[o].dir;
 const dir=map.map(o=>dirOf[o]==='CCW'?1:0);
 const ok1=await act('/api/motors/save-order','Motor order saved',{map:map});
 const ok2=ok1&&await act('/api/motors/save-direction','Directions saved',{dir:dir});
 document.getElementById('mw_tag').textContent=(ok1&&ok2)?'saved':'save failed';
 document.getElementById('mw_tag').className='tag '+((ok1&&ok2)?'ok':'err');
 mwDraw();moPoll();
}
function mwDraw(){
 const el=document.getElementById('mw_body');if(!el)return;
 if(!MW.run){if(!Object.keys(MW.ans).length)el.innerHTML='<div class="dim small">Not started.</div>';return;}
 const a=MW.ans[MW.out]||{};
 const corner=(p,l)=>'<button class="btn sm'+(a.pos===p?' pri':'')+'" onclick="mwPick(\'pos\',\''+p+'\')">'+l+'</button>';
 const dirb=(dv,l)=>'<button class="btn sm'+(a.dir===dv?' pri':'')+'" onclick="mwPick(\'dir\',\''+dv+'\')">'+l+'</button>';
 el.innerHTML=
  '<div class="warnbox">Hold <b>OUT '+MW.out+'</b> above (props off). Watch which corner motor turns.</div>'
 +'<div class="row"><label>Corner that spun</label><div class="btns">'
 +corner('FR','Front-right')+corner('RR','Rear-right')+corner('FL','Front-left')+corner('RL','Rear-left')+'</div></div>'
 +'<div class="row"><label>Direction (from above)</label><div class="btns">'
 +dirb('CW','Clockwise')+dirb('CCW','Counter-clockwise')+'</div></div>'
 +'<div class="btns"><button class="btn pri" onclick="mwNext()">'+(MW.out<4?'Next output':'Finish & save')+'</button></div>';
}
STAGE.motors=()=>{moBindHold();moPoll();
 if(!MO.pollTimer)MO.pollTimer=setInterval(()=>{if(activeTab==='motors')moPoll();else{clearInterval(MO.pollTimer);MO.pollTimer=null;}},600);};
// ===== Setup / Instruments tab ==============================================
// Artificial horizon + heading tape drawn from the corrected estimator
// attitude (m.att.cr/cp) and the active heading source. All 2D canvas — no
// libraries, works offline on the FCU AP.
const AF_NAMES=['THROTTLE_HIGH','KILL_SWITCH','RX_INVALID','GYRO_NOT_CAL','GYRO_UNSTABLE',
 'ACCEL_NOT_CAL','MAG_NOT_CAL','GPS_NO_FIX','GPS_SATS_LOW','GPS_HDOP_HIGH','BARO_MISSING',
 'BATTERY','MOTOR_TEST','CONFIG_WRITE','FAILSAFE','BOARD_TILT','SAFE_BOOT','IMU_INVALID',
 'ESC_NOT_READY','ARM_SWITCH_CYCLE_REQUIRED','NAV_MODE_UNSAFE'];
const AF_HINT={THROTTLE_HIGH:'lower the throttle stick',RX_INVALID:'no RC link',
 GYRO_NOT_CAL:'run gyro calibration',GYRO_UNSTABLE:'keep the craft still',
 ACCEL_NOT_CAL:'run accel calibration',MAG_NOT_CAL:'calibrate the compass',
 GPS_NO_FIX:'wait for GPS fix',GPS_SATS_LOW:'wait for more satellites',
 GPS_HDOP_HIGH:'GPS accuracy poor',BARO_MISSING:'barometer unhealthy',
 BATTERY:'battery low or absent',MOTOR_TEST:'motor test running',
 CONFIG_WRITE:'config write in progress',FAILSAFE:'clear the failsafe',
 BOARD_TILT:'level the craft',SAFE_BOOT:'hold throttle low after boot',
 IMU_INVALID:'IMU not ready',ESC_NOT_READY:'ESC init incomplete',
 ARM_SWITCH_CYCLE_REQUIRED:'cycle the arm switch OFF then ON',
 NAV_MODE_UNSAFE:'nav mode selected without GPS/compass quality'};
function headingSource(m){
 const g=m.sen&&m.sen.mag,e=g&&g.ext||{};
 if(g&&g.v===1&&g.src===2)return{deg:+(e.hdg??g.hdg)||0,src:'mag'};
 return{deg:((+m.att.ry%360)+360)%360,src:'gyro'};
}
function drawAH(cv,rollDeg,pitchDeg){
 const d=Math.min(window.devicePixelRatio||1,2),S=330;
 if(cv.width!==S*d){cv.width=S*d;cv.height=S*d;}
 const x=cv.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,S,S);
 const c=S/2,R=c-6,ppd=3.2; // pixels per degree of pitch
 x.save();x.beginPath();x.arc(c,c,R,0,7);x.clip();
 x.translate(c,c);x.rotate(-rollDeg*Math.PI/180);
 const off=pitchDeg*ppd;
 // sky / ground split by the horizon (positive pitch = nose up = more ground visible below center)
 x.fillStyle='#2d6cd0';x.fillRect(-S,-S*1.5+off,2*S,1.5*S);
 x.fillStyle='#7a5230';x.fillRect(-S,off,2*S,1.5*S);
 x.strokeStyle='#e7ebf2';x.lineWidth=2;x.beginPath();x.moveTo(-S,off);x.lineTo(S,off);x.stroke();
 // pitch ladder every 10 deg, minor 5 deg
 x.font='10px system-ui';x.textAlign='center';x.lineWidth=1.2;
 for(let p=-40;p<=40;p+=5){if(p===0)continue;const y=off+(-p)*ppd*2;const w=(p%10===0)?38:20;
  x.strokeStyle=p>0?'#ffffffcc':'#ffffff88';x.beginPath();x.moveTo(-w,y);x.lineTo(w,y);x.stroke();
  if(p%10===0){x.fillStyle='#fff';x.fillText(Math.abs(p),w+13,y+3);x.fillText(Math.abs(p),-w-13,y+3);}}
 x.restore();
 // roll arc + pointer
 x.save();x.translate(c,c);
 x.strokeStyle='#8a94a6';x.lineWidth=1.5;
 for(const a of[-60,-45,-30,-20,-10,0,10,20,30,45,60]){
  const t=(a-90)*Math.PI/180,l=(a%30===0)?12:7;
  x.beginPath();x.moveTo((R-2)*Math.cos(t),(R-2)*Math.sin(t));
  x.lineTo((R-2-l)*Math.cos(t),(R-2-l)*Math.sin(t));x.stroke();}
 x.rotate(-rollDeg*Math.PI/180);
 x.fillStyle='#ffb020';x.beginPath();x.moveTo(0,-R+16);x.lineTo(-7,-R+30);x.lineTo(7,-R+30);x.closePath();x.fill();
 x.restore();
 // fixed reticle (the aircraft symbol never moves)
 x.strokeStyle='#ffb020';x.lineWidth=3;x.beginPath();
 x.moveTo(c-58,c);x.lineTo(c-18,c);x.moveTo(c+18,c);x.lineTo(c+58,c);
 x.moveTo(c-18,c);x.lineTo(c-8,c+9);x.lineTo(c,c);x.lineTo(c+8,c+9);x.lineTo(c+18,c);x.stroke();
 x.strokeStyle='#28303f';x.lineWidth=2;x.beginPath();x.arc(c,c,R,0,7);x.stroke();
}
function drawTape(cv,hdgDeg){
 const{ctx:x,W,H}=canvasCtx(cv);
 const ppd=W/80; // 80 deg visible
 x.font='11px system-ui';x.textAlign='center';
 for(let a=-45;a<=45;a+=5){
  const deg=((Math.round((hdgDeg+a)/5)*5)%360+360)%360;
  const dd=(deg-hdgDeg+540)%360-180;const xx=W/2+dd*ppd;
  if(xx<-10||xx>W+10)continue;
  const major=deg%30===0;
  x.strokeStyle=major?'#e7ebf2':'#5d6678';x.lineWidth=major?2:1;
  x.beginPath();x.moveTo(xx,H-14);x.lineTo(xx,H-(major?30:22));x.stroke();
  if(major){const names={0:'N',90:'E',180:'S',270:'W'};
   x.fillStyle=names[deg]?'#3fa9ff':'#8a94a6';x.fillText(names[deg]||deg,xx,H-33);}}
 x.fillStyle='#ffb020';x.beginPath();x.moveTo(W/2,H-2);x.lineTo(W/2-6,H-12);x.lineTo(W/2+6,H-12);x.closePath();x.fill();
}
let suMagAcc=0;
function renderSetup(m){
 const st=m.sys,att=m.att,sen=m.sen;
 const h=headingSource(m);
 drawAH(document.getElementById('su_ah'),+att.cr||0,+att.cp||0);
 drawTape(document.getElementById('su_tape'),h.deg);
 // compass rose shares the container with the ADI; same heading source as the tape
 suMagAcc+=((h.deg-suMagAcc)%360+540)%360-180;
 const snd=document.getElementById('su_needle');if(snd)snd.style.transform='rotate('+suMagAcc+'deg)';
 const sht=document.getElementById('su_hdg');if(sht)sht.textContent=f(h.deg,0)+'° '+compassPoint(h.deg);
 const shs=document.getElementById('su_hdgsrc');if(shs)shs.textContent=h.src==='mag'?'external mag':'gyro (mag unavailable)';
 const srg=document.getElementById('su_ring');
 if(srg){const mg=sen.mag,mok=mg&&mg.v===1&&mg.src===2;
  srg.setAttribute('stroke',mok?magColour(+((mg.ext&&mg.ext.f)??mg.f)||0):'#5d6678');}
 document.getElementById('su_nums').innerHTML=
  kv('Roll / Pitch',f(att.cr,1)+'° / '+f(att.cp,1)+'°')
 +kv('Gyro yaw',f(att.ry,1)+'°')
 +kv('Mag field',sen.mag?f((sen.mag.ext&&sen.mag.ext.f)??sen.mag.f,1)+' µT':'--')
 +kv('Baro altitude',sen.baro.v?f(sen.baro.alt,2)+' m':'<span class="dim">--</span>')
 +kv('Loop',st.loop+' Hz · max '+st.maxus+' µs · ovr '+st.ovr);
 // arming card
 const flags=+st.armf||0,latch=!!st.armlatch;
 const tag=document.getElementById('su_armtag');
 if(st.armed){tag.className='tag err';tag.textContent='ARMED';}
 else if(flags===0){tag.className='tag ok';tag.textContent='READY TO ARM';}
 else{tag.className='tag warn';tag.textContent='BLOCKED';}
 let ah='';
 if(st.armed)ah='<div class="warnbox err">Craft is ARMED. Do not touch props.</div>';
 else if(flags===0)ah='<div class="warnbox ok">All arming checks pass.</div>';
 else{ah='<div class="dim small" style="margin-bottom:6px">Arming disabled because:</div>';
  for(let b=0;b<AF_NAMES.length;b++)if(flags&(1<<b))
   ah+='<div class="warnbox">'+AF_NAMES[b]+(AF_HINT[AF_NAMES[b]]?' — <span class="dim">'+AF_HINT[AF_NAMES[b]]+'</span>':'')+'</div>';}
 document.getElementById('su_arm').innerHTML=ah;
 // sensor health card
 const g=sen.gps,mg=sen.mag,e=mg&&mg.ext||{};
 document.getElementById('su_sens').innerHTML=
  kv('IMU',(m.imu.rdy?led('ok'):led('err'))+(m.imu.rdy?'ready':'not ready'))
 +kv('Gyro cal',(m.imu.gbv?led('ok'):led('warn'))+(m.imu.gbv?'valid':'missing'))
 +kv('Accel cal',(m.imu.av?led('ok'):led('warn'))+(m.imu.av?'valid':'missing'))
 +kv('GPS',(g.comp?(g.fix?led('ok'):led('warn')):led('grey'))+(g.comp?(g.fix?'fix q'+g.q:'no fix')+' · '+g.sats+' sats · HDOP '+(g.hdv?f(g.hdop,1):'--'):'not compiled'))
 +kv('Compass',((mg.v===1)?led('ok'):led('warn'))+((e.cal||mg.cal)?'calibrated':'uncalibrated')+' · '+f(e.f??mg.f,0)+' µT')
 +kv('Baro',(sen.baro.v?led('ok'):led('err'))+(sen.baro.v?f(sen.baro.alt,2)+' m':'invalid'))
 +kv('Rangefinder',(sen.tof.comp?(sen.tof.rng?led('ok'):led('warn')):led('grey'))+(sen.tof.comp?(sen.tof.rng?sen.tof.mm+' mm':'not ranging'):'not compiled'));
 alignTick(m,h);
}
// ---- board alignment wizard ----
// Steps verify estimator sign conventions against physical motion.
const AL_STEPS=[
 {id:'level',t:'Place the craft LEVEL and still',chk:(m)=>Math.abs(m.att.cr)<6&&Math.abs(m.att.cp)<6,
  fail:null,hint:'roll/pitch should read near 0°'},
 {id:'nose',t:'Tilt the NOSE DOWN ~20°',chk:(m)=>m.att.cp<-12,fail:(m)=>m.att.cp>12,
  hint:'pitch must go NEGATIVE (horizon drops). If it goes positive, pitch axis is reversed.'},
 {id:'roll',t:'Return level, then ROLL RIGHT ~20° (right side down)',chk:(m)=>m.att.cr>12,fail:(m)=>m.att.cr<-12,
  hint:'roll must go POSITIVE. If it goes negative, roll axis is reversed.'},
 {id:'yaw',t:'Return level, then YAW CLOCKWISE ~45° (viewed from above)',chk:null,fail:null,
  hint:'heading must INCREASE clockwise. Uses compass when valid, else gyro yaw.'},
];
let AL={run:false,step:0,state:[],yawRef:null,dwell:0};
function alignStart(){AL={run:true,step:0,state:AL_STEPS.map(()=>0),yawRef:null,dwell:0};
 document.getElementById('su_albtn').textContent='Restart';document.getElementById('su_alskip').disabled=false;alignDraw();}
function alignSkip(){if(!AL.run)return;AL.state[AL.step]=AL.state[AL.step]||3;alignAdvance();}
function alignAdvance(){if(AL.step<AL_STEPS.length-1){AL.step++;AL.yawRef=null;AL.dwell=0;}
 else{AL.run=false;document.getElementById('su_alskip').disabled=true;
  document.getElementById('su_altag').textContent=AL.state.every(s=>s===2)?'PASS':'CHECK FAILED STEPS';
  document.getElementById('su_altag').className='tag '+(AL.state.every(s=>s===2)?'ok':'warn');}
 alignDraw();}
function alignTick(m,h){
 if(!AL.run)return;
 const s=AL_STEPS[AL.step];let pass=false,fail=false;
 if(s.id==='yaw'){
  if(AL.yawRef==null)AL.yawRef=h.deg;
  const dd=((h.deg-AL.yawRef+540)%360)-180;
  pass=dd>20;fail=dd<-20;
 }else{pass=s.chk&&s.chk(m);fail=s.fail&&s.fail(m);}
 if(fail&&AL.state[AL.step]!==2){AL.state[AL.step]=3;alignDraw();}
 if(pass){if(AL.state[AL.step]!==2){AL.state[AL.step]=2;AL.dwell=Date.now();alignDraw();}
  else if(Date.now()-AL.dwell>900)alignAdvance();}
}
function alignDraw(){
 const el=document.getElementById('su_alsteps');if(!el)return;
 if(!AL.run&&AL.state.every(s=>s===0)){el.innerHTML='<div class="dim small">Not started.</div>';return;}
 el.innerHTML=AL_STEPS.map((s,i)=>{
  const st=AL.state[i],cur=AL.run&&i===AL.step;
  const icon=st===2?'&#10003;':(st===3?'&#10007;':(cur?'&#9654;':'&#9675;'));
  const cls=st===2?'ok':(st===3?'err':'');
  return '<div class="warnbox '+cls+'" style="'+(cur?'outline:1px solid var(--accent);':'')+'">'
   +icon+' <b>Step '+(i+1)+':</b> '+s.t+' <span class="dim small">— '+s.hint+'</span></div>';
 }).join('');
 document.getElementById('su_altag').textContent=AL.run?('step '+(AL.step+1)+'/'+AL_STEPS.length):document.getElementById('su_altag').textContent;
}
// ===== Mission planner tab (proof of concept) ================================
// INAV-style waypoint planner. Leaflet + tiles are fetched from the internet
// ONLY when this tab is first opened (SRI-pinned), so the rest of the dashboard
// stays fully offline on the FCU AP. Planning is client-side: missions persist
// in localStorage and export/import as JSON. Upload to the FC is NOT wired yet.
const MS={map:null,wps:[],sel:-1,rth:true,base:null,lbl:null,layer:'sat',leaf:0,
 route:[],rthLine:null,mkrs:[],home:null,homeLL:null,drone:null,centered:false,pv:null,saveT:0,homeWas:false};
const MS_DEF={alt:30,spd:8,hold:5};
const MS_ACT=[['WP','Waypoint'],['HOLD','Hold'],['LAND','Land']];
const MS_TILES={
 dark:['https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',{maxZoom:20,subdomains:'abcd',attribution:'&copy; OpenStreetMap &copy; CARTO'}],
 sat:['https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',{maxZoom:19,attribution:'Imagery &copy; Esri'}],
 osm:['https://tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap'}]};
function msLoadLeaflet(){
 if(MS.leaf===2)return Promise.resolve(true);
 if(window._msLoading)return window._msLoading;
 window._msLoading=new Promise(res=>{
  const css=document.createElement('link');css.rel='stylesheet';
  css.href='https://unpkg.com/leaflet@1.9.4/dist/leaflet.css';
  css.integrity='sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=';css.crossOrigin='';
  document.head.appendChild(css);
  const s=document.createElement('script');
  s.src='https://unpkg.com/leaflet@1.9.4/dist/leaflet.js';
  s.integrity='sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo=';s.crossOrigin='';
  s.onload=()=>{MS.leaf=2;res(true);};
  s.onerror=()=>{window._msLoading=null;res(false);};
  document.head.appendChild(s);});
 return window._msLoading;
}
STAGE.mission=()=>{
 if(MS.map){setTimeout(()=>MS.map.invalidateSize(),60);return;}
 msBoot();
};
async function msBoot(){
 const b=document.getElementById('ms_boot');b.style.display='flex';
 b.innerHTML='<div class="ms-spin"></div><div class="dim">Loading map engine&hellip;</div>';
 const ok=await msLoadLeaflet();
 if(!ok){b.innerHTML='<div style="font-size:30px">&#128246;</div>'
  +'<div style="max-width:320px"><b>No route to the map CDN.</b><br><span class="dim small">The map library and tiles need the browsing device to be online. Connect this device to the internet (or run the FCU in STA mode on a routed network) and retry.</span></div>'
  +'<button class="btn pri" onclick="msBoot()">Retry</button>';return;}
 msInit();
}
function msInit(){
 const saved=msLoadStore();
 MS.map=L.map('ms_map',{zoomControl:false,worldCopyJump:true});
 L.control.zoom({position:'topright'}).addTo(MS.map);
 L.control.scale({imperial:false,position:'bottomright'}).addTo(MS.map);
 msSetLayer((saved&&saved.layer)||'sat');
 let c=[42.30453,-83.06233],z=17; // default: U Windsor Ed Lumley CEI; saved view / GPS fix / home override
 const g=window.LAST&&LAST.sen&&LAST.sen.gps;
 if(saved&&saved.view&&saved.view.c){c=saved.view.c;z=saved.view.z;MS.centered=true;}
 else if(g&&g.home&&g.home.set){c=[g.home.lat/1e7,g.home.lon/1e7];z=17;MS.centered=true;}
 else if(g&&g.fix){c=[g.lat/1e7,g.lon/1e7];z=17;MS.centered=true;}
 MS.map.setView(c,z);
 MS.map.on('click',e=>{if(MS.pv)return;msAdd(e.latlng);});
 MS.map.on('mousemove',e=>{const el=document.getElementById('ms_coords');
  if(el)el.textContent=e.latlng.lat.toFixed(5)+', '+e.latlng.lng.toFixed(5);});
 MS.map.on('moveend zoomend',msPersist);
 if(saved&&Array.isArray(saved.wps))MS.wps=saved.wps;
 if(saved&&saved.rth!=null)MS.rth=!!saved.rth;
 document.getElementById('ms_rth').checked=MS.rth;
 document.getElementById('ms_boot').style.display='none';
 msRefresh(true);
 if(window.LAST)msTelem(LAST);
 setTimeout(()=>MS.map.invalidateSize(),60);
}
function msSetLayer(name){
 MS.layer=name;
 document.querySelectorAll('#ms_layers button').forEach(b=>b.classList.toggle('active',b.dataset.l===name));
 if(!MS.map)return;
 if(MS.base)MS.map.removeLayer(MS.base);
 if(MS.lbl){MS.map.removeLayer(MS.lbl);MS.lbl=null;}
 const t=MS_TILES[name]||MS_TILES.sat;
 MS.base=L.tileLayer(t[0],t[1]).addTo(MS.map);
 if(name==='sat')MS.lbl=L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_only_labels/{z}/{x}/{y}{r}.png',
  {maxZoom:20,subdomains:'abcd'}).addTo(MS.map);
 msPersist();
}
function msAdd(ll){
 MS.wps.push({lat:+ll.lat.toFixed(7),lon:+ll.lng.toFixed(7),alt:MS_DEF.alt,spd:MS_DEF.spd,hold:MS_DEF.hold,act:'WP'});
 MS.sel=MS.wps.length-1;msRefresh(true);
}
function msRefresh(struct){
 if(struct){msRebuildMarkers();msList();}
 msRoute();msStats();msPersist();
 const em=document.getElementById('ms_hint');if(em)em.style.display=MS.wps.length?'none':'block';
}
function msIcon(i){
 const w=MS.wps[i];
 const cls='ms-wp'+(w.act==='HOLD'?' hold':(w.act==='LAND'?' land':''))+(i===MS.sel?' sel':'');
 return L.divIcon({className:'ms-nul',html:'<div class="'+cls+'">'+(i+1)+'</div>',iconSize:[32,32],iconAnchor:[16,16]});
}
function msTipTxt(i){const w=MS.wps[i];return 'WP'+(i+1)+' &middot; '+w.alt+' m &middot; '+w.spd+' m/s';}
function msRebuildMarkers(){
 if(!MS.map)return;
 MS.mkrs.forEach(m=>MS.map.removeLayer(m));MS.mkrs=[];
 MS.wps.forEach((w,i)=>{
  const mk=L.marker([w.lat,w.lon],{icon:msIcon(i),draggable:true,keyboard:false});
  mk.on('click',()=>msSelect(i));
  mk.on('drag',()=>{const p=mk.getLatLng();w.lat=+p.lat.toFixed(7);w.lon=+p.lng.toFixed(7);msRoute();msStats();});
  mk.on('dragend',()=>{msList();msPersist();});
  mk.bindTooltip(msTipTxt(i),{direction:'top',offset:[0,-16],className:'ms-tip'});
  mk.addTo(MS.map);MS.mkrs.push(mk);});
}
function msSelect(i){
 MS.sel=i;MS.mkrs.forEach((m,j)=>m.setIcon(msIcon(j)));msList();
 const el=document.querySelector('.ms-item[data-i="'+i+'"]');
 if(el)el.scrollIntoView({block:'nearest',behavior:'smooth'});
}
function msPan(i){const w=MS.wps[i];if(w&&MS.map)MS.map.panTo([w.lat,w.lon]);}
function msRoute(){
 if(!MS.map)return;
 MS.route.forEach(l=>MS.map.removeLayer(l));MS.route=[];
 if(MS.rthLine){MS.map.removeLayer(MS.rthLine);MS.rthLine=null;}
 const pts=MS.wps.map(w=>[w.lat,w.lon]);
 if(MS.homeLL&&pts.length)
  MS.route.push(L.polyline([MS.homeLL,pts[0]],{color:'#33d17a',weight:2,opacity:.6,dashArray:'2 8',interactive:false}).addTo(MS.map));
 if(pts.length>1){
  MS.route.push(L.polyline(pts,{color:'#3fa9ff',weight:10,opacity:.16,lineCap:'round',interactive:false}).addTo(MS.map));
  MS.route.push(L.polyline(pts,{color:'#3fa9ff',weight:3,opacity:.95,interactive:false}).addTo(MS.map));
  MS.route.push(L.polyline(pts,{color:'#e8f4ff',weight:2,opacity:.9,className:'ms-route-dash',interactive:false}).addTo(MS.map));
 }
 if(MS.rth&&MS.homeLL&&pts.length)
  MS.rthLine=L.polyline([pts[pts.length-1],MS.homeLL],{color:'#ffb020',weight:2.5,opacity:.85,dashArray:'7 9',interactive:false}).addTo(MS.map);
}
function msHav(a,b){
 const R=6371000,dr=Math.PI/180;
 const dLa=(b[0]-a[0])*dr,dLo=(b[1]-a[1])*dr,la1=a[0]*dr,la2=b[0]*dr;
 const h=Math.sin(dLa/2)**2+Math.cos(la1)*Math.cos(la2)*Math.sin(dLo/2)**2;
 return 2*R*Math.asin(Math.sqrt(h));
}
function msStats(){
 let d=0,t=0;const w=MS.wps;
 for(let i=1;i<w.length;i++){const leg=msHav([w[i-1].lat,w[i-1].lon],[w[i].lat,w[i].lon]);d+=leg;t+=leg/Math.max(1,w[i].spd);}
 w.forEach(x=>{if(x.act==='HOLD')t+=x.hold;});
 if(MS.homeLL&&w.length){
  const e=msHav(MS.homeLL,[w[0].lat,w[0].lon]);d+=e;t+=e/Math.max(1,w[0].spd);
  if(MS.rth){const r=msHav([w[w.length-1].lat,w[w.length-1].lon],MS.homeLL);d+=r;t+=r/MS_DEF.spd;}
 }
 document.getElementById('ms_s_n').textContent=w.length;
 document.getElementById('ms_cnt').textContent=w.length;
 document.getElementById('ms_s_d').textContent=w.length?(d>=1000?(d/1000).toFixed(2)+' km':d.toFixed(0)+' m'):'--';
 document.getElementById('ms_s_t').textContent=w.length?(Math.floor(t/60)+':'+String(Math.round(t%60)).padStart(2,'0')):'--';
}
function msList(){
 const el=document.getElementById('ms_list');if(!el)return;
 if(!MS.wps.length){el.innerHTML='<div class="ms-none">No waypoints yet.<br><span class="dim small">Click the map to drop WP1.</span></div>';return;}
 el.innerHTML=MS.wps.map((w,i)=>{
  return '<div class="ms-item'+(i===MS.sel?' sel':'')+'" data-i="'+i+'" onclick="msSelect('+i+');msPan('+i+')">'
  +'<div class="hd"><span class="ms-num '+w.act.toLowerCase()+'">'+(i+1)+'</span>'
  +'<select class="ms-act" onclick="event.stopPropagation()" onchange="msField('+i+',&quot;act&quot;,this.value)">'
   +MS_ACT.map(a=>'<option value="'+a[0]+'"'+(w.act===a[0]?' selected':'')+'>'+a[1]+'</option>').join('')
  +'</select><span class="spacer"></span>'
  +'<button class="ms-ib" title="Move up" onclick="event.stopPropagation();msMove('+i+',-1)">&#9650;</button>'
  +'<button class="ms-ib" title="Move down" onclick="event.stopPropagation();msMove('+i+',1)">&#9660;</button>'
  +'<button class="ms-ib del" title="Delete" onclick="event.stopPropagation();msDel('+i+')">&#10005;</button></div>'
  +'<div class="ms-coord mono">'+(+w.lat).toFixed(6)+', '+(+w.lon).toFixed(6)+'</div>'
  +'<div class="ms-fields">'
   +'<div><label>Alt m</label><input type="number" value="'+w.alt+'" min="1" max="500" step="1" onclick="event.stopPropagation()" onchange="msField('+i+',&quot;alt&quot;,+this.value)"></div>'
   +'<div><label>Spd m/s</label><input type="number" value="'+w.spd+'" min="1" max="25" step="0.5" onclick="event.stopPropagation()" onchange="msField('+i+',&quot;spd&quot;,+this.value)"></div>'
   +(w.act==='HOLD'?'<div><label>Hold s</label><input type="number" value="'+w.hold+'" min="1" max="600" step="1" onclick="event.stopPropagation()" onchange="msField('+i+',&quot;hold&quot;,+this.value)"></div>':'')
  +'</div></div>';}).join('');
}
function msField(i,k,v){
 const w=MS.wps[i];if(!w)return;
 if(k==='act'){w.act=v;msRefresh(true);return;}
 if(k==='alt')w.alt=Math.max(1,Math.min(500,v||MS_DEF.alt));
 if(k==='spd')w.spd=Math.max(1,Math.min(25,v||MS_DEF.spd));
 if(k==='hold')w.hold=Math.max(1,Math.min(600,v||MS_DEF.hold));
 if(MS.mkrs[i])MS.mkrs[i].setTooltipContent(msTipTxt(i));
 msStats();msPersist();
}
function msMove(i,d){
 const j=i+d;if(j<0||j>=MS.wps.length)return;
 const t=MS.wps[i];MS.wps[i]=MS.wps[j];MS.wps[j]=t;
 if(MS.sel===i)MS.sel=j;else if(MS.sel===j)MS.sel=i;
 msRefresh(true);
}
function msDel(i){
 MS.wps.splice(i,1);
 if(MS.sel===i)MS.sel=-1;else if(MS.sel>i)MS.sel--;
 msRefresh(true);
}
function msUndo(){if(!MS.wps.length)return;MS.wps.pop();if(MS.sel>=MS.wps.length)MS.sel=-1;msRefresh(true);}
function msClear(){
 if(!MS.wps.length)return;
 if(!confirm('Delete all '+MS.wps.length+' waypoints?'))return;
 MS.wps=[];MS.sel=-1;msRefresh(true);
}
function msRthTgl(v){MS.rth=!!v;msRoute();msStats();msPersist();}
function msFit(){
 if(!MS.map)return;
 const pts=MS.wps.map(w=>[w.lat,w.lon]);
 if(MS.homeLL)pts.push(MS.homeLL);
 if(MS.drone){const p=MS.drone.getLatLng();pts.push([p.lat,p.lng]);}
 if(!pts.length){toast('Nothing to fit yet',true);return;}
 if(pts.length===1){MS.map.setView(pts[0],16);return;}
 MS.map.fitBounds(L.latLngBounds(pts),{padding:[46,46]});
}
function msPersist(){
 clearTimeout(MS.saveT);
 MS.saveT=setTimeout(()=>{try{
  const v=MS.map?{c:[MS.map.getCenter().lat,MS.map.getCenter().lng],z:MS.map.getZoom()}:null;
  localStorage.setItem('fcu_mission',JSON.stringify({wps:MS.wps,rth:MS.rth,layer:MS.layer,view:v}));
 }catch(e){}},400);
}
function msLoadStore(){try{return JSON.parse(localStorage.getItem('fcu_mission')||'null');}catch(e){return null;}}
function msExport(){
 if(!MS.wps.length){toast('No waypoints to export',true);return;}
 const wp=MS.wps.map((w,i)=>({no:i+1,action:w.act==='HOLD'?'POSHOLD_TIME':(w.act==='LAND'?'LAND':'WAYPOINT'),
  lat:w.lat,lon:w.lon,alt_m:w.alt,speed_ms:w.spd,hold_s:w.act==='HOLD'?w.hold:0}));
 const blob=new Blob([JSON.stringify({fcu_mission:1,rth:MS.rth,wp:wp},null,1)],{type:'application/json'});
 const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='mission.json';a.click();
 setTimeout(()=>URL.revokeObjectURL(a.href),3000);toast('Mission exported');
}
function msImport(inp){
 const f=inp.files&&inp.files[0];if(!f)return;
 const r=new FileReader();
 r.onload=()=>{try{
  const j=JSON.parse(r.result);const src=j.wp||j.wps||[];
  MS.wps=src.map(w=>({lat:+w.lat,lon:+(w.lon!=null?w.lon:w.lng),
   alt:+(w.alt_m!=null?w.alt_m:(w.alt!=null?w.alt:MS_DEF.alt)),
   spd:+(w.speed_ms!=null?w.speed_ms:(w.spd!=null?w.spd:MS_DEF.spd)),
   hold:(+(w.hold_s!=null?w.hold_s:(w.hold!=null?w.hold:MS_DEF.hold)))||MS_DEF.hold,
   act:(w.action==='POSHOLD_TIME'||w.act==='HOLD')?'HOLD':((w.action==='LAND'||w.act==='LAND')?'LAND':'WP')}))
   .filter(w=>isFinite(w.lat)&&isFinite(w.lon)&&Math.abs(w.lat)<=90&&Math.abs(w.lon)<=180);
  if(j.rth!=null)MS.rth=!!j.rth;
  document.getElementById('ms_rth').checked=MS.rth;
  MS.sel=-1;msRefresh(true);msFit();
  toast('Mission imported: '+MS.wps.length+' waypoints');
 }catch(e){toast('Import failed: not a valid mission file',true);}};
 r.readAsText(f);inp.value='';
}
function msUpload(){toast('Mission upload endpoint is not wired up yet',true);}
function msPreview(){
 if(MS.pv){msPvStop();return;}
 if(!MS.map)return;
 const pts=[];
 if(MS.homeLL&&MS.wps.length)pts.push({p:MS.homeLL,spd:MS.wps[0].spd,hold:0});
 MS.wps.forEach(w=>pts.push({p:[w.lat,w.lon],spd:w.spd,hold:w.act==='HOLD'?w.hold:0}));
 if(MS.rth&&MS.homeLL&&MS.wps.length)pts.push({p:MS.homeLL,spd:MS_DEF.spd,hold:0});
 if(pts.length<2){toast('Need at least 2 waypoints (or home + 1 WP)',true);return;}
 const mk=L.marker(pts[0].p,{icon:L.divIcon({className:'ms-nul',html:'<div class="ms-ghost"></div>',iconSize:[18,18],iconAnchor:[9,9]}),
  interactive:false,zIndexOffset:900}).addTo(MS.map);
 MS.pv={mk:mk,seg:1,t0:performance.now(),legT:0,holdT:0,raf:0};
 document.getElementById('ms_pvbtn').innerHTML='&#9632;&nbsp; Stop preview';
 const step=now=>{
  const pv=MS.pv;if(!pv)return;
  let dt=(now-pv.t0)/1000*8;pv.t0=now; // 8x real time
  while(dt>0&&pv.seg<pts.length){
   const a=pts[pv.seg-1],b=pts[pv.seg];
   const legDur=msHav(a.p,b.p)/Math.max(1,b.spd);
   if(pv.legT<legDur){
    const adv=Math.min(dt,legDur-pv.legT);pv.legT+=adv;dt-=adv;
    const f=legDur>0?pv.legT/legDur:1;
    mk.setLatLng([a.p[0]+(b.p[0]-a.p[0])*f,a.p[1]+(b.p[1]-a.p[1])*f]);
   }else if(pv.holdT<b.hold){const adv=Math.min(dt,b.hold-pv.holdT);pv.holdT+=adv;dt-=adv;}
   else{pv.seg++;pv.legT=0;pv.holdT=0;}
  }
  if(pv.seg>=pts.length){msPvStop();return;}
  pv.raf=requestAnimationFrame(step);};
 MS.pv.raf=requestAnimationFrame(step);
}
function msPvStop(){
 if(!MS.pv)return;
 cancelAnimationFrame(MS.pv.raf);
 if(MS.map)MS.map.removeLayer(MS.pv.mk);
 MS.pv=null;
 const b=document.getElementById('ms_pvbtn');if(b)b.innerHTML='&#9654;&nbsp; Preview flight';
}
function msHomeIcon(){return L.divIcon({className:'ms-nul',iconSize:[28,28],iconAnchor:[14,14],
 html:'<div class="ms-home"><svg viewBox="0 0 24 24" width="13" height="13" fill="#fff"><path d="M12 3l9 8h-3v9h-4v-6h-4v6H6v-9H3z"/></svg></div>'});}
function msDroneIcon(){return L.divIcon({className:'ms-nul',iconSize:[34,34],iconAnchor:[17,17],
 html:'<div class="ms-dwrap"><div class="ms-pulse"></div><div class="ms-drone-rot"><svg viewBox="0 0 24 24" width="20" height="20"><path d="M12 2L19 20L12 15L5 20Z" fill="#25e0ff" stroke="#062a33" stroke-width="1"/></svg></div></div>'});}
function msTelem(m){
 const g=m.sen&&m.sen.gps;if(!g)return;
 const fx=document.getElementById('ms_fix');
 if(fx){const b=fx.querySelector('b');
  if(!g.comp){b.textContent='n/a';b.style.color='var(--dim2)';}
  else if(g.fix){b.textContent=g.sats+' sats';b.style.color='var(--ok)';}
  else{b.textContent='no fix';b.style.color='var(--warn)';}}
 const h=g.home||{};
 if(h.set)MS.homeLL=[h.lat/1e7,h.lon/1e7];
 if(!MS.map)return;
 if(h.set){
  if(!MS.home)MS.home=L.marker(MS.homeLL,{icon:msHomeIcon(),interactive:false,zIndexOffset:500}).addTo(MS.map);
  else MS.home.setLatLng(MS.homeLL);
  if(!MS.homeWas){MS.homeWas=true;msRoute();msStats();}
 }
 if(g.fix){
  const ll=[g.lat/1e7,g.lon/1e7];
  if(!MS.drone)MS.drone=L.marker(ll,{icon:msDroneIcon(),interactive:false,zIndexOffset:1000}).addTo(MS.map);
  else MS.drone.setLatLng(ll);
  const rot=MS.drone._icon&&MS.drone._icon.querySelector('.ms-drone-rot');
  if(rot)rot.style.transform='rotate('+headingSource(m).deg.toFixed(0)+'deg)';
  if(!MS.centered){MS.centered=true;MS.map.setView(ll,16);}
 }
}
(function(){const prev=window.onRender;window.onRender=m=>{if(prev)prev(m);if(activeTab==='mission')msTelem(m);};})();
</script>
</body></html>)DASH";
