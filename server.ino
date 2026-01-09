#include <WiFi.h>
#include <WebServer.h>

// ================= CẤU HÌNH WIFI =================
const char* ssid = "Robot_Control";
const char* password = "12345678";

WebServer server(80);

// ================= GIAO DIỆN WEB (HTML CỦA BẠN) =================
// Đặt trong PROGMEM để tiết kiệm RAM
const char index_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover" />
  <meta name="color-scheme" content="dark" />
  <title>ROBOT TUNING</title>
  <style>
    :root{ --bg:#0b0f14; --panel:#111824; --card:#0f1622; --card2:#0d1420; --border:rgba(255,255,255,.08); --text:#e9eef7; --muted:rgba(233,238,247,.65); --muted2:rgba(233,238,247,.45); --accent:#7cf5d2; --warn:#ffd37c; --bad:#ff6b6b; --ok:#60ffa1; --radius:16px; --gap:12px; --shadow: 0 10px 30px rgba(0,0,0,.35); --shadow2: 0 6px 18px rgba(0,0,0,.28); }
    *{ box-sizing:border-box; } html,body{ height:100%; }
    body{ margin:0; font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif; background: radial-gradient(1000px 600px at 10% 0%, rgba(124,245,210,.10), transparent 55%), radial-gradient(900px 500px at 90% 10%, rgba(255,211,124,.08), transparent 55%), var(--bg); color:var(--text); -webkit-tap-highlight-color: transparent; }
    .wrap{ max-width: 860px; margin: 0 auto; padding: 16px 14px 86px; }
    header{ display:flex; align-items:flex-end; justify-content:space-between; gap:12px; padding: 10px 4px 14px; }
    .title{ display:flex; flex-direction:column; gap:4px; min-width:0; }
    h1{ margin:0; font-size: 18px; letter-spacing:.08em; text-transform:uppercase; font-weight:800; }
    .sub{ font-size:12px; color:var(--muted); white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
    .grid{ display:grid; grid-template-columns: 1fr; gap: var(--gap); }
    @media (min-width: 820px){ .grid{ grid-template-columns: 1fr 1fr; } }
    .card{ background: linear-gradient(180deg, rgba(255,255,255,.03), transparent 22%), linear-gradient(180deg, var(--card), var(--card2)); border: 1px solid var(--border); border-radius: var(--radius); box-shadow: var(--shadow2); overflow:hidden; }
    .card .hd{ padding: 14px 14px 10px; border-bottom: 1px solid var(--border); display:flex; align-items:center; justify-content:space-between; gap:10px; }
    .card .hd h2{ margin:0; font-size: 14px; letter-spacing:.08em; text-transform:uppercase; color: rgba(233,238,247,.9); }
    .card .bd{ padding: 12px 14px 14px; display:flex; flex-direction:column; gap: 12px; }
    .row{ display:grid; grid-template-columns: 1fr; gap: 10px; padding: 10px; border: 1px solid var(--border); border-radius: 14px; background: rgba(0,0,0,.12); }
    .rowTop{ display:flex; align-items:center; justify-content:space-between; gap:10px; }
    .label{ display:flex; flex-direction:column; min-width:0; gap:2px; }
    .label b{ font-size:13px; font-weight:750; letter-spacing:.02em; }
    .label span{ font-size:11px; color: var(--muted2); }
    .ctrls{ display:flex; align-items:center; gap:10px; flex-shrink:0; }
    input[type="number"]{ width: 108px; padding: 9px 10px; border-radius: 12px; border: 1px solid var(--border); background: rgba(0,0,0,.25); color: var(--text); font-size: 13px; outline:none; box-shadow: inset 0 0 0 1px rgba(0,0,0,.12); }
    input[type="number"]:focus{ border-color: rgba(124,245,210,.45); box-shadow: 0 0 0 3px rgba(124,245,210,.12); }
    input[type="range"]{ width:100%; accent-color: var(--accent); height: 34px; margin: 0; }
    .btnRow{ display:flex; flex-wrap:wrap; gap:10px; }
    button{ appearance:none; border:1px solid var(--border); background: rgba(255,255,255,.05); color: var(--text); padding: 10px 12px; border-radius: 14px; font-weight:700; letter-spacing:.02em; cursor:pointer; user-select:none; box-shadow: var(--shadow2); transition: transform .06s ease, border-color .15s ease, background .15s ease; touch-action: manipulation; }
    button:active{ transform: translateY(1px) scale(.99); }
    button:disabled{ opacity:.55; cursor:not-allowed; transform:none; }
    .primary{ border-color: rgba(124,245,210,.35); background: linear-gradient(180deg, rgba(124,245,210,.18), rgba(124,245,210,.06)); }
    .warn{ border-color: rgba(255,211,124,.35); background: linear-gradient(180deg, rgba(255,211,124,.18), rgba(255,211,124,.06)); }
    .ghost{ background: rgba(255,255,255,.03); }
    .pill{ font-size: 11px; padding: 6px 10px; border-radius: 999px; border:1px solid var(--border); color: var(--muted); background: rgba(0,0,0,.18); white-space:nowrap; }
    .status{ position: fixed; left: 0; right: 0; bottom: 0; padding: 10px 12px calc(10px + env(safe-area-inset-bottom)); background: rgba(10,14,20,.72); backdrop-filter: blur(10px); border-top: 1px solid rgba(255,255,255,.08); display:flex; align-items:center; justify-content:space-between; gap:10px; font-size:12px; z-index: 50; }
    .status .left{ display:flex; flex-direction:column; gap:2px; min-width:0; }
    .status .right{ display:flex; align-items:center; gap:8px; flex-shrink:0; }
    .mono{ font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace; }
    .status .last{ color: var(--muted); white-space:nowrap; overflow:hidden; text-overflow:ellipsis; max-width: 68vw; }
    .status .resp{ padding: 6px 10px; border-radius: 999px; border: 1px solid var(--border); background: rgba(0,0,0,.22); min-width: 88px; text-align:center; font-weight: 800; letter-spacing:.02em; }
    .resp.ok{ color: var(--ok); border-color: rgba(96,255,161,.35); }
    .resp.bad{ color: var(--bad); border-color: rgba(255,107,107,.35); }
    .resp.wait{ color: var(--warn); border-color: rgba(255,211,124,.35); }
    .miniHint{ font-size: 11px; color: var(--muted2); line-height: 1.35; }
  </style>
</head>
<body>
  <div class="wrap">
    <header>
      <div class="title"><h1>ROBOT TUNING</h1><div class="sub">ESP32 WebServer • /set?c=&lt;cmd&gt;&amp;v=&lt;val&gt;</div></div>
      <div class="pill mono" id="ipHint">/</div>
    </header>
    <div class="grid">
      <section class="card"><div class="hd"><h2>PID Control</h2><span class="pill">Kp / Ki / Kd</span></div><div class="bd" id="pidCard"></div></section>
      <section class="card"><div class="hd"><h2>Settings</h2><span class="pill">Target / Speed</span></div><div class="bd" id="settingsCard"></div></section>
      <section class="card"><div class="hd"><h2>Quick Presets</h2><span class="pill">Batch send</span></div><div class="bd"><div class="btnRow"><button class="ghost" id="presetSoft">SOFT</button><button class="primary" id="presetNormal">NORMAL</button><button class="warn" id="presetAggressive">AGGRESSIVE</button></div></div></section>
      <section class="card"><div class="hd"><h2>Actions & Storage</h2><span class="pill">Tools</span></div><div class="bd"><div class="btnRow"><button class="primary" id="btnCalib">AUTO CALIBRATE</button><button class="ghost" id="btnDebug">GET DEBUG INFO</button></div><div class="btnRow" style="margin-top:6px;"><button class="ghost" id="btnSave">Save to localStorage</button><button class="ghost" id="btnLoad">Load</button><button class="ghost" id="btnReset">Reset</button></div></div></section>
    </div>
  </div>
  <div class="status" role="status"><div class="left"><div class="mono last" id="lastSent">Last: (none)</div><div class="mono" id="lastTime" style="color:var(--muted2);">—</div></div><div class="right"><div class="resp wait mono" id="httpState">IDLE</div></div></div>
<script>
(() => {
  "use strict";
  const CFG = {
    endpoint: "/set", fetchTimeoutMs: 1800, debounceSliderMs: 200, debounceNumberMs: 300, batchDelayMs: 140, storageKey: "robot_tuning_v1",
    params: {
      kp: { label: "Kp", desc: "Angle P", cmd: "p", min: 0, max: 400, step: 0.1, def: 68.7 },
      ki: { label: "Ki", desc: "Integral", cmd: "i", min: 0, max: 200, step: 0.01, def: 20.0 },
      kd: { label: "Kd", desc: "Derivative", cmd: "d", min: 0, max: 50, step: 0.01, def: 1.6 },
      target: { label: "Target", desc: "Angle target", cmd: "t", min: 150, max: 210, step: 0.1, def: 180 },
      maxSpeed:{ label: "MaxSpeed", desc: "Stepper cap", cmd: "m", min: 1000,max: 25000,step: 100, def: 15000 },
      speedScale:{ label: "SpeedScale", desc: "Scale factor", cmd: "s", min: 0.1, max: 40, step: 0.1, def: 12.0 },
    },
    presets: {
      soft: { name: "SOFT", values: { kp: 45, ki: 8, kd: 1.6, target: 180, maxSpeed: 12000, speedScale: 10.5 } },
      normal: { name: "NORMAL", values: { kp: 68.7, ki: 20, kd: 1.6, target: 180, maxSpeed: 15000, speedScale: 12.0 } },
      aggressive: { name: "AGGRESSIVE", values: { kp: 90, ki: 30, kd: 3.8, target: 180, maxSpeed: 18000, speedScale: 14.0 } }
    }
  };
  const state = {}, timers = {}, els = {};
  const $ = (sel) => document.querySelector(sel);
  function nowText(){ const d = new Date(); return `${String(d.getHours()).padStart(2,"0")}:${String(d.getMinutes()).padStart(2,"0")}:${String(d.getSeconds()).padStart(2,"0")}`; }
  function clampToStep(value, min, max, step){ let v = Number(value); if (!Number.isFinite(v)) v = min; v = Math.min(max, Math.max(min, v)); const inv = 1 / step; return Math.round(v * inv) / inv; }
  function formatByStep(value, step){ const dp = (String(step).split(".")[1] || "").length; return dp ? Number(value).toFixed(dp) : String(Math.round(value)); }
  function setHttpState(kind, text){ const el = els.httpState; el.classList.remove("ok","bad","wait"); if (kind === "ok") el.classList.add("ok"); else if (kind === "bad") el.classList.add("bad"); else el.classList.add("wait"); el.textContent = text; }
  function setLastSent(cmd, val){ els.lastSent.textContent = `Last: ${cmd}=${val}`; els.lastTime.textContent = `@ ${nowText()}`; }
  async function send(cmd, val){
    const url = `${CFG.endpoint}?c=${encodeURIComponent(cmd)}&v=${encodeURIComponent(val)}`; setLastSent(cmd, val); setHttpState("wait", "SENDING");
    try{ const res = await fetch(url, { method: "GET", cache: "no-store" }); if (!res.ok){ setHttpState("bad", `HTTP ${res.status}`); return false; } setHttpState("ok", "OK"); return true; } 
    catch(err){ setHttpState("bad", "OFFLINE"); return false; }
  }
  function buildParamRow(key, cfg){
    const row = document.createElement("div"); row.className = "row";
    row.innerHTML = `<div class="rowTop"><div class="label"><b>${cfg.label}</b><span>${cfg.desc} • <span class="mono">${cfg.min}..${cfg.max}</span></span></div><div class="ctrls"><input type="number" id="num_${key}"></div></div><div><input type="range" id="rng_${key}"></div>`;
    const num = row.querySelector(`#num_${key}`), rng = row.querySelector(`#rng_${key}`);
    num.min = rng.min = cfg.min; num.max = rng.max = cfg.max; num.step = rng.step = cfg.step;
    state[key] = clampToStep(cfg.def, cfg.min, cfg.max, cfg.step);
    num.value = formatByStep(cfg.def, cfg.step); rng.value = String(cfg.def);
    const sync = (v) => { v = clampToStep(v, cfg.min, cfg.max, cfg.step); state[key] = v; rng.value = String(v); num.value = formatByStep(v, cfg.step); return v; };
    rng.addEventListener("input", () => { const v = sync(rng.value); clearTimeout(timers[key]); timers[key] = setTimeout(() => send(cfg.cmd, v), CFG.debounceSliderMs); });
    num.addEventListener("input", () => { const v = sync(num.value); clearTimeout(timers[key]); timers[key] = setTimeout(() => send(cfg.cmd, v), CFG.debounceNumberMs); });
    return row;
  }
  function render(){
    els.pidCard = $("#pidCard"); els.settingsCard = $("#settingsCard"); els.lastSent = $("#lastSent"); els.lastTime = $("#lastTime"); els.httpState = $("#httpState");
    for(const k of ["kp","ki","kd"]) els.pidCard.appendChild(buildParamRow(k, CFG.params[k]));
    for(const k of ["target","maxSpeed","speedScale"]) els.settingsCard.appendChild(buildParamRow(k, CFG.params[k]));
    $("#ipHint").textContent = location.pathname || "/";
  }
  function saveLocal(){ localStorage.setItem(CFG.storageKey, JSON.stringify({ values: state })); setHttpState("ok", "SAVED"); }
  function loadLocal(){ try{ const raw = localStorage.getItem(CFG.storageKey); if(raw) applyValues(JSON.parse(raw).values, false); }catch(e){} }
  function applyValues(vals, sendIt){ for(const k in vals){ if(!CFG.params[k])continue; const v = vals[k]; state[k]=v; $(`#num_${k}`).value=v; $(`#rng_${k}`).value=v; if(sendIt) send(CFG.params[k].cmd, v); } }
  function wireActions(){
    $("#btnCalib").onclick = () => send("c", "0"); $("#btnDebug").onclick = () => send("?", "0");
    $("#btnSave").onclick = saveLocal; $("#btnLoad").onclick = loadLocal;
    $("#presetSoft").onclick = () => applyValues(CFG.presets.soft.values, true);
    $("#presetNormal").onclick = () => applyValues(CFG.presets.normal.values, true);
    $("#presetAggressive").onclick = () => applyValues(CFG.presets.aggressive.values, true);
  }
  render(); wireActions(); loadLocal();
})();
</script>
</body>
</html>
)rawliteral";

// ================= XỬ LÝ LỆNH TỪ WEB =================
void handleRoot() {
  server.send(200, "text/html", index_html);
}

void handleSet() {
  if (server.hasArg("c") && server.hasArg("v")) {
    String cmd = server.arg("c");
    String val = server.arg("v");
    
    // Gửi lệnh xuống ESP Chính qua UART2 (Chân 17 TX)
    Serial2.print(cmd);
    Serial2.println(val);
    
    Serial.print("Web Sent: "); Serial.print(cmd); Serial.println(val);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "BAD REQ");
  }
}

// ================= SETUP & LOOP =================
void setup() {
  Serial.begin(115200);
  
  // Khởi động UART2 để nói chuyện với Robot
  // TX=17, RX=16 (Nối chéo với Robot)
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  // Tạo Wifi
  WiFi.softAP(ssid, password);
  Serial.print("IP Web: ");
  Serial.println(WiFi.softAPIP());

  // Web Server
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.begin();
}

void loop() {
  server.handleClient();
  
  // Nếu Robot gửi gì lên (VD: Debug info), in ra Serial máy tính để xem
  while (Serial2.available()) {
    Serial.write(Serial2.read());
  }
}
