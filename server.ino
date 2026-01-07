/*
 * ESP32 AUXILIARY - VIETNAMESE CONTROLLER V8.0
 * - PID Range: 0 - 200
 * - Sync Defaults: Khớp với thông số mặc định của xe
 * - Removed: Auto Raise
 */

#include <WiFi.h>
#include <WebServer.h>

// ================= CẤU HÌNH =================
#define RX_PIN 16 
#define TX_PIN 17 
#define BAUD_RATE 115200

const char* ssid = "BROBOT_V8"; // Tên Wifi mới
const char* password = "12345678";

WebServer server(80);
HardwareSerial LinkSerial(2); 

String telemetryData = "{}"; 

// HTML GIAO DIỆN
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>BROBOT V8 CONTROL</title>
  <style>
    :root { --bg: #0f0f0f; --panel: #1a1a1a; --text: #eee; --accent: #00e676; --blue: #2979ff; --red: #ff1744; --orange: #ff9100; }
    body { font-family: sans-serif; background: var(--bg); color: var(--text); margin: 0; padding: 10px; padding-bottom: 100px; user-select: none; }
    
    .hud { background: #000; border: 2px solid #333; border-radius: 12px; height: 100px; position: relative; overflow: hidden; margin-bottom: 15px; }
    .horizon { width: 150%; height: 2px; background: var(--accent); position: absolute; top: 50%; left: -25%; transition: transform 0.1s; box-shadow: 0 0 10px var(--accent); }
    .val-display { position: absolute; top: 10px; left: 10px; font-family: monospace; font-size: 1.2em; color: var(--blue); font-weight: bold; }

    .tabs { display: flex; margin-bottom: 15px; background: #000; border-radius: 8px; padding: 5px; }
    .tab { flex: 1; padding: 12px; text-align: center; cursor: pointer; border-radius: 6px; font-weight: bold; color: #666; transition: 0.3s; }
    .tab.active { background: var(--panel); color: var(--accent); box-shadow: 0 2px 5px rgba(0,0,0,0.3); }
    .tab-content { display: none; }
    .tab-content.active { display: block; animation: fadeIn 0.3s; }
    @keyframes fadeIn { from { opacity: 0; } to { opacity: 1; } }

    .section { background: var(--panel); padding: 15px; border-radius: 12px; margin-bottom: 12px; border-left: 5px solid var(--blue); }
    h3 { margin: 0 0 15px 0; color: #888; font-size: 0.9em; text-transform: uppercase; letter-spacing: 1px; }

    .slider-container { display: flex; align-items: center; margin-bottom: 20px; }
    .slider-label { flex: 1; font-weight: bold; font-size: 0.85em; }
    .slider-val { width: 50px; text-align: right; color: var(--accent); font-family: monospace; font-weight: bold; }
    input[type=range] { flex: 2; margin: 0 10px; height: 30px; -webkit-appearance: none; background: transparent; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; height: 22px; width: 22px; border-radius: 50%; background: var(--accent); margin-top: -9px; border: 2px solid #fff; }
    input[type=range]::-webkit-slider-runnable-track { width: 100%; height: 4px; background: #444; border-radius: 2px; }

    .btn-stop { width: 100%; padding: 15px; background: var(--red); color: white; border: none; border-radius: 8px; font-weight: bold; font-size: 1.2em; margin-top: 10px; }
    .btn-calib { width: 100%; padding: 12px; background: #333; color: #ccc; border: none; border-radius: 8px; margin-top: 5px; font-weight: bold; }

    .joystick-wrapper { display: flex; justify-content: center; padding: 20px; }
    .d-pad { display: grid; grid-template-columns: 1fr 1fr 1fr; grid-template-rows: 1fr 1fr 1fr; width: 240px; height: 240px; background: #2a2a2a; border-radius: 50%; gap: 8px; padding: 10px; box-shadow: 0 10px 20px rgba(0,0,0,0.5); border: 4px solid #333; }
    .btn-joy { background: #383838; border: none; color: white; font-size: 1.5em; border-radius: 10px; display: flex; justify-content: center; align-items: center; transition: all 0.1s; cursor: pointer; }
    .btn-joy:active { background: var(--accent); color: #000; transform: scale(0.95); }
    .up { grid-column: 2; grid-row: 1; border-radius: 20px 20px 5px 5px; }
    .left { grid-column: 1; grid-row: 2; border-radius: 20px 5px 5px 20px; }
    .right { grid-column: 3; grid-row: 2; border-radius: 5px 20px 20px 5px; }
    .down { grid-column: 2; grid-row: 3; border-radius: 5px 5px 20px 20px; }
    .center { grid-column: 2; grid-row: 2; background: #151515; border-radius: 50%; font-size: 0.8em; border: 1px solid #444;}
  </style>
</head>
<body>

  <div class="hud">
    <div class="val-display">ANG: <span id="disp_ang">0.0</span> | KP: <span id="disp_kp">0</span></div>
    <div class="horizon" id="horizon"></div>
  </div>

  <div class="tabs">
    <div class="tab active" onclick="setTab(0)">CẤU HÌNH</div>
    <div class="tab" onclick="setTab(1)">LÁI XE</div>
  </div>

  <div id="tab0" class="tab-content active">
    
    <div class="section">
      <h3>1. Cân Bằng (Stability)</h3>
      <div class="slider-container">
        <div class="slider-label">Kp (Độ cứng)</div>
        <input type="range" id="s_p" min="0" max="200" step="0.5" value="35.0" oninput="u('p',this.value)">
        <div class="slider-val" id="v_p">35.0</div>
      </div>
      <div class="slider-container">
        <div class="slider-label">Kd (Giảm rung)</div>
        <input type="range" id="s_d" min="0" max="100" step="0.1" value="0.6" oninput="u('d',this.value)">
        <div class="slider-val" id="v_d">0.6</div>
      </div>
      <div class="slider-container">
        <div class="slider-label">Ki (Hồi vị)</div>
        <input type="range" id="s_i" min="0" max="200" step="0.1" value="0.0" oninput="u('i',this.value)">
        <div class="slider-val" id="v_i">0.0</div>
      </div>
    </div>

    <div class="section" style="border-left-color: var(--orange)">
      <h3>2. Chống Trôi (Anti-Drift)</h3>
      <div class="slider-container">
        <div class="slider-label">Kpos (Vị trí)</div>
        <input type="range" id="s_x" min="0" max="0.5" step="0.001" value="0.0" oninput="u('x',this.value)">
        <div class="slider-val" id="v_x">0.0</div>
      </div>
      <div class="slider-container">
        <div class="slider-label">Kvel (Hãm tốc)</div>
        <input type="range" id="s_y" min="0" max="5.0" step="0.01" value="0.0" oninput="u('y',this.value)">
        <div class="slider-val" id="v_y">0.0</div>
      </div>
    </div>

    <div class="section" style="border-left-color: #fbbf24">
      <h3>3. Vật Lý & Tốc Độ</h3>
      <div class="slider-container">
        <div class="slider-label">Góc Target</div>
        <input type="range" id="s_t" min="170" max="190" step="0.1" value="180.0" oninput="u('t',this.value)">
        <div class="slider-val" id="v_t">180.0</div>
      </div>
      <div class="slider-container">
        <div class="slider-label">Độ nhạy (Scale)</div>
        <input type="range" id="s_s" min="1" max="50" step="0.5" value="12.0" oninput="u('s',this.value)">
        <div class="slider-val" id="v_s">12.0</div>
      </div>
      <button class="btn-calib" onclick="send('c',1)">CALIB CẢM BIẾN (ĐIỂM 0)</button>
    </div>
  </div>

  <div id="tab1" class="tab-content">
    <div class="section" style="border:none; background: transparent; padding: 0;">
      
      <div class="joystick-wrapper">
        <div class="d-pad">
          <button class="btn-joy up" ontouchstart="drive('M', 200)" ontouchend="drive('M', 0)" onmousedown="drive('M', 200)" onmouseup="drive('M', 0)">▲</button>
          <button class="btn-joy left" ontouchstart="drive('T', -80)" ontouchend="drive('T', 0)" onmousedown="drive('T', -80)" onmouseup="drive('T', 0)">◄</button>
          <div class="btn-joy center">BROBOT</div>
          <button class="btn-joy right" ontouchstart="drive('T', 80)" ontouchend="drive('T', 0)" onmousedown="drive('T', 80)" onmouseup="drive('T', 0)">►</button>
          <button class="btn-joy down" ontouchstart="drive('M', -200)" ontouchend="drive('M', 0)" onmousedown="drive('M', -200)" onmouseup="drive('M', 0)">▼</button>
        </div>
      </div>
      
      <button class="btn-stop" onclick="send('p',0);send('i',0)">DỪNG KHẨN CẤP (PID=0)</button>
    </div>
  </div>

  <script>
    function setTab(idx) {
      document.querySelectorAll('.tab').forEach((t, i) => t.classList.toggle('active', i===idx));
      document.querySelectorAll('.tab-content').forEach((c, i) => c.classList.toggle('active', i===idx));
    }

    let lastSend = 0;
    let pendingCmd = null;

    function u(key, val) {
      document.getElementById('v_'+key).innerText = val; 
      const now = Date.now();
      if (now - lastSend > 100) { 
        send(key, val);
        lastSend = now;
      } else {
        clearTimeout(pendingCmd);
        pendingCmd = setTimeout(() => { send(key, val); }, 110);
      }
    }

    function send(key, val) { fetch('/set?c=' + key + '&v=' + val).catch(e=>{}); }
    function drive(cmd, val) { fetch('/set?c=' + cmd + '&v=' + val).catch(e=>{}); }

    setInterval(async () => {
      try {
        const res = await fetch('/tele');
        const json = await res.json();
        if(json.ang !== undefined) {
          document.getElementById('disp_ang').innerText = json.ang;
          document.getElementById('disp_kp').innerText = json.kp;
          let rot = (json.ang - 180) * 3; 
          document.getElementById('horizon').style.transform = `translate(0, -50%) rotate(${rot}deg)`;
        }
      } catch(e) {}
    }, 250);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", INDEX_HTML); }
void handleSet() {
  if (server.hasArg("c") && server.hasArg("v")) {
    LinkSerial.print(server.arg("c")); LinkSerial.print(server.arg("v")); LinkSerial.print('\n'); 
    server.send(200, "text/plain", "OK");
  } else { server.send(400, "text/plain", "Error"); }
}
void handleTele() { server.send(200, "application/json", telemetryData); }

void setup() {
  Serial.begin(115200);
  LinkSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/tele", handleTele);
  server.begin();
}

void loop() {
  server.handleClient();
  if (LinkSerial.available()) {
    String line = LinkSerial.readStringUntil('\n');
    if (line.indexOf("Angle:") >= 0) {
      float ang = extractVal(line, "Angle:");
      float kp = extractVal(line, "Kp:");
      telemetryData = "{";
      telemetryData += "\"ang\":" + String(ang) + ",";
      telemetryData += "\"kp\":" + String(kp);
      telemetryData += "}";
    }
  }
}

float extractVal(String str, String label) {
  int idx = str.indexOf(label);
  if (idx == -1) return 0;
  int start = idx + label.length();
  int end = str.indexOf(' ', start);
  if (end == -1) end = str.length();
  return str.substring(start, end).toFloat();
}
