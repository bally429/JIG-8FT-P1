/* * 模組角色：ESP32 (ESPWM32EN16) 掃描器橋接程式與 Wi-Fi/Web 整合
 * 通訊協定：JIG_8CP (ASCII Protocol) [STX][Cmd][Data][Checksum][CR]
 * ===========================================================================================
 * Project: JIG-8FT-P1 _WIFIBLE (ESP32 控制端)
 * MCU: ESP32-WROOM-32E (ESPWM32EN16)
 * * [版本更新紀錄]
 * -------------------------------------------------------------------------------------------
 * 版本    日期        更新說明
 * -------------------------------------------------------------------------------------------
 * V1.0    2026/06/29 專案正式建立。
 * V1.1    2026/07/07 擴充 SD卡、Wi-Fi 狀態燈、Captive Portal Web 配網、UART WI? IP 查詢。
 * V1.2    2026/07/07 新增 5 組 Wi-Fi 帳密儲存與「優先級自動切換」備援連線機制。
 * V1.3    2026/07/07 導入 RSSI 掃描排序機制，智慧優先連線環境中「訊號最強」的已知網路。
 * V1.4    2026/07/07 [全新升級] 1. 新增入口首頁選單。 2. 優化 Wi-Fi 設定為單一 User + 5組 SSID。
 * 3. 新增「電壓電流偵測」即時儀表板 (純原生 Canvas 無網可運作) 與 PW/PD 雙向通訊。
 * -------------------------------------------------------------------------------------------
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <SPI.h>
#include <SD.h>

// --- 定義通訊協定字元 ---
#define JIG_8CP_STX 0x02
#define JIG_8CP_CR  0x0D

// --- 腳位定義 ---
#define SCANNER_RX_PIN 16
#define SCANNER_TX_PIN 17
#define WIFI_LED_PIN   4
#define SD_MOSI_PIN    23
#define SD_MISO_PIN    19
#define SD_SCK_PIN     18
#define SD_CS_PIN      5

// --- 全域變數：系統與網路 ---
String globalUser = ""; // 單一 User
struct WiFiConfig {
  String ssid;
  String pass;
};
WiFiConfig wifiList[5];

struct MatchedWiFi {
  String ssid;
  String pass;
  int32_t rssi;
};

bool isAPMode = false;
const byte DNS_PORT = 53;
DNSServer dnsServer;
WebServer server(80);
SPIClass spiSD(VSPI);

// --- LED 狀態機變數 ---
enum LedState { LED_OFF, LED_BLINK_CONNECTING, LED_BLINK_FAIL, LED_ON };
LedState currentLedState = LED_OFF;
unsigned long previousLedMillis = 0;
int failBlinkCount = 0;
bool ledStateHigh = false;

// --- 全域變數：電壓電流即時數據 ---
float current_mA = 0.0;
float current_V = 0.0;
String power_state = "OFF";

// --- 函式宣告 ---
void setupSDCard();
void loadWiFiConfig();
void saveWiFiConfig();
void setupWiFi();
void startAPMode();
void handleRoot();
void handleWiFiSet();
void handleSaveWiFi();
void handleMonitor();
void handleApiData();
void handleApiPower();
void handleNotFound();
void updateLED();
void setLedState(LedState state);
void sendToM031_JIG_8CP(String cmd, String data);
void processM031Command(String packet);
void readHostUART();

void setup() {
  Serial.begin(115200); 
  Serial2.begin(115200, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);

  // 1. LED 初始化
  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, LOW);

  // 2. 初始值
  globalUser = "";
  for (int i = 0; i < 5; i++) {
    wifiList[i].ssid = ""; wifiList[i].pass = "";
  }

  // 3. SD 卡初始化與讀取設定
  setupSDCard();
  loadWiFiConfig();

  // 4. 智慧型 Wi-Fi 選網與連線
  setupWiFi();

  // 5. 設定 Web Server 路由
  server.on("/", handleRoot);
  server.on("/wifi", handleWiFiSet);
  server.on("/save", HTTP_POST, handleSaveWiFi);
  server.on("/monitor", handleMonitor);
  server.on("/api/data", handleApiData);     // 前端 AJAX 抓取 PD 數據
  server.on("/api/power", handleApiPower);   // 前端 AJAX 控制 PW 開關
  server.onNotFound(handleNotFound); 
  server.begin();
}

void loop() {
  if (isAPMode) dnsServer.processNextRequest();
  server.handleClient();
  updateLED();

  // 檢查掃描機是否有刷入條碼
  if (Serial2.available()) {
    String barcodeData = Serial2.readStringUntil('\n'); 
    barcodeData.trim();
    if (barcodeData.length() > 0) {
      sendToM031_JIG_8CP("SC", barcodeData);
    }
  }

  // 檢查 M031 UART
  readHostUART();
}

// ==========================================
// [LED 燈號控制]
// ==========================================
void setLedState(LedState state) {
  currentLedState = state; failBlinkCount = 0; ledStateHigh = false;
  digitalWrite(WIFI_LED_PIN, LOW); previousLedMillis = millis();
}

void updateLED() {
  unsigned long currentMillis = millis();
  if (currentLedState == LED_ON) digitalWrite(WIFI_LED_PIN, HIGH);
  else if (currentLedState == LED_OFF) digitalWrite(WIFI_LED_PIN, LOW);
  else if (currentLedState == LED_BLINK_CONNECTING) {
    if (currentMillis - previousLedMillis >= 500) {
      previousLedMillis = currentMillis; ledStateHigh = !ledStateHigh;
      digitalWrite(WIFI_LED_PIN, ledStateHigh ? HIGH : LOW);
    }
  } else if (currentLedState == LED_BLINK_FAIL) {
    if (failBlinkCount < 8) {
      if (currentMillis - previousLedMillis >= 100) {
        previousLedMillis = currentMillis; ledStateHigh = !ledStateHigh;
        digitalWrite(WIFI_LED_PIN, ledStateHigh ? HIGH : LOW); failBlinkCount++;
      }
    } else {
      digitalWrite(WIFI_LED_PIN, LOW); currentLedState = LED_OFF; 
    }
  }
}

// ==========================================
// [SD 卡與設定檔處理]
// ==========================================
void setupSDCard() {
  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spiSD)) return;
  if (!SD.exists("/Users")) SD.mkdir("/Users");

  if (!SD.exists("/Users/WiFi_begin.txt")) {
    File file = SD.open("/Users/WiFi_begin.txt", FILE_WRITE);
    if (file) {
      file.printf("User = \"\"\n");
      for (int i = 0; i < 5; i++) {
        file.printf("ssid%d = \"\"\n", i + 1);
        file.printf("password%d = \"\"\n", i + 1);
      }
      file.close();
    }
  }
}

String extractQuote(String line) {
  int firstQuote = line.indexOf('"');
  int lastQuote = line.lastIndexOf('"');
  if (firstQuote >= 0 && lastQuote > firstQuote) return line.substring(firstQuote + 1, lastQuote);
  return "";
}

void loadWiFiConfig() {
  if (SD.exists("/Users/WiFi_begin.txt")) {
    File file = SD.open("/Users/WiFi_begin.txt", FILE_READ);
    while (file.available()) {
      String line = file.readStringUntil('\n'); line.trim();
      if (line.startsWith("User =")) globalUser = extractQuote(line);
      for (int i = 0; i < 5; i++) {
        String s_tag = "ssid" + String(i + 1);
        String p_tag = "password" + String(i + 1);
        if (line.startsWith(s_tag)) wifiList[i].ssid = extractQuote(line);
        else if (line.startsWith(p_tag)) wifiList[i].pass = extractQuote(line);
      }
    }
    file.close();
  }
}

void saveWiFiConfig() {
  File file = SD.open("/Users/WiFi_begin.txt", FILE_WRITE);
  if (file) {
    file.print("User = \""); file.print(globalUser); file.println("\"");
    for (int i = 0; i < 5; i++) {
      file.print("ssid"); file.print(i + 1); file.print(" = \""); file.print(wifiList[i].ssid); file.println("\"");
      file.print("password"); file.print(i + 1); file.print(" = \""); file.print(wifiList[i].pass); file.println("\"");
    }
    file.close();
  }
}

// ==========================================
// [智慧選網備援機制]
// ==========================================
void setupWiFi() {
  setLedState(LED_BLINK_CONNECTING);
  WiFi.mode(WIFI_STA); WiFi.disconnect(); delay(100);
  int n = WiFi.scanNetworks();
  MatchedWiFi matches[5]; int matchCount = 0; bool anyConnected = false;

  if (n > 0) {
    for (int i = 0; i < n; ++i) {
      String scannedSSID = WiFi.SSID(i); int32_t scannedRSSI = WiFi.RSSI(i);
      for (int j = 0; j < 5; j++) {
        if (wifiList[j].ssid != "" && wifiList[j].ssid == scannedSSID) {
          bool alreadyAdded = false;
          for(int k = 0; k < matchCount; k++) {
            if(matches[k].ssid == scannedSSID) {
              alreadyAdded = true;
              if(scannedRSSI > matches[k].rssi) matches[k].rssi = scannedRSSI;
              break;
            }
          }
          if(!alreadyAdded && matchCount < 5) {
            matches[matchCount].ssid = wifiList[j].ssid; matches[matchCount].pass = wifiList[j].pass; matches[matchCount].rssi = scannedRSSI; matchCount++;
          }
        }
      }
    }
  }

  for (int i = 0; i < matchCount - 1; i++) {
    for (int j = i + 1; j < matchCount; j++) {
      if (matches[j].rssi > matches[i].rssi) {
        MatchedWiFi temp = matches[i]; matches[i] = matches[j]; matches[j] = temp;
      }
    }
  }

  for (int i = 0; i < matchCount; i++) {
    WiFi.begin(matches[i].ssid.c_str(), matches[i].pass.c_str());
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); updateLED(); attempts++; }
    if (WiFi.status() == WL_CONNECTED) { anyConnected = true; break; }
  }

  if (!anyConnected) {
    for (int i = 0; i < 5; i++) {
      if (wifiList[i].ssid == "") continue;
      WiFi.begin(wifiList[i].ssid.c_str(), wifiList[i].pass.c_str());
      int attempts = 0;
      while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); updateLED(); attempts++; }
      if (WiFi.status() == WL_CONNECTED) { anyConnected = true; break; }
    }
  }

  if (anyConnected) { setLedState(LED_ON); isAPMode = false; } 
  else { setLedState(LED_BLINK_FAIL); startAPMode(); }
}

void startAPMode() {
  isAPMode = true; WiFi.mode(WIFI_AP);
  WiFi.softAP("JIG_8FT_P1_WIFIset");
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
}

// ==========================================
// [Web Server 頁面路由]
// ==========================================

// 1. 首頁 (Portal)
void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
  <title>JIG-8FT-P1 總控制台</title>
  <style>
    body{font-family: Arial, sans-serif; text-align: center; padding: 40px; background-color: #f4f4f9;}
    .btn {display: block; width: 80%; max-width: 300px; margin: 20px auto; padding: 20px; font-size: 20px; font-weight: bold; color: white; background-color: #0056b3; border: none; border-radius: 10px; cursor: pointer; text-decoration: none; box-shadow: 0 4px 6px rgba(0,0,0,0.1);}
    .btn:hover {background-color: #004494;}
    .btn.alt {background-color: #28a745;}
    .btn.alt:hover {background-color: #218838;}
  </style></head><body>
  <h2>⚙️ JIG-8FT-P1 控制面板</h2>
  <a href='/wifi' class='btn'>🌐 網路備援設定</a>
  <a href='/monitor' class='btn alt'>⚡ 電壓電流偵測</a>
  </body></html>
  )rawliteral";
  server.send(200, "text/html", html);
}

// 2. 網路設定頁面
void handleWiFiSet() {
  String html = "<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>網路備援設定</title>";
  html += "<style>body{font-family:Arial; padding:20px; background:#f4f4f4;} .card{background:#fff; padding:15px; margin-bottom:15px; border-radius:8px; box-shadow:0 2px 4px rgba(0,0,0,0.1);} .back-btn{display:inline-block; margin-bottom:15px; text-decoration:none; color:#0056b3; font-weight:bold;}</style>";
  html += "</head><body><a href='/' class='back-btn'>⬅ 返回首頁</a><h2>網路備援設定 (系統將自動挑選最強訊號)</h2><form action='/save' method='POST'>";

  html += "<div class='card'><h3>全域使用者 (User)</h3><input type='text' name='globalUser' value='" + globalUser + "' style='width:100%; padding:8px;'></div>";

  for (int i = 0; i < 5; i++) {
    html += "<div class='card'><h3>備援組 " + String(i + 1) + "</h3>";
    html += "網路名稱 (SSID):<br><input type='text' name='ssid" + String(i + 1) + "' value='" + wifiList[i].ssid + "' style='width:100%; padding:8px; margin-bottom:10px;'><br>";
    html += "密碼 (Password):<br><input type='text' name='pass" + String(i + 1) + "' value='" + wifiList[i].pass + "' style='width:100%; padding:8px;'><br></div>";
  }

  html += "<input type='submit' value='💾 儲存並重啟設備' style='width:100%; padding:15px; font-size:18px; background:#4CAF50; color:white; border:none; border-radius:8px; cursor:pointer;'>";
  html += "</form></body></html>";
  server.send(200, "text/html", html);
}

void handleSaveWiFi() {
  if (server.hasArg("globalUser")) globalUser = server.arg("globalUser");
  for (int i = 0; i < 5; i++) {
    String s_tag = "ssid" + String(i + 1); String p_tag = "pass" + String(i + 1);
    if (server.hasArg(s_tag)) wifiList[i].ssid = server.arg(s_tag);
    if (server.hasArg(p_tag)) wifiList[i].pass = server.arg(p_tag);
  }
  saveWiFiConfig();
  server.send(200, "text/html", "<meta charset='UTF-8'><h2>✅ 設定已儲存！ESP32 將重新啟動...</h2>");
  delay(1000); ESP.restart();
}

// 3. 電壓電流偵測頁面 (純淨版 JS/HTML)
void handleMonitor() {
  // 進入此頁面時，主動發送指令向 M031 詢問電源狀態
  sendToM031_JIG_8CP("PW", "?");

  String html = R"rawliteral(
  <!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
  <title>電壓電流即時分析儀</title>
  <style>
    body{font-family: Arial, sans-serif; background: #222; color: #fff; margin: 0; padding: 10px; text-align: center;}
    .back-btn{display: block; text-align: left; color: #00c3ff; text-decoration: none; margin-bottom: 10px; font-weight: bold;}
    .dashboard {display: flex; flex-wrap: wrap; justify-content: center; gap: 20px;}
    .panel {background: #333; border-radius: 10px; padding: 15px; width: 100%; max-width: 400px;}
    .val-text {font-size: 24px; font-weight: bold; color: #0f0;}
    .controls {margin-top: 15px; display: flex; justify-content: space-around; align-items: center;}
    .controls div {display: flex; flex-direction: column; font-size:14px;}
    input {width: 60px; padding: 5px; text-align: center; border-radius:5px; border:none;}
    button.power {padding: 10px 20px; font-size: 16px; font-weight: bold; border: none; border-radius: 5px; cursor: pointer; color:#fff;}
    .power-off {background: #d9534f;} .power-on {background: #28a745;}
    canvas {background: #111; border-radius: 5px; margin-top:10px; width: 100%; height: 180px;}
    /* 簡易 CSS 指針儀表 */
    .gauge-container {position: relative; width: 200px; height: 100px; overflow: hidden; margin: 0 auto;}
    .gauge-bg {position: absolute; width: 200px; height: 200px; border-radius: 50%; border: 15px solid #444; border-top-color: #00c3ff; border-right-color: #00c3ff; box-sizing: border-box; transform: rotate(-45deg); top: 0; left: 0;}
    .gauge-needle {position: absolute; bottom: 0; left: 50%; width: 4px; height: 80px; background: red; transform-origin: bottom center; transform: rotate(-90deg); transition: transform 0.2s ease-out;}
    .gauge-center {position: absolute; bottom: -10px; left: 50%; width: 20px; height: 20px; background: #fff; border-radius: 50%; transform: translateX(-50%);}
  </style>
  </head><body>
  <a href='/' class='back-btn'>⬅ 返回首頁</a>
  <h2>⚡ 設備能耗即時分析儀</h2>
  <div class="dashboard">
    <div class="panel">
      <h3>即時電流 (mA)</h3>
      <div class="gauge-container">
        <div class="gauge-bg"></div>
        <div class="gauge-needle" id="needle"></div>
        <div class="gauge-center"></div>
      </div>
      <div class="val-text" id="curText">0.0 mA</div>
      <div style="color:#ffcc00; margin-top:5px; font-size:18px;">電壓: <span id="volText">0.00 V</span></div>
      
      <div class="controls">
        <div>
           <label>最大刻度</label>
           <input type="number" id="maxScale" value="500">
        </div>
        <div>
           <label>Limit 警報</label>
           <input type="number" id="limitScale" value="400">
        </div>
        <button id="pwrBtn" class="power power-off" onclick="togglePower()">電源 OFF</button>
      </div>
    </div>
    
    <div class="panel">
      <h3>動態折線圖 (每 250ms)</h3>
      <canvas id="lineChart"></canvas>
    </div>
  </div>

  <script>
    const cvs = document.getElementById('lineChart');
    const ctx = cvs.getContext('2d');
    let historyData = [];
    const maxPoints = 50;

    // 定期抓取 ESP32 數據
    setInterval(() => {
      fetch('/api/data').then(res => res.json()).then(data => {
        updateUI(data);
      });
    }, 250);

    function updateUI(data) {
      // 1. 更新數值
      document.getElementById('curText').innerText = data.mA.toFixed(1) + ' mA';
      document.getElementById('volText').innerText = data.v.toFixed(2) + ' V';
      
      // 2. 更新電源按鈕
      const btn = document.getElementById('pwrBtn');
      if(data.power === "ON") {
        btn.className = "power power-on"; btn.innerText = "電源 ON";
      } else {
        btn.className = "power power-off"; btn.innerText = "電源 OFF";
      }

      // 3. 更新指針 (-90deg 到 90deg)
      const maxScale = parseFloat(document.getElementById('maxScale').value) || 500;
      let ratio = data.mA / maxScale;
      if(ratio > 1) ratio = 1;
      const angle = -90 + (ratio * 180);
      document.getElementById('needle').style.transform = `rotate(${angle}deg)`;

      // 4. 更新折線圖
      historyData.push(data.mA);
      if(historyData.length > maxPoints) historyData.shift();
      drawChart();
    }

    function drawChart() {
      // 修正 Canvas 繪圖解析度
      cvs.width = cvs.clientWidth; cvs.height = cvs.clientHeight;
      const w = cvs.width, h = cvs.height;
      ctx.clearRect(0, 0, w, h);
      
      const maxScale = parseFloat(document.getElementById('maxScale').value) || 500;
      const limitVal = parseFloat(document.getElementById('limitScale').value) || 400;

      // 畫 Limit 虛線
      const limitY = h - (limitVal / maxScale) * h;
      ctx.beginPath();
      ctx.setLineDash([5, 5]);
      ctx.moveTo(0, limitY); ctx.lineTo(w, limitY);
      ctx.strokeStyle = 'rgba(255,0,0,0.6)'; ctx.stroke();
      ctx.setLineDash([]);

      // 畫折線
      ctx.beginPath();
      ctx.strokeStyle = '#0f0';
      ctx.lineWidth = 2;
      const xStep = w / (maxPoints - 1);
      
      for(let i=0; i<historyData.length; i++) {
        let x = i * xStep;
        let y = h - (historyData[i] / maxScale) * h;
        if(i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    // 發送電源控制命令
    function togglePower() {
      const btn = document.getElementById('pwrBtn');
      const newState = btn.innerText.includes("OFF") ? "ON" : "OFF";
      fetch('/api/power?state=' + newState, {method: 'POST'});
    }
  </script>
  </body></html>
  )rawliteral";
  server.send(200, "text/html", html);
}

// 4. 前端 AJAX 取得數據接口
void handleApiData() {
  String json = "{";
  json += "\"mA\":" + String(current_mA) + ",";
  json += "\"v\":" + String(current_V) + ",";
  json += "\"power\":\"" + power_state + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// 5. 前端 AJAX 控制電源接口
void handleApiPower() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "ON") {
      sendToM031_JIG_8CP("PW", "PA8ON");
      power_state = "ON"; // 先暫時設定，等待 M031 真正回傳覆寫
    } else {
      sendToM031_JIG_8CP("PW", "PA8OFF");
      power_state = "OFF";
    }
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void handleNotFound() {
  if (isAPMode) {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
    server.send(302, "text/plain", "");
  } else {
    server.send(404, "text/plain", "Not Found");
  }
}

// ==========================================
// [UART 雙向通訊機制 (JIG_8CP)]
// ==========================================
void sendToM031_JIG_8CP(String cmd, String data) {
  String payload = cmd + data; 
  unsigned int sum = 0;
  for (int i = 0; i < payload.length(); i++) { sum += payload[i]; }
  byte checksum = sum & 0xFF; 
  char hexSum[3]; sprintf(hexSum, "%02X", checksum);
  
  Serial.write(JIG_8CP_STX); 
  Serial.print(payload);     
  Serial.print(hexSum);      
  Serial.write(JIG_8CP_CR);  
}

void readHostUART() {
  static String rxBuffer = "";
  static bool isReceiving = false;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == JIG_8CP_STX) { rxBuffer = ""; isReceiving = true; } 
    else if (c == JIG_8CP_CR && isReceiving) {
      isReceiving = false;
      processM031Command(rxBuffer);
    } 
    else if (isReceiving) { rxBuffer += c; }
  }
}

void processM031Command(String packet) {
  if (packet.length() < 4) return; 

  String cmdData = packet.substring(0, packet.length() - 2);
  String receivedChk = packet.substring(packet.length() - 2);

  unsigned int sum = 0;
  for (int i = 0; i < cmdData.length(); i++) { sum += cmdData[i]; }
  char calcChk[3]; sprintf(calcChk, "%02X", sum & 0xFF);
  if (receivedChk != String(calcChk)) return; 

  String cmd = cmdData.substring(0, 2);
  String data = cmdData.substring(2);

  // 1. IP 查詢
  if (cmd == "WI" && data == "?") {
    String currentIP = isAPMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    sendToM031_JIG_8CP("WI", currentIP); 
  }
  // 2. 解析 PD (Power Data) -> 格式: PD125.5,5.01
  else if (cmd == "PD") {
    int commaIdx = data.indexOf(',');
    if(commaIdx > 0) {
      current_mA = data.substring(0, commaIdx).toFloat();
      current_V = data.substring(commaIdx + 1).toFloat();
    }
  }
  // 3. 解析 PW (Power 狀態同步) -> 預期收到 PA8ON 或 PA8OFF
  else if (cmd == "PW") {
    if(data.indexOf("ON") >= 0) power_state = "ON";
    else if(data.indexOf("OFF") >= 0) power_state = "OFF";
  }
}