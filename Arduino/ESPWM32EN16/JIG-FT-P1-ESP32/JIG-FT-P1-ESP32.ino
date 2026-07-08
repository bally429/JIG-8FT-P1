/* * 模組角色：ESP32 (ESPWM32EN16) 掃描器橋接程式與 Wi-Fi/Web 整合
 * 通訊協定：JIG_8CP (ASCII Protocol) [STX][Cmd][Data][Checksum][CR]
 * ===========================================================================================
 * Project: JIG-8FT-P1 _WIFIBLE (ESP32 控制端)
 * MCU: ESP32-WROOM-32E (ESPWM32EN16)
 * * [版本更新紀錄]
 * -------------------------------------------------------------------------------------------
 * V1.6.1  2026/07/08 [Bug 修復版] 修正 WebSocket UI 狀態卡死問題與安全熔斷盲區。
 * V1.6.2  2026/07/08 [穩定性升級] 
 * 1. 導入「硬體啟動寬限期 (Grace Period)」：通電前 2 秒暫停極限審查，根除開機瞬間電容充電造成的 0V 誤判熔斷。
 * 2. 修正 CSV 中文亂碼：在檔案開頭寫入 UTF-8 BOM，讓 Microsoft Excel 正確解析。
 * 3. 確保實體按鍵 (M031) 開機時，ESP32 也會同步給予寬限期保護。
 * -------------------------------------------------------------------------------------------
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <SPI.h>
#include <SD.h>
#include <WebSocketsServer.h> 

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
String globalUser = ""; 
struct WiFiConfig { String ssid; String pass; };
WiFiConfig wifiList[5];
struct MatchedWiFi { String ssid; String pass; int32_t rssi; };

bool isAPMode = false;
const byte DNS_PORT = 53;
DNSServer dnsServer;
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81); 
SPIClass spiSD(VSPI);

// --- LED 狀態機變數 ---
enum LedState { LED_OFF, LED_BLINK_CONNECTING, LED_BLINK_FAIL, LED_ON };
LedState currentLedState = LED_OFF;
unsigned long previousLedMillis = 0;
int failBlinkCount = 0;
bool ledStateHigh = false;

// --- 全域變數：電壓電流即時數據與安全限制 ---
float current_mA = 0.0;
float current_V = 0.0;
String power_state = "OFF";

float conf_maxScale = 500.0;
float conf_limitScale = 400.0;
float conf_minVoltage = 0.0;
float conf_maxVoltage = 12.0;
int conf_limitDuration = 3;  

float cumulative_mAh = 0.0;
float cumulative_mWh = 0.0;
bool isRecordingCSV = false;
String logFilename = "";
File csvFile;

unsigned long lastPdTime = 0;          
unsigned long overCurrentStartTime = 0; 
unsigned long powerOnTime = 0; // [新增] 電源開啟時間，用於硬體啟動寬限期
String systemWarning = "";             

// --- 函式宣告 ---
void setupSDCard();
String extractQuote(String line);
void loadWiFiConfig();
void saveWiFiConfig();
void loadPowerConfig();
void savePowerConfig();
void setLedState(LedState state);
void updateLED();
void setupWiFi();
void startAPMode();
void checkSafetyLimits();
void broadcastWebSocket();
void handleRoot();
void handleWiFiSet();
void handleSaveWiFi();
void handleMonitor();
void handleApiData();
void handleApiPower();
void handleApiSaveConfig();
void handleDownloadCSV();
void handleNotFound();
void sendToM031_JIG_8CP(String cmd, String data);
void processM031Command(String packet);
void readHostUART();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void setup() {
  Serial.begin(115200); 
  Serial2.begin(115200, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);

  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, LOW);

  for (int i = 0; i < 5; i++) { wifiList[i].ssid = ""; wifiList[i].pass = ""; }

  setupSDCard();
  loadWiFiConfig();
  loadPowerConfig(); 

  setupWiFi();

  server.on("/", handleRoot);
  server.on("/wifi", handleWiFiSet);
  server.on("/save", HTTP_POST, handleSaveWiFi);
  server.on("/monitor", handleMonitor);
  server.on("/api/data", handleApiData);     
  server.on("/api/power", HTTP_POST, handleApiPower);   
  server.on("/api/saveConfig", HTTP_POST, handleApiSaveConfig); 
  server.on("/api/downloadCSV", handleDownloadCSV); 
  server.onNotFound(handleNotFound); 
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  if (isAPMode) dnsServer.processNextRequest();
  server.handleClient();
  webSocket.loop(); 
  updateLED();
  
  checkSafetyLimits();
  readHostUART();

  if (Serial2.available()) {
    String barcodeData = Serial2.readStringUntil('\n'); 
    barcodeData.trim();
    if (barcodeData.length() > 0) sendToM031_JIG_8CP("SC", barcodeData);
  }
}

// ==========================================
// [UI 主動推播系統 (解決網頁卡死核心)]
// ==========================================
void broadcastWebSocket() {
  String wsJson = "{\"mA\":" + String(current_mA, 1) + 
                  ",\"v\":" + String(current_V, 2) + 
                  ",\"power\":\"" + power_state + "\"" + 
                  ",\"warning\":\"" + systemWarning + "\"" + 
                  ",\"mAh\":" + String(cumulative_mAh, 4) + 
                  ",\"mWh\":" + String(cumulative_mWh, 4) + 
                  ",\"logging\":" + (isRecordingCSV ? "true" : "false") + "}";
  webSocket.broadcastTXT(wsJson);
}

// ==========================================
// [WebSocket 事件攔截器]
// ==========================================
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) {
    // 當網頁一打開，立刻向 M031 詢問狀態並刷新 UI
    sendToM031_JIG_8CP("PW", "?");
    broadcastWebSocket();
  }
  else if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    if (msg.startsWith("REC_START:")) {
      logFilename = msg.substring(10);
      if (!logFilename.endsWith(".csv")) logFilename += ".csv";
      String fullPath = "/logs/" + logFilename;
      csvFile = SD.open(fullPath.c_str(), FILE_WRITE);
      if (csvFile) {
        // [關鍵修復] 寫入 UTF-8 BOM，防止 Microsoft Excel 開啟時變成中文亂碼
        const uint8_t bom[] = {0xEF, 0xBB, 0xBF};
        csvFile.write(bom, sizeof(bom));
        csvFile.println("時間(ms),電壓(V),電流(mA),容量(mAh),能量(mWh)");
        isRecordingCSV = true;
        broadcastWebSocket(); // 更新按鈕狀態
      }
    } 
    else if (msg == "REC_STOP") {
      if (isRecordingCSV) {
        csvFile.close();
        isRecordingCSV = false;
        broadcastWebSocket(); // 更新按鈕狀態
      }
    }
  }
}

// ==========================================
// [安全防禦與邏輯熔斷機制]
// ==========================================
void checkSafetyLimits() {
  unsigned long currentMillis = millis();
  bool stateChanged = false;

  // 1. Watchdog: 超過 1.5s 沒收到 PD，判定硬體斷電
  if (currentMillis - lastPdTime > 1500) {
    if (power_state == "ON") {
      power_state = "OFF";
      overCurrentStartTime = 0;
      if (isRecordingCSV) { csvFile.close(); isRecordingCSV = false; }
      stateChanged = true;
    }
  }

  // 2. 實時極限熔斷監控 
  if (power_state == "ON" && systemWarning == "") {
    
    // [關鍵修復] 硬體啟動寬限期 (2秒)：避免電容充電瞬間的突波被誤判為異常
    if (currentMillis - powerOnTime > 2000) {
        
        if (current_V < conf_minVoltage || current_V > conf_maxVoltage) {
          sendToM031_JIG_8CP("PW", "PA8OFF"); // 送出指令關閉 M031 治具
          power_state = "OFF";
          systemWarning = "【電壓超標熔斷】實時電壓 " + String(current_V, 2) + "V 觸及安全邊界 (" + String(conf_minVoltage, 1) + "V ~ " + String(conf_maxVoltage, 1) + "V)！";
          if (isRecordingCSV) { csvFile.println("ERROR,電壓超標安全熔斷"); csvFile.close(); isRecordingCSV = false; }
          stateChanged = true;
        }
        else if (current_mA > conf_limitScale) {
          if (overCurrentStartTime == 0) overCurrentStartTime = currentMillis; 
          else if (currentMillis - overCurrentStartTime >= conf_limitDuration * 1000UL) {
            sendToM031_JIG_8CP("PW", "PA8OFF"); // 送出指令關閉 M031 治具
            power_state = "OFF";
            systemWarning = "【電流超載延時熔斷】電流連續 " + String(conf_limitDuration) + " 秒超標 (" + String(current_mA, 1) + "mA > " + String(conf_limitScale, 1) + "mA)！";
            if (isRecordingCSV) { csvFile.println("ERROR,電流超載安全熔斷"); csvFile.close(); isRecordingCSV = false; }
            overCurrentStartTime = 0;
            stateChanged = true;
          }
        } else {
          overCurrentStartTime = 0; 
        }
    }
  }

  // 若狀態有改變 (包含被熔斷)，主動推播給網頁，網頁才不會卡死
  if (stateChanged) {
    broadcastWebSocket();
  }
}

// ==========================================
// [SD 卡與工具函式]
// ==========================================
void setupSDCard() {
  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spiSD)) return;
  if (!SD.exists("/Users")) SD.mkdir("/Users");
  if (!SD.exists("/PowerSet")) SD.mkdir("/PowerSet");
  if (!SD.exists("/logs")) SD.mkdir("/logs"); 
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
        String s_tag = "ssid" + String(i + 1); String p_tag = "password" + String(i + 1);
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
    for (int i = 0; i < 5; i++) { file.print("ssid"); file.print(i + 1); file.print(" = \""); file.print(wifiList[i].ssid); file.println("\""); file.print("password"); file.print(i + 1); file.print(" = \""); file.print(wifiList[i].pass); file.println("\""); }
    file.close();
  }
}
void loadPowerConfig() {
  if (SD.exists("/PowerSet/config.txt")) {
    File file = SD.open("/PowerSet/config.txt", FILE_READ);
    while (file.available()) {
      String line = file.readStringUntil('\n'); line.trim();
      if (line.startsWith("maxScale=")) conf_maxScale = line.substring(9).toFloat();
      else if (line.startsWith("limitScale=")) conf_limitScale = line.substring(11).toFloat();
      else if (line.startsWith("minVoltage=")) conf_minVoltage = line.substring(11).toFloat();
      else if (line.startsWith("maxVoltage=")) conf_maxVoltage = line.substring(11).toFloat();
      else if (line.startsWith("limitDuration=")) conf_limitDuration = line.substring(14).toInt();
    }
    file.close();
  } else { savePowerConfig(); }
}
void savePowerConfig() {
  File file = SD.open("/PowerSet/config.txt", FILE_WRITE);
  if (file) { file.println("maxScale=" + String(conf_maxScale)); file.println("limitScale=" + String(conf_limitScale)); file.println("minVoltage=" + String(conf_minVoltage)); file.println("maxVoltage=" + String(conf_maxVoltage)); file.println("limitDuration=" + String(conf_limitDuration)); file.close(); }
}

// ==========================================
// [LED 燈號控制系統]
// ==========================================
void setLedState(LedState state) { 
  currentLedState = state; 
  failBlinkCount = 0; 
  ledStateHigh = false; 
  digitalWrite(WIFI_LED_PIN, LOW); 
  previousLedMillis = millis(); 
}

void updateLED() { 
  unsigned long cm = millis(); 
  if (currentLedState == LED_ON) { digitalWrite(WIFI_LED_PIN, HIGH); } 
  else if (currentLedState == LED_OFF) { digitalWrite(WIFI_LED_PIN, LOW); } 
  else if (currentLedState == LED_BLINK_CONNECTING) { if (cm - previousLedMillis >= 500) { previousLedMillis = cm; ledStateHigh = !ledStateHigh; digitalWrite(WIFI_LED_PIN, ledStateHigh ? HIGH : LOW); } } 
  else if (currentLedState == LED_BLINK_FAIL) { if (failBlinkCount < 8) { if (cm - previousLedMillis >= 100) { previousLedMillis = cm; ledStateHigh = !ledStateHigh; digitalWrite(WIFI_LED_PIN, ledStateHigh ? HIGH : LOW); failBlinkCount++; } } else { digitalWrite(WIFI_LED_PIN, LOW); currentLedState = LED_OFF; } }
}

void setupWiFi() {
  setLedState(LED_BLINK_CONNECTING); WiFi.mode(WIFI_STA); WiFi.disconnect(); delay(100);
  int n = WiFi.scanNetworks(); MatchedWiFi matches[5]; int matchCount = 0; bool anyConnected = false;
  if (n > 0) {
    for (int i = 0; i < n; ++i) {
      String scannedSSID = WiFi.SSID(i); int32_t scannedRSSI = WiFi.RSSI(i);
      for (int j = 0; j < 5; j++) { if (wifiList[j].ssid != "" && wifiList[j].ssid == scannedSSID) { bool alreadyAdded = false; for(int k=0; k<matchCount; k++) { if(matches[k].ssid == scannedSSID) { alreadyAdded = true; if(scannedRSSI > matches[k].rssi) matches[k].rssi = scannedRSSI; break; } } if(!alreadyAdded && matchCount < 5) { matches[matchCount].ssid = wifiList[j].ssid; matches[matchCount].pass = wifiList[j].pass; matches[matchCount].rssi = scannedRSSI; matchCount++; } } }
    }
  }
  for (int i = 0; i < matchCount - 1; i++) { for (int j = i + 1; j < matchCount; j++) { if (matches[j].rssi > matches[i].rssi) { MatchedWiFi temp = matches[i]; matches[i] = matches[j]; matches[j] = temp; } } }
  for (int i = 0; i < matchCount; i++) { WiFi.begin(matches[i].ssid.c_str(), matches[i].pass.c_str()); int attempts = 0; while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); updateLED(); attempts++; } if (WiFi.status() == WL_CONNECTED) { anyConnected = true; break; } }
  if (!anyConnected) { for (int i = 0; i < 5; i++) { if (wifiList[i].ssid == "") continue; WiFi.begin(wifiList[i].ssid.c_str(), wifiList[i].pass.c_str()); int attempts = 0; while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); updateLED(); attempts++; } if (WiFi.status() == WL_CONNECTED) { anyConnected = true; break; } } }
  if (anyConnected) { setLedState(LED_ON); isAPMode = false; } else { setLedState(LED_BLINK_FAIL); startAPMode(); }
}
void startAPMode() { isAPMode = true; WiFi.mode(WIFI_AP); WiFi.softAP("JIG_8FT_P1_WIFIset"); dnsServer.start(DNS_PORT, "*", WiFi.softAPIP()); }

// ==========================================
// [Web Server 頁面路由與下載模組]
// ==========================================
void handleRoot() {
  String html = R"rawliteral(<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>JIG-8FT-P1 總控制台</title><style>body{font-family: Arial, sans-serif; text-align: center; padding: 40px; background-color: #f4f4f9;} .btn {display: block; width: 80%; max-width: 300px; margin: 20px auto; padding: 20px; font-size: 20px; font-weight: bold; color: white; background-color: #0056b3; border: none; border-radius: 10px; cursor: pointer; text-decoration: none; box-shadow: 0 4px 6px rgba(0,0,0,0.1);} .btn:hover {background-color: #004494;} .btn.alt {background-color: #28a745;} .btn.alt:hover {background-color: #218838;}</style></head><body><h2>⚙️ JIG-8FT-P1 控制面板</h2><a href='/wifi' class='btn'>🌐 網路備援設定</a><a href='/monitor' class='btn alt'>⚡ 電壓電流偵測</a></body></html>)rawliteral";
  server.send(200, "text/html", html);
}
void handleWiFiSet() {
  String html = "<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>網路備援設定</title><style>body{font-family:Arial; padding:20px; background:#f4f4f4;} .card{background:#fff; padding:15px; margin-bottom:15px; border-radius:8px; box-shadow:0 2px 4px rgba(0,0,0,0.1);} .back-btn{display:inline-block; margin-bottom:15px; text-decoration:none; color:#0056b3; font-weight:bold;}</style></head><body><a href='/' class='back-btn'>⬅ 返回首頁</a><h2>網路備援設定</h2><form action='/save' method='POST'>";
  html += "<div class='card'><h3>全域使用者 (User)</h3><input type='text' name='globalUser' value='" + globalUser + "' style='width:100%; padding:8px;'></div>";
  for (int i = 0; i < 5; i++) { html += "<div class='card'><h3>備援組 " + String(i + 1) + "</h3>網路名稱 (SSID):<br><input type='text' name='ssid" + String(i + 1) + "' value='" + wifiList[i].ssid + "' style='width:100%; padding:8px; margin-bottom:10px;'><br>密碼 (Password):<br><input type='text' name='pass" + String(i + 1) + "' value='" + wifiList[i].pass + "' style='width:100%; padding:8px;'><br></div>"; }
  html += "<input type='submit' value='💾 儲存並重啟設備' style='width:100%; padding:15px; font-size:18px; background:#4CAF50; color:white; border:none; border-radius:8px; cursor:pointer;'></form></body></html>";
  server.send(200, "text/html", html);
}
void handleSaveWiFi() {
  if (server.hasArg("globalUser")) globalUser = server.arg("globalUser");
  for (int i = 0; i < 5; i++) { if (server.hasArg("ssid"+String(i+1))) wifiList[i].ssid = server.arg("ssid"+String(i+1)); if (server.hasArg("pass"+String(i+1))) wifiList[i].pass = server.arg("pass"+String(i+1)); }
  saveWiFiConfig(); server.send(200, "text/html", "<meta charset='UTF-8'><h2>✅ 設定已儲存！ESP32 將重新啟動...</h2>"); delay(1000); ESP.restart();
}

void handleMonitor() {
  String html = R"rawliteral(
  <!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
  <title>電壓電流即時分析儀 V1.6.2</title>
  <style>
    body{font-family: Arial, sans-serif; background: #1e1e1e; color: #fff; margin: 0; padding: 10px; text-align: center;}
    .back-btn{display: block; text-align: left; color: #00c3ff; text-decoration: none; margin-bottom: 10px; font-weight: bold;}
    .dashboard {display: flex; flex-wrap: wrap; justify-content: center; gap: 15px;}
    .panel {background: #2d2d2d; border-radius: 10px; padding: 15px; width: 100%; max-width: 440px; box-sizing:border-box;}
    .val-text {font-size: 32px; font-weight: bold; color: #00ff66; text-shadow: 0 0 10px rgba(0,255,102,0.3);}
    .integ-box {background:#1a1a1a; border-radius:6px; padding:10px; margin:10px 0; display:flex; justify-content:space-around;}
    .integ-val {font-size:16px; color:#ffcc00; font-weight:bold;}
    .config-grid {display: grid; grid-template-columns: 1fr 1fr; gap: 8px; margin-top: 10px; text-align: left;}
    .config-grid div {display: flex; flex-direction: column; font-size:12px; color:#aaa;}
    input {width: 100%; padding: 5px; margin-top:2px; box-sizing: border-box; border-radius:4px; border:1px solid #444; background:#111; color:#fff;}
    button {padding: 10px; font-size: 14px; font-weight: bold; border: none; border-radius: 5px; cursor: pointer; color:#fff; width:100%; margin-top:8px;}
    .btn-save {background: #17a2b8;}
    .btn-rec {background: #ff8800; font-size:16px;} .btn-recording {background: #cc0000; animation: blink 1s infinite;}
    button.power {font-size:18px;}
    .power-off {background: #d9534f;} .power-on {background: #28a745;}
    canvas {background: #111; border-radius: 5px; margin-top:10px; width: 100%; height: 200px;}
    .modal {display:none; position:fixed; top:50%; left:50%; transform:translate(-50%,-50%); background:#e74c3c; padding:25px; border-radius:10px; z-index:999; width:80%; max-width:350px; box-shadow: 0 0 20px rgba(0,0,0,0.8);}
    .modal-overlay {display:none; position:fixed; top:0; left:0; width:100%; height:100%; background:rgba(0,0,0,0.6); z-index:998;}
    @keyframes blink { 0% {opacity:1;} 50% {opacity:0.5;} 100% {opacity:1;} }
  </style>
  </head><body>
  <a href='/' class='back-btn'>⬅ 返回首頁</a>
  <h2>⚡ JIG-8FT-P1 即時能耗分析儀</h2>
  
  <div class="dashboard">
    <div class="panel">
      <div class="val-text" id="curText">0.0 mA</div>
      <div style="color:#00c3ff; margin-top:3px; font-size:22px; font-weight:bold;">電壓: <span id="volText">0.00 V</span></div>
      
      <div class="integ-box">
        <div>🔋 累計容量:<br><span class="integ-val" id="mAhText">0.0000 mAh</span></div>
        <div>⚡ 累計能量:<br><span class="integ-val" id="mWhText">0.0000 mWh</span></div>
      </div>

      <button id="pwrBtn" class="power power-off" onclick="togglePower()">連線同步中...</button>
      <button id="recBtn" class="btn-rec" onclick="toggleRecording()">🔴 開始錄製測試報告 (.CSV)</button>

      <div class="config-grid">
         <div><label>刻度最大值 (mA)</label><input type="number" id="maxScale" value=")rawliteral" + String(conf_maxScale) + R"rawliteral("></div>
         <div><label>電流警報上限 (mA)</label><input type="number" id="limitScale" value=")rawliteral" + String(conf_limitScale) + R"rawliteral("></div>
         <div><label>容許超載時間 (秒)</label><input type="number" id="dur" value=")rawliteral" + String(conf_limitDuration) + R"rawliteral("></div>
         <div><label>電壓最低限制 (V)</label><input type="number" id="minVol" value=")rawliteral" + String(conf_minVoltage) + R"rawliteral("></div>
         <div><label>電壓最高限制 (V)</label><input type="number" id="maxVol" value=")rawliteral" + String(conf_maxVoltage) + R"rawliteral("></div>
      </div>
      <button class="btn-save" onclick="saveConfig()">💾 儲存保護參數至 SD 卡</button>
    </div>
    
    <div class="panel" style="max-width: 550px;">
      <canvas id="lineChart"></canvas>
    </div>
  </div>

  <div class="modal-overlay" id="modalOverlay"></div>
  <div class="modal" id="warnModal">
    <h2 style="margin:0 0 10px 0;">⚠ 安全防禦介入斷電</h2>
    <p id="warnMsg" style="font-size:16px; line-height:1.4;"></p>
    <button style="background:#fff; color:#e74c3c; padding:8px; margin-top:10px;" onclick="closeModal()">了解</button>
  </div>

  <script>
    const cvs = document.getElementById('lineChart');
    const ctx = cvs.getContext('2d');
    const maxPoints = 200; 
    let historyData = new Array(maxPoints).fill(0); 
    let ws;
    let localRecording = false;
    let localFilename = "";
    let JS_PowerState = "OFF"; // JS 內部獨立維護狀態

    function initWebSocket() {
      ws = new WebSocket('ws://' + window.location.hostname + ':81/');
      ws.onmessage = function(event) {
        const data = JSON.parse(event.data);
        updateUI(data);
      };
      ws.onclose = function() {
        document.getElementById('pwrBtn').innerText = "❌ 網路中斷，重連中...";
        setTimeout(initWebSocket, 2000); 
      };
    }

    initWebSocket();

    function updateUI(data) {
      JS_PowerState = data.power; // 確保前端變數 100% 同步

      document.getElementById('curText').innerText = data.mA.toFixed(1) + ' mA';
      document.getElementById('volText').innerText = data.v.toFixed(2) + ' V';
      document.getElementById('mAhText').innerText = data.mAh.toFixed(4) + ' mAh';
      document.getElementById('mWhText').innerText = data.mWh.toFixed(4) + ' mWh';
      
      const btn = document.getElementById('pwrBtn');
      if(data.power === "ON") {
        btn.className = "power power-on"; btn.innerText = "🔌 治具通電中 (點擊斷電)";
      } else {
        btn.className = "power power-off"; btn.innerText = "❌ 治具斷電中 (點擊通電)";
      }

      const recBtn = document.getElementById('recBtn');
      if(data.logging) {
        recBtn.className = "btn-rec btn-recording";
        recBtn.innerText = "⏸ 錄製中: " + localFilename + " (點擊結束)";
        localRecording = true;
      } else {
        recBtn.className = "btn-rec";
        recBtn.innerText = "🔴 開始錄製測試報告 (.CSV)";
        localRecording = false;
      }

      if(data.warning !== "") {
         document.getElementById("warnMsg").innerText = data.warning;
         document.getElementById("warnModal").style.display = "block";
         document.getElementById("modalOverlay").style.display = "block";
      }

      historyData.push(data.power === "ON" ? data.mA : 0);
      if(historyData.length > maxPoints) historyData.shift();
      drawChart();
    }

    function toggleRecording() {
      if(!localRecording) {
        let name = prompt("請輸入測試報告 CSV 檔名:", "JIG_TEST_" + Date.now() + ".csv");
        if(name === null || name.trim() === "") return;
        localFilename = name.trim();
        ws.send("REC_START:" + localFilename);
      } else {
        ws.send("REC_STOP");
        setTimeout(() => { window.location.href = "/api/downloadCSV?file=" + localFilename; }, 500);
      }
    }

    function drawChart() {
      cvs.width = cvs.clientWidth; cvs.height = cvs.clientHeight;
      const w = cvs.width, h = cvs.height;
      ctx.clearRect(0, 0, w, h);
      const maxScale = parseFloat(document.getElementById('maxScale').value) || 500;
      const limitVal = parseFloat(document.getElementById('limitScale').value) || 400;

      ctx.strokeStyle = '#383838'; ctx.fillStyle = '#888'; ctx.font = '10px Arial';
      ctx.beginPath();
      for(let i=0; i<=5; i++) {
        let y = Math.floor(i * (h/5)); ctx.moveTo(0, y); ctx.lineTo(w, y);
        ctx.fillText((maxScale - i * (maxScale/5)).toFixed(0), 5, y + 12);
      }
      for(let i=0; i<=10; i++) { let x = Math.floor(i * (w/10)); ctx.moveTo(x, 0); ctx.lineTo(x, h); }
      ctx.stroke();

      const limitY = h - (limitVal / maxScale) * h;
      if (limitY > 0 && limitY < h) {
        ctx.beginPath(); ctx.setLineDash([5, 5]); ctx.moveTo(0, limitY); ctx.lineTo(w, limitY);
        ctx.strokeStyle = '#ff3333'; ctx.stroke(); ctx.setLineDash([]);
      }

      ctx.beginPath(); ctx.strokeStyle = '#00ff66'; ctx.lineWidth = 2;
      const xStep = w / (maxPoints - 1);
      for(let i=0; i<historyData.length; i++) {
        let x = i * xStep; let y = h - (historyData[i] / maxScale) * h;
        if(i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    function togglePower() {
      const newState = (JS_PowerState === "OFF") ? "ON" : "OFF";
      fetch('/api/power', { method: 'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'}, body: 'state=' + newState });
      closeModal();
    }

    function saveConfig() {
      const params = new URLSearchParams({
        maxScale: document.getElementById('maxScale').value, limitScale: document.getElementById('limitScale').value,
        minVol: document.getElementById('minVol').value, maxVol: document.getElementById('maxVol').value, dur: document.getElementById('dur').value
      });
      fetch('/api/saveConfig', { method: 'POST', body: params }).then(() => alert("💾 安全配置已寫入 SD 卡保存！"));
    }

    function closeModal() { document.getElementById("warnModal").style.display = "none"; document.getElementById("modalOverlay").style.display = "none"; }
  </script>
  </body></html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleApiData() {
  String json = "{\"mA\":" + String(current_mA) + ",\"v\":" + String(current_V) + ",\"power\":\"" + power_state + "\",\"warning\":\"" + systemWarning + "\",\"mAh\":" + String(cumulative_mAh, 4) + ",\"mWh\":" + String(cumulative_mWh, 4) + ",\"logging\":" + (isRecordingCSV ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

// ==========================================
// [Web 控制按鈕接收器] 
// ==========================================
void handleApiPower() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "ON") {
      sendToM031_JIG_8CP("PW", "PA8ON"); 
      power_state = "ON"; 
      systemWarning = ""; 
      cumulative_mAh = 0.0; cumulative_mWh = 0.0; 
      overCurrentStartTime = 0;
      lastPdTime = millis(); // 給 Watchdog 寬限期
      powerOnTime = millis(); // [新增] 給硬體啟動寬限期，避免瞬間突波被判斷為超限
    } else {
      sendToM031_JIG_8CP("PW", "PA8OFF"); 
      power_state = "OFF";
      if (isRecordingCSV) { csvFile.close(); isRecordingCSV = false; }
    }
    broadcastWebSocket(); // 主動告訴前端按鈕狀態已變更
    server.send(200, "text/plain", "OK");
  }
}

void handleApiSaveConfig() {
  if (server.hasArg("maxScale")) conf_maxScale = server.arg("maxScale").toFloat();
  if (server.hasArg("limitScale")) conf_limitScale = server.arg("limitScale").toFloat();
  if (server.hasArg("minVol")) conf_minVoltage = server.arg("minVol").toFloat();
  if (server.hasArg("maxVol")) conf_maxVoltage = server.arg("maxVol").toFloat();
  if (server.hasArg("dur")) conf_limitDuration = server.arg("dur").toInt();
  savePowerConfig(); server.send(200, "text/plain", "OK");
}

void handleDownloadCSV() {
  if (server.hasArg("file")) {
    String path = "/logs/" + server.arg("file");
    if (SD.exists(path)) {
      File f = SD.open(path, FILE_READ);
      server.streamFile(f, "text/csv");
      f.close();
      return;
    }
  }
  server.send(404, "text/plain", "Log File Not Found");
}

void handleNotFound() {
  if (isAPMode) { server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true); server.send(302, "text/plain", ""); } 
  else { server.send(404, "text/plain", "Not Found"); }
}

void sendToM031_JIG_8CP(String cmd, String data) {
  String payload = cmd + data; unsigned int sum = 0;
  for (int i = 0; i < payload.length(); i++) { sum += payload[i]; }
  char hexSum[3]; sprintf(hexSum, "%02X", sum & 0xFF);
  Serial.write(JIG_8CP_STX); Serial.print(payload); Serial.print(hexSum); Serial.write(JIG_8CP_CR);  
}

void readHostUART() {
  static String rxBuffer = ""; static bool isReceiving = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == JIG_8CP_STX) { rxBuffer = ""; isReceiving = true; } 
    else if (c == JIG_8CP_CR && isReceiving) { isReceiving = false; processM031Command(rxBuffer); } 
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

  String cmd = cmdData.substring(0, 2); String data = cmdData.substring(2);

  if (cmd == "WI" && data == "?") {
    String currentIP = isAPMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    sendToM031_JIG_8CP("WI", currentIP); 
  }
  else if (cmd == "PW") {
    if (data.indexOf("ON") >= 0) {
      if (power_state != "ON") {
        powerOnTime = millis(); // [新增] 如果是實體按鈕開電，同樣給予啟動寬限期
        lastPdTime = millis();
      }
      power_state = "ON";
    }
    else if (data.indexOf("OFF") >= 0) power_state = "OFF";
    broadcastWebSocket();
  }
  else if (cmd == "PD") {
    lastPdTime = millis(); 

    int commaIdx = data.indexOf(',');
    if(commaIdx > 0) {
      current_mA = data.substring(0, commaIdx).toFloat();
      current_V = data.substring(commaIdx + 1).toFloat();
    }

    if (power_state == "ON") {
      cumulative_mAh += current_mA * (0.25 / 3600.0);
      cumulative_mWh += (current_mA * current_V) * (0.25 / 3600.0);

      if (isRecordingCSV && csvFile) {
        csvFile.printf("%lu,%.2f,%.1f,%.4f,%.4f\n", millis(), current_V, current_mA, cumulative_mAh, cumulative_mWh);
        csvFile.flush(); 
      }
    }
    
    // M031 定期回報時，也推播給網頁
    broadcastWebSocket();
  }
}