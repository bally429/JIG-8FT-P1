/*
 * ===========================================================================================
 * Project: JIG-8FT-P1 _WIFIBLE (ESP32 控制端)
 * MCU: ESP32-WROOM-32E (ESPWM32EN16)
 * [版本更新紀錄]
 * -------------------------------------------------------------------------------------------
 * V1.5    2026/07/08 [核心安全升級] Web圖表擴增時間軸、網格與刻度；導入隨機斷訊與超限熔斷機制。
 * V1.6    2026/07/08 移除輪 polled 改用 WebSocket，新增容量與能量計量，修正 CSV 首列亂碼。
 * V1.7    2026/07/09 [防護與人機優化] 
 * 1. 修正電壓保護邏輯：僅在電源 ON 時觸發防禦，徹底根除關電時「零點飄移」造成的連環警報。
 * 2. 實作參數即時同步：點擊網頁保存時，同步發送 [CF] 封包下發給 M031 更新保護硬體。
 * 3. 首頁增設動態恭敬尊稱：自動識別全域 User 變數並顯示專屬官方歡迎語。
 * V1.8    2026/07/14 [安全警示與狀態同步優化] 
 * 1. 修正安全警示視窗無法正確顯示問題：引入 safetyTripOccurred 標記，確保警示訊息在保護觸發後能被廣播一次，
 *    並在電源狀態穩定為 OFF 後自動清除，避免重複彈窗。
 * 2. 啟用 broadcastWebSocket Debug 輸出，便於監控警示訊息的發送。
 * V1.9    2026/07/14 [圖表與通訊同步優化] 
 * 1. 將 ESP32 WebSocket 廣播頻率從 50ms 調整為 20ms，與 M032 的 PD 指令發送頻率同步。
 * 2. 將時間軸繪製整合至主圖 lineChart，移除獨立的 timeAxisChart canvas，確保刻度與數據點精確對齊。
 * 3. 圖表時間軸單位從 50ms/格調整為 20ms/格，與廣播頻率一致。
 * V2.0    2026/07/16 [SD卡檔案管理功能] 
 * 1. 在首頁增加「📂 SD卡資料夾」按鈕。
 * 2. Web 檔案總管介面：實作了一個類似隨身碟的網頁介面，支援：
 *    - 瀏覽：點擊資料夾進入下一層。
 *    - 下載：點擊檔案即可下載到電腦。
 *    - 線上編輯：點擊 .txt 或 .csv 等文字檔，會開啟編輯器，修改後點擊儲存可直接覆蓋 SD 卡內的原始檔案。
 *    - 上傳：支援將電腦檔案拖曳或選擇上傳至當前資料夾。
 *    - 刪除：可刪除不需要的檔案或空資料夾。
 * V2.1    2026/07/16 [電源狀態保護優化]
 * 1. 修正當開啟電源後若沒有電流，不執行任何保護動作，也不發送關閉電源指令給 M031。
 * 2. 僅在有實際電流的情況下進行超載保護檢查。
 *V2.2    2026/07/22 [高負載緩衝與 SD 卡路徑修復]
 * 1. 擴大 UART RX Buffer 至 2048 bytes，完美吸收 SD 卡瀏覽等阻塞任務產生的數據洪峰，防止溢位。
 * 2. 優化 readHostUART 狀態機，加入長度防護防止 rxBuffer 無限增長耗盡記憶體。
 * 3. 修復 SD 卡編輯功能：對 server.arg("file") 套用 urlDecode()，解決子目錄路徑 "File Not Found" 問題。
===========================================================================================
*/

#define FIRMWARE_VERSION "V2.2"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <SPI.h>
#include <SD.h>
#include <WebSocketsServer.h> 
#include <time.h>

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
float conf_minScale = -50.0; // 🌟 [V2.2 新增] 預設折線圖刻度最小值，支援負值 (例如: -100mA)
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
unsigned long powerOnTime = 0; 
unsigned long lastWsBroadcastTime = 0; 
String systemWarning = "";             

// --- [新增] 全域變數：鬧鐘陣列 ---
struct AlarmConfig { int h; int m; int s; int en; };
AlarmConfig alarms[6] = { {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} };

// =======================================================
// [WIFI 與通訊狀態變數]
// =======================================================
char g_szWifiIP[20] = "";
uint8_t g_u8WifiConnected = 0;

// 【重大修復】將電源狀態升級為「系統全域變數」，讓網頁指令與實體紅按鈕共用同一個大腦
volatile int g_iPowerState = 0;

// --- [V1.8 新增] 全域變數：安全保護觸發標記 ---
bool safetyTripOccurred = false; // 標記是否發生了安全保護觸發

// --- 函式宣告 ---
void setupSDCard();
String extractQuote(String line);
void loadWiFiConfig();
void saveWiFiConfig();
void loadPowerConfig();
void savePowerConfig();
void loadAlarmsConfig(); // [新增]
void saveAlarmsConfig(); // [新增]
void setLedState(LedState state);
void updateLED();
void setupWiFi();
void startAPMode();
void initNTP();
void sendTimeToM031(struct tm *timeinfo);
void sendConfigToM031();
String getCSVStartTime();
void checkSafetyLimits();
void broadcastWebSocket();
void handleRoot();
void handleWiFiSet();
void handleSaveWiFi();
void handleMonitor();
void handleAlarmsSet();  // [新增] Web 鬧鐘介面
void handleApiSaveAlarms(); // [新增] Web 儲存鬧鐘
void handleApiData();
void handleApiPower();
void handleApiSaveConfig();
void handleDownloadCSV();
void handleNotFound();
void sendToM031_JIG_8CP(String cmd, String data);
void processM031Command(String packet);
void readHostUART();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

// --- [V2.0] 新增函式宣告：SD卡檔案管理 ---
void handleSdCard();
void handleSdBrowse();
void handleSdDownload();
void handleSdUpload();
void handleSdDelete();
void handleSdEdit();
void handleSdSaveFile();
String getContentType(String filename);
String getIconClass(const String& name, bool isDir);
String urlDecode(String str);
bool isTextFile(const String& filename);

void setup() {
  Serial.begin(115200);
  // 【關鍵修復】將預設 128 bytes 擴大至 2048 bytes，防止 SD 卡讀取阻塞時 UART 溢位
  Serial.setRxBufferSize(2048); 
    
  Serial2.begin(115200, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);
  Serial2.setRxBufferSize(2048);
    
  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, LOW);

  for (int i = 0; i < 5; i++) { wifiList[i].ssid = ""; wifiList[i].pass = ""; }

  setupSDCard();
  loadWiFiConfig();
  loadPowerConfig(); 
  loadAlarmsConfig(); // [新增] 讀取 SD 卡鬧鐘設定

  setupWiFi();

  server.on("/", handleRoot);
  server.on("/wifi", handleWiFiSet);
  server.on("/save", HTTP_POST, handleSaveWiFi);
  server.on("/monitor", handleMonitor);
  server.on("/alarms", handleAlarmsSet); // [新增] 路由
  server.on("/api/saveAlarms", HTTP_POST, handleApiSaveAlarms); // [新增] 路由
  server.on("/api/data", handleApiData);     
  server.on("/api/power", HTTP_POST, handleApiPower);   
  server.on("/api/saveConfig", HTTP_POST, handleApiSaveConfig); 
  server.on("/api/downloadCSV", handleDownloadCSV);
  // --- [V2.0] 新增路由 ---
  server.on("/sdcard", handleSdCard);
  server.on("/sd/browse", handleSdBrowse);
  server.on("/sd/download", handleSdDownload);
  server.on("/sd/upload", HTTP_POST, handleSdUpload);
  server.on("/sd/delete", HTTP_POST, handleSdDelete);
  server.on("/sd/edit", handleSdEdit);
  server.on("/sd/save", HTTP_POST, handleSdSaveFile);
  // --- END [V2.0] 新增路由 ---
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

  // --- [V1.9] 調整廣播頻率 ---
  if (millis() - lastWsBroadcastTime >= 20) { // 20ms
    broadcastWebSocket();
    lastWsBroadcastTime = millis();
  }
  // --- END [V1.9] 調整廣播頻率 ---
}

// ==========================================
// [SD 卡檔案系統與初始化]
// ==========================================
void setupSDCard() {
  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spiSD)) return;
  if (!SD.exists("/Users")) SD.mkdir("/Users");
  if (!SD.exists("/PowerSet")) SD.mkdir("/PowerSet");
  if (!SD.exists("/logs")) SD.mkdir("/logs"); 
  // --- [V2.0] 新增測試目錄 ---
  if (!SD.exists("/test")) SD.mkdir("/test");
  // --- END [V2.0] 新增測試目錄 ---
}

// ... (WiFi & PowerConfig load/save functions remain exactly the same) ...
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
      else if (line.startsWith("minScale=")) conf_minScale = line.substring(9).toFloat(); // 🌟 [V2.2 新增]
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
  if (file) { 
    file.println("maxScale=" + String(conf_maxScale)); 
    file.println("minScale=" + String(conf_minScale)); // 🌟 [V2.2 新增]
    file.println("limitScale=" + String(conf_limitScale)); 
    file.println("minVoltage=" + String(conf_minVoltage)); 
    file.println("maxVoltage=" + String(conf_maxVoltage)); 
    file.println("limitDuration=" + String(conf_limitDuration)); 
    file.close(); 
  }
}

// ==========================================
// [新增] 鬧鐘 SD 卡讀取與儲存引擎
// ==========================================
void loadAlarmsConfig() {
  if (SD.exists("/PowerSet/alarms.txt")) {
    File file = SD.open("/PowerSet/alarms.txt", FILE_READ);
    while (file.available()) {
      String line = file.readStringUntil('\n'); line.trim();
      if (line.length() == 0) continue;
      int idx, h, m, s, en;
      if (sscanf(line.c_str(), "%d,%d,%d,%d,%d", &idx, &h, &m, &s, &en) == 5) {
        if (idx >= 0 && idx < 6) {
          alarms[idx].h = h; alarms[idx].m = m; alarms[idx].s = s; alarms[idx].en = en;
        }
      }
    }
    file.close();
  } else { 
    saveAlarmsConfig(); // 初始化建立預設檔案
  }
}

void saveAlarmsConfig() {
  File file = SD.open("/PowerSet/alarms.txt", FILE_WRITE);
  if (file) {
    for (int i = 0; i < 6; i++) {
      file.printf("%d,%02d,%02d,%02d,%d\n", i, alarms[i].h, alarms[i].m, alarms[i].s, alarms[i].en);
    }
    file.close();
  }
}

// ==========================================
// [新增] SD卡檔案管理功能函式
// ==========================================

String getIconClass(const String& name, bool isDir) {
  if (isDir) return "📁";
  String ext = name.substring(name.lastIndexOf('.'));
  if (ext.equalsIgnoreCase(".txt") || ext.equalsIgnoreCase(".csv") || ext.equalsIgnoreCase(".json") || ext.equalsIgnoreCase(".xml") || ext.equalsIgnoreCase(".ini") || ext.equalsIgnoreCase(".cfg")) {
    return "📝";
  } else if (ext.equalsIgnoreCase(".jpg") || ext.equalsIgnoreCase(".jpeg") || ext.equalsIgnoreCase(".png") || ext.equalsIgnoreCase(".gif") || ext.equalsIgnoreCase(".bmp")) {
    return "🖼️";
  } else if (ext.equalsIgnoreCase(".pdf")) {
    return "📄";
  } else if (ext.equalsIgnoreCase(".zip") || ext.equalsIgnoreCase(".rar") || ext.equalsIgnoreCase(".7z")) {
    return "📦";
  }
  return "📄";
}

String urlDecode(String str) {
  String decoded = "";
  char c;
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (c == '+') {
      decoded += ' ';
    } else if (c == '%' && i + 2 < str.length()) {
      char hex[3] = {str.charAt(i+1), str.charAt(i+2), 0};
      decoded += (char) strtol(hex, NULL, 16);
      i += 2;
    } else {
      decoded += c;
    }
  }
  return decoded;
}

bool isTextFile(const String& filename) {
  String ext = filename.substring(filename.lastIndexOf('.'));
  return (ext.equalsIgnoreCase(".txt") || ext.equalsIgnoreCase(".csv") || ext.equalsIgnoreCase(".json") || ext.equalsIgnoreCase(".xml") || ext.equalsIgnoreCase(".ini") || ext.equalsIgnoreCase(".cfg") || ext.equalsIgnoreCase(".log"));
}

void handleSdCard() {
  String html = R"rawliteral(<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>SD Card File Manager</title><style>body{font-family: Arial, sans-serif; text-align: center; padding: 40px; background-color: #f4f4f9;} .btn {display: block; width: 80%; max-width: 300px; margin: 20px auto; padding: 20px; font-size: 20px; font-weight: bold; color: white; background-color: #0056b3; border: none; border-radius: 10px; cursor: pointer; text-decoration: none; box-shadow: 0 4px 6px rgba(0,0,0,0.1); margin-bottom: 20px;} .btn:hover {background-color: #004494;} .btn.alt {background-color: #28a745;} .btn.alt:hover {background-color: #218838;} .btn.alt2 {background-color: #f39c12;} .btn.alt2:hover {background-color: #e67e22;} .btn.sd {background-color: #8e44ad;} .btn.sd:hover {background-color: #7d3c98;}</style></head><body><h2>💾 SD Card Management</h2><a href='/' class='btn'>🏠 返回首頁</a><a href='/sd/browse' class='btn sd'>📂 瀏覽 SD 卡</a></body></html>)rawliteral";
  server.send(200, "text/html", html);
}

void handleSdBrowse() {
  String path = server.arg("path");
  if (path == "") path = "/";
  if (path.charAt(0) != '/') path = "/" + path;
  if (path.charAt(path.length()-1) != '/') path += "/";

  File root = SD.open(path);
  if (!root || !root.isDirectory()) {
    server.send(404, "text/plain", "Directory not found");
    return;
  }

  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0'>
    <title>SD Card Browser</title>
    <style>
      body { font-family: Arial, sans-serif; background-color: #f4f4f9; padding: 20px; }
      .header { background: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin-bottom: 20px; }
      .file-list { background: #fff; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); overflow: hidden; }
      .file-item { padding: 12px 15px; border-bottom: 1px solid #eee; display: flex; justify-content: space-between; align-items: center; }
      .file-item:last-child { border-bottom: none; }
      .file-name { font-weight: bold; color: #333; cursor: pointer; }
      .file-name:hover { color: #007bff; }
      .file-info { color: #666; font-size: 0.9em; }
      .file-actions a { margin-left: 10px; text-decoration: none; font-weight: bold; color: #007bff; }
      .file-actions a.delete { color: #dc3545; }
      .file-actions a.edit { color: #28a745; }
      .upload-section { background: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin-top: 20px; }
      .back-btn { display: inline-block; margin-bottom: 15px; text-decoration: none; color: #0056b3; font-weight: bold; }
      .upload-area { border: 2px dashed #ccc; padding: 20px; text-align: center; border-radius: 5px; margin-top: 10px; }
      .upload-area.dragover { border-color: #007bff; background-color: #f8f9fa; }
    </style>
  </head>
  <body>
    <a href='/sdcard' class='back-btn'>🏠 SD 卡總覽</a>
    <div class='header'>
      <h2>📂 Current Path: )rawliteral" + path + R"rawliteral(</h2>
    </div>
    <div class='file-list'>
  )rawliteral";

  File file = root.openNextFile();
  while (file) {
    String fileName = file.name();
    String displayName = fileName.substring(fileName.lastIndexOf('/') + 1);
    String sizeStr = file.isDirectory() ? "(dir)" : String(file.size()) + " bytes";
    bool isDir = file.isDirectory();

    html += "<div class='file-item'>";
    html += "<div class='file-name' onclick=\"location.href='/sd/browse?path=" + path + displayName + "'\">" + getIconClass(displayName, isDir) + " " + displayName + "</div>";
    html += "<div class='file-info'>" + sizeStr + "</div>";
    html += "<div class='file-actions'>";
    if (!isDir) {
        html += "<a href='/sd/download?file=" + path + displayName + "'>⬇️ Download</a>";
        if (isTextFile(displayName)) {
            html += "<a href='/sd/edit?file=" + path + displayName + "' class='edit'>✏️ Edit</a>";
        }
        html += "<a href='#' class='delete' onclick='deleteFile(\"" + path + displayName + "\")'>🗑️ Delete</a>";
    } else {
        html += "<a href='#' class='delete' onclick='deleteFolder(\"" + path + displayName + "\")'>🗑️ Delete</a>";
    }
    html += "</div>";
    html += "</div>";

    file = root.openNextFile();
  }
  root.close();

  html += R"rawliteral(
    </div>
    <div class='upload-section'>
      <h3>📤 Upload to this folder</h3>
      <form id='uploadForm' enctype='multipart/form-data' method='post' action='/sd/upload'>
        <input type='hidden' name='path' value=')rawliteral" + path + R"rawliteral('>
        <input type='file' name='upload' required>
        <button type='submit'>Upload</button>
      </form>
      <div id='dropArea' class='upload-area'>
        <p>Drag & Drop files here or click to select</p>
        <input type='file' id='hiddenFileInput' style='display: none;' multiple>
      </div>
    </div>

    <script>
      function deleteFile(filePath) {
        if (confirm('Are you sure you want to delete ' + filePath + '?')) {
          fetch('/sd/delete', {
            method: 'POST',
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: 'file=' + encodeURIComponent(filePath)
          }).then(response => response.text()).then(result => {
            if(result === 'OK') location.reload();
            else alert('Delete failed: ' + result);
          });
        }
      }
      function deleteFolder(folderPath) {
        if (confirm('Are you sure you want to delete folder ' + folderPath + '? Only empty folders can be deleted.')) {
          fetch('/sd/delete', {
            method: 'POST',
            headers: {'Content-Type': 'application/x-www-form-urlencoded'},
            body: 'folder=' + encodeURIComponent(folderPath)
          }).then(response => response.text()).then(result => {
            if(result === 'OK') location.reload();
            else alert('Delete failed: ' + result);
          });
        }
      }

      // Drag & drop functionality
      const dropArea = document.getElementById('dropArea');
      const hiddenFileInput = document.getElementById('hiddenFileInput');

      ['dragenter', 'dragover', 'dragleave', 'drop'].forEach(eventName => {
        dropArea.addEventListener(eventName, preventDefaults, false);
      });

      function preventDefaults(e) {
        e.preventDefault();
        e.stopPropagation();
      }

      ['dragenter', 'dragover'].forEach(eventName => {
        dropArea.addEventListener(eventName, highlight, false);
      });

      ['dragleave', 'drop'].forEach(eventName => {
        dropArea.addEventListener(eventName, unhighlight, false);
      });

      function highlight(e) {
        dropArea.classList.add('dragover');
      }

      function unhighlight(e) {
        dropArea.classList.remove('dragover');
      }

      dropArea.addEventListener('drop', handleDrop, false);

      function handleDrop(e) {
        const dt = e.dataTransfer;
        const files = dt.files;
        uploadFiles(files);
      }

      dropArea.addEventListener('click', () => {
        hiddenFileInput.click();
      });

      hiddenFileInput.addEventListener('change', (e) => {
        uploadFiles(e.target.files);
      });

      function uploadFiles(files) {
        const formData = new FormData();
        const path = document.querySelector('input[name="path"]').value;
        for (let i = 0; i < files.length; i++) {
          formData.append('files', files[i]);
        }
        formData.append('path', path);

        fetch('/sd/upload', {
          method: 'POST',
          body: formData
        }).then(response => response.text()).then(result => {
          if(result === 'OK') location.reload();
          else alert('Upload failed: ' + result);
        });
      }
    </script>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleSdDownload() {
  String filePath = server.arg("file");
  if (filePath == "" || !SD.exists(filePath)) {
    server.send(404, "text/plain", "File Not Found");
    return;
  }
  File file = SD.open(filePath, FILE_READ);
  if (!file) {
    server.send(500, "text/plain", "Could not open file");
    return;
  }
  server.streamFile(file, "application/octet-stream");
  file.close();
}

void handleSdUpload() {
    String path = server.arg("path");
    if (path == "") path = "/";

    // Handle single file upload via form
    if (server.hasArg("upload")) {
        HTTPUpload& upload = server.upload();
        if (upload.filename != "") {
            String full_path = path + upload.filename;
            File f = SD.open(full_path, FILE_WRITE);
            if (f) {
                f.write(upload.buf, upload.currentSize);
                f.close();
                server.send(200, "text/plain", "OK");
                return;
            } else {
                server.send(500, "text/plain", "Could not create file");
                return;
            }
        }
    }

    // Handle multipart file uploads (drag & drop)
    if (server.hasArg("files")) {
        HTTPUpload& upload = server.upload();
        if (upload.filename != "") {
            String full_path = path + upload.filename;
            File f = SD.open(full_path, FILE_WRITE);
            if (f) {
                f.write(upload.buf, upload.currentSize);
                f.close();
            } else {
                server.send(500, "text/plain", "Could not create file: " + full_path);
                return;
            }
        }
    }

    server.send(200, "text/plain", "OK");
}

void handleSdDelete() {
  String fileArg = server.arg("file");
  String folderArg = server.arg("folder");

  if (fileArg != "") {
    if (SD.exists(fileArg)) {
      if (SD.remove(fileArg)) {
        server.send(200, "text/plain", "OK");
        return;
      } else {
        server.send(500, "text/plain", "Could not delete file");
        return;
      }
    } else {
      server.send(404, "text/plain", "File not found");
      return;
    }
  }

  if (folderArg != "") {
    if (SD.exists(folderArg)) {
      if (SD.rmdir(folderArg)) {
        server.send(200, "text/plain", "OK");
        return;
      } else {
        server.send(500, "text/plain", "Could not delete folder (not empty?)");
        return;
      }
    } else {
      server.send(404, "text/plain", "Folder not found");
      return;
    }
  }

  server.send(400, "text/plain", "No file or folder specified");
}

void handleSdEdit() {
  String filePath = urlDecode(server.arg("file")); // <--- 加上 urlDecode
  if (filePath == "" || !SD.exists(filePath) || !isTextFile(filePath)) {
      server.send(400, "text/plain", "Invalid file for editing");
      return;
  }
  File file = SD.open(filePath, FILE_READ);
  if (!file) {
      server.send(500, "text/plain", "Could not open file");
      return;
  }

  String content = file.readString();
  file.close();

  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0'>
    <title>Edit File: )rawliteral" + filePath + R"rawliteral(</title>
    <style>
      body { font-family: Arial, sans-serif; background-color: #f4f4f9; padding: 20px; }
      .editor-container { background: #fff; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); padding: 20px; }
      textarea { width: 100%; height: 60vh; padding: 10px; font-family: monospace; font-size: 14px; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
      .button-group { margin-top: 15px; display: flex; justify-content: space-between; }
      button { padding: 10px 20px; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; }
      .save-btn { background-color: #28a745; color: white; }
      .back-btn { background-color: #6c757d; color: white; }
    </style>
  </head>
  <body>
    <div class='editor-container'>
      <h2>✏️ Editing: )rawliteral" + filePath + R"rawliteral(</h2>
      <form method='post' action='/sd/save'>
        <input type='hidden' name='file' value=')rawliteral" + filePath + R"rawliteral('>
        <textarea id='editor' name='content'>)rawliteral" + content + R"rawliteral(</textarea>
        <div class='button-group'>
          <a href='/sd/browse?path=)rawliteral" + filePath.substring(0, filePath.lastIndexOf('/')) + R"rawliteral(' class='back-btn'>Back</a>
          <button type='submit' class='save-btn'>💾 Save Changes</button>
        </div>
      </form>
    </div>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleSdSaveFile() {
  String filePath = urlDecode(server.arg("file")); // <--- 加上 urlDecode
  String content = server.arg("content");

  if (filePath == "" || content == "") {
    server.send(400, "text/plain", "Missing file or content");
    return;
  }

  File file = SD.open(filePath, FILE_WRITE);
  if (!file) {
    server.send(500, "text/plain", "Could not open file for writing");
    return;
  }

  file.print(content);
  file.close();

  server.send(200, "text/plain", "OK");
}

// ... (其余函数保持不变) ...
void handleRoot() {
  // 動態判定全域使用者尊稱
  String welcomeMsg = "歡迎您使用生技治具控制台！";
  if (globalUser != "" && globalUser.length() > 0) {
    welcomeMsg = "歡迎 「" + globalUser + "」 使用生技治具控制台！";
  }

  String html = R"rawliteral(<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>JIG-8FT-P1 總控制台</title><style>body{font-family: Arial, sans-serif; text-align: center; padding: 40px; background-color: #f4f4f9;} .btn {display: block; width: 80%; max-width: 300px; margin: 20px auto; padding: 20px; font-size: 20px; font-weight: bold; color: white; background-color: #0056b3; border: none; border-radius: 10px; cursor: pointer; text-decoration: none; box-shadow: 0 4px 6px rgba(0,0,0,0.1); margin-bottom: 20px;} .btn:hover {background-color: #004494;} .btn.alt {background-color: #28a745;} .btn.alt:hover {background-color: #218838;} .btn.alt2 {background-color: #f39c12;} .btn.alt2:hover {background-color: #e67e22;} .btn.sd {background-color: #8e44ad;} .btn.sd:hover {background-color: #7d3c98;} .welcome-msg {font-size: 18px; color: #2c3e50; margin: 15px 0; font-weight: bold;} </style></head><body><h2>🕹️ JIG-8FT-P1 控制面板 ()rawliteral" + String(FIRMWARE_VERSION) + R"rawliteral()</h2><div class='welcome-msg'>)rawliteral" 
  + welcomeMsg + 
  R"rawliteral(</div><a href='/wifi' class='btn'>🌐 網路備援設定</a><a href='/monitor' class='btn alt'>⚡ 電壓電流設定</a><a href='/alarms' class='btn alt2'>⏰ 多工鬧鐘設定</a><a href='/sdcard' class='btn sd'>📂 SD卡資料夾</a></body></html>)rawliteral";
  
  server.send(200, "text/html", html);
}

// [新增] Web UI：鬧鐘設定面板
void handleAlarmsSet() {
  String html = "<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>系統鬧鐘設定</title>";
  html += "<style>body{font-family:Arial; padding:20px; background:#f4f4f4;} .card{background:#fff; padding:15px; margin-bottom:15px; border-radius:8px; box-shadow:0 2px 4px rgba(0,0,0,0.1); display:flex; justify-content:space-between; align-items:center;} .back-btn{display:inline-block; margin-bottom:15px; text-decoration:none; color:#0056b3; font-weight:bold;}</style>";
  html += "<script>function saveAll() { let fd = new FormData(document.getElementById('aForm')); fetch('/api/saveAlarms', {method:'POST', body:new URLSearchParams(fd)}).then(()=>alert('✅ 鬧鐘已更新並成功同步至 M031 主板！')); }</script>";
  html += "</head><body><a href='/' class='back-btn'>⬅ 返回首頁</a><h2>⏰ 系統鬧鐘設定</h2><form id='aForm' onsubmit='event.preventDefault(); saveAll();'>";

  for (int i=0; i<6; i++) {
     char tbuf[16];
     snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d", alarms[i].h, alarms[i].m, alarms[i].s);
     String checked = alarms[i].en ? "checked" : "";
     html += "<div class='card'><div><h3 style='margin-top:0;'>鬧鐘 " + String(i+1) + "</h3>";
     html += "<input type='time' step='1' name='t" + String(i) + "' value='" + String(tbuf) + "' style='font-size:18px; padding:5px;'></div>";
     html += "<div><label style='font-size:18px; font-weight:bold;'><input type='checkbox' name='en" + String(i) + "' value='1' " + checked + " style='width:22px; height:22px; vertical-align:middle;'> 啟用開關</label></div></div>";
  }
  html += "<button type='submit' style='width:100%; padding:15px; font-size:18px; background:#4CAF50; color:white; border:none; border-radius:8px; cursor:pointer;'>💾 儲存並同步至設備</button></form></body></html>";
  server.send(200, "text/html", html);
}

// [新增] Web API：接收網頁的鬧鐘修改，並主動下發給 M031
void handleApiSaveAlarms() {
  for (int i=0; i<6; i++) {
     String t_val = server.arg("t" + String(i)); // Web form 送出的時間字串 "HH:MM:SS" 或 "HH:MM"
     int en = server.hasArg("en" + String(i)) ? 1 : 0;
     
     if (t_val.length() >= 5) {
        int h = t_val.substring(0, 2).toInt();
        int m = t_val.substring(3, 5).toInt();
        int s = (t_val.length() >= 8) ? t_val.substring(6, 8).toInt() : 0; // 若沒輸入秒數，預設 0
        alarms[i].h = h; alarms[i].m = m; alarms[i].s = s; alarms[i].en = en;

        // 【同步至 M031】
        char buf[32];
        snprintf(buf, sizeof(buf), "%d,%02d,%02d,%02d,%d", i, h, m, s, en);
        sendToM031_JIG_8CP("AL", buf);
        delay(80); // 給 M031 UART 處理時間，避免封包黏在一起被丟棄
     }
  }
  saveAlarmsConfig(); // 將最新狀態寫入 SD 卡
  server.send(200, "text/plain", "OK");
}

// ... (UI HTML of Monitor & WiFi settings remain same as V1.7.0, omitted for brevity) ...
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
  <title>電壓電流即時分析儀 V2.2</title>
  <style>
    body{font-family: Arial, sans-serif; background: #1e1e1e; color: #fff; margin: 0; padding: 10px; text-align: center;}
    .back-btn{display: block; text-align: left; color: #00c3ff; text-decoration: none; margin-bottom: 10px; font-weight: bold;}
    .dashboard {display: flex; flex-wrap: wrap; justify-content: center; gap: 15px;}
    .panel {background: #2d2d2d; border-radius: 10px; padding: 15px; width: 100%; max-width: 440px; box-sizing:border-box;}
    .val-text {font-size: 32px; font-weight: bold; color: #00ff66; text-shadow: 0 0 10px rgba(0,255,102,0.3);}
    .panel-chart {background: #2d2d2d; border-radius: 10px; padding: 15px; width: 100%; max-width: 600px; box-sizing:border-box;}
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
    canvas {background: #111; border-radius: 5px; margin-top:10px; width: 100%; height: 260px;}
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
         <div><label>刻度最小值 (mA)</label><input type="number" id="minScale" value=")rawliteral" + String(conf_minScale) + R"rawliteral("></div>
         <div><label>電流警報上限 (mA)</label><input type="number" id="limitScale" value=")rawliteral" + String(conf_limitScale) + R"rawliteral("></div>
         <div><label>容許超載時間 (秒)</label><input type="number" id="dur" value=")rawliteral" + String(conf_limitDuration) + R"rawliteral("></div>
         <div><label>電壓最低限制 (V)</label><input type="number" id="minVol" value=")rawliteral" + String(conf_minVoltage) + R"rawliteral("></div>
         <div><label>電壓最高限制 (V)</label><input type="number" id="maxVol" value=")rawliteral" + String(conf_maxVoltage) + R"rawliteral("></div>
      </div>
      <button class="btn-save" onclick="saveConfig()">💾 儲存保護參數至 SD 卡</button>
    </div>
    
    <div class="panel-chart">
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
    let JS_PowerState = "OFF";
    let lastDisplayedWarning = ""; 

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
      JS_PowerState = data.power; 

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

      if (data.warning !== "") {
        if (data.warning !== lastDisplayedWarning) {
           document.getElementById("warnMsg").innerText = data.warning;
           document.getElementById("warnModal").style.display = "block";
           document.getElementById("modalOverlay").style.display = "block";
           lastDisplayedWarning = data.warning; 
        }
      } else {
        lastDisplayedWarning = ""; 
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
      cvs.width = cvs.clientWidth; 
      cvs.height = cvs.clientHeight;
      const w = cvs.width, h = cvs.clientHeight;
      ctx.clearRect(0, 0, w, h);

      // 🌟 [V2.2 重要重構] 取得最大值與最小值
      const maxScale = parseFloat(document.getElementById('maxScale').value) || 500;
      const minScale = parseFloat(document.getElementById('minScale').value) || 0;
      const limitVal = parseFloat(document.getElementById('limitScale').value) || 400;
      const range = maxScale - minScale; // 刻度全區間

      // 繪製背景網格與動態 Y 軸刻度
      ctx.strokeStyle = '#383838'; ctx.fillStyle = '#888'; ctx.font = '10px Arial';
      ctx.beginPath();
      for(let i=0; i<=5; i++) {
        let y = Math.floor(i * (h/5)); 
        ctx.moveTo(0, y); ctx.lineTo(w, y);
        // 動態計算每一格的數值
        let gridVal = maxScale - (i * (range / 5));
        ctx.fillText(gridVal.toFixed(0), 5, y + 12);
      }
      for(let i=0; i<=10; i++) { let x = Math.floor(i * (w/10)); ctx.moveTo(x, 0); ctx.lineTo(x, h); }
      ctx.stroke();

      // 🌟 [V2.2 修正] 繪製過流保護限制紅虛線位置公式
      const limitY = h - ((limitVal - minScale) / range) * h;
      if (limitY > 0 && limitY < h) {
        ctx.beginPath(); ctx.setLineDash([5, 5]); ctx.moveTo(0, limitY); ctx.lineTo(w, limitY);
        ctx.strokeStyle = '#ff3333'; ctx.stroke(); ctx.setLineDash([]);
      }

      // 🌟 [V2.2 修正] 數據點繪製公式
      ctx.beginPath(); ctx.strokeStyle = '#00ff66'; ctx.lineWidth = 2;
      const xStep = w / (maxPoints - 1);
      for(let i=0; i<historyData.length; i++) {
        let x = i * xStep; 
        // 帶入公式，精確處理包含負值的像素 Y 軸映射
        let y = h - ((historyData[i] - minScale) / range) * h;
        if(i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();

      // 繪製時間軸
      const axisY = h - 10; 
      ctx.beginPath(); ctx.moveTo(0, axisY); ctx.lineTo(w, axisY);
      ctx.strokeStyle = '#aaa'; ctx.stroke();

      ctx.strokeStyle = '#555'; ctx.lineWidth = 1;
      const MS_PER_POINT = 20; 
      const GRID_POINTS = 5; 
      const totalGrids = Math.floor(historyData.length / GRID_POINTS);

      for(let i = 0; i <= totalGrids; i++) {
        let pointIndex = i * GRID_POINTS;
        if (pointIndex < historyData.length) {
          let x = pointIndex * xStep;
          if (x <= w) {
            ctx.beginPath(); ctx.moveTo(x, axisY); ctx.lineTo(x, axisY + 10); ctx.stroke();
          }
        }
      }

      ctx.fillStyle = '#aaa'; ctx.font = '12px Arial'; ctx.textAlign = 'right'; ctx.textBaseline = 'bottom';
      ctx.fillText(MS_PER_POINT * GRID_POINTS + ' ms/格', w - 5, h - 5);
    }

    function togglePower() {
      const newState = (JS_PowerState === "OFF") ? "ON" : "OFF";
      fetch('/api/power', { method: 'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'}, body: 'state=' + newState });
      closeModal();
    }

    function saveConfig() {
      // 🌟 [V2.2 新增] 打包 minScale 引數送回伺服器
      const params = new URLSearchParams({
        maxScale: document.getElementById('maxScale').value, 
        minScale: document.getElementById('minScale').value, 
        limitScale: document.getElementById('limitScale').value,
        minVol: document.getElementById('minVol').value, 
        maxVol: document.getElementById('maxVol').value, 
        dur: document.getElementById('dur').value
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

void handleApiPower() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "ON") {
      sendToM031_JIG_8CP("PW", "ON"); 
      power_state = "ON"; systemWarning = ""; cumulative_mAh = 0.0; cumulative_mWh = 0.0; overCurrentStartTime = 0; lastPdTime = millis(); powerOnTime = millis(); 
    } else {
      sendToM031_JIG_8CP("PW", "OFF"); power_state = "OFF";
      systemWarning = ""; // 【修復點：手動斷電時清空警報字串，徹底解鎖網頁 Alert 視窗】
      if (isRecordingCSV) { csvFile.close(); isRecordingCSV = false; }
    }
    broadcastWebSocket();
    server.send(200, "text/plain", "OK");
  }
}

void handleApiSaveConfig() {
  if (server.hasArg("maxScale")) conf_maxScale = server.arg("maxScale").toFloat();
  if (server.hasArg("minScale")) conf_minScale = server.arg("minScale").toFloat(); // 🌟 [V2.2 新增]
  if (server.hasArg("limitScale")) conf_limitScale = server.arg("limitScale").toFloat();
  if (server.hasArg("minVol")) conf_minVoltage = server.arg("minVol").toFloat();
  if (server.hasArg("maxVol")) conf_maxVoltage = server.arg("maxVol").toFloat();
  if (server.hasArg("dur")) conf_limitDuration = server.arg("dur").toInt();
  
  savePowerConfig(); // 儲存至 ESP32 本端 SD 卡
  
  sendConfigToM031(); // 即時同步：主動發送下發 [CF] 封包給 M031 更新硬體熔斷保護臨界值
  
  server.send(200, "text/plain", "OK");
}

void handleDownloadCSV() {
  if (server.hasArg("file")) {
    String path = "/logs/" + server.arg("file");
    if (SD.exists(path)) { File f = SD.open(path, FILE_READ); server.streamFile(f, "text/csv"); f.close(); return; }
  }
  server.send(404, "text/plain", "Log File Not Found");
}

void handleNotFound() {
  if (isAPMode) { server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true); server.send(302, "text/plain", ""); } 
  else { server.send(404, "text/plain", "Not Found"); }
}

// ==========================================
// [通訊與網路模組核心]
// ==========================================
// ... (WIFI and LED Setup remain unchanged)
void setLedState(LedState state) { currentLedState = state; failBlinkCount = 0; ledStateHigh = false; digitalWrite(WIFI_LED_PIN, LOW); previousLedMillis = millis(); }
void updateLED() { unsigned long cm = millis(); if (currentLedState == LED_ON) { digitalWrite(WIFI_LED_PIN, HIGH); } else if (currentLedState == LED_OFF) { digitalWrite(WIFI_LED_PIN, LOW); } else if (currentLedState == LED_BLINK_CONNECTING) { if (cm - previousLedMillis >= 500) { previousLedMillis = cm; ledStateHigh = !ledStateHigh; digitalWrite(WIFI_LED_PIN, ledStateHigh ? HIGH : LOW); } } else if (currentLedState == LED_BLINK_FAIL) { if (failBlinkCount < 8) { if (cm - previousLedMillis >= 100) { previousLedMillis = cm; ledStateHigh = !ledStateHigh; digitalWrite(WIFI_LED_PIN, ledStateHigh ? HIGH : LOW); failBlinkCount++; } } else { digitalWrite(WIFI_LED_PIN, LOW); currentLedState = LED_OFF; } } }
void startAPMode() { isAPMode = true; WiFi.mode(WIFI_AP); WiFi.softAP("JIG_8FT_P1_WIFIset"); dnsServer.start(DNS_PORT, "*", WiFi.softAPIP()); }
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
  
  if (anyConnected) { setLedState(LED_ON); isAPMode = false; initNTP(); } 
  else { setLedState(LED_BLINK_FAIL); startAPMode(); }
}

void initNTP() {
    configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    struct tm timeinfo;
    int retry = 0;
    while (!getLocalTime(&timeinfo) && retry < 10) { delay(500); retry++; }
    if (retry < 10) { sendTimeToM031(&timeinfo); }
}

void sendTimeToM031(struct tm *timeinfo) {
    char dataBuf[32]; snprintf(dataBuf, sizeof(dataBuf), "%04d,%02d,%02d,%02d,%02d,%02d", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    sendToM031_JIG_8CP("ST", dataBuf);
}

void sendConfigToM031() {
    char dataBuf[64]; snprintf(dataBuf, sizeof(dataBuf), "%.1f,%.1f,%.1f,%.1f,%d", conf_maxScale, conf_limitScale, conf_minVoltage, conf_maxVoltage, conf_limitDuration);
    sendToM031_JIG_8CP("CF", dataBuf); 
}

void broadcastWebSocket() {
  String wsJson = "{\"mA\":" + String(current_mA, 1) + ",\"v\":" + String(current_V, 2) + ",\"power\":\"" + power_state + "\"" + ",\"warning\":\"" + systemWarning + "\"" + ",\"mAh\":" + String(cumulative_mAh, 4) + ",\"mWh\":" + String(cumulative_mWh, 4) + ",\"logging\":" + (isRecordingCSV ? "true" : "false") + "}";
  
  // --- [V1.8] Debug Print ---
  Serial.println("WS Broadcast: " + wsJson);
  // --- END [V1.8] Debug Print ---

  webSocket.broadcastTXT(wsJson);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) { sendToM031_JIG_8CP("PW", "?"); broadcastWebSocket(); }
  else if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    if (msg.startsWith("REC_START:")) {
      logFilename = msg.substring(10); if (!logFilename.endsWith(".csv")) logFilename += ".csv";
      String fullPath = "/logs/" + logFilename; csvFile = SD.open(fullPath.c_str(), FILE_WRITE);
      if (csvFile) { const uint8_t bom[] = {0xEF, 0xBB, 0xBF}; csvFile.write(bom, sizeof(bom)); csvFile.println("日期時間(含毫秒),電壓(V),電流(mA),容量(mAh),能量(mWh)"); isRecordingCSV = true; broadcastWebSocket(); }
    } else if (msg == "REC_STOP") { if (isRecordingCSV) { csvFile.close(); isRecordingCSV = false; broadcastWebSocket(); } }
  }
}

String getCSVStartTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) { return String(millis()); }
    char timeStringBuff[64]; snprintf(timeStringBuff, sizeof(timeStringBuff), "%04d/%02d/%02d %02d:%02d:%02d.%03lu", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, millis() % 1000);
    return String(timeStringBuff);
}

void checkSafetyLimits() {
  unsigned long currentMillis = millis(); bool stateChanged = false;

  // --- 檢查 M032 是否停止發送 PD 指令 ---
  const unsigned long PD_TIMEOUT_MS = 10000; 
  if (currentMillis - lastPdTime > PD_TIMEOUT_MS) { 
    if (power_state == "ON") { 
      power_state = "OFF"; 
      overCurrentStartTime = 0; 
      if (isRecordingCSV) { csvFile.close(); isRecordingCSV = false; } 
      stateChanged = true; 
      systemWarning = "【M032 通訊逾時】超過 " + String(PD_TIMEOUT_MS/1000.0, 1) + " 秒未收到電力數據，自動斷電保護！";
      safetyTripOccurred = true; 
    }
  }

  // --- 檢查硬體安全限制 ---
  if (power_state == "ON") { // 只有在通電時才檢查
    if (currentMillis - powerOnTime > 2000) {
        // 檢查電壓
        if (current_V < conf_minVoltage || current_V > conf_maxVoltage) {
          // --- [V2.1] 修正：只有在有電流時才觸發保護 ---
          if (current_mA > 0) { // 僅在有實際電流的情況下進行保護檢查
            if (!safetyTripOccurred) {
              sendToM031_JIG_8CP("PW", "OFF"); 
              systemWarning = "【電壓超標熔斷】實時電壓 " + String(current_V, 2) + "V 觸及安全邊界！";
              safetyTripOccurred = true; 
              if (isRecordingCSV) { csvFile.println("ERROR,電壓超標安全熔斷"); csvFile.close(); isRecordingCSV = false; }
              overCurrentStartTime = 0; // 清除電流計時器
              stateChanged = true;
              power_state = "OFF"; 
            }
          }
        }
        // 檢查電流
        else if (current_mA > conf_limitScale) {
          // --- [V2.1] 修正：只有在有電流時才觸發保護 ---
          if (current_mA > 0) { // 僅在有實際電流的情況下進行保護檢查
            if (overCurrentStartTime == 0) overCurrentStartTime = currentMillis;
            else if (currentMillis - overCurrentStartTime >= conf_limitDuration * 1000UL) {
              if (!safetyTripOccurred) {
                sendToM031_JIG_8CP("PW", "OFF");
                systemWarning = "【電流超載延時熔斷】電流連續超標！";
                safetyTripOccurred = true; 
                if (isRecordingCSV) { csvFile.println("ERROR,電流超載安全熔斷"); csvFile.close(); isRecordingCSV = false; }
                overCurrentStartTime = 0;
                stateChanged = true;
                power_state = "OFF"; 
              }
            }
          } else {
             // 如果電流不在超標範圍，重置計時器
             overCurrentStartTime = 0;
          }
        } else { 
          // 如果電流未超標，重置計時器
          overCurrentStartTime = 0; 
        }
    }
  }

  // --- 修正：清除邏輯 (在狀態穩定後) ---
  
  // 【刪除情況1】: 不要急著在這裡清除 systemWarning，保留給 WebSocket 發送給網頁！
  // 網頁接收到警告並彈出視窗後，只要使用者重新按下電源開關(handleApiPower)，警告字串自然會被清空。

  // 情況2: 電源狀態為 ON (例如用戶手動打開) -> 重置標記，準備下次保護
  if (power_state == "ON" && safetyTripOccurred) {
      safetyTripOccurred = false; // 重置保護觸發標記
      Serial.println("Reset Tripped Flag (ON)"); // Debug
  }

  // --- END 修正 ---

  if (stateChanged) { 
      broadcastWebSocket(); 
  }
}

void sendToM031_JIG_8CP(String cmd, String data) {
  String payload = cmd + data; unsigned int sum = 0;
  for (int i = 0; i < payload.length(); i++) { sum += payload[i]; }
  char hexSum[3]; sprintf(hexSum, "%02X", sum & 0xFF);
  Serial.write(JIG_8CP_STX); Serial.print(payload); Serial.print(hexSum); Serial.write(JIG_8CP_CR);  
}

void readHostUART() {
    static String rxBuffer = ""; 
    static bool isReceiving = false;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == JIG_8CP_STX) { 
            rxBuffer = ""; 
            isReceiving = true; 
        }
        else if (c == JIG_8CP_CR && isReceiving) { 
            isReceiving = false; 
            processM031Command(rxBuffer); 
            rxBuffer = ""; // 確保解析後清空
        }
        else if (isReceiving) { 
            rxBuffer += c;
            // <--- 新增防護機制：防止因丟失 CR 導致緩衝區無限增長耗盡記憶體
            if (rxBuffer.length() > 128) { 
                isReceiving = false;
                rxBuffer = "";
            }
        }
    }
}

void processM031Command(String packet) {
  if (packet.length() < 4) return; 
  String cmdData = packet.substring(0, packet.length() - 2);
  String receivedChk = packet.substring(packet.length() - 2);
  unsigned int sum = 0; for (int i = 0; i < cmdData.length(); i++) { sum += cmdData[i]; }
  char calcChk[3]; sprintf(calcChk, "%02X", sum & 0xFF);
  if (receivedChk != String(calcChk)) return; 

  String cmd = cmdData.substring(0, 2); String data = cmdData.substring(2);

  if (cmd == "WI" && data == "?") {
    String currentIP = isAPMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    sendToM031_JIG_8CP("WI", currentIP); 
  }
  else if (cmd == "GC" && data == "?") { sendConfigToM031(); }
  
  // ===============================================
  // [新增] 接收 M031 的鬧鐘相關請求
  // ===============================================
  else if (cmd == "AL" && data == "?") {
    // M031 開機請求讀取全域 6 組鬧鐘
    // 清空 M031 端可能存在的舊設定（可選，確保一致性）
    // for(int i = 0; i < 6; i++) { sendToM031_JIG_8CP("AL", String(i)+",0,0,0,0"); }
    for(int i = 0; i < 6; i++) {
       char buf[32];
       snprintf(buf, sizeof(buf), "%d,%02d,%02d,%02d,%d", i, alarms[i].h, alarms[i].m, alarms[i].s, alarms[i].en);
       sendToM031_JIG_8CP("AL", buf);
       delay(80); // 必須有小延遲，防止 M031 UART 處理不及漏封包
    }
    // 發送完畢後，可選發送一個結束標誌（雖然 M032 不一定處理）
    // sendToM031_JIG_8CP("AL_END", "");
  }
  // --- 修改：處理 M031 的 AS 請求 ---
  else if (cmd == "AS") {
    // M031 按下黃色鍵離開選單，要求存入 SD 卡
    int idx=0, h=0, m=0, s=0, en=0;
    if (sscanf(data.c_str(), "%d,%d,%d,%d,%d", &idx, &h, &m, &s, &en) == 5) {
       if(idx >= 0 && idx < 6) {
          alarms[idx].h = h; alarms[idx].m = m; alarms[idx].s = s; alarms[idx].en = en;
          saveAlarmsConfig(); // 直接更新寫入
          // 回應 OK 確認 (新增)
          sendToM031_JIG_8CP("AS_OK", "OK");
       }
    }
  }
  // ===============================================
  
  else if (cmd == "ST" && data == "OK") { Serial.println("RTC Sync OK"); }
  else if (cmd == "PW") {
    if (data.indexOf("ON") >= 0) { if (power_state != "ON") { powerOnTime = millis(); lastPdTime = millis(); } power_state = "ON"; }
    else if (data.indexOf("OFF") >= 0) power_state = "OFF";
    broadcastWebSocket();
  }
  else if (cmd == "PD") {
    unsigned long currentMillis = millis(); float deltaSec = 0.0;
    if (lastPdTime > 0) { deltaSec = (currentMillis - lastPdTime) / 1000.0; if (deltaSec > 2.0 || deltaSec <= 0) deltaSec = 0.1; }
    lastPdTime = currentMillis; 
    int commaIdx = data.indexOf(',');
    if(commaIdx > 0) { current_mA = data.substring(0, commaIdx).toFloat(); current_V = data.substring(commaIdx + 1).toFloat(); }
    if (power_state == "ON") {
      cumulative_mAh += current_mA * (deltaSec / 3600.0); cumulative_mWh += (current_mA * current_V) * (deltaSec / 3600.0);
      if (isRecordingCSV && csvFile) {
        csvFile.printf("%s,%.2f,%.1f,%.4f,%.4f\n", getCSVStartTime().c_str(), current_V, current_mA, cumulative_mAh, cumulative_mWh);
        static unsigned long lastFlushTime = 0; if (currentMillis - lastFlushTime >= 1000) { csvFile.flush(); lastFlushTime = currentMillis; }
      }
    }
  }
}