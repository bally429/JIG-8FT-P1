/*
===========================================================================================
Project: JIG-8FT-P1 _WIFIBLE (ESP32 控制端)
MCU: ESP32-WROOM-32E (ESPWM32EN16)
[版本更新紀錄]
V1.5 ~ V3.3 (略) ... 包含圖表優化、SD卡管理、電壓折線圖、手動下載、排版對齊等。
V3.4    2026/07/24 [Web OTA 韌體更新]
  1. 新增 Web OTA 功能：支援透過網頁直接上傳 .bin 檔案更新 ESP32 韌體。
  2. 首頁新增「🔄 韌體更新」入口，內建 AJAX 上傳與即時進度條顯示。
  3. 更新完成後設備自動重啟，無需插拔 SD 卡或連接 USB 線。
V4.0  2026/07/28 [M031 OTA 第2階段] 新增 /api/isp_test，驗證 ESP32→M031 LDROM 之 UART1 ISP 握手
      (CONNECT+GET_DEVICEID+RUN_APROM)，全程不寫 Flash。
V4.1  2026/07/28 [M031 OTA 第3a步] 新增 /m031_ota 上傳 + 校驗 + dry-run 預檢。
      檔名安全閘 m031_*.bin；版本由檔名解析；dry-run 純 host 端模擬位址覆蓋，不擦 M031。
      真正燒錄於 V4.2 啟用。
V4.2  2026/07/29 [M031 OTA 第3步] 新增 /m031_ota_burn 真正燒錄：經 LDROM UPDATE_APROM 擦寫 APROM。
      同步阻塞於 handler（天然隔離 JIG_8CP 背景流量）；首包長 timeout 容納整片擦除；
      每包 checksum 比對＝寫後驗證；燒完 RUN_APROM + VR 版本比對 + 雙 reset 同步。
      新增 /api/restart；前端預估進度條。通電強制斷電雙層防護。
V4.3  2026/07/30  🌐 網路備援設定 介面優化
===========================================================================================
*/
#define FIRMWARE_VERSION  "V4.3"

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <SPI.h>
#include <SD.h>
#include <WebSocketsServer.h>
#include <Update.h> //  [V3.4 新增] OTA 更新標頭檔
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
float conf_minScale = -50.0;
float conf_limitScale = 400.0;
float conf_minVoltage = 0.0;
float conf_maxVoltage = 12.0;
int conf_limitDuration = 3;
float conf_vMaxScale = 15.0;
float conf_vMinScale = 0.0;

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

// --- 鬧鐘陣列 ---
struct AlarmConfig { int h; int m; int s; int en; };
AlarmConfig alarms[6] = { {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0} };

// --- 函式宣告 ---
void setupSDCard();
String extractQuote(String line);
void loadWiFiConfig();
void saveWiFiConfig();
void loadPowerConfig();
void savePowerConfig();
void loadAlarmsConfig();
void saveAlarmsConfig();
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
void handleAlarmsSet();
void handleApiSaveAlarms();
void handleApiData();
void handleApiPower();
void handleApiSaveConfig();
void handleDownloadCSV();
void handleNotFound();
void sendToM031_JIG_8CP(String cmd, String data);
void processM031Command(String packet);
void readHostUART();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

// --- SD卡檔案管理 ---
void handleSdCard();
void handleSdBrowse();
void handleSdDownload();
void handleSdUpload();
void handleSdDelete();
void handleSdEdit();
void handleSdSaveFile();
String getIconClass(const String& name, bool isDir);
String urlDecode(String str);
bool isTextFile(const String& filename);

// 🌟 [V3.4 新增] OTA 韌體更新函式宣告
void handleFirmwareUpdate();
void handleFirmwareUpload();
// ===== [V4.1 M031 OTA 第3a步] forward declaration =====
void handleM031Ota();
void handleM031OtaUpload();
void handleM031OtaUploadDone();
void handleM031OtaReport();
void handleM031OtaDryrun();
void handleM031OtaBurn();
void handleRestart();


void setup() {
  Serial.begin(115200);
  Serial.setRxBufferSize(2048);
  
  Serial2.begin(115200, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);
  Serial2.setRxBufferSize(2048);
  
  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, LOW);
  for (int i = 0; i < 5; i++) { wifiList[i].ssid = ""; wifiList[i].pass = ""; }
  
  setupSDCard();
  loadWiFiConfig();
  loadPowerConfig();
  loadAlarmsConfig();
  setupWiFi();
  
  // --- 網頁路由設定 ---
  server.on("/", handleRoot);
  server.on("/wifi", handleWiFiSet);
  server.on("/save", HTTP_POST, handleSaveWiFi);
  server.on("/monitor", handleMonitor);
  server.on("/alarms", handleAlarmsSet);
  server.on("/api/saveAlarms", HTTP_POST, handleApiSaveAlarms);
  server.on("/api/data", handleApiData); 
  server.on("/api/power", HTTP_POST, handleApiPower); 
  server.on("/api/saveConfig", HTTP_POST, handleApiSaveConfig);
  server.on("/api/downloadCSV", handleDownloadCSV);
  // server.on("/api/isp_test", HTTP_GET, handleIspTest);   // [OTA 第2階段] M031 LDROM 握手驗證
  server.on("/api/wifi/scan",        HTTP_GET, handleApiWifiScan);
  server.on("/api/wifi/scan_result", HTTP_GET, handleApiWifiScanResult);

  // SD卡路由
  server.on("/sdcard", handleSdCard);
  server.on("/sd/browse", handleSdBrowse);
  server.on("/sd/download", handleSdDownload);
  server.on("/sd/upload", HTTP_POST, handleSdUpload);
  server.on("/sd/delete", HTTP_POST, handleSdDelete);
  server.on("/sd/edit", handleSdEdit);
  server.on("/sd/save", HTTP_POST, handleSdSaveFile);

  // 🌟 [V3.4 新增] OTA 韌體更新路由
  server.on("/firmware", HTTP_GET, handleFirmwareUpdate);
  server.on("/firmware", HTTP_POST, 
    [](){ 
      server.sendHeader("Connection", "close"); 
      server.send(200, "text/plain; charset=UTF-8", "OK"); 
    }, 
    handleFirmwareUpload
  );
  
  server.on("/m031_ota", HTTP_GET, handleM031Ota);
  server.on("/m031_ota_upload", HTTP_POST, handleM031OtaUploadDone, handleM031OtaUpload);
  server.on("/m031_ota_report", HTTP_GET, handleM031OtaReport);
  server.on("/api/m031_ota_dryrun", HTTP_GET, handleM031OtaDryrun);
  server.on("/m031_ota_burn", HTTP_POST, handleM031OtaBurn);
  server.on("/api/restart", HTTP_GET, handleRestart);

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
  if (millis() - lastWsBroadcastTime >= 20) { 
    broadcastWebSocket();
    lastWsBroadcastTime = millis();
  }
}

// ==========================================
// [SD 卡與系統初始化]
// ==========================================
void setupSDCard() {
  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spiSD)) return;
  if (!SD.exists("/Users")) SD.mkdir("/Users");
  if (!SD.exists("/PowerSet")) SD.mkdir("/PowerSet");
  if (!SD.exists("/logs")) SD.mkdir("/logs");
  if (!SD.exists("/test")) SD.mkdir("/test");
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
      if (line.startsWith("User = ")) globalUser = extractQuote(line);
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
    for (int i = 0; i < 5; i++) { 
      file.print("ssid"); file.print(i + 1); file.print(" = \""); file.print(wifiList[i].ssid); file.println("\""); 
      file.print("password"); file.print(i + 1); file.print(" = \""); file.print(wifiList[i].pass); file.println("\""); 
    }
    file.close();
  }
}

void loadPowerConfig() {
  if (SD.exists("/PowerSet/config.txt")) {
    File file = SD.open("/PowerSet/config.txt", FILE_READ);
    while (file.available()) {
      String line = file.readStringUntil('\n'); line.trim();
      if (line.startsWith("maxScale=")) conf_maxScale = line.substring(9).toFloat();
      else if (line.startsWith("minScale=")) conf_minScale = line.substring(9).toFloat();
      else if (line.startsWith("limitScale=")) conf_limitScale = line.substring(11).toFloat();
      else if (line.startsWith("minVoltage=")) conf_minVoltage = line.substring(11).toFloat();
      else if (line.startsWith("maxVoltage=")) conf_maxVoltage = line.substring(11).toFloat();
      else if (line.startsWith("limitDuration=")) conf_limitDuration = line.substring(14).toInt();
      else if (line.startsWith("vMaxScale=")) conf_vMaxScale = line.substring(10).toFloat();
      else if (line.startsWith("vMinScale=")) conf_vMinScale = line.substring(10).toFloat();
    }
    file.close();
  } else { savePowerConfig(); }
}

void savePowerConfig() {
  File file = SD.open("/PowerSet/config.txt", FILE_WRITE);
  if (file) {
    file.println("maxScale=" + String(conf_maxScale));
    file.println("minScale=" + String(conf_minScale));
    file.println("limitScale=" + String(conf_limitScale));
    file.println("minVoltage=" + String(conf_minVoltage));
    file.println("maxVoltage=" + String(conf_maxVoltage));
    file.println("limitDuration=" + String(conf_limitDuration));
    file.println("vMaxScale=" + String(conf_vMaxScale));
    file.println("vMinScale=" + String(conf_vMinScale));
    file.close();
  }
}

void loadAlarmsConfig() {
  if (SD.exists("/PowerSet/alarms.txt")) {
    File file = SD.open("/PowerSet/alarms.txt", FILE_READ);
    while (file.available()) {
      String line = file.readStringUntil('\n'); line.trim();
      if (line.length() == 0) continue;
      int idx, h, m, s, en;
      if (sscanf(line.c_str(), "%d,%d,%d,%d,%d", &idx, &h, &m, &s, &en) == 5) {
        if (idx >= 0 && idx < 6) { alarms[idx].h = h; alarms[idx].m = m; alarms[idx].s = s; alarms[idx].en = en; }
      }
    }
    file.close();
  } else { saveAlarmsConfig(); }
}

void saveAlarmsConfig() {
  File file = SD.open("/PowerSet/alarms.txt", FILE_WRITE);
  if (file) {
    for (int i = 0; i < 6; i++) { file.printf("%d,%02d,%02d,%02d,%d\n", i, alarms[i].h, alarms[i].m, alarms[i].s, alarms[i].en); }
    file.close();
  }
}

// ==========================================
// [SD卡檔案管理功能]
// ==========================================
String getIconClass(const String & name, bool isDir) {
  if (isDir) return "📁";
  String ext = name.substring(name.lastIndexOf('.'));
  if (ext.equalsIgnoreCase(".txt") || ext.equalsIgnoreCase(".csv") || ext.equalsIgnoreCase(".json") || ext.equalsIgnoreCase(".xml") || ext.equalsIgnoreCase(".ini") || ext.equalsIgnoreCase(".cfg")) return "📝";
  else if (ext.equalsIgnoreCase(".jpg") || ext.equalsIgnoreCase(".jpeg") || ext.equalsIgnoreCase(".png") || ext.equalsIgnoreCase(".gif") || ext.equalsIgnoreCase(".bmp")) return "🖼️";
  else if (ext.equalsIgnoreCase(".pdf")) return "📄";
  else if (ext.equalsIgnoreCase(".zip") || ext.equalsIgnoreCase(".rar") || ext.equalsIgnoreCase(".7z")) return "📦";
  return "📄";
}

String urlDecode(String str) {
  String decoded = "";
  char c;
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (c == '+') { decoded += ' '; } 
    else if (c == '%' && i + 2 < str.length()) {
      char hex[3] = {str.charAt(i+1), str.charAt(i+2), 0};
      decoded += (char) strtol(hex, NULL, 16);
      i += 2;
    } else { decoded += c; }
  }
  return decoded;
}

bool isTextFile(const String & filename) {
  String ext = filename.substring(filename.lastIndexOf('.'));
  return (ext.equalsIgnoreCase(".txt") || ext.equalsIgnoreCase(".csv") || ext.equalsIgnoreCase(".json") || ext.equalsIgnoreCase(".xml") || ext.equalsIgnoreCase(".ini") || ext.equalsIgnoreCase(".cfg") || ext.equalsIgnoreCase(".log"));
}

void handleSdCard() {
  String html = R"rawliteral(<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>SD Card File Manager</title><style>body{font-family: Arial, sans-serif; text-align: center; padding: 40px; background-color: #f4f4f9;} .btn {display: block; width: 80%; max-width: 300px; margin: 20px auto; padding: 20px; font-size: 20px; font-weight: bold; color: white; background-color: #0056b3; border: none; border-radius: 10px; cursor: pointer; text-decoration: none; box-shadow: 0 4px 6px rgba(0,0,0,0.1); margin-bottom: 20px;} .btn:hover {background-color: #004494;} .btn.sd {background-color: #8e44ad;} .btn.sd:hover {background-color: #7d3c98;} </style></head><body><h2> SD Card Management</h2><a href='/' class='btn'>🏠 返回首頁</a><a href='/sd/browse' class='btn sd'>📂 瀏覽 SD 卡</a></body></html>)rawliteral";
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleSdBrowse() {
  String path = server.arg("path");
  if (path == "") path = "/";
  if (path.charAt(0) != '/') path = "/" + path;
  if (path.charAt(path.length()-1) != '/') path += "/";
  File root = SD.open(path);
  if (!root || !root.isDirectory()) { server.send(404, "text/plain", "Directory not found"); return; }

  String html = R"rawliteral(
<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>SD Card Browser</title>
<style>
body { font-family: Arial, sans-serif; background-color: #f4f4f9; padding: 20px; }
.header { background: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin-bottom: 20px; }
.file-list { background: #fff; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); overflow: hidden; }
.file-item { display: flex; align-items: center; padding: 12px 15px; border-bottom: 1px solid #eee; }
.file-item:last-child { border-bottom: none; }
.col-name { flex: 3; font-weight: bold; color: #333; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; margin-right: 10px; cursor: pointer;}
.col-name:hover { color: #007bff; }
.col-size { flex: 1; text-align: center; color: #666; font-size: 0.9em; display: flex; flex-direction: column; justify-content: center;}
.col-action { flex: 2; text-align: right; display: flex; justify-content: flex-end; gap: 15px; align-items: center;}
.col-action a { text-decoration: none; font-weight: bold; color: #007bff; }
.col-action a.delete { color: #dc3545; }
.col-action a.edit { color: #28a745; }
.back-btn { display: inline-block; margin-bottom: 15px; text-decoration: none; color: #0056b3; font-weight: bold; }
.warn-badge { color: #d9534f; font-size: 0.75em; font-weight: bold; margin-top: 2px; }
.warn-text { color: #d9534f; font-size: 0.85em; font-weight: bold; }
</style></head><body>
<a href='/sdcard' class='back-btn'>🏠 SD 卡總覽</a>
<div class='header'><h2>📂 Current Path: )rawliteral" + path + R"rawliteral(</h2></div>
<div class='file-list'>)rawliteral";

  File file = root.openNextFile();
  while (file) {
    String fileName = file.name();
    String displayName = fileName.substring(fileName.lastIndexOf('/') + 1);
    bool isDir = file.isDirectory();
    unsigned long size = isDir ? 0 : file.size();
    
    String sizeStr;
    if (size < 1024) sizeStr = String(size) + " B";
    else if (size < 1024 * 1024) sizeStr = String(size / 1024.0, 1) + " KB";
    else if (size < 1024 * 1024 * 1024) sizeStr = String(size / (1024.0 * 1024.0), 1) + " MB";
    else sizeStr = String(size / (1024.0 * 1024.0 * 1024.0), 1) + " GB";

    bool isLargeFile = (!isDir && size > 3 * 1024 * 1024); 

    html += "<div class='file-item'>";
    if (isDir) {
      html += "<div class='col-name' onclick=\"location.href='/sd/browse?path=" + path + displayName + "'\">📁 " + displayName + "</div>";
    } else {
      html += "<div class='col-name' onclick=\"location.href='/sd/browse?path=" + path + displayName + "'\">" + getIconClass(displayName, false) + " " + displayName + "</div>";
    }

    html += "<div class='col-size'>" + sizeStr;
    if (isLargeFile) html += "<span class='warn-badge'>⚠️ >3MB</span>";
    html += "</div>";

    html += "<div class='col-action'>";
    if (!isDir) {
      html += "<a href='/sd/download?file=" + path + displayName + "'>⬇️ Download</a>";
      if (isTextFile(displayName)) {
        if (isLargeFile) html += "<span class='warn-text'>⚠️ 大檔不建議編輯</span>";
        else html += "<a href='/sd/edit?file=" + path + displayName + "' class='edit'>✏️ Edit</a>";
      }
      html += "<a href='#' class='delete' onclick='deleteFile(\"" + path + displayName + "\")'>🗑️ Delete</a>";
    } else {
      html += "<a href='#' class='delete' onclick='deleteFolder(\"" + path + displayName + "\")'>🗑️ Delete</a>";
    }
    html += "</div></div>";
    file = root.openNextFile();
  }
  root.close();

  html += R"rawliteral(</div>
<script>
function deleteFile(filePath) { if (confirm('確定要刪除 ' + filePath + ' ?')) { fetch('/sd/delete', { method: 'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'}, body: 'file=' + encodeURIComponent(filePath) }).then(r => r.text()).then(res => { if(res === 'OK') location.reload(); else alert('刪除失敗: ' + res); }); } }
function deleteFolder(folderPath) { if (confirm('確定要刪除資料夾 ' + folderPath + ' ? (僅限空資料夾)')) { fetch('/sd/delete', { method: 'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'}, body: 'folder=' + encodeURIComponent(folderPath) }).then(r => r.text()).then(res => { if(res === 'OK') location.reload(); else alert('刪除失敗: ' + res); }); } }
</script></body></html>)rawliteral";
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleSdDownload() {
  String filePath = server.arg("file");
  if (filePath == "" || !SD.exists(filePath)) { server.send(404, "text/plain", "File Not Found"); return; }
  File file = SD.open(filePath, FILE_READ);
  if (!file) { server.send(500, "text/plain", "Could not open file"); return; }
  String fileName = filePath.substring(filePath.lastIndexOf('/') + 1);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + fileName + "\"");
  server.streamFile(file, "application/octet-stream");
  file.close();
}

void handleSdUpload() {
  String path = server.arg("path");
  if (path == "") path = "/";
  if (server.hasArg("upload")) {
    HTTPUpload& upload = server.upload();
    if (upload.filename != "") {
      String full_path = path + upload.filename;
      File f = SD.open(full_path, FILE_WRITE);
      if (f) { f.write(upload.buf, upload.currentSize); f.close(); server.send(200, "text/plain; charset=UTF-8", "OK"); return; } 
      else { server.send(500, "text/plain", "Could not create file"); return; }
    }
  }
  server.send(200, "text/plain; charset=UTF-8", "OK");
}

void handleSdDelete() {
  String fileArg = server.arg("file");
  String folderArg = server.arg("folder");
  if (fileArg != "") {
    if (SD.exists(fileArg)) { if (SD.remove(fileArg)) { server.send(200, "text/plain; charset=UTF-8", "OK"); return; } else { server.send(500, "text/plain", "Could not delete file"); return; } } 
    else { server.send(404, "text/plain", "File not found"); return; }
  }
  if (folderArg != "") {
    if (SD.exists(folderArg)) { if (SD.rmdir(folderArg)) { server.send(200, "text/plain; charset=UTF-8","OK"); return; } else { server.send(500, "text/plain", "Could not delete folder (not empty?)"); return; } } 
    else { server.send(404, "text/plain", "Folder not found"); return; }
  }
  server.send(400, "text/plain", "No file or folder specified");
}

void handleSdEdit() {
  String filePath = urlDecode(server.arg("file"));
  if (filePath == "" || !SD.exists(filePath) || !isTextFile(filePath)) { server.send(400, "text/plain", "Invalid file for editing"); return; }
  File file = SD.open(filePath, FILE_READ);
  if (!file) { server.send(500, "text/plain", "Could not open file"); return; }
  String content = file.readString();
  file.close();
  String html = R"rawliteral(<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Edit File</title><style>body{font-family:Arial; padding:20px;} textarea{width:100%; height:60vh; font-family:monospace;}</style></head><body><h2>✏️ Editing: )rawliteral" + filePath + R"rawliteral(</h2><form method='post' action='/sd/save'><input type='hidden' name='file' value=')rawliteral" + filePath + R"rawliteral('><textarea name='content'>)rawliteral" + content + R"rawliteral(</textarea><br><button type='submit'>💾 Save</button> <a href='/sd/browse'>Cancel</a></form></body></html>)rawliteral";
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleSdSaveFile() {
  String filePath = urlDecode(server.arg("file"));
  String content = server.arg("content");
  if (filePath == "" || content == "") { server.send(400, "text/plain", "Missing file or content"); return; }
  File file = SD.open(filePath, FILE_WRITE);
  if (!file) { server.send(500, "text/plain", "Could not open file for writing"); return; }
  file.print(content);
  file.close();
  server.send(200, "text/plain; charset=UTF-8", "OK");
}

// 🌟 [V3.4 新增] OTA 韌體更新處理函式
void handleFirmwareUpdate() {
  String html = R"rawliteral(
<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
<title>體更新</title>
<style>
body{font-family: Arial, sans-serif; text-align: center; padding: 40px; background-color: #f4f4f9;}
.container{background: #fff; padding: 30px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); max-width: 500px; margin: 0 auto;}
h2{color: #333;}
input[type="file"]{margin: 20px 0; padding: 10px; border: 1px solid #ccc; border-radius: 5px; width: 100%; box-sizing: border-box;}
button{padding: 12px 24px; font-size: 16px; font-weight: bold; color: white; background-color: #28a745; border: none; border-radius: 5px; cursor: pointer; width: 100%;}
button:hover{background-color: #218838;}
#progress{display:none; margin-top: 20px; text-align: left;}
#bar-container{width: 100%; background-color: #e9ecef; border-radius: 5px; overflow: hidden; height: 20px; margin-top: 10px;}
#bar{height: 100%; width: 0%; background-color: #007bff; transition: width 0.3s;}
.back-btn{display:inline-block; margin-top: 20px; text-decoration: none; color: #0056b3; font-weight: bold;}
</style></head><body>
<div class='container'>
<h2>🔄 ESP32 韌體更新 (OTA)</h2>
<p>請選擇 `.bin` 檔案進行上傳。更新過程中請勿關閉電源或斷開網路。</p>
<form id='uploadForm'>
  <input type='file' name='firmware' accept='.bin' required>
  <button type='submit' id='submitBtn'>開始更新</button>
</form>
<div id='progress'>
  <p id='statusText'>準備中...</p>
  <div id='bar-container'><div id='bar'></div></div>
</div>
<a href='/' class='back-btn'>⬅ 返回首頁</a>
</div>
<script>
document.getElementById('uploadForm').addEventListener('submit', function(e) {
  e.preventDefault();
  const formData = new FormData(this);
  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/firmware', true);
  document.getElementById('progress').style.display = 'block';
  document.getElementById('submitBtn').disabled = true;
  
  xhr.upload.onprogress = function(e) {
    if (e.lengthComputable) {
      const percent = (e.loaded / e.total) * 100;
      document.getElementById('bar').style.width = percent + '%';
      document.getElementById('statusText').innerText = '上傳與寫入中... ' + Math.round(percent) + '%';
    }
  };
  xhr.onload = function() {
    if (xhr.status === 200) {
      document.getElementById('statusText').innerText = '✅ 更新成功！設備正在重啟...';
      document.getElementById('bar').style.backgroundColor = '#28a745';
      setTimeout(() => { window.location.href = '/'; }, 4000);
    } else {
      document.getElementById('statusText').innerText = '❌ 更新失敗！請檢查檔案是否正確。';
      document.getElementById('bar').style.backgroundColor = '#dc3545';
      document.getElementById('submitBtn').disabled = false;
    }
  };
  xhr.send(formData);
});
</script></body></html>)rawliteral";
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleFirmwareUpload() {
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    Serial.printf("Update: %s\n", upload.filename.c_str());
    if(!Update.begin(UPDATE_SIZE_UNKNOWN)){
      Update.printError(Serial);
    }
  } else if(upload.status == UPLOAD_FILE_WRITE){
    if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
      Update.printError(Serial);
    }
  } else if(upload.status == UPLOAD_FILE_END){
    if(Update.end(true)){
      Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  }
}

// ==========================================
// [首頁與其他網頁介面]
// ==========================================
void handleRoot() {
  String welcomeMsg = "歡迎您使用生技治具控制台！";
  if (globalUser != "" && globalUser.length() > 0) {
    welcomeMsg = "歡迎 「" + globalUser + "」 使用生技治具控制台！";
  }
  // 頁尾資訊：IP + 連線模式（先算好，footer 只切一個點，降低拼接出錯機率）
  String footStr = (isAPMode ? WiFi.softAPIP() : WiFi.localIP()).toString()
                   + "　·　" + (isAPMode ? "AP 模式" : "STA 模式");

  // ── 拼接地圖（共 3 個切點，皆為「插入 C++ 變數」才切；純 HTML 不切）──
  //   段1 ... <span class='ver-pill'>   [切點A: +FIRMWARE_VERSION+ ]   </span></h2><div class='welcome-msg'>
  //   段2                                [切點B: +welcomeMsg+      ]   </div> ...所有按鈕... <b>
  //   段3                                [切點C: +footStr+         ]   </b></div></div></body></html>
  //  注意：ver-pill 的 <span> 開標籤在段1、</span> 閉標籤在段2，跨切點拼接後仍連續正確。
  String html = R"rawliteral(<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>JIG-8FT-P1 總控制台</title><style>
*{box-sizing:border-box}
body{margin:0;min-height:100vh;display:flex;align-items:flex-start;justify-content:center;padding:28px 14px;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","PingFang TC","Microsoft JhengHei",Roboto,Helvetica,Arial,sans-serif;color:#1f2933;background:radial-gradient(1100px 520px at 12% -8%,#e8f0fe 0%,rgba(232,240,254,0) 60%),radial-gradient(820px 460px at 100% 0%,#e6fcf5 0%,rgba(230,252,245,0) 55%),linear-gradient(180deg,#f6f7fb 0%,#eceef3 100%);}
.console{width:100%;max-width:430px;background:#fff;border:1px solid rgba(31,41,51,.06);border-radius:18px;padding:26px 20px 16px;box-shadow:0 12px 34px rgba(31,41,51,.10),0 2px 6px rgba(31,41,51,.06);}
h2{margin:0 0 2px;text-align:center;font-size:clamp(1.35rem,5vw,1.9rem);font-weight:800;letter-spacing:-.02em;line-height:1.2;color:#102a43;display:flex;align-items:center;justify-content:center;gap:9px;flex-wrap:wrap;}
.ver-pill{font-size:.7rem;font-weight:700;letter-spacing:.09em;text-transform:uppercase;color:#0b7285;background:#e6fcf5;border:1px solid #96f2d7;padding:3px 9px;border-radius:999px;}
.welcome-msg{text-align:center;font-size:.95rem;font-weight:500;color:#52606d;margin:8px 0 20px;}
.btn{position:relative;overflow:hidden;display:flex;align-items:center;justify-content:center;width:100%;margin:0 0 12px;padding:15px 18px;font-size:1.02rem;font-weight:600;color:#fff;background-color:#0056b3;border:none;border-radius:12px;cursor:pointer;text-decoration:none;box-shadow:0 3px 9px rgba(0,0,0,.13);transition:transform .16s ease,box-shadow .16s ease,background-color .16s ease,filter .16s ease;}
.btn:last-of-type{margin-bottom:0;}
.btn:hover{transform:translateY(-3px);box-shadow:0 9px 20px rgba(0,0,0,.19);filter:brightness(1.07);}
.btn:active{transform:translateY(-1px);box-shadow:0 4px 11px rgba(0,0,0,.16);}
.btn::after{content:"";position:absolute;top:0;left:-65%;width:45%;height:100%;background:linear-gradient(120deg,transparent,rgba(255,255,255,.38),transparent);transform:skewX(-20deg);transition:left .55s ease;pointer-events:none;}
.btn:hover::after{left:135%;}
.btn.alt{background-color:#28a745;}
.btn.alt2{background-color:#f39c12;}
.btn.sd{background-color:#8e44ad;}
.btn.ota{background-color:#6c757d;}
.btn.m031{background-color:#00897b;}
.foot{margin-top:18px;text-align:center;font-size:.78rem;color:#8993a4;letter-spacing:.02em;}
.foot b{color:#52606d;font-weight:600;}
</style></head><body><div class='console'><h2>⚙️ JIG-8FT-P1 控制台 <span class='ver-pill'>)rawliteral" + String(FIRMWARE_VERSION) + R"rawliteral(</span></h2><div class='welcome-msg'>)rawliteral" + welcomeMsg + R"rawliteral(</div><a href='/wifi' class='btn'>🌐 網路備援設定</a><a href='/monitor' class='btn alt'>⚡ 電壓電流設定</a><a href='/alarms' class='btn alt2'>⏰ 多工鬧鐘設定</a><a href='/sdcard' class='btn sd'>📂 SD 卡資料夾</a><a href='/firmware' class='btn ota'>🔄 ESP32 韌體更新</a><a href='/m031_ota' class='btn m031'>🛠️ M031 韌體更新</a><div class='foot'>設備在線　<b>)rawliteral" + footStr + R"rawliteral(</b></div></div></body></html>)rawliteral";
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleAlarmsSet() {
  String html = "<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>系統鬧鐘設定</title>";
  html += "<style>body{font-family:Arial; padding:20px; background:#f4f4f4;} .card{background:#fff; padding:15px; margin-bottom:15px; border-radius:8px; box-shadow:0 2px 4px rgba(0,0,0,0.1); display:flex; justify-content:space-between; align-items:center;} .back-btn{display:inline-block; margin-bottom:15px; text-decoration:none; color:#0056b3; font-weight:bold;} </style>";
  html += "<script>function saveAll() { let fd = new FormData(document.getElementById('aForm')); fetch('/api/saveAlarms', {method:'POST', body:new URLSearchParams(fd)}).then(()=>alert('✅ 鬧鐘已更新並成功同步至 M031 主板！')); } </script>";
  html += "</head><body><a href='/' class='back-btn'>⬅ 返回首頁</a><h2> 系統鬧鐘設定</h2><form id='aForm' onsubmit='event.preventDefault(); saveAll();'>";
  for (int i=0; i<6; i++) {
    char tbuf[16]; snprintf(tbuf, sizeof(tbuf), "%02d:%02d:%02d", alarms[i].h, alarms[i].m, alarms[i].s);
    String checked = alarms[i].en ? "checked" : "";
    html += "<div class='card'><div><h3 style='margin-top:0;'>鬧鐘 " + String(i+1) + "</h3>";
    html += "<input type='time' step='1' name='t" + String(i) + "' value='" + String(tbuf) + "' style='font-size:18px; padding:5px;'></div>";
    html += "<div><label style='font-size:18px; font-weight:bold;'><input type='checkbox' name='en" + String(i) + "' value='1' " + checked + " style='width:22px; height:22px; vertical-align:middle;'> 啟用開關</label></div></div>";
  }
  html += "<button type='submit' style='width:100%; padding:15px; font-size:18px; background:#4CAF50; color:white; border:none; border-radius:8px; cursor:pointer;'>💾 儲存並同步至設備</button></form></body></html>";
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleApiSaveAlarms() {
  for (int i=0; i<6; i++) {
    String t_val = server.arg("t" + String(i));
    int en = server.hasArg("en" + String(i)) ? 1 : 0;
    if (t_val.length() >= 5) {
      int h = t_val.substring(0, 2).toInt(); int m = t_val.substring(3, 5).toInt(); int s = (t_val.length() >= 8) ? t_val.substring(6, 8).toInt() : 0;
      alarms[i].h = h; alarms[i].m = m; alarms[i].s = s; alarms[i].en = en;
      char buf[32]; snprintf(buf, sizeof(buf), "%d,%02d,%02d,%02d,%d", i, h, m, s, en);
      sendToM031_JIG_8CP("AL", buf); delay(80);
    }
  }
  saveAlarmsConfig(); server.send(200, "text/plain; charset=UTF-8", "OK");
}
// ===== [Wi-Fi 設定頁重做] 掃描 API + JSON 轉義 =====
static String jsonEsc(String s) {
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  s.replace("\r", " ");
  s.replace("\n", " ");
  return s;
}

// 觸發非同步掃描（不阻塞 loop，對當前連線較溫和）
void handleApiWifiScan() {
  WiFi.scanNetworks(true);   // async = true
  server.send(200, "application/json", "{\"state\":\"scanning\"}");
}

// 輪詢掃描結果
void handleApiWifiScanResult() {
  int n = WiFi.scanComplete();
  if (n == -1) { server.send(200, "application/json", "{\"state\":\"scanning\"}"); return; }
  if (n <  0)  { server.send(200, "application/json", "{\"state\":\"idle\",\"count\":0,\"nets\":[]}"); return; }
  String j = "{\"state\":\"done\",\"count\":" + String(n) + ",\"nets\":[";
  bool first = true;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) continue;                 // 過濾隱藏/空 SSID
    if (!first) j += ","; first = false;
    int rssi = WiFi.RSSI(i);
    int enc  = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? 0 : 1;
    j += "{\"ssid\":\"" + jsonEsc(ssid) + "\",\"rssi\":" + String(rssi) + ",\"enc\":" + String(enc) + "}";
  }
  j += "]}";
  WiFi.scanDelete();
  server.send(200, "application/json", j);
}
void handleWiFiSet() {
  // ---- 組裝初始資料 JSON（User / 當前連線 / 5 備援槽）----
  String initJson = "{\"user\":\"" + jsonEsc(globalUser) + "\""
    ",\"cur\":{\"ssid\":\"" + jsonEsc(WiFi.SSID()) + "\","
              "\"rssi\":" + String((int)WiFi.RSSI()) + ","
              "\"ip\":\"" + (isAPMode ? WiFi.softAPIP() : WiFi.localIP()).toString() + "\","
              "\"ap\":" + (isAPMode ? "true" : "false") + "}"
    ",\"slots\":[";
  for (int i = 0; i < 5; i++) {
    if (i) initJson += ",";
    initJson += "{\"ssid\":\"" + jsonEsc(wifiList[i].ssid) + "\",\"pass\":\"" + jsonEsc(wifiList[i].pass) + "\"}";
  }
  initJson += "]}";

  String html = R"rawliteral(<!DOCTYPE html><html lang="zh-Hant"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0, viewport-fit=cover">
<title>Wi‑Fi 備援設定</title>
<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Sora:wght@400;600;700;800&family=Noto+Sans+TC:wght@400;500;700&family=JetBrains+Mono:wght@400;500;700&display=swap" rel="stylesheet">
<style>
:root{--bg:#0d131b;--panel:#141b24;--panel2:#19222d;--line:rgba(255,255,255,.07);--line2:rgba(255,255,255,.12);
--ink:#e8eef5;--dim:#93a1b2;--faint:#5d6b7c;--teal:#2dd4bf;--sky:#38bdf8;--danger:#f87171;
--disp:"Sora","Noto Sans TC",sans-serif;--body:"Noto Sans TC","Sora",system-ui,sans-serif;--mono:"JetBrains Mono",ui-monospace,monospace;}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
html{scroll-behavior:smooth}
body{font-family:var(--body);color:var(--ink);background:var(--bg);min-height:100vh;padding-bottom:120px;
background-image:radial-gradient(820px 460px at 88% -6%,rgba(45,212,191,.10),transparent 60%),radial-gradient(720px 420px at -8% 8%,rgba(56,189,248,.09),transparent 58%),linear-gradient(transparent 31px,rgba(255,255,255,.02) 32px),linear-gradient(90deg,transparent 31px,rgba(255,255,255,.02) 32px);background-size:auto,auto,32px 32px,32px 32px;}
.wrap{max-width:640px;margin:0 auto;padding:0 16px}
nav{position:sticky;top:0;z-index:30;display:flex;align-items:center;justify-content:space-between;
padding:14px 16px;backdrop-filter:blur(14px);background:rgba(13,19,27,.72);border-bottom:1px solid var(--line)}
nav .t{font-family:var(--disp);font-weight:700;font-size:1.05rem;letter-spacing:.01em}
.iconbtn{width:42px;height:42px;border-radius:50%;border:1px solid var(--line2);background:var(--panel);
color:var(--ink);display:grid;place-items:center;cursor:pointer;transition:.2s;text-decoration:none}
.iconbtn:hover{border-color:var(--teal);color:var(--teal);transform:translateY(-1px)}
.iconbtn.spin svg{animation:spin 1.1s linear infinite}
nav .r{display:flex;gap:8px;align-items:center}
.sec{margin-top:22px;opacity:0;transform:translateY(16px)}
.sec.in{animation:rise .6s ease forwards}
.sec-h{display:flex;align-items:baseline;justify-content:space-between;margin:0 2px 10px}
.sec-h h2{font-family:var(--disp);font-size:.82rem;font-weight:700;letter-spacing:.16em;text-transform:uppercase;color:var(--dim)}
.sec-h .hint{font-size:.72rem;color:var(--faint)}
.card{background:linear-gradient(180deg,var(--panel),var(--panel2));border:1px solid var(--line);border-radius:16px;padding:16px;position:relative;overflow:hidden}
/* 狀態卡 */
.status{display:flex;align-items:center;gap:14px}
.status .gly{flex:0 0 auto}
.status .mid{flex:1;min-width:0}
.status .ssid{font-family:var(--disp);font-weight:700;font-size:1.12rem;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.status .meta{display:flex;gap:8px;align-items:center;margin-top:4px;flex-wrap:wrap}
.badge{font-family:var(--mono);font-size:.66rem;letter-spacing:.08em;padding:3px 8px;border-radius:999px;border:1px solid var(--line2);color:var(--dim)}
.badge.ap{color:#fbbf24;border-color:rgba(251,191,36,.4)}
.badge.lvl{border:none;color:var(--ink)}
.status .ip{font-family:var(--mono);font-size:.82rem;color:var(--sky)}
.live{position:absolute;top:14px;right:16px;display:flex;align-items:center;gap:6px;font-size:.66rem;color:var(--dim);font-family:var(--mono)}
.live .dot{width:8px;height:8px;border-radius:50%;background:var(--teal);box-shadow:0 0 0 0 rgba(45,212,191,.6);animation:pulse 2s infinite}
.live.ap .dot{background:#fbbf24;box-shadow:0 0 0 0 rgba(251,191,36,.6)}
/* 表單元素 */
label.fld{display:block;font-size:.74rem;color:var(--dim);margin-bottom:7px;letter-spacing:.02em}
.inp{width:100%;background:#0c1219;border:1px solid var(--line2);border-radius:11px;color:var(--ink);
font-family:var(--body);font-size:.95rem;padding:12px 13px;transition:.18s;outline:none}
.inp:focus{border-color:var(--teal);box-shadow:0 0 0 3px rgba(45,212,191,.14)}
.inp.mono{font-family:var(--mono)}
.pw{position:relative}
.pw .inp{padding-right:44px}
.eye{position:absolute;right:8px;top:50%;transform:translateY(-50%);width:32px;height:32px;border:none;background:transparent;color:var(--faint);cursor:pointer;display:grid;place-items:center;border-radius:8px}
.eye:hover{color:var(--teal)}
.note{font-size:.72rem;color:var(--faint);margin-top:8px;line-height:1.5}
/* 備援槽 */
.slot{margin-bottom:12px;border:1px solid var(--line);border-radius:15px;background:var(--panel);overflow:hidden;transition:.2s}
.slot.active{border-color:var(--teal);box-shadow:0 0 0 1px var(--teal),0 10px 30px -16px rgba(45,212,191,.5)}
.slot.flash{animation:flash .7s ease}
.slot-head{display:flex;align-items:center;gap:10px;padding:12px 14px;cursor:pointer;user-select:none}
.slot-num{width:26px;height:26px;border-radius:8px;display:grid;place-items:center;font-family:var(--disp);font-weight:700;font-size:.82rem;
background:var(--panel2);border:1px solid var(--line2);color:var(--dim);flex:0 0 auto}
.slot.active .slot-num{background:var(--teal);color:#06231f;border-color:var(--teal)}
.slot-title{font-family:var(--disp);font-weight:600;font-size:.92rem;flex:1}
.slot-target{font-family:var(--mono);font-size:.62rem;letter-spacing:.08em;color:var(--teal);opacity:0;transition:.2s}
.slot.active .slot-target{opacity:1}
.slot-gly{flex:0 0 auto;display:flex;align-items:center;gap:6px}
.slot-gly .lt{font-size:.66rem;color:var(--faint);font-family:var(--mono)}
.slot-body{padding:0 14px 14px;display:grid;gap:10px}
/* 附近網路列表 */
.nets{background:var(--panel);border:1px solid var(--line);border-radius:16px;overflow:hidden}
.net{display:flex;align-items:center;gap:12px;padding:13px 15px;border-bottom:1px solid var(--line);cursor:pointer;transition:.16s}
.net:last-child{border-bottom:none}
.net:hover{background:linear-gradient(90deg,rgba(45,212,191,.08),transparent);transform:translateX(3px)}
.net .nm{flex:1;min-width:0;font-weight:500;font-size:.96rem;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;display:flex;align-items:center;gap:8px}
.net .lk{color:var(--faint);flex:0 0 auto}
.net .rg{display:flex;align-items:center;gap:9px;flex:0 0 auto}
.net .dbm{font-family:var(--mono);font-size:.72rem;color:var(--faint);min-width:42px;text-align:right}
.empty{padding:30px 16px;text-align:center;color:var(--faint);font-size:.85rem}
/* 雷達 */
.radar{position:relative;width:88px;height:88px;margin:24px auto}
.radar .ring{position:absolute;inset:0;border-radius:50%;border:1px solid rgba(45,212,191,.22);animation:rpulse 2.2s ease-out infinite}
.radar .ring:nth-child(2){animation-delay:.7s}.radar .ring:nth-child(3){animation-delay:1.4s}
.radar .sweep{position:absolute;inset:0;border-radius:50%;background:conic-gradient(from 0deg,rgba(45,212,191,.4),transparent 75deg);animation:spin 1.7s linear infinite;-webkit-mask:radial-gradient(circle,transparent 28%,#000 30%);mask:radial-gradient(circle,transparent 28%,#000 30%)}
.radar .core{position:absolute;left:50%;top:50%;width:8px;height:8px;border-radius:50%;background:var(--teal);transform:translate(-50%,-50%);box-shadow:0 0 12px var(--teal)}
.radar-txt{text-align:center;color:var(--dim);font-size:.8rem;font-family:var(--mono);letter-spacing:.06em}
/* 儲存列 */
.savebar{position:fixed;left:0;right:0;bottom:0;z-index:25;padding:14px 16px calc(14px + env(safe-area-inset-bottom));
background:linear-gradient(180deg,transparent,rgba(13,19,27,.92) 38%);backdrop-filter:blur(8px)}
.savebar .wrap{display:grid;gap:8px}
.savebtn{width:100%;border:none;border-radius:14px;padding:15px;font-family:var(--disp);font-weight:700;font-size:1rem;color:#06231f;
background:linear-gradient(120deg,var(--teal),var(--sky));cursor:pointer;position:relative;overflow:hidden;transition:.18s;letter-spacing:.02em}
.savebtn:hover{transform:translateY(-2px);box-shadow:0 14px 30px -12px rgba(45,212,191,.6)}
.savebtn:active{transform:translateY(0)}
.savebtn::after{content:"";position:absolute;top:0;left:-60%;width:40%;height:100%;background:linear-gradient(120deg,transparent,rgba(255,255,255,.5),transparent);transform:skewX(-20deg);transition:left .6s}
.savebtn:hover::after{left:130%}
.savebtn:disabled{opacity:.6;cursor:wait;transform:none}
.savewarn{font-size:.7rem;color:var(--faint);text-align:center;line-height:1.5}
/* toast */
#toast{position:fixed;left:50%;bottom:96px;transform:translate(-50%,20px);z-index:60;background:var(--panel2);border:1px solid var(--line2);
color:var(--ink);padding:11px 18px;border-radius:12px;font-size:.85rem;box-shadow:0 12px 30px -10px #000;opacity:0;pointer-events:none;transition:.3s;max-width:88vw;text-align:center}
#toast.show{opacity:1;transform:translate(-50%,0)}
#toast.ok{border-color:rgba(45,212,191,.5)}
#toast.err{border-color:rgba(248,113,113,.5)}
@keyframes spin{to{transform:rotate(360deg)}}
@keyframes rise{to{opacity:1;transform:none}}
@keyframes pulse{0%{box-shadow:0 0 0 0 rgba(45,212,191,.5)}70%{box-shadow:0 0 0 8px transparent}100%{box-shadow:0 0 0 0 transparent}}
@keyframes rpulse{0%{transform:scale(.4);opacity:.9}100%{transform:scale(1);opacity:0}}
@keyframes flash{0%{background:rgba(45,212,191,.22)}100%{background:var(--panel)}}
@media(prefers-reduced-motion:reduce){*{animation:none!important}}
</style></head><body>
<nav>
  <a class="iconbtn" href="/" aria-label="返回"><svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M15 18l-6-6 6-6"/></svg></a>
  <span class="t">Wi‑Fi 備援設定</span>
  <span class="r"><button class="iconbtn" id="scanBtn" aria-label="掃描"><svg width="19" height="19" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><path d="M5 12.5a10 10 0 0 1 14 0"/><path d="M8.5 16a5 5 0 0 1 7 0"/><circle cx="12" cy="19" r="1.2" fill="currentColor" stroke="none"/></svg></button></span>
</nav>
<div class="wrap">
  <section class="sec" id="secStatus"></section>
  <section class="sec" id="secUser"></section>
  <section class="sec"><div class="sec-h"><h2>備援網路</h2><span class="hint">開機依序自動嘗試 · 點選卡片設為填入目標</span></div><div id="slots"></div></section>
  <section class="sec"><div class="sec-h"><h2>附近網路</h2><span class="hint" id="netHint">—</span></div><div class="nets" id="nets"></div></section>
</div>
<div class="savebar"><div class="wrap">
  <button class="savebtn" id="saveBtn">儲存並重啟裝置</button>
  <div class="savewarn">儲存後裝置將重啟，約 10 秒後以新設定重新上線；若新設定無法連線，將回落 AP 熱點 <b>JIG_8FT_P1_WIFIset</b>。</div>
</div></div>
<div id="toast"></div>
<script>const INIT=)rawliteral" + initJson + R"rawliteral(;
const LCOL=['#64748b','#f87171','#f59e0b','#84cc16','#22c55e'];
const LTXT=['—','微弱','普通','良好','極佳'];
function lvl(r){if(r==null)return 0;if(r>=-55)return 4;if(r>=-67)return 3;if(r>=-78)return 2;return 1;}
function glyph(rssi,px){px=px||24;const n=lvl(rssi),col=LCOL[n],k=0.7071,cx=12,cy=18,rs=[4,7,10,13];let a='';
 for(let i=0;i<4;i++){const r=rs[i],lx=(cx-k*r).toFixed(2),ly=(cy-k*r).toFixed(2),rx=(cx+k*r).toFixed(2),ry=(cy-k*r).toFixed(2);
 const on=n>0&&i<n,c=on?col:'rgba(148,163,184,.22)';a+='<path d="M'+lx+' '+ly+' A '+r+' '+r+' 0 0 1 '+rx+' '+ry+'" fill="none" stroke="'+c+'" stroke-width="2.1" stroke-linecap="round"/>';}
 const d=n>0?col:'rgba(148,163,184,.35)';
 return '<svg width="'+px+'" height="'+px+'" viewBox="0 0 24 24" aria-label="'+LTXT[n]+'">'+a+'<circle cx="'+cx+'" cy="'+cy+'" r="1.7" fill="'+d+'"/></svg>';}
const lock='<svg class="lk" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="4" y="11" width="16" height="9" rx="2"/><path d="M8 11V8a4 4 0 0 1 8 0v3"/></svg>';
const eyeOpen='<svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M2 12s3.5-7 10-7 10 7 10 7-3.5 7-10 7-10-7-10-7z"/><circle cx="12" cy="12" r="3"/></svg>';
const eyeOff='<svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M3 3l18 18"/><path d="M10.6 5.1A10.9 10.9 0 0 1 12 5c6.5 0 10 7 10 7a18 18 0 0 1-3.2 4M6.6 6.6A18 18 0 0 0 2 12s3.5 7 10 7a10.9 10.9 0 0 0 4-.8"/></svg>';
let slots=INIT.slots.map(s=>({ssid:s.ssid,pass:s.pass}));
let active=0,lastNets=null,scanning=false;
(function(){for(let i=0;i<5;i++){if(!slots[i].ssid){active=i;break;}}})();
const $=s=>document.querySelector(s);
function esc(s){return (s||'').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));}
function rssiFor(ssid){if(!ssid||!lastNets)return null;const m=lastNets.find(n=>n.ssid===ssid);return m?m.rssi:null;}
let toastT;function toast(msg,kind){const t=$('#toast');t.textContent=msg;t.className='show '+(kind||'');clearTimeout(toastT);toastT=setTimeout(()=>t.className='',2600);}
function renderStatus(){const c=INIT.cur,ap=!!c.ap,n=ap?0:lvl(c.rssi);
 $('#secStatus').innerHTML='<div class="card"><div class="live '+(ap?'ap':'')+'"><span class="dot"></span>'+(ap?'AP 熱點':'ONLINE')+'</div>'+
 '<div class="status"><span class="gly">'+glyph(ap?null:c.rssi,40)+'</span><div class="mid">'+
 '<div class="ssid">'+(ap?'JIG_8FT_P1_WIFIset':esc(c.ssid||'（未連線）'))+'</div>'+
 '<div class="meta"><span class="badge '+(ap?'ap':'')+'">'+(ap?'AP MODE':'STA')+'</span>'+
 (ap?'':('<span class="badge lvl" style="color:'+LCOL[n]+'">'+LTXT[n]+' · '+c.rssi+' dBm</span>'))+
 '<span class="ip">'+esc(c.ip)+'</span></div></div></div></div>';}
function renderUser(){$('#secUser').innerHTML='<div class="card"><label class="fld">全域使用者 (User)</label>'+
 '<input class="inp" id="userInp" value="'+esc(INIT.user)+'" placeholder="例如：BALLY">'+
 '<div class="note">顯示於裝置歡迎訊息與測試紀錄，留空則顯示預設問候語。</div></div>';}
function renderSlots(){const wrap=$('#slots');wrap.innerHTML='';
 slots.forEach((s,i)=>{const r=rssiFor(s.ssid),n=lvl(r);
  const el=document.createElement('div');el.className='slot'+(i===active?' active':'');el.dataset.i=i;
  el.innerHTML='<div class="slot-head" data-head="'+i+'"><span class="slot-num">'+(i+1)+'</span>'+
   '<span class="slot-title">備援 '+(i+1)+'</span><span class="slot-target">● 填入目標</span>'+
   '<span class="slot-gly">'+glyph(r,22)+'<span class="lt">'+(s.ssid?(r!=null?LTXT[n]:'未掃描'):'空')+'</span></span></div>'+
   '<div class="slot-body"><div><label class="fld">網路名稱 (SSID)</label><input class="inp mono" data-ssid="'+i+'" value="'+esc(s.ssid)+'" placeholder="手動輸入或從下方選取"></div>'+
   '<div><label class="fld">密碼 (Password)</label><div class="pw"><input class="inp mono" type="password" data-pass="'+i+'" value="'+esc(s.pass)+'" placeholder="開放網路可留空"><button class="eye" data-eye="'+i+'" type="button">'+eyeOff+'</button></div></div></div>';
  wrap.appendChild(el);});
 wrap.querySelectorAll('[data-head]').forEach(h=>h.onclick=()=>{active=+h.dataset.head;renderSlots();});
 wrap.querySelectorAll('[data-ssid]').forEach(inp=>inp.oninput=e=>{slots[+e.target.dataset.ssid].ssid=e.target.value;refreshSlotGly(+e.target.dataset.ssid);});
 wrap.querySelectorAll('[data-pass]').forEach(inp=>inp.oninput=e=>{slots[+e.target.dataset.pass].pass=e.target.value;});
 wrap.querySelectorAll('[data-eye]').forEach(b=>b.onclick=()=>{const inp=wrap.querySelector('[data-pass="'+b.dataset.eye+'"]');const show=inp.type==='password';inp.type=show?'text':'password';b.innerHTML=show?eyeOpen:eyeOff;});}
function refreshSlotGly(i){const el=$('#slots').children[i];if(!el)return;const r=rssiFor(slots[i].ssid),n=lvl(r);
 el.querySelector('.slot-gly').innerHTML=glyph(r,22)+'<span class="lt">'+(slots[i].ssid?(r!=null?LTXT[n]:'未掃描'):'空')+'</span>';}
function renderNets(){const box=$('#nets');
 if(scanning){box.innerHTML='<div class="empty"><div class="radar"><div class="ring"></div><div class="ring"></div><div class="ring"></div><div class="sweep"></div><div class="core"></div></div><div class="radar-txt">掃描附近網路…</div></div>';$('#netHint').textContent='';return;}
 if(!lastNets){box.innerHTML='<div class="empty">點右上角雷達按鈕掃描附近網路</div>';$('#netHint').textContent='';return;}
 if(!lastNets.length){box.innerHTML='<div class="empty">未掃描到任何網路</div>';$('#netHint').textContent='0 個';return;}
 $('#netHint').textContent=lastNets.length+' 個 · 由強到弱';
 box.innerHTML='';lastNets.forEach((nt,idx)=>{const row=document.createElement('div');row.className='net';row.dataset.idx=idx;
  row.innerHTML='<span class="nm">'+esc(nt.ssid)+(nt.enc?lock:'')+'</span><span class="rg">'+glyph(nt.rssi,22)+'<span class="dbm" title="'+nt.rssi+' dBm">'+nt.rssi+'</span></span>';
  row.onclick=()=>pickNet(nt);box.appendChild(row);});}
function pickNet(nt){slots[active].ssid=nt.ssid;renderSlots();
 const pe=$('#slots').querySelector('[data-pass="'+active+'"]');if(pe){pe.focus();pe.type='text';const eb=$('#slots').querySelector('[data-eye="'+active+'"]');if(eb)eb.innerHTML=eyeOpen;}
 const card=$('#slots').children[active];card.classList.remove('flash');void card.offsetWidth;card.classList.add('flash');
 toast('已填入「'+nt.ssid+'」→ 備援 '+(active+1),'ok');}
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
async function doScan(){if(scanning)return;scanning=true;$('#scanBtn').classList.add('spin');renderNets();
 try{await fetch('/api/wifi/scan',{cache:'no-store'});}catch(e){}
 const t0=Date.now();let nets=null;
 while(Date.now()-t0<18000){await sleep(1200);try{const r=await fetch('/api/wifi/scan_result',{cache:'no-store'});const j=await r.json();
   if(j.state==='done'){nets=j.nets;break;}if(j.state==='idle')break;}catch(e){}}
 scanning=false;$('#scanBtn').classList.remove('spin');
 if(nets){nets.sort((a,b)=>b.rssi-a.rssi);lastNets=nets;renderNets();renderSlots();toast('掃描完成，找到 '+nets.length+' 個網路','ok');}
 else{renderNets();toast('掃描逾時，請重試','err');}}
async function doSave(){const btn=$('#saveBtn');btn.disabled=true;btn.textContent='儲存中…';
 const sp=new URLSearchParams();sp.append('globalUser',$('#userInp').value);
 for(let i=0;i<5;i++){sp.append('ssid'+(i+1),slots[i].ssid);sp.append('pass'+(i+1),slots[i].pass);}
 try{await fetch('/save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:sp});}catch(e){}
 btn.textContent='已送出 · 裝置重啟中';toast('設定已儲存，裝置正在重啟…','ok');
 setTimeout(()=>{btn.textContent='請重新整理頁面';btn.disabled=false;},4000);}
document.addEventListener('DOMContentLoaded',()=>{renderStatus();renderUser();renderSlots();renderNets();
 $('#scanBtn').onclick=doScan;$('#saveBtn').onclick=doSave;
 const io=new IntersectionObserver(es=>es.forEach(e=>{if(e.isIntersecting){e.target.classList.add('in');io.unobserve(e.target);}}),{threshold:.12});
 document.querySelectorAll('.sec').forEach((s,i)=>{s.style.animationDelay=(i*.07)+'s';io.observe(s);});
 setTimeout(doScan,500);});
</script></body></html>)rawliteral";
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleSaveWiFi() {
  if (server.hasArg("globalUser")) globalUser = server.arg("globalUser");
  for (int i = 0; i < 5; i++) { if (server.hasArg("ssid"+String(i+1))) wifiList[i].ssid = server.arg("ssid"+String(i+1)); if (server.hasArg("pass"+String(i+1))) wifiList[i].pass = server.arg("pass"+String(i+1)); }
  saveWiFiConfig(); server.send(200, "text/html", "<meta charset='UTF-8'><h2>✅ 設定已儲存！ESP32 將重新啟動...</h2>"); delay(1000); ESP.restart();
}

void handleMonitor() {
  // ★ [V4.3 新增] 請求 M031 進入 Power Monitor
  // M031 端會自行判斷 g_u8InPowerMonitor：
  //   已在 PM → 靜默忽略，不中斷 PD 發送
  //   不在 PM → 設定旗標，在下一個安全點自動進入
  sendToM031_JIG_8CP("PM", "ENTER");

  String html = R"rawliteral(
<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>
<title>電壓電流即時分析儀 )rawliteral" + String(FIRMWARE_VERSION) + R"rawliteral(</title>
<style>
body{font-family: Arial, sans-serif; background: #1e1e1e; color: #fff; margin: 0; padding: 10px; text-align: center;}
.back-btn{display: block; text-align: left; color: #00c3ff; text-decoration: none; margin-bottom: 10px; font-weight: bold;}
.dashboard {display: flex; flex-wrap: wrap; justify-content: center; gap: 15px;}
.panel {background: #2d2d2d; border-radius: 10px; padding: 15px; width: 100%; max-width: 440px; box-sizing:border-box;}
.section-box { border-radius: 8px; padding: 12px; margin-bottom: 15px; border: 1px solid; }
.current-box { background: rgba(255, 50, 50, 0.08); border-color: rgba(255, 50, 50, 0.4); }
.voltage-box { background: rgba(50, 255, 50, 0.08); border-color: rgba(50, 255, 50, 0.4); }
.section-title { font-size: 14px; font-weight: bold; margin-bottom: 10px; text-align: left; padding-left: 5px; }
.title-current { color: #ff4d4d; } .title-voltage { color: #4dff4d; }
.val-text {font-size: 32px; font-weight: bold; color: #00ff66; text-shadow: 0 0 10px rgba(0,255,102,0.3);}
.panel-chart {background: #2d2d2d; border-radius: 10px; padding: 15px; width: 100%; max-width: 600px; box-sizing:border-box;}
.integ-box {background:#1a1a1a; border-radius:6px; padding:10px; margin:10px 0; display:flex; justify-content:space-around;}
.integ-val {font-size:16px; color:#ffcc00; font-weight:bold;}
.config-grid {display: grid; grid-template-columns: 1fr 1fr; gap: 10px; text-align: left;}
.config-grid div {display: flex; flex-direction: column; font-size:12px; color:#ccc;}
input {width: 100%; padding: 6px; margin-top:4px; box-sizing: border-box; border-radius:4px; border:1px solid #555; background:#111; color:#fff; font-size: 14px;}
button {padding: 10px; font-size: 14px; font-weight: bold; border: none; border-radius: 5px; cursor: pointer; color:#fff; width:100%; margin-top:8px;}
.btn-save {background: #17a2b8;} .btn-rec {background: #ff8800; font-size:16px;} .btn-recording {background: #cc0000; animation: blink 1s infinite;}
button.power {font-size:18px;} .power-off {background: #d9534f;} .power-on {background: #28a745;}
canvas {background: #111; border-radius: 5px; margin-top:10px; width: 100%; height: 280px;}
.modal {display:none; position:fixed; top:50%; left:50%; transform:translate(-50%,-50%); background:#e74c3c; padding:25px; border-radius:10px; z-index:999; width:80%; max-width:350px; box-shadow: 0 0 20px rgba(0,0,0,0.8);}
.modal-overlay {display:none; position:fixed; top:0; left:0; width:100%; height:100%; background:rgba(0,0,0,0.6); z-index:998;}
@keyframes blink { 0% {opacity:1;} 50% {opacity:0.5;} 100% {opacity:1;} }
</style></head><body>
<a href='/' class='back-btn'>⬅ 返回首頁</a>
<h2>⚡ JIG-8FT-P1 即時能耗分析儀 )rawliteral" + String(FIRMWARE_VERSION) + R"rawliteral(</h2>
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
   <div style="margin-top: 15px; text-align: center;">
     <button id="downloadBtn" class="btn-rec" onclick="downloadCSV()" style="display: none; background: #17a2b8;">⬇️ 手動下載測試報告</button>
   </div>
   <div class="section-box current-box">
     <div class="section-title title-current">🔋 電流保護與刻度設定 (mA)</div>
     <div class="config-grid">
        <div><label>電流警報上限</label><input type="number" id="limitScale" value=")rawliteral" + String(conf_limitScale) + R"rawliteral(" step="1"></div>
        <div><label>容許超載時間 (秒)</label><input type="number" id="dur" value=")rawliteral" + String(conf_limitDuration) + R"rawliteral(" step="1"></div>
        <div><label>電流刻度最大值</label><input type="number" id="maxScale" value=")rawliteral" + String(conf_maxScale) + R"rawliteral(" step="10"></div>
        <div><label>電流刻度最小值</label><input type="number" id="minScale" value=")rawliteral" + String(conf_minScale) + R"rawliteral(" step="10"></div>
     </div>
   </div>
   <div class="section-box voltage-box">
     <div class="section-title title-voltage">⚡ 電壓保護與刻度設定 (V)</div>
     <div class="config-grid">
        <div><label>電壓刻度最大值</label><input type="number" id="vMaxScale" value=")rawliteral" + String(conf_vMaxScale) + R"rawliteral(" step="0.5"></div>
        <div><label>電壓刻度最小值</label><input type="number" id="vMinScale" value=")rawliteral" + String(conf_vMinScale) + R"rawliteral(" step="0.5"></div>
        <div><label>電壓最高限制</label><input type="number" id="maxVol" value=")rawliteral" + String(conf_maxVoltage) + R"rawliteral(" step="0.1"></div>
        <div><label>電壓最低限制</label><input type="number" id="minVol" value=")rawliteral" + String(conf_minVoltage) + R"rawliteral(" step="0.1"></div>
     </div>
   </div>
   <button class="btn-save" onclick="saveConfig()">💾 儲存保護參數與圖表刻度至 SD 卡</button>
 </div>
 <div class="panel-chart"><h3 style="color:#00ff66; margin:0 0 10px 0; text-align:left;">🔋 電流即時折線圖 (mA)</h3><canvas id="lineChart"></canvas></div>
 <div class="panel-chart"><h3 style="color:#00c3ff; margin:0 0 10px 0; text-align:left;">⚡ 電壓即時折線圖 (V)</h3><canvas id="voltageChart"></canvas></div>
</div>
<div class="modal-overlay" id="modalOverlay"></div>
<div class="modal" id="warnModal"><h2 style="margin:0 0 10px 0;">⚠ 安全防禦介入斷電</h2><p id="warnMsg" style="font-size:16px; line-height:1.4;"></p><button style="background:#fff; color:#e74c3c; padding:8px; margin-top:10px;" onclick="closeModal()">了解</button></div>
<script>
const cvs = document.getElementById('lineChart'); const ctx = cvs.getContext('2d');
const vCvs = document.getElementById('voltageChart'); const vCtx = vCvs.getContext('2d');
const maxPoints = 200; let historyData = new Array(maxPoints).fill(0); let historyVoltageData = new Array(maxPoints).fill(0);
let ws; let localRecording = false; let localFilename = ""; let JS_PowerState = "OFF"; let lastDisplayedWarning = "";
function initWebSocket() { ws = new WebSocket('ws://' + window.location.hostname + ':81/'); ws.onmessage = function(event) { updateUI(JSON.parse(event.data)); }; ws.onclose = function() { document.getElementById('pwrBtn').innerText = "❌ 網路中斷，重連中..."; setTimeout(initWebSocket, 2000); }; }
initWebSocket();
function updateUI(data) {
  JS_PowerState = data.power; 
  document.getElementById('curText').innerText = data.mA.toFixed(1) + ' mA';
  document.getElementById('volText').innerText = data.v.toFixed(2) + ' V';
  document.getElementById('mAhText').innerText = data.mAh.toFixed(4) + ' mAh';
  document.getElementById('mWhText').innerText = data.mWh.toFixed(4) + ' mWh';
  const btn = document.getElementById('pwrBtn');
  if(data.power === "ON") { btn.className = "power power-on"; btn.innerText = "🔌 治具通電中 (點擊斷電)"; } 
  else { btn.className = "power power-off"; btn.innerText = " 治具斷電中 (點擊通電)"; }
  const recBtn = document.getElementById('recBtn');
  if(data.logging) { recBtn.className = "btn-rec btn-recording"; recBtn.innerText = "⏸ 錄製中: " + localFilename + " (點擊結束)"; localRecording = true; document.getElementById('downloadBtn').style.display = "none"; } 
  else { recBtn.className = "btn-rec"; recBtn.innerText = "🔴 開始錄製測試報告 (.CSV)"; localRecording = false; }
  if (data.warning !== "") { if (data.warning !== lastDisplayedWarning) { document.getElementById("warnMsg").innerText = data.warning; document.getElementById("warnModal").style.display = "block"; document.getElementById("modalOverlay").style.display = "block"; lastDisplayedWarning = data.warning; } } else { lastDisplayedWarning = ""; }
  historyData.push(data.power === "ON" ? data.mA : 0); if(historyData.length > maxPoints) historyData.shift();
  historyVoltageData.push(data.power === "ON" ? data.v : 0); if(historyVoltageData.length > maxPoints) historyVoltageData.shift();
  drawChart(); drawVoltageChart();
}
function toggleRecording() {
  if(!localRecording) { let name = prompt("請輸入測試報告 CSV 檔名: ", "JIG_TEST_" + Date.now() + ".csv"); if(name === null || name.trim() === "") return; localFilename = name.trim(); ws.send("REC_START:" + localFilename); document.getElementById('downloadBtn').style.display = "none"; } 
  else { ws.send("REC_STOP"); document.getElementById('recBtn').innerText = "✅ 錄製已停止，點擊按鈕手動下載"; document.getElementById('recBtn').className = "btn-rec"; document.getElementById('downloadBtn').style.display = "block"; }
}
function downloadCSV() { if (localFilename) { window.location.href = "/api/downloadCSV?file=" + localFilename; } }
function drawChart() {
  cvs.width = cvs.clientWidth; cvs.height = cvs.clientHeight; const w = cvs.width, h = cvs.clientHeight; ctx.clearRect(0, 0, w, h);
  const maxScale = parseFloat(document.getElementById('maxScale').value) || 500; const minScale = parseFloat(document.getElementById('minScale').value) || 0; const limitVal = parseFloat(document.getElementById('limitScale').value) || 400; const range = maxScale - minScale;
  ctx.strokeStyle = '#383838'; ctx.fillStyle = '#888'; ctx.font = '10px Arial'; ctx.beginPath();
  for(let i=0; i<=5; i++) { let y = Math.floor(i * (h/5)); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.fillText((maxScale - (i * (range / 5))).toFixed(0), 5, y + 12); }
  for(let i=0; i<=10; i++) { let x = Math.floor(i * (w/10)); ctx.moveTo(x, 0); ctx.lineTo(x, h); } ctx.stroke();
  const limitY = h - ((limitVal - minScale) / range) * h;
  if (limitY > 0 && limitY < h) { ctx.beginPath(); ctx.setLineDash([5, 5]); ctx.moveTo(0, limitY); ctx.lineTo(w, limitY); ctx.strokeStyle = '#ff3333'; ctx.stroke(); ctx.setLineDash([]); ctx.fillStyle = '#ff3333'; ctx.font = '11px Arial'; ctx.textAlign = 'right'; ctx.textBaseline = 'bottom'; ctx.fillText('Limit: ' + limitVal.toFixed(0) + ' mA', w - 5, limitY - 2); }
  ctx.beginPath(); ctx.strokeStyle = '#00ff66'; ctx.lineWidth = 2; const xStep = w / (maxPoints - 1);
  for(let i=0; i<historyData.length; i++) { let x = i * xStep; let y = h - ((historyData[i] - minScale) / range) * h; if(i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y); } ctx.stroke();
  let cMax = -Infinity, cMin = Infinity, cSum = 0; for(let i=0; i<historyData.length; i++) { let val = historyData[i]; if(val > cMax) cMax = val; if(val < cMin) cMin = val; cSum += val; }
  let cAvg = historyData.length > 0 ? cSum / historyData.length : 0; let cPP = cMax - cMin; if(cMax === -Infinity) cMax = 0; if(cMin === Infinity) cMin = 0;
  let boxX = 10, boxY = 10, boxW = 135, boxH = 75; ctx.fillStyle = 'rgba(0, 0, 0, 0.65)'; ctx.fillRect(boxX, boxY, boxW, boxH); ctx.strokeStyle = 'rgba(0, 255, 102, 0.3)'; ctx.lineWidth = 1; ctx.strokeRect(boxX, boxY, boxW, boxH);
  ctx.fillStyle = '#00ff66'; ctx.font = '12px Arial'; ctx.textAlign = 'left'; ctx.textBaseline = 'top';
  ctx.fillText('Max: ' + cMax.toFixed(1) + ' mA', boxX + 8, boxY + 8); ctx.fillText('Min: ' + cMin.toFixed(1) + ' mA', boxX + 8, boxY + 24); ctx.fillText('Avg: ' + cAvg.toFixed(1) + ' mA', boxX + 8, boxY + 40); ctx.fillText('P-P: ' + cPP.toFixed(1) + ' mA', boxX + 8, boxY + 56);
  const axisY = h - 10; ctx.beginPath(); ctx.moveTo(0, axisY); ctx.lineTo(w, axisY); ctx.strokeStyle = '#aaa'; ctx.stroke();
  ctx.fillStyle = '#aaa'; ctx.font = '12px Arial'; ctx.textAlign = 'right'; ctx.textBaseline = 'bottom'; ctx.fillText('100 ms/格', w - 5, h - 5);
}
function drawVoltageChart() {
  vCvs.width = vCvs.clientWidth; vCvs.height = vCvs.clientHeight; const w = vCvs.width, h = vCvs.clientHeight; vCtx.clearRect(0, 0, w, h);
  const vMaxScale = parseFloat(document.getElementById('vMaxScale').value) || 15; const vMinScale = parseFloat(document.getElementById('vMinScale').value) || 0; const minVol = parseFloat(document.getElementById('minVol').value) || 0; const maxVol = parseFloat(document.getElementById('maxVol').value) || 12; const vRange = vMaxScale - vMinScale;
  vCtx.strokeStyle = '#383838'; vCtx.fillStyle = '#888'; vCtx.font = '10px Arial'; vCtx.beginPath();
  for(let i=0; i<=5; i++) { let y = Math.floor(i * (h/5)); vCtx.moveTo(0, y); vCtx.lineTo(w, y); vCtx.fillText((vMaxScale - (i * (vRange / 5))).toFixed(2), 5, y + 12); }
  for(let i=0; i<=10; i++) { let x = Math.floor(i * (w/10)); vCtx.moveTo(x, 0); vCtx.lineTo(x, h); } vCtx.stroke();
  const maxVolY = h - ((maxVol - vMinScale) / vRange) * h;
  if (maxVolY > 0 && maxVolY < h) { vCtx.beginPath(); vCtx.setLineDash([5, 5]); vCtx.moveTo(0, maxVolY); vCtx.lineTo(w, maxVolY); vCtx.strokeStyle = '#ff3333'; vCtx.stroke(); vCtx.setLineDash([]); vCtx.fillStyle = '#ff3333'; vCtx.font = '11px Arial'; vCtx.textAlign = 'right'; vCtx.textBaseline = 'bottom'; vCtx.fillText('Max Limit: ' + maxVol.toFixed(2) + 'V', w - 5, maxVolY - 2); }
  const minVolY = h - ((minVol - vMinScale) / vRange) * h;
  if (minVolY > 0 && minVolY < h) { vCtx.beginPath(); vCtx.setLineDash([5, 5]); vCtx.moveTo(0, minVolY); vCtx.lineTo(w, minVolY); vCtx.strokeStyle = '#ff3333'; vCtx.stroke(); vCtx.setLineDash([]); vCtx.fillStyle = '#ff3333'; vCtx.font = '11px Arial'; vCtx.textAlign = 'right'; vCtx.textBaseline = 'top'; vCtx.fillText('Min Limit: ' + minVol.toFixed(2) + 'V', w - 5, minVolY + 2); }
  vCtx.beginPath(); vCtx.strokeStyle = '#00c3ff'; vCtx.lineWidth = 2; const xStep = w / (maxPoints - 1);
  for(let i=0; i<historyVoltageData.length; i++) { let x = i * xStep; let y = h - ((historyVoltageData[i] - vMinScale) / vRange) * h; if(i === 0) vCtx.moveTo(x, y); else vCtx.lineTo(x, y); } vCtx.stroke();
  let vMax = -Infinity, vMin = Infinity, vSum = 0; for(let i=0; i<historyVoltageData.length; i++) { let val = historyVoltageData[i]; if(val > vMax) vMax = val; if(val < vMin) vMin = val; vSum += val; }
  let vAvg = historyVoltageData.length > 0 ? vSum / historyVoltageData.length : 0; let vPP = vMax - vMin; if(vMax === -Infinity) vMax = 0; if(vMin === Infinity) vMin = 0;
  let boxX = 10, boxY = 10, boxW = 125, boxH = 75; vCtx.fillStyle = 'rgba(0, 0, 0, 0.65)'; vCtx.fillRect(boxX, boxY, boxW, boxH); vCtx.strokeStyle = 'rgba(0, 195, 255, 0.3)'; vCtx.lineWidth = 1; vCtx.strokeRect(boxX, boxY, boxW, boxH);
  vCtx.fillStyle = '#00c3ff'; vCtx.font = '12px Arial'; vCtx.textAlign = 'left'; vCtx.textBaseline = 'top';
  vCtx.fillText('Max: ' + vMax.toFixed(2) + ' V', boxX + 8, boxY + 8); vCtx.fillText('Min: ' + vMin.toFixed(2) + ' V', boxX + 8, boxY + 24); vCtx.fillText('Avg: ' + vAvg.toFixed(2) + ' V', boxX + 8, boxY + 40); vCtx.fillText('P-P: ' + vPP.toFixed(2) + ' V', boxX + 8, boxY + 56);
  const axisY = h - 10; vCtx.beginPath(); vCtx.moveTo(0, axisY); vCtx.lineTo(w, axisY); vCtx.strokeStyle = '#aaa'; vCtx.stroke();
  vCtx.fillStyle = '#aaa'; vCtx.font = '12px Arial'; vCtx.textAlign = 'right'; vCtx.textBaseline = 'bottom'; vCtx.fillText('100 ms/格', w - 5, h - 5);
}
function togglePower() { const newState = (JS_PowerState === "OFF") ? "ON" : "OFF"; fetch('/api/power', { method: 'POST', headers: {'Content-Type': 'application/x-www-form-urlencoded'}, body: 'state=' + newState }); closeModal(); }
function saveConfig() { const params = new URLSearchParams({ maxScale: document.getElementById('maxScale').value, minScale: document.getElementById('minScale').value, limitScale: document.getElementById('limitScale').value, minVol: document.getElementById('minVol').value, maxVol: document.getElementById('maxVol').value, dur: document.getElementById('dur').value, vMaxScale: document.getElementById('vMaxScale').value, vMinScale: document.getElementById('vMinScale').value }); fetch('/api/saveConfig', { method: 'POST', body: params }).then(() => alert("💾 安全配置與圖表刻度已寫入 SD 卡保存！")); }
function closeModal() { document.getElementById("warnModal").style.display = "none"; document.getElementById("modalOverlay").style.display = "none"; }
</script></body></html>)rawliteral";
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleApiData() {
  String json = "{\"mA\":" + String(current_mA) + ",\"v\":" + String(current_V) + ",\"power\":\"" + power_state + "\",\"warning\":\"" + systemWarning + "\",\"mAh\":" + String(cumulative_mAh, 4) + ",\"mWh\":" + String(cumulative_mWh, 4) + ",\"logging\":" + (isRecordingCSV ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void handleApiPower() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "ON") { sendToM031_JIG_8CP("PW", "ON"); power_state = "ON"; systemWarning = ""; cumulative_mAh = 0.0; cumulative_mWh = 0.0; overCurrentStartTime = 0; lastPdTime = millis(); powerOnTime = millis(); } 
    else { sendToM031_JIG_8CP("PW", "OFF"); power_state = "OFF"; systemWarning = ""; if (isRecordingCSV) { csvFile.close(); isRecordingCSV = false; } }
    broadcastWebSocket(); server.send(200, "text/plain; charset=UTF-8", "OK");
  }
}

void handleApiSaveConfig() {
  if (server.hasArg("maxScale")) conf_maxScale = server.arg("maxScale").toFloat();
  if (server.hasArg("minScale")) conf_minScale = server.arg("minScale").toFloat();
  if (server.hasArg("limitScale")) conf_limitScale = server.arg("limitScale").toFloat();
  if (server.hasArg("minVol")) conf_minVoltage = server.arg("minVol").toFloat();
  if (server.hasArg("maxVol")) conf_maxVoltage = server.arg("maxVol").toFloat();
  if (server.hasArg("dur")) conf_limitDuration = server.arg("dur").toInt();
  if (server.hasArg("vMaxScale")) conf_vMaxScale = server.arg("vMaxScale").toFloat();
  if (server.hasArg("vMinScale")) conf_vMinScale = server.arg("vMinScale").toFloat();
  savePowerConfig(); sendConfigToM031(); server.send(200, "text/plain; charset=UTF-8", "OK");
}

void handleDownloadCSV() {
  if (server.hasArg("file")) { String path = "/logs/" + server.arg("file"); if (SD.exists(path)) { File f = SD.open(path, FILE_READ); server.streamFile(f, "text/csv"); f.close(); return; } }
  server.send(404, "text/plain", "Log File Not Found");
}

void handleNotFound() {
  if (isAPMode) { server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true); server.send(302, "text/plain", ""); }
  else { server.send(404, "text/plain", "Not Found"); }
}

// ==========================================
// [通訊與網路核心]
// ==========================================
void setLedState(LedState state) { currentLedState = state; failBlinkCount = 0; ledStateHigh = false; digitalWrite(WIFI_LED_PIN, LOW); previousLedMillis = millis(); }
void updateLED() { unsigned long cm = millis(); if (currentLedState == LED_ON) { digitalWrite(WIFI_LED_PIN, HIGH); } else if (currentLedState == LED_OFF) { digitalWrite(WIFI_LED_PIN, LOW); } else if (currentLedState == LED_BLINK_CONNECTING) { if (cm - previousLedMillis >= 500) { previousLedMillis = cm; ledStateHigh = !ledStateHigh; digitalWrite(WIFI_LED_PIN, ledStateHigh ? HIGH : LOW); } } else if (currentLedState == LED_BLINK_FAIL) { if (failBlinkCount < 8) { if (cm - previousLedMillis >= 100) { previousLedMillis = cm; ledStateHigh = !ledStateHigh; digitalWrite(WIFI_LED_PIN, ledStateHigh ? HIGH : LOW); failBlinkCount++; } } else { digitalWrite(WIFI_LED_PIN, LOW); currentLedState = LED_OFF; } } }
void startAPMode() { isAPMode = true; WiFi.mode(WIFI_AP); WiFi.softAP("JIG_8FT_P1_WIFIset"); dnsServer.start(DNS_PORT, "*", WiFi.softAPIP()); }
void setupWiFi() {
  setLedState(LED_BLINK_CONNECTING); WiFi.mode(WIFI_STA); WiFi.disconnect(); delay(100);
  int n = WiFi.scanNetworks(); MatchedWiFi matches[5]; int matchCount = 0; bool anyConnected = false;
  if (n > 0) { for (int i = 0; i < n; ++i) { String scannedSSID = WiFi.SSID(i); int32_t scannedRSSI = WiFi.RSSI(i); for (int j = 0; j < 5; j++) { if (wifiList[j].ssid != "" && wifiList[j].ssid == scannedSSID) { bool alreadyAdded = false; for(int k=0; k<matchCount; k++) { if(matches[k].ssid == scannedSSID) { alreadyAdded = true; if(scannedRSSI > matches[k].rssi) matches[k].rssi = scannedRSSI; break; } } if(!alreadyAdded && matchCount < 5) { matches[matchCount].ssid = wifiList[j].ssid; matches[matchCount].pass = wifiList[j].pass; matches[matchCount].rssi = scannedRSSI; matchCount++; } } } } }
  for (int i = 0; i < matchCount - 1; i++) { for (int j = i + 1; j < matchCount; j++) { if (matches[j].rssi > matches[i].rssi) { MatchedWiFi temp = matches[i]; matches[i] = matches[j]; matches[j] = temp; } } }
  for (int i = 0; i < matchCount; i++) { WiFi.begin(matches[i].ssid.c_str(), matches[i].pass.c_str()); int attempts = 0; while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); updateLED(); attempts++; } if (WiFi.status() == WL_CONNECTED) { anyConnected = true; break; } }
  if (!anyConnected) { for (int i = 0; i < 5; i++) { if (wifiList[i].ssid == "") continue; WiFi.begin(wifiList[i].ssid.c_str(), wifiList[i].pass.c_str()); int attempts = 0; while (WiFi.status() != WL_CONNECTED && attempts < 20) { delay(500); updateLED(); attempts++; } if (WiFi.status() == WL_CONNECTED) { anyConnected = true; break; } } }
  if (anyConnected) { setLedState(LED_ON); isAPMode = false; initNTP(); } else { setLedState(LED_BLINK_FAIL); startAPMode(); }
}
void initNTP() { configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov"); struct tm timeinfo; int retry = 0; while (!getLocalTime(&timeinfo) && retry < 10) { delay(500); retry++; } if (retry < 10) { sendTimeToM031(&timeinfo); } }
void sendTimeToM031(struct tm *timeinfo) { char dataBuf[32]; snprintf(dataBuf, sizeof(dataBuf), "%04d,%02d,%02d,%02d,%02d,%02d", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec); sendToM031_JIG_8CP("ST", dataBuf); }
void sendConfigToM031() { char dataBuf[64]; snprintf(dataBuf, sizeof(dataBuf), "%.1f,%.1f,%.1f,%.1f,%d", conf_maxScale, conf_limitScale, conf_minVoltage, conf_maxVoltage, conf_limitDuration); sendToM031_JIG_8CP("CF", dataBuf); }
void broadcastWebSocket() { String wsJson = "{\"mA\":" + String(current_mA, 1) + ",\"v\":" + String(current_V, 2) + ",\"power\":\"" + power_state + "\",\"warning\":\"" + systemWarning + "\",\"mAh\":" + String(cumulative_mAh, 4) + ",\"mWh\":" + String(cumulative_mWh, 4) + ",\"logging\":" + (isRecordingCSV ? "true" : "false") + "}"; webSocket.broadcastTXT(wsJson); }
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_CONNECTED) { sendToM031_JIG_8CP("PW", "?"); broadcastWebSocket(); }
  else if (type == WStype_TEXT) { String msg = String((char*)payload); if (msg.startsWith("REC_START:")) { logFilename = msg.substring(10); if (!logFilename.endsWith(".csv")) logFilename += ".csv"; String fullPath = "/logs/" + logFilename; csvFile = SD.open(fullPath.c_str(), FILE_WRITE); if (csvFile) { const uint8_t bom[] = {0xEF, 0xBB, 0xBF}; csvFile.write(bom, sizeof(bom)); csvFile.println("日期時間(含毫秒),電壓(V),電流(mA),容量(mAh),能量(mWh)"); isRecordingCSV = true; broadcastWebSocket(); } } else if (msg == "REC_STOP") { if (isRecordingCSV) { csvFile.close(); isRecordingCSV = false; broadcastWebSocket(); } } }
}
String getCSVStartTime() { struct tm timeinfo; if (!getLocalTime(&timeinfo)) { return String(millis()); } char timeStringBuff[64]; snprintf(timeStringBuff, sizeof(timeStringBuff), "%04d/%02d/%02d %02d:%02d:%02d.%03lu", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, millis() % 1000); return String(timeStringBuff); }
void checkSafetyLimits() {
  unsigned long currentMillis = millis(); bool stateChanged = false; const unsigned long PD_TIMEOUT_MS = 10000;
  if (currentMillis - lastPdTime > PD_TIMEOUT_MS) { if (power_state == "ON") { power_state = "OFF"; overCurrentStartTime = 0; if (isRecordingCSV) { csvFile.close(); isRecordingCSV = false; } stateChanged = true; systemWarning = "【M032 通訊逾時】超過 " + String(PD_TIMEOUT_MS/1000.0, 1) + " 秒未收到電力數據，自動斷電保護！"; } }
  if (power_state == "ON") { if (currentMillis - powerOnTime > 2000) { if (current_V < conf_minVoltage || current_V > conf_maxVoltage) { if (current_mA > 0) { sendToM031_JIG_8CP("PW", "OFF"); systemWarning = "【電壓超標熔斷】實時電壓 " + String(current_V, 2) + "V 觸及安全邊界！"; if (isRecordingCSV) { csvFile.println("ERROR,電壓超標安全熔斷"); csvFile.close(); isRecordingCSV = false; } overCurrentStartTime = 0; stateChanged = true; power_state = "OFF"; } } else if (current_mA > conf_limitScale) { if (current_mA > 0) { if (overCurrentStartTime == 0) overCurrentStartTime = currentMillis; else if (currentMillis - overCurrentStartTime >= conf_limitDuration * 1000UL) { sendToM031_JIG_8CP("PW", "OFF"); systemWarning = "【電流超載延時熔斷】電流連續超標！"; if (isRecordingCSV) { csvFile.println("ERROR,電流超載安全熔斷"); csvFile.close(); isRecordingCSV = false; } overCurrentStartTime = 0; stateChanged = true; power_state = "OFF"; } } else { overCurrentStartTime = 0; } } else { overCurrentStartTime = 0; } } }
  if (stateChanged) { broadcastWebSocket(); }
}
void sendToM031_JIG_8CP(String cmd, String data) { String payload = cmd + data; unsigned int sum = 0; for (int i = 0; i < payload.length(); i++) { sum += payload[i]; } char hexSum[3]; sprintf(hexSum, "%02X", sum & 0xFF); Serial.write(JIG_8CP_STX); Serial.print(payload); Serial.print(hexSum); Serial.write(JIG_8CP_CR); }
void readHostUART() { static String rxBuffer = ""; static bool isReceiving = false; while (Serial.available()) { char c = Serial.read(); if (c == JIG_8CP_STX) { rxBuffer = ""; isReceiving = true; } else if (c == JIG_8CP_CR && isReceiving) { isReceiving = false; processM031Command(rxBuffer); rxBuffer = ""; } else if (isReceiving) { rxBuffer += c; if (rxBuffer.length() > 128) { isReceiving = false; rxBuffer = ""; } } } }
void processM031Command(String packet) {
  if (packet.length() < 4) return; String cmdData = packet.substring(0, packet.length() - 2); String receivedChk = packet.substring(packet.length() - 2); unsigned int sum = 0; for (int i = 0; i < cmdData.length(); i++) { sum += cmdData[i]; } char calcChk[3]; sprintf(calcChk, "%02X", sum & 0xFF); if (receivedChk != String(calcChk)) return;
  String cmd = cmdData.substring(0, 2); String data = cmdData.substring(2);
  if (cmd == "WI" && data == "?") { String currentIP = isAPMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString(); sendToM031_JIG_8CP("WI", currentIP); }
  else if (cmd == "GC" && data == "?") { sendConfigToM031(); }
  else if (cmd == "AL" && data == "?") { for(int i = 0; i < 6; i++) { char buf[32]; snprintf(buf, sizeof(buf), "%d,%02d,%02d,%02d,%d", i, alarms[i].h, alarms[i].m, alarms[i].s, alarms[i].en); sendToM031_JIG_8CP("AL", buf); delay(80); } }
  else if (cmd == "AS") { int idx=0, h=0, m=0, s=0, en=0; if (sscanf(data.c_str(), "%d,%d,%d,%d,%d", &idx, &h, &m, &s, &en) == 5) { if(idx >= 0 && idx < 6) { alarms[idx].h = h; alarms[idx].m = m; alarms[idx].s = s; alarms[idx].en = en; saveAlarmsConfig(); sendToM031_JIG_8CP("AS_OK", "OK"); } } }
  else if (cmd == "ST" && data == "OK") { Serial.println("RTC Sync OK"); }
  else if (cmd == "PW") { if (data.indexOf("ON") >= 0) { if (power_state != "ON") { powerOnTime = millis(); lastPdTime = millis(); } power_state = "ON"; } else if (data.indexOf("OFF") >= 0) power_state = "OFF"; broadcastWebSocket(); }
  else if (cmd == "PD") { unsigned long currentMillis = millis(); float deltaSec = 0.0; if (lastPdTime > 0) { deltaSec = (currentMillis - lastPdTime) / 1000.0; if (deltaSec > 2.0 || deltaSec <= 0) deltaSec = 0.1; } lastPdTime = currentMillis; int commaIdx = data.indexOf(','); if(commaIdx > 0) { current_mA = data.substring(0, commaIdx).toFloat(); current_V = data.substring(commaIdx + 1).toFloat(); } if (power_state == "ON") { cumulative_mAh += current_mA * (deltaSec / 3600.0); cumulative_mWh += (current_mA * current_V) * (deltaSec / 3600.0); if (isRecordingCSV && csvFile) { csvFile.printf("%s,%.2f,%.1f,%.4f,%.4f\n", getCSVStartTime().c_str(), current_V, current_mA, cumulative_mAh, cumulative_mWh); static unsigned long lastFlushTime = 0; if (currentMillis - lastFlushTime >= 1000) { csvFile.flush(); lastFlushTime = currentMillis; } } } }
}

// =======================================================
// [OTA 第2階段] 驗證 ESP32 → M031 LDROM 的 UART1 ISP 握手
// 僅送 CONNECT + GET_DEVICEID + RUN_APROM，【不寫 Flash】，零變磚風險。
// 觸發：瀏覽器直接訪問 http://<ESP32_IP>/api/isp_test
// =======================================================
#define ISP_PKT_SIZE 64

// 計算 host 送出封包的 checksum（與 device 端 Checksum() 對稱：整包 64B 累加截 uint16）
static uint16_t isp_checksum(const uint8_t *pkt) {
    uint16_t c = 0;
    for (int i = 0; i < ISP_PKT_SIZE; i++) c += pkt[i];
    return c;
}

// 一次連續送出 64 bytes（滿足 device「收滿 64 才 ready」）
static void isp_send(const uint8_t *pkt) {
    Serial.write(pkt, ISP_PKT_SIZE);
}

// 讀取 64 bytes response，超時回傳 false
static bool isp_recv(uint8_t *buf, uint32_t timeout_ms) {
    uint32_t t0 = millis();
    int got = 0;
    while (got < ISP_PKT_SIZE) {
        if (Serial.available()) {
            buf[got++] = Serial.read();
        } else if (millis() - t0 > timeout_ms) {
            return false;   // 超時（多半是 CONNECT 落在 300ms 握手窗口外）
        }
    }
    return true;
}

// void handleIspTest() {
//     String r = "=== M031 LDROM ISP 握手測試 (第2階段, 不寫Flash) ===\r\n";

//     // 0. 清空 Serial RX，避免 JIG_8CP 殘留字元混入
//     while (Serial.available()) Serial.read();

//     // 1. 送 JIG_8CP "OT" 觸發 M031 進 LDROM（此刻 M031 仍在 APROM，認 JIG_8CP）
//     sendToM031_JIG_8CP("OT", "");
//     delay(100);   // 等 M031 收 OT + reset + LDROM 開機 + UART1 就緒（窗口約 reset 後 20~320ms）
//     while (Serial.available()) Serial.read();   // 清掉 reset 期間可能的雜訊

//     // 2. CONNECT
//     uint8_t pkt[ISP_PKT_SIZE] = {0};
//     pkt[0] = 0xAE; pkt[1] = 0x00; pkt[2] = 0x00; pkt[3] = 0x00;   // CMD_CONNECT (LE)
//     uint16_t exp = isp_checksum(pkt);
//     isp_send(pkt);
//     uint8_t resp[ISP_PKT_SIZE];
//     bool ok1 = isp_recv(resp, 500);
//     uint16_t got1 = ok1 ? (uint16_t)(resp[0] | (resp[1] << 8)) : 0xFFFF;
//     r += "[CONNECT] 送出checksum=0x" + String(exp, HEX) +
//          " 收到=0x" + String(got1, HEX) +
//          " => " + String((ok1 && got1 == exp) ? "OK" : "FAIL") + "\r\n";

//     // 3. GET_DEVICEID
//     memset(pkt, 0, ISP_PKT_SIZE);
//     pkt[0] = 0xB1; pkt[1] = 0x00; pkt[2] = 0x00; pkt[3] = 0x00;   // CMD_GET_DEVICEID (LE)
//     exp = isp_checksum(pkt);
//     isp_send(pkt);
//     bool ok2 = isp_recv(resp, 500);
//     uint16_t got2 = ok2 ? (uint16_t)(resp[0] | (resp[1] << 8)) : 0xFFFF;
//     uint32_t pdid = ok2 ? ((uint32_t)resp[8] | ((uint32_t)resp[9] << 8) |
//                            ((uint32_t)resp[10] << 16) | ((uint32_t)resp[11] << 24)) : 0;
//     r += "[GET_DEVID] 送出checksum=0x" + String(exp, HEX) +
//          " 收到=0x" + String(got2, HEX) +
//          " => " + String((ok2 && got2 == exp) ? "OK" : "FAIL") + "\r\n";
//     r += "[PDID] 0x" + String(pdid, HEX) +
//          " => " + String((pdid != 0 && pdid != 0xFFFFFFFF) ? "合理(非0/非全F)" : "異常") + "\r\n";

//     // 4. RUN_APROM 讓 M031 跳回 APROM（此命令【無 response】，不可等）
//     memset(pkt, 0, ISP_PKT_SIZE);
//     pkt[0] = 0xAB; pkt[1] = 0x00; pkt[2] = 0x00; pkt[3] = 0x00;   // CMD_RUN_APROM (LE)
//     isp_send(pkt);
//     delay(200);   // 等 M031 software reset 回 APROM
//     r += "[RUN_APROM] 已送出(無response), 等M031回APROM\r\n";

//     // 5. 總判
//     bool pass = ok1 && (got1 == isp_checksum((const uint8_t[]){0xAE,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}))
//                     && ok2 && (got2 == isp_checksum((const uint8_t[]){0xB1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}))
//                     && (pdid != 0 && pdid != 0xFFFFFFFF);
//     r += "\r\n>>> 第2階段 " + String(pass ? "PASS <<<" : "FAIL <<<") + "\r\n";

//     server.send(200, "text/plain; charset=utf-8", r);
// }

// =======================================================
// [OTA 區塊 V4.2] 從此行整段覆蓋到檔案結尾
// =======================================================
#define M031_OTA_PENDING   "/ota/m031_pending.bin"
#define M031_APROM_MAX     (128 * 1024)

static String  g_ota_pending_path = "";
static uint32_t g_ota_pending_size = 0;
static String  g_ota_expected_ver = "";
static bool    g_ota_dryrun_ok = false;
static String  g_ota_dryrun_report = "";
static int     g_ota_upload_status = 200;
static String  g_ota_upload_body   = "";

static String ota_parse_version(const String &fname) {
  int i = fname.indexOf('V');
  while (i >= 0) {
    int p = i + 1, dots = 0, digits = 0, end = p;
    int len = fname.length();
    for (int k = p; k < len; k++) {
      char c = fname.charAt(k);
      if (c >= '0' && c <= '9') { digits++; end = k + 1; }
      else if (c == '.' && digits > 0 && dots < 2) { dots++; digits = 0; end = k + 1; }
      else { break; }
    }
    if (dots == 2 && digits > 0) return fname.substring(i, end);
    i = fname.indexOf('V', i + 1);
  }
  return "";
}

static bool ota_dryrun(uint32_t bin_size, String &report) {
  report = "";
  if (bin_size == 0 || (bin_size % 4) != 0 || bin_size > M031_APROM_MAX) {
    report += "FAIL: 大小非法或非4倍數或逾上限\r\n";
    return false;
  }
  uint32_t addr = 0, remaining = bin_size;
  int pkt = 0; bool ok = true;
  {
    uint32_t first_data = remaining < 48 ? remaining : 48;
    if ((first_data % 4) != 0) ok = false;
    char line[96];
    snprintf(line, sizeof(line), "pkt#%d cmd=0xA0 off=16 len=%u -> flash 0x%05X..0x%05X (%u)\r\n",
             pkt, (unsigned)first_data, (unsigned)addr, (unsigned)(addr + first_data - 1), (unsigned)first_data);
    report += line;
    addr += first_data; remaining -= first_data; pkt++;
  }
  while (remaining > 0) {
    uint32_t chunk = remaining < 56 ? remaining : 56;
    if ((chunk % 4) != 0) ok = false;
    char line[96];
    snprintf(line, sizeof(line), "pkt#%d cmd=0x00 off=8  len=%u -> flash 0x%05X..0x%05X (%u)\r\n",
             pkt, (unsigned)chunk, (unsigned)addr, (unsigned)(addr + chunk - 1), (unsigned)chunk);
    report += line;
    addr += chunk; remaining -= chunk; pkt++;
  }
  char tail[96];
  snprintf(tail, sizeof(tail), "---- 共 %d 包, 覆蓋 flash [0x00000..0x%05X], bin_size=%u, 連續=%s, 4對齊=%s\r\n",
           pkt, (unsigned)(bin_size - 1), (unsigned)bin_size, (addr == bin_size) ? "YES" : "NO", ok ? "YES" : "NO");
  report += tail;
  if (addr != bin_size) ok = false;
  return ok;
}

void handleM031Ota() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>M031 韌體更新</title>";
  html += "<style>body{font-family:Arial,sans-serif;text-align:center;padding:30px;background:#f4f4f9;}.box{background:#fff;max-width:560px;margin:0 auto;padding:25px;border-radius:10px;box-shadow:0 4px 6px rgba(0,0,0,.1);}input[type=file]{margin:15px 0;width:100%;}button{padding:12px 24px;font-size:16px;font-weight:bold;color:#fff;background:#8e44ad;border:none;border-radius:5px;cursor:pointer;width:100%;}.note{font-size:13px;color:#666;text-align:left;background:#fffbe6;border:1px solid #ffe58f;padding:10px;border-radius:5px;margin-top:15px;}.back{display:inline-block;margin-top:18px;color:#0056b3;text-decoration:none;font-weight:bold;}</style>";
  html += "</head><body><div class='box'><h2>🛠️ M031 韌體更新</h2>";
  html += "<form action='/m031_ota_upload' method='POST' enctype='multipart/form-data'>";
  html += "<input type='file' name='binfile' accept='.bin' required><button type='submit'>上傳並預檢</button></form>";
  html += "<div class='note'><b>檔名規格：</b><code>m031_Vx.y.z.bin</code>（例 <code>m031_V5.5.0.bin</code>）。<br>前綴 <code>m031_</code> 與副檔名 <code>.bin</code> 為強制安全閘。<br>版本字串須與 bin 內 FIRMWARE_VERSION 逐字相同。</div>";
  html += "<a href='/' class='back'>⬅ 返回首頁</a></div></body></html>";
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleM031OtaUpload() {
  HTTPUpload &upload = server.upload();
  static File wf;
  static String fname = "";
  static bool rejected = false;
  if (upload.status == UPLOAD_FILE_START) {
    fname = upload.filename;
    rejected = false;
    g_ota_upload_status = 200;
    g_ota_upload_body = "";
    if (wf) wf.close();
    String low = fname; low.toLowerCase();
    if (!low.startsWith("m031_") || !low.endsWith(".bin")) { rejected = true; return; }
    if (!SD.exists("/ota")) SD.mkdir("/ota");
    wf = SD.open(M031_OTA_PENDING, FILE_WRITE);
    if (!wf) { rejected = true; g_ota_upload_status = 500; g_ota_upload_body = "無法建立暫存檔 /ota/m031_pending.bin"; }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!rejected && wf) wf.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (wf) wf.close();
    if (rejected) {
      SD.remove(M031_OTA_PENDING);
      g_ota_upload_status = 400;
      g_ota_upload_body = "拒絕：檔名不符安全閘（須為 m031_*.bin），未寫入任何檔案。";
      return;
    }
    uint32_t sz = upload.totalSize;
    g_ota_pending_path = M031_OTA_PENDING;
    g_ota_pending_size = sz;
    g_ota_expected_ver = ota_parse_version(fname);
    g_ota_dryrun_ok = ota_dryrun(sz, g_ota_dryrun_report);
    g_ota_upload_status = 200;
    g_ota_upload_body = g_ota_dryrun_report;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (wf) wf.close();
    SD.remove(M031_OTA_PENDING);
    g_ota_upload_status = 400;
    g_ota_upload_body = "上傳中止";
  }
}

// 主 handler：上傳成功且預檢通過 → 重導向到帶燒錄按鈕的報告頁
void handleM031OtaUploadDone() {
  if (g_ota_upload_status == 200 && g_ota_dryrun_ok && g_ota_pending_size > 0) {
    server.sendHeader("Location", "/m031_ota_report");
    server.send(303, "text/plain; charset=UTF-8", "redirecting to report");
    return;
  }
  server.send(g_ota_upload_status, "text/plain; charset=UTF-8", g_ota_upload_body);
}

void handleM031OtaReport() {
  bool canBurn = g_ota_dryrun_ok && (g_ota_pending_size > 0);
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>M031 OTA 報告</title>";
  html += "<style>body{font-family:Arial,sans-serif;padding:25px;background:#f4f4f9;}.box{background:#fff;max-width:720px;margin:0 auto;padding:20px;border-radius:10px;box-shadow:0 4px 6px rgba(0,0,0,.1);}pre{background:#1e1e1e;color:#0f0;padding:12px;border-radius:6px;white-space:pre-wrap;font-size:13px;}.ok{color:#28a745;font-weight:bold;}.fail{color:#dc3545;font-weight:bold;}.back{display:inline-block;margin-top:15px;color:#0056b3;text-decoration:none;font-weight:bold;}.burn{margin-top:18px;padding:14px 24px;font-size:17px;font-weight:bold;color:#fff;background:#c0392b;border:none;border-radius:6px;cursor:pointer;width:100%;}.burn:disabled{background:#999;cursor:not-allowed;}#progMask{display:none;margin-top:18px;}#barC{width:100%;background:#e9ecef;border-radius:5px;overflow:hidden;height:22px;}#bar{height:100%;width:0%;background:#007bff;transition:width .2s;}#result{margin-top:14px;font-size:15px;line-height:1.6;}.warn{background:#fff3cd;border:1px solid #ffe58f;color:#856404;padding:10px;border-radius:5px;margin-top:12px;font-size:13px;}</style>";
  html += "</head><body><div class='box'><h2>📋 M031 OTA 預檢報告</h2>";
  html += "<p>待燒檔: <code>" + g_ota_pending_path + "</code></p>";
  html += "<p>大小: " + String(g_ota_pending_size) + " byte　|　預期版本: " + (g_ota_expected_ver.length() ? g_ota_expected_ver : String("（未指定）")) + "</p>";
  html += "<p>dry-run: " + String(g_ota_dryrun_ok ? "<span class='ok'>PASS</span>" : "<span class='fail'>FAIL</span>") + "</p>";
  html += "<pre>" + g_ota_dryrun_report + "</pre>";
  if (canBurn) {
    html += "<div class='warn'>⚠ 燒錄將先強制斷電並擦除 M031 整片 APROM，約 12–15 秒，期間請勿斷電或關網頁。萬一失敗，以 Nu-Link 重燒 APROM+LDROM 救回。</div>";
    html += "<button id='burnBtn' class='burn' onclick='startBurn()'>🔥 確認燒錄 M031</button>";
    html += "<div id='progMask'><div id='barC'><div id='bar'></div></div><p id='ptxt'>準備中...</p></div>";
    html += "<div id='result'></div>";
    html += "<script>function startBurn(){if(!confirm('燒錄將擦除 M031 APROM，約 12-15 秒，期間請勿斷電。確認？'))return;var b=document.getElementById('burnBtn');b.disabled=true;document.getElementById('progMask').style.display='block';document.getElementById('result').innerHTML='';var p=0;var iv=setInterval(function(){if(p<90){p+=90/120;}document.getElementById('bar').style.width=p+'%';document.getElementById('ptxt').innerText='燒錄中 '+Math.round(p)+'%（預估，請勿斷電）';},100);fetch('/m031_ota_burn',{method:'POST'}).then(function(r){return r.json();}).then(function(j){clearInterval(iv);document.getElementById('bar').style.width='100%';var h='';if(j.status==='OK'){h+='<span class=\"ok\">✅ 燒錄成功</span>　共 '+j.packets+' 包<br>';h+='預期版本：'+j.expected_ver+'　實際回讀：'+(j.actual_ver&&j.actual_ver.length?j.actual_ver:'(未讀取)')+'<br>';if(j.ver_match===false)h+='<span style=\"color:#f39c12\">⚠ 版本不符或未讀取，請人工確認 M031 開機版號</span><br>';h+='<b>ESP32 重啟，與 M031 同步握手...</b>';document.getElementById('ptxt').innerText='完成 100%';document.getElementById('result').innerHTML=h;setTimeout(function(){fetch('/api/restart');setTimeout(function(){location.href='/';},1000);},500);}else{h+='<span class=\"fail\">❌ 燒錄失敗</span>　階段：'+j.stage+'　原因：'+j.error+'<br>';h+='請以 Nu-Link 重燒 M031 APROM+LDROM 救回。';document.getElementById('ptxt').innerText='失敗';document.getElementById('result').innerHTML=h;document.getElementById('progMask').style.display='none';b.disabled=false;}}).catch(function(e){clearInterval(iv);document.getElementById('result').innerHTML='<span class=\"fail\">❌ 通訊錯誤：'+e+'</span>　請以 Nu-Link 檢查 M031。';document.getElementById('progMask').style.display='none';b.disabled=false;});}</script>";
  } else {
    html += "<p class='fail'>預檢未通過，無法燒錄。請重新上傳符合 m031_Vx.y.z.bin 的檔案。</p>";
  }
  html += "<a href='/m031_ota' class='back'>⬅ 重新上傳</a>　<a href='/' class='back'>🏠 首頁</a></div></body></html>";
  server.send(200, "text/html; charset=UTF-8", html);
}

void handleM031OtaDryrun() {
  server.send(200, "text/plain; charset=UTF-8",
    "expected_ver=" + g_ota_expected_ver + "\r\nsize=" + String(g_ota_pending_size) +
    "\r\ndryrun_ok=" + String(g_ota_dryrun_ok ? "1" : "0") + "\r\n\r\n" + g_ota_dryrun_report);
}

// =======================================================
// [OTA 第3步 V4.2] 真正燒錄 M031 APROM
// =======================================================
#define OTA_ERASE_TIMEOUT_MS  15000
#define OTA_PKT_TIMEOUT_MS     2000

static String ota_makeFail(const char* stage, const char* msg, int packets) {
  return String("{\"status\":\"FAIL\",\"stage\":\"") + stage +
         "\",\"error\":\"" + msg +
         "\",\"expected_ver\":\"" + g_ota_expected_ver +
         "\",\"actual_ver\":\"\",\"packets\":" + packets + "}";
}

static String readJIG8CP(const char* expectCmd, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  uint8_t buf[160]; int idx = 0; bool inStx = false;
  while (millis() - t0 < timeout_ms) {
    if (Serial.available()) {
      uint8_t c = Serial.read();
      if (c == JIG_8CP_STX) { idx = 0; inStx = true; continue; }
      if (!inStx) continue;
      if (c == JIG_8CP_CR) {
        inStx = false;
        if (idx >= 3) {
          char recvChk[3] = { (char)buf[idx-2], (char)buf[idx-1], 0 };
          buf[idx-2] = 0;
          uint8_t sum = 0; for (int i = 0; i < idx-2; i++) sum += buf[i];
          char calcChk[3]; sprintf(calcChk, "%02X", sum);
          if (strcmp(recvChk, calcChk) == 0 && buf[0] == expectCmd[0] && buf[1] == expectCmd[1]) {
            return String((char*)(buf + 2));
          }
        }
        idx = 0;
      } else {
        if (idx < (int)sizeof(buf) - 1) buf[idx++] = c; else { inStx = false; idx = 0; }
      }
    } else { delay(1); }
  }
  return "";
}

void handleM031OtaBurn() {
  if (!g_ota_dryrun_ok || g_ota_pending_size == 0 || !SD.exists(M031_OTA_PENDING)) {
    server.send(400, "application/json", ota_makeFail("PRE", "no valid pending bin", 0)); return;
  }
  sendToM031_JIG_8CP("PW", "OFF"); power_state = "OFF"; delay(100);
  while (Serial.available()) Serial.read();
  sendToM031_JIG_8CP("OT", "");
  String ot = readJIG8CP("OT", 1000);
  if (ot.length() == 0)     { server.send(500, "application/json", ota_makeFail("OT", "no response", 0)); return; }
  if (ot.startsWith("ERR")) { server.send(409, "application/json", ota_makeFail("OT", "M031 reject power-on", 0)); return; }
  delay(200); while (Serial.available()) Serial.read();
  uint8_t pkt[64], resp[64];
  memset(pkt, 0, 64); pkt[0] = 0xAE;
  uint16_t exp = isp_checksum(pkt); isp_send(pkt);
  bool ok = isp_recv(resp, 1000);
  uint16_t got = ok ? (uint16_t)(resp[0] | (resp[1] << 8)) : 0xFFFF;
  if (!ok || got != exp) { server.send(500, "application/json", ota_makeFail("CONNECT", "checksum/timeout", 0)); return; }
  File bf = SD.open(M031_OTA_PENDING, FILE_READ);
  if (!bf) { server.send(500, "application/json", ota_makeFail("OPEN", "sd read fail", 0)); return; }
  memset(pkt, 0, 64);
  pkt[0] = 0xA0;
  pkt[12] =  g_ota_pending_size        & 0xFF;
  pkt[13] = (g_ota_pending_size >>  8) & 0xFF;
  pkt[14] = (g_ota_pending_size >> 16) & 0xFF;
  pkt[15] = (g_ota_pending_size >> 24) & 0xFF;
  int rd = bf.read(pkt + 16, 48);
  exp = isp_checksum(pkt); isp_send(pkt);
  ok = isp_recv(resp, OTA_ERASE_TIMEOUT_MS);
  got = ok ? (uint16_t)(resp[0] | (resp[1] << 8)) : 0xFFFF;
  if (!ok || got != exp) { bf.close(); server.send(500, "application/json", ota_makeFail("ERASE", "first pkt fail", 1)); return; }
  uint32_t sent = (uint32_t)rd; int packets = 1;
  while (sent < g_ota_pending_size) {
    memset(pkt, 0, 64);
    int chunk = (int)(g_ota_pending_size - sent); if (chunk > 56) chunk = 56;
    int r2 = bf.read(pkt + 8, chunk);
    exp = isp_checksum(pkt); isp_send(pkt);
    ok = isp_recv(resp, OTA_PKT_TIMEOUT_MS);
    got = ok ? (uint16_t)(resp[0] | (resp[1] << 8)) : 0xFFFF;
    if (!ok || got != exp) { bf.close(); server.send(500, "application/json", ota_makeFail("WRITE", ("pkt" + String(packets)).c_str(), packets)); return; }
    sent += (uint32_t)r2; packets++;
  }
  bf.close();
  memset(pkt, 0, 64); pkt[0] = 0xAB; isp_send(pkt);
  delay(2500); while (Serial.available()) Serial.read();
  sendToM031_JIG_8CP("VR", "");
  String avr = readJIG8CP("VR", 3000);
  bool vmatch = (avr.length() > 0 && avr == g_ota_expected_ver);
  String j = String("{\"status\":\"OK\",\"expected_ver\":\"") + g_ota_expected_ver +
             "\",\"actual_ver\":\"" + avr +
             "\",\"ver_match\":" + (vmatch ? "true" : "false") +
             ",\"packets\":" + packets + "}";
  server.send(200, "application/json", j);
}

void handleRestart() { server.send(200, "text/plain; charset=UTF-8", "restarting"); delay(500); ESP.restart(); }
// ===== OTA 區塊結束 =====