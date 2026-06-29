/* * 模組角色：ESP32 (ESPWM32EN16) 掃描器橋接程式
 * 通訊協定：JIG_8CP (ASCII Protocol) [STX][Cmd][Data][Checksum][CR]
 * ===========================================================================================
 * Project: JIG-8FT-P1 _WIFIBLE (ESP32 控制端)
 * MCU: ESP32-WROOM-32E (ESPWM32EN16)
 * * [版本更新紀錄]
 * -------------------------------------------------------------------------------------------
 * 版本    日期       更新說明
 * -------------------------------------------------------------------------------------------
 * V1.0    2026/06/29 專案正式建立：
 * 1. 實作基於 TR515 標準協定的封包產生與 Checksum 計算機制。
 * 2. 建立 UART2 (Scanner) 與 UART0 (Host) 的非阻塞式通訊通道。
 * 3. 整合基礎硬體初始化與偵測功能。
 * -------------------------------------------------------------------------------------------
 */

#define JIG_8CP_STX 0x02
#define JIG_8CP_CR  0x0D

#define SCANNER_RX_PIN 16
#define SCANNER_TX_PIN 17

void setup() {
  // 1. 初始化與 M031 連接的 UART0 (TX0, RX0)
  Serial.begin(115200); 

  // 2. 初始化與掃描機連接的 UART2
  Serial2.begin(115200, SERIAL_8N1, SCANNER_RX_PIN, SCANNER_TX_PIN);
}

void loop() {
  // 檢查掃描機是否有刷入條碼
  if (Serial2.available()) {
    String barcodeData = Serial2.readStringUntil('\n'); 
    barcodeData.trim(); // 清除掃描機自帶的換行或空白字元
    
    if (barcodeData.length() > 0) {
      // 呼叫封裝函式，"SC" 代表 SCanner (掃描機)
      sendToM031_JIG_8CP("SC", barcodeData);
    }
  }
}

// 依照 JIG_8CP 協定格式，自動計算 Checksum 並發送給 M031
void sendToM031_JIG_8CP(String cmd, String data) {
  String payload = cmd + data; // 組合 Command 與 Data
  
  // 1. 計算 Checksum (所有字元的 ASCII 總和)
  unsigned int sum = 0;
  for (int i = 0; i < payload.length(); i++) {
    sum += payload[i];
  }
  
  // 取低 8 位元 (00~FF)
  byte checksum = sum & 0xFF; 
  
  // 將 Checksum 轉換為 2 個大寫的 Hex 字元 (例如 "39")
  char hexSum[3];
  sprintf(hexSum, "%02X", checksum);
  
  // 2. 依序發送：[STX] + [Cmd+Data] + [Checksum] + [CR]
  Serial.write(JIG_8CP_STX); // 發送起始碼 \02
  Serial.print(payload);     // 發送字串內容，如 "SCABFD123"
  Serial.print(hexSum);      // 發送檢查碼字元，如 "39"
  Serial.write(JIG_8CP_CR);  // 發送結束碼 \0D
}