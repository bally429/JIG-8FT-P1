/*
 * ESP32 UART0 模擬掃描機測試韌體 (Baud: 115200)
 * 每 3 秒發送一次符合 TR515 協定與 SCAN 標籤的假條碼
 */

#define STX 0x02
#define ETX 0x03
#define CR  0x0D
#define LF  0x0A

void setup() {
  // 開啟 UART0 (連接到 M031 的 PB.2 / PB.3)
  Serial.begin(115200);
}

void loop() {
  // 1. 送出封包頭 STX
  Serial.write(STX);
  
  // 2. 送出條碼內容與辨識標籤
  Serial.print("SCAN:6938576108056");
  
  // 3. 送出換行 CR LF
  Serial.write(CR);
  Serial.write(LF);
  
  // 4. 送出封包尾 ETX
  Serial.write(ETX);

  // 暫停 3 秒後重複發送
  delay(3000);
}