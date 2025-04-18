// --- ALICI KODU (TÜM TANIMLAMALAR EKLENDİ - DÜZELTİLMİŞ) ---
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <Wire.h>
// SBUS kütüphanesi YOK

// --- Pin Tanımlamaları ---
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS 5
#define LORA_RST 14
#define LORA_DIO0 2
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); // <<<--- u8g2 NESNESİ TANIMLI
#define SBUS_TX_PIN 17 // UART2 TX

// --- LoRa Ayarları ---
#define LORA_FREQUENCY 433E6
#define RECEIVER_ADDRESS 0xBB

// --- SBUS Ayarları (Manuel) ---
#define SBUS_BAUD 100000
#define SBUS_CONFIG SERIAL_8E2
HardwareSerial& sbusSerial = Serial2; // UART2
// --- EKSİK SABİTLER EKLENDİ ---
const int SBUS_RAW_MIN = 172;
const int SBUS_RAW_MAX = 1811;
const int SBUS_RAW_CENTER = 992;

// --- Alınan Veriler ve SBUS Kanalları ---
uint16_t sbusChannels[16];
bool failsafe_status = true;
int last_rssi = 0;

// --- RX Gösterge ve Failsafe Zamanlama (EKSİK TANIMLAMALAR EKLENDİ) ---
bool show_rx_indicator = false; // Eksikti
long lastRxTime = 0;            // Eksikti
const int FAILSAFE_TIMEOUT = 1000;
const int RX_INDICATOR_DURATION = 200; // Eksikti

// --- Ekran Güncelleme Zamanlaması (EKSİK TANIMLAMALAR EKLENDİ) ---
long lastDisplayUpdateTime = 0;     // Eksikti
int displayUpdateInterval = 50;     // Eksikti

// --- SBUS Gönderme Zamanlaması (EKSİK TANIMLAMALAR EKLENDİ) ---
long lastSbusSendTime = 0;          // Eksikti
const int SBUS_INTERVAL = 14;       // Eksikti

// --- setup() Fonksiyonu ---
void setup() {
  Serial.begin(115200);
  Serial.println("LoRa SBUS Alıcı Başlatılıyor (Düzeltilmiş)...");
  setFailsafeValues();
  Wire.begin();
  Wire.setClock(100000L);
  if (!u8g2.begin()) { Serial.println("OLED Hatası!"); while (true); }
  Serial.println("OLED Başlatıldı");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.clearBuffer(); u8g2.drawStr(0, 10, "Alici Bekliyor..."); u8g2.sendBuffer();
  delay(1000);
  Serial.println("LoRa Başlatılıyor...");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) { Serial.println("LoRa Hatası!"); /*...*/ while (true); }
  Serial.println("LoRa Başlatıldı.");
  sbusSerial.begin(SBUS_BAUD, SBUS_CONFIG, -1, SBUS_TX_PIN, false);
  if (!sbusSerial) { Serial.println("UART2 başlatılamadı!"); while(1); }
  Serial.println("UART2 SBUS için başlatıldı");
}

// --- loop() Fonksiyonu --- (Artık derlenmeli)
void loop() {
  receiveLoRaPacket();
  checkFailsafe();
  if (millis() - lastSbusSendTime > SBUS_INTERVAL) {
    lastSbusSendTime = millis();
    sendSbusPacket();
  }
  if (millis() - lastDisplayUpdateTime > displayUpdateInterval) {
    lastDisplayUpdateTime = millis();
    updateDisplay();
  }
}

// --- Diğer Fonksiyonlar (receiveLoRaPacket, checkFailsafe, setFailsafeValues, sendSbusPacket, updateDisplay) ---
// Bu fonksiyonların içeriği bir önceki mesajdaki ile aynı kalabilir,
// çünkü hatalar global değişken/sabit tanımlamalarındaydı.
// Sadece emin olmak için, aşağıya tekrar ekliyorum:

// --- Gelen LoRa Paketini İşleme Fonksiyonu ---
void receiveLoRaPacket() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;
  int recipient = LoRa.read(); byte sender = LoRa.read(); byte incomingLength = LoRa.read();
  if (recipient != RECEIVER_ADDRESS && recipient != 0xFF) { while (LoRa.available()) { LoRa.read(); } return; }
  String incomingMsg = ""; while (LoRa.available()) { incomingMsg += (char)LoRa.read(); }
  if (incomingLength != incomingMsg.length()) { Serial.println("Hata: Uzunluk!"); return; }
  StaticJsonDocument<256> doc; DeserializationError error = deserializeJson(doc, incomingMsg);
  if (error) { Serial.println("Hata: JSON!"); return; }

  sbusChannels[0] = map(doc["rl"] | SBUS_RAW_CENTER, 1000, 2000, SBUS_RAW_MIN, SBUS_RAW_MAX);
  sbusChannels[1] = map(doc["pt"] | SBUS_RAW_CENTER, 1000, 2000, SBUS_RAW_MIN, SBUS_RAW_MAX);
  sbusChannels[2] = map(doc["th"] | SBUS_RAW_MIN,    1000, 2000, SBUS_RAW_MIN, SBUS_RAW_MAX); // Map aralığı düzeltildi (Min=1000 -> Min=172)
  sbusChannels[3] = map(doc["yw"] | SBUS_RAW_CENTER, 1000, 2000, SBUS_RAW_MIN, SBUS_RAW_MAX);
  sbusChannels[4] = (doc["s1"] | false) ? SBUS_RAW_MAX : SBUS_RAW_MIN;
  sbusChannels[5] = (doc["s2"] | false) ? SBUS_RAW_MAX : SBUS_RAW_MIN;
  sbusChannels[6] = map(doc["a1"] | SBUS_RAW_CENTER, 1000, 2000, SBUS_RAW_MIN, SBUS_RAW_MAX);
  sbusChannels[7] = map(doc["a2"] | SBUS_RAW_CENTER, 1000, 2000, SBUS_RAW_MIN, SBUS_RAW_MAX);
  for (int i = 8; i < 16; i++) { sbusChannels[i] = SBUS_RAW_MIN; }

  for (int i = 0; i < 16; i++) { sbusChannels[i] = constrain(sbusChannels[i], 0, 2047); }

  last_rssi = LoRa.packetRssi();
  lastRxTime = millis();
  failsafe_status = false;
  show_rx_indicator = true;
}

// --- Failsafe Kontrol Fonksiyonu ---
void checkFailsafe() {
  if (!failsafe_status && (millis() - lastRxTime > FAILSAFE_TIMEOUT)) {
    Serial.println("FAILSAFE AKTIF!");
    setFailsafeValues();
    failsafe_status = true;
  }
}

// --- Failsafe Değerlerini Ayarlama Fonksiyonu ---
void setFailsafeValues() {
  sbusChannels[0] = SBUS_RAW_CENTER; sbusChannels[1] = SBUS_RAW_CENTER; sbusChannels[2] = SBUS_RAW_MIN; sbusChannels[3] = SBUS_RAW_CENTER;
  sbusChannels[4] = SBUS_RAW_MIN; sbusChannels[5] = SBUS_RAW_MIN; sbusChannels[6] = SBUS_RAW_CENTER; sbusChannels[7] = SBUS_RAW_CENTER;
  for (int i = 8; i < 16; i++) { sbusChannels[i] = SBUS_RAW_MIN; }
}

// --- SBUS Paketini Manuel Oluşturma ve Gönderme ---
void sendSbusPacket() {
  uint8_t sbusData[25]; sbusData[0] = 0x0F;
  sbusData[1] = (uint8_t) ((sbusChannels[0] & 0x07FF));
  sbusData[2] = (uint8_t) ((sbusChannels[0] & 0x07FF) >> 8 | (sbusChannels[1] & 0x07FF) << 3);
  sbusData[3] = (uint8_t) ((sbusChannels[1] & 0x07FF) >> 5 | (sbusChannels[2] & 0x07FF) << 6);
  sbusData[4] = (uint8_t) ((sbusChannels[2] & 0x07FF) >> 2);
  sbusData[5] = (uint8_t) ((sbusChannels[2] & 0x07FF) >> 10 | (sbusChannels[3] & 0x07FF) << 1);
  sbusData[6] = (uint8_t) ((sbusChannels[3] & 0x07FF) >> 7 | (sbusChannels[4] & 0x07FF) << 4);
  sbusData[7] = (uint8_t) ((sbusChannels[4] & 0x07FF) >> 4 | (sbusChannels[5] & 0x07FF) << 7);
  sbusData[8] = (uint8_t) ((sbusChannels[5] & 0x07FF) >> 1);
  sbusData[9] = (uint8_t) ((sbusChannels[5] & 0x07FF) >> 9 | (sbusChannels[6] & 0x07FF) << 2);
  sbusData[10] = (uint8_t) ((sbusChannels[6] & 0x07FF) >> 6 | (sbusChannels[7] & 0x07FF) << 5);
  sbusData[11] = (uint8_t) ((sbusChannels[7] & 0x07FF) >> 3);
  sbusData[12] = (uint8_t) ((sbusChannels[8] & 0x07FF));
  sbusData[13] = (uint8_t) ((sbusChannels[8] & 0x07FF) >> 8 | (sbusChannels[9] & 0x07FF) << 3);
  sbusData[14] = (uint8_t) ((sbusChannels[9] & 0x07FF) >> 5 | (sbusChannels[10] & 0x07FF) << 6);
  sbusData[15] = (uint8_t) ((sbusChannels[10] & 0x07FF) >> 2);
  sbusData[16] = (uint8_t) ((sbusChannels[10] & 0x07FF) >> 10 | (sbusChannels[11] & 0x07FF) << 1);
  sbusData[17] = (uint8_t) ((sbusChannels[11] & 0x07FF) >> 7 | (sbusChannels[12] & 0x07FF) << 4);
  sbusData[18] = (uint8_t) ((sbusChannels[12] & 0x07FF) >> 4 | (sbusChannels[13] & 0x07FF) << 7);
  sbusData[19] = (uint8_t) ((sbusChannels[13] & 0x07FF) >> 1);
  sbusData[20] = (uint8_t) ((sbusChannels[13] & 0x07FF) >> 9 | (sbusChannels[14] & 0x07FF) << 2);
  sbusData[21] = (uint8_t) ((sbusChannels[14] & 0x07FF) >> 6 | (sbusChannels[15] & 0x07FF) << 5);
  sbusData[22] = (uint8_t) ((sbusChannels[15] & 0x07FF) >> 3);
  sbusData[23] = 0x00; if (failsafe_status) { sbusData[23] |= (1 << 3); }
  sbusData[24] = 0x00;
  sbusSerial.write(sbusData, 25);
}

// --- OLED Ekranı Güncelleme Fonksiyonu ---
void updateDisplay() {
    u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tf);
    char buffer[25]; int yPos = 10; int xCol1 = 0; int xCol2 = 66;
    bool rx_active = !failsafe_status && show_rx_indicator && (millis() - lastRxTime < RX_INDICATOR_DURATION);
    if (failsafe_status) { u8g2.drawStr(98, yPos, "FAIL"); }
    else {
        sprintf(buffer, "%d", last_rssi); int rssi_width = u8g2.getStrWidth(buffer);
        u8g2.drawStr(128 - rssi_width, yPos, buffer);
        if (rx_active) { u8g2.drawStr(128 - rssi_width - 15 , yPos, "RX"); show_rx_indicator = true; } // show_rx_indicator reset'i kaldırıldı, loop'ta kontrol ediliyor.
        else { show_rx_indicator = false; } // Süre dolunca false yapılıyor.
    }
    yPos += 12;
    sprintf(buffer, "R:%4d  P:%4d", sbusChannels[0], sbusChannels[1]); u8g2.drawStr(xCol1, yPos, buffer); yPos += 11;
    sprintf(buffer, "T:%4d  Y:%4d", sbusChannels[2], sbusChannels[3]); u8g2.drawStr(xCol1, yPos, buffer); yPos += 11;
    sprintf(buffer, "A1:%4d A2:%4d", sbusChannels[6], sbusChannels[7]); u8g2.drawStr(xCol1, yPos, buffer); yPos += 11;
    sprintf(buffer, "S1:%s S2:%s", (sbusChannels[4] > SBUS_RAW_CENTER) ? "ON " : "OFF", (sbusChannels[5] > SBUS_RAW_CENTER) ? "ON " : "OFF");
    u8g2.drawStr(xCol1, yPos, buffer);
    u8g2.sendBuffer();
}
