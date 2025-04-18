#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <U8g2lib.h> // U8g2lib KULLANIYORUZ
#include <Wire.h>

// --- Pin Tanımlamaları ---
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS 5
#define LORA_RST 14
#define LORA_DIO0 2
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#define JOY1_X_PIN 34 // ROLL
#define JOY1_Y_PIN 35 // PITCH
#define JOY2_X_PIN 32 // YAW
#define JOY2_Y_PIN 33 // THROTTLE
#define POT1_PIN 36 // AUX1
#define POT2_PIN 39 // AUX2
#define SW1_PIN 25
#define SW2_PIN 26
#define BATT_PIN 4  // Pil Pini

// --- LoRa Ayarları ---
#define LORA_FREQUENCY 433E6
byte localAddress = 0xAA;
byte destinationAddress = 0xBB;
// byte msgCount = 0; // Msg sayacı kaldırıldı
long lastSendTime = 0;
int sendInterval = 100;

// --- Kanal ve SBUS Ayarları ---
int joy1X_raw, joy1Y_raw, joy2X_raw, joy2Y_raw, pot1_raw, pot2_raw, batt_raw;
int roll_map, pitch_map, yaw_map, throttle_map, aux1_map, aux2_map;
bool sw1_state, sw2_state;
const int SBUS_MIN = 1000;
const int SBUS_MAX = 2000;
const int SBUS_CENTER = 1500;
const int ADC_CENTER = 2048;
const int DEADZONE_WIDTH = 100;

// --- Pil, TX/ERR Gösterge ve Voltaj Yumuşatma Değişkenleri ---
float smoothed_battery_voltage = 0.0;
const int VOLTAGE_SMOOTHING_SAMPLES = 10;
int voltage_readings[VOLTAGE_SMOOTHING_SAMPLES];
int voltage_read_index = 0;
long voltage_total = 0;
// bool lora_tx_indicator = false; // Eski TX göstergesi kaldırıldı
// --- YENİ TX/ERR DURUM DEĞİŞKENLERİ ---
enum TxStatus { TX_IDLE, TX_SUCCESS, TX_FAIL };
TxStatus lastTxStatus = TX_IDLE;
long lastTxStatusTime = 0;
const int TX_STATUS_DISPLAY_DURATION = 300; // ms cinsinden gösterge süresi

// --- Ekran Güncelleme Zamanlaması ---
long lastDisplayUpdateTime = 0;
int displayUpdateInterval = 50;

// --- setup() Fonksiyonu ---
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa SBUS Verici Başlatılıyor (TX Durum Göstergeli)...");

  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  pinMode(BATT_PIN, INPUT);

  Wire.begin();
  Wire.setClock(100000L);

  if (!u8g2.begin()) {
    Serial.println("OLED başlatılamadı!");
    while (true);
  }
  Serial.println("OLED Başlatıldı (U8g2)");

  for (int i = 0; i < VOLTAGE_SMOOTHING_SAMPLES; i++) { voltage_readings[i] = 0; }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf); // Stabil fontumuz
  u8g2.drawStr(0, 10, "Verici Hazir!");
  u8g2.sendBuffer();
  delay(1500);

  Serial.println("LoRa Başlatılıyor...");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa başlatma başarısız.");
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "LoRa Hatasi!");
    u8g2.sendBuffer();
    while (true);
  }
  Serial.println("LoRa başarıyla başlatıldı.");
}

// --- loop() Fonksiyonu --- (Aynı)
void loop() {
  readInputs();
  if (millis() - lastDisplayUpdateTime > displayUpdateInterval) {
    lastDisplayUpdateTime = millis();
    updateDisplay();
  }
  sendLoRaPacket();
}

// --- readInputs() Fonksiyonu --- (Aynı)
void readInputs() {
    // ... (Önceki kodla aynı) ...
    joy1X_raw = analogRead(JOY1_X_PIN); joy1Y_raw = analogRead(JOY1_Y_PIN);
    joy2X_raw = analogRead(JOY2_X_PIN); joy2Y_raw = analogRead(JOY2_Y_PIN);
    pot1_raw = analogRead(POT1_PIN); pot2_raw = analogRead(POT2_PIN);
    batt_raw = analogRead(BATT_PIN);
    if (abs(joy1X_raw - ADC_CENTER) < DEADZONE_WIDTH) { roll_map = SBUS_CENTER; } else { roll_map = map(joy1X_raw, 0, 4095, SBUS_MIN, SBUS_MAX); }
    if (abs(joy1Y_raw - ADC_CENTER) < DEADZONE_WIDTH) { pitch_map = SBUS_CENTER; } else { pitch_map = map(joy1Y_raw, 0, 4095, SBUS_MAX, SBUS_MIN); }
    if (abs(joy2X_raw - ADC_CENTER) < DEADZONE_WIDTH) { yaw_map = SBUS_CENTER; } else { yaw_map = map(joy2X_raw, 0, 4095, SBUS_MIN, SBUS_MAX); }
    if (abs(joy2Y_raw - ADC_CENTER) < DEADZONE_WIDTH) { throttle_map = SBUS_CENTER; } else { throttle_map = map(joy2Y_raw, 0, 4095, SBUS_MIN, SBUS_MAX); }
    aux1_map = map(pot1_raw, 0, 4095, SBUS_MIN, SBUS_MAX); aux2_map = map(pot2_raw, 0, 4095, SBUS_MIN, SBUS_MAX);
    roll_map = constrain(roll_map, SBUS_MIN, SBUS_MAX); pitch_map = constrain(pitch_map, SBUS_MIN, SBUS_MAX);
    yaw_map = constrain(yaw_map, SBUS_MIN, SBUS_MAX); throttle_map = constrain(throttle_map, SBUS_MIN, SBUS_MAX);
    aux1_map = constrain(aux1_map, SBUS_MIN, SBUS_MAX); aux2_map = constrain(aux2_map, SBUS_MIN, SBUS_MAX);
    sw1_state = (digitalRead(SW1_PIN) == LOW); sw2_state = (digitalRead(SW2_PIN) == LOW);
    voltage_total = voltage_total - voltage_readings[voltage_read_index];
    voltage_readings[voltage_read_index] = batt_raw;
    voltage_total = voltage_total + voltage_readings[voltage_read_index];
    voltage_read_index++; if (voltage_read_index >= VOLTAGE_SMOOTHING_SAMPLES) { voltage_read_index = 0; }
    int average_batt_raw = voltage_total / VOLTAGE_SMOOTHING_SAMPLES;
    smoothed_battery_voltage = (average_batt_raw * 3.3 / 4095.0) * 3.0;
}


// --- updateDisplay() Fonksiyonu (Msg Kaldırıldı, TX/ERR Eklendi) ---
void updateDisplay() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  char buffer[25];
  int yPos = 10;
  int xCol1 = 0;
  int xCol2 = 68;

  // Satır 1: Pil ve TX/ERR Göstergesi
  sprintf(buffer, "%.1fV", smoothed_battery_voltage);
  u8g2.drawStr(xCol1, yPos, buffer);
  // --- YENİ: TX/ERR Durumunu Göster ---
  if (millis() - lastTxStatusTime < TX_STATUS_DISPLAY_DURATION) {
    if (lastTxStatus == TX_SUCCESS) {
       u8g2.drawStr(xCol2 + 30, yPos, "TX"); // Başarılı
    } else if (lastTxStatus == TX_FAIL) {
       u8g2.drawStr(xCol2 + 25, yPos, "ERR"); // Başarısız (Biraz sola)
    }
     // IDLE durumunda bir şey gösterme
  }
  yPos += 11;

  // Satır 2: ROLL ve PITCH
  sprintf(buffer, "ROLL:%4d PITCH:%4d", roll_map, pitch_map);
  u8g2.drawStr(xCol1, yPos, buffer);
  yPos += 11;

  // Satır 3: THROTTLE ve YAW
  sprintf(buffer, "THRO:%4d  YAW:%4d", throttle_map, yaw_map);
  u8g2.drawStr(xCol1, yPos, buffer);
  yPos += 11;

  // Satır 4: AUX1 ve AUX2
  sprintf(buffer, "AUX1:%4d AUX2:%4d", aux1_map, aux2_map);
  u8g2.drawStr(xCol1, yPos, buffer);
  yPos += 11;

  // Satır 5: SW1 ve SW2
  sprintf(buffer, "S1:%s S2:%s", sw1_state ? "ON" : "OFF ", sw2_state ? "ON" : "OFF "); // Boşluklar eklendi
  u8g2.drawStr(xCol1, yPos, buffer);

  // Msg sayacı kaldırıldı

  u8g2.sendBuffer();
}


// --- sendLoRaPacket() Fonksiyonu (TX/ERR Durumu Kaydediliyor) ---
void sendLoRaPacket() {
  if (millis() - lastSendTime > sendInterval) {
    lastSendTime = millis();
    StaticJsonDocument<256> doc;
    doc["rl"] = roll_map; doc["pt"] = pitch_map; doc["yw"] = yaw_map;
    doc["th"] = throttle_map; doc["a1"] = aux1_map; doc["a2"] = aux2_map;
    doc["s1"] = sw1_state; doc["s2"] = sw2_state;
    String outgoing_msg;
    serializeJson(doc, outgoing_msg);
    LoRa.beginPacket();
    LoRa.write(destinationAddress); LoRa.write(localAddress); // MsgCount gönderilmiyor
    LoRa.write(outgoing_msg.length()); LoRa.print(outgoing_msg);
    bool sent = LoRa.endPacket();

    // --- YENİ: Gönderim Durumunu Kaydet ---
    lastTxStatusTime = millis(); // Durumun zamanını kaydet
    if (sent) {
      lastTxStatus = TX_SUCCESS; // Durumu başarılı olarak ayarla
      // msgCount++; // Sayacı artırmıyoruz artık
    } else {
      lastTxStatus = TX_FAIL;    // Durumu başarısız olarak ayarla
      Serial.println("LoRa gonderme basarisiz!");
    }
  }
}
