// Use [env:8266-ESP07]   for ESP07 module(ESP8266 based)
// Use [env:nodemcuv2]    for D1 Mini module(ESP8266 based)
// Use [env:m5stack-atom] for M5Stack ATOM Lite(ESP32 based)
// Use [env:esp32dev]     for Espressif ESP32 DevKit(ESP32 based)


#include <Arduino.h>

#ifdef USE096_128X64_OLED
  #include <U8g2lib.h>
  // ESP32 GP1012 接 OLED SCL will cause flashing error
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /*SCL*/13, /*SDA*/14); // ESP32 Thing, HW I2C with pin remapping
#endif

#ifdef USE091_128X32_OLED
  #include <U8g2lib.h>
  // ESP32 GP1012 接 OLED SCL will cause flashing error
  U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /*SCL*/13, /*SDA*/14); // ESP32 Thing, HW I2C with pin remapping
#endif

// platformio.ini 裡 
//   platform = espressif32 會 define ESP32
//   platform = espressif8266 會 define ESP8266
#ifdef ESP32 
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>
#endif   

#ifdef USE_M5Atom 
  #include <M5Atom.h> // only needed for M5 ATOM
#endif

//GPIO2  -> LED and drive low at startup
//GPIO13 -> connected to OLED SCL will cause flashing error
#define DPISW_D1_PIN  17  //GPI17
#define DPISW_D2_PIN  16  //GPI16 
#define DPISW_D3_PIN  4   //GPIO4
#define DPISW_D4_PIN  15  //GPIO15

// Wi-Fi基地台的名稱和密碼
// const char* ssid = "SA0001";
// const char* password = "123456789";
const char* ssid = "UCM_DEMO";
const char* password = "11223344";

unsigned long previousMillis = 0;
const long interval = 2000;

void setup(void) {

  pinMode(DPISW_D1_PIN, INPUT_PULLUP);  // DIP switch position 1
  pinMode(DPISW_D2_PIN, INPUT_PULLUP);  // DIP switch position 2
  pinMode(DPISW_D3_PIN, INPUT_PULLUP);  // DIP switch position 3
  pinMode(DPISW_D4_PIN, INPUT_PULLUP);  // DIP switch position 4

  int ssid_num =    (1-digitalRead(DPISW_D1_PIN)) * 2 + (1-digitalRead(DPISW_D2_PIN));
  int channel_num = (1-digitalRead(DPISW_D3_PIN)) * 2 + (1-digitalRead(DPISW_D4_PIN));

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  //delay(500);

  u8g2.begin();
  u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function
  u8g2.setFont(u8g2_font_unifont_t_chinese1);
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(0, 15);
  u8g2.print("HELLO");
  u8g2.sendBuffer();

  Serial.printf("bit1 %d, bit2 %d, bit3 %d, bit4 %d\n", digitalRead(DPISW_D1_PIN), digitalRead(DPISW_D2_PIN), digitalRead(DPISW_D3_PIN), digitalRead(DPISW_D4_PIN));

  Serial.printf("Connecting Wi-Fi %s, SSID num: %d, channel num: %d\n", ssid, ssid_num, channel_num);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print("\nConnected to: ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");
  delay(1000);
}

void loop(void) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // 讀取RSSI數值
    int rssi = WiFi.RSSI();
    
    Serial.print("RSSI: ");
    Serial.println(rssi);

    Serial.printf("bit1 %d, bit2 %d, bit3 %d, bit4 %d\n", digitalRead(DPISW_D1_PIN), digitalRead(DPISW_D2_PIN), digitalRead(DPISW_D3_PIN), digitalRead(DPISW_D4_PIN));

  }
}
