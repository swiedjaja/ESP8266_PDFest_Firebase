/*
  #define D0 16
  #define D1 5 // I2C Bus SCL (clock)
  #define D2 4 // I2C Bus SDA (data)
  #define D3 0
  #define D4 2 // Same as "LED_BUILTIN", but inverted logic
  #define D5 14 // SPI Bus SCK (clock)
  #define D6 12 // SPI Bus MISO
  #define D7 13 // SPI Bus MOSI
  #define D8 15 // SPI Bus SS (CS)
  #define D9 3 // RX0 (Serial console)
  #define D10 1 // TX0 (Serial console)

  #define S3 10
  #define RX0 D9
  #define TX0 D10
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <DHTesp.h>
#include <Firebase_ESP_Client.h>
#include <ArduinoJson.h>

#define WIFI_SSID "Steff-IoT"
#define WIFI_PASS "steffiot123"

#define FIREBASE_HOST "https://pdfestiot-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "lhKo93h0sGZeFKYmXwZRpehPbQpaPpmRCX76gX5E"

FirebaseData fbdo;
FirebaseConfig fbConfig;
FirebaseData fbdoStream;
void Firebase_Init();
void onFirebaseStream(FirebaseStream data);

#define MAX_CHANNEL 4
#define PIN_DHT11 D3
const uint8 g_arChannel[MAX_CHANNEL] = {D5, D6, D7, D8};
DHTesp dht;
String g_ChipId;
void OnReadSensor();

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8 i=0; i<4; i++)
  {
    pinMode(g_arChannel[i], OUTPUT);
    digitalWrite(g_arChannel[i], LOW);    
  }
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
    continue;
  }
  // get ESP8266 ChipId
  char szChipID[21];
  sprintf(szChipID, "%08X", ESP.getChipId());
  g_ChipId = szChipID;

  Serial.println("Booting...");
  dht.setup(PIN_DHT11, DHTesp::DHT22);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.print("System online with IP address: ");
  Serial.println(WiFi.localIP());  
  Serial.printf("Signal rssi: %d\n", WiFi.RSSI());
  Firebase_Init();
}

void loop() {
  // put your main code here, to run repeatedly:
  OnReadSensor();
  delay(10000);
}

void OnReadSensor()
{
  digitalWrite(LED_BUILTIN, LOW);

  float h = dht.getHumidity();
  float t = dht.getTemperature();
  if (dht.getStatus() == DHTesp::ERROR_NONE)
  {
      FirebaseJson json;
      json.set("temp", t);
      json.set("humidity", h);

      String path=g_ChipId+"/data";
      Serial.printf("Temp: %.2f, Humidity: %.2f\n", t, h);
      Firebase.RTDB.set(&fbdo, path.c_str(), &json);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void onFirebaseStream(FirebaseStream data)
{//onFirebaseStream: /008A5549/cmd /led1 int 0
  // Serial.printf("onFirebaseStream: %s %s %s %s\n", data.streamPath().c_str(),
  //   data.dataPath().c_str(), data.dataType().c_str(), data.stringData().c_str());

  Serial.printf("Cmd Recv: %s -> %d\n", data.dataPath().c_str(), data.stringData().toInt());
  int nValue = data.stringData().toInt();  
  if (data.dataPath()=="/led1")
    digitalWrite(g_arChannel[0], nValue);
  else  
  if (data.dataPath()=="/led2")
    digitalWrite(g_arChannel[1], nValue);
  else  
  if (data.dataPath()=="/led3")
    digitalWrite(g_arChannel[2], nValue);
  else  
  if (data.dataPath()=="/led4")
    digitalWrite(g_arChannel[3], nValue);
}

void Firebase_Init()
{
  FirebaseAuth fbAuth;
  fbConfig.host = FIREBASE_HOST;
  fbConfig.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);

#if defined(ESP8266)
  //Set the size of WiFi rx/tx buffers in the case where we want to work with large data.
  fbdo.setBSSLBufferSize(2*1024, 1024);
#endif

  //Set the size of HTTP response buffers in the case where we want to work with large data.
  fbdo.setResponseSize(1024);

  //Set database read timeout to 1 minute (max 15 minutes)
  // Firebase.RTDB.setReadTimeout(&fbdo, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.RTDB.setwriteSizeLimit(&fbdo, "small");

  //optional, set the decimal places for float and double data to be stored in database
  // Firebase.setFloatDigits(2);
  // Firebase.setDoubleDigits(6);
  while (!Firebase.ready())
  {
    Serial.println("Connecting to firebase...");
    delay(1000);
  }
   String path = g_ChipId + "/cmd";
  if (Firebase.RTDB.beginStream(&fbdoStream, path.c_str()))
  {
    Serial.println("Firebase stream on "+ path);
    Firebase.RTDB.setStreamCallback(&fbdoStream, onFirebaseStream, 0);
  }
  else
    Serial.println("Firebase stream failed: "+fbdoStream.errorReason());
}
