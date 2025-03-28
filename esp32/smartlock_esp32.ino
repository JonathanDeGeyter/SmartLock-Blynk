/***************************************************
 * ESP32 Main Code
 * 
 * Responsibilities:
 *  - Connect to Wi-Fi and MQTT
 *  - Handle PN5180 NFC for reading keycards
 *  - Control DFPlayer Mini for alarm/track looping
 *  - Toggle Red/Green LEDs as commanded
 *  - Monitor a push-button for reset requests
 *  - Publish separate MQTT topics for button presses, card results, and health
 *  - NOW integrates with Blynk:
 *     * Terminal on V9 for "unlock"/"reset" commands
 *     * V10 for system locked/unlocked (0 = unlocked, 1 = locked)
 *     * V11 for drawer closed/opened (0 = closed, 1 = opened)
 ***************************************************/

#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"
#include "PN5180.h"
#include "PN5180ISO15693.h"

/************** NEW: Blynk Includes **************/
#define BLYNK_TEMPLATE_ID "SmartLock123" // Example
#define BLYNK_TEMPLATE_NAME "Smart Lock"
#define BLYNK_PRINT Serial               // Debug prints
#include <BlynkSimpleEsp32.h>

/************** Wi-Fi Credentials **************/
const char* ssid       = "WiFi-2.4-6A9C_EXT";
const char* password   = "w2p7k76kpcrfu";

/************** Blynk Credentials **************/
// IMPORTANT: Replace with your local Blynk server token
char blynkAuth[] = "3JK0mI-Qg1eTnrftQwCb-81WzS8uxGYU";

/************** MQTT Configuration **************/
const char* mqttServer  = "192.168.1.59";
const int   mqttPort    = 1883;
const char* mqttUser    = "jona";
const char* mqttPass    = "jona";
const char* mqttClientID= "smart_lock_esp32";

/************** MQTT Topics **************/
#define TOPIC_CMD         "lock/esp32/cmd"
#define TOPIC_CARD_RESULT "lock/esp32/card_event"
#define TOPIC_BUTTON      "lock/esp32/button_event"
#define TOPIC_HEALTH      "lock/esp32/health"
#define TOPIC_LCD         "lock/esp32/lcd"

// NOTE: The ESP32 will update its Blynk charts based on the state messages it receives from the Pi.

 /************** DFPlayer / MP3 Setup **************/
static const int MP3_RX = 35;
static const int MP3_TX = 26;
SoftwareSerial mp3Serial(MP3_RX, MP3_TX);
DFRobotDFPlayerMini myDFPlayer;
bool mp3PlayingContinuous = false;

/************** PN5180 NFC Setup **************/
#define PN5180_NSS  16
#define PN5180_BUSY 5
#define PN5180_RST  17
PN5180ISO15693 nfc(PN5180_NSS, PN5180_BUSY, PN5180_RST);
uint8_t correctUID[8] = {0xE0, 0x04, 0x01, 0x53, 0x1A, 0x30, 0x51, 0x77};

const int pinRedLED   = 2;
const int pinGreenLED = 4;
const int pinButton   = 33;
bool systemLocked     = false;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
BlynkTimer timer;

/***************************************************
 * Forward Declarations
 ***************************************************/
void setupWifi();
void reconnectMQTT();
void callback(char* topic, byte* payload, unsigned int length);
void handleCommand(const String &cmd);
void readCardAndRespond();
void startContinuousPlay();
void stopContinuousPlay();
void publishButtonEvent(const char* msg);
void publishCardResult(const char* msg);
void publishHealthInfo();

/***************************************************
 * BLYNK Terminal on V9
 * Type "unlock" or "reset" to forward commands to Pi
 ***************************************************/
BLYNK_WRITE(V9)
{
  String command = param.asStr();
  command.trim();
  Serial.print("Blynk Terminal received: ");
  Serial.println(command);
  if(command.equalsIgnoreCase("unlock")){
    mqttClient.publish(TOPIC_CMD, "UNLOCK");
  }
  else if(command.equalsIgnoreCase("reset")){
    mqttClient.publish(TOPIC_CMD, "reset_pressed");
  }
}

/***************************************************
 * setup()
 ***************************************************/
void setup() {
  Serial.begin(115200);

  pinMode(pinRedLED, OUTPUT);
  pinMode(pinGreenLED, OUTPUT);
  digitalWrite(pinRedLED, LOW);
  digitalWrite(pinGreenLED, LOW);

  pinMode(pinButton, INPUT_PULLUP);

  setupWifi();
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);

  // Start Blynk with local server
  Blynk.begin(blynkAuth, ssid, password, "192.168.1.59", 8080);

  mp3Serial.begin(9600);
  if(!myDFPlayer.begin(mp3Serial, false)) {
    Serial.println("DFPlayer not responding. Check wiring/power.");
  } else {
    Serial.println("DFPlayer Mini online.");
    myDFPlayer.volume(20);
  }

  nfc.begin();
  nfc.reset();
  nfc.setupRF();

  Serial.println("ESP32 setup complete. Awaiting commands.");
}

/***************************************************
 * loop()
 ***************************************************/
void loop() {
  Blynk.run();
  timer.run();

  if(!mqttClient.connected()){
    reconnectMQTT();
  }
  mqttClient.loop();

  if(systemLocked && digitalRead(pinButton) == LOW) {
    Serial.println("Local button pressed -> sending 'reset_pressed'");
    mqttClient.publish(TOPIC_BUTTON, "reset_pressed");
    delay(500);
  }

  if(mp3PlayingContinuous && myDFPlayer.available()){
    uint8_t type = myDFPlayer.readType();
    if(type == DFPlayerPlayFinished){
      Serial.println("Track ended -> restarting track #1");
      myDFPlayer.play(1);
    }
  }

  static unsigned long lastHealth = 0;
  if(millis() - lastHealth > 10000) {
    lastHealth = millis();
    publishHealthInfo();
  }

  delay(10);
}

/***************************************************
 * setupWifi()
 ***************************************************/
void setupWifi() {
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Wi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

/***************************************************
 * publishHealthInfo()
 ***************************************************/
void publishHealthInfo() {
  char msg[50];
  int rssi = WiFi.RSSI();
  sprintf(msg, "ONLINE RSSI:%d", rssi);
  mqttClient.publish(TOPIC_HEALTH, msg);
}

/***************************************************
 * reconnectMQTT()
 ***************************************************/
void reconnectMQTT() {
  while(!mqttClient.connected()){
    Serial.print("Connecting to MQTT... ");
    if(mqttClient.connect(mqttClientID, mqttUser, mqttPass)){
      Serial.println("✅ MQTT connected!");
      mqttClient.subscribe(TOPIC_CMD);
      mqttClient.subscribe("lock/esp32/system_state");
      mqttClient.subscribe("lock/esp32/drawer_state");
      mqttClient.subscribe("lock/esp32/lcd");

    } else {
      Serial.print("MQTT connection failed (rc=");
      Serial.print(mqttClient.state());
      Serial.println("). Retrying in 2s...");
      delay(2000);
    }
  }
}

/***************************************************
 * callback(...)
 ***************************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  String incoming = "";
  for(unsigned int i = 0; i < length; i++){
    incoming += (char)payload[i];
  }
  incoming.trim();
  Serial.print("MQTT message received [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(incoming);

  if(String(topic) == TOPIC_CMD) {
    handleCommand(incoming);
  }
  else if(String(topic) == "lock/esp32/system_state") {
    updateSystemStateBlynk(incoming);
  }
  else if(String(topic) == "lock/esp32/drawer_state") {
    updateDrawerStateBlynk(incoming);
  }
  else if(String(topic) == "lock/esp32/lcd") {
  Blynk.virtualWrite(V5, incoming);
}

}

/***************************************************
 * handleCommand(...)
 ***************************************************/
void handleCommand(const String &cmd) {
  if(cmd == "RED_ON")
    digitalWrite(pinRedLED, HIGH);
  else if(cmd == "RED_OFF")
    digitalWrite(pinRedLED, LOW);
  else if(cmd == "GREEN_ON")
    digitalWrite(pinGreenLED, HIGH);
  else if(cmd == "GREEN_OFF")
    digitalWrite(pinGreenLED, LOW);
  else if(cmd == "PLAY_CONTINUOUS")
    startContinuousPlay();
  else if(cmd == "STOP_PLAY")
    stopContinuousPlay();
  else if(cmd == "READ_CARD")
    readCardAndRespond();
  else if(cmd == "LOCK") {
    systemLocked = true;
    digitalWrite(pinRedLED, HIGH);
    startContinuousPlay();
  }
  else if(cmd == "UNLOCK") {
    systemLocked = false;
    digitalWrite(pinRedLED, LOW);
    stopContinuousPlay();
  }
  else {
    Serial.println("Unknown command: " + cmd);
  }
}

/***************************************************
 * readCardAndRespond()
 ***************************************************/
void readCardAndRespond() {
  Serial.println("Waiting up to 5 seconds for user to tap card...");
  bool foundCard = false;
  uint8_t uid[8];
  unsigned long startTime = millis();

  while(millis() - startTime < 5000) {
    if(nfc.getInventory(uid) == ISO15693_EC_OK) {
      foundCard = true;
      break;
    }
    delay(100);
  }

  if(!foundCard) {
    Serial.println("No card found -> CARD_BAD");
    publishCardResult("BAD");
    return;
  }

  bool match = true;
  for(int i = 0; i < 8; i++){
    if(uid[7 - i] != correctUID[i]){
      match = false;
      break;
    }
  }
  if(match) {
    Serial.println("Card matched -> CARD_GOOD");
    publishCardResult("GOOD");
  } else {
    Serial.println("Card mismatch -> CARD_BAD");
    publishCardResult("BAD");
  }
}

/***************************************************
 * startContinuousPlay()
 ***************************************************/
void startContinuousPlay() {
  if(!mp3PlayingContinuous) {
    mp3PlayingContinuous = true;
    myDFPlayer.loop(1);
    Serial.println("DFPlayer: looping track #1");
  }
}

/***************************************************
 * stopContinuousPlay()
 ***************************************************/
void stopContinuousPlay() {
  if(mp3PlayingContinuous) {
    mp3PlayingContinuous = false;
    myDFPlayer.stop();
    Serial.println("DFPlayer: track stopped");
  }
}

/***************************************************
 * publishButtonEvent(...)
 ***************************************************/
void publishButtonEvent(const char* msg) {
  mqttClient.publish(TOPIC_BUTTON, msg);
}

/***************************************************
 * publishCardResult(...)
 ***************************************************/
void publishCardResult(const char* msg) {
  mqttClient.publish(TOPIC_CARD_RESULT, msg);
}

/***************************************************
 * Blynk SuperChart Update Helpers
 ***************************************************/
void updateSystemStateBlynk(const String &state)
{
  // If state equals "locked", send 1; otherwise 0.
  if(state == "1")
    Blynk.virtualWrite(V10, 1);
  else
    Blynk.virtualWrite(V10, 0);
}

void updateDrawerStateBlynk(const String &state)
{
  // If state equals "opened", send 1; otherwise 0.
  if(state == "1")
    Blynk.virtualWrite(V11, 1);
  else
    Blynk.virtualWrite(V11, 0);
}
