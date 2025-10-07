#include <WiFi.h>
#include <PubSubClient.h>
#include <esp32-hal-ledc.h>
#include <TM1637Display.h>

// Pin Definitions
#define TRIG_PIN    18
#define ECHO_PIN    19
#define PIR_PIN     33
#define RED_PIN     26
#define GREEN_PIN   27
#define BLUE_PIN    25
#define CLK_PIN     32
#define DIO_PIN     21

// Wi-Fi & MQTT
const char* WIFI_SSID   = "Scutum";
const char* WIFI_PASS   = "00000111";
const char* MQTT_SERVER = "broker.hivemq.com";
const int   MQTT_PORT   = 1883;
const char* MQTT_TOPIC  = "seat/1/status";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
TM1637Display display(CLK_PIN, DIO_PIN);

// State machine
enum State {
  IDLE,
  MOTION_CHECK,
  OCCUPIED_HUMAN,
  OCCUPIED_OBJECT
};
State state = IDLE;

// Timing
unsigned long stateStart = 0;
const unsigned long checkInt     = 500;   // ultrasonic check interval
const unsigned long motionWin    = 10000; // 10 s window for PIR
const unsigned long settleDelay  = 5000;  // 5 s before we start PIR checking

// Seat release timeout
unsigned long leaveStart = 0;
const unsigned long leaveTimeout = 5000;  // 5s buffer

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);

  // RGB LED channels (frequency: 5kHz, resolution: 8 bits)
  ledcAttach(RED_PIN,   5000, 8);
  ledcAttach(GREEN_PIN, 5000, 8);
  ledcAttach(BLUE_PIN,  5000, 8);

  display.setBrightness(7); // Max brightness for 7-segment display

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println("WiFi connected");

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  reconnectMQTT();
}

// ---------- Main Loop ----------
void loop() {
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();

  unsigned long now = millis();

  switch (state) {
    case IDLE: {
      float d = measureDistance();
      display.showNumberDec((int)d, false);
      setLED(150, 150, 200); // grey

      if (d < 40.0) {
        state = MOTION_CHECK;
        stateStart = now;
        Serial.println("→ MOTION_CHECK");
      }
      delay(checkInt);
      break;
    }

    case MOTION_CHECK: {
      unsigned long elapsed = now - stateStart;

      if (elapsed < settleDelay) {
        setLED(150, 150, 200);
        display.showNumberDec(0, false);
        delay(50);
        break;
      }

      unsigned long pirElapsed = elapsed - settleDelay;
      setLED(150, 150, 200);

      if (digitalRead(PIR_PIN) == HIGH && pirElapsed <= motionWin) {
        state = OCCUPIED_HUMAN;
        mqttClient.publish(MQTT_TOPIC, "1"); // 1 = human detected
        Serial.println("PIR → HUMAN");
      } else if (pirElapsed > motionWin) {
        state = OCCUPIED_OBJECT;
        mqttClient.publish(MQTT_TOPIC, "2"); // no human
        Serial.println("NO PIR → OBJECT");
      }
      delay(50);
      break;
    }

    case OCCUPIED_HUMAN: {
      float d = measureDistance();
      display.showNumberDec((int)d, false);
      setLED(0, 255, 0); // green

      if (d >= 40.0) {
        if (leaveStart == 0) leaveStart = now;
        else if (now - leaveStart >= leaveTimeout) {
          state = IDLE;
          leaveStart = 0;
          Serial.println("← IDLE (human left)");
          mqttClient.publish(MQTT_TOPIC, "0");
          Serial.println("seat released status published");
        }
      } else leaveStart = 0;

      delay(checkInt);
      break;
    }

    case OCCUPIED_OBJECT: {
      float d = measureDistance();
      display.showNumberDec((int)d, false);
      setLED(255, 0, 0); // red

      if (d >= 40.0) {
        if (leaveStart == 0) leaveStart = now;
        else if (now - leaveStart >= leaveTimeout) {
          state = IDLE;
          leaveStart = 0;
          Serial.println("← IDLE (object removed)");
          mqttClient.publish(MQTT_TOPIC, "0");//idle state code is 0
          Serial.println("seat released status published");
        }
      } else leaveStart = 0;

      delay(checkInt);
      break;
    }
  }
}

//idle state code is 0
//human occupied state is 1
//object occupied state is 2
// ---------- Helper Functions ----------

float measureDistance() {
  const int targetSamples = 7;
  float validReadings[targetSamples];
  int count = 0;

  while (count < targetSamples) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long dur = pulseIn(ECHO_PIN, HIGH, 25000);
    float dist = dur * 0.0343f / 2.0f;

    if (dur > 0 && dist >= 2.0 && dist <= 200.0)
      validReadings[count++] = dist;

    delay(10);
  }

  // sort readings
  for (int i = 0; i < targetSamples - 1; i++) {
    for (int j = i + 1; j < targetSamples; j++) {
      if (validReadings[i] > validReadings[j]) {
        float tmp = validReadings[i];
        validReadings[i] = validReadings[j];
        validReadings[j] = tmp;
      }
    }
  }

  return validReadings[targetSamples / 2]; // median
}

void setLED(int r, int g, int b) {
  ledcWrite(RED_PIN, r);
  ledcWrite(GREEN_PIN, g);
  ledcWrite(BLUE_PIN, b);
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    String clientId = "ESP32Client_" + String(ESP.getEfuseMac(), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("MQTT connected");
    } else {
      Serial.print("MQTT fail, rc=");
      Serial.print(mqttClient.state());
      delay(2000);
    }
  }
}
