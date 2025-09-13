#include <WiFi.h>
#include <PubSubClient.h>
#include <esp32-hal-ledc.h>
#include <TM1637Display.h>

// Pins
#define TRIG_PIN    13
#define ECHO_PIN    14
#define PIR_PIN     35
#define RED_PIN     26
#define GREEN_PIN   25
#define BLUE_PIN    27
#define CLK_PIN     33
#define DIO_PIN     32

// Wi-Fi & MQTT
const char* WIFI_SSID     = "Scutum";
const char* WIFI_PASS     = "00000111";
const char* MQTT_SERVER   = "broker.hivemq.com";
const int   MQTT_PORT     = 1883;
const char* MQTT_TOPIC    = "seat/1/status";


WiFiClient     wifiClient;
PubSubClient   mqttClient(wifiClient);
TM1637Display  display(CLK_PIN, DIO_PIN);

// State machine
enum State {
  IDLE,
  MOTION_CHECK,
  OCCUPIED_HUMAN,
  OCCUPIED_OBJECT
};
State state = IDLE;

// Timing
unsigned long stateStart      = 0;
const unsigned long checkInt  = 500;    // ultrasonic check interval
const unsigned long motionWin = 10000;  // 10 s window for PIR
const unsigned long settleDelay = 5000;   // 5 s before we start PIR checking

//seat release timeout variables
unsigned long leaveStart = 0;
const unsigned long leaveTimeout = 5000; // 5s buffer

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  
  // Configure each pin with a frequency of 5000Hz and an 8-bit resolution
  ledcAttach(RED_PIN, 5000, 8);
  ledcAttach(GREEN_PIN, 5000, 8);
  ledcAttach(BLUE_PIN, 5000, 8);


  display.setBrightness(7);//max ha ye, and this if for 7 segment display

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println("WiFi connected");

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  reconnectMQTT();
}

void loop() {
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();

  unsigned long now = millis();

  switch(state) {
    case IDLE: {
      // Idle: ultrasonic on, look for any object <40cm
      float d = measureDistance();
      display.showNumberDec((int)d, false);
      setLED(150,150,200); // grey

      if (d < 40.0) {
        state       = MOTION_CHECK;
        stateStart  = now;
        Serial.println("→ MOTION_CHECK");
      }
      delay(checkInt);
      break;
    }

  case MOTION_CHECK: {
    unsigned long elapsed = now - stateStart;
  
    // 1) During the first 5 s, do nothing but show grey
    if (elapsed < settleDelay) {
      setLED(150,150,200); // grey
      display.showNumberDec(0, false); // or keep last distance
      delay(50);
      break;
    }
  
    // 2) Now start PIR window logic
    unsigned long pirElapsed = elapsed - settleDelay;
    setLED(150,150,200); // still grey until resolved
  
    if (digitalRead(PIR_PIN) == HIGH && pirElapsed <= motionWin) {
      state = OCCUPIED_HUMAN;
      mqttClient.publish(MQTT_TOPIC, "1");//1 means human ha seat pa
      Serial.println("  PIR → HUMAN");
    }
    else if (pirElapsed > motionWin) {
      state = OCCUPIED_OBJECT;
      mqttClient.publish(MQTT_TOPIC, "41");//no human
      Serial.println("  NO PIR → OBJECT");
    }
    delay(50);
    break;
  }
  case OCCUPIED_HUMAN: {
    float d = measureDistance();
    display.showNumberDec((int)d, false);
    setLED(0,255,0); // green
  
    if (d >= 40.0) {
      if (leaveStart == 0) {
        leaveStart = now;       // start buffer
      } else if (now - leaveStart >= leaveTimeout) {
        state = IDLE;
        leaveStart = 0;
        Serial.println("← IDLE (human left)");
        mqttClient.publish(MQTT_TOPIC, "41");//no human
        Serial.println("seat realesed status published");
      }
    } else {
      leaveStart = 0;          // back within range
    }
    delay(checkInt);
    break;
  }
  
  case OCCUPIED_OBJECT: {
    float d = measureDistance();
    display.showNumberDec((int)d, false);
    setLED(255,0,0); // red
  
    if (d >= 40.0) {
      if (leaveStart == 0) {
        leaveStart = now;
      } else if (now - leaveStart >= leaveTimeout) {
        state = IDLE;
        leaveStart = 0;
        Serial.println("← IDLE (object removed)");
        mqttClient.publish(MQTT_TOPIC, "41");//no human
        Serial.println("seat realesed status published");
      }
    } else {
      leaveStart = 0;
    }
    delay(checkInt);
    break;
  }
  }
}

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
  
      long dur = pulseIn(ECHO_PIN, HIGH, 25000); // 25 ms timeout
      float dist = dur * 0.0343f / 2.0f;
  
      // Accept only reasonable distances (say, 2 cm to 200 cm)
      if (dur > 0 && dist >= 2.0 && dist <= 200.0) {
        validReadings[count++] = dist;
      }
  
      delay(10);
    }
  
    // Sort valid readings
    for (int i = 0; i < targetSamples - 1; i++) {
      for (int j = i + 1; j < targetSamples; j++) {
        if (validReadings[i] > validReadings[j]) {
          float temp = validReadings[i];
          validReadings[i] = validReadings[j];
          validReadings[j] = temp;
        }
      }
    }
  
    return validReadings[targetSamples / 2]; // median
  }

void setLED(int r, int g, int b) {
  // Set the PWM duty cycle for each channel with the calculated values
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
