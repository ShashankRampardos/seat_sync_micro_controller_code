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
const char* WIFI_SSID   = "Scutum"; //"POCO X6 PRO";
const char* WIFI_PASS   = "00000111";//"ansh4830";
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
State state = State :: IDLE;

class Color {
public:
    int r, g, b;

    Color(int red, int green, int blue) : r(red), g(green), b(blue) {}

    std::string toString() const {
        return "(" + std::to_string(r) + ", "
                   + std::to_string(g) + ", "
                   + std::to_string(b) + ")";
    }
};

// Enum for seat statuses
enum class SeatStatus {
    AVAILABLE,               // 0 Seat Available
    OCCUPIED,                // 1 Seat Occupied
    ON_HOLD,                 // 2 Seat Occupied but Put on Hold by Someone
    UNAUTHORIZED_OCCUPIED,   // 3 Unauthorized Seat Occupied
    BOOKING_IN_PROGRESS,     // 4 Seat Booking in Progress (OTP Displayed)
    RESERVED,                // 5 Reserved Seat
    BLOCKED,                 // 6 Blocked Seat (Damaged/Sensor Fault)
    OCCUPIED_BY_OBJECT       // 7 Seat Occupied by Object (not human)
};

// Get seat label
std::string getSeatLabel(SeatStatus status) {
    switch (status) {
        case SeatStatus::AVAILABLE:             return "Seat Available";
        case SeatStatus::OCCUPIED:              return "Seat Occupied";
        case SeatStatus::ON_HOLD:               return "On Hold";
        case SeatStatus::UNAUTHORIZED_OCCUPIED: return "Unauthorized Occupied";
        case SeatStatus::BOOKING_IN_PROGRESS:   return "Booking in Progress";
        case SeatStatus::RESERVED:              return "Reserved Seat";
        case SeatStatus::BLOCKED:               return "Blocked Seat";
        case SeatStatus::OCCUPIED_BY_OBJECT:    return "Occupied by Object";
        default:                                return "Unknown";
    }
}

// Get seat color in RGB
Color getSeatColor(SeatStatus status) {
    switch (status) {
        case SeatStatus::AVAILABLE:             return Color(100, 255, 100);  // light green
        case SeatStatus::OCCUPIED:              return Color(255, 0, 0);      // red
        case SeatStatus::ON_HOLD:               return Color(0, 128, 255);    // blue
        case SeatStatus::UNAUTHORIZED_OCCUPIED: return Color(255, 64, 128);   // pink/red
        case SeatStatus::BOOKING_IN_PROGRESS:   return Color(128, 0, 255);    // purple
        case SeatStatus::RESERVED:              return Color(255, 255, 0);    // yellow
        case SeatStatus::BLOCKED:               return Color(64, 64, 64);     // grey
        case SeatStatus::OCCUPIED_BY_OBJECT:    return Color(255, 140, 0);    // orange
        default:                                return Color(255, 255, 255);  // white fallback
    }
}

SeatStatus status = SeatStatus :: AVAILABLE;
std :: string uid = "null";

// Timing
unsigned long stateStart = 0;
const unsigned long checkInt     = 500;   // ultrasonic check interval
const unsigned long motionWin    = 10000; // 10 s window for PIR
const unsigned long settleDelay  = 5000;  // 5 s before we start PIR checking
unsigned long motionCount = 2;

//for refresh seat state in the mobile phone app, refresh rate is 6 seconds for now (may change later on if required))
unsigned long lastPublishTime = 0;  // track last periodic publish
const unsigned long publishInterval = 6000; // constant 6 seconds for now, badme dekh lange according to need

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
  // In your setup() function
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

    case MOTION_CHECK: {//do nothing for 5 seconds, and after 5 seconds start pir checking
      unsigned long elapsed = now - stateStart;//time passed in motion check state

      if (elapsed < settleDelay) {//for 5 seconds motion check nahi hoga
        setLED(150, 150, 200);
        display.showNumberDec(0, false);// show 0 on 4 digit seven segment
        delay(50);
        break;
      }

      unsigned long pirElapsed = elapsed - settleDelay;//time passing after 5 second motion delay
      setLED(150, 150, 200);

      if (digitalRead(PIR_PIN) == HIGH && pirElapsed <= motionWin) {//10 second motion check delay
        motionCount--;
      }else if(motionCount == 0){
        state = OCCUPIED_HUMAN;
        mqttClient.publish(MQTT_TOPIC, "1"); // 1 = human detected
        Serial.println("PIR → HUMAN");
        motionCount = 2;
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
      delay(1000);
      mqttClient.publish(MQTT_TOPIC,"1");
      Serial.println("sending 1");
      delay(500);
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
      delay(1000);
      mqttClient.publish(MQTT_TOPIC,"2");
      Serial.println("sending 2");
      delay(500);
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
    //here i wanna publish the status in every 6 seconds interval {Available, Occupied_human, Occupied_object}
   
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

    long dur = pulseIn(ECHO_PIN, HIGH, 25000);//ye bhul gaya kya tha, samaj lena dubara later
    float dist = dur * 0.0343f / 2.0f;

    if (dur > 0)// && dist >= 2.0 && dist <= 200.0
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
