#include <WiFi.h>
#include <PubSubClient.h>
#include <esp32-hal-ledc.h>
#include <TM1637Display.h>
#include <ArduinoJson.h>


// Pin Definitions
#define TRIG_PIN    18
#define ECHO_PIN    19
#define PIR_PIN     33
#define RED_PIN     26
#define GREEN_PIN   27
#define BLUE_PIN    25
#define CLK_PIN     32
#define DIO_PIN     21

#define SEAT_INDEX 1

// Wi-Fi & MQTT
const char* WIFI_SSID   = "Scutum"; //"POCO X6 PRO";
const char* WIFI_PASS   = "00000111";//"ansh4830";
const char* MQTT_SERVER = "broker.hivemq.com";
const int   MQTT_PORT   = 1883;

//mqtt topics, some are for subscribe and listen and some are for publishing 
const char* MQTT_TOPIC  = "seat/1/status";//for publishing seat status {0,1,2}
char* colorListenTopic = "seat/1/color";      // "255,0,0"

char* occupancyDurationListenTopic = "seat/1/occupancy";  // "60"   (in minutes maybe)
char* seatHoldDurationListenTopic = "seat/1/hold";       // "5"    (short break duration)
char* otpRequestTopic = "seat/1/otp_request";//listen to otp request and get user uid
char* otpResponseTopic = "seat/1/otp";//publish otp to app

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
TM1637Display display(CLK_PIN, DIO_PIN);


// --- types: put these BEFORE any functions that use them ---
struct Color {
  int r;
  int g;
  int b;
  Color() : r(255), g(255), b(255) {}
  Color(int rr, int gg, int bb) : r(rr), g(gg), b(bb) {}
};

enum SeatStatus {
  AVAILABLE,
  OCCUPIED,
  ON_HOLD,
  UNAUTHORIZED_OCCUPIED,
  BOOKING_IN_PROGRESS,
  RESERVED,
  BLOCKED,
  OCCUPIED_BY_OBJECT
};

// State machine
enum State {
  IDLE,
  MOTION_CHECK,
  OCCUPIED_HUMAN,
  OCCUPIED_OBJECT,
  DO_NOTHING
};
State state = State :: IDLE;


String currentOTP = "";
unsigned long otpGeneratedAt = 0;
unsigned long holdStartedAt = 0;

unsigned long holdTime = LLONG_MAX;
String requestedBy = "null";
bool otpRequested = false;
SeatStatus status = AVAILABLE;


String generateOTP() {//generates true random numbers
  // 1. Get a true random 32-bit number from the hardware
  uint32_t raw_random = esp_random();

  // 2. Scale this number to your range (1000-9999)
  //    (raw_random % 9000) gives a number between 0 and 8999
  //    Adding 1000 shifts the range to 1000-9999
  int otp = (raw_random % 9000) + 1000;
  
  return String(otp);
}

void showOTP(String otp) {
  display.showNumberDec(otp.toInt(), false);
}

void mqttCallback(char* topic, byte* message, unsigned int length) {
  String incomingTopic = String(topic);
  String payload = "";

  for (int i = 0; i < length; i++) {
    payload += (char)message[i];
  }

  Serial.println("MQTT Message Received:");
  Serial.println("Topic: " + incomingTopic);
  Serial.println("Payload: " + payload);

  if (incomingTopic == "seat/1/otp_request") {
    requestedBy = payload;  // directly the UID string
    otpRequested = true;
    if(requestedBy == "guest"){
      Serial.println("OTP requested by UID: " + requestedBy);
  
      currentOTP = generateOTP();
      otpGeneratedAt = millis();//it is useful for expiring of otp
      showOTP(currentOTP);
  
      String msg = currentOTP;
      mqttClient.publish(otpResponseTopic, msg.c_str());
      status = BOOKING_IN_PROGRESS;//booking in progress
      Color c = getSeatColor(status);
      setLED(c.r,c.g,c.b);
      mqttClient.publish(MQTT_TOPIC,"4");//booling in progress
      Serial.println("OTP Generated and Published: " + currentOTP);
    }
  } else if(incomingTopic == "seat/1/hold") {
    Serial.print("Message arrived on topic: ");
    Serial.println(topic);
  
    // Convert payload to Arduino core String
    String message;
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }
  
    Serial.print("Payload: ");
    Serial.println(message);
  
    // === Parse JSON ===
    StaticJsonDocument<200> doc;  // size in bytes, adjust if message is large
    DeserializationError error = deserializeJson(doc, message);
  
    if (error) {
      Serial.print("JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }
  
    // Access JSON fields
    String uid = doc["uid"];  // user uid
    unsigned long duration = doc["duration"];  // duration time in milli seconds
  
    Serial.println("Parsed values:");
    Serial.println(uid);
    Serial.println(duration);
  
    if(requestedBy == uid) {
       holdTime = duration;
       status = ON_HOLD;
       state = IDLE;
       holdStartedAt = millis();
       mqttClient.publish(seatHoldDurationListenTopic,"0");//successful hold attempt
       mqttClient.publish(MQTT_TOPIC,"2");
    } else if (duration == 0 && requestedBy == uid) {// hold stop signal
       holdTime = 0;
    } else {
      mqttClient.publish(seatHoldDurationListenTopic,"-1");// -1 for invalid hold attempt, hold failed
      holdTime = LLONG_MAX;
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

      // Subscribe to OTP request topic
      mqttClient.subscribe(otpRequestTopic);
      //subscribe to hold topic 
      mqttClient.subscribe(seatHoldDurationListenTopic);
      Serial.println("Subscribed to OTP Request Topic: " + String(otpRequestTopic));
    } else {
      Serial.print("MQTT fail, rc=");
      Serial.print(mqttClient.state());
      delay(2000);
    }
  }
}


// Get seat label
String getSeatLabel(SeatStatus status) {
    switch (status) {
        case AVAILABLE:             return "Seat Available";
        case OCCUPIED:              return "Seat Occupied";
        case ON_HOLD:               return "On Hold";
        case UNAUTHORIZED_OCCUPIED: return "Unauthorized Occupied";
        case BOOKING_IN_PROGRESS:   return "Booking in Progress";
        case RESERVED:              return "Reserved Seat";
        case BLOCKED:               return "Blocked Seat";
        case OCCUPIED_BY_OBJECT:    return "Occupied by Object";
        default:                                return "Unknown";
    }
}

//Get seat color in RGB
Color getSeatColor(SeatStatus status) {
    switch (status) {
        case AVAILABLE:             return Color(100, 255, 100);  // light green
        case OCCUPIED:              return Color(0, 255, 0);      // green
        case ON_HOLD:               return Color(0, 128, 255);    // blue
        case UNAUTHORIZED_OCCUPIED: return Color(255, 0, 0);   // red
        case BOOKING_IN_PROGRESS:   return Color(128, 0, 255);    // purple
        case RESERVED:              return Color(255, 255, 0);    // yellow
        case BLOCKED:               return Color(64, 64, 64);     // grey
        case OCCUPIED_BY_OBJECT:    return Color(255, 140, 0);    // orange
        default:                                return Color(255, 255, 255);  // white fallback
    }
}

// SeatStatus status = AVAILABLE;

// Timing
unsigned long stateStart = 0;
const unsigned long checkInt     = 1500;   // ultrasonic check interval
const unsigned long motionWin    = 10000; // 10 s window for PIR
const unsigned long settleDelay  = 5000;  // 5 s before we start PIR checking
unsigned long motionCount = 3;

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
  mqttClient.setCallback(mqttCallback);
  reconnectMQTT();

  //randomSeed(esp_random());
}

unsigned long reservedAt = 0;
// ---------- Main Loop ----------
void loop() {
  if (!mqttClient.connected()) reconnectMQTT();
 
  mqttClient.loop();

  if ((currentOTP != "" && millis() - otpGeneratedAt > 60000) || status == OCCUPIED) {
    currentOTP = "";
    display.clear();
    mqttClient.publish(otpResponseTopic,"null");
    Serial.println("OTP expired");
    if(status == BOOKING_IN_PROGRESS){
      status = AVAILABLE;
    }
    if(state != OCCUPIED_HUMAN){
      requestedBy = "null";
    }
  }
  
  if(holdTime != LLONG_MAX && millis() - holdStartedAt > holdTime) {
    state = IDLE;
    status = RESERVED;// until user sits
    reservedAt = millis();
  }
  if(millis() - reservedAt > 10000) {
    status = AVAILABLE;
  }
  unsigned long now = millis();

  switch (state) {
    case IDLE: { 
     if(status != BOOKING_IN_PROGRESS){
      mqttClient.publish(MQTT_TOPIC,"0");
      Color c = getSeatColor(AVAILABLE);
      setLED(c.r,c.g,c.b); // grey 
     }
      float d = measureDistance();
     
     //display.showNumberDec((int)d, false);
     
     if (d < 24.0) {
       state = MOTION_CHECK;
        stateStart = now;
         Serial.println("→ MOTION_CHECK");
      }
      //delay(checkInt);
      break;
    }

    case MOTION_CHECK: {//do nothing for 5 seconds, and after 5 seconds start pir checking
      unsigned long elapsed = now - stateStart;//time passed in motion check state

      if (elapsed < settleDelay) {//for 5 seconds motion check nahi hoga
        //setLED(150, 150, 200);
        //display.showNumberDec(0, false);// show 0 on 4 digit seven segment
        Serial.print(0);
        delay(50);
        break;
      }

      unsigned long pirElapsed = elapsed - settleDelay;//time passing after 5 second motion delay
      // if(status != UNAUTHORIZED_OCCUPIED)
      //   setLED(150, 150, 200);

      if (digitalRead(PIR_PIN) == HIGH && pirElapsed <= motionWin && motionCount > 0) {//10 second motion check delay
        motionCount--;
      }else if(motionCount == 0){
        state = OCCUPIED_HUMAN;
        
        if(requestedBy == "null" || requestedBy == "guest" || status == ON_HOLD){
          status = UNAUTHORIZED_OCCUPIED;
          mqttClient.publish(MQTT_TOPIC, "3"); // 3 = unauthorized human detected
          Serial.println("PIR → unauthorized HUMAN");
        }else{
          if(status == ON_HOLD){
           mqttClient.publish(MQTT_TOPIC,"2");// on hold
          } else {
          status = OCCUPIED;
          mqttClient.publish(MQTT_TOPIC,"1");
          }
          Color c = getSeatColor(status);
          setLED(c.r,c.g,c.b);
        }
        motionCount = 3;
      } else if (pirElapsed > motionWin) {
        state = OCCUPIED_OBJECT;
        if(status != ON_HOLD)
         status = OCCUPIED_BY_OBJECT;

        mqttClient.publish(MQTT_TOPIC, "7"); // object detected
        Serial.println("NO PIR → OBJECT");
      }
      delay(50);
      break;
    }

    case OCCUPIED_HUMAN: {
      float d = measureDistance();
      //display.showNumberDec((int)d, false);
      Serial.print((int)d);
      //setLED(0, 255, 0); // green
      //delay(1000);
      if(requestedBy == "null" || requestedBy == "guest" || status == UNAURTHORZED_OCCUPIED){
        status = UNAUTHORIZED_OCCUPIED;
        mqttClient.publish(MQTT_TOPIC,"3");
        Serial.println("sending 3");
      }else{
        status = OCCUPIED;
        mqttClient.publish(MQTT_TOPIC,"1");
        Serial.println("sending 1");
      }
      delay(100);
      if (d >= 24.0) {
        if (leaveStart == 0) leaveStart = now;
        else if (now - leaveStart >= leaveTimeout) {
          if(holdDuration != LLONGMAX){
           state = IDLE;//unauthorized person gone
           status = ON_HOLD;
           mqttClient.publish(MQTT_TOPIC, "2");
          } else {
          state = IDLE;
          status = AVAILABLE;
          leaveStart = 0;
          Color c = getSeatColor(AVAILABLE);
          setLED(c.r,c.g,c.b);
          Serial.println("← IDLE (human left)");
          mqttClient.publish(MQTT_TOPIC, "0");
          Serial.println("seat released status published");
          requestedBy = "null";
          }
        }
      } else leaveStart = 0;

      break;
    }

    case OCCUPIED_OBJECT: {
      float d = measureDistance();
      //display.showNumberDec((int)d, false);
      Serial.print((int)d);
      Color c = getSeatColor(OCCUPIED_BY_OBJECT);
      setLED(c.r,c.g,c.b);
      //delay(1000);
      mqttClient.publish(MQTT_TOPIC,"7");
      Serial.println("sending 7");
      delay(100);
      if (d >= 24.0) {
        if (leaveStart == 0) leaveStart = now;
        else if (now - leaveStart >= leaveTimeout) {
          state = IDLE;
          status = AVAILABLE;
          leaveStart = 0;
          Serial.println("← IDLE (object removed)");
          mqttClient.publish(MQTT_TOPIC, "0");//idle state code is 0
          Serial.println("seat released status published");
        }
      } else leaveStart = 0;

      //delay(checkInt);//1500 milli second delay
      break;
    }
    default: {
      Serial.print("defalut case");
    }
  }
  Color c = getSeatColor(status);
  setLED(c.r,c.g,c.b);
  delay(checkInt);//1500 milli seconds delay
}

