/*
 * Integrated Seat Node with Firebase Firestore
 * - Controls LEDs, Sensors, MQTT
 * - Uploads session logs to Firestore on seat exit
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <TM1637Display.h>
#include <esp32-hal-ledc.h>
#include <ArduinoJson.h>
#include <time.h> 

// --- FIREBASE LIBRARIES ---
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>

// ----------------------- PIN DEFINITIONS -------------------
#define TRIG_PIN    18
#define ECHO_PIN    19
#define PIR_PIN     33
#define RED_PIN     26
#define GREEN_PIN   27
#define BLUE_PIN    25
#define CLK_PIN     32
#define DIO_PIN     21

// ----------------------- CREDENTIALS -----------------------
#define WIFI_SSID       "Scutum"
#define WIFI_PASS       "00000111"

// MQTT
#define MQTT_SERVER     "broker.hivemq.com"
#define MQTT_PORT       1883

// FIREBASE & PROJECT
#define API_KEY                 "AIzaSyC4ErKsA8zzGaOYLCkuJwS3f-8hTU6J-Pk"
#define FIREBASE_PROJECT_ID     "seatsync-fdd39"
#define FIREBASE_USER_EMAIL     "esp@test.me"
#define FIREBASE_USER_PASSWORD  "123456"

// ----------------------- TOPICS & CONSTANTS ------------------
#define SEAT_INDEX 1
const char* MQTT_TOPIC    = "seat/1/status";
const char* colorListenTopic = "seat/1/color";
const char* occupancyDurationListenTopic = "seat/1/occupancy";
const char* seatHoldDurationListenTopic = "seat/1/hold";
const char* otpRequestTopic = "seat/1/otp_request";
const char* otpResponseTopic = "seat/1/otp";

// ----------------------- OBJECTS -----------------------
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
TM1637Display display(CLK_PIN, DIO_PIN);

// Firebase Objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool firebaseReady = false;

// ----------------------- STATE VARIABLES -----------------------
unsigned long sitting_start_time = 0;
time_t session_start_epoch = 0; // To store the real Date/Time when sitting started

struct Color {
  int r, g, b;
  Color() : r(255), g(255), b(255) {}
  Color(int rr, int gg, int bb) : r(rr), g(gg), b(bb) {}
};

enum SeatStatus {
  AVAILABLE, OCCUPIED, ON_HOLD, UNAUTHORIZED_OCCUPIED,
  BOOKING_IN_PROGRESS, RESERVED, BLOCKED, OCCUPIED_BY_OBJECT
};

enum State {
  IDLE, MOTION_CHECK, OCCUPIED_HUMAN, OCCUPIED_OBJECT
};

State state = IDLE;
SeatStatus status = AVAILABLE;

String currentOTP = "";
unsigned long otpGeneratedAt = 0;
const unsigned long OTP_TTL = 60000UL;

String requestedBy = "null";

// HOLD variables
bool holdActive = false;
unsigned long holdExpiresAt = 0;

// Timing & counters
unsigned long stateStart = 0;
const unsigned long checkInterval = 200UL;
const unsigned long motionWindow = 10000UL;
const unsigned long settleDelay = 5000UL;
const int motionCountInit = 3;
int motionCount = motionCountInit;

unsigned long lastDistanceCheck = 0;
float lastMeasuredDistance = 999.0f;

unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 6000UL;

unsigned long leaveStart = 0;
const unsigned long leaveTimeout = 5000UL;

// Moved this UP here so 'ensureMqttConnected' can see it
unsigned long lastMqttAttempt = 0; 

// ----------------------- FUNCTION DECLARATIONS -----------------------
float measureDistance();
void setLED(int r, int g, int b);
Color getSeatColor(SeatStatus s);
String generateOTP();
void showOTP(const String &otp);
void handleOtpRequest(const String &uid);
void handleHoldMessage(const String &payload);
void uploadLogToFirestore(String uid, long durationSec, time_t startTime);
String getISOTime(time_t epoch);

// ----------------------- FIREBASE HELPER -----------------------
// Helper to format time as ISO8601 for Firestore Timestamp
String getISOTime(time_t epoch) {
  struct tm timeinfo;
  gmtime_r(&epoch, &timeinfo);
  char buffer[30];
  strftime(buffer, 30, "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buffer);
}

void uploadLogToFirestore(String uid, long durationSec, time_t startTime) {
  if (!Firebase.ready()) return;

  Serial.println("[FIREBASE] Uploading session log...");
  
  FirebaseJson content;
  // 1. Seat ID
  content.set("fields/seat_id/stringValue", "Seat_01");
  
  // 2. User ID
  content.set("fields/user_id/stringValue", uid);
  
  // 3. Duration
  content.set("fields/duration_seconds/integerValue", durationSec);
  
  // 4. Start Time (Real Date/Time)
  String timeStr = getISOTime(startTime);
  content.set("fields/start_time/timestampValue", timeStr);

  // Document Path: seat_info/log_{millis}
  String documentPath = "seat_info/log_" + String(millis());

  if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "" /*default db*/, documentPath.c_str(), content.raw())) {
    Serial.println("[FIREBASE] Upload success!");
    Serial.println(fbdo.payload());
  } else {
    Serial.println("[FIREBASE] Upload failed: " + fbdo.errorReason());
  }
}

// ----------------------- MQTT CALLBACK -----------------------
void mqttCallback(char* topic, byte* message, unsigned int length) {
  String incomingTopic = String(topic);
  String payload;
  payload.reserve(length + 1);

  for (unsigned int i = 0; i < length; ++i) payload += (char)message[i];

  Serial.println("[MQTT] in -> " + incomingTopic + " : " + payload);

  if (incomingTopic == String(otpRequestTopic)) {
    handleOtpRequest(payload);
  } 
  else if (incomingTopic == String(occupancyDurationListenTopic)) {
    Serial.println("[MQTT] occupancy duration msg: " + payload);
  } 
  else if (incomingTopic == String(colorListenTopic)) {
    Serial.println("[MQTT] color msg: " + payload);
  }
  else if (incomingTopic == String(seatHoldDurationListenTopic)) {
    // handle hold JSON { "uid":"...", "duration": <seconds> }
    handleHoldMessage(payload);
  }
}

// ----------------------- LOGIC HANDLERS -----------------------
String generateOTP() {
  uint32_t raw = esp_random();
  int otp = (raw % 9000) + 1000;
  return String(otp);
}

void showOTP(const String &otp) {
  display.showNumberDec(otp.toInt(), false);
}

// ------------otp handler -----------
void handleOtpRequest(const String &uid) {
  // Save requester UID
  requestedBy = uid;
  Serial.println("[OTP] request from: " + requestedBy);

  // Generate, show and publish OTP
  currentOTP = generateOTP();
  otpGeneratedAt = millis();
  showOTP(currentOTP);

  // Reply with OTP to app
  mqttClient.publish(otpResponseTopic, currentOTP.c_str());

  // Set booking-in-progress state & update LED and status topic
  status = BOOKING_IN_PROGRESS;
  Color c = getSeatColor(status);
  setLED(c.r, c.g, c.b);

  mqttClient.publish(MQTT_TOPIC, "4"); // booking-in-progress code
  Serial.println("[OTP] Generated & published: " + currentOTP);
}

//---------------hold handler -------------
void handleHoldMessage(const String &payload) {
  Serial.println("[HOLD] rx -> " + payload);

  // Parse JSON
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.println("[HOLD] JSON parse error: " + String(err.c_str()));
    return;
  }

  String uid = doc["uid"] | String("");
  long durationSec = doc["duration"] | -1L;

  if (uid.length() == 0 || durationSec < 0) {
    Serial.println("[HOLD] invalid payload fields");
    return;
  }

  // Check UID match
  if (uid != requestedBy) {
    // UID mismatch => publish failure "1"
    mqttClient.publish(seatHoldDurationListenTopic, "1");
    Serial.println("[HOLD] UID mismatch, published 1");
    return;
  }

  // UID matched
  if (durationSec == 0) {
    // Stop hold request
    if (holdActive) {
      holdActive = false;
      holdExpiresAt = 0;
      status = AVAILABLE;
      mqttClient.publish(seatHoldDurationListenTopic, "0"); // ack
      mqttClient.publish(MQTT_TOPIC, "0"); // status available
      Serial.println("[HOLD] stopped by user, ack 0, status->0");
    } else {
      // not active but user wants to stop - still ack success
      mqttClient.publish(seatHoldDurationListenTopic, "0");
      Serial.println("[HOLD] stop requested but hold not active, ack 0");
    }
    return;
  }

  // Start hold for durationSec seconds
  unsigned long now = millis();
  holdActive = true;
  holdExpiresAt = now + (unsigned long)durationSec * 1000UL;
  status = ON_HOLD;
  mqttClient.publish(seatHoldDurationListenTopic, "0"); // ack success
  mqttClient.publish(MQTT_TOPIC, "2"); // publish ON_HOLD
  Serial.println("[HOLD] started for uid=" + uid + " dur=" + String(durationSec) + "s, ack 0");
}

float measureDistance() {
  const int targetSamples = 7;
  float validReadings[targetSamples];
  int count = 0;

  while (count < targetSamples) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long dur = pulseIn(ECHO_PIN, HIGH, 25000);
    float dist = dur * 0.0343f / 2.0f;
    if (dur > 0) validReadings[count++] = dist;
    delay(10);
  }
  // Bubble sort for median
  for (int i = 0; i < targetSamples - 1; i++) {
    for (int j = i + 1; j < targetSamples; j++) {
      if (validReadings[i] > validReadings[j]) {
        float tmp = validReadings[i];
        validReadings[i] = validReadings[j];
        validReadings[j] = tmp;
      }
    }
  }
  return validReadings[targetSamples / 2];
}

void setLED(int r, int g, int b) {
  ledcWrite(0, r); ledcWrite(1, g); ledcWrite(2, b);
}

Color getSeatColor(SeatStatus s) {
  switch (s) {
    case AVAILABLE: return Color(100, 255, 100);
    case OCCUPIED: return Color(0, 255, 0);
    case ON_HOLD: return Color(0, 128, 255);
    case UNAUTHORIZED_OCCUPIED: return Color(255, 0, 0);
    case BOOKING_IN_PROGRESS: return Color(128, 0, 255);
    case OCCUPIED_BY_OBJECT: return Color(255, 140, 0);
    default: return Color(255, 255, 255);
  }
}

void ensureMqttConnected() {
  if (mqttClient.connected()) return;
  unsigned long now = millis();
  if (now - lastMqttAttempt > 2000UL) {
     lastMqttAttempt = now;
     String clientId = "ESP32Client_" + String((uint64_t)ESP.getEfuseMac(), HEX);
     if (mqttClient.connect(clientId.c_str())) {
        mqttClient.setCallback(mqttCallback);
        mqttClient.subscribe(otpRequestTopic);
        mqttClient.subscribe(occupancyDurationListenTopic);
        mqttClient.subscribe(colorListenTopic);
        mqttClient.subscribe(seatHoldDurationListenTopic);
        mqttClient.publish(MQTT_TOPIC, "0");
     }
  }
}

// ----------------------- SETUP -----------------------
void setup() {
  Serial.begin(115200);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);

  ledcSetup(0, 5000, 8); ledcAttachPin(RED_PIN, 0);
  ledcSetup(1, 5000, 8); ledcAttachPin(GREEN_PIN, 1);
  ledcSetup(2, 5000, 8); ledcAttachPin(BLUE_PIN, 2);

  display.setBrightness(7);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WIFI] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("."); delay(300);
  }
  Serial.println("\n[WIFI] Connected: " + WiFi.localIP().toString());

  // 1. SYNC TIME (Important for Date logging)
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  Serial.print("[TIME] syncing");
  time_t now;
  while ((now = time(nullptr)) < 100000) {
    Serial.print("."); delay(500);
  }
  Serial.println("\n[TIME] Synced!");

  // 2. SETUP FIREBASE
  config.api_key = API_KEY;
  auth.user.email = FIREBASE_USER_EMAIL;
  auth.user.password = FIREBASE_USER_PASSWORD;
  config.token_status_callback = tokenStatusCallback;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  Color c = getSeatColor(status);
  setLED(c.r, c.g, c.b);
  lastDistanceCheck = millis();
  lastPublishTime = millis();
}

// ----------------------- LOOP -----------------------
void loop() {
  ensureMqttConnected();
  if (mqttClient.connected()) mqttClient.loop();

  unsigned long now = millis();

  // HOLD Logic
  if (holdActive && now >= holdExpiresAt) {
    holdActive = false; holdExpiresAt = 0;
    status = AVAILABLE;
    mqttClient.publish(MQTT_TOPIC, "0");
  }

  // OTP Expiry
  if (currentOTP.length() > 0 && (now - otpGeneratedAt >= OTP_TTL)) {
    currentOTP = ""; display.clear();
    mqttClient.publish(otpResponseTopic, "null");
    if (status == BOOKING_IN_PROGRESS) status = AVAILABLE;
    if (state != OCCUPIED_HUMAN) requestedBy = "null";
  }

  if (now - lastPublishTime >= publishInterval) {
    lastPublishTime = now;
    char buf[8]; snprintf(buf, sizeof(buf), "%d", (int)status);
    mqttClient.publish(MQTT_TOPIC, buf);
  }

  if (holdActive) {
    // Always show hold color
    Color c = getSeatColor(ON_HOLD);
    setLED(c.r, c.g, c.b);
    delay(10);
    return;
  }
  // DISTANCE CHECK
  if (now - lastDistanceCheck >= checkInterval) {
    lastDistanceCheck = now;
    lastMeasuredDistance = measureDistance();

    if (lastMeasuredDistance < 24.0f && state == IDLE) {
      stateStart = now;
      state = MOTION_CHECK;
      motionCount = motionCountInit;
    }
  }

  // STATE MACHINE
  switch (state) {
    case IDLE:
      if (status != BOOKING_IN_PROGRESS) {
        Color c = getSeatColor(AVAILABLE);
        setLED(c.r, c.g, c.b);
      }
      break;

    case MOTION_CHECK: {
      unsigned long elapsed = now - stateStart;
      if (elapsed < settleDelay) break;
      unsigned long pirElapsed = elapsed - settleDelay;

      if (digitalRead(PIR_PIN) == HIGH && pirElapsed <= motionWindow && motionCount > 0) {
        motionCount--;
      } 
      else if (motionCount == 0) {
        // --- TRANSITION TO OCCUPIED ---
        state = OCCUPIED_HUMAN;
        
        // ** CRITICAL FIX: CAPTURE START TIME HERE ONLY ONCE **
        sitting_start_time = millis(); 
        time(&session_start_epoch); // Capture real NTP Date/Time
        
        Serial.println("[STATE] -> OCCUPIED_HUMAN");
      } 
      else if (pirElapsed > motionWindow) {
        state = OCCUPIED_OBJECT;
      }
      break;
    }

    case OCCUPIED_HUMAN: {
      float d = lastMeasuredDistance;

      if (requestedBy == "null" || requestedBy == "guest") {
        status = UNAUTHORIZED_OCCUPIED;
        mqttClient.publish(MQTT_TOPIC, "3");
      } else {
        status = OCCUPIED;
        mqttClient.publish(MQTT_TOPIC, "1");
        // NOTE: removed sitting_start_time = millis() from here to prevent reset
      }

      if (d >= 24.0f) {
        if (leaveStart == 0) leaveStart = now;
        else if (now - leaveStart >= leaveTimeout) {
          // --- USER HAS LEFT ---
          
          // 1. Calculate Duration
          unsigned long durationMs = now - sitting_start_time;
          long durationSec = durationMs / 1000;

          // 2. Upload to Firestore
          Serial.print("[LOG] User left. Duration: "); Serial.print(durationSec); Serial.println("s");
          
          // Only upload if it was a valid session (or even if unauthorized, your choice)
          // Here uploading for everyone not null.
          if (requestedBy != "null") {
             uploadLogToFirestore(requestedBy, durationSec, session_start_epoch);
          }

          // 3. Reset State
          status = AVAILABLE;
          mqttClient.publish(MQTT_TOPIC, "0");
          requestedBy = "null";
          state = IDLE;
          leaveStart = 0;
          Serial.println("[LEAVE] Human left -> AVAILABLE");
        }
      } else leaveStart = 0;
      break;
    }

    case OCCUPIED_OBJECT: {
      float d = lastMeasuredDistance;
      status = OCCUPIED_BY_OBJECT;
      mqttClient.publish(MQTT_TOPIC, "7");

      if (d >= 24.0f) {
        if (leaveStart == 0) leaveStart = now;
        else if (now - leaveStart >= leaveTimeout) {
          state = IDLE;
          status = AVAILABLE;
          mqttClient.publish(MQTT_TOPIC, "0");
          leaveStart = 0;
        }
      } else leaveStart = 0;
      break;
    }
  }

  Color c = getSeatColor(status);
  setLED(c.r, c.g, c.b);
  delay(10);
}
