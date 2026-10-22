/*
  =====================================================================
   MARS EXPLORATION ROVER — INTEGRATED FIRMWARE (ESP32)
  =====================================================================
  Combines:
    - GPS (NEO-6M via TinyGPS++)               -> UART2
    - ToF obstacle sensor (VL53L0X) + pan servo -> I2C + PWM
    - 6x DC gear motors on 2x BTS7960 drivers   -> left side / right side
      (front-left & front-right motors have quadrature encoders,
       the other 4 are power-only, wired in parallel per side)
    - MQ-5 gas sensor                            -> analog
    - DHT11 temp/humidity                        -> digital
    - WebSocket server (ESPAsyncWebServer)        -> live telemetry +
                                                      drive commands

  LIBRARIES REQUIRED (Arduino Library Manager):
    - ESPAsyncWebServer (+ AsyncTCP dependency)
    - ArduinoJson (v6)
    - TinyGPS++
    - Adafruit VL53L0X
    - ESP32Servo
    - DHT sensor library (Adafruit) + Adafruit Unified Sensor

  ---------------------------------------------------------------------
  PIN MAP (change here only — everything else references these)
  ---------------------------------------------------------------------
    LEFT  BTS7960   RPWM=18  LPWM=19  EN=5   (R_EN+L_EN tied to pin 5)
    RIGHT BTS7960   RPWM=27  LPWM=14  EN=23  (R_EN+L_EN tied to pin 23)

    LEFT  encoder (front-left motor)   A=25  B=26
    RIGHT encoder (front-right motor)  A=32  B=33

    GPS NEO-6M   ESP32 RX2=16 (<- GPS TX)   ESP32 TX2=17 (-> GPS RX)
    ToF VL53L0X  I2C  SDA=21  SCL=22
    Pan Servo (aims ToF for obstacle scan)  PIN=13
    DHT11                                   PIN=4
    MQ-5 analog out                         PIN=34

  All GNDs (ESP32, both BTS7960 boards, battery -, motors, sensors)
  MUST share a common ground.
  =====================================================================
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>
#include <DHT.h>

// ---------------------- Access Point credentials ----------------------
// The ESP32 creates its OWN WiFi network instead of joining your router.
// Connect your phone/laptop directly to this SSID to control the rover.
// Password must be at least 8 characters (ESP32 requirement).
const char* AP_SSID = "MarsRover-AP";
const char* AP_PASSWORD = "rover1234";

// ---------------------- Motor driver pins ---------------------------
#define LEFT_RPWM   18
#define LEFT_LPWM   19
#define LEFT_EN     5
#define RIGHT_RPWM  27
#define RIGHT_LPWM  14
#define RIGHT_EN    23

// ---------------------- Encoder pins --------------------------------
#define LEFT_ENC_A  25
#define LEFT_ENC_B  26
#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33

// ---------------------- Other sensors --------------------------------
#define GPS_RX 16
#define GPS_TX 17
#define TOF_SDA 21
#define TOF_SCL 22
#define SERVO_PIN 13
#define DHT_PIN 4
#define MQ5_PIN 34
#define BUZZER_PIN 15   // active buzzer - beeps while an obstacle is within threshold

// ---------------------- Encoder / gearbox constants -------------------
const int PPR = 7;
const int MULTIPLIER = 4;
const int GEAR_RATIO = 20;
const long COUNTS_PER_OUTPUT_REV = (long)PPR * MULTIPLIER * GEAR_RATIO; // 560
const unsigned long RPM_SAMPLE_MS = 100;

// ---------------------- Obstacle avoidance -----------------------------
const int OBSTACLE_THRESHOLD_MM = 600;    // 60 cm
const int SCAN_LEFT_ANGLE = 150;
const int SCAN_RIGHT_ANGLE = 30;
const int SCAN_CENTER_ANGLE = 90;
const unsigned long TOF_CHECK_MS = 120;
const unsigned long AVOID_TURN_MS = 400;  // how long to turn during an avoid maneuver
const unsigned long BACKUP_MS = 500;      // reverse duration before scanning ("2-3 steps" worth of travel)
const float AVOID_SPEED_SCALE = 0.55;     // maneuver speed as a fraction of normal driveSpeed
const int MIN_AVOID_SPEED = 110;          // floor so PWM doesn't drop below what the motors can actually move at

// ---------------------- Timers ------------------------------------------
const unsigned long TELEMETRY_MS = 300;
const unsigned long DHT_READ_MS = 2000;   // DHT11 max ~1 Hz
const unsigned long GAS_READ_MS = 500;
const unsigned long BUZZER_BEEP_MS = 150; // beep-beep toggle rate while obstacle present
const unsigned long WIFI_CHECK_MS = 5000; // how often to verify WiFi is still connected

// ============================ Objects ====================================
TinyGPSPlus gps;
HardwareSerial GPSSerial(2);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo panServo;
DHT dht(DHT_PIN, DHT11);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ============================ Shared state ================================
volatile long leftPulseCount = 0;
volatile long rightPulseCount = 0;
float leftWheelRPM = 0, rightWheelRPM = 0;

float lastLat = 0, lastLng = 0, lastAltitude = 0, lastGpsSpeed = 0;
int lastSatellites = 0;
bool gpsFixValid = false;

int lastDistanceMM = -1;
bool obstacleFlag = false;

float lastTemp = NAN, lastHumidity = NAN;
int lastGasRaw = 0;

String currentCommand = "stop";
int driveSpeed = 200;          // 0-255
bool autoAvoidEnabled = true;
bool avoidManeuverActive = false;
bool autoMode = false;         // true = autonomous wander, false = manual dashboard control

unsigned long tRPM = 0, tTelemetry = 0, tDHT = 0, tGas = 0, tToF = 0, tBuzzer = 0, tWifiCheck = 0;
bool buzzerOn = false;

// ============================ Connection watchdog =================================
// In Access Point mode the ESP32 IS the network, so there's no "reconnect
// to router" step needed. The real safety risk here is different: if the
// dashboard's WebSocket disconnects while the rover is driving, nothing
// should keep moving with no one able to send a stop command. This checks
// periodically and safety-stops if no dashboard is currently connected.
void updateConnectionWatchdog() {
  unsigned long now = millis();
  if (now - tWifiCheck < WIFI_CHECK_MS) return;
  tWifiCheck = now;

  if (ws.count() == 0 && currentCommand != "stop") {
    Serial.println("No dashboard connected — stopping rover for safety.");
    stopMotors();
    currentCommand = "stop";
    autoMode = false;
  }
}

// ============================ Buzzer (radar alarm) =================================
// Beeps on/off repeatedly while obstacleFlag is true - a classic
// proximity-alarm sound. Non-blocking, checked every loop() pass.
void updateBuzzer() {
  if (obstacleFlag) {
    unsigned long now = millis();
    if (now - tBuzzer >= BUZZER_BEEP_MS) {
      tBuzzer = now;
      buzzerOn = !buzzerOn;
      digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
    }
  } else if (buzzerOn) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerOn = false;
  }
}

// ============================ Encoder ISRs =================================
void IRAM_ATTR leftEncISR_A() {
  int a = digitalRead(LEFT_ENC_A);
  int b = digitalRead(LEFT_ENC_B);
  if (a == b) leftPulseCount++; else leftPulseCount--;
}
void IRAM_ATTR leftEncISR_B() {
  int a = digitalRead(LEFT_ENC_A);
  int b = digitalRead(LEFT_ENC_B);
  if (a != b) leftPulseCount++; else leftPulseCount--;
}
void IRAM_ATTR rightEncISR_A() {
  int a = digitalRead(RIGHT_ENC_A);
  int b = digitalRead(RIGHT_ENC_B);
  if (a == b) rightPulseCount++; else rightPulseCount--;
}
void IRAM_ATTR rightEncISR_B() {
  int a = digitalRead(RIGHT_ENC_A);
  int b = digitalRead(RIGHT_ENC_B);
  if (a != b) rightPulseCount++; else rightPulseCount--;
}

// ============================ Motor control =================================
void setLeftMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) { analogWrite(LEFT_LPWM, 0); analogWrite(LEFT_RPWM, pwm); }
  else          { analogWrite(LEFT_RPWM, 0); analogWrite(LEFT_LPWM, -pwm); }
}
void setRightMotor(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm >= 0) { analogWrite(RIGHT_LPWM, 0); analogWrite(RIGHT_RPWM, pwm); }
  else          { analogWrite(RIGHT_RPWM, 0); analogWrite(RIGHT_LPWM, -pwm); }
}
void stopMotors()      { setLeftMotor(0); setRightMotor(0); }
void driveForward(int s)  { setLeftMotor(s);  setRightMotor(s); }
void driveBackward(int s) { setLeftMotor(-s); setRightMotor(-s); }
void turnLeftPivot(int s)  { setLeftMotor(-s); setRightMotor(s); }
void turnRightPivot(int s) { setLeftMotor(s);  setRightMotor(-s); }

// Executes whatever the current commanded direction is, at driveSpeed
void applyCurrentCommand() {
  if (currentCommand == "forward")      driveForward(driveSpeed);
  else if (currentCommand == "backward") driveBackward(driveSpeed);
  else if (currentCommand == "left")     turnLeftPivot(driveSpeed);
  else if (currentCommand == "right")    turnRightPivot(driveSpeed);
  else                                   stopMotors();
}

// ============================ Obstacle avoidance =============================
int readDistanceMM() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) return measure.RangeMilliMeter;
  return -1; // out of range / invalid
}

// Blocking scan-and-turn maneuver — only triggered when an obstacle is
// actually detected in front, so a few hundred ms of blocking is acceptable
// for this safety-critical action.
//
// Sequence: stop -> back up (creates clearance so the pivot turn that
// follows doesn't scrape whatever triggered it) -> scan left/right with
// the servo -> turn toward the more open side -> resume.
void runAvoidManeuver() {
  avoidManeuverActive = true;
  stopMotors();

  // Slower speed for the backup + turn only — computed fresh from the
  // current driveSpeed each time, never overwrites driveSpeed itself, so
  // normal speed resumes automatically the instant the maneuver ends.
  int avoidSpeed = (int)(driveSpeed * AVOID_SPEED_SCALE);
  if (avoidSpeed < MIN_AVOID_SPEED) avoidSpeed = MIN_AVOID_SPEED;

  // Back up a short distance before turning, so there's room to pivot
  // without immediately re-triggering on the same obstacle.
  driveBackward(avoidSpeed);
  delay(BACKUP_MS);
  stopMotors();
  delay(80); // brief settle before the servo starts moving

  panServo.write(SCAN_LEFT_ANGLE);
  delay(200);   // just enough for the servo to settle
  int leftDist = readDistanceMM();

  panServo.write(SCAN_RIGHT_ANGLE);
  delay(280);   // this sweep is wider (150° to 30°) so it needs a bit more than the next one
  int rightDist = readDistanceMM();

  panServo.write(SCAN_CENTER_ANGLE);
  delay(150);

  if (leftDist < 0) leftDist = 3000;   // treat "out of range" as clear
  if (rightDist < 0) rightDist = 3000;

  if (leftDist > rightDist) {
    turnLeftPivot(avoidSpeed);
  } else {
    turnRightPivot(avoidSpeed);
  }
  delay(AVOID_TURN_MS);
  stopMotors();

  // In manual mode, force the user to issue a fresh command after an
  // avoid event. In autonomous mode, keep wandering forward instead.
  if (autoMode) {
    currentCommand = "forward";
    applyCurrentCommand();
  } else {
    currentCommand = "stop";
  }
  avoidManeuverActive = false;

  StaticJsonDocument<128> ev;
  ev["event"] = "obstacle_avoided";
  ev["left_mm"] = leftDist;
  ev["right_mm"] = rightDist;
  String out;
  serializeJson(ev, out);
  ws.textAll(out);
}

// ============================ WebSocket handling =============================
void handleCommand(const String& cmd) {
  // Any manual drive command coming from the dashboard cancels
  // autonomous wandering, so button presses always take priority.
  autoMode = false;
  currentCommand = cmd;
  if (cmd == "forward" && autoAvoidEnabled && obstacleFlag) {
    // Don't allow driving straight into a known obstacle;
    // trigger the avoid maneuver instead.
    runAvoidManeuver();
    return;
  }
  applyCurrentCommand();
}

// Switches between manual (dashboard button) control and autonomous
// wandering (drive forward continuously, let obstacle avoidance steer).
void setAutoMode(bool enabled) {
  autoMode = enabled;
  if (autoMode) {
    currentCommand = "forward";
    applyCurrentCommand();
  } else {
    currentCommand = "stop";
    stopMotors();
  }
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client #%u connected\n", client->id());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, data, len);
    if (err) return;

    if (doc.containsKey("cmd")) {
      String c = String((const char*)doc["cmd"]);
      Serial.print("Received command: ");
      Serial.println(c);
      handleCommand(c);
    }
    if (doc.containsKey("speed")) {
      driveSpeed = constrain((int)doc["speed"], 0, 255);
    }
    if (doc.containsKey("auto_avoid")) {
      autoAvoidEnabled = doc["auto_avoid"];
    }
    if (doc.containsKey("auto_mode")) {
      setAutoMode(doc["auto_mode"]);
    }
  }
}

// ============================ Sensor tasks ====================================
void updateGPS() {
  while (GPSSerial.available() > 0) gps.encode(GPSSerial.read());
  if (gps.location.isValid() && gps.location.isUpdated()) {
    lastLat = gps.location.lat();
    lastLng = gps.location.lng();
    lastSatellites = gps.satellites.value();
    lastAltitude = gps.altitude.meters();
    lastGpsSpeed = gps.speed.kmph();
    gpsFixValid = true;
  }
}

void updateRPM() {
  unsigned long now = millis();
  if (now - tRPM < RPM_SAMPLE_MS) return;
  unsigned long elapsed = now - tRPM;
  tRPM = now;

  noInterrupts();
  long lCount = leftPulseCount;  leftPulseCount = 0;
  long rCount = rightPulseCount; rightPulseCount = 0;
  interrupts();

  leftWheelRPM  = ((float)lCount / COUNTS_PER_OUTPUT_REV) * (60000.0 / elapsed);
  rightWheelRPM = ((float)rCount / COUNTS_PER_OUTPUT_REV) * (60000.0 / elapsed);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {   // print once a second, not every 100ms
    lastPrint = millis();
    Serial.printf("Encoder pulses -> L:%ld R:%ld | RPM -> L:%.1f R:%.1f\n",
                  lCount, rCount, leftWheelRPM, rightWheelRPM);
  }
}

void updateToF() {
  unsigned long now = millis();
  if (now - tToF < TOF_CHECK_MS) return;
  tToF = now;
  if (avoidManeuverActive) return; // servo is busy scanning

  int d = readDistanceMM();
  lastDistanceMM = d;
  obstacleFlag = (d > 0 && d < OBSTACLE_THRESHOLD_MM);

  // Safety net: if driving forward and an obstacle suddenly appears,
  // react immediately instead of waiting for the next command.
  if (obstacleFlag && currentCommand == "forward" && autoAvoidEnabled) {
    runAvoidManeuver();
  }
}

void updateDHT() {
  unsigned long now = millis();
  if (now - tDHT < DHT_READ_MS) return;
  tDHT = now;
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) { lastHumidity = h; lastTemp = t; }
}

void updateGas() {
  unsigned long now = millis();
  if (now - tGas < GAS_READ_MS) return;
  tGas = now;
  lastGasRaw = analogRead(MQ5_PIN);
}

void broadcastTelemetry() {
  unsigned long now = millis();
  if (now - tTelemetry < TELEMETRY_MS) return;
  tTelemetry = now;

  StaticJsonDocument<512> doc;
  doc["lat"] = lastLat;
  doc["lng"] = lastLng;
  doc["alt_m"] = lastAltitude;
  doc["gps_speed_kmh"] = lastGpsSpeed;
  doc["sats"] = lastSatellites;
  doc["gps_fix"] = gpsFixValid;
  doc["distance_mm"] = lastDistanceMM;
  doc["obstacle"] = obstacleFlag;
  doc["gas_raw"] = lastGasRaw;
  doc["temp_c"] = lastTemp;
  doc["humidity"] = lastHumidity;
  doc["left_rpm"] = leftWheelRPM;
  doc["right_rpm"] = rightWheelRPM;
  doc["command"] = currentCommand;
  doc["speed_pwm"] = driveSpeed;
  doc["auto_avoid"] = autoAvoidEnabled;
  doc["auto_mode"] = autoMode;

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

// ================================= Setup / Loop ==================================
void setup() {
  Serial.begin(115200);
  delay(300);

  // Motor pins
  pinMode(LEFT_RPWM, OUTPUT);  pinMode(LEFT_LPWM, OUTPUT);  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT); pinMode(RIGHT_LPWM, OUTPUT); pinMode(RIGHT_EN, OUTPUT);
  digitalWrite(LEFT_EN, HIGH);
  digitalWrite(RIGHT_EN, HIGH);

  // Encoder pins + interrupts
  pinMode(LEFT_ENC_A, INPUT_PULLUP);  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), leftEncISR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightEncISR_B, CHANGE);

  // GPS
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // ToF
  Wire.begin(TOF_SDA, TOF_SCL);
  if (!lox.begin()) {
    Serial.println("ERROR: VL53L0X not detected! Disabling auto-avoid so it doesn't block driving.");
    autoAvoidEnabled = false;
  }

  // Servo
  panServo.attach(SERVO_PIN);
  panServo.write(SCAN_CENTER_ANGLE);

  // DHT11
  dht.begin();

  // MQ-5
  pinMode(MQ5_PIN, INPUT);

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // WiFi — Access Point mode: the ESP32 broadcasts its own network instead
  // of joining an external router. Connect your phone/laptop directly to
  // AP_SSID, then browse to the fixed IP printed below (normally 192.168.4.1).
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  delay(200);
  Serial.println("Access Point started.");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("Rover IP (connect your dashboard here): ");
  Serial.println(WiFi.softAPIP());

  // WebSocket server — this is the only interface the rover needs now.
  // The Flutter app connects directly to ws://<esp32-ip>/ws, no web
  // page involved. This "/" route is just a quick sanity check you can
  // open in any browser to confirm the ESP32 is alive and reachable.
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain",
      "Mars Rover ESP32 online. WebSocket control at /ws — use the Flutter app.");
  });
  server.begin();

  tRPM = tTelemetry = tDHT = tGas = tToF = millis();
}

void loop() {
  updateGPS();      // continuous, non-blocking (drains UART buffer)
  updateRPM();       // every 100 ms
  updateToF();        // every 120 ms, may trigger a blocking avoid maneuver
  updateDHT();          // every 2000 ms
  updateGas();            // every 500 ms
  updateBuzzer();          // every loop pass, non-blocking beep toggle
  updateConnectionWatchdog(); // every 5000 ms, safety-stops if no dashboard connected
  broadcastTelemetry();      // every 300 ms
  ws.cleanupClients();
}
