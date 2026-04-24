// ===================================================================
// SMART MOTION BIN - ESP32 LINE FOLLOWER WITH FIREBASE INTEGRATION
// ===================================================================
// Integrates with:
// - Firebase Realtime Database (motion-bin)
// - Web Dashboard (Real-time sync)
// - PID Line Following
// - Point/Marker Detection
// - Obstacle Avoidance
// ===================================================================
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Firebase_ESP_Client.h>
#include <time.h>

// ===== Wi-Fi CREDENTIALS =====
const char* ssid = "Inuwa's A16";
const char* password = "rrrrrrrr";

// ===== FIREBASE CONFIGURATION =====
#define FIREBASE_HOST "https://motion-bin-default-rtdb.firebaseio.com"
#define API_KEY "AIzaSyD9z1HOU7BTJqXFiuDuo05T1ppxjOlH_Vk"

FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;

// ===== SENSOR PINS (LINE FOLLOWING - 3 IR) =====
const int S_LEFT   = 34;
const int S_CENTER = 35;
const int S_RIGHT  = 32;

// ===== POINT DETECTION PINS (2 IR sensors for marker detection) =====
const int POINT_DETECT_1 = 33;  // Point detection sensor 1
const int POINT_DETECT_2 = 36;  // Point detection sensor 2

// ===== ULTRASONIC PINS (OBSTACLE) =====
const int TRIG_PIN = 25;
const int ECHO_PIN = 26;

// ===== BATTERY MONITORING =====
const int BATT_PIN = 4;  // ADC pin for battery voltage
const float BATT_MAX_V = 4.2;  // Max voltage (2S LiPo)
const float BATT_MIN_V = 3.0;  // Min voltage (2S LiPo)

// ===== MOTOR PINS =====
// Right Motor
const int IN1 = 18;
const int IN2 = 19;
const int ENA = 5;

// Left Motor
const int IN3 = 21;
const int IN4 = 22;
const int ENB = 23;

// ===== PWM SETTINGS =====
const int PWM_R = 0;
const int PWM_L = 1;
const int freq = 20000;
const int resolution = 8;

// ===== PID PARAMETERS =====
float Kp = 22;
float Ki = 0;
float Kd = 10;

float error = 0;
float previousError = 0;
float integral = 0;

// ===== SPEED PARAMETERS =====
int baseSpeed = 240;
int maxSpeed  = 255;
int minSpeed  = 240;
int threshold = 2000;
int pivotSpeed = 200;

// ===== OBSTACLE SETTINGS =====
int safeDistance = 20;  // cm

// ===== LOCATION SYSTEM =====
enum Location { START, POINT_1, POINT_2, POINT_3, MOVING, UNKNOWN };
Location currentLocation = START;
Location targetLocation = START;
Location lastLocation = START;

const char* locationNames[] = {"Start", "Point 1", "Point 2", "Point 3", "Moving", "Unknown"};

// ===== TIMING =====
unsigned long lastFirebaseUpdate = 0;
const unsigned long FIREBASE_UPDATE_INTERVAL = 1000;  // Update Firebase every 1 second
unsigned long lastDistanceCheck = 0;
const unsigned long DISTANCE_CHECK_INTERVAL = 200;  // Check distance every 200ms

// ===== OBSTACLE STATE =====
bool obstacleDetected = false;
bool lastObstacleState = false;

// ===== FUNCTION DECLARATIONS =====
void setupWiFi();
void setupFirebase();
void readSensors();
void updateFirebase();
void readFirebaseCommand();
void runPID();
void moveMotors(int rightSpeed, int leftSpeed);
void pivotLeft();
void pivotRight();
void stopMotors();
int getDistance();
int getBatteryPercent();
bool checkPointDetection();
void updateLocation();
void logToFirebase(const char* message);

// ===================================================================
// SETUP
// ===================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n========================================");
  Serial.println("🚀 SMART MOTION BIN - STARTING");
  Serial.println("========================================");

  // Initialize pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BATT_PIN, INPUT);

  // PWM setup
  ledcAttach(ENA, freq, resolution);
  ledcAttach(ENB, freq, resolution);

  // Initialize subsystems
  setupWiFi();
  setupFirebase();

  logToFirebase("System initialized. Dashboard connected.");
  Serial.println("✅ Setup complete!");
}

// ===================================================================
// MAIN LOOP
// ===================================================================
void loop() {
  // Keep Firebase connected
  if (!Firebase.ready()) {
    Serial.println("❌ Firebase disconnected!");
    setupFirebase();
    return;
  }

  // Read command from web dashboard
  readFirebaseCommand();

  // Read all sensors
  readSensors();

  // Check for obstacles
  if (millis() - lastDistanceCheck >= DISTANCE_CHECK_INTERVAL) {
    int distance = getDistance();
    obstacleDetected = (distance > 0 && distance < safeDistance);
    
    // Log obstacle state changes
    if (obstacleDetected != lastObstacleState) {
      lastObstacleState = obstacleDetected;
      if (obstacleDetected) {
        Serial.print("🚧 Obstacle detected: ");
        Serial.print(distance);
        Serial.println(" cm");
        logToFirebase("Obstacle detected. Stopped.");
      } else {
        Serial.println("✅ Obstacle cleared");
        logToFirebase("Obstacle cleared. Resuming.");
      }
    }
    lastDistanceCheck = millis();
  }

  // ===== OBSTACLE MODE =====
  if (obstacleDetected) {
    stopMotors();
    currentLocation = MOVING;
  }
  // ===== NORMAL LINE FOLLOWING =====
  else {
    runPID();
  }

  // Update Firebase (send sensor data to web dashboard)
  if (millis() - lastFirebaseUpdate >= FIREBASE_UPDATE_INTERVAL) {
    updateFirebase();
    lastFirebaseUpdate = millis();
  }

  delay(1);
}

// ===================================================================
// WiFi SETUP
// ===================================================================
void setupWiFi() {
  Serial.print("📡 Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ WiFi Failed - Operating in offline mode");
  }
}

// ===================================================================
// FIREBASE SETUP
// ===================================================================
void setupFirebase() {
  Serial.println("🔥 Initializing Firebase...");

  config.database_url = FIREBASE_HOST;
  config.api_key = API_KEY;

  config.signer.test_mode = true;   // ✅ ADD THIS LINE
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);

  // Wait for Firebase to connect
  int attempts = 0;
  while (!Firebase.ready() && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (Firebase.ready()) {
    Serial.println("\n✅ Firebase Connected!");
  } else {
    Serial.println("\n⚠ Firebase initialization incomplete (network may be slow)");
  }
}

// ===================================================================
// READ FIREBASE COMMAND (target location from web dashboard)
// ===================================================================
void readFirebaseCommand() {
  if (Firebase.RTDB.getJSON(&fbdo, "/bin_commands/target_location")) {
    String command = fbdo.stringData();
    
    // Parse command and set target location
    if (command == "Start") targetLocation = START;
    else if (command == "Point 1") targetLocation = POINT_1;
    else if (command == "Point 2") targetLocation = POINT_2;
    else if (command == "Point 3") targetLocation = POINT_3;

    if (targetLocation != lastLocation) {
      Serial.print("📍 New command received: Navigate to ");
      Serial.println(locationNames[targetLocation]);
      lastLocation = targetLocation;
    }
  }
}

// ===================================================================
// READ SENSORS (IR line-following, point detection)
// ===================================================================
void readSensors() {
  // Already handled in runPID()
}

// ===================================================================
// CHECK POINT DETECTION (2 IR sensors for marker arrival)
// ===================================================================
bool checkPointDetection() {
  int detect1 = digitalRead(POINT_DETECT_1);
  int detect2 = digitalRead(POINT_DETECT_2);
  
  // Both sensors detect marker = at checkpoint
  return (detect1 == HIGH && detect2 == HIGH);
}

// ===================================================================
// UPDATE LOCATION BASED ON POINT DETECTION
// ===================================================================
void updateLocation() {
  if (checkPointDetection()) {
    // Mark as arrived at target
    if (currentLocation != targetLocation) {
      currentLocation = targetLocation;
      Serial.print("✓ Successfully reached: ");
      Serial.println(locationNames[currentLocation]);
      logToFirebase("Successfully reached waypoint.");
    }
  } else if (currentLocation != MOVING && currentLocation != targetLocation) {
    // Moving towards target
    currentLocation = MOVING;
  }
}

// ===================================================================
// GET BATTERY PERCENT (0-100)
// ===================================================================
int getBatteryPercent() {
  // Read ADC value (0-4095 for 12-bit)
  int adcValue = analogRead(BATT_PIN);
  
  // Convert to voltage (assuming 3.3V reference and voltage divider)
  float voltage = (adcValue / 4095.0) * 3.3 * 2;  // *2 for voltage divider
  
  // Constrain and convert to percentage
  voltage = constrain(voltage, BATT_MIN_V, BATT_MAX_V);
  int percent = map(voltage * 100, BATT_MIN_V * 100, BATT_MAX_V * 100, 0, 100);
  
  return constrain(percent, 0, 100);
}

// ===================================================================
// GET DISTANCE (ULTRASONIC SENSOR)
// ===================================================================
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duration == 0) return -1;
  
  int distance = duration * 0.034 / 2;
  return distance;
}

// ===================================================================
// UPDATE FIREBASE (Send sensor data to web dashboard)
// ===================================================================
void updateFirebase() {
  if (!Firebase.ready()) return;

  // Read sensors
  int L = analogRead(S_LEFT);
  int C = analogRead(S_CENTER);
  int R = analogRead(S_RIGHT);
  
  int distance = getDistance();
  int batteryPercent = getBatteryPercent();
  
  bool pointDetect1 = (digitalRead(POINT_DETECT_1) == HIGH);
  bool pointDetect2 = (digitalRead(POINT_DETECT_2) == HIGH);
  
  // Update location
  updateLocation();

  // Create JSON object with sensor data
  FirebaseJson json;
  json.set("current_location", locationNames[currentLocation]);
  json.set("destination", locationNames[targetLocation]);
  json.set("distance_cm", distance);
  json.set("ir_left", (L > threshold) ? 1 : 0);
  json.set("ir_center", (C > threshold) ? 1 : 0);
  json.set("ir_right", (R > threshold) ? 1 : 0);
  json.set("point_detection_1", pointDetect1);
  json.set("point_detection_2", pointDetect2);
  json.set("battery_percent", batteryPercent);
  json.set("timestamp", millis());

  // Send to Firebase
  if (Firebase.RTDB.setJSON(&fbdo, "/bin_status", &json)) {
    // Silently update (no spam in serial)
  } else {
    Serial.print("❌ Firebase update failed: ");
    Serial.println(fbdo.errorReason());
  }
}

// ===================================================================
// LOG MESSAGE TO FIREBASE
// ===================================================================
void logToFirebase(const char* message) {
  if (!Firebase.ready()) {
    Serial.println(message);
    return;
  }

  if (Firebase.RTDB.setString(&fbdo, "/bin_status/latest_log", message)) {
    Serial.print("[LOG] ");
    Serial.println(message);
  }
}

// ===================================================================
// PID LINE FOLLOWING LOGIC (Original code + point detection check)
// ===================================================================
void runPID() {
  
  int L = analogRead(S_LEFT);
  int C = analogRead(S_CENTER);
  int R = analogRead(S_RIGHT);

  bool left   = (L > threshold);
  bool center = (C > threshold);
  bool right  = (R > threshold);

  // ===== ERROR CALCULATION =====
  if (left && !center && !right)       error = -2;
  else if (left && center && !right)   error = -1;
  else if (!left && center && !right)  error = 0;
  else if (!left && center && right)   error = 1;
  else if (!left && !center && right)  error = 2;
  else {
    // Line lost - pivot
    if (previousError < 0) pivotLeft();
    else pivotRight();
    return;
  }

  // ===== BASE SPEED =====
  baseSpeed = 240;

  // ===== PID CALCULATION =====
  integral += error;
  float derivative = error - previousError;

  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  correction = constrain(correction, -180, 180);

  previousError = error;

  int rightSpeed = baseSpeed - correction;
  int leftSpeed  = baseSpeed + correction;

  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);
  leftSpeed  = constrain(leftSpeed, minSpeed, maxSpeed);

  moveMotors(rightSpeed, leftSpeed);
}

// ===================================================================
// MOTOR CONTROL FUNCTIONS
// ===================================================================

void moveMotors(int rightSpeed, int leftSpeed) {
  // Right motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, rightSpeed);

  // Left motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENB, leftSpeed);
}

void pivotLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ENA, pivotSpeed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENB, pivotSpeed);
}

void pivotRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  ledcWrite(ENA, pivotSpeed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ENB, pivotSpeed);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}