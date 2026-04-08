/********************************************************************
* Team Id: HB_2442
* Author List: Kumar Sushant Raj, Atharva Rana, Aditya Kumar
* Filename: esp32_robot_controller.cpp
* Theme: eYRC Holo Battalion
* Functions: setup_wifi(), setContinuousServo(), mqttCallback(),
*            reconnect(), publishSensorData(), setup(), loop()
* Global Variables: ROBOT_ID, ssid, password, broker_ip,
*                   FWD_OFFSET, REV_OFFSET,
*                   M1_PIN, M2_PIN, M3_PIN,
*                   BASE_PIN, ELBOW_PIN,
*                   SOLENOID_PIN, IR_PIN,
*                   espClient, mqttClient,
*                   m1Servo, m2Servo, m3Servo,
*                   baseServo, elbowServo,
*                   client_id, cmd_topic, sensor_topic
********************************************************************/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

/* ================== USER CONFIGURATION ================== */
// !!! CHANGE THIS FOR EACH ROBOT !!!
#define ROBOT_ID 2 // Set to 0, 1, 2, or 4 based on the  robot's marker ID

const char* ssid = "POCO M3";
const char* password = "saksham123";
const char* broker_ip = "10.80.183.168";

/* ================== MOTOR CALIBRATION ================== */
// Deadzone offsets. 
#define FWD_OFFSET 63  // Added to 1500 for positive speed
#define REV_OFFSET 55// Subtracted from 1500 for negative speed

/* ================== Pin Mapping ================== */
#define M1_PIN 18
#define M2_PIN 17
#define M3_PIN 4

#define BASE_PIN 14
#define ELBOW_PIN 26

#define SOLENOID_PIN 32
#define IR_PIN 36

/* ================== Globals (Dynamic Topics) ================== */
WiFiClient espClient;
PubSubClient mqttClient(espClient);

Servo m1Servo, m2Servo, m3Servo;
Servo baseServo, elbowServo;

// Dynamic buffers for topics based on ID
char client_id[32];
char cmd_topic[32];
char sensor_topic[32];

/*
------------------------------------------------------------
Function: setup_wifi

Input:
    None

Output:
    None

Logic Explanation:

    1. Initiates WiFi connection using configured SSID and password.

    2. Continuously checks connection status until WL_CONNECTED.

    3. Prints connection progress via Serial monitor.

    4. Displays assigned local IP address upon success.

    5. Ensures ESP32 has network connectivity before MQTT starts.

Example:
    setup_wifi();
------------------------------------------------------------
*/
void setup_wifi() {
  delay(10);
  Serial.printf("\n[WiFi] Connecting to %s for Robot ID: %d\n", ssid, ROBOT_ID);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n[WiFi] Connected");
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.localIP());
}

/*
------------------------------------------------------------
Function: setContinuousServo

Input:
    s (Servo reference)
    speed (float, -480 to +480)
    name (const char*)

Output:
    None

Logic Explanation:

    1. Constrains speed within safe operational range.

    2. Uses 1500µs as neutral PWM (stop position).

    3. Applies forward offset for positive speeds.

    4. Applies reverse offset for negative speeds.

    5. Enforces PWM safety limits (1020–1980µs).

    6. Sends final PWM signal to continuous servo.

    7. Compensates for motor deadzone drift.

Example:
    setContinuousServo(m1Servo, 300, "M1");
------------------------------------------------------------
*/
void setContinuousServo(Servo &s, float speed, const char* name) {
  speed = constrain(speed, -480, 480);

  float pwm_us = 1500;

  // Deadband check: if speed is effectively 0, stop
  if (speed > -0.01 && speed < 0.01) {
    pwm_us = 1500;
  }
  // FORWARD MOVEMENTS
  else if (speed > 0) {
    pwm_us = 1500 + FWD_OFFSET + speed;
  }
  // BACKWARD MOVEMENTS
  else {
    pwm_us = 1500 - REV_OFFSET + speed;
  }

  pwm_us = constrain(pwm_us, 1020, 1980);
  s.writeMicroseconds(pwm_us);

}

/*
------------------------------------------------------------
Function: mqttCallback

Input:
    topic (char*)
    payload (byte*)
    length (unsigned int)

Output:
    None

Logic Explanation:

    1. Parses incoming MQTT JSON payload.

    2. Extracts motor speeds (m1, m2, m3).

    3. Extracts arm angles (base, elbow).

    4. Extracts solenoid state.

    5. Applies continuous motor control via setContinuousServo().

    6. Writes constrained angles to servo motors.

    7. Controls solenoid output pin.

    8. Implements safe default values in case of missing JSON fields.

Example:
    Triggered automatically on MQTT message arrival.
------------------------------------------------------------
*/
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("[ERROR] JSON parse failed: ");
    Serial.println(error.f_str());
    return;
  }

  /* -------- SAFE FLOAT PARSING -------- */
  float m1_f = doc["m1"] | 0.0;
  float m2_f = doc["m2"] | 0.0;
  float m3_f = doc["m3"] | 0.0;

  int base = doc["base"] | 90;   // Default safety angle
  int elbow = doc["elbow"] | 90; // Default safety angle
  int solenoid = doc["solenoid"] | 0;

  /* -------- APPLY MOTOR COMMANDS -------- */
  setContinuousServo(m1Servo, m1_f, "M1");
  setContinuousServo(m2Servo, m2_f, "M2");
  setContinuousServo(m3Servo, m3_f, "M3");

  /* -------- APPLY SERVO COMMANDS -------- */
  baseServo.write(constrain(base, 0, 180));
  elbowServo.write(constrain(elbow, 0, 180));

  /* -------- APPLY SOLENOID -------- */
  digitalWrite(SOLENOID_PIN, solenoid ? HIGH : LOW);

}

/*
------------------------------------------------------------
Function: reconnect

Input:
    None

Output:
    None

Logic Explanation:

    1. Checks if MQTT client is disconnected.

    2. Attempts connection using dynamic client_id.

    3. On success:
         - Subscribes to robot-specific command topic.

    4. On failure:
         - Prints error state.
         - Waits 3 seconds before retrying.

    5. Ensures persistent MQTT communication reliability.

Example:
    reconnect();
------------------------------------------------------------
*/
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.printf("[MQTT] Attempting connection as %s...\n", client_id);

    if (mqttClient.connect(client_id)) {
      Serial.println("[MQTT] Connected");

      // Subscribe to dynamic topic
      mqttClient.subscribe(cmd_topic);
      Serial.printf("[MQTT] Subscribed to %s\n", cmd_topic);

    } else {
      Serial.print("[MQTT] Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 3s");
      delay(3000);
    }
  }
}

/*
------------------------------------------------------------
Function: publishSensorData

Input:
    None

Output:
    None

Logic Explanation:

    1. Reads IR sensor digital value (Active LOW logic).

    2. Adds robot ID for identification.

    3. Adds system uptime (millis).

    4. Serializes data into JSON format.

    5. Publishes JSON to robot-specific MQTT sensor topic.

    6. Runs periodically (~20Hz from loop).

Example:
    publishSensorData();
------------------------------------------------------------
*/
void publishSensorData() {
  StaticJsonDocument<128> doc;

  // Logic: Active LOW sensor usually means 0=Detected, 1=Empty
  // We send the raw value, ROS node handles the logic.
  doc["ir"] = digitalRead(IR_PIN);
  doc["id"] = ROBOT_ID; // Send ID just in case
  doc["uptime"] = millis();

  char buffer[128];
  serializeJson(doc, buffer);
  mqttClient.publish(sensor_topic, buffer);
}

/*
------------------------------------------------------------
Function: setup

Input:
    None

Output:
    None

Logic Explanation:

    1. Initializes Serial communication.

    2. Dynamically generates:
         - MQTT client ID
         - Command topic
         - Sensor topic

    3. Allocates ESP32 PWM timers.

    4. Attaches motors and arm servos with frequency settings.

    5. Configures solenoid and IR GPIO pins.

    6. Establishes WiFi connection.

    7. Configures MQTT broker and callback.

    8. Disables WiFi sleep for lower latency.

    9. Prepares ESP32 as full ROS–MQTT hardware bridge.

Example:
    Automatically executed at boot.
------------------------------------------------------------
*/
void setup() {
  Serial.begin(115200);
  delay(500);

  // --- Generate Dynamic IDs/Topics ---
  sprintf(client_id, "ESP32_ROBOT_%d", ROBOT_ID);
  sprintf(cmd_topic, "esp/cmd/%d", ROBOT_ID);
  sprintf(sensor_topic, "esp/sensor/%d", ROBOT_ID);

  Serial.println("\n========================================");
  Serial.printf("   ESP32 CONTROLLER FOR ROBOT ID: %d\n", ROBOT_ID);
  Serial.printf("   CMD TOPIC: %s\n", cmd_topic);
  Serial.println("========================================");

  // --- Servo / Motor Setup ---
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  m1Servo.setPeriodHertz(50); m1Servo.attach(M1_PIN, 1000, 2000);
  m2Servo.setPeriodHertz(50); m2Servo.attach(M2_PIN, 1000, 2000);
  m3Servo.setPeriodHertz(50); m3Servo.attach(M3_PIN, 1000, 2000);

  baseServo.setPeriodHertz(50);  baseServo.attach(BASE_PIN, 500, 2400);
  elbowServo.setPeriodHertz(50); elbowServo.attach(ELBOW_PIN, 500, 2400);

  // --- IO Setup ---
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(SOLENOID_PIN, LOW); // Force OFF on boot

  pinMode(IR_PIN, INPUT);

  // --- Comms Setup ---
  setup_wifi();
  mqttClient.setServer(broker_ip, 1883);
  mqttClient.setCallback(mqttCallback);

  // Keep WiFi latency low
  WiFi.setSleep(false);
}

/*
------------------------------------------------------------
Function: loop

Input:
    None

Output:
    None

Logic Explanation:

    1. Checks MQTT connection status.
       - Reconnects if necessary.

    2. Processes incoming MQTT messages.

    3. Publishes sensor data every 50ms (~20Hz).

    4. Maintains real-time bidirectional communication
       between ESP32 hardware and ROS system.

Example:
    Automatically runs continuously.
------------------------------------------------------------
*/

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  // Publish sensor data at ~20Hz (50ms)
  static unsigned long lastPub = 0;
  if (millis() - lastPub > 50) {
    lastPub = millis();
    publishSensorData();
  }
}