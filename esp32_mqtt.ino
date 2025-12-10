#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <MPU6050.h>

// ---------------- WIFI + MQTT ----------------
const char* ssid = "TP-Link_F998";
const char* password = "23423725";

const char* mqtt_server = "169.38.14.163";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// ---------------- DS18B20 ----------------
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ---------------- MPU6050 ----------------
MPU6050 mpu;

// ---------------- ACS712 ----------------
#define ACS_PIN 34
float sensitivity = 0.100; 
float offsetVoltage = 2.5;

// ---------------- MACHINE NUMBER ----------------
int machine = 1;

// =========================================================
// WIFI
// =========================================================
void setup_wifi() {
  Serial.print("Connecting to "); Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

// =========================================================
// MQTT RECONNECT
// =========================================================
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection... ");

    if (client.connect("ESP32_Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// =========================================================
// SETUP
// =========================================================
void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  // DS18B20
  sensors.begin();
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);

  // MPU6050
  Wire.begin(21, 22);
  mpu.initialize();

  if (!mpu.testConnection())
    Serial.println("MPU6050 connection FAILED");
  else
    Serial.println("MPU6050 connected.");
}

// =========================================================
// READ CURRENT (ACS712)
// =========================================================
float readCurrent() {
  int raw = analogRead(ACS_PIN);
  float voltage = (raw / 4095.0) * 3.3;
  return (voltage - offsetVoltage) / sensitivity;
}

// =========================================================
// CALCULATE VIBRATION (RMS of accel values)
// =========================================================
float readVibration() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // convert raw accel to g-force (approx)
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float vibration = sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
  return vibration;
}

// =========================================================
// LOOP
// =========================================================
void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  // -------- Temperature --------
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);

  // -------- Current --------
  float current = readCurrent();

  // -------- Vibration --------
  float vibration = readVibration();

  // -------- PRINT --------
  Serial.println("--- SENSOR DATA ---");
  Serial.print("Temp: "); Serial.println(temperature);
  Serial.print("Current: "); Serial.println(current);
  Serial.print("Vibration: "); Serial.println(vibration);
  Serial.println("--------------------");

  // -------- MQTT PUBLISH --------
  String topic = "machine/" + String(machine) + "/data";

  String payload = "{";
  payload += "\"temp\":" + String(temperature) + ",";
  payload += "\"current\":" + String(current) + ",";
  payload += "\"vibration\":" + String(vibration);
  payload += "}";

  client.publish(topic.c_str(), payload.c_str());

  delay(1000);
}
