#include <ArduinoJson.h>

constexpr int LEFT_PWM_PIN = 25;
constexpr int LEFT_DIR_PIN = 26;
constexpr int RIGHT_PWM_PIN = 27;
constexpr int RIGHT_DIR_PIN = 14;

constexpr int PWM_FREQ = 20000;
constexpr int PWM_RES = 8;
constexpr int LEFT_CHANNEL = 0;
constexpr int RIGHT_CHANNEL = 1;

constexpr float OBSTACLE_THRESHOLD = 0.55f;
constexpr float TURN_GAIN = 0.7f;
constexpr float CRUISE_SPEED = 0.65f;
constexpr float STOP_THRESHOLD = 0.20f;
constexpr unsigned long SENSOR_TIMEOUT_MS = 200;

unsigned long lastSensorMillis = 0;
long lastSeq = -1;
float lastVL = 0.0f;
float lastVR = 0.0f;
String lastState = "stop";

float clamp01(float value) {
  if (value > 1.0f) return 1.0f;
  if (value < -1.0f) return -1.0f;
  return value;
}

void setMotor(float normalizedLeft, float normalizedRight) {
  float left = clamp01(normalizedLeft);
  float right = clamp01(normalizedRight);

  digitalWrite(LEFT_DIR_PIN, left >= 0.0f ? HIGH : LOW);
  digitalWrite(RIGHT_DIR_PIN, right >= 0.0f ? HIGH : LOW);

  int leftDuty = static_cast<int>(fabs(left) * 255.0f);
  int rightDuty = static_cast<int>(fabs(right) * 255.0f);

  ledcWrite(LEFT_CHANNEL, leftDuty);
  ledcWrite(RIGHT_CHANNEL, rightDuty);
}

void emitControl(long seq, float vL, float vR, const String &state) {
  StaticJsonDocument<192> out;
  out["seq"] = seq;
  out["vL"] = clamp01(vL);
  out["vR"] = clamp01(vR);
  out["state"] = state;

  serializeJson(out, Serial);
  Serial.println();
}

void computeControl(float dF, float dL, float dR, float &vL, float &vR, String &state) {
  vL = CRUISE_SPEED;
  vR = CRUISE_SPEED;
  state = "navigate";

  if (dF <= STOP_THRESHOLD) {
    vL = 0.0f;
    vR = 0.0f;
    state = "stop";
    return;
  }

  if (dF < OBSTACLE_THRESHOLD) {
    float turnDirection = (dL > dR) ? 1.0f : -1.0f;
    float turn = TURN_GAIN * turnDirection;
    vL = clamp01(0.35f - turn * 0.5f);
    vR = clamp01(0.35f + turn * 0.5f);
    state = "avoid";
  }
}

bool parseSensorMessage(const String &line, long &seq, float &dF, float &dL, float &dR, float &dt) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.print("WARN parse error: ");
    Serial.println(err.c_str());
    return false;
  }

  if (!doc.containsKey("seq") || !doc.containsKey("dF") || !doc.containsKey("dL") || !doc.containsKey("dR") || !doc.containsKey("dt")) {
    Serial.println("WARN missing required field(s)");
    return false;
  }

  if (!doc["seq"].is<long>() || !doc["dF"].is<float>() || !doc["dL"].is<float>() || !doc["dR"].is<float>() || !doc["dt"].is<float>()) {
    Serial.println("WARN invalid field type(s)");
    return false;
  }

  seq = doc["seq"].as<long>();
  dF = doc["dF"].as<float>();
  dL = doc["dL"].as<float>();
  dR = doc["dR"].as<float>();
  dt = doc["dt"].as<float>();

  return true;
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  ledcSetup(LEFT_CHANNEL, PWM_FREQ, PWM_RES);
  ledcSetup(RIGHT_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_PWM_PIN, LEFT_CHANNEL);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_CHANNEL);

  setMotor(0.0f, 0.0f);
  lastSensorMillis = millis();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      long seq = -1;
      float dF = 0.0f;
      float dL = 0.0f;
      float dR = 0.0f;
      float dt = 0.0f;

      if (parseSensorMessage(line, seq, dF, dL, dR, dt)) {
        if (lastSeq >= 0 && seq != lastSeq + 1) {
          Serial.print("WARN sequence gap. expected=");
          Serial.print(lastSeq + 1);
          Serial.print(" got=");
          Serial.println(seq);
        }

        float vL = 0.0f;
        float vR = 0.0f;
        String state = "stop";
        computeControl(dF, dL, dR, vL, vR, state);

        setMotor(vL, vR);
        emitControl(seq, vL, vR, state);

        lastSeq = seq;
        lastSensorMillis = millis();
        lastVL = vL;
        lastVR = vR;
        lastState = state;
      }
    }
  }

  if (millis() - lastSensorMillis > SENSOR_TIMEOUT_MS) {
    if (lastState != "timeout" || lastVL != 0.0f || lastVR != 0.0f) {
      setMotor(0.0f, 0.0f);
      emitControl(lastSeq, 0.0f, 0.0f, "timeout");
      lastVL = 0.0f;
      lastVR = 0.0f;
      lastState = "timeout";
    }
  }
}
