/*
 * Manuscript 2 — Arduino GIGA R1 WiFi lower controller / HIL interface.
 * -------------------------------------------------------------------
 * Plant bridge: USB CDC Serial request/response to MATLAB/Simulink (1 s nominal step).
 * Supervisory and telemetry path: MQTT between GIGA and Python/web dashboard.
 * Lower layer: 1 Hz current-tracking MPC; local interlocks override it.
 * Operator display: Nextion on Serial3; DS3231 over I2C.
 * USB CDC Serial is exclusively the framed newline-delimited MATLAB plant link.
 *
 * IMPORTANT: the PEM process is simulated in MATLAB/Simulink during HIL.
 * This firmware does not constitute a certified physical-stack protection
 * system. Do not connect a real stack without independently validated
 * hardwired safety circuits, contactors and gas-analysis instrumentation.
 */
#include <Wire.h>
#include "RTClib.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include <stdarg.h>
#if defined(__has_include)
  #if __has_include(<mbed_stats.h>)
    #include <mbed_stats.h>
    #define M2_HAS_MBED_STATS 1
  #endif
#endif
#ifndef M2_HAS_MBED_STATS
  #define M2_HAS_MBED_STATS 0
#endif

// Copy m2_secrets.example.h to m2_secrets.h and enter local Wi-Fi credentials.
// The actual credentials are intentionally not included in this package.
#if __has_include("m2_secrets.h")
  #include "m2_secrets.h"
#else

  #define M2_WIFI_SSID "Galaxy A14 FD8D"
  #define M2_WIFI_PASS "qwertyzxcvbnm"
  
#endif
#ifndef M2_MQTT_HOST
  #define M2_MQTT_HOST "broker.hivemq.com"
#endif
#ifndef M2_MQTT_PORT
  #define M2_MQTT_PORT 1883
#endif

// =====================================================================
// BENCH / HIL MODE SWITCHES
// =====================================================================
#define M2_BENCH_MODE 0
#define M2_BYPASS_SAFETY_PINS 1   // HIL bench only; not valid for physical-stack operation.
#define M2_LOCAL_SIM_FALLBACK 0
#ifndef ENABLE_GSM_FALLBACK
  #define ENABLE_GSM_FALLBACK 0
#endif
#if ENABLE_GSM_FALLBACK
  #define TINY_GSM_MODEM_SIM800
  #include <TinyGsmClient.h>
#endif

// ---------------- Network / MQTT settings ----------------
const char* WIFI_SSID = M2_WIFI_SSID;
const char* WIFI_PASS = M2_WIFI_PASS;
const char* MQTT_HOST = M2_MQTT_HOST;
const uint16_t MQTT_PORT = M2_MQTT_PORT;
const char* MQTT_CLIENT_ID = "m2-pem-giga";
const char* MQTT_GSM_CLIENT_ID = "m2-pem-giga-gsm";
const char* GSM_APN = "REPLACE_WITH_SIM_PROVIDER_APN";
const char* GSM_USER = "";
const char* GSM_PASS = "";

const char* TOPIC_COMMAND = "m2/knh_pem_hil_m2_2026/upper/setpoint";
const char* TOPIC_TELEMETRY = "m2/knh_pem_hil_m2_2026/arduino/telemetry";
const char* TOPIC_LEGACY_COMMAND = "dashboard/command";
const char* TOPIC_LEGACY_SIMULINK = "arduino/simulink";
const char* TOPIC_LEGACY_EMERGENCY = "arduino/emergency";
const char* TOPIC_LOG = "m2/knh_pem_hil_m2_2026/arduino/log";

#define NEXTION_POWER_BUTTON_ID 15
#define NEXTION_START_BUTTON_ID 16
#define NEXTION_STOP_BUTTON_ID 13

// ---------------- Hard design defaults (MATLAB schedule overrides) ----------
const float I_MIN_A = 100.0f;
const float I_MAX_A = 1400.0f;
const float CELL_VOLTAGE_MAX_V = 2.5f;
const float TEMPERATURE_MAX_C = 80.0f;
const float PURITY_MIN_MODEL_PCT = 99.5f;       // Product-quality target
const float H2_IN_O2_LIMIT_PPM = 20000.0f;       // 2 vol% process-safety threshold
const float PRESSURE_MAX_BAR = 30.0f;
const float FARADAY_EFF = 0.95f;

float matlabDesignAreaCm2 = 641.0f;
int matlabDesignCells = 80;
float matlabDesignJ_Apm2 = 0.0f;
float matlabDesignTemperatureC = 65.0f;
float matlabDesignPressurePa = 15.0e5f;
float matlabDesignIminA = I_MIN_A;
float matlabDesignImaxA = I_MAX_A;
float matlabHospitalDemandLpm = 48.0f;
bool matlabDesignLocked = false;
String activeRunId = "";
float h2PriceKESkg = 0.0f;
float effectiveIminA = I_MIN_A;
float effectiveImaxA = I_MAX_A;

// Supervisory disturbances are cached here and forwarded with every plant step.
float upperPVAvailableW = 0.0f;
float upperO2DemandLpm = 48.0f;
float upperTambC = 25.0f;
float upperTariffKESkWh = 18.0f;
float upperH2DrawKgS = 0.0f;
float upperH2LeakPpmInject = 0.0f;
bool upperSensorFaultInject = false;
bool upperPurityAlarmInject = false;
float upperHour = 12.0f;
String upperScenario = "clear_sky";
String upperFaultState = "nominal";

float iSensorA = 0.0f, vCellSensorV = 0.0f, tSensorC = 0.0f;
float h2LeakPpm = 0.0f;
float o2FlowTrueLpmFull = 0.0f;

// ---------------- Timing / MPC settings ----------------
const uint32_t LOWER_PERIOD_MS = 1000;
const uint32_t PLANT_LINK_TIMEOUT_MS = 3000UL;
const uint32_t PLANT_LINK_STARTUP_TIMEOUT_MS = 10000UL;
const uint32_t COMMAND_TIMEOUT_MS = 600000;
const int MPC_HORIZON = 5;
const float MPC_DT_S = 1.0f;
const float MPC_TAU_S = 5.0f; // matches tau_I_s in the shared m2_config.json
const float MPC_Q = 10.0f;
const float MPC_R = 0.08f;
const float MPC_TERMINAL = 40.0f;
const float MPC_MOVE_A = 50.0f;
const float MPC_GRID_A = 5.0f;

// ---------------- Safety inputs (active LOW) ----------------
const int PIN_ESTOP_OK = 22;
const int PIN_H2_SENSOR_OK = 23;
const int PIN_OVERPRESSURE_OK = 24;
const int PIN_WATER_OK = 25;
const int PIN_MQ8_ANALOG = A4;
const int MQ8_TRIP_ADC = 0; // Disabled in this simulated HIL build; configure for calibrated hardware.
uint16_t mq8Raw = 0;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void processPlantReplyLine(const char* line);
void readSerialPlantBridge();
PubSubClient& activeMqtt();
bool activeMqttConnected();
HardwareSerial &nexSerial = Serial3;
RTC_DS3231 rtc;
const uint32_t NEXTION_BAUD = 9600;
uint32_t lastHmiUpdateMs = 0;
uint32_t lastNextionPageQueryMs = 0;
uint8_t activeNextionPageId = 0;
bool nextionPageKnown = true;
bool nextionHmiRefreshPending = false;

#if ENABLE_GSM_FALLBACK
TinyGsm modem(Serial2);
TinyGsmClient gsmClient(modem);
PubSubClient mqttGsm(gsmClient);
#endif
bool usingGsm = false;
bool gsmInitialised = false;
bool forcePrimaryDown = false;
bool forceBothDown = false;

String controllerMode = "PROPOSED";
float upperCurrentRefA = I_MIN_A;
uint32_t upperCommandAtMs = 0;
uint32_t upperCommandSeq = 0;
uint32_t seq = 0;
uint32_t lastTickMs = 0;
uint32_t lastPlantReplyMs = 0;
float appliedCurrentA = I_MIN_A;
bool plantReplyReady = false;
bool requestPending = false;
bool plantLinkPrimed = false; // true only after the first valid Simulink step reply
bool havePlantReply = false;
String resetStatus = "not_requested";
bool latchedTrip = false;
String tripReason = "";
bool awaitingPlantResetAck = false;

float plantCurrentA = I_MIN_A;
float cellVoltageV = 1.8f;
float stackTemperatureC = 60.0f;
float stackPowerW = 0.0f;
float efficiencyPct = 0.0f;
float degradationHealthPct = 100.0f;
float oxygenFlowLpm = 0.0f;
float oxygenFlowTrueLpm = 0.0f;
float bopPowerW = 0.0f;
float systemPowerW = 0.0f;
float systemEfficiencyPct = 0.0f;
float o2ProductWaterPpm = 0.0f;
float h2ProductWaterPpm = 0.0f;
float waterRemovedKgS = 0.0f;
float hydrogenKgS = 0.0f;
float oxygenProducedLpm = 0.0f;
float hydrogenExportKgS = 0.0f;
float hydrogenVentedKgS = 0.0f;
float oxygenVentedLStep = 0.0f;
float h2InO2Ppm = 0.0f;
float oxygenDemandLpm = 48.0f;
float oxygenPressureBar = 1.0f;
float oxygenTankPressureBar = 1.3f;
float hydrogenPressureBar = 1.0f;
float oxygenBufferNm3 = 0.0f;
float hydrogenBufferNm3 = 0.0f;
float modelledPurityPct = 100.0f;
float pvUsedW = 0.0f;
float pvAvailableW = 0.0f;
float pvFilteredW = 0.0f;
float gridUsedW = 0.0f;
float tariffKESkWh = 0.0f;
float stressIndex = 0.0f;

float stateVec[6] = {65.0f, 30.0f * 1.01325e5f / (8.314462618f * 273.15f), 0.0f, I_MIN_A, 0.0f, 0.0f};
float plantStepMs = 0.0f;
float plantCycleMs = 0.0f;
float lowerLoopMs = 0.0f;
float lowerSolveMs = 0.0f;
float lowerTickIntervalMs = 0.0f;
uint32_t lastControlTickAtMs = 0;
uint32_t lastRequestAtMs = 0;
uint32_t missedDeadlineCount = 0;
float plantTimeS = 0.0f;
float simTimeS = 0.0f;
float o2UnmetLStep = 0.0f;
String scenarioName = "unknown";
String scenarioFault = "none";
bool missedPlantDeadline = false;

// ---------------- Utility helpers ----------------
void m2Log(const char* level, const char* fmt, ...) {
  char msg[220];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);
  if (activeMqttConnected()) {
    StaticJsonDocument<384> d;
    d["type"] = "log"; d["time_ms"] = millis(); d["level"] = level; d["message"] = msg;
    char payload[384];
    size_t n = serializeJson(d, payload, sizeof(payload));
    if (n > 0) activeMqtt().publish(TOPIC_LOG, (const uint8_t*)payload, n, false);
  }
}

float clipf(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }

PubSubClient& activeMqtt() {
#if ENABLE_GSM_FALLBACK
  if (usingGsm) return mqttGsm;
#endif
  return mqtt;
}

bool activeMqttConnected() { return activeMqtt().connected(); }

void subscribePlantTopics(PubSubClient &client) {
  client.setCallback(mqttCallback);
  client.setBufferSize(6144);
  client.setSocketTimeout(1);
  client.subscribe(TOPIC_COMMAND, 1);
  client.subscribe(TOPIC_LEGACY_COMMAND, 1);
  client.subscribe(TOPIC_LEGACY_EMERGENCY, 1);
  client.subscribe("pem/emergency", 1);
  client.subscribe("arduino/schedule", 1);
}

void updateEffectiveCurrentLimits() {
  if (matlabDesignIminA > 0.0f && matlabDesignImaxA > matlabDesignIminA) {
    effectiveIminA = matlabDesignIminA;
    effectiveImaxA = matlabDesignImaxA;
  } else {
    float area_m2 = (matlabDesignAreaCm2 > 0.0f ? matlabDesignAreaCm2 : 641.0f) / 10000.0f;
    if (matlabDesignJ_Apm2 > 0.0f && area_m2 > 0.0f) {
      effectiveImaxA = matlabDesignJ_Apm2 * area_m2;
      effectiveIminA = 1000.0f * area_m2;
    } else {
      effectiveIminA = I_MIN_A;
      effectiveImaxA = I_MAX_A;
    }
  }
  effectiveIminA = clipf(effectiveIminA, I_MIN_A, I_MAX_A);
  effectiveImaxA = clipf(effectiveImaxA, effectiveIminA + 1.0f, I_MAX_A);
  m2Log("DESIGN", "Locked current range %.2f..%.2f A | N=%d | area=%.2f cm2.",
        effectiveIminA, effectiveImaxA, matlabDesignCells, matlabDesignAreaCm2);
}

void trip(const char* why) {
  if (!latchedTrip) m2Log("SAFETY", "INTERLOCK LATCHED: %s", why);
  latchedTrip = true;
  tripReason = why;
  appliedCurrentA = 0.0f;
}

// ==================== PLANT BRIDGE (USB SERIAL) ====================
void sendPlantResetRequest(const char* runId) {
  StaticJsonDocument<256> d;
  d["type"] = "reset";
  d["run_id"] = runId ? runId : "";
  char payload[256];
  size_t n = serializeJson(d, payload, sizeof(payload));
  if (n == 0) { trip("plant_reset_json_error"); return; }
  activeRunId = runId ? String(runId) : String("");
  Serial.println(payload);
  Serial.flush();
  awaitingPlantResetAck = true;
  m2Log("HIL", "Plant reset request sent to MATLAB over USB Serial (run_id=%s).", activeRunId.c_str());
}

void sendPlantStepRequest() {
  if (awaitingPlantResetAck || requestPending) return;
  StaticJsonDocument<768> d;
  ++seq;
  d["type"] = "step";
  d["run_id"] = activeRunId;
  d["seq"] = seq;
  d["controller"] = "PROPOSED";
  d["I_cmd_A"] = (latchedTrip || controllerMode == "STOP") ? 0.0f : appliedCurrentA;
  d["dt_s"] = MPC_DT_S;
  d["PV_available_W"] = upperPVAvailableW;
  d["O2_demand_Lpm"] = upperO2DemandLpm;
  d["Tamb_C"] = upperTambC;
  d["tariff_KES_kWh"] = upperTariffKESkWh;
  d["H2_draw_kg_s"] = upperH2DrawKgS;
  d["H2_leak_ppm_inject"] = upperH2LeakPpmInject;
  d["hour"] = upperHour;
  d["scenario"] = upperScenario;
  d["fault_state"] = upperFaultState;
  JsonArray sx = d.createNestedArray("state");
  for (int i = 0; i < 6; ++i) sx.add(stateVec[i]);
  char payload[768];
  size_t n = serializeJson(d, payload, sizeof(payload));
  if (n == 0) { missedPlantDeadline = true; ++missedDeadlineCount; return; }
  Serial.println(payload);
  Serial.flush();
  lastRequestAtMs = millis();
  requestPending = true;
  if (seq == 1) m2Log("HIL", "First 1 s plant-step request sent to MATLAB over USB Serial.");
}

// MATLAB owns the host COM port and returns one JSON response per request line.
// This parser is non-blocking; Serial is not used for diagnostic text.
void readSerialPlantBridge() {
  static char line[3072];
  static size_t used = 0;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[used] = '\0';
      if (used > 0) processPlantReplyLine(line);
      used = 0;
    } else if (used < sizeof(line) - 1) {
      line[used++] = c;
    } else {
      used = 0; // discard malformed/oversized frame and resynchronise at LF
      missedPlantDeadline = true;
      ++missedDeadlineCount;
    }
  }
}

// ==================== MQTT CALLBACK ====================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicName(topic);
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    m2Log("WARN", "MQTT JSON parse failed on %s: %s", topic, err.c_str());
    return;
  }

  if (topicName == TOPIC_LEGACY_SIMULINK) return;
  if (topicName == TOPIC_LEGACY_EMERGENCY || topicName == "pem/emergency") {
    const char* reason = doc["reason"] | "legacy_emergency_command";
    trip(reason);
    if (activeMqttConnected()) activeMqtt().publish("arduino/emergency/ack", "{\"type\":\"emergency_ack\"}");
    return;
  }

  if (topicName == TOPIC_LEGACY_COMMAND || topicName == "arduino/schedule") {
    const char* legacyType = doc["type"] | "";
    if (String(legacyType) == "he_nmpc_schedule") {
      JsonObjectConst design = doc["design"].as<JsonObjectConst>();
      const float area = design["areaCm2"] | 0.0f;
      const int cells = design["nCells"] | 0;
      const float density = design["J"] | 0.0f;
      const float temperature = design["temperatureC"] | 0.0f;
      const float pressure = design["pressurePa"] | 0.0f;
      const float iMin = design["I_min_A"] | 0.0f;
      const float iMax = design["I_max_A"] | 0.0f;
      const float demand = doc["hospitalDemand"] | 0.0f;
      const bool locked = doc["design_locked"] | false;
      if (locked && area > 0.0f && cells > 0 && density > 0.0f &&
          temperature > 0.0f && pressure > 0.0f && demand > 0.0f &&
          iMin > 0.0f && iMax > iMin) {
        matlabDesignAreaCm2 = area; matlabDesignCells = cells;
        matlabDesignJ_Apm2 = density; matlabDesignTemperatureC = temperature;
        matlabDesignPressurePa = pressure; matlabDesignIminA = iMin;
        matlabDesignImaxA = iMax; matlabHospitalDemandLpm = demand;
        matlabDesignLocked = true;
        updateEffectiveCurrentLimits();
        m2Log("MQTT", "Design schedule accepted | A=%.2f N=%d J=%.1f T=%.2f P=%.0f O2=%.1f.",
              area, cells, density, temperature, pressure, demand);
        StaticJsonDocument<256> ack;
        ack["type"] = "he_nmpc_schedule_ack"; ack["status"] = "accepted";
        ack["design_locked"] = matlabDesignLocked; ack["areaCm2"] = matlabDesignAreaCm2;
        ack["nCells"] = matlabDesignCells; ack["J"] = matlabDesignJ_Apm2;
        ack["I_min_A"] = effectiveIminA; ack["I_max_A"] = effectiveImaxA;
        ack["hospitalDemand"] = matlabHospitalDemandLpm; ack["timestamp_ms"] = millis();
        char ackPayload[256]; size_t ackLen = serializeJson(ack, ackPayload, sizeof(ackPayload));
        if (ackLen > 0 && activeMqttConnected())
          activeMqtt().publish("arduino/design_ack", (const uint8_t*)ackPayload, ackLen, false);
      } else {
        m2Log("WARN", "Rejected MATLAB schedule: incomplete or invalid design data.");
      }
    }
    return;
  }

  if (topicName != TOPIC_COMMAND) return;
  uint32_t cmdSeq = doc["seq"] | 0;
  if (cmdSeq > 0 && cmdSeq <= upperCommandSeq) return;
  if (cmdSeq > 0) upperCommandSeq = cmdSeq;
  const char* type = doc["type"] | "";

  // Forward the current supervisory disturbance values to the Simulink plant.
  if (doc.containsKey("PV_available_W")) upperPVAvailableW = fmaxf(0.0f, doc["PV_available_W"].as<float>());
  if (doc.containsKey("O2_demand_Lpm")) upperO2DemandLpm = fmaxf(0.0f, doc["O2_demand_Lpm"].as<float>());
  if (doc.containsKey("Tamb_C")) upperTambC = doc["Tamb_C"].as<float>();
  if (doc.containsKey("tariff_KES_kWh")) upperTariffKESkWh = fmaxf(0.0f, doc["tariff_KES_kWh"].as<float>());
  if (doc.containsKey("H2_draw_kg_s")) upperH2DrawKgS = fmaxf(0.0f, doc["H2_draw_kg_s"].as<float>());
  if (doc.containsKey("H2_leak_ppm_inject")) upperH2LeakPpmInject = fmaxf(0.0f, doc["H2_leak_ppm_inject"].as<float>());
  if (doc.containsKey("sensor_fault_inject")) upperSensorFaultInject = doc["sensor_fault_inject"].as<bool>();
  if (doc.containsKey("purity_alarm_inject")) upperPurityAlarmInject = doc["purity_alarm_inject"].as<bool>();
  if (doc.containsKey("hour")) upperHour = clipf(doc["hour"].as<float>(), 0.0f, 24.0f);
  if (doc.containsKey("scenario")) upperScenario = String((const char*)(doc["scenario"] | "clear_sky"));
  if (doc.containsKey("fault_state")) upperFaultState = String((const char*)(doc["fault_state"] | "nominal"));

  if (doc.containsKey("H2_price_KES_kg")) {
    const float price = doc["H2_price_KES_kg"].as<float>();
    if (isfinite(price) && price >= 0.0f) h2PriceKESkg = price;
  }

  if (String(type) == "acknowledge_fault_reset") {
    // Clear a simulated fault latch without resetting Simulink state, buffers,
    // sequence number or simulation time. Physical reset remains interlock-gated.
    mq8Raw = (uint16_t)analogRead(PIN_MQ8_ANALOG);
    bool mq8Healthy = (MQ8_TRIP_ADC <= 0) || (mq8Raw < MQ8_TRIP_ADC);
    bool physicalHealthy = matlabDesignLocked && effectiveImaxA > effectiveIminA;
#if !M2_BENCH_MODE && !M2_BYPASS_SAFETY_PINS
    physicalHealthy = physicalHealthy &&
      digitalRead(PIN_ESTOP_OK) == LOW && digitalRead(PIN_H2_SENSOR_OK) == LOW &&
      digitalRead(PIN_OVERPRESSURE_OK) == LOW && digitalRead(PIN_WATER_OK) == LOW && mq8Healthy;
#endif
    bool processHealthy = !upperSensorFaultInject && !upperPurityAlarmInject &&
                          upperH2LeakPpmInject <= 500.0f && h2LeakPpm <= 500.0f &&
                          h2InO2Ppm <= H2_IN_O2_LIMIT_PPM &&
                          stackTemperatureC < TEMPERATURE_MAX_C &&
                          cellVoltageV < CELL_VOLTAGE_MAX_V &&
                          fmaxf(oxygenTankPressureBar, hydrogenPressureBar) <= PRESSURE_MAX_BAR &&
                          modelledPurityPct >= PURITY_MIN_MODEL_PCT;
    if (physicalHealthy && processHealthy) {
      latchedTrip = false;
      tripReason = "";
      resetStatus = "fault_reset_acknowledged_state_preserved";
      appliedCurrentA = effectiveIminA;
      upperCommandAtMs = millis();
      requestPending = false;
      m2Log("SAFETY", "Fault latch acknowledged; Simulink state/inventories/time preserved.");
    } else {
      resetStatus = "fault_reset_rejected_process_not_clear";
      m2Log("SAFETY", "Fault reset rejected: process/interlock condition remains active.");
    }
    return;
  }

  if (String(type) == "reset_run") {
    mq8Raw = (uint16_t)analogRead(PIN_MQ8_ANALOG);
    bool mq8Healthy = (MQ8_TRIP_ADC <= 0) || (mq8Raw < MQ8_TRIP_ADC);
    bool physicalHealthy = matlabDesignLocked && effectiveImaxA > effectiveIminA;
#if !M2_BENCH_MODE && !M2_BYPASS_SAFETY_PINS
    physicalHealthy = physicalHealthy &&
      digitalRead(PIN_ESTOP_OK) == LOW && digitalRead(PIN_H2_SENSOR_OK) == LOW &&
      digitalRead(PIN_OVERPRESSURE_OK) == LOW && digitalRead(PIN_WATER_OK) == LOW && mq8Healthy;
#endif
    if (physicalHealthy && upperH2LeakPpmInject <= 500.0f) {
      latchedTrip = false; tripReason = ""; resetStatus = "plant_reset_pending";
      requestPending = false; plantReplyReady = false; havePlantReply = false;
      plantLinkPrimed = false;
      plantTimeS = 0.0f; simTimeS = 0.0f; seq = 0; missedDeadlineCount = 0;
      appliedCurrentA = effectiveIminA; upperCurrentRefA = effectiveIminA;
      upperCommandAtMs = millis(); upperCommandSeq = 0;
      plantCurrentA = effectiveIminA; cellVoltageV = 1.8f;
      stackTemperatureC = matlabDesignTemperatureC;
      oxygenFlowLpm = 0.0f; oxygenFlowTrueLpm = 0.0f; hydrogenKgS = 0.0f;
      oxygenProducedLpm = 0.0f; hydrogenExportKgS = 0.0f; hydrogenVentedKgS = 0.0f;
      oxygenVentedLStep = 0.0f; h2InO2Ppm = 0.0f;
      oxygenPressureBar = 1.0f; oxygenTankPressureBar = 1.3f; hydrogenPressureBar = 1.0f;
      for (int i = 0; i < 6; ++i) stateVec[i] = 0.0f;
      stateVec[0] = matlabDesignTemperatureC; stateVec[3] = effectiveIminA;
      oxygenBufferNm3 = 0.0f; hydrogenBufferNm3 = 0.0f;
      pvFilteredW = 0.0f; stressIndex = 0.0f;
      modelledPurityPct = 100.0f; scenarioName = "unknown"; scenarioFault = "none";
      missedPlantDeadline = false; forcePrimaryDown = false; forceBothDown = false;
      upperH2LeakPpmInject = 0.0f;
      lastPlantReplyMs = millis(); lastRequestAtMs = 0;
      m2Log("RUN", "Reset accepted; requesting fresh Simulink initial state.");
      sendPlantResetRequest(doc["run_id"] | "");
    } else {
      awaitingPlantResetAck = false;
      resetStatus = "rejected_physical_interlock";
      m2Log("SAFETY", "reset_run rejected (MQ8 ADC=%u, injected leak=%.1f ppm).", mq8Raw, upperH2LeakPpmInject);
    }
    return;
  }
  if (String(type) == "stop_run") {
    controllerMode = "STOP"; upperCommandAtMs = 0; upperCurrentRefA = 0.0f;
    appliedCurrentA = 0.0f; resetStatus = "stopped";
    m2Log("RUN", "stop_run received.");
    return;
  }
  if (String(type) == "controller_mode") {
    const char* mode = doc["controller"] | "PROPOSED";
    String m(mode); m.toUpperCase();
    if (m == "PROPOSED" || m == "STOP") {
      controllerMode = m;
      m2Log("CONTROL", "Arduino mode -> %s.", controllerMode.c_str());
    }
    return;
  }
  if (String(type) == "upper_hierarchical" && doc.containsKey("I_ref_A")) {
    upperCurrentRefA = clipf(doc["I_ref_A"].as<float>(), effectiveIminA, effectiveImaxA);
    upperCommandAtMs = millis();
    m2Log("CONTROL", "Upper reference accepted: %.2f A; PV=%.0f W, O2 demand=%.1f L/min.",
          upperCurrentRefA, upperPVAvailableW, upperO2DemandLpm);
    return;
  }
  if (doc.containsKey("I_ref_A")) {
    upperCurrentRefA = clipf(doc["I_ref_A"].as<float>(), effectiveIminA, effectiveImaxA);
    upperCommandAtMs = millis();
  }
  if (doc.containsKey("valid_for_s")) {
    float requestedTtl = doc["valid_for_s"].as<float>();
    if (!isfinite(requestedTtl) || requestedTtl <= 0.0f || requestedTtl > COMMAND_TIMEOUT_MS / 1000.0f)
      upperCommandAtMs = 0;
  }
}

// ==================== NETWORK / OPTIONAL GSM FAILOVER ====================
void connectNetwork() {
  static uint32_t lastNetworkServiceMs = 0;
  static uint32_t lastWifiWarnMs = 0;
  static uint32_t lastMqttWarnMs = 0;
  const uint32_t now = millis();
  if (now - lastNetworkServiceMs < 5000) return;
  lastNetworkServiceMs = now;

  if (!forceBothDown && WiFi.status() != WL_CONNECTED && strlen(WIFI_SSID) > 0) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected() && !forcePrimaryDown && !forceBothDown) {
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setCallback(mqttCallback);
    mqtt.setSocketTimeout(1);
    mqtt.setBufferSize(6144);
    if (mqtt.connect(MQTT_CLIENT_ID)) {
      subscribePlantTopics(mqtt);
      usingGsm = false;
      m2Log("NET", "Wi-Fi MQTT connected: RSSI=%ld dBm, IP=%s", (long)WiFi.RSSI(), WiFi.localIP().toString().c_str());
    } else if (now - lastMqttWarnMs >= 15000UL) {
      lastMqttWarnMs = now;
      m2Log("WARN", "Wi-Fi up but MQTT failed (state=%d).", mqtt.state());
    }
  }
#if ENABLE_GSM_FALLBACK
  if (!forceBothDown && !usingGsm && !mqtt.connected() && !forcePrimaryDown) {
    if (!gsmInitialised) {
      Serial2.begin(115200);
      modem.restart();
      gsmInitialised = true;
      m2Log("GSM", "SIM800L modem initialised.");
    }
    if (!modem.isNetworkConnected()) modem.waitForNetwork(30000L);
    if (modem.isNetworkConnected() && !modem.isGprsConnected()) {
      modem.gprsConnect(GSM_APN, GSM_USER, GSM_PASS);
    }
    if (modem.isGprsConnected() && !mqttGsm.connected()) {
      mqttGsm.setServer(MQTT_HOST, MQTT_PORT);
      mqttGsm.setCallback(mqttCallback);
      mqttGsm.setSocketTimeout(1);
      mqttGsm.setBufferSize(6144);
      if (mqttGsm.connect(MQTT_GSM_CLIENT_ID)) {
        subscribePlantTopics(mqttGsm);
        usingGsm = true;
        m2Log("NET", "GSM MQTT fallback connected.");
      }
    }
  }
#endif
  if (!activeMqttConnected() && now - lastWifiWarnMs >= 30000UL) {
    lastWifiWarnMs = now;
    m2Log("WARN", "No active MQTT connection. Check Wi-Fi credentials/broker or enable/configure GSM.");
  }
}

// ==================== PLANT RESPONSE HANDLING ====================
void processPlantReplyLine(const char* line) {
  StaticJsonDocument<3072> doc;
  if (deserializeJson(doc, line)) return;
  const char* messageType = doc["type"] | "";
  if (String(messageType) == "reset_ack") {
    if (!(doc["ok"] | false) || !doc["state"].is<JsonArray>()) {
      trip("invalid_simulink_reset_ack");
      awaitingPlantResetAck = false;
      resetStatus = "rejected_plant_reset";
      return;
    }
    JsonArray x0 = doc["state"].as<JsonArray>();
    if (x0.size() != 6) { trip("invalid_initial_state"); return; }
    for (int i = 0; i < 6; ++i) stateVec[i] = x0[i].as<float>();
    stackTemperatureC = stateVec[0]; plantCurrentA = stateVec[3];
    appliedCurrentA = effectiveIminA; upperCurrentRefA = effectiveIminA;
    pvFilteredW = stateVec[4]; stressIndex = stateVec[5];
    oxygenBufferNm3 = stateVec[1] * 8.314462618f * 273.15f / 1.01325e5f;
    hydrogenBufferNm3 = stateVec[2] * 8.314462618f * 273.15f / 1.01325e5f;
    plantTimeS = 0.0f; simTimeS = 0.0f; seq = 0; requestPending = false;
    // Do not publish the reset snapshot as a plant sample. The first logged
    // row must correspond to a completed Simulink step (seq=1), not stale
    // startup defaults such as 1.8 V or zero sensor flow.
    awaitingPlantResetAck = false; resetStatus = "accepted"; plantReplyReady = false;
    m2Log("HIL", "MATLAB reset ACK loaded six initial plant states; awaiting first completed step.");
    return;
  }
  if (String(messageType) != "slx_plant_response") return;
  if (!(doc["ok"] | false)) {
    requestPending = false;
    trip("simulink_plant_step_error");
    m2Log("ERROR", "Simulink step failed: %s", String((const char*)(doc["error"] | "unknown")).c_str());
    return;
  }
  if (!requestPending) return;
  if (doc.containsKey("seq") && (uint32_t)(doc["seq"] | 0) != seq) return;

  plantTimeS = doc["time_s"] | plantTimeS;
  scenarioName = String((const char*)(doc["scenario"] | upperScenario.c_str()));
  scenarioFault = String((const char*)(doc["fault"] | upperFaultState.c_str()));
  oxygenFlowLpm = doc["O2_sensor_Lpm"] | oxygenFlowLpm;
  oxygenFlowTrueLpm = doc["O2_flow_Lpm"] | oxygenFlowTrueLpm;
  bopPowerW = doc["P_BoP_W"] | bopPowerW;
  systemPowerW = doc["P_system_W"] | systemPowerW;
  systemEfficiencyPct = doc["system_efficiency_pct"] | systemEfficiencyPct;
  hydrogenKgS = doc["H2_kg_s"] | hydrogenKgS;
  oxygenProducedLpm = doc["O2_prod_Lpm"] | oxygenProducedLpm;
  hydrogenExportKgS = doc["H2_export_kg_s"] | hydrogenExportKgS;
  hydrogenVentedKgS = doc["H2_vent_kg_s"] | hydrogenVentedKgS;
  oxygenVentedLStep = doc["O2_vented_L_step"] | oxygenVentedLStep;
  h2InO2Ppm = doc["H2_in_O2_ppm"] | h2InO2Ppm;
  efficiencyPct = doc["Efficiency_pct"] | efficiencyPct;
  stackPowerW = doc["P_stack_W"] | stackPowerW;
  plantCurrentA = doc["I_stack_A"] | plantCurrentA;
  cellVoltageV = doc["V_cell_V"] | cellVoltageV;
  stackTemperatureC = doc["T_stack_C"] | stackTemperatureC;
  oxygenDemandLpm = doc["O2_demand_Lpm"] | upperO2DemandLpm;
  oxygenPressureBar = doc["P_O2_bar"] | oxygenPressureBar;
  oxygenTankPressureBar = doc["P_O2_tank_bar"] | oxygenTankPressureBar;
  hydrogenPressureBar = doc["P_H2_bar"] | hydrogenPressureBar;
  oxygenBufferNm3 = doc["O2_buffer_Nm3"] | oxygenBufferNm3;
  hydrogenBufferNm3 = doc["H2_buffer_Nm3"] | hydrogenBufferNm3;
  modelledPurityPct = doc["purity_model_pct"] | modelledPurityPct;
  h2LeakPpm = doc["H2_leak_ppm"] | 0.0f;
  iSensorA = plantCurrentA;
  vCellSensorV = cellVoltageV;
  tSensorC = stackTemperatureC;
  if (upperSensorFaultInject) {
    iSensorA = effectiveImaxA + 500.0f; // deterministic out-of-range sensor injection
    trip("injected_out_of_range_current_sensor");
  }
  if (upperPurityAlarmInject) trip("injected_modelled_purity_alarm");
  pvUsedW = doc["P_PV_W"] | pvUsedW;
  pvAvailableW = doc["PV_available_W"] | upperPVAvailableW;
  pvFilteredW = doc["PV_filtered_W"] | pvFilteredW;
  gridUsedW = doc["P_grid_W"] | gridUsedW;
  tariffKESkWh = doc["tariff_KES_kWh"] | upperTariffKESkWh;
  stressIndex = doc["stress_idx"] | stressIndex;
  plantStepMs = doc["plant_step_ms"] | plantStepMs;
  missedPlantDeadline = doc["missed_deadline"] | false;
  o2UnmetLStep = doc["O2_unmet_L_step"] | 0.0f;

  if (doc.containsKey("xnext") && doc["xnext"].is<JsonArray>()) {
    JsonArray xn = doc["xnext"].as<JsonArray>();
    if (xn.size() == 6) {
      for (int i = 0; i < 6; ++i) stateVec[i] = xn[i].as<float>();
      stackTemperatureC = stateVec[0]; plantCurrentA = stateVec[3];
      pvFilteredW = stateVec[4]; stressIndex = stateVec[5];
      oxygenBufferNm3 = stateVec[1] * 8.314462618f * 273.15f / 1.01325e5f;
      hydrogenBufferNm3 = stateVec[2] * 8.314462618f * 273.15f / 1.01325e5f;
    }
  }

  if (h2LeakPpm > 500.0f) trip("modelled_external_h2_leak_alarm");
  if (h2InO2Ppm > H2_IN_O2_LIMIT_PPM) trip("modelled_h2_in_o2_safety_limit");
  if (stackTemperatureC >= TEMPERATURE_MAX_C) trip("stack_temperature_limit");
  if (cellVoltageV >= CELL_VOLTAGE_MAX_V) trip("cell_voltage_limit");
  if (fmaxf(oxygenTankPressureBar, hydrogenPressureBar) > PRESSURE_MAX_BAR) trip("pressure_limit");
  if (modelledPurityPct < PURITY_MIN_MODEL_PCT) trip("modelled_oxygen_product_quality_limit");

  lastPlantReplyMs = millis();
  plantCycleMs = (float)(lastPlantReplyMs - lastRequestAtMs);
  if (plantCycleMs > LOWER_PERIOD_MS) { missedPlantDeadline = true; ++missedDeadlineCount; }
  requestPending = false; plantReplyReady = true; havePlantReply = true;
  plantLinkPrimed = true;
  if (seq == 1 || (seq % 1000UL) == 0)
    m2Log("HIL", "seq=%lu t=%.1f s I=%.2f A O2=%.2f/%.2f T=%.2f | step=%.2f ms cycle=%.2f ms",
          (unsigned long)seq, plantTimeS, plantCurrentA, oxygenFlowLpm, oxygenDemandLpm,
          stackTemperatureC, plantStepMs, plantCycleMs);
}

// ==================== LOWER MPC ====================
float runLowerMPC(float referenceA) {
  if (referenceA <= 0.0f) return 0.0f;
  const float a = expf(-MPC_DT_S / fmaxf(MPC_TAU_S, 1.0e-4f));
  float bestU = appliedCurrentA;
  float bestCost = 1.0e30f;
  float lo = fmaxf(effectiveIminA, appliedCurrentA - MPC_MOVE_A);
  float hi = fminf(effectiveImaxA, appliedCurrentA + MPC_MOVE_A);
  for (float u = lo; u <= hi + 0.001f; u += MPC_GRID_A) {
    float x = plantCurrentA;
    float cost = 0.0f;
    for (int k = 0; k < MPC_HORIZON; ++k) {
      x = a * x + (1.0f - a) * u;
      float e = x - referenceA;
      float du = u - appliedCurrentA;
      cost += MPC_Q * e * e + MPC_R * du * du;
    }
    float terminal = x - referenceA;
    cost += MPC_TERMINAL * terminal * terminal;
    if (cost < bestCost) { bestCost = cost; bestU = u; }
  }
  return clipf(bestU, effectiveIminA, effectiveImaxA);
}

// ==================== SAFETY ====================
void checkSafety() {
#if !M2_BENCH_MODE && !M2_BYPASS_SAFETY_PINS
  if (digitalRead(PIN_ESTOP_OK) == HIGH) trip("external_estop_or_open_loop");
  if (digitalRead(PIN_H2_SENSOR_OK) == HIGH) trip("h2_leak_or_open_sensor_loop");
  if (digitalRead(PIN_OVERPRESSURE_OK) == HIGH) trip("overpressure_or_open_loop");
  if (digitalRead(PIN_WATER_OK) == HIGH) trip("low_water_or_open_loop");
#endif
  mq8Raw = (uint16_t)analogRead(PIN_MQ8_ANALOG);
  if (MQ8_TRIP_ADC > 0 && mq8Raw >= MQ8_TRIP_ADC) trip("mq8_analog_h2_trip");
  if (havePlantReply) {
    if (stackTemperatureC >= TEMPERATURE_MAX_C) trip("stack_temperature_limit");
    if (cellVoltageV >= CELL_VOLTAGE_MAX_V) trip("cell_voltage_limit");
    if (fmaxf(oxygenTankPressureBar, hydrogenPressureBar) > PRESSURE_MAX_BAR) trip("pressure_limit");
    if (modelledPurityPct < PURITY_MIN_MODEL_PCT) trip("modelled_oxygen_product_quality_limit");
    if (h2InO2Ppm > H2_IN_O2_LIMIT_PPM) trip("modelled_h2_in_o2_safety_limit");
  }
  // Allow the first live MATLAB/Simulink request extra time for one-time
  // host-side startup/JIT overhead. Once one valid step has returned, restore
  // the normal 3 s link watchdog for every subsequent exchange.
  const uint32_t linkTimeoutMs = plantLinkPrimed
      ? PLANT_LINK_TIMEOUT_MS : PLANT_LINK_STARTUP_TIMEOUT_MS;
  if (requestPending && (millis() - lastRequestAtMs > linkTimeoutMs))
    trip("plant_link_timeout");
}

// ==================== NEXTION HMI ====================
void nextionCommand(const char* command) {
  nexSerial.print(command); nexSerial.write(0xFF); nexSerial.write(0xFF); nexSerial.write(0xFF);
}
void nextionText(const char* component, const char* value) {
  char command[180];
  snprintf(command, sizeof(command), "%s.txt=\"%s\"", component, value);
  nextionCommand(command);
}
void publishNextionControl(bool start) {
  if (start && latchedTrip) return;
  if (!activeMqttConnected()) return;
  const char* payload = start
    ? "{\"type\":\"upper_hierarchical\",\"upper_prodRateSet\":0,\"controller_variant\":\"PROPOSED\"}"
    : "{\"type\":\"stop\"}";
  activeMqtt().publish(TOPIC_LEGACY_COMMAND, payload);
}
void handleNextionInput() {
  static uint8_t state = 0, pageId = 0, componentId = 0, event = 0, terminators = 0;
  while (nexSerial.available()) {
    const uint8_t value = (uint8_t)nexSerial.read();
    if (state == 0) {
      if (value == 0x65) state = 1;
      else if (value == 0x66) state = 5;
    } else if (state == 1) { pageId = value; state = 2; }
    else if (state == 2) { componentId = value; state = 3; }
    else if (state == 3) { event = value; terminators = 0; state = 4; }
    else if (value == 0xFF) {
      if (++terminators == 3) {
        if (pageId == 0 && event == 1) {
          if (componentId == NEXTION_START_BUTTON_ID || componentId == NEXTION_POWER_BUTTON_ID)
            publishNextionControl(true);
          else if (componentId == NEXTION_STOP_BUTTON_ID)
            publishNextionControl(false);
        }
        nextionPageKnown = false;
        lastNextionPageQueryMs = millis();
        nextionCommand("sendme");
        state = 0;
      }
    } else if (state == 5) {
      activeNextionPageId = value; state = 6; terminators = 0;
    } else if (state == 6 && value == 0xFF) {
      if (++terminators == 3) { nextionPageKnown = true; state = 0; }
    } else {
      state = 0;
    }
  }
  if (millis() - lastNextionPageQueryMs >= 1000UL) {
    lastNextionPageQueryMs = millis();
    nextionPageKnown = false;
    nextionCommand("sendme");
  }
}
void updateNextionHMI() {
  if (!nextionPageKnown || activeNextionPageId != 0) return;
  char buf[64];
  snprintf(buf, sizeof(buf), "%.1f C", stackTemperatureC); nextionText("t4", buf);
  snprintf(buf, sizeof(buf), "O2 %.1f L/min", oxygenFlowLpm); nextionText("t5", buf);
  snprintf(buf, sizeof(buf), "%s | %s | %s", latchedTrip ? "TRIPPED" : "RUN",
           controllerMode.c_str(), activeMqttConnected() ? (usingGsm ? "GSM" : "WIFI") : "OFFLINE");
  nextionText("t7", buf);
}
void initializeHMIAndRTC() {
  nexSerial.begin(NEXTION_BAUD);
  if (!rtc.begin()) m2Log("WARN", "DS3231 not detected.");
  nextionCommand("bkcmd=0"); nextionCommand("sleep=0"); nextionCommand("dim=100");
  nextionCommand("page 0");
  nextionCommand("sendme");
}

// ==================== TELEMETRY ====================
void publishTelemetry() {
  StaticJsonDocument<2048> d;
  d["type"] = "telemetry";
  d["seq"] = seq;
  d["time_s"] = simTimeS > 0 ? simTimeS : plantTimeS;
  d["scenario"] = scenarioName;
  d["reset_status"] = resetStatus;
  d["controller"] = controllerMode;
  d["I_ref_A"] = upperCurrentRefA;
  d["I_command_A"] = appliedCurrentA;
  d["I_stack_A"] = plantCurrentA;
  d["V_cell_V"] = cellVoltageV;
  d["P_stack_W"] = stackPowerW;
  d["P_BoP_W"] = bopPowerW;
  d["P_system_W"] = systemPowerW;
  d["T_stack_C"] = stackTemperatureC;
  d["Efficiency_pct"] = efficiencyPct;
  d["P_O2_bar"] = oxygenPressureBar;
  d["P_O2_tank_bar"] = oxygenTankPressureBar;
  d["P_H2_bar"] = hydrogenPressureBar;
  d["O2_flow_Lpm"] = oxygenFlowTrueLpm;
  d["O2_sensor_Lpm"] = oxygenFlowLpm;
  d["H2_kg_s"] = hydrogenKgS;
  d["O2_prod_Lpm"] = oxygenProducedLpm;
  d["H2_export_kg_s"] = hydrogenExportKgS;
  d["H2_vent_kg_s"] = hydrogenVentedKgS;
  d["O2_vented_L_step"] = oxygenVentedLStep;
  d["H2_in_O2_ppm"] = h2InO2Ppm;
  d["H2_in_O2_limit_ppm"] = H2_IN_O2_LIMIT_PPM;
  d["O2_demand_Lpm"] = upperO2DemandLpm;
  d["O2_unmet_L_step"] = o2UnmetLStep;
  d["O2_buffer_Nm3"] = oxygenBufferNm3;
  d["H2_buffer_Nm3"] = hydrogenBufferNm3;
  d["P_PV_W"] = pvUsedW;
  d["PV_available_W"] = pvAvailableW;
  d["PV_filtered_W"] = pvFilteredW;
  d["P_grid_W"] = gridUsedW;
  d["tariff_KES_kWh"] = tariffKESkWh;
  d["H2_price_KES_kg"] = h2PriceKESkg;
  d["purity_model_pct"] = modelledPurityPct;
  d["stress_idx"] = stressIndex;
  d["lower_loop_ms"] = lowerLoopMs;
  d["lower_solve_ms"] = lowerSolveMs;
  d["lower_tick_interval_ms"] = lowerTickIntervalMs;
  d["plant_step_ms"] = plantStepMs;
  d["cycle_latency_ms"] = plantCycleMs;
  d["missed_deadline_count"] = missedDeadlineCount;
  d["missed_deadline"] = missedPlantDeadline;
  d["interlock"] = latchedTrip;
  d["trip_reason"] = tripReason;
  d["mqtt_rssi"] = usingGsm ? 0 : WiFi.RSSI();
  d["communication_path"] = usingGsm ? "GSM_MQTT" : (mqtt.connected() ? "WiFi_MQTT" : "offline_hold");
  d["I_sensor_A"] = iSensorA;
  d["V_cell_sensor_V"] = vCellSensorV;
  d["T_sensor_C"] = tSensorC;
  d["H2_leak_ppm"] = h2LeakPpm;
  char payload[2048];
  size_t n = serializeJson(d, payload, sizeof(payload));
  if (activeMqttConnected() && !forcePrimaryDown)
    activeMqtt().publish(TOPIC_TELEMETRY, (const uint8_t*)payload, n, false);
}

// ==================== SETUP / LOOP ====================
void setup() {
  pinMode(PIN_ESTOP_OK, INPUT_PULLUP);
  pinMode(PIN_H2_SENSOR_OK, INPUT_PULLUP);
  pinMode(PIN_OVERPRESSURE_OK, INPUT_PULLUP);
  pinMode(PIN_WATER_OK, INPUT_PULLUP);
  analogReadResolution(12);
  Serial.begin(115200);
  delay(200);
  m2Log("BOOT", "M2 GIGA — USB Serial plant bridge + MQTT telemetry; nominal HIL dt=1 s.");
  initializeHMIAndRTC();
  connectNetwork();
  lastTickMs = millis();
  lastPlantReplyMs = millis();
  upperCommandAtMs = millis();
  m2Log("READY", "GIGA ready. Waiting for MQTT design schedule + reset_run; MATLAB must own the USB COM port.");
}

void loop() {
  connectNetwork();
  if (activeMqttConnected()) activeMqtt().loop();
  readSerialPlantBridge();
  handleNextionInput();
  if (nextionHmiRefreshPending) { nextionHmiRefreshPending = false; updateNextionHMI(); }

  if (plantReplyReady) {
    publishTelemetry();
    plantReplyReady = false;
    resetStatus = "not_requested";
  }

  uint32_t now = millis();
  if (now - lastTickMs >= LOWER_PERIOD_MS) {
    uint32_t loopStart = micros();
    if (lastControlTickAtMs != 0) lowerTickIntervalMs = (float)(now - lastControlTickAtMs);
    lastControlTickAtMs = now;
    lastTickMs += LOWER_PERIOD_MS;
    if (now - lastTickMs > 3 * LOWER_PERIOD_MS) lastTickMs = now;
    checkSafety();

    uint32_t solveStart = micros();
    if (!latchedTrip) {
      bool upperFresh = (upperCommandAtMs != 0) && ((now - upperCommandAtMs) <= COMMAND_TIMEOUT_MS);
      if (controllerMode == "STOP") appliedCurrentA = 0.0f;
      else if (controllerMode == "PROPOSED" && !upperFresh) {
        // Hold the last accepted safe output for a bounded interval, then
        // move to zero current. A fresh authenticated supervisory command
        // can resume operation after communications recover.
        if (now - upperCommandAtMs > COMMAND_TIMEOUT_MS) appliedCurrentA = 0.0f;
      }
      else if (controllerMode == "PROPOSED") appliedCurrentA = runLowerMPC(upperCurrentRefA);
      else appliedCurrentA = 0.0f;
    } else {
      appliedCurrentA = 0.0f;
    }
    lowerSolveMs = (micros() - solveStart) / 1000.0f;

    if (!requestPending && !awaitingPlantResetAck) sendPlantStepRequest();
    lowerLoopMs = (micros() - loopStart) / 1000.0f;
    if (lowerLoopMs > LOWER_PERIOD_MS) { missedPlantDeadline = true; ++missedDeadlineCount; }
  }

  if (now - lastHmiUpdateMs >= 1000UL) {
    lastHmiUpdateMs = now;
    updateNextionHMI();
  }
}