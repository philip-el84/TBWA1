#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <DallasTemperature.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <OneWire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include "soc/gpio_struct.h"
#include <atomic>
#include <cmath>
#include "driver/gpio.h"
#include "protocol.h"

namespace {
constexpr int UART_PIN = 13;
constexpr uint8_t PUMP_PIN_1 = 25;
constexpr uint8_t PUMP_PIN_2 = 26;
constexpr uint8_t PUMP_PIN_3 = 27;
constexpr uint8_t LED_PIN = 2;
constexpr uint8_t ONE_WIRE_PIN = 4;

constexpr uint8_t kPumpCount = 3;
constexpr uint8_t kPwmResolutionBits = 8;
constexpr uint32_t kPwmFrequencyHz = 2000;
constexpr uint8_t kPwmChannels[kPumpCount] = {0, 1, 2};
constexpr uint8_t kPumpPins[kPumpCount] = {PUMP_PIN_1, PUMP_PIN_2, PUMP_PIN_3};

constexpr TickType_t BUS_OK_AGE_TICKS = pdMS_TO_TICKS(5000);
constexpr TickType_t TELEMETRY_PERIOD_TICKS = pdMS_TO_TICKS(2000);
constexpr TickType_t DS18_SAMPLE_PERIOD_TICKS = pdMS_TO_TICKS(2000);
constexpr TickType_t BUS_POLL_TICKS = pdMS_TO_TICKS(2000);
constexpr TickType_t WIFI_RETRY_TICKS = pdMS_TO_TICKS(2000);
constexpr TickType_t WS_PUSH_PERIOD_TICKS = pdMS_TO_TICKS(500);

constexpr const char *kWifiSsid = "YOUR_WIFI_SSID";
constexpr const char *kWifiPassword = "YOUR_WIFI_PASSWORD";
constexpr const char *kMdnsHost = "heizungkasten";

struct SensorSnapshot {
  PayloadSensor packet;
  TickType_t rx_tick;
  bool valid;
};

enum class PumpCommandType : uint8_t { SensorUpdate, Timeout, SetModeAuto, SetModeManual, SetManualMask, SetThreshold };

struct PumpCommand {
  PumpCommandType type;
  PayloadSensor packet;
  uint8_t manual_mask;
  uint16_t threshold_raw;
};

enum class LedEventType : uint8_t { RxPacket };
struct LedEvent {
  LedEventType type;
};

struct BusTxMessage {
  PacketType type;
  uint8_t len;
  uint8_t payload[kMaxPayloadSize];
};

struct PwmTargetMessage {
  uint8_t pump_index;
  uint8_t target_pwm;
};

QueueHandle_t gPumpQueue = nullptr;
QueueHandle_t gLedQueue = nullptr;
QueueHandle_t gBusTxQueue = nullptr;
QueueHandle_t gSensorSnapshotQueue = nullptr;
QueueHandle_t gPwmTargetQueue = nullptr;
TimerHandle_t gPumpTimeoutTimer = nullptr;

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);
FrameParser gParser;
AsyncWebServer gServer(80);
AsyncWebSocket gWs("/ws");
Preferences preferences;

std::atomic<bool> telemetryActive{false};
std::atomic<float> gTemp0C{NAN}, gTemp1C{NAN}, gTemp2C{NAN};
std::atomic<uint8_t> gPumpMask{0};
std::atomic<uint8_t> gCurrentPwm[kPumpCount];
std::atomic<uint8_t> gTargetPwm[kPumpCount];
std::atomic<uint8_t> gMaxPwmValue[kPumpCount];
std::atomic<uint16_t> gPumpThresholds[kPumpCount];
std::atomic<uint16_t> gRampSpeedMs{20};
std::atomic<uint32_t> gPumpTimeoutMs{10000};
std::atomic<bool> gAutoMode{true};
std::atomic<uint8_t> gManualMask{0};
volatile TickType_t gLastPacketTick = 0;

void queuePwmTarget(const uint8_t pumpIndex, const uint8_t targetPwm) {
  if (pumpIndex >= kPumpCount) {
    return;
  }
  PwmTargetMessage msg{pumpIndex, targetPwm};
  xQueueSend(gPwmTargetQueue, &msg, 0);
}

void setPumpTargetsFromMask(uint8_t mask) {
  for (uint8_t i = 0; i < kPumpCount; ++i) {
    const bool enable = (mask & (1u << i)) != 0;
    const uint8_t target = enable ? gMaxPwmValue[i].load(std::memory_order_relaxed) : 0;
    queuePwmTarget(i, target);
  }
}

bool queueBusMessage(PacketType type, const uint8_t *data, uint8_t len) {
  if (len > kMaxPayloadSize) {
    return false;
  }
  BusTxMessage out{};
  out.type = type;
  out.len = len;
  if (len > 0 && data != nullptr) {
    memcpy(out.payload, data, len);
  }
  return xQueueSend(gBusTxQueue, &out, pdMS_TO_TICKS(50)) == pdPASS;
}

void sendCommand(const char *cmd) {
  const size_t n = strnlen(cmd, kMaxTextPayload - 1);
  queueBusMessage(PacketType::Command, reinterpret_cast<const uint8_t *>(cmd), static_cast<uint8_t>(n));
}

void pumpTimeoutCallback(TimerHandle_t) {
  PumpCommand cmd{};
  cmd.type = PumpCommandType::Timeout;
  xQueueSend(gPumpQueue, &cmd, 0);
}

void armPumpTimeoutTimer() {
  const uint32_t timeoutMs = gPumpTimeoutMs.load(std::memory_order_relaxed);
  if (timeoutMs == 0) {
    xTimerStop(gPumpTimeoutTimer, 0);
    return;
  }
  xTimerChangePeriod(gPumpTimeoutTimer, pdMS_TO_TICKS(timeoutMs), 0);
  xTimerReset(gPumpTimeoutTimer, 0);
}

void pushTelemetryJson() {
  SensorSnapshot snap{};
  const bool haveSnap = xQueuePeek(gSensorSnapshotQueue, &snap, 0) == pdPASS && snap.valid;

  StaticJsonDocument<768> doc;
  doc["mode"] = gAutoMode.load(std::memory_order_relaxed) ? "auto" : "manual";
  doc["manual_mask"] = gManualMask.load(std::memory_order_relaxed);
  doc["pump_mask"] = gPumpMask.load(std::memory_order_relaxed);
  doc["online"] = haveSnap;
  doc["ramp_speed"] = gRampSpeedMs.load(std::memory_order_relaxed);
  doc["pump_timeout_s"] = gPumpTimeoutMs.load(std::memory_order_relaxed) / 1000;
  JsonObject temp = doc.createNestedObject("temperature_c");
  temp["t0"] = gTemp0C.load(std::memory_order_relaxed);
  temp["t1"] = gTemp1C.load(std::memory_order_relaxed);
  temp["t2"] = gTemp2C.load(std::memory_order_relaxed);

  JsonArray pumps = doc.createNestedArray("pumps");
  for (uint8_t i = 0; i < kPumpCount; ++i) {
    JsonObject p = pumps.createNestedObject();
    p["id"] = i;
    p["current_pwm"] = gCurrentPwm[i].load(std::memory_order_relaxed);
    p["target_pwm"] = gTargetPwm[i].load(std::memory_order_relaxed);
    p["max_pwm_value"] = gMaxPwmValue[i].load(std::memory_order_relaxed);
    p["threshold"] = gPumpThresholds[i].load(std::memory_order_relaxed);
  }

  JsonObject sensor = doc.createNestedObject("sensor");
  if (haveSnap) {
    JsonArray hw = sensor.createNestedArray("hw390_raw");
    for (uint8_t i = 0; i < 5; ++i) hw.add(snap.packet.hw390_raw[i]);
    JsonArray ldr = sensor.createNestedArray("ldr_raw");
    for (uint8_t i = 0; i < 2; ++i) ldr.add(snap.packet.ldr_raw[i]);
    sensor["tank_full"] = snap.packet.tank_full;
  }

  String payload;
  serializeJson(doc, payload);
  gWs.textAll(payload);
}

void handleApiConfig(AsyncWebServerRequest *request, const uint8_t *data, size_t len) {
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) {
    request->send(400, "application/json", "{\"ok\":false,\"error\":\"invalid_json\"}");
    return;
  }

  if (doc.containsKey("mode")) {
    const char *mode = doc["mode"];
    PumpCommand cmd{};
    if (mode != nullptr && strcasecmp(mode, "auto") == 0) {
      cmd.type = PumpCommandType::SetModeAuto;
      xQueueSend(gPumpQueue, &cmd, pdMS_TO_TICKS(10));
      gAutoMode.store(true, std::memory_order_relaxed);
    } else if (mode != nullptr && strcasecmp(mode, "manual") == 0) {
      cmd.type = PumpCommandType::SetModeManual;
      xQueueSend(gPumpQueue, &cmd, pdMS_TO_TICKS(10));
      gAutoMode.store(false, std::memory_order_relaxed);
    }
  }

  if (doc.containsKey("manual_mask")) {
    PumpCommand cmd{};
    cmd.type = PumpCommandType::SetManualMask;
    cmd.manual_mask = static_cast<uint8_t>(doc["manual_mask"].as<uint16_t>() & 0x07u);
    xQueueSend(gPumpQueue, &cmd, pdMS_TO_TICKS(10));
    gManualMask.store(cmd.manual_mask, std::memory_order_relaxed);
  }

  if (doc.containsKey("ramp_speed")) {
    const uint16_t rampMs = static_cast<uint16_t>(doc["ramp_speed"].as<uint32_t>());
    const uint16_t safeRampMs = max<uint16_t>(1, rampMs);
    gRampSpeedMs.store(safeRampMs, std::memory_order_relaxed);
    preferences.putUShort("ramp_ms", safeRampMs);
  }

  if (doc.containsKey("pump_timeout_s")) {
    const uint32_t timeoutS = static_cast<uint32_t>(constrain(doc["pump_timeout_s"].as<int>(), 0, 60));
    gPumpTimeoutMs.store(timeoutS * 1000u, std::memory_order_relaxed);
    preferences.putUInt("timeout_s", timeoutS);
  }

  if (doc.containsKey("max_pwm_value") && doc["max_pwm_value"].is<JsonArray>()) {
    JsonArray values = doc["max_pwm_value"].as<JsonArray>();
    for (uint8_t i = 0; i < kPumpCount && i < values.size(); ++i) {
      const uint8_t pwm = static_cast<uint8_t>(constrain(values[i].as<int>(), 0, 255));
      gMaxPwmValue[i].store(pwm, std::memory_order_relaxed);
      char key[8];
      snprintf(key, sizeof(key), "max%u", i);
      preferences.putUChar(key, pwm);
    }
  }

  if (doc.containsKey("pump_thresholds") && doc["pump_thresholds"].is<JsonArray>()) {
    JsonArray values = doc["pump_thresholds"].as<JsonArray>();
    for (uint8_t i = 0; i < kPumpCount && i < values.size(); ++i) {
      const uint16_t threshold = static_cast<uint16_t>(constrain(values[i].as<int>(), 0, 4095));
      gPumpThresholds[i].store(threshold, std::memory_order_relaxed);
      char key[8];
      snprintf(key, sizeof(key), "thr%u", i);
      preferences.putUShort(key, threshold);
    }
    PumpCommand cmd{};
    cmd.type = PumpCommandType::SetThreshold;
    cmd.threshold_raw = gPumpThresholds[0].load(std::memory_order_relaxed);
    xQueueSend(gPumpQueue, &cmd, pdMS_TO_TICKS(10));
  }

  request->send(200, "application/json", "{\"ok\":true}");
}

void onWsEvent(AsyncWebSocket *, AsyncWebSocketClient *, AwsEventType type, void *, uint8_t *, size_t) {
  if (type == WS_EVT_CONNECT) {
    pushTelemetryJson();
  }
}

void configureWebServer() {
  gWs.onEvent(onWsEvent);
  gServer.addHandler(&gWs);

  gServer.on("/api/config", HTTP_POST,
             [](AsyncWebServerRequest *) {},
             nullptr,
             [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
               if (index != 0 || len != total) {
                 request->send(400, "application/json", "{\"ok\":false,\"error\":\"chunked_not_supported\"}");
                 return;
               }
               handleApiConfig(request, data, len);
             });

  gServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<256> doc;
    doc["mode"] = gAutoMode.load(std::memory_order_relaxed) ? "auto" : "manual";
    doc["manual_mask"] = gManualMask.load(std::memory_order_relaxed);
    doc["pump_mask"] = gPumpMask.load(std::memory_order_relaxed);
    doc["ramp_speed"] = gRampSpeedMs.load(std::memory_order_relaxed);
    doc["pump_timeout_s"] = gPumpTimeoutMs.load(std::memory_order_relaxed) / 1000;
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });

  gServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  gServer.begin();
}

void handleInboundFrame(const FrameMessage &frame) {
  LedEvent ev{LedEventType::RxPacket};
  xQueueSend(gLedQueue, &ev, 0);
  gLastPacketTick = xTaskGetTickCount();

  if (frame.type == PacketType::SensorData && frame.length == sizeof(PayloadSensor)) {
    PumpCommand cmd{};
    cmd.type = PumpCommandType::SensorUpdate;
    memcpy(&cmd.packet, frame.payload, sizeof(PayloadSensor));
    cmd.packet.ds18_c[0] = gTemp0C.load(std::memory_order_relaxed);
    cmd.packet.ds18_c[1] = gTemp1C.load(std::memory_order_relaxed);
    cmd.packet.ds18_c[2] = gTemp2C.load(std::memory_order_relaxed);
    xQueueSend(gPumpQueue, &cmd, 0);

    SensorSnapshot snap{cmd.packet, gLastPacketTick, true};
    xQueueOverwrite(gSensorSnapshotQueue, &snap);
  } else if (frame.type == PacketType::TextMessage && frame.length > 0) {
    char text[kMaxPayloadSize + 1] = {0};
    const uint8_t cpy = min<uint8_t>(frame.length, kMaxPayloadSize);
    memcpy(text, frame.payload, cpy);
    Serial.printf("[STM32] %s\n", text);
  }
}

void Task_UART_Comms(void *) {
  uint8_t frame[kMaxFrameSize] = {0};
  for (;;) {
    BusTxMessage outbound{};
    while (xQueueReceive(gBusTxQueue, &outbound, 0) == pdPASS) {
      const size_t len = encodeFrame(outbound.type, outbound.payload, outbound.len, frame, sizeof(frame));
      if (len > 0) {
        Serial2.write(frame, len);
        Serial2.flush();

        // --- DIE ECHO-VERNICHTUNG ---
        while (Serial2.available() > 0) {
          Serial2.read();
        }
      }
    }

    while (Serial2.available() > 0) {
      const uint8_t b = static_cast<uint8_t>(Serial2.read());
      FrameMessage incoming{};
      if (gParser.parseByte(b, incoming)) {
        handleInboundFrame(incoming);
      }
    }
    vTaskDelay(1);
  }
}

void Task_Bus_Supervisor(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    sendCommand("POLL");
    vTaskDelayUntil(&lastWake, BUS_POLL_TICKS);
  }
}

void Task_Temperatures(void *) {
  ds18b20.begin();
  for (;;) {
    ds18b20.requestTemperatures();
    float temps[3] = {NAN, NAN, NAN};
    for (uint8_t i = 0; i < 3; ++i) {
      const float t = ds18b20.getTempCByIndex(i);
      if (t != DEVICE_DISCONNECTED_C && t > -126.0f && t < 126.0f) {
        temps[i] = t;
      }
    }
    gTemp0C.store(temps[0], std::memory_order_relaxed);
    gTemp1C.store(temps[1], std::memory_order_relaxed);
    gTemp2C.store(temps[2], std::memory_order_relaxed);
    vTaskDelay(DS18_SAMPLE_PERIOD_TICKS);
  }
}

void Task_Pump_Logic(void *) {
  PumpCommand cmd{};
  TickType_t lastSensorRxTick = 0;

  for (;;) {
    if (xQueueReceive(gPumpQueue, &cmd, pdMS_TO_TICKS(200)) == pdPASS) {
      if (cmd.type == PumpCommandType::SetModeAuto) {
        gAutoMode.store(true, std::memory_order_relaxed);
        gManualMask.store(0, std::memory_order_relaxed);
        setPumpTargetsFromMask(0);
        xTimerStop(gPumpTimeoutTimer, 0);
        continue;
      }
      if (cmd.type == PumpCommandType::SetModeManual) {
        gAutoMode.store(false, std::memory_order_relaxed);
        gManualMask.store(0, std::memory_order_relaxed);
        setPumpTargetsFromMask(0);
        xTimerStop(gPumpTimeoutTimer, 0);
        continue;
      }
      if (cmd.type == PumpCommandType::SetThreshold) {
        continue;
      }
      if (cmd.type == PumpCommandType::SetManualMask) {
        const uint8_t manualMask = cmd.manual_mask & 0x07u;
        gManualMask.store(manualMask, std::memory_order_relaxed);
        if (!gAutoMode.load(std::memory_order_relaxed)) {
          setPumpTargetsFromMask(manualMask);
          if (manualMask != 0) {
            armPumpTimeoutTimer();
          } else {
            xTimerStop(gPumpTimeoutTimer, 0);
          }
        }
        continue;
      }
      if (cmd.type == PumpCommandType::Timeout) {
        setPumpTargetsFromMask(0);
        continue;
      }
      if (cmd.type == PumpCommandType::SensorUpdate) {
        lastSensorRxTick = xTaskGetTickCount();
        if (!gAutoMode.load(std::memory_order_relaxed)) {
          continue;
        }
        const auto &p = cmd.packet;
        if (p.tank_full == 0) {
          setPumpTargetsFromMask(0);
          continue;
        }

        uint8_t autoMask = 0;
        for (uint8_t i = 0; i < kPumpCount; ++i) {
          const uint16_t threshold = gPumpThresholds[i].load(std::memory_order_relaxed);
          if (p.hw390_raw[i] < threshold) {
            autoMask |= static_cast<uint8_t>(1u << i);
          }
        }
        setPumpTargetsFromMask(autoMask);
        if (autoMask != 0) {
          armPumpTimeoutTimer();
        } else {
          xTimerStop(gPumpTimeoutTimer, 0);
        }
      }
    }

    if (lastSensorRxTick != 0 && (xTaskGetTickCount() - lastSensorRxTick) > BUS_OK_AGE_TICKS) {
      lastSensorRxTick = 0;
      setPumpTargetsFromMask(0);
      xTimerStop(gPumpTimeoutTimer, 0);
    }
  }
}

void Task_PWM_Ramp(void *) {
  uint8_t current[kPumpCount] = {0, 0, 0};
  uint8_t target[kPumpCount] = {0, 0, 0};

  for (;;) {
    PwmTargetMessage msg{};
    while (xQueueReceive(gPwmTargetQueue, &msg, 0) == pdPASS) {
      if (msg.pump_index < kPumpCount) {
        target[msg.pump_index] = msg.target_pwm;
      }
    }

    bool anyOn = false;
    for (uint8_t i = 0; i < kPumpCount; ++i) {
      if (current[i] < target[i]) {
        ++current[i];
      } else if (current[i] > target[i]) {
        --current[i];
      }
      ledcWrite(kPwmChannels[i], current[i]);
      gCurrentPwm[i].store(current[i], std::memory_order_relaxed);
      gTargetPwm[i].store(target[i], std::memory_order_relaxed);
      anyOn = anyOn || current[i] > 0;
    }

    uint8_t mask = 0;
    for (uint8_t i = 0; i < kPumpCount; ++i) {
      if (current[i] > 0) {
        mask |= static_cast<uint8_t>(1u << i);
      }
    }
    gPumpMask.store(mask, std::memory_order_relaxed);

    const uint16_t rampMs = max<uint16_t>(1, gRampSpeedMs.load(std::memory_order_relaxed));
    (void)anyOn;
    vTaskDelay(pdMS_TO_TICKS(rampMs));
  }
}

void Task_Telemetry(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (telemetryActive.load(std::memory_order_relaxed)) {
      SensorSnapshot snap{};
      const bool haveSnap = xQueuePeek(gSensorSnapshotQueue, &snap, 0) == pdPASS && snap.valid;
      Serial.printf("[TEL] online=%u hw390=[%u,%u,%u,%u,%u] ldr=[%u,%u] ds18=[%.2f,%.2f,%.2f] pwm=[%u,%u,%u] target=[%u,%u,%u] mask=0x%02X mode=%s\n",
                    haveSnap ? 1 : 0,
                    haveSnap ? snap.packet.hw390_raw[0] : 0,
                    haveSnap ? snap.packet.hw390_raw[1] : 0,
                    haveSnap ? snap.packet.hw390_raw[2] : 0,
                    haveSnap ? snap.packet.hw390_raw[3] : 0,
                    haveSnap ? snap.packet.hw390_raw[4] : 0,
                    haveSnap ? snap.packet.ldr_raw[0] : 0,
                    haveSnap ? snap.packet.ldr_raw[1] : 0,
                    gTemp0C.load(std::memory_order_relaxed),
                    gTemp1C.load(std::memory_order_relaxed),
                    gTemp2C.load(std::memory_order_relaxed),
                    gCurrentPwm[0].load(std::memory_order_relaxed),
                    gCurrentPwm[1].load(std::memory_order_relaxed),
                    gCurrentPwm[2].load(std::memory_order_relaxed),
                    gTargetPwm[0].load(std::memory_order_relaxed),
                    gTargetPwm[1].load(std::memory_order_relaxed),
                    gTargetPwm[2].load(std::memory_order_relaxed),
                    gPumpMask.load(std::memory_order_relaxed),
                    gAutoMode.load(std::memory_order_relaxed) ? "auto" : "manual");
    }
    vTaskDelayUntil(&lastWake, TELEMETRY_PERIOD_TICKS);
  }
}

void Task_HMI(void *) {
  String line;
  for (;;) {
    while (Serial.available() > 0) {
      const char c = static_cast<char>(Serial.read());
      if (c == '\r') continue;
      if (c == '\n') {
        line.trim();
        if (line.equalsIgnoreCase("tel on")) telemetryActive.store(true, std::memory_order_relaxed);
        else if (line.equalsIgnoreCase("tel off")) telemetryActive.store(false, std::memory_order_relaxed);
        else if (line.startsWith("cmd ")) {
          const String cmd = line.substring(4);
          sendCommand(cmd.c_str());
        }
        line = "";
      } else if (line.length() < 127) {
        line += c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void Task_WiFi(void *) {
  bool serverStarted = false;
  TickType_t lastWsPush = xTaskGetTickCount();

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      if (WiFi.getMode() != WIFI_STA) {
        WiFi.mode(WIFI_STA);
      }
      WiFi.begin(kWifiSsid, kWifiPassword);
      while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(WIFI_RETRY_TICKS);
      }

      if (!MDNS.begin(kMdnsHost)) {
        Serial.println("[WiFi] mDNS start failed");
      } else {
        MDNS.addService("http", "tcp", 80);
        Serial.printf("[WiFi] mDNS active: %s.local\n", kMdnsHost);
      }

      ArduinoOTA.setHostname(kMdnsHost);
      ArduinoOTA.begin();

      if (!serverStarted) {
        if (!LittleFS.begin(true)) {
          Serial.println("[FS] LittleFS mount failed");
        } else {
          configureWebServer();
          serverStarted = true;
        }
      }
      Serial.printf("[WiFi] Connected, IP=%s\n", WiFi.localIP().toString().c_str());
    }

    ArduinoOTA.handle();
    gWs.cleanupClients();

    const TickType_t now = xTaskGetTickCount();
    if ((now - lastWsPush) >= WS_PUSH_PERIOD_TICKS) {
      pushTelemetryJson();
      lastWsPush = now;
    }

    vTaskDelay(pdMS_TO_TICKS(25));
  }
}
}  // namespace

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(LED_PIN, OUTPUT);
  preferences.begin("scada", false);
  const uint16_t storedRampMs = preferences.getUShort("ramp_ms", 20);
  const uint32_t storedTimeoutS = preferences.getUInt("timeout_s", 10);
  gRampSpeedMs.store(max<uint16_t>(1, storedRampMs), std::memory_order_relaxed);
  gPumpTimeoutMs.store(constrain(storedTimeoutS, 0u, 60u) * 1000u, std::memory_order_relaxed);

  for (uint8_t i = 0; i < kPumpCount; ++i) {
    gCurrentPwm[i].store(0, std::memory_order_relaxed);
    gTargetPwm[i].store(0, std::memory_order_relaxed);
    char maxKey[8];
    char thrKey[8];
    snprintf(maxKey, sizeof(maxKey), "max%u", i);
    snprintf(thrKey, sizeof(thrKey), "thr%u", i);
    gMaxPwmValue[i].store(preferences.getUChar(maxKey, 255), std::memory_order_relaxed);
    gPumpThresholds[i].store(preferences.getUShort(thrKey, 2200), std::memory_order_relaxed);
  }
  for (uint8_t i = 0; i < kPumpCount; ++i) {
    ledcSetup(kPwmChannels[i], kPwmFrequencyHz, kPwmResolutionBits);
    ledcAttachPin(kPumpPins[i], kPwmChannels[i]);
    ledcWrite(kPwmChannels[i], 0);
  }

  gPumpQueue = xQueueCreate(16, sizeof(PumpCommand));
  gLedQueue = xQueueCreate(16, sizeof(LedEvent));
  gBusTxQueue = xQueueCreate(16, sizeof(BusTxMessage));
  gSensorSnapshotQueue = xQueueCreate(1, sizeof(SensorSnapshot));
  gPwmTargetQueue = xQueueCreate(16, sizeof(PwmTargetMessage));
  gPumpTimeoutTimer = xTimerCreate("PumpTimeout", pdMS_TO_TICKS(gPumpTimeoutMs.load(std::memory_order_relaxed)), pdFALSE, nullptr, pumpTimeoutCallback);

  xTaskCreatePinnedToCore(Task_UART_Comms, "Task_UART_Comms", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr, 0);
  xTaskCreatePinnedToCore(Task_Pump_Logic, "Task_Pump_Logic", 4096, nullptr, tskIDLE_PRIORITY + 3, nullptr, 1);
  xTaskCreatePinnedToCore(Task_PWM_Ramp, "Task_PWM_Ramp", 4096, nullptr, tskIDLE_PRIORITY + 3, nullptr, 1);
  xTaskCreatePinnedToCore(Task_WiFi, "Task_WiFi", 8192, nullptr, tskIDLE_PRIORITY + 2, nullptr, 0);
  xTaskCreatePinnedToCore(Task_HMI, "Task_HMI", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Temperatures, "Task_Temperatures", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Telemetry, "Task_Telemetry", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Bus_Supervisor, "Task_Bus_Supervisor", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
