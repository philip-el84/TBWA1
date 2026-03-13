#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include <atomic>
#include <cmath>

#include "protocol.h"

namespace {
constexpr int UART_PIN = 13;
constexpr uint8_t PUMP_PIN_1 = 25;
constexpr uint8_t PUMP_PIN_2 = 26;
constexpr uint8_t PUMP_PIN_3 = 27;
constexpr uint8_t LED_PIN = 2;
constexpr uint8_t ONE_WIRE_PIN = 4;

constexpr TickType_t PUMP_TIMEOUT_TICKS = pdMS_TO_TICKS(10000);
constexpr TickType_t BUS_OK_AGE_TICKS = pdMS_TO_TICKS(5000);
constexpr TickType_t TELEMETRY_PERIOD_TICKS = pdMS_TO_TICKS(2000);
constexpr TickType_t DS18_SAMPLE_PERIOD_TICKS = pdMS_TO_TICKS(2000);
constexpr TickType_t BUS_POLL_TICKS = pdMS_TO_TICKS(2000);

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
struct LedEvent { LedEventType type; };

struct BusTxMessage {
  PacketType type;
  uint8_t len;
  uint8_t payload[kMaxPayloadSize];
};

QueueHandle_t gPumpQueue = nullptr;
QueueHandle_t gLedQueue = nullptr;
QueueHandle_t gBusTxQueue = nullptr;
QueueHandle_t gSensorSnapshotQueue = nullptr;
TimerHandle_t gPumpTimeoutTimer = nullptr;

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);
FrameParser gParser;
std::atomic<bool> telemetryActive{false};
std::atomic<float> gTemp0C{NAN}, gTemp1C{NAN}, gTemp2C{NAN};
std::atomic<uint8_t> gPumpMask{0};
volatile TickType_t gLastPacketTick = 0;

void setPumps(bool p1_on, bool p2_on, bool p3_on) {
  digitalWrite(PUMP_PIN_1, p1_on ? HIGH : LOW);
  digitalWrite(PUMP_PIN_2, p2_on ? HIGH : LOW);
  digitalWrite(PUMP_PIN_3, p3_on ? HIGH : LOW);
  uint8_t mask = (p1_on ? 0x01u : 0u) | (p2_on ? 0x02u : 0u) | (p3_on ? 0x04u : 0u);
  gPumpMask.store(mask, std::memory_order_relaxed);
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
  bool autoMode = true;
  uint8_t manualMask = 0;
  uint16_t thresholdRaw = 2200;
  TickType_t lastSensorRxTick = 0;

  for (;;) {
    if (xQueueReceive(gPumpQueue, &cmd, pdMS_TO_TICKS(200)) == pdPASS) {
      if (cmd.type == PumpCommandType::SetModeAuto) {
        autoMode = true;
        manualMask = 0;
        setPumps(false, false, false);
        xTimerStop(gPumpTimeoutTimer, 0);
        continue;
      }
      if (cmd.type == PumpCommandType::SetModeManual) {
        autoMode = false;
        manualMask = 0;
        setPumps(false, false, false);
        xTimerStop(gPumpTimeoutTimer, 0);
        continue;
      }
      if (cmd.type == PumpCommandType::SetThreshold) {
        thresholdRaw = cmd.threshold_raw;
        continue;
      }
      if (cmd.type == PumpCommandType::SetManualMask) {
        manualMask = cmd.manual_mask & 0x07u;
        if (!autoMode) {
          setPumps(manualMask & 0x01u, manualMask & 0x02u, manualMask & 0x04u);
          if (manualMask != 0) xTimerReset(gPumpTimeoutTimer, 0); else xTimerStop(gPumpTimeoutTimer, 0);
        }
        continue;
      }
      if (cmd.type == PumpCommandType::Timeout) {
        setPumps(false, false, false);
        continue;
      }
      if (cmd.type == PumpCommandType::SensorUpdate) {
        lastSensorRxTick = xTaskGetTickCount();
        if (!autoMode) continue;
        const auto &p = cmd.packet;
        if (p.tank_full == 0) {
          setPumps(false, false, false);
          continue;
        }
        const bool pump1 = p.hw390_raw[0] < thresholdRaw;
        const bool pump3 = p.hw390_raw[1] < thresholdRaw;
        setPumps(pump1, false, pump3);
        if (pump1 || pump3) xTimerReset(gPumpTimeoutTimer, 0); else xTimerStop(gPumpTimeoutTimer, 0);
      }
    }

    if (lastSensorRxTick != 0 && (xTaskGetTickCount() - lastSensorRxTick) > BUS_OK_AGE_TICKS) {
      lastSensorRxTick = 0;
      setPumps(false, false, false);
      xTimerStop(gPumpTimeoutTimer, 0);
    }
  }
}

void Task_Telemetry(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (telemetryActive.load(std::memory_order_relaxed)) {
      SensorSnapshot snap{};
      const bool haveSnap = xQueuePeek(gSensorSnapshotQueue, &snap, 0) == pdPASS && snap.valid;
      Serial.printf("[TEL] online=%u hw390=[%u,%u,%u,%u,%u] ldr=[%u,%u] ds18=[%.2f,%.2f,%.2f] mask=0x%02X\n",
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
                    gPumpMask.load(std::memory_order_relaxed));
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
        if (line.equalsIgnoreCase("telemetry on")) telemetryActive.store(true, std::memory_order_relaxed);
        else if (line.equalsIgnoreCase("telemetry off")) telemetryActive.store(false, std::memory_order_relaxed);
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
}  // namespace

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, UART_PIN, UART_PIN);

  pinMode(PUMP_PIN_1, OUTPUT);
  pinMode(PUMP_PIN_2, OUTPUT);
  pinMode(PUMP_PIN_3, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  setPumps(false, false, false);

  gPumpQueue = xQueueCreate(16, sizeof(PumpCommand));
  gLedQueue = xQueueCreate(16, sizeof(LedEvent));
  gBusTxQueue = xQueueCreate(16, sizeof(BusTxMessage));
  gSensorSnapshotQueue = xQueueCreate(1, sizeof(SensorSnapshot));
  gPumpTimeoutTimer = xTimerCreate("PumpTimeout", PUMP_TIMEOUT_TICKS, pdFALSE, nullptr, pumpTimeoutCallback);

  xTaskCreatePinnedToCore(Task_UART_Comms, "Task_UART_Comms", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr, 0);
  xTaskCreatePinnedToCore(Task_Pump_Logic, "Task_Pump_Logic", 4096, nullptr, tskIDLE_PRIORITY + 3, nullptr, 1);
  xTaskCreatePinnedToCore(Task_HMI, "Task_HMI", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Temperatures, "Task_Temperatures", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Telemetry, "Task_Telemetry", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Bus_Supervisor, "Task_Bus_Supervisor", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
