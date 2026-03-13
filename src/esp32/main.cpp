#include <Arduino.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <PJONSoftwareBitBang.h>

#include <atomic>
#include <cmath>

#include "protocol.h"

namespace {
constexpr uint8_t BUS_PIN = 13;
constexpr uint8_t PUMP_PIN_1 = 25;
constexpr uint8_t PUMP_PIN_2 = 26;
constexpr uint8_t PUMP_PIN_3 = 27;
constexpr uint8_t LED_PIN = 2;
constexpr uint8_t ONE_WIRE_PIN = 4;

constexpr uint8_t LED_ON = HIGH;
constexpr uint8_t LED_OFF = LOW;

constexpr TickType_t PUMP_TIMEOUT_TICKS = pdMS_TO_TICKS(10000);
constexpr TickType_t BUS_OK_AGE_TICKS = pdMS_TO_TICKS(5000);
constexpr TickType_t BUS_RECOVERY_TICKS = pdMS_TO_TICKS(3000);
constexpr TickType_t BUS_HARD_RESET_TICKS = pdMS_TO_TICKS(20000);
constexpr TickType_t BUS_REINIT_COOLDOWN_TICKS = pdMS_TO_TICKS(30000);
constexpr TickType_t BUS_COLDSTART_REINIT_TICKS = pdMS_TO_TICKS(8000);
constexpr TickType_t TELEMETRY_PERIOD_TICKS = pdMS_TO_TICKS(2000);
constexpr TickType_t DS18_SAMPLE_PERIOD_TICKS = pdMS_TO_TICKS(2000);
constexpr TickType_t BUS_PING_OFFLINE_TICKS = pdMS_TO_TICKS(3000);
constexpr TickType_t BUS_PING_ONLINE_TICKS = pdMS_TO_TICKS(15000);
constexpr TickType_t BUS_POLL_OFFLINE_TICKS = pdMS_TO_TICKS(1000);
constexpr TickType_t BUS_POLL_ONLINE_TICKS = pdMS_TO_TICKS(3000);

struct SensorSnapshot {
  PayloadSensor packet;
  TickType_t rx_tick;
  bool valid;
};

enum class PumpCommandType : uint8_t {
  SensorUpdate,
  Timeout,
  SetModeAuto,
  SetModeManual,
  SetManualMask,
  SetThreshold,
};

struct PumpCommand {
  PumpCommandType type;
  PayloadSensor packet;
  uint8_t manual_mask;
  uint16_t threshold_raw;
};

enum class LedEventType : uint8_t {
  RxPacket,
};

struct LedEvent {
  LedEventType type;
};

struct BusTxMessage {
  uint16_t len;
  uint8_t data[kMaxTextPayload];
};

QueueHandle_t gPumpQueue = nullptr;
QueueHandle_t gLedQueue = nullptr;
QueueHandle_t gBusTxQueue = nullptr;
QueueHandle_t gSensorSnapshotQueue = nullptr;
TimerHandle_t gPumpTimeoutTimer = nullptr;
PJONSoftwareBitBang bus(MASTER_ID);
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);

std::atomic<bool> telemetryActive{false};
std::atomic<float> gTemp0C{NAN};
std::atomic<float> gTemp1C{NAN};
std::atomic<uint8_t> gTempSensorCount{0};
std::atomic<uint8_t> gPumpMask{0};

volatile TickType_t gLastPacketTick = 0;
std::atomic<uint32_t> gDroppedBusTxMessages{0};
std::atomic<bool> gBusReinitRequested{false};
std::atomic<uint32_t> gBusReinitCount{0};
std::atomic<uint32_t> gPollRequestsSent{0};
std::atomic<TickType_t> gBootTick{0};

bool queueBusMessage(const uint8_t *data, uint16_t len, TickType_t timeoutTicks = pdMS_TO_TICKS(50)) {
  if (data == nullptr || len == 0 || len > kMaxTextPayload) {
    return false;
  }

  BusTxMessage out{};
  memcpy(out.data, data, len);
  out.len = len;

  if (xQueueSend(gBusTxQueue, &out, timeoutTicks) == pdPASS) {
    return true;
  }

  gDroppedBusTxMessages.fetch_add(1, std::memory_order_relaxed);
  return false;
}

void setPumps(bool p1_on, bool p2_on, bool p3_on) {
  digitalWrite(PUMP_PIN_1, p1_on ? HIGH : LOW);
  digitalWrite(PUMP_PIN_2, p2_on ? HIGH : LOW);
  digitalWrite(PUMP_PIN_3, p3_on ? HIGH : LOW);
  uint8_t mask = 0;
  if (p1_on) {
    mask |= 0x01u;
  }
  if (p2_on) {
    mask |= 0x02u;
  }
  if (p3_on) {
    mask |= 0x04u;
  }
  gPumpMask.store(mask, std::memory_order_relaxed);
}

void setLed(bool on) { digitalWrite(LED_PIN, on ? LED_ON : LED_OFF); }

void enqueueTextMessage(const String &msg) {
  uint8_t payload[kMaxTextPayload] = {0};
  payload[0] = static_cast<uint8_t>(PacketType::TextMessage);
  const String trimmed = msg.substring(0, kMaxTextPayload - 2);
  const size_t copied = trimmed.length();
  trimmed.toCharArray(reinterpret_cast<char *>(&payload[1]), kMaxTextPayload - 1);
  payload[copied + 1] = '\0';
  queueBusMessage(payload, static_cast<uint16_t>(copied + 2));
}


void enqueuePollRequest() {
  const uint8_t payload[1] = {static_cast<uint8_t>(PacketType::PollRequest)};
  if (queueBusMessage(payload, sizeof(payload), pdMS_TO_TICKS(10))) {
    gPollRequestsSent.fetch_add(1, std::memory_order_relaxed);
  }
}

void pumpTimeoutCallback(TimerHandle_t) {
  PumpCommand cmd{};
  cmd.type = PumpCommandType::Timeout;
  xQueueSend(gPumpQueue, &cmd, 0);
}

void receiverFunction(uint8_t *payload, uint16_t length, const PJON_Packet_Info &) {
  if (length < 1) {
    return;
  }

  LedEvent ev{};
  ev.type = LedEventType::RxPacket;
  xQueueSend(gLedQueue, &ev, 0);

  gLastPacketTick = xTaskGetTickCount();

  const PacketType type = static_cast<PacketType>(payload[0]);
  if (type == PacketType::SensorData && length >= sizeof(PayloadSensor)) {
    PumpCommand cmd{};
    cmd.type = PumpCommandType::SensorUpdate;
    memcpy(&cmd.packet, payload, sizeof(PayloadSensor));
    xQueueSend(gPumpQueue, &cmd, 0);

    SensorSnapshot snap{};
    memcpy(&snap.packet, payload, sizeof(PayloadSensor));
    snap.rx_tick = gLastPacketTick;
    snap.valid = true;
    xQueueOverwrite(gSensorSnapshotQueue, &snap);
    return;
  }

  if (type == PacketType::TextMessage && length > 1) {
    char text[kMaxTextPayload] = {0};
    const uint16_t copyLen = min<uint16_t>(length - 1, kMaxTextPayload - 1);
    memcpy(text, payload + 1, copyLen);
    text[copyLen] = '\0';
    Serial.printf("[STM32]: %s\n", text);
  }
}

void printHelp() {
  Serial.println(F("[HMI] commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  mode auto|manual"));
  Serial.println(F("  pump <1|2|3> <on|off>"));
  Serial.println(F("  pump all <on|off>"));
  Serial.println(F("  threshold <raw 0..4095>"));
  Serial.println(F("  timeout <sec>"));
  Serial.println(F("  cmd <text>"));
  Serial.println(F("  telemetry on|off"));
  Serial.println(F("  status"));
}

void printTelemetryBlock(const char *reason) {
  SensorSnapshot snap{};
  const bool haveSnap = xQueuePeek(gSensorSnapshotQueue, &snap, 0) == pdPASS && snap.valid;
  const TickType_t now = xTaskGetTickCount();
  const uint32_t packetAgeMs = (gLastPacketTick == 0) ? 0xFFFFFFFFu : static_cast<uint32_t>((now - gLastPacketTick) * portTICK_PERIOD_MS);
  const bool online = (gLastPacketTick != 0) && ((now - gLastPacketTick) <= BUS_OK_AGE_TICKS);

  Serial.printf("[TELEMETRY][%s] {heap:%u, slave:%s, packet_age_ms:%lu, dropped_bus_tx:%lu, bus_reinits:%lu, poll_tx:%lu, boot_ms:%lu, ds18_count:%u, ds18_0:%.2f, ds18_1:%.2f, adc_pa2:%u, adc_pa3:%u, tank_full:%u, pumps_mask:0x%02X}\n",
                reason,
                static_cast<unsigned>(ESP.getFreeHeap()),
                online ? "online" : "offline",
                static_cast<unsigned long>(packetAgeMs),
                static_cast<unsigned long>(gDroppedBusTxMessages.load(std::memory_order_relaxed)),
                static_cast<unsigned long>(gBusReinitCount.load(std::memory_order_relaxed)),
                static_cast<unsigned long>(gPollRequestsSent.load(std::memory_order_relaxed)),
                static_cast<unsigned long>((now - gBootTick.load(std::memory_order_relaxed)) * portTICK_PERIOD_MS),
                static_cast<unsigned>(gTempSensorCount.load(std::memory_order_relaxed)),
                gTemp0C.load(std::memory_order_relaxed),
                gTemp1C.load(std::memory_order_relaxed),
                haveSnap ? snap.packet.moisture_raw[0] : 0,
                haveSnap ? snap.packet.moisture_raw[1] : 0,
                haveSnap ? snap.packet.tank_full : 0,
                gPumpMask.load(std::memory_order_relaxed));
}

void Task_Bus_Supervisor(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  TickType_t lastStreamReqTick = 0;
  TickType_t lastPingTick = 0;
  TickType_t lastPollTick = 0;
  TickType_t lastHardResetTick = 0;
  uint32_t pingCounter = 0;

  enqueueTextMessage("stream on");
  enqueuePollRequest();

  for (;;) {
    const TickType_t now = xTaskGetTickCount();
    const TickType_t lastPacket = gLastPacketTick;
    const bool online = (lastPacket != 0) && ((now - lastPacket) <= BUS_OK_AGE_TICKS);

    if (!online && (lastStreamReqTick == 0 || (now - lastStreamReqTick) >= BUS_RECOVERY_TICKS)) {
      enqueueTextMessage("stream on");
      lastStreamReqTick = now;
    }

    const TickType_t pollPeriod = online ? BUS_POLL_ONLINE_TICKS : BUS_POLL_OFFLINE_TICKS;
    if (lastPollTick == 0 || (now - lastPollTick) >= pollPeriod) {
      enqueuePollRequest();
      lastPollTick = now;
    }

    const TickType_t pingPeriod = online ? BUS_PING_ONLINE_TICKS : BUS_PING_OFFLINE_TICKS;
    if (lastPingTick == 0 || (now - lastPingTick) >= pingPeriod) {
      enqueueTextMessage(String("ping ") + String(pingCounter++));
      lastPingTick = now;
    }

    const bool neverReceivedPacket = (lastPacket == 0);
    const bool staleLink = (!online && !neverReceivedPacket && (now - lastPacket) >= BUS_HARD_RESET_TICKS);
    const bool coldStartTimeout = (neverReceivedPacket && (now - gBootTick.load(std::memory_order_relaxed)) >= BUS_COLDSTART_REINIT_TICKS);
    if (staleLink || coldStartTimeout) {
      if ((lastHardResetTick == 0) || ((now - lastHardResetTick) >= BUS_REINIT_COOLDOWN_TICKS)) {
        Serial.println(F("[BUS] scheduling hard reset of PJON interface"));
        gBusReinitRequested.store(true, std::memory_order_relaxed);
        lastHardResetTick = now;
      }
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(250));
  }
}

void printBootManifest() {
  ds18b20.begin();
  ds18b20.requestTemperatures();
  const uint8_t dsCount = ds18b20.getDS18Count();
  Serial.println(F("========== BOOT MANIFEST =========="));
  Serial.printf("PJON master id: %u, slave id: %u, bus pin: %u\n", MASTER_ID, SLAVE_ID, BUS_PIN);
  Serial.printf("Pins: LED=%u, OneWire=%u, Pumps=[%u,%u,%u]\n", LED_PIN, ONE_WIRE_PIN, PUMP_PIN_1, PUMP_PIN_2, PUMP_PIN_3);
  Serial.printf("OneWire DS18B20 found: %u\n", dsCount);
  Serial.printf("Free heap: %u bytes\n", static_cast<unsigned>(ESP.getFreeHeap()));
  Serial.println(F("==================================="));
}

void Task_PJON_Comms(void *) {
  BusTxMessage outbound{};

  for (;;) {
    while (xQueueReceive(gBusTxQueue, &outbound, 0) == pdPASS) {
      if (outbound.len > 0) {
        bus.send(SLAVE_ID, outbound.data, outbound.len);
      }
    }

    if (gBusReinitRequested.exchange(false, std::memory_order_relaxed)) {
      bus.begin();
      gBusReinitCount.fetch_add(1, std::memory_order_relaxed);
      Serial.println(F("[BUS] PJON interface restarted"));
    }

    bus.receive(20);
    bus.update();
    vTaskDelay(1);
  }
}

void Task_Temperatures(void *) {
  for (;;) {
    ds18b20.requestTemperatures();
    const uint8_t count = ds18b20.getDS18Count();
    gTempSensorCount.store(count, std::memory_order_relaxed);

    float t0 = NAN;
    float t1 = NAN;
    if (count > 0) {
      const float temp = ds18b20.getTempCByIndex(0);
      if (temp != -127.0f && temp != 85.0f && temp > -126.0f && temp < 126.0f) {
        t0 = temp;
      }
    }
    if (count > 1) {
      const float temp = ds18b20.getTempCByIndex(1);
      if (temp != -127.0f && temp != 85.0f && temp > -126.0f && temp < 126.0f) {
        t1 = temp;
      }
    }
    gTemp0C.store(t0, std::memory_order_relaxed);
    gTemp1C.store(t1, std::memory_order_relaxed);

    vTaskDelay(DS18_SAMPLE_PERIOD_TICKS);
  }
}

void Task_Telemetry(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (telemetryActive.load(std::memory_order_relaxed)) {
      printTelemetryBlock("periodic");
    }
    vTaskDelayUntil(&lastWake, TELEMETRY_PERIOD_TICKS);
  }
}

void Task_Status_LED(void *) {
  TickType_t lastRxTick = 0;
  TickType_t lastBlinkTick = xTaskGetTickCount();
  bool ledState = false;
  setLed(false);

  for (;;) {
    LedEvent ev{};
    while (xQueueReceive(gLedQueue, &ev, 0) == pdPASS) {
      if (ev.type == LedEventType::RxPacket) {
        lastRxTick = xTaskGetTickCount();
      }
    }

    const TickType_t now = xTaskGetTickCount();
    const bool busHealthy = (lastRxTick != 0) && ((now - lastRxTick) <= BUS_OK_AGE_TICKS);

    if (busHealthy) {
      if (!ledState) {
        ledState = true;
        setLed(true);
      }
    } else if ((now - lastBlinkTick) >= pdMS_TO_TICKS(500)) {
      lastBlinkTick = now;
      ledState = !ledState;
      setLed(ledState);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
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
        Serial.println(F("[HMI] mode=auto"));
        continue;
      }

      if (cmd.type == PumpCommandType::SetModeManual) {
        autoMode = false;
        manualMask = 0;
        setPumps(false, false, false);
        xTimerStop(gPumpTimeoutTimer, 0);
        Serial.println(F("[HMI] mode=manual"));
        continue;
      }

      if (cmd.type == PumpCommandType::SetThreshold) {
        thresholdRaw = cmd.threshold_raw;
        Serial.printf("[HMI] threshold=%u\n", thresholdRaw);
        continue;
      }

      if (cmd.type == PumpCommandType::SetManualMask) {
        manualMask = cmd.manual_mask & 0x07u;
        if (!autoMode) {
          const bool p1 = (manualMask & 0x01u) != 0;
          const bool p2 = (manualMask & 0x02u) != 0;
          const bool p3 = (manualMask & 0x04u) != 0;
          setPumps(p1, p2, p3);
          if (manualMask != 0) {
            xTimerReset(gPumpTimeoutTimer, 0);
          } else {
            xTimerStop(gPumpTimeoutTimer, 0);
          }
        }
        Serial.printf("[HMI] manual_mask=0x%02X\n", manualMask);
        continue;
      }

      if (cmd.type == PumpCommandType::Timeout) {
        setPumps(false, false, false);
        if (!autoMode) {
          manualMask = 0;
        }
        Serial.println(F("[SAFE] Pump timeout -> all OFF"));
        continue;
      }

      if (cmd.type == PumpCommandType::SensorUpdate) {
        lastSensorRxTick = xTaskGetTickCount();
        if (!autoMode) {
          continue;
        }

        const PayloadSensor &p = cmd.packet;
        if (p.tank_full == 0) {
          setPumps(false, false, false);
          xTimerStop(gPumpTimeoutTimer, 0);
          continue;
        }

        const bool pump1 = p.moisture_raw[0] < thresholdRaw;
        const bool pump3 = p.moisture_raw[1] < thresholdRaw;

        setPumps(pump1, false, pump3);

        if (pump1 || pump3) {
          xTimerReset(gPumpTimeoutTimer, 0);
        } else {
          xTimerStop(gPumpTimeoutTimer, 0);
        }
      }
    }

    const TickType_t now = xTaskGetTickCount();
    if (lastSensorRxTick != 0 && (now - lastSensorRxTick) > BUS_OK_AGE_TICKS) {
      lastSensorRxTick = 0;
      setPumps(false, false, false);
      xTimerStop(gPumpTimeoutTimer, 0);
      Serial.println(F("[SAFE] STM32 offline -> pumps forced OFF"));
    }
  }
}

void Task_HMI(void *) {
  String line;
  line.reserve(128);

  Serial.println(F("[HMI] online. type 'help'"));

  for (;;) {
    while (Serial.available() > 0) {
      const char c = static_cast<char>(Serial.read());
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        line.trim();
        if (line.length() == 0) {
          continue;
        }

        PumpCommand cmd{};

        if (line.equalsIgnoreCase("help")) {
          printHelp();
        } else if (line.equalsIgnoreCase("status")) {
          printTelemetryBlock("status");
        } else if (line.equalsIgnoreCase("telemetry on")) {
          telemetryActive.store(true, std::memory_order_relaxed);
          Serial.println(F("[HMI] telemetry=on"));
        } else if (line.equalsIgnoreCase("telemetry off")) {
          telemetryActive.store(false, std::memory_order_relaxed);
          Serial.println(F("[HMI] telemetry=off"));
        } else if (line.equalsIgnoreCase("mode auto")) {
          cmd.type = PumpCommandType::SetModeAuto;
          xQueueSend(gPumpQueue, &cmd, pdMS_TO_TICKS(50));
        } else if (line.equalsIgnoreCase("mode manual")) {
          cmd.type = PumpCommandType::SetModeManual;
          xQueueSend(gPumpQueue, &cmd, pdMS_TO_TICKS(50));
        } else if (line.equalsIgnoreCase("pump all on")) {
          cmd.type = PumpCommandType::SetManualMask;
          cmd.manual_mask = 0x07u;
          xQueueSend(gPumpQueue, &cmd, pdMS_TO_TICKS(50));
        } else if (line.equalsIgnoreCase("pump all off")) {
          cmd.type = PumpCommandType::SetManualMask;
          cmd.manual_mask = 0x00u;
          xQueueSend(gPumpQueue, &cmd, pdMS_TO_TICKS(50));
        } else if (line.startsWith("pump ")) {
          int idx = 0;
          char state[8] = {0};
          if (sscanf(line.c_str(), "pump %d %7s", &idx, state) == 2 && idx >= 1 && idx <= 3) {
            static uint8_t currentMask = 0;
            const bool on = (strcasecmp(state, "on") == 0);
            const bool off = (strcasecmp(state, "off") == 0);
            if (on || off) {
              const uint8_t bit = static_cast<uint8_t>(1u << (idx - 1));
              currentMask = on ? static_cast<uint8_t>(currentMask | bit) : static_cast<uint8_t>(currentMask & ~bit);
              cmd.type = PumpCommandType::SetManualMask;
              cmd.manual_mask = currentMask;
              xQueueSend(gPumpQueue, &cmd, pdMS_TO_TICKS(50));
            }
          }
        } else if (line.startsWith("threshold ")) {
          uint32_t threshold = 0;
          if (sscanf(line.c_str(), "threshold %lu", &threshold) == 1 && threshold <= 4095) {
            cmd.type = PumpCommandType::SetThreshold;
            cmd.threshold_raw = static_cast<uint16_t>(threshold);
            xQueueSend(gPumpQueue, &cmd, pdMS_TO_TICKS(50));
          }
        } else if (line.startsWith("timeout ")) {
          uint32_t timeoutSec = 0;
          if (sscanf(line.c_str(), "timeout %lu", &timeoutSec) == 1 && timeoutSec >= 1 && timeoutSec <= 600) {
            xTimerChangePeriod(gPumpTimeoutTimer, pdMS_TO_TICKS(timeoutSec * 1000UL), pdMS_TO_TICKS(100));
            Serial.printf("[HMI] timeout=%lus\n", static_cast<unsigned long>(timeoutSec));
          }
        } else if (line.startsWith("cmd ")) {
          String command = line.substring(4);
          command.trim();
          if (!command.isEmpty()) {
            enqueueTextMessage(command);
            Serial.printf("[HMI] cmd->STM32: %s\n", command.c_str());
          }
        } else if (line.startsWith("ping ")) {
          String msg = String("ping ") + line.substring(5);
          msg.trim();
          enqueueTextMessage(msg);
        } else {
          Serial.println(F("[HMI] unknown command"));
        }

        line.remove(0);
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
  gBootTick.store(xTaskGetTickCount(), std::memory_order_relaxed);

  pinMode(PUMP_PIN_1, OUTPUT);
  pinMode(PUMP_PIN_2, OUTPUT);
  pinMode(PUMP_PIN_3, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  setPumps(false, false, false);
  setLed(false);

  printBootManifest();

  gPumpQueue = xQueueCreate(16, sizeof(PumpCommand));
  gLedQueue = xQueueCreate(16, sizeof(LedEvent));
  gBusTxQueue = xQueueCreate(8, sizeof(BusTxMessage));
  gSensorSnapshotQueue = xQueueCreate(1, sizeof(SensorSnapshot));
  gPumpTimeoutTimer = xTimerCreate("PumpTimeout", PUMP_TIMEOUT_TICKS, pdFALSE, nullptr, pumpTimeoutCallback);

  bus.strategy.set_pin(BUS_PIN);
  bus.set_receiver(receiverFunction);
  bus.begin();

  xTaskCreatePinnedToCore(Task_PJON_Comms, "Task_PJON_Comms", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr, 0);
  xTaskCreatePinnedToCore(Task_Pump_Logic, "Task_Pump_Logic", 4096, nullptr, tskIDLE_PRIORITY + 3, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Status_LED, "Task_Status_LED", 2048, nullptr, tskIDLE_PRIORITY + 2, nullptr, 1);
  xTaskCreatePinnedToCore(Task_HMI, "Task_HMI", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Temperatures, "Task_Temperatures", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Telemetry, "Task_Telemetry", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Bus_Supervisor, "Task_Bus_Supervisor", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
