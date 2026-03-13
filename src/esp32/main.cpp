#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PJONSoftwareBitBang.h>

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
constexpr TickType_t TEMP_POLL_PERIOD = pdMS_TO_TICKS(10000);

enum class PumpCommandType : uint8_t {
  SensorUpdate,
  Timeout,
  SetModeAuto,
  SetModeManual,
  SetManualMask,
  SetThreshold
};

struct PumpCommand {
  PumpCommandType type;
  SensorPacket packet;
  uint8_t manual_mask;
  uint16_t threshold_raw;
};

enum class LedEventType : uint8_t {
  RxPacket
};

struct LedEvent {
  LedEventType type;
};

QueueHandle_t gPumpQueue = nullptr;
QueueHandle_t gLedQueue = nullptr;
QueueHandle_t gBusTxQueue = nullptr;
TimerHandle_t gPumpTimeoutTimer = nullptr;
PJONSoftwareBitBang bus(MASTER_ID);
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);

void setPumps(bool p1_on, bool p2_on, bool p3_on) {
  digitalWrite(PUMP_PIN_1, p1_on ? HIGH : LOW);
  digitalWrite(PUMP_PIN_2, p2_on ? HIGH : LOW);
  digitalWrite(PUMP_PIN_3, p3_on ? HIGH : LOW);
}

void setLed(bool on) {
  digitalWrite(LED_PIN, on ? LED_ON : LED_OFF);
}

void pumpTimeoutCallback(TimerHandle_t) {
  PumpCommand cmd{};
  cmd.type = PumpCommandType::Timeout;
  xQueueSend(gPumpQueue, &cmd, 0);
}

void receiverFunction(uint8_t *payload, uint16_t length, const PJON_Packet_Info &) {
  if (length != sizeof(SensorPacket)) {
    return;
  }

  PumpCommand cmd{};
  cmd.type = PumpCommandType::SensorUpdate;
  memcpy(&cmd.packet, payload, sizeof(SensorPacket));
  xQueueSend(gPumpQueue, &cmd, 0);

  LedEvent ev{};
  ev.type = LedEventType::RxPacket;
  xQueueSend(gLedQueue, &ev, 0);
}

void printHelp() {
  Serial.println(F("[HMI] commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  mode auto"));
  Serial.println(F("  mode manual"));
  Serial.println(F("  pump <1|2|3> <on|off>"));
  Serial.println(F("  pump all <on|off>"));
  Serial.println(F("  threshold <raw 0..4095>"));
  Serial.println(F("  timeout <sec>"));
  Serial.println(F("  ping <text>"));
}

void Task_PJON_Comms(void *) {
  CommandPacket outbound{};

  for (;;) {
    while (xQueueReceive(gBusTxQueue, &outbound, 0) == pdPASS) {
      bus.send(SLAVE_ID, reinterpret_cast<const uint8_t *>(outbound.text), strlen(outbound.text));
    }

    bus.receive(50);
    bus.update();
    vTaskDelay(1);
  }
}

void Task_Temperatures(void *) {
  ds18b20.begin();

  for (;;) {
    ds18b20.requestTemperatures();
    const uint8_t count = ds18b20.getDS18Count();
    for (uint8_t i = 0; i < count; ++i) {
      const float temp = ds18b20.getTempCByIndex(i);
      if (temp > -126.0f && temp < 126.0f) {
        Serial.printf("[TEMP] DS18B20[%u]=%.2fC\n", i, temp);
      } else {
        Serial.printf("[TEMP] DS18B20[%u]=invalid\n", i);
      }
    }
    vTaskDelay(TEMP_POLL_PERIOD);
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

  for (;;) {
    if (xQueueReceive(gPumpQueue, &cmd, portMAX_DELAY) != pdPASS) {
      continue;
    }

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

    if (cmd.type != PumpCommandType::SensorUpdate || !autoMode) {
      continue;
    }

    const SensorPacket &p = cmd.packet;
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

void Task_HMI(void *) {
  String line;
  line.reserve(96);

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
            } else {
              Serial.println(F("[HMI] invalid state, use on/off"));
            }
          } else {
            Serial.println(F("[HMI] invalid pump command"));
          }
        } else if (line.startsWith("threshold ")) {
          uint32_t threshold = 0;
          if (sscanf(line.c_str(), "threshold %lu", &threshold) == 1 && threshold <= 4095) {
            cmd.type = PumpCommandType::SetThreshold;
            cmd.threshold_raw = static_cast<uint16_t>(threshold);
            xQueueSend(gPumpQueue, &cmd, pdMS_TO_TICKS(50));
          } else {
            Serial.println(F("[HMI] threshold range 0..4095"));
          }
        } else if (line.startsWith("timeout ")) {
          uint32_t timeoutSec = 0;
          if (sscanf(line.c_str(), "timeout %lu", &timeoutSec) == 1 && timeoutSec >= 1 && timeoutSec <= 600) {
            xTimerChangePeriod(gPumpTimeoutTimer, pdMS_TO_TICKS(timeoutSec * 1000UL), pdMS_TO_TICKS(100));
            Serial.printf("[HMI] timeout=%lus\n", static_cast<unsigned long>(timeoutSec));
          } else {
            Serial.println(F("[HMI] timeout range 1..600"));
          }
        } else if (line.startsWith("ping ")) {
          String msg = line.substring(5);
          msg.trim();
          if (msg.length() == 0) {
            Serial.println(F("[HMI] ping text missing"));
          } else {
            CommandPacket out{};
            msg.toCharArray(out.text, sizeof(out.text));
            xQueueSend(gBusTxQueue, &out, pdMS_TO_TICKS(50));
            Serial.printf("[HMI] ping -> '%s'\n", out.text);
          }
        } else {
          Serial.println(F("[HMI] unknown command"));
        }

        line.remove(0);
      } else if (line.length() < 95) {
        line += c;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
}  // namespace

void setup() {
  Serial.begin(115200);

  pinMode(PUMP_PIN_1, OUTPUT);
  pinMode(PUMP_PIN_2, OUTPUT);
  pinMode(PUMP_PIN_3, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  setPumps(false, false, false);
  setLed(false);

  gPumpQueue = xQueueCreate(16, sizeof(PumpCommand));
  gLedQueue = xQueueCreate(16, sizeof(LedEvent));
  gBusTxQueue = xQueueCreate(8, sizeof(CommandPacket));
  gPumpTimeoutTimer = xTimerCreate("PumpTimeout", PUMP_TIMEOUT_TICKS, pdFALSE, nullptr, pumpTimeoutCallback);

  bus.strategy.set_pin(BUS_PIN);
  bus.set_receiver(receiverFunction);
  bus.begin();

  xTaskCreatePinnedToCore(Task_PJON_Comms, "Task_PJON_Comms", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr, 0);
  xTaskCreatePinnedToCore(Task_Pump_Logic, "Task_Pump_Logic", 4096, nullptr, tskIDLE_PRIORITY + 3, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Status_LED, "Task_Status_LED", 2048, nullptr, tskIDLE_PRIORITY + 2, nullptr, 1);
  xTaskCreatePinnedToCore(Task_HMI, "Task_HMI", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
  xTaskCreatePinnedToCore(Task_Temperatures, "Task_Temperatures", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
