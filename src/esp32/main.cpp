#include <Arduino.h>
#include <PJONSoftwareBitBang.h>

#include "protocol.h"

namespace {
constexpr uint8_t BUS_PIN = 16;
constexpr uint8_t PUMP_PIN_1 = 25;
constexpr uint8_t PUMP_PIN_2 = 26;
constexpr uint8_t PUMP_PIN_3 = 27;
constexpr TickType_t PUMP_TIMEOUT_TICKS = pdMS_TO_TICKS(10000);

enum class PumpCommandType : uint8_t {
  SensorUpdate,
  Timeout
};

struct PumpCommand {
  PumpCommandType type;
  SensorPacket packet;
};

QueueHandle_t gPumpQueue = nullptr;
TimerHandle_t gPumpTimeoutTimer = nullptr;
PJONSoftwareBitBang bus(MASTER_ID);

void setPumps(bool p1_on, bool p2_on, bool p3_on) {
  digitalWrite(PUMP_PIN_1, p1_on ? LOW : HIGH);
  digitalWrite(PUMP_PIN_2, p2_on ? LOW : HIGH);
  digitalWrite(PUMP_PIN_3, p3_on ? LOW : HIGH);
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
}

void Task_PJON_Comms(void *) {
  for (;;) {
    bus.receive(1000);
    bus.update();
    taskYIELD();
  }
}

void Task_Pump_Logic(void *) {
  PumpCommand cmd{};

  for (;;) {
    if (xQueueReceive(gPumpQueue, &cmd, portMAX_DELAY) != pdPASS) {
      continue;
    }

    if (cmd.type == PumpCommandType::Timeout) {
      setPumps(false, false, false);
      continue;
    }

    const SensorPacket &p = cmd.packet;
    if (p.tank_full == 0) {
      setPumps(false, false, false);
      xTimerStop(gPumpTimeoutTimer, 0);
      continue;
    }

    const bool t0_valid = (p.valid_mask & 0x01u) != 0u;
    const bool t1_valid = (p.valid_mask & 0x02u) != 0u;
    const bool pump1 = t0_valid && p.temp_c[0] > 28.0f;
    const bool pump2 = t1_valid && p.temp_c[1] > 28.0f;
    const bool pump3 = pump1 || pump2;

    setPumps(pump1, pump2, pump3);

    if (pump1 || pump2 || pump3) {
      xTimerReset(gPumpTimeoutTimer, 0);
    } else {
      xTimerStop(gPumpTimeoutTimer, 0);
    }
  }
}
}  // namespace

void setup() {
  pinMode(PUMP_PIN_1, OUTPUT);
  pinMode(PUMP_PIN_2, OUTPUT);
  pinMode(PUMP_PIN_3, OUTPUT);
  setPumps(false, false, false);

  gPumpQueue = xQueueCreate(8, sizeof(PumpCommand));
  gPumpTimeoutTimer = xTimerCreate("PumpTimeout", PUMP_TIMEOUT_TICKS, pdFALSE, nullptr, pumpTimeoutCallback);

  bus.strategy.set_pin(BUS_PIN);
  bus.set_receiver(receiverFunction);
  bus.begin();

  xTaskCreatePinnedToCore(Task_PJON_Comms, "Task_PJON_Comms", 4096, nullptr, configMAX_PRIORITIES - 1, nullptr, 0);
  xTaskCreatePinnedToCore(Task_Pump_Logic, "Task_Pump_Logic", 4096, nullptr, tskIDLE_PRIORITY + 2, nullptr, 1);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
