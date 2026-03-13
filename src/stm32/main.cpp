#include <Arduino.h>
#include <PJONSoftwareBitBang.h>

#include <STM32FreeRTOS.h>

#include "protocol.h"

namespace {
constexpr uint8_t BUS_PIN = PA0;
constexpr uint8_t REED_PIN = PA1;
constexpr uint8_t MOISTURE_PIN_1 = PA2;
constexpr uint8_t MOISTURE_PIN_2 = PA3;
constexpr uint8_t LED_PIN = PC13;  // Black Pill onboard LED (active low)

constexpr TickType_t SENSOR_POLL_PERIOD = pdMS_TO_TICKS(2000);
constexpr TickType_t BUS_OK_AGE_TICKS = pdMS_TO_TICKS(5000);

struct LedEvent {
  TickType_t tx_tick;
};

QueueHandle_t gSensorQueue = nullptr;
QueueHandle_t gLedQueue = nullptr;
PJONSoftwareBitBang bus(SLAVE_ID);

void setLed(bool on) {
  digitalWrite(LED_PIN, on ? LOW : HIGH);
}

void receiverFunction(uint8_t *payload, uint16_t length, const PJON_Packet_Info &) {
  if (length == 0) {
    return;
  }

  CommandPacket cmd{};
  const uint16_t copyLen = min<uint16_t>(length, sizeof(cmd.text) - 1);
  memcpy(cmd.text, payload, copyLen);
  cmd.text[copyLen] = '\0';
  Serial.printf("[ESP32 CMD]: %s\n", cmd.text);
}

void Task_Sensors(void *) {
  SensorPacket packet{};
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    packet.uptime_ms = millis();
    packet.tank_full = (digitalRead(REED_PIN) == LOW) ? 1 : 0;
    packet.moisture_raw[0] = static_cast<uint16_t>(analogRead(MOISTURE_PIN_1));
    packet.moisture_raw[1] = static_cast<uint16_t>(analogRead(MOISTURE_PIN_2));

    xQueueOverwrite(gSensorQueue, &packet);
    vTaskDelayUntil(&lastWake, SENSOR_POLL_PERIOD);
  }
}

void Task_PJON(void *) {
  SensorPacket packet{};
  for (;;) {
    if (xQueueReceive(gSensorQueue, &packet, 0) == pdPASS) {
      bus.send(MASTER_ID, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
      LedEvent ev{};
      ev.tx_tick = xTaskGetTickCount();
      xQueueSend(gLedQueue, &ev, 0);
    }
    bus.receive(5);
    bus.update();
    vTaskDelay(1);
  }
}

void Task_Status_LED(void *) {
  TickType_t lastTxTick = 0;
  TickType_t lastBlinkTick = xTaskGetTickCount();
  bool ledState = false;
  setLed(false);

  for (;;) {
    LedEvent ev{};
    while (xQueueReceive(gLedQueue, &ev, 0) == pdPASS) {
      lastTxTick = ev.tx_tick;
    }

    const TickType_t now = xTaskGetTickCount();
    const bool busHealthy = (lastTxTick != 0) && ((now - lastTxTick) <= BUS_OK_AGE_TICKS);

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
}  // namespace

void setup() {
  Serial.begin(115200);

  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(MOISTURE_PIN_1, INPUT_ANALOG);
  pinMode(MOISTURE_PIN_2, INPUT_ANALOG);
  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  analogReadResolution(12);

  bus.strategy.set_pin(BUS_PIN);
  bus.set_receiver(receiverFunction);
  bus.begin();

  gSensorQueue = xQueueCreate(1, sizeof(SensorPacket));
  gLedQueue = xQueueCreate(8, sizeof(LedEvent));

  xTaskCreate(Task_Sensors, "Task_Sensors", 384, nullptr, tskIDLE_PRIORITY + 2, nullptr);
  xTaskCreate(Task_PJON, "Task_PJON", 512, nullptr, configMAX_PRIORITIES - 1, nullptr);
  xTaskCreate(Task_Status_LED, "Task_Status_LED", 256, nullptr, tskIDLE_PRIORITY + 1, nullptr);

  vTaskStartScheduler();
}

void loop() {}
