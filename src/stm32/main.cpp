#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PJONSoftwareBitBang.h>

#include <STM32FreeRTOS.h>

#include "protocol.h"

namespace {
constexpr uint8_t BUS_PIN = PB12;
constexpr uint8_t REED_PIN = PB13;
constexpr uint8_t ONE_WIRE_PIN = PB14;

constexpr TickType_t SENSOR_POLL_PERIOD = pdMS_TO_TICKS(2000);
constexpr TickType_t TEMP_POLL_PERIOD = pdMS_TO_TICKS(10000);

QueueHandle_t gSensorQueue = nullptr;

OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature ds18b20(&oneWire);
PJONSoftwareBitBang bus(SLAVE_ID);

void Task_Sensors(void *);
void Task_PJON(void *);

void Task_Sensors(void *) {
  SensorPacket packet{};
  TickType_t lastWake = xTaskGetTickCount();
  TickType_t lastTempRead = 0;

  ds18b20.begin();

  for (;;) {
    packet.uptime_ms = millis();
    packet.tank_full = (digitalRead(REED_PIN) == LOW) ? 1 : 0;

    const TickType_t now = xTaskGetTickCount();
    if ((now - lastTempRead) >= TEMP_POLL_PERIOD || lastTempRead == 0) {
      packet.valid_mask = 0;
      ds18b20.requestTemperatures();
      for (uint8_t i = 0; i < 2; ++i) {
        const float temp = ds18b20.getTempCByIndex(i);
        if (temp > -126.0f && temp < 126.0f) {
          packet.temp_c[i] = temp;
          packet.valid_mask |= (1u << i);
        } else {
          packet.temp_c[i] = NAN;
        }
      }
      lastTempRead = now;
    }

    xQueueOverwrite(gSensorQueue, &packet);
    vTaskDelayUntil(&lastWake, SENSOR_POLL_PERIOD);
  }
}

void Task_PJON(void *) {
  SensorPacket packet{};
  for (;;) {
    if (xQueueReceive(gSensorQueue, &packet, 0) == pdPASS) {
      bus.send(MASTER_ID, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    }
    bus.update();
    taskYIELD();
  }
}
}  // namespace

void setup() {
  pinMode(REED_PIN, INPUT_PULLUP);

  bus.strategy.set_pin(BUS_PIN);
  bus.begin();

  gSensorQueue = xQueueCreate(1, sizeof(SensorPacket));

  xTaskCreate(Task_Sensors, "Task_Sensors", 384, nullptr, tskIDLE_PRIORITY + 2, nullptr);
  xTaskCreate(Task_PJON, "Task_PJON", 512, nullptr, configMAX_PRIORITIES - 1, nullptr);

  vTaskStartScheduler();
}

void loop() {}
