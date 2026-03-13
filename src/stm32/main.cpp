#include <Arduino.h>
#include <STM32FreeRTOS.h>

#include <atomic>
#include <cstring>

#include "protocol.h"

namespace {
constexpr uint8_t UART_PIN = PA9;
constexpr uint8_t REED_PIN = PA1;
constexpr uint8_t HW390_PINS[5] = {PA2, PA3, PA4, PA5, PA6};
constexpr uint8_t LDR_PINS[2] = {PB0, PB1};
constexpr uint8_t LED_PIN = PC13;

constexpr TickType_t SENSOR_POLL_NORMAL = pdMS_TO_TICKS(2000);
constexpr TickType_t SENSOR_POLL_STREAM = pdMS_TO_TICKS(250);
constexpr TickType_t BUS_OK_AGE_TICKS = pdMS_TO_TICKS(5000);
constexpr TickType_t HEARTBEAT_PERIOD_TICKS = pdMS_TO_TICKS(3000);
constexpr TickType_t ANNOUNCE_SENSOR_TICKS = pdMS_TO_TICKS(1000);

struct LedEvent {
  enum Type : uint8_t { RxPacket, BlinkRequest } type;
  uint16_t blink_count;
};

struct ControlCommand {
  enum Type : uint8_t { SetStream, BlinkLed } type;
  uint16_t value;
};

struct BusTxMessage {
  PacketType type;
  uint8_t len;
  uint8_t payload[kMaxPayloadSize];
};

QueueHandle_t gSensorQueue = nullptr;
QueueHandle_t gLedQueue = nullptr;
QueueHandle_t gControlQueue = nullptr;
QueueHandle_t gBusTxQueue = nullptr;
volatile TickType_t gLastMasterPacketTick = 0;
PayloadSensor gLastSample{};
volatile bool gLastSampleValid = false;
std::atomic<uint32_t> gAnnounceCount{0};
FrameParser gParser;

void setLed(bool on) { digitalWrite(LED_PIN, on ? LOW : HIGH); }

void queueTextReply(const char *text) {
  BusTxMessage out{};
  out.type = PacketType::TextMessage;
  out.len = static_cast<uint8_t>(strnlen(text, kMaxPayloadSize - 1));
  memcpy(out.payload, text, out.len);
  xQueueSend(gBusTxQueue, &out, pdMS_TO_TICKS(10));
}

void queueSensorReply() {
  if (!gLastSampleValid) {
    return;
  }
  BusTxMessage out{};
  out.type = PacketType::SensorData;
  out.len = sizeof(PayloadSensor);
  memcpy(out.payload, &gLastSample, sizeof(PayloadSensor));
  xQueueSend(gBusTxQueue, &out, pdMS_TO_TICKS(10));
}

void handleCommand(const char *text) {
  if (strcmp(text, "POLL") == 0) {
    queueSensorReply();
    return;
  }

  if (strncmp(text, "blink ", 6) == 0) {
    unsigned blinkCount = 0;
    if (sscanf(text, "blink %u", &blinkCount) == 1 && blinkCount > 0 && blinkCount <= 1000) {
      ControlCommand cmd{};
      cmd.type = ControlCommand::BlinkLed;
      cmd.value = static_cast<uint16_t>(blinkCount);
      xQueueSend(gControlQueue, &cmd, 0);
      queueTextReply("ack blink");
      return;
    }
    queueTextReply("err blink");
    return;
  }

  if (strcmp(text, "flush") == 0) {
    queueTextReply("ack flush");
    return;
  }

  if (strcmp(text, "stream on") == 0) {
    ControlCommand cmd{};
    cmd.type = ControlCommand::SetStream;
    cmd.value = 1;
    xQueueSend(gControlQueue, &cmd, 0);
    queueTextReply("ack stream on");
    return;
  }

  if (strcmp(text, "stream off") == 0) {
    ControlCommand cmd{};
    cmd.type = ControlCommand::SetStream;
    cmd.value = 0;
    xQueueSend(gControlQueue, &cmd, 0);
    queueTextReply("ack stream off");
    return;
  }

  char generic[kMaxPayloadSize] = {0};
  snprintf(generic, sizeof(generic), "ack %s", text);
  queueTextReply(generic);
}

void Task_UART(void *) {
  uint8_t outFrame[kMaxFrameSize] = {0};

  for (;;) {
    BusTxMessage tx{};
    while (xQueueReceive(gBusTxQueue, &tx, 0) == pdPASS) {
      const size_t len = encodeFrame(tx.type, tx.payload, tx.len, outFrame, sizeof(outFrame));
      if (len > 0) {
        Serial1.write(outFrame, len);
        Serial1.flush();
      }
    }

    while (Serial1.available() > 0) {
      FrameMessage in{};
      const uint8_t b = static_cast<uint8_t>(Serial1.read());
      if (gParser.parseByte(b, in)) {
        LedEvent ledEv{};
        ledEv.type = LedEvent::RxPacket;
        xQueueSend(gLedQueue, &ledEv, 0);
        gLastMasterPacketTick = xTaskGetTickCount();

        if (in.type == PacketType::Command) {
          char text[kMaxPayloadSize + 1] = {0};
          memcpy(text, in.payload, in.length);
          text[in.length] = '\0';
          handleCommand(text);
        }
      }
    }

    vTaskDelay(1);
  }
}

void Task_Sensors(void *) {
  TickType_t periodTicks = SENSOR_POLL_NORMAL;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    ControlCommand ctrl{};
    while (xQueueReceive(gControlQueue, &ctrl, 0) == pdPASS) {
      if (ctrl.type == ControlCommand::SetStream) {
        periodTicks = (ctrl.value != 0) ? SENSOR_POLL_STREAM : SENSOR_POLL_NORMAL;
      } else if (ctrl.type == ControlCommand::BlinkLed) {
        LedEvent led{};
        led.type = LedEvent::BlinkRequest;
        led.blink_count = ctrl.value;
        xQueueSend(gLedQueue, &led, 0);
      }
    }

    gLastSample.uptime_ms = millis();
    gLastSample.tank_full = (digitalRead(REED_PIN) == LOW) ? 1 : 0;
    for (uint8_t i = 0; i < 5; ++i) {
      gLastSample.hw390_raw[i] = static_cast<uint16_t>(analogRead(HW390_PINS[i]));
    }
    for (uint8_t i = 0; i < 2; ++i) {
      gLastSample.ldr_raw[i] = static_cast<uint16_t>(analogRead(LDR_PINS[i]));
    }
    gLastSample.ds18_c[0] = NAN;
    gLastSample.ds18_c[1] = NAN;
    gLastSample.ds18_c[2] = NAN;

    gLastSampleValid = true;
    xQueueOverwrite(gSensorQueue, &gLastSample);
    vTaskDelayUntil(&lastWake, periodTicks);
  }
}

void Task_Heartbeat(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    const TickType_t now = xTaskGetTickCount();
    if (gLastMasterPacketTick == 0 || (now - gLastMasterPacketTick) > BUS_OK_AGE_TICKS) {
      // Anti-Panik: keine aktive Busübertragung ohne Master-Poll.
    }
    vTaskDelayUntil(&lastWake, HEARTBEAT_PERIOD_TICKS);
  }
}

void Task_Announce_Sensor(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (gLastSampleValid) {
      xQueueOverwrite(gSensorQueue, &gLastSample);
      gAnnounceCount.fetch_add(1, std::memory_order_relaxed);
    }
    vTaskDelayUntil(&lastWake, ANNOUNCE_SENSOR_TICKS);
  }
}

void Task_Status_LED(void *) {
  TickType_t lastRxTick = 0;
  TickType_t lastBlinkTick = xTaskGetTickCount();
  uint16_t blinkRemaining = 0;
  bool ledState = false;
  setLed(false);

  for (;;) {
    LedEvent ev{};
    while (xQueueReceive(gLedQueue, &ev, 0) == pdPASS) {
      if (ev.type == LedEvent::RxPacket) {
        lastRxTick = xTaskGetTickCount();
      } else if (ev.type == LedEvent::BlinkRequest) {
        blinkRemaining = static_cast<uint16_t>(ev.blink_count * 2u);
      }
    }

    if (blinkRemaining > 0) {
      ledState = !ledState;
      setLed(ledState);
      blinkRemaining--;
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
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
}  // namespace

void setup() {
  Serial.begin(115200);
  Serial1.setHalfDuplex();
  Serial1.begin(115200);

  pinMode(UART_PIN, INPUT_PULLUP);
  pinMode(REED_PIN, INPUT_PULLUP);
  for (uint8_t pin : HW390_PINS) pinMode(pin, INPUT_ANALOG);
  for (uint8_t pin : LDR_PINS) pinMode(pin, INPUT_ANALOG);
  pinMode(LED_PIN, OUTPUT);
  setLed(false);
  analogReadResolution(12);

  gSensorQueue = xQueueCreate(1, sizeof(PayloadSensor));
  gLedQueue = xQueueCreate(16, sizeof(LedEvent));
  gControlQueue = xQueueCreate(16, sizeof(ControlCommand));
  gBusTxQueue = xQueueCreate(8, sizeof(BusTxMessage));

  xTaskCreate(Task_Sensors, "Task_Sensors", 512, nullptr, tskIDLE_PRIORITY + 2, nullptr);
  xTaskCreate(Task_UART, "Task_UART", 768, nullptr, configMAX_PRIORITIES - 1, nullptr);
  xTaskCreate(Task_Status_LED, "Task_Status_LED", 320, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  xTaskCreate(Task_Heartbeat, "Task_Heartbeat", 320, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  xTaskCreate(Task_Announce_Sensor, "Task_Announce_Sensor", 320, nullptr, tskIDLE_PRIORITY + 1, nullptr);

  vTaskStartScheduler();
}

void loop() {}
