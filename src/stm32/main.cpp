#include <Arduino.h>
#include <PJONSoftwareBitBang.h>
#include <STM32FreeRTOS.h>

#include <cstring>

#include "protocol.h"

namespace {
constexpr uint8_t BUS_PIN = PA0;
constexpr uint8_t REED_PIN = PA1;
constexpr uint8_t MOISTURE_PIN_1 = PA2;
constexpr uint8_t MOISTURE_PIN_2 = PA3;
constexpr uint8_t LED_PIN = PC13;

constexpr TickType_t SENSOR_POLL_NORMAL = pdMS_TO_TICKS(2000);
constexpr TickType_t SENSOR_POLL_STREAM = pdMS_TO_TICKS(250);
constexpr TickType_t BUS_OK_AGE_TICKS = pdMS_TO_TICKS(5000);
constexpr TickType_t HEARTBEAT_PERIOD_TICKS = pdMS_TO_TICKS(3000);

struct LedEvent {
  enum Type : uint8_t { RxPacket, BlinkRequest } type;
  uint16_t blink_count;
};

struct ControlCommand {
  enum Type : uint8_t { SetStream, BlinkLed } type;
  uint16_t value;
};

struct BusTxMessage {
  uint16_t len;
  uint8_t data[kMaxTextPayload];
};

QueueHandle_t gSensorQueue = nullptr;
QueueHandle_t gLedQueue = nullptr;
QueueHandle_t gControlQueue = nullptr;
QueueHandle_t gBusTxQueue = nullptr;
PJONSoftwareBitBang bus(SLAVE_ID);
volatile TickType_t gLastMasterPacketTick = 0;

void setLed(bool on) { digitalWrite(LED_PIN, on ? LOW : HIGH); }

void queueTextReply(const char *text) {
  BusTxMessage out{};
  out.data[0] = static_cast<uint8_t>(PacketType::TextMessage);
  const size_t copyLen = strnlen(text, kMaxTextPayload - 2);
  memcpy(&out.data[1], text, copyLen);
  out.data[copyLen + 1] = '\0';
  out.len = static_cast<uint16_t>(copyLen + 2);
  xQueueSend(gBusTxQueue, &out, pdMS_TO_TICKS(10));
}

void handleTextCommand(const char *text) {
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

  if (strcmp(text, "reset") == 0) {
    queueTextReply("ack reset");
    delay(10);
    NVIC_SystemReset();
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

  if (strncmp(text, "ping", 4) == 0) {
    queueTextReply("pong");
    return;
  }

  char generic[kMaxTextPayload - 1] = {0};
  snprintf(generic, sizeof(generic), "ack %s", text);
  queueTextReply(generic);
}

void receiverFunction(uint8_t *payload, uint16_t length, const PJON_Packet_Info &) {
  if (length < 1) {
    return;
  }

  LedEvent ledEv{};
  ledEv.type = LedEvent::RxPacket;
  xQueueSend(gLedQueue, &ledEv, 0);
  gLastMasterPacketTick = xTaskGetTickCount();

  const PacketType type = static_cast<PacketType>(payload[0]);
  if (type == PacketType::TextMessage && length > 1) {
    char text[kMaxTextPayload] = {0};
    const uint16_t copyLen = min<uint16_t>(length - 1, kMaxTextPayload - 1);
    memcpy(text, payload + 1, copyLen);
    text[copyLen] = '\0';
    handleTextCommand(text);
  }
}

void Task_Sensors(void *) {
  PayloadSensor packet{};
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

    packet.type = PacketType::SensorData;
    packet.uptime_ms = millis();
    packet.tank_full = (digitalRead(REED_PIN) == LOW) ? 1 : 0;
    packet.moisture_raw[0] = static_cast<uint16_t>(analogRead(MOISTURE_PIN_1));
    packet.moisture_raw[1] = static_cast<uint16_t>(analogRead(MOISTURE_PIN_2));

    xQueueOverwrite(gSensorQueue, &packet);
    vTaskDelayUntil(&lastWake, periodTicks);
  }
}


void Task_Heartbeat(void *) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    const TickType_t now = xTaskGetTickCount();
    if (gLastMasterPacketTick == 0 || (now - gLastMasterPacketTick) > BUS_OK_AGE_TICKS) {
      char msg[kMaxTextPayload - 1] = {0};
      snprintf(msg, sizeof(msg), "hb %lu", static_cast<unsigned long>(millis()));
      queueTextReply(msg);
    }
    vTaskDelayUntil(&lastWake, HEARTBEAT_PERIOD_TICKS);
  }
}

void Task_PJON(void *) {
  PayloadSensor packet{};
  BusTxMessage tx{};

  for (;;) {
    while (xQueueReceive(gBusTxQueue, &tx, 0) == pdPASS) {
      if (tx.len > 0) {
        bus.send(MASTER_ID, tx.data, tx.len);
      }
    }

    if (xQueueReceive(gSensorQueue, &packet, 0) == pdPASS) {
      bus.send(MASTER_ID, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
    }

    bus.receive(5);
    bus.update();
    vTaskDelay(1);
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

  pinMode(REED_PIN, INPUT_PULLUP);
  pinMode(MOISTURE_PIN_1, INPUT_ANALOG);
  pinMode(MOISTURE_PIN_2, INPUT_ANALOG);
  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  analogReadResolution(12);

  bus.strategy.set_pin(BUS_PIN);
  bus.set_receiver(receiverFunction);
  bus.begin();

  gSensorQueue = xQueueCreate(1, sizeof(PayloadSensor));
  gLedQueue = xQueueCreate(16, sizeof(LedEvent));
  gControlQueue = xQueueCreate(16, sizeof(ControlCommand));
  gBusTxQueue = xQueueCreate(8, sizeof(BusTxMessage));

  xTaskCreate(Task_Sensors, "Task_Sensors", 512, nullptr, tskIDLE_PRIORITY + 2, nullptr);
  xTaskCreate(Task_PJON, "Task_PJON", 768, nullptr, configMAX_PRIORITIES - 1, nullptr);
  xTaskCreate(Task_Status_LED, "Task_Status_LED", 320, nullptr, tskIDLE_PRIORITY + 1, nullptr);
  xTaskCreate(Task_Heartbeat, "Task_Heartbeat", 320, nullptr, tskIDLE_PRIORITY + 1, nullptr);

  vTaskStartScheduler();
}

void loop() {}
