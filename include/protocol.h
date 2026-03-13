#pragma once

#include <Arduino.h>

constexpr uint8_t MASTER_ID = 44;
constexpr uint8_t SLAVE_ID = 45;
constexpr size_t kMaxTextPayload = 80;

enum class PacketType : uint8_t {
  SensorData = 0x01,
  TextMessage = 0x02,
};

struct __attribute__((packed)) PayloadSensor {
  PacketType type = PacketType::SensorData;
  uint32_t uptime_ms;
  uint8_t tank_full;
  uint16_t moisture_raw[2];
};

struct __attribute__((packed)) TextPacket {
  PacketType type = PacketType::TextMessage;
  char text[kMaxTextPayload - 1];
};

static_assert(sizeof(PayloadSensor) == 10, "Unexpected PayloadSensor size");
static_assert(sizeof(TextPacket) == kMaxTextPayload, "Unexpected TextPacket size");
