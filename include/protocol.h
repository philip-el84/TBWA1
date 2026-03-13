#pragma once

#include <Arduino.h>

constexpr uint8_t MASTER_ID = 44;
constexpr uint8_t SLAVE_ID = 45;

struct __attribute__((packed)) SensorPacket {
  uint32_t uptime_ms;
  uint8_t tank_full;
  uint16_t moisture_raw[2];
};

struct __attribute__((packed)) CommandPacket {
  char text[64];
};

static_assert(sizeof(SensorPacket) == 9, "Unexpected SensorPacket size");
static_assert(sizeof(CommandPacket) == 64, "Unexpected CommandPacket size");
