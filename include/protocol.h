#pragma once

#include <Arduino.h>

constexpr uint8_t MASTER_ID = 44;
constexpr uint8_t SLAVE_ID = 45;

struct __attribute__((packed)) SensorPacket {
  uint32_t uptime_ms;
  uint8_t tank_full;
  uint8_t valid_mask;
  float temp_c[2];
};

static_assert(sizeof(SensorPacket) == 14, "Unexpected SensorPacket size");
