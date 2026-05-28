#pragma once

#include <cstdint>
#include <string>

namespace odesc_hardware
{

// ──────────────────────────────────────────
// CAN protocol constants (ODrive v0.5.1)
// ──────────────────────────────────────────
namespace can_cmd
{
  constexpr uint8_t HEARTBEAT        = 0x001;
  constexpr uint8_t SET_AXIS_STATE   = 0x007;
  constexpr uint8_t GET_ENCODER_EST  = 0x009;
  constexpr uint8_t SET_INPUT_POS    = 0x00C;
  constexpr uint8_t SET_INPUT_VEL    = 0x00D;
  constexpr uint8_t GET_VBUS_VOLTAGE = 0x017;  // ODrive 3.6 CAN Simple protocol
  constexpr uint8_t CLEAR_ERRORS     = 0x018;
}

// ──────────────────────────────────────────
// ODrive axis states
// ──────────────────────────────────────────
namespace axis_state
{
  constexpr uint32_t IDLE                = 1;
  constexpr uint32_t CLOSED_LOOP_CONTROL = 8;
}

// ──────────────────────────────────────────
// One axis descriptor
// ──────────────────────────────────────────
struct AxisConfig
{
  std::string joint_name;   // matches URDF joint name
  uint8_t     node_id;      // CAN node ID (5,6,7,8)
  bool        invert;       // flip direction if true
};

// ──────────────────────────────────────────
// Live state of one axis (filled by read())
// ──────────────────────────────────────────
struct AxisState
{
  double   position_rad  = 0.0;
  double   velocity_rads = 0.0;
  uint32_t error         = 0;
  uint32_t state         = 0;
  float    bus_voltage   = 0.0f;
};

// ──────────────────────────────────────────
// Helper: build CAN arbitration ID
// ──────────────────────────────────────────
inline uint32_t make_can_id(uint8_t node_id, uint8_t cmd_id)
{
  return (static_cast<uint32_t>(node_id) << 5) | cmd_id;
}

} // namespace odesc_hardware