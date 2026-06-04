#pragma once

#include <cstdint>
#include <cstdio>
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
// ODrive v0.5.1 axis error bit flags
// ──────────────────────────────────────────
namespace axis_error
{
  constexpr uint32_t INITIALIZING                = 0x00000001;
  constexpr uint32_t SYSTEM_LEVEL                = 0x00000002;
  constexpr uint32_t TIMING_ERROR                = 0x00000004;
  constexpr uint32_t MISSING_ESTIMATE            = 0x00000008;
  constexpr uint32_t BAD_CONFIG                  = 0x00000010;
  constexpr uint32_t DRV_FAULT                   = 0x00000020;
  constexpr uint32_t MISSING_INPUT               = 0x00000040;
  constexpr uint32_t DC_BUS_OVER_VOLTAGE         = 0x00000100;
  constexpr uint32_t DC_BUS_UNDER_VOLTAGE        = 0x00000200;
  constexpr uint32_t DC_BUS_OVER_CURRENT         = 0x00000400;
  constexpr uint32_t DC_BUS_OVER_REGEN_CURRENT   = 0x00000800;
  constexpr uint32_t CURRENT_LIMIT_VIOLATION     = 0x00001000;
  constexpr uint32_t MOTOR_OVER_TEMP             = 0x00002000;
  constexpr uint32_t INVERTER_OVER_TEMP          = 0x00004000;
  constexpr uint32_t VELOCITY_LIMIT_VIOLATION    = 0x00008000;
  constexpr uint32_t WATCHDOG_TIMER_EXPIRED      = 0x01000000;
  constexpr uint32_t ESTOP_REQUESTED             = 0x04000000;
  constexpr uint32_t SPINOUT_DETECTED            = 0x08000000;
  constexpr uint32_t BRAKE_RESISTOR_DISARMED     = 0x10000000;
  constexpr uint32_t THERMISTOR_DISCONNECTED     = 0x20000000;
  constexpr uint32_t CALIBRATION_ERROR           = 0x40000000;
}

// ──────────────────────────────────────────
// ODrive v0.5.1 encoder error bit flags
// ──────────────────────────────────────────
namespace encoder_error
{
  constexpr uint32_t UNSTABLE_GAIN               = 0x00000001;
  constexpr uint32_t CPR_POLEPAIRS_MISMATCH      = 0x00000002;
  constexpr uint32_t NO_RESPONSE                 = 0x00000004;
  constexpr uint32_t UNSUPPORTED_ENCODER_MODE    = 0x00000008;
  constexpr uint32_t ILLEGAL_HALL_STATE          = 0x00000010;
  constexpr uint32_t INDEX_NOT_FOUND_YET         = 0x00000020;
  constexpr uint32_t ABS_SPI_TIMEOUT             = 0x00000040;
  constexpr uint32_t ABS_SPI_COM_FAIL            = 0x00000080;
  constexpr uint32_t ABS_SPI_NOT_READY           = 0x00000100;
  constexpr uint32_t HALL_NOT_CALIBRATED_YET     = 0x00000200;
}

// Returns human-readable names of set bits in an error word
inline std::string axis_error_string(uint32_t err)
{
  if (err == 0) return "NONE";
  std::string s;
  auto add = [&](const char * name, uint32_t bit) {
    if (err & bit) { if (!s.empty()) s += '|'; s += name; }
  };
  add("INITIALIZING",             axis_error::INITIALIZING);
  add("SYSTEM_LEVEL",             axis_error::SYSTEM_LEVEL);
  add("TIMING_ERROR",             axis_error::TIMING_ERROR);
  add("MISSING_ESTIMATE",         axis_error::MISSING_ESTIMATE);
  add("BAD_CONFIG",               axis_error::BAD_CONFIG);
  add("DRV_FAULT",                axis_error::DRV_FAULT);
  add("DC_BUS_OVER_VOLTAGE",      axis_error::DC_BUS_OVER_VOLTAGE);
  add("DC_BUS_UNDER_VOLTAGE",     axis_error::DC_BUS_UNDER_VOLTAGE);
  add("DC_BUS_OVER_CURRENT",      axis_error::DC_BUS_OVER_CURRENT);
  add("CURRENT_LIMIT_VIOLATION",  axis_error::CURRENT_LIMIT_VIOLATION);
  add("MOTOR_OVER_TEMP",          axis_error::MOTOR_OVER_TEMP);
  add("VELOCITY_LIMIT_VIOLATION", axis_error::VELOCITY_LIMIT_VIOLATION);
  add("WATCHDOG_TIMER_EXPIRED",   axis_error::WATCHDOG_TIMER_EXPIRED);
  add("SPINOUT_DETECTED",         axis_error::SPINOUT_DETECTED);
  add("CALIBRATION_ERROR",        axis_error::CALIBRATION_ERROR);
  // unknown bits
  uint32_t known = axis_error::INITIALIZING | axis_error::SYSTEM_LEVEL |
    axis_error::TIMING_ERROR | axis_error::MISSING_ESTIMATE | axis_error::BAD_CONFIG |
    axis_error::DRV_FAULT | axis_error::DC_BUS_OVER_VOLTAGE | axis_error::DC_BUS_UNDER_VOLTAGE |
    axis_error::DC_BUS_OVER_CURRENT | axis_error::CURRENT_LIMIT_VIOLATION |
    axis_error::MOTOR_OVER_TEMP | axis_error::VELOCITY_LIMIT_VIOLATION |
    axis_error::WATCHDOG_TIMER_EXPIRED | axis_error::SPINOUT_DETECTED |
    axis_error::CALIBRATION_ERROR;
  if (err & ~known) {
    char buf[12]; std::snprintf(buf, sizeof(buf), "0x%08X", err & ~known);
    if (!s.empty()) s += '|';
    s += "UNKNOWN("; s += buf; s += ')';
  }
  return s;
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