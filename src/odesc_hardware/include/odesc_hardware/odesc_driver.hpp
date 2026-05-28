#pragma once

#include <vector>
#include <memory>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "odesc_hardware/odesc_types.hpp"
#include "odesc_hardware/can_interface.hpp"

namespace odesc_hardware
{

class OdescDriver
{
public:

  OdescDriver(std::shared_ptr<CanInterface> can,
              const std::vector<AxisConfig> & axes);

  void activate();
  void deactivate();

  void set_velocity(size_t axis_index, double velocity_rad_s);
  void set_position(size_t axis_index, double position_rad);

  void request_states();
  void request_voltages();
  void on_can_frame(uint32_t can_id, const uint8_t * data, uint8_t len);

  double   get_position(size_t i) const;
  double   get_velocity(size_t i) const;
  uint32_t get_error   (size_t i) const;
  uint32_t get_state   (size_t i) const;
  float    get_voltage (size_t i) const;

private:

  std::shared_ptr<CanInterface> can_;
  std::vector<AxisConfig>       axes_;
  std::vector<AxisState>        states_;
};

} // namespace odesc_hardware
