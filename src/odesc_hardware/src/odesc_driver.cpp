#include "odesc_hardware/odesc_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace odesc_hardware
{

static constexpr double TWO_PI = 2.0 * M_PI;

static inline float rad_to_turns(double rad)
{
  return static_cast<float>(rad / TWO_PI);
}

static inline double turns_to_rad(float turns)
{
  return static_cast<double>(turns) * TWO_PI;
}

OdescDriver::OdescDriver(std::shared_ptr<CanInterface> can,
                         const std::vector<AxisConfig> & axes)
: can_(can), axes_(axes)
{
  states_.resize(axes_.size());
}

void OdescDriver::activate()
{
  // ODrive 3.6: IDLE alone does not clear latched errors. Must send
  // CLEAR_ERRORS (0x18) explicitly before CLOSED_LOOP_CONTROL will be accepted.
  for (const auto & axis : axes_) {
    can_->send_axis_state(axis.node_id, axis_state::IDLE);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  for (const auto & axis : axes_) {
    can_->send_clear_errors(axis.node_id);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  for (const auto & axis : axes_) {
    RCLCPP_INFO(
      rclcpp::get_logger("OdescDriver"),
      "Activating axis node_id=%d (%s)",
      axis.node_id, axis.joint_name.c_str());
    can_->send_axis_state(axis.node_id, axis_state::CLOSED_LOOP_CONTROL);
  }
}

void OdescDriver::deactivate()
{
  for (const auto & axis : axes_) {
    can_->send_velocity(axis.node_id, 0.0f, 0.0f);
    can_->send_axis_state(axis.node_id, axis_state::IDLE);
    RCLCPP_INFO(
      rclcpp::get_logger("OdescDriver"),
      "Deactivated axis node_id=%d (%s)",
      axis.node_id, axis.joint_name.c_str());
  }
}

void OdescDriver::set_velocity(size_t axis_index, double velocity_rad_s)
{
  if (axis_index >= axes_.size()) return;
  const auto & axis = axes_[axis_index];
  double vel = axis.invert ? -velocity_rad_s : velocity_rad_s;
  can_->send_velocity(axis.node_id, rad_to_turns(vel), 0.0f);
}

void OdescDriver::set_position(size_t axis_index, double position_rad)
{
  if (axis_index >= axes_.size()) return;
  const auto & axis = axes_[axis_index];
  double pos = axis.invert ? -position_rad : position_rad;
  can_->send_position(axis.node_id, rad_to_turns(pos), 0.0f);
}

void OdescDriver::request_states()
{
  for (const auto & axis : axes_) {
    can_->request_encoder_estimates(axis.node_id);
  }
}

void OdescDriver::on_can_frame(uint32_t can_id, const uint8_t * data, uint8_t len)
{
  uint8_t node_id = static_cast<uint8_t>(can_id >> 5);
  uint8_t cmd_id  = static_cast<uint8_t>(can_id & 0x1F);

  for (size_t i = 0; i < axes_.size(); i++) {
    if (axes_[i].node_id != node_id) continue;

    switch (cmd_id) {

      case can_cmd::GET_ENCODER_EST:
      {
        if (len < 8) break;
        float pos_turns, vel_turns_s;
        std::memcpy(&pos_turns,   data,     4);
        std::memcpy(&vel_turns_s, data + 4, 4);

        double pos_rad = turns_to_rad(pos_turns);
        double vel_rad = turns_to_rad(vel_turns_s);

        if (axes_[i].invert) {
          pos_rad = -pos_rad;
          vel_rad = -vel_rad;
        }

        states_[i].position_rad  = pos_rad;
        states_[i].velocity_rads = vel_rad;
        break;
      }

      case can_cmd::GET_VBUS_VOLTAGE:
      {
        if (len < 4) break;
        std::memcpy(&states_[i].bus_voltage, data, 4);  // bytes 0-3 = Vbus float
        break;
      }


      case can_cmd::HEARTBEAT:
      {
        if (len < 8) break;
        std::memcpy(&states_[i].error, data,     4);
        std::memcpy(&states_[i].state, data + 4, 4);

        if (states_[i].error != 0) {
          RCLCPP_WARN(
            rclcpp::get_logger("OdescDriver"),
            "Axis node_id=%d error=0x%08X",
            node_id, states_[i].error);
        }
        break;
      }

      default:
        break;
    }
    break;
  }
}

void OdescDriver::request_voltages()
{
  for (const auto & axis : axes_) can_->request_vbus_voltage(axis.node_id);
}

double   OdescDriver::get_position(size_t i) const { return i < states_.size() ? states_[i].position_rad  : 0.0;  }
double   OdescDriver::get_velocity(size_t i) const { return i < states_.size() ? states_[i].velocity_rads : 0.0;  }
uint32_t OdescDriver::get_error   (size_t i) const { return i < states_.size() ? states_[i].error          : 0;    }
uint32_t OdescDriver::get_state   (size_t i) const { return i < states_.size() ? states_[i].state          : 0;    }
float    OdescDriver::get_voltage (size_t i) const { return i < states_.size() ? states_[i].bus_voltage    : 0.0f; }

} // namespace odesc_hardware
