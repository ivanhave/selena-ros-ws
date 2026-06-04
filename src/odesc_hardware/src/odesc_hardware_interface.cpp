#include "odesc_hardware/odesc_hardware_interface.hpp"
#include "odesc_hardware/odesc_driver.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace odesc_hardware
{

// ──────────────────────────────────────────────────────────────
// on_init — parse parameters from URDF
// ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn OdescHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read CAN interface name from URDF hardware parameters
  can_interface_name_ = "can0";
  if (info.hardware_parameters.count("can_interface")) {
    can_interface_name_ = info.hardware_parameters.at("can_interface");
  }

  if (!parse_joint_params(info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.assign(axis_configs_.size(), 0.0);
  hw_velocities_.assign(axis_configs_.size(), 0.0);
  hw_commands_.assign(axis_configs_.size(), 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger("OdescHardwareInterface"),
    "Initialized with %zu axes on %s",
    axis_configs_.size(), can_interface_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────
// parse_joint_params
// ──────────────────────────────────────────────────────────────

bool OdescHardwareInterface::parse_joint_params(
  const hardware_interface::HardwareInfo & info)
{
  for (const auto & joint : info.joints) {
    AxisConfig cfg;
    cfg.joint_name = joint.name;

    if (!joint.parameters.count("can_node_id")) {
      RCLCPP_ERROR(
        rclcpp::get_logger("OdescHardwareInterface"),
        "Joint '%s' missing parameter 'can_node_id'", joint.name.c_str());
      return false;
    }
    cfg.node_id = static_cast<uint8_t>(
      std::stoi(joint.parameters.at("can_node_id")));

    cfg.invert = false;
    if (joint.parameters.count("invert")) {
      cfg.invert = (joint.parameters.at("invert") == "true");
    }

    axis_configs_.push_back(cfg);

    RCLCPP_INFO(
      rclcpp::get_logger("OdescHardwareInterface"),
      "Joint '%s' → node_id=%d invert=%s",
      cfg.joint_name.c_str(), cfg.node_id, cfg.invert ? "true" : "false");
  }
  return true;
}

// ──────────────────────────────────────────────────────────────
// on_configure — open CAN socket, create driver
// ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn OdescHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  can_ = std::make_shared<CanInterface>(can_interface_name_);
  if (!can_->open()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("OdescHardwareInterface"),
      "Failed to open CAN interface '%s'", can_interface_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  driver_ = std::make_shared<OdescDriver>(can_, axis_configs_);
  can_->set_frame_callback(
    [this](uint32_t can_id, const uint8_t * data, uint8_t len) {
      driver_->on_can_frame(can_id, data, len);
    });

  voltage_pub_node_ = rclcpp::Node::make_shared("odesc_voltage_pub");
  voltage_left_pub_  = voltage_pub_node_->create_publisher<std_msgs::msg::Float32>(
      "/odesc/voltage_left",  10);
  voltage_right_pub_ = voltage_pub_node_->create_publisher<std_msgs::msg::Float32>(
      "/odesc/voltage_right", 10);

  RCLCPP_INFO(
    rclcpp::get_logger("OdescHardwareInterface"),
    "CAN interface '%s' opened successfully", can_interface_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────
// on_activate — send closed loop to all axes
// ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn OdescHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver_->activate();

  RCLCPP_INFO(
    rclcpp::get_logger("OdescHardwareInterface"),
    "ODrive axes activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────
// on_deactivate — zero velocity and idle all axes
// ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn OdescHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver_->deactivate();

  RCLCPP_INFO(
    rclcpp::get_logger("OdescHardwareInterface"),
    "ODrive axes deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ──────────────────────────────────────────────────────────────
// export_state_interfaces
// ──────────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
OdescHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < axis_configs_.size(); i++) {
    state_interfaces.emplace_back(
      axis_configs_[i].joint_name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    state_interfaces.emplace_back(
      axis_configs_[i].joint_name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }

  return state_interfaces;
}

// ──────────────────────────────────────────────────────────────
// export_command_interfaces
// ──────────────────────────────────────────────────────────────

std::vector<hardware_interface::CommandInterface>
OdescHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < axis_configs_.size(); i++) {
    command_interfaces.emplace_back(
      axis_configs_[i].joint_name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

// ──────────────────────────────────────────────────────────────
// read
// ──────────────────────────────────────────────────────────────

hardware_interface::return_type OdescHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  driver_->request_states();
  can_->spin_once();

  // Auto-recover any axis that has dropped out of CLOSED_LOOP_CONTROL.
  // check_and_recover() is rate-limited internally (2 s cooldown per axis).
  driver_->check_and_recover();

  for (size_t i = 0; i < axis_configs_.size(); i++) {
    hw_positions_[i]  = driver_->get_position(i);
    hw_velocities_[i] = driver_->get_velocity(i);
  }

  if (++voltage_tick_ >= 25) {          // ~2 Hz at 50 Hz control loop
    voltage_tick_ = 0;

    // Read values populated by the previous RTR cycle — those responses
    // arrived within ~1 ms of the last request and were processed by the
    // top-level spin_once() call above on subsequent read() invocations.
    float sum_l = 0; int cnt_l = 0;
    float sum_r = 0; int cnt_r = 0;
    for (size_t i = 0; i < axis_configs_.size(); i++) {
      float v = driver_->get_voltage(i);
      if (v < 1.0f) continue;
      if (axis_configs_[i].joint_name.find("left") != std::string::npos)
        { sum_l += v; cnt_l++; }
      else
        { sum_r += v; cnt_r++; }
    }
    std_msgs::msg::Float32 msg;
    if (cnt_l > 0) { msg.data = sum_l / cnt_l; voltage_left_pub_->publish(msg); }
    if (cnt_r > 0) { msg.data = sum_r / cnt_r; voltage_right_pub_->publish(msg); }

    // Send RTR request; response arrives within ~1 ms and will be processed
    // by spin_once() at the top of the next read() cycle.
    driver_->request_voltages();
  }

  return hardware_interface::return_type::OK;
}

// ──────────────────────────────────────────────────────────────
// write
// ──────────────────────────────────────────────────────────────

hardware_interface::return_type OdescHardwareInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < axis_configs_.size(); i++) {
    driver_->set_velocity(i, hw_commands_[i]);
  }

  return hardware_interface::return_type::OK;
}

} // namespace odesc_hardware

PLUGINLIB_EXPORT_CLASS(
  odesc_hardware::OdescHardwareInterface,
  hardware_interface::SystemInterface)