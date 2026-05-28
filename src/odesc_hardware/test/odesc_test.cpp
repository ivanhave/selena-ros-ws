// Standalone CAN test — no ROS needed
// Tests can_interface and odesc_driver directly
// Usage: ./odesc_test

/* 

How to run the code:

cd ~/selena_ros_ws
colcon build --packages-select odesc_hardware --cmake-args -DCMAKE_BUILD_TYPE=Debug
~/selena_ros_ws/install/odesc_hardware/lib/odesc_hardware/odesc_test

*/

#include <iostream>
#include <thread>
#include <chrono>

#include "odesc_hardware/can_interface.hpp"
#include "odesc_hardware/odesc_driver.hpp"

using namespace odesc_hardware;
using namespace std::chrono_literals;

int main()
{
  // ── Setup ────────────────────────────────────────────────────

  // Two wheels connected, no inversion
  std::vector<AxisConfig> axes = {
    {"left_leg_front_wheel_joint", 6, false},
    {"left_leg_back_wheel_joint",  5, false},
  };

  auto can = std::make_shared<CanInterface>("can0");

  if (!can->open()) {
    std::cerr << "ERROR: Failed to open can0\n";
    return 1;
  }
  std::cout << "CAN opened successfully\n";

  OdescDriver driver(can, axes);

  // Register frame callback
  can->set_frame_callback(
    [&driver](uint32_t can_id, const uint8_t * data, uint8_t len) {
      driver.on_can_frame(can_id, data, len);
    });

  // ── Activate ─────────────────────────────────────────────────
  // Motors already in closed loop from startup_closed_loop_control
  // but we send the state command anyway to be safe
  std::cout << "Activating axes...\n";
  driver.activate();
  std::this_thread::sleep_for(500ms);

  // ── Read initial state ───────────────────────────────────────
  std::cout << "Reading initial encoder state...\n";
  driver.request_states();
  std::this_thread::sleep_for(100ms);
  can->spin_once();

  std::cout << "  FL pos=" << driver.get_position(0)
            << " vel=" << driver.get_velocity(0) << "\n";
  std::cout << "  RL pos=" << driver.get_position(1)
            << " vel=" << driver.get_velocity(1) << "\n";

  // ── Test 1: slow forward velocity ───────────────────────────
  std::cout << "\nTest 1: Forward at 1 rad/s for 2 seconds...\n";
  for (int i = 0; i < 20; i++) {
    driver.set_velocity(0, 6.0);  // FL
    driver.set_velocity(1, 6.0);  // RL

    driver.request_states();
    std::this_thread::sleep_for(100ms);
    can->spin_once();

    std::cout << "  FL pos=" << driver.get_position(0)
              << " vel=" << driver.get_velocity(0)
              << "  RL pos=" << driver.get_position(1)
              << " vel=" << driver.get_velocity(1) << "\n";
  }

  // ── Test 2: stop ─────────────────────────────────────────────
  std::cout << "\nTest 2: Stop...\n";
  driver.set_velocity(0, 0.0);
  driver.set_velocity(1, 0.0);
  std::this_thread::sleep_for(500ms);

  // ── Test 3: slow reverse ─────────────────────────────────────
  std::cout << "\nTest 3: Reverse at 1 rad/s for 2 seconds...\n";
  for (int i = 0; i < 20; i++) {
    driver.set_velocity(0, -6.0);
    driver.set_velocity(1, -6.0);

    driver.request_states();
    std::this_thread::sleep_for(100ms);
    can->spin_once();

    std::cout << "  FL pos=" << driver.get_position(0)
              << " vel=" << driver.get_velocity(0)
              << "  RL pos=" << driver.get_position(1)
              << " vel=" << driver.get_velocity(1) << "\n";
  }

  // ── Deactivate ───────────────────────────────────────────────
  std::cout << "\nDeactivating...\n";
  driver.deactivate();

  can->close();
  std::cout << "Done.\n";
  return 0;
}