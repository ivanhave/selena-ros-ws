#pragma once

#include <string>
#include <cstdint>
#include <vector>
#include <functional>

#include "odesc_hardware/odesc_types.hpp"

namespace odesc_hardware
{

class CanInterface
{
public:

  // ──────────────────────────────────────────
  // Lifecycle
  // ──────────────────────────────────────────

  explicit CanInterface(const std::string & can_interface_name);
  ~CanInterface();

  // Open the SocketCAN interface (e.g. "can0")
  // Returns true on success
  bool open();

  // Close the socket
  void close();

  bool is_open() const { return socket_fd_ >= 0; }

  // ──────────────────────────────────────────
  // Raw send / receive
  // ──────────────────────────────────────────

  // Send a CAN frame
  // can_id  : arbitration ID (already built with make_can_id())
  // data    : up to 8 bytes
  // len     : number of bytes
  bool send_frame(uint32_t can_id, const uint8_t * data, uint8_t len);

  // Receive one frame (non-blocking)
  // Returns false if no frame available
  bool receive_frame(uint32_t & can_id, uint8_t * data, uint8_t & len);

  // ──────────────────────────────────────────
  // ODrive specific helpers
  // ──────────────────────────────────────────

  // Send position command (turns, velocity_ff in turns/s)
  bool send_position(uint8_t node_id, float position_turns, float velocity_ff = 0.0f);

  // Send velocity command (turns/s, current_ff in Amps)
  bool send_velocity(uint8_t node_id, float velocity_turns_s, float current_ff = 0.0f);

  // Request encoder estimates (RTR frame)
  bool request_encoder_estimates(uint8_t node_id);

  // Request Vbus voltage (RTR frame, cmd 0x017)
  bool request_vbus_voltage(uint8_t node_id);

  // Clear axis errors (cmd 0x018)
  bool send_clear_errors(uint8_t node_id);

  // Set axis state (e.g. IDLE or CLOSED_LOOP_CONTROL)
  bool send_axis_state(uint8_t node_id, uint32_t state);

  // ──────────────────────────────────────────
  // Callback for incoming frames
  // ──────────────────────────────────────────

  // Register a callback that fires for every received frame
  // Used by odesc_driver to parse heartbeats and encoder estimates
  using FrameCallback = std::function<void(uint32_t can_id, const uint8_t * data, uint8_t len)>;
  void set_frame_callback(FrameCallback cb) { frame_callback_ = cb; }

  // Call this in the read loop — drains all pending frames and fires callback
  void spin_once();

private:

  std::string      iface_name_;
  int              socket_fd_ = -1;
  FrameCallback    frame_callback_;
};

} // namespace odesc_hardware