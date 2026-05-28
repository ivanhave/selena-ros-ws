#include "odesc_hardware/can_interface.hpp"

#include <cstring>
#include <cstdio>

// SocketCAN headers
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>

namespace odesc_hardware
{

// ──────────────────────────────────────────────────────────────
// Constructor / Destructor
// ──────────────────────────────────────────────────────────────

CanInterface::CanInterface(const std::string & can_interface_name)
: iface_name_(can_interface_name)
{
}

CanInterface::~CanInterface()
{
  close();
}

// ──────────────────────────────────────────────────────────────
// Open
// ──────────────────────────────────────────────────────────────

bool CanInterface::open()
{
  // 1. Create raw CAN socket
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::perror("CanInterface: socket()");
    return false;
  }

  // 2. Look up interface index by name (e.g. "can0")
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, iface_name_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::perror("CanInterface: ioctl(SIOCGIFINDEX)");
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // 3. Bind socket to the interface
  struct sockaddr_can addr{};
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    std::perror("CanInterface: bind()");
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // 4. Set non-blocking so spin_once() never stalls the control loop
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

  return true;
}

// ──────────────────────────────────────────────────────────────
// Close
// ──────────────────────────────────────────────────────────────

void CanInterface::close()
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

// ──────────────────────────────────────────────────────────────
// Raw send
// ──────────────────────────────────────────────────────────────

bool CanInterface::send_frame(uint32_t can_id, const uint8_t * data, uint8_t len)
{
  if (socket_fd_ < 0) return false;

  struct can_frame frame{};
  frame.can_id  = can_id;
  frame.can_dlc = len;
  std::memcpy(frame.data, data, len);

  ssize_t bytes = write(socket_fd_, &frame, sizeof(frame));
  return bytes == sizeof(frame);
}

// ──────────────────────────────────────────────────────────────
// Raw receive (non-blocking)
// ──────────────────────────────────────────────────────────────

bool CanInterface::receive_frame(uint32_t & can_id, uint8_t * data, uint8_t & len)
{
  if (socket_fd_ < 0) return false;

  struct can_frame frame{};
  ssize_t bytes = read(socket_fd_, &frame, sizeof(frame));
  if (bytes < 0) return false;  // EAGAIN = no frame available

  can_id = frame.can_id;
  len    = frame.can_dlc;
  std::memcpy(data, frame.data, len);
  return true;
}

// ──────────────────────────────────────────────────────────────
// spin_once — drain all pending frames, fire callback
// ──────────────────────────────────────────────────────────────

void CanInterface::spin_once()
{
  uint32_t can_id;
  uint8_t  data[8];
  uint8_t  len;

  while (receive_frame(can_id, data, len)) {
    if (frame_callback_) {
      frame_callback_(can_id, data, len);
    }
  }
}

// ──────────────────────────────────────────────────────────────
// ODrive helpers
// ──────────────────────────────────────────────────────────────

bool CanInterface::send_position(uint8_t node_id, float position_turns, float velocity_ff)
{
  uint8_t data[8];
  std::memcpy(data,     &position_turns, 4);  // bytes 0-3 : position (little-endian float)
  std::memcpy(data + 4, &velocity_ff,    4);  // bytes 4-7 : velocity feedforward
  return send_frame(make_can_id(node_id, can_cmd::SET_INPUT_POS), data, 8);
}

bool CanInterface::send_velocity(uint8_t node_id, float velocity_turns_s, float current_ff)
{
  uint8_t data[8];
  std::memcpy(data,     &velocity_turns_s, 4);  // bytes 0-3 : velocity
  std::memcpy(data + 4, &current_ff,       4);  // bytes 4-7 : current feedforward
  return send_frame(make_can_id(node_id, can_cmd::SET_INPUT_VEL), data, 8);
}

bool CanInterface::request_encoder_estimates(uint8_t node_id)
{
  // RTR frame — no data, just a request
  if (socket_fd_ < 0) return false;

  struct can_frame frame{};
  frame.can_id  = make_can_id(node_id, can_cmd::GET_ENCODER_EST) | CAN_RTR_FLAG;
  frame.can_dlc = 8;

  ssize_t bytes = write(socket_fd_, &frame, sizeof(frame));
  return bytes == sizeof(frame);
}

bool CanInterface::request_vbus_voltage(uint8_t node_id)
{
  if (socket_fd_ < 0) return false;

  struct can_frame frame{};
  frame.can_id  = make_can_id(node_id, can_cmd::GET_VBUS_VOLTAGE) | CAN_RTR_FLAG;
  frame.can_dlc = 8;

  ssize_t bytes = write(socket_fd_, &frame, sizeof(frame));
  return bytes == sizeof(frame);
}

bool CanInterface::send_clear_errors(uint8_t node_id)
{
  uint8_t data[4] = {0, 0, 0, 0};
  return send_frame(make_can_id(node_id, can_cmd::CLEAR_ERRORS), data, 4);
}

bool CanInterface::send_axis_state(uint8_t node_id, uint32_t state)
{
  uint8_t data[4];
  std::memcpy(data, &state, 4);
  return send_frame(make_can_id(node_id, can_cmd::SET_AXIS_STATE), data, 4);
}

} // namespace odesc_hardware