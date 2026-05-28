#pragma once

#include <cstdint>
#include <string>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace robot_gripper
{

// ── Configuration — edit here only ──────────────────────
static constexpr const char*    CAN_INTERFACE  = "can0";
static constexpr uint32_t       GRIPPER_CAN_ID = 0x200;
static constexpr int            ANGLE_MIN      = 0;
static constexpr int            ANGLE_MAX      = 55;
static constexpr int            CONFIRM_TIMEOUT_MS = 500;
// ────────────────────────────────────────────────────────

class MG996RDriver
{
public:

    MG996RDriver();
    ~MG996RDriver();

    // Open CAN socket — call once at startup
    bool open();

    // Close CAN socket
    void close();

    bool is_open() const { return socket_fd_ >= 0; }

    // Send angle command to Arduino via CAN
    // Returns true if Arduino confirmed the move
    bool set_angle(int angle);

    // Last confirmed angle
    int get_angle() const { return last_angle_; }

private:

    int socket_fd_ = -1;
    int last_angle_ = 0;

    bool wait_for_confirmation(int expected_angle);
};

} // namespace robot_gripper