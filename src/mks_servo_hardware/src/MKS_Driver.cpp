
#include "mks_servo_hardware/MKS_Driver.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <sys/select.h>
#include <csignal>

// Tasks
// automatic flow of state data.
// go home, a bit back.
// check/learn stall methods. Check stall and stall release with tests

// Lifecycle
MKSDriver::MKSDriver(const std::string &can_network_name, std::uint8_t motor_can_id)
    : can_network_name_(can_network_name),
      motor_can_id_(motor_can_id),
      listening_running_(false),
      direction_multiplier_(1.0),
      last_valid_pos_(0.0),
      last_valid_vel_(0.0) {}

MKSDriver::~MKSDriver()
{
    close_bus();
}

bool MKSDriver::init()
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    // 1. Create the Socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0)
    {
        std::cerr << "MKS_Driver: CRITICAL - Could not create CAN socket." << std::endl;
        return false;
    }

    // 2. Identify the Interface
    std::strncpy(ifr.ifr_name, can_network_name_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        std::cerr << "MKS_Driver: ERROR - Interface '" << can_network_name_
                  << "' not found." << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // 3. Bind the Socket
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "MKS_Driver: ERROR - Failed to bind to "
                  << can_network_name_ << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // 4. Disable receiving own messages
    int loopback = 0;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
                   &loopback, sizeof(loopback)) < 0)
    {
        std::cerr << "MKS_Driver: WARNING - Failed to disable CAN loopback." << std::endl;
    }

    // Filter: only receive frames from our specific motor ID
    struct can_filter filter;
    filter.can_id = motor_can_id_;
    filter.can_mask = CAN_SFF_MASK; // Match exact ID

    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER,
                   &filter, sizeof(filter)) < 0)
    {
        std::cerr << "MKS_Driver: WARNING - Failed to set CAN filter." << std::endl;
    }

    // 5. Spawn the background listener thread
    listener_thread_ = std::thread(&MKSDriver::listenerLoop, this);

    // Small delay to ensure thread is running before first request
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::cout << "MKS_Driver: Connected to " << can_network_name_
              << " (FD: " << socket_fd_ << "). Listener thread spawned." << std::endl;

    // 6. Auto-detect direction — relies on listener thread being alive
    if (!autoDetectDirection())
    {
        std::cerr << "MKS_Driver: WARNING - Could not read homing direction." << std::endl;
    }

    return true;
}

void MKSDriver::close_bus()
{
    shutdownSocket();

    if (listener_thread_.joinable())
        listener_thread_.join();
}

bool MKSDriver::recover()
{
    close_bus();  // stop listener and close existing socket
    motor_offline_.store(false);
    return init();  // re-open socket and restart listener
}

// --- MODE ACTIVATION ---

void MKSDriver::activateAbsolutePositionMode()
{
    if (isStalled())
        releaseStall(); // Only release if actually stalled

    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    send_mks_frame(motor_can_id_, 0xF5, data, 6);
    listening_running_ = true;

    std::cout << "MKS_Driver: Absolute Position Mode activated." << std::endl;
}

void MKSDriver::activateVelocityMode()
{
    if (isStalled())
        releaseStall(); // Only release if actually stalled

    uint8_t data[3] = {0, 0, 0}; // 3 bytes only
    send_mks_frame(motor_can_id_, 0xF6, data, 3);
    listening_running_ = true;

    std::cout << "MKS_Driver: Velocity Mode activated." << std::endl;
}

void MKSDriver::deactivate()
{
    if (socket_fd_ < 0)
        return;

    listening_running_ = false;
    send_mks_frame(motor_can_id_, 0xF7, nullptr, 0);

    { // Only clear motion data — keep config data like 0x90
        std::lock_guard<std::mutex> lock(board_mutex_);
        status_board_.erase(0x31); // position
        status_board_.erase(0x32); // velocity
        status_board_.erase(0x3E); // stall
    }
    std::cout << "MKS_Driver: Motor deactivated." << std::endl;
}

// --- COMMANDS (WRITE) ---

void MKSDriver::setTargetPositionAbsoluteRadian(double pos_rad, double vel_rad_s, uint8_t acc)
{
    int32_t absolute_steps = static_cast<int32_t>(pos_rad * direction_multiplier_ * RAD_TO_STEPS);

    uint16_t speed_rpm = static_cast<uint16_t>(std::abs(vel_rad_s * RAD_TO_RPM));
    if (speed_rpm > 3000)
        speed_rpm = 3000;

    uint8_t data[6];
    data[0] = (speed_rpm >> 8) & 0xFF;
    data[1] = speed_rpm & 0xFF;
    data[2] = acc;
    data[3] = (absolute_steps >> 16) & 0xFF;
    data[4] = (absolute_steps >> 8) & 0xFF;
    data[5] = absolute_steps & 0xFF;

    send_mks_frame(motor_can_id_, 0xF5, data, 6);
}

void MKSDriver::setTargetVelocityRadianPerSec(double vel_rad_s, uint8_t acc)
{
    if (socket_fd_ == -1)
        return;

    double motor_rpm = (vel_rad_s * RAD_TO_RPM) * direction_multiplier_;

    uint8_t dir = (motor_rpm >= 0) ? 1 : 0;
    uint16_t speed_val = static_cast<uint16_t>(std::min(std::abs(motor_rpm), 3000.0));

    uint8_t data[3];
    data[0] = (dir << 7) | ((speed_val >> 8) & 0x0F);
    data[1] = speed_val & 0xFF;
    data[2] = acc;

    send_mks_frame(motor_can_id_, 0xF6, data, 3);
}

// --- FEEDBACK (READ) ---

double MKSDriver::getPositionRadian()
{
    std::vector<uint8_t> raw;
    if (getRecentData(0x31, raw, 100))
    {
        // Frame: [0x31][B1][B2][B3][B4][B5][B6][CRC]
        // Need at least 7 bytes: cmd + 6 data bytes
        if (raw.size() >= 7)
        {
            // Assemble 48-bit signed integer from bytes 1-6
            int64_t raw_value = 0;
            for (int i = 1; i <= 6; ++i)
                raw_value = (raw_value << 8) | raw[i];

            // Sign extension from 48-bit to 64-bit
            if (raw_value & 0x0000800000000000)
                raw_value |= 0xFFFF000000000000;

            last_valid_pos_ = (static_cast<double>(raw_value) / RAD_TO_STEPS) * direction_multiplier_;
        }
    }
    return last_valid_pos_;
}

double MKSDriver::getVelocityRadianPerSec()
{
    std::vector<uint8_t> raw;
    if (getRecentData(0x32, raw, 100))
    {
        if (raw.size() >= 3)
        {
            // 16-bit signed RPM — bytes 1 and 2
            int16_t rpm_raw = static_cast<int16_t>(
                (static_cast<uint16_t>(raw[1]) << 8) | raw[2]);
            last_valid_vel_ = (static_cast<double>(rpm_raw) / RAD_TO_RPM) * direction_multiplier_;
        }
    }
    return last_valid_vel_;
}

// --- SPECIFIC COMMANDS ---

bool MKSDriver::goHome()
{
    send_mks_frame(motor_can_id_, 0x91, nullptr, 0);
    std::cout << "MKS_Driver: ID " << (int)motor_can_id_
              << " homing triggered." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    int stationary_count = 0;
    auto start = std::chrono::steady_clock::now();

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 60)
        {
            std::cerr << "MKS_Driver: ID " << (int)motor_can_id_
                      << " homing timed out." << std::endl;
            return false;
        }

        double vel = std::abs(getVelocityRadianPerSec());
        if (vel < 0.05)
        {
            stationary_count++;
            if (stationary_count >= 10)
            {
                std::cout << "MKS_Driver: ID " << (int)motor_can_id_
                          << " reached home and stopped." << std::endl;
                return true;
            }
        }
        else
        {
            stationary_count = 0;
        }
    }
}

// --- STALL PROTECTION ---

// Enable stall protection ex. — 300ms window, 180 degree error threshold
void MKSDriver::enableStallProtection(uint16_t time_ms, uint16_t error_counts)
{
    uint16_t tim = time_ms / 15;

    uint8_t data[5];
    data[0] = 0x01;
    data[1] = (tim >> 8) & 0xFF;
    data[2] = tim & 0xFF;
    data[3] = (error_counts >> 8) & 0xFF;
    data[4] = error_counts & 0xFF;

    send_mks_frame(motor_can_id_, 0x9D, data, 5);

    if (waitForResponse(0x9D, 100))
    {
        std::cout << "MKS_Driver: Stall protection enabled ("
                  << time_ms << "ms window, "
                  << error_counts << " error counts)." << std::endl;
    }
    else
    {
        std::cerr << "MKS_Driver: WARNING! Stall protection ack timeout." << std::endl;
    }
}

bool MKSDriver::isStalled()
{
    std::vector<uint8_t> raw;
    if (getRecentData(0x3E, raw, 100))
    {
        if (raw.size() >= 2)
        {
            return raw[1] == 0x01;
        }
    }
    return false;
}

bool MKSDriver::releaseStall()
{
    if (socket_fd_ == -1)
        return false;

    send_mks_frame(motor_can_id_, 0x3D, nullptr, 0);

    if (waitForResponse(0x3D, 200))
    {
        {
            std::lock_guard<std::mutex> lock(board_mutex_);
            status_board_.erase(0x3E); // Clear stall flag from board
        }
        std::cout << "MKS_Driver: Stall released successfully." << std::endl;
        return true;
    }

    std::cerr << "MKS_Driver: releaseStall() timeout on ID "
              << (int)motor_can_id_ << std::endl;
    return false;
}

// PRIVATE HELPERS

void MKSDriver::send_mks_frame(uint8_t id, uint8_t cmd, uint8_t *data, uint8_t len)
{
    if (socket_fd_ == -1)
        return;
    if (len > 6)
    {
        std::cerr << "MKS_Driver: Data length " << (int)len << " too long!" << std::endl;
        return;
    }

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(struct can_frame));

    frame.can_id = id;
    frame.can_dlc = len + 2;
    frame.data[0] = cmd;

    for (int i = 0; i < len; ++i)
        frame.data[i + 1] = data[i];

    uint32_t checksum = id + cmd;
    for (int i = 0; i < len; ++i)
        checksum += data[i];
    frame.data[len + 1] = static_cast<uint8_t>(checksum & 0xFF);

    // Protect write() — can be called from multiple contexts
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (write(socket_fd_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
        std::cerr << "MKS_Driver: CAN write error on " << can_network_name_ << std::endl;
}

// Detects automatically the positive direction based on how the Hm_Dir was set manually.
bool MKSDriver::autoDetectDirection()
{
    // Per manual 5.1.10: send 0x00 with subcode 0x90 to read homing config
    uint8_t code[1] = {0x90};
    send_mks_frame(motor_can_id_, 0x00, code, 1);

    if (waitForResponse(0x90, 200))
    {
        std::vector<uint8_t> raw;
        {
            std::lock_guard<std::mutex> lock(board_mutex_);
            raw = status_board_[0x90].data;
        }

        if (raw.size() >= 7) // Full response: [0x90][homeTrig][homeDir][speedH][speedL][endLimit][hm_mode][CRC]
        {
            uint8_t home_dir = raw[2]; // homeDir is byte index 2
            direction_multiplier_ = (home_dir == 0) ? -1.0 : 1.0;

            std::cout << "MKS_Driver: 0x90 response bytes: ";
            for (auto b : raw)
                std::cout << std::hex << (int)b << " ";
            std::cout << std::dec << std::endl;

            std::cout << "MKS_Driver: ID " << (int)motor_can_id_
                      << " hmDir=" << (int)home_dir
                      << " multiplier=" << direction_multiplier_ << std::endl;
            return true;
        }
        std::cerr << "MKS_Driver: 0x90 response too short ("
                  << raw.size() << " bytes)" << std::endl;
    }

    std::cerr << "MKS_Driver: Failed to detect direction. Defaulting to 1.0." << std::endl;
    direction_multiplier_ = 1.0;
    return false;
}

void MKSDriver::updateState()
{
    if (socket_fd_ == -1)
        return;

    struct can_frame frame;

    // Drain every message currently waiting in the socket buffer.
    // We loop until the buffer is empty to ensure minimum latency.
    while (recv(socket_fd_, &frame, sizeof(struct can_frame), MSG_DONTWAIT) > 0)
    {
        // Safety: Only process frames from our specific motor.
        if (frame.can_id != motor_can_id_)
            continue;

        // MKS Protocol: First byte is always the Command/Function code (0x31, 0x32, etc.)
        uint8_t cmd = frame.data[0];

        // Prepare the payload (includes cmd byte and all data bytes)
        std::vector<uint8_t> payload(frame.data, frame.data + frame.can_dlc);

        {
            // LOCK: Protect the map during the write operation.
            std::lock_guard<std::mutex> lock(board_mutex_);

            // Overwrite the existing entry for this command with fresh data.
            status_board_[cmd].data = payload;
            status_board_[cmd].timestamp = std::chrono::steady_clock::now();
        }
    }
}

bool MKSDriver::getRecentData(uint8_t cmd, std::vector<uint8_t> &out_data, int max_age_ms)
{
    std::lock_guard<std::mutex> lock(board_mutex_);

    // 1. Check if we have ever received this specific command
    auto it = status_board_.find(cmd);
    if (it == status_board_.end())
        return false;

    // 2. Check the "Freshness"
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second.timestamp).count();

    if (elapsed > max_age_ms)
    {
        return false; // Data is "ghost data" from the past
    }

    // 3. Data is fresh! Copy it out to the caller
    out_data = it->second.data;
    return true;
}

void MKSDriver::nudgeIfStale(uint8_t cmd, int max_age_ms)
{
    if (socket_fd_ == -1)
    {
        std::cerr << "nudgeIfStale: socket is -1, skipping" << std::endl;
        return;
    }

    std::vector<uint8_t> dummy;
    if (!getRecentData(cmd, dummy, max_age_ms))
    {
        send_mks_frame(motor_can_id_, cmd, nullptr, 0);
    }
}

void MKSDriver::listenerLoop()
{
    int consecutive_misses = 0;
    const int MAX_MISSES = 20; // 20 * 20ms = 400ms

    while (true)
    {
        int fd = socket_fd_.load();
        if (fd == -1) break;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        struct timeval tv = {0, 5000};
        select(fd + 1, &readfds, nullptr, nullptr, &tv);

        updateState();

        if (listening_running_)
        {
            nudgeIfStale(0x31, 20);
            nudgeIfStale(0x32, 20);
            nudgeIfStale(0x3E, 50);
            checkMotorOnline(consecutive_misses, MAX_MISSES);
        }
    }
}

bool MKSDriver::waitForResponse(uint8_t cmd, int timeout_ms)
{
    auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now() - start_time)
               .count() < timeout_ms)
    {
        {
            std::lock_guard<std::mutex> lock(board_mutex_);
            if (status_board_.count(cmd) &&
                status_board_[cmd].timestamp >= start_time)
            {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    return false; // Timed out
}

void MKSDriver::checkMotorOnline(int &consecutive_misses, const int max_misses)
{
    std::vector<uint8_t> dummy;
    if (getRecentData(0x32, dummy, 400))
    {
        consecutive_misses = 0;
        return;
    }

    consecutive_misses++;
    if (consecutive_misses >= max_misses)
    {
        std::cerr << "MKS_Driver: Motor ID " << (int)motor_can_id_
                  << " not responding for 400ms — motor offline." << std::endl;

        listening_running_ = false;
        motor_offline_ = true; // Signal to hardware interface
        shutdownSocket();
    }
}

void MKSDriver::shutdownSocket()
{
    listening_running_ = false;

    if (socket_fd_ != -1)
    {
        int fd_to_close = socket_fd_;
        socket_fd_ = -1;
        shutdown(fd_to_close, SHUT_RDWR);
        close(fd_to_close);
        {
            std::lock_guard<std::mutex> lock(board_mutex_);
            status_board_.clear();
        }
        std::cerr << "MKS_Driver: Socket " << fd_to_close
                  << " closed. Bus inactive." << std::endl;
    }
}