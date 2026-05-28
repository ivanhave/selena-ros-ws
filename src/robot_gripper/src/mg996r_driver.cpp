#include "robot_gripper/mg996r_driver.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <poll.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>
#include <iostream>

namespace robot_gripper
{

MG996RDriver::MG996RDriver() {}

MG996RDriver::~MG996RDriver()
{
    close();
}

bool MG996RDriver::open()
{
    // 1. Create socket
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "MG996RDriver: Failed to open CAN socket." << std::endl;
        return false;
    }

    // 2. Find interface
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, CAN_INTERFACE, IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "MG996RDriver: CAN interface '"
                  << CAN_INTERFACE << "' not found." << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // 3. Bind
    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "MG996RDriver: Failed to bind CAN socket." << std::endl;
        ::close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // 4. Filter — only receive frames from GRIPPER_CAN_ID
    struct can_filter filter;
    filter.can_id   = GRIPPER_CAN_ID;
    filter.can_mask = CAN_SFF_MASK;
    setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

    std::cout << "MG996RDriver: CAN socket opened on "
              << CAN_INTERFACE << " ID=0x"
              << std::hex << GRIPPER_CAN_ID << std::dec << std::endl;
    return true;
}

void MG996RDriver::close()
{
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
        socket_fd_ = -1;
        std::cout << "MG996RDriver: CAN socket closed." << std::endl;
    }
}

bool MG996RDriver::set_angle(int angle)
{
    if (socket_fd_ < 0) {
        std::cerr << "MG996RDriver: Socket not open." << std::endl;
        return false;
    }

    // Clamp angle
    int clamped = std::clamp(angle, ANGLE_MIN, ANGLE_MAX);
    if (clamped != angle) {
        std::cerr << "MG996RDriver: Angle " << angle
                  << " clamped to " << clamped << std::endl;
    }

    // Send CAN frame
    struct can_frame frame{};
    frame.can_id  = GRIPPER_CAN_ID;
    frame.can_dlc = 1;
    frame.data[0] = static_cast<uint8_t>(clamped);

    if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)) {
        std::cerr << "MG996RDriver: Failed to write CAN frame." << std::endl;
        return false;
    }

    // Wait for confirmation
    if (!wait_for_confirmation(clamped)) {
        std::cerr << "MG996RDriver: No confirmation from Arduino." << std::endl;
        return false;
    }

    last_angle_ = clamped;
    return true;
}

bool MG996RDriver::wait_for_confirmation(int expected_angle)
{
    struct pollfd fds;
    fds.fd     = socket_fd_;
    fds.events = POLLIN;

    if (poll(&fds, 1, CONFIRM_TIMEOUT_MS) <= 0) {
        return false;
    }

    struct can_frame reply{};
    read(socket_fd_, &reply, sizeof(reply));

    return reply.can_id == GRIPPER_CAN_ID &&
           reply.can_dlc >= 1 &&
           static_cast<int>(reply.data[0]) == expected_angle;
}

} // namespace robot_gripper