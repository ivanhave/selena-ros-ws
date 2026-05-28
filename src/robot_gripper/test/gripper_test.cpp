/*
Test the gripper by alternating between open and closed.
colcon build --packages-select robot_gripper
source install/setup.bash
ros2 run robot_gripper gripper_test
*/

#include <iostream>
#include <unistd.h>
#include "robot_gripper/mg996r_driver.hpp"

using namespace robot_gripper;

int main()
{
    std::cout << "Gripper Test — alternating open/close" << std::endl;

    MG996RDriver driver;

    if (!driver.open()) {
        std::cerr << "ERROR: Failed to open CAN." << std::endl;
        return 1;
    }

    bool open = false;

    while (true) {
        int angle = open ? ANGLE_MAX : ANGLE_MIN;

        std::cout << "\nSending: " << angle
                  << " degrees (" << (open ? "OPEN" : "CLOSED") << ")"
                  << std::endl;

        if (driver.set_angle(angle)) {
            std::cout << "SUCCESS: confirmed at "
                      << driver.get_angle() << " degrees." << std::endl;
        } else {
            std::cout << "WARNING: no confirmation." << std::endl;
        }

        open = !open;
        sleep(2);
    }

    driver.close();
    return 0;
}