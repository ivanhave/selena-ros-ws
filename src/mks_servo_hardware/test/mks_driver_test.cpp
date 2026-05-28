// RUNNING COMMAND:

// colcon build --packages-select mks_servo_hardware
// source install/setup.bash
// ros2 run mks_servo_hardware mks_driver_test can0


#include "mks_servo_hardware/MKS_Driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <future>

int main(int argc, char** argv) {
    std::string interface = (argc > 1) ? argv[1] : "can0";

    MKSDriver motor1(interface, 0x01);
    MKSDriver motor2(interface, 0x02);

    if (!motor1.init()) { std::cerr << "Motor 1 init failed." << std::endl; return -1; }
    if (!motor2.init()) { std::cerr << "Motor 2 init failed." << std::endl; return -1; }

    motor1.activateAbsolutePositionMode();
    motor2.activateAbsolutePositionMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Homing both motors..." << std::endl;
    std::future<bool> home1 = std::async(std::launch::async, [&]{ return motor1.goHome(); });
    std::future<bool> home2 = std::async(std::launch::async, [&]{ return motor2.goHome(); });

    bool h1 = home1.get();
    bool h2 = home2.get();

    if (!h1) { std::cerr << "Motor 1 homing failed." << std::endl; return -1; }
    if (!h2) { std::cerr << "Motor 2 homing failed." << std::endl; return -1; }

    std::cout << "Both motors homed." << std::endl;
    std::cout << "M1 pos=" << motor1.getPositionRadian()
              << " vel=" << motor1.getVelocityRadianPerSec() << std::endl;
    std::cout << "M2 pos=" << motor2.getPositionRadian()
              << " vel=" << motor2.getVelocityRadianPerSec() << std::endl;

    motor1.deactivate();
    motor2.deactivate();
    return 0;
}


/*

// Test Both motors go home

int main(int argc, char** argv) {
    std::string interface = (argc > 1) ? argv[1] : "can0";

    MKSDriver motor1(interface, 0x01);
    MKSDriver motor2(interface, 0x02);

    if (!motor1.init()) { std::cerr << "Motor 1 init failed." << std::endl; return -1; }
    if (!motor2.init()) { std::cerr << "Motor 2 init failed." << std::endl; return -1; }

    motor1.activateAbsolutePositionMode();
    motor2.activateAbsolutePositionMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Homing both motors..." << std::endl;
    std::future<bool> home1 = std::async(std::launch::async, [&]{ return motor1.goHome(); });
    std::future<bool> home2 = std::async(std::launch::async, [&]{ return motor2.goHome(); });

    bool h1 = home1.get();
    bool h2 = home2.get();

    if (!h1) { std::cerr << "Motor 1 homing failed." << std::endl; return -1; }
    if (!h2) { std::cerr << "Motor 2 homing failed." << std::endl; return -1; }
    std::cout << "Both motors homed." << std::endl;

    // Read current positions after homing
    double pos1 = motor1.getPositionRadian();
    double pos2 = motor2.getPositionRadian();
    double target1 = pos1 + (5.0 * 2.0 * M_PI);
    double target2 = pos2 + (5.0 * 2.0 * M_PI);

    std::cout << "M1 current: " << pos1 << " target: " << target1 << std::endl;
    std::cout << "M2 current: " << pos2 << " target: " << target2 << std::endl;

    motor1.setTargetPositionAbsoluteRadian(target1, 5.0, 200);
    motor2.setTargetPositionAbsoluteRadian(target2, 5.0, 200);

    std::cout << std::fixed << std::setprecision(4);
    for (int i = 0; i < 150; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        double p1 = motor1.getPositionRadian();
        double v1 = motor1.getVelocityRadianPerSec();
        double p2 = motor2.getPositionRadian();
        double v2 = motor2.getVelocityRadianPerSec();

        bool m1_done = std::abs(p1 - target1) < 0.1;
        bool m2_done = std::abs(p2 - target2) < 0.1;

        std::cout << "M1 pos: " << std::setw(9) << p1
                  << " vel: " << std::setw(8) << v1
                  << " err: " << std::setw(8) << std::abs(p1 - target1)
                  << (m1_done ? " [DONE]" : "")
                  << " | M2 pos: " << std::setw(9) << p2
                  << " vel: " << std::setw(8) << v2
                  << " err: " << std::setw(8) << std::abs(p2 - target2)
                  << (m2_done ? " [DONE]" : "")
                  << std::endl;

        if (m1_done && m2_done) {
            std::cout << "Both motors reached target!" << std::endl;
            break;
        }
    }

    motor1.deactivate();
    motor2.deactivate();
    return 0;
} */

/*

// Test: Both x motors go home.

int main(int argc, char** argv) {
    std::string interface = (argc > 1) ? argv[1] : "can0";

    MKSDriver motor1(interface, 0x01);
    MKSDriver motor2(interface, 0x02);

    if (!motor1.init()) { std::cerr << "Motor 1 init failed." << std::endl; return -1; }
    if (!motor2.init()) { std::cerr << "Motor 2 init failed." << std::endl; return -1; }

    motor1.activateAbsolutePositionMode();
    motor2.activateAbsolutePositionMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << "Homing both motors..." << std::endl;
    std::future<bool> home1 = std::async(std::launch::async, [&]{ return motor1.goHome(); });
    std::future<bool> home2 = std::async(std::launch::async, [&]{ return motor2.goHome(); });

    bool h1 = home1.get();
    bool h2 = home2.get();

    if (!h1) { std::cerr << "Motor 1 homing failed." << std::endl; return -1; }
    if (!h2) { std::cerr << "Motor 2 homing failed." << std::endl; return -1; }

    std::cout << "Both motors homed." << std::endl;
    std::cout << "M1 pos=" << motor1.getPositionRadian()
              << " vel=" << motor1.getVelocityRadianPerSec() << std::endl;
    std::cout << "M2 pos=" << motor2.getPositionRadian()
              << " vel=" << motor2.getVelocityRadianPerSec() << std::endl;

    motor1.deactivate();
    motor2.deactivate();
    return 0;
} */

/*
//Test move in the positive direction 5 rotations

int main(int argc, char** argv) {
    std::string interface = (argc > 1) ? argv[1] : "can0";
    uint8_t motor_id = (argc > 2) ? std::stoi(argv[2]) : 2;

    MKSDriver motor(interface, motor_id);
    if (!motor.init()) return -1;

    motor.activateAbsolutePositionMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    double start_pos = motor.getPositionRadian();
    double target    = start_pos + (5.0 * 2.0 * M_PI);

    std::cout << "Current pos: " << start_pos << " rad" << std::endl;
    std::cout << "Target pos:  " << target    << " rad" << std::endl;

    motor.setTargetPositionAbsoluteRadian(target, 2.0, 30);

    for (int i = 0; i < 100; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        double pos = motor.getPositionRadian();
        double vel = motor.getVelocityRadianPerSec();
        std::cout << "pos: " << std::setw(9) << std::fixed << std::setprecision(4)
                  << pos << " rad | vel: " << std::setw(9) << vel
                  << " rad/s | error: " << std::abs(pos - target) << std::endl;
        if (std::abs(pos - target) < 0.1) {
            std::cout << "Target reached!" << std::endl;
            break;
        }
    }

    motor.deactivate();
    return 0;
} */

/*

// The code below tests the stall.
// It will go back first to confirm the 0, so press the end switch when think is necessary.


// Helper to wait for motion to stop (velocity near zero)
void waitForStop(MKSDriver& driver, const std::string& phase) {
    std::cout << "Waiting for " << phase << "..." << std::endl;
    int stationary_count = 0;
    while (stationary_count < 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (std::abs(driver.getVelocityRadianPerSec()) < 0.05) {
            stationary_count++;
        } else {
            stationary_count = 0;
        }
    }
}

int main(int argc, char** argv) {
    std::string interface = (argc > 1) ? argv[1] : "can0";

    MKSDriver driver(interface, 0x01);
    if (!driver.init()) return 1;

    // 1. Establish Source of Truth: Home first
    std::cout << "\n[1/3] Homing to establish zero coordinate..." << std::endl;
    driver.goHome();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    waitForStop(driver, "Homing");

    // Set current position as the definitive zero
    driver.setCoordinateZero();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "Origin locked at: " << driver.getPositionRadian() << " rad" << std::endl;

    // 2. Setup Stall Protection
    std::cout << "\n[2/3] Enabling stall protection..." << std::endl;
    driver.enableStallProtection(300, 14000); // 300ms, 180 deg
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 3. The Stress Test
    std::cout << "\n[3/3] Spinning motor (3.0 rad/s) — BLOCK SHAFT NOW..." << std::endl;
    driver.activateVelocityMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    driver.setTargetVelocityRadianPerSec(3.0, 100);

    // Poll for stall for up to 10 seconds (50 * 200ms)
    for (int i = 0; i < 50; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        double pos = driver.getPositionRadian();
        double vel = driver.getVelocityRadianPerSec();
        std::cout << "  Pos: " << pos << " rad | Vel: " << vel << " rad/s" << std::endl;

        if (driver.isStalled()) {
            std::cerr << "\n>>> STALL DETECTED! Triggering handleStall()..." << std::endl;
            driver.handleStall();
            break;
        }
    }

    std::cout << "\nTest sequence complete. Closing connection." << std::endl;
    driver.close_bus();
    return 0;
} */

/*
// The code below tests going home, position and velocity mode and their directions
// which must coincide.

void waitForStop(MKSDriver& driver, const std::string& phase_name) {
    std::cout << "Waiting for " << phase_name << " to complete..." << std::endl;
    int zero_count = 0;
    const int required_zeros = 5;
    while (zero_count < required_zeros) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        double vel = driver.getVelocityRadianPerSec();
        double pos = driver.getPositionRadian();
        std::cout << "  vel: " << vel << " rad/s  |  pos: " << pos << " rad" << std::endl;
        if (std::abs(vel) < 0.05) {
            zero_count++;
        } else {
            zero_count = 0;
        }
    }
    std::cout << ">> " << phase_name << " confirmed stopped." << std::endl;
}

int main(int argc, char** argv) {
    std::string interface = (argc > 1) ? argv[1] : "can0";

    MKSDriver driver(interface, 0x01);
    if (!driver.init()) return 1;

    // -------------------------------------------------------
    // PHASE 1: HOMING
    // -------------------------------------------------------
    std::cout << "\n[1/5] Homing..." << std::endl;
    driver.goHome();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    waitForStop(driver, "Homing");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    driver.setCoordinateZero();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    std::cout << "Position after home (must be 0.0): "
              << driver.getPositionRadian() << " rad" << std::endl;

    // -------------------------------------------------------
    // PHASE 2: POSITION MODE — move +1 rotation away from home
    // -------------------------------------------------------
    const double one_rotation = 2.0 * M_PI;
    std::cout << "\n[2/5] Position mode: moving to +" << one_rotation << " rad..." << std::endl;
    driver.activateAbsolutePositionMode();
    driver.setTargetPositionAbsoluteRadian(one_rotation, 2.0, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    waitForStop(driver, "Move to +1 rotation");
    std::cout << "Position at target: " << driver.getPositionRadian() << " rad" << std::endl;

    // -------------------------------------------------------
    // PHASE 3: POSITION MODE — return to zero
    // -------------------------------------------------------
    std::cout << "\n[3/5] Position mode: returning to zero..." << std::endl;
    driver.setTargetPositionAbsoluteRadian(0.0, 2.0, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    waitForStop(driver, "Return to zero");
    double pos_after_return = driver.getPositionRadian();
    std::cout << "Position at zero (must be ~0.0): " << pos_after_return << " rad" << std::endl;

    if (std::abs(pos_after_return) > 0.1) {
        std::cout << "FAILURE: Position mode direction is wrong. Stopping test." << std::endl;
        driver.close_bus();
        return 1;
    }
    std::cout << "OK: Position mode direction confirmed correct." << std::endl;

    // -------------------------------------------------------
    // PHASE 4: VELOCITY MODE — spin at +3 rad/s
    // -------------------------------------------------------
    std::cout << "\n[4/5] Velocity mode: +3.0 rad/s for 7 seconds..." << std::endl;
    driver.activateVelocityMode();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    driver.setTargetVelocityRadianPerSec(3.0, 100);

    for (int i = 0; i < 35; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        double vel = driver.getVelocityRadianPerSec();
        double pos = driver.getPositionRadian();
        std::cout << "  vel: " << vel << " rad/s  |  pos: " << pos << " rad" << std::endl;
    }

    std::cout << "Stopping velocity mode..." << std::endl;
    driver.setTargetVelocityRadianPerSec(0.0, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    waitForStop(driver, "Velocity stop");

    // -------------------------------------------------------
    // PHASE 5: POSITION MODE — return to zero
    // -------------------------------------------------------
    std::cout << "\n[5/5] Position mode: returning to zero..." << std::endl;
    driver.activateAbsolutePositionMode();
    driver.setTargetPositionAbsoluteRadian(0.0, 2.0, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    waitForStop(driver, "Final return to zero");

    double final_pos = driver.getPositionRadian();
    std::cout << "\nFinal Position: " << final_pos << " rad" << std::endl;
    if (std::abs(final_pos) < 0.1) {
        std::cout << "SUCCESS: Velocity and position directions are consistent." << std::endl;
    } else {
        std::cout << "FAILURE: Motor stopped at " << final_pos
                  << " — velocity and position directions are inconsistent." << std::endl;
    }

    driver.close_bus();
    return 0;
} */
