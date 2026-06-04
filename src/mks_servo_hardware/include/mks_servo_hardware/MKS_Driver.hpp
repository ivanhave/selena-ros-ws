#ifndef MKS_DRIVER_HPP
#define MKS_DRIVER_HPP

#include <string>
#include <cstdint>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>

class MKSDriver
{
public:
    // --- CONSTRUCTOR AND DESTRUCTOR ---
    MKSDriver(const std::string &can_network_name, uint8_t motor_can_id);
    ~MKSDriver();

    bool init();
    void close_bus();

    // --- MODE ACTIVATION ---
    void activateAbsolutePositionMode();
    void activateVelocityMode();
    void deactivate();

    // --- COMMANDS (WRITE) ---
    void setTargetPositionAbsoluteRadian(double pos_rad, double vel_rad_s, uint8_t acc);
    void setTargetVelocityRadianPerSec(double vel_rad_s, uint8_t acc);

    // --- FEEDBACK (READ) ---
    double getPositionRadian();
    double getVelocityRadianPerSec();

    // --- SERVO SPECIFIC COMMANDS ---
    void startHoming(); // Non-blocking: sends 0x91 and returns immediately (use with homeXAxisCoordinated)
    bool goHome(); // Blocking: sends 0x91 then waits until stationary (use for single-motor axes)
    void zeroPositionAtHome(); // Call after goHome() — records current step count as home offset

    // --- STALL PROTECTION ---
    void enableStallProtection(uint16_t time_ms, uint16_t error_counts);
    bool isStalled();
    bool releaseStall();

    bool isOffline() const { return motor_offline_.load(); }

    // Re-open CAN socket and reset offline state — call from on_activate() to recover
    bool recover();

    // Clear a false-positive offline flag after homing.
    // MKS motors stop responding to 0x32 velocity polls while executing 0x91 homing,
    // which causes the online checker to time them out. Call this after homing completes
    // to reset the flag and force a fresh velocity query.
    void clearOfflineFlag();

private:
    // 1. Basic Identifiers
    std::string can_network_name_;
    uint8_t motor_can_id_;
    std::atomic<int> socket_fd_{-1};
    double direction_multiplier_;

    // 2. Threading & Lifecycle
    std::atomic<bool> listening_running_{false};
    std::thread listener_thread_;

    // 3. The Code Buffer (Robot State)
    struct RobotState
    {
        std::vector<uint8_t> data;
        std::chrono::steady_clock::time_point timestamp;
    };

    std::unordered_map<uint8_t, RobotState> status_board_;
    std::mutex board_mutex_;
    std::mutex send_mutex_;

    std::atomic<bool> motor_offline_{false};

    // 4. Feedback Caching (to avoid jumps if data is temporarily stale)
    double last_valid_pos_ = 0.0;
    double last_valid_vel_ = 0.0;
    int64_t pos_home_offset_steps_ = 0; // Motor step count at home; subtracted from all position reads

    // 5. Constants
    static constexpr double STEPS_PER_REV = 16384.0;
    static constexpr double RAD_TO_RPM = 60.0 / (2.0 * M_PI);
    static constexpr double RAD_TO_STEPS = 16384.0 / (2.0 * M_PI);

    // 6. Private Architecture Methods
    void listenerLoop();                            // The thread's main function
    void updateState();                             // Drains CAN buffer into status_board_
    void nudgeIfStale(uint8_t cmd, int max_age_ms); // Requests data if board is old

    void shutdownSocket();
    void checkMotorOnline(int &consecutive_misses, const int max_misses);

    // Checks the board for fresh data
    bool getRecentData(uint8_t cmd, std::vector<uint8_t> &out_data, int max_age_ms);

    // Polite wait for initialization commands (like autoDetectDirection)
    bool waitForResponse(uint8_t cmd, int timeout_ms);

    // Helpers
    void send_mks_frame(uint8_t id, uint8_t cmd, uint8_t *data, uint8_t len);
    bool autoDetectDirection();
};

#endif // MKS_DRIVER_HPP