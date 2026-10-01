#ifndef REBEL_HPP_
#define REBEL_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <thread>
#include <mutex>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "igus_rebel_msgs/msg/digital_output.hpp"
#include "igus_rebel_msgs/srv/set_digital_output.hpp"
#include <hardware_interface/system_interface.hpp>

#include "igus_rebel/RebelSocket.hpp"
#include "igus_rebel/CriMessages.hpp"

// Measured: ALIVEJOG 1 % = 0.60 deg/s on every joint at override 100 (rebel_jog_calib.py)
#define JOINT_VELOCITY_SCALE (1.0 / 0.60)

using namespace hardware_interface;

namespace Igus
{
    class Rebel : public SystemInterface
    {
    public:
        enum class ControlMode
        {
            POSITION,
            VELOCITY
        };

    private:
        rclcpp::Node::SharedPtr node_;

        std::shared_ptr<RebelSocket> rebelSocket;

        // Latest status from the robot, written by the message thread, read by the control loop
        CriMessages::Status currentStatus;
        std::mutex statusLock;
        std::atomic<unsigned int> statusCount{0};
        // True while the robot reports no errors on any of the 6 joints
        std::atomic<bool> jointsReady{false};

        // Current commanded jog, sent by the alive thread
        std::array<float, 6> jog{};
        std::mutex jogLock;
        std::atomic<ControlMode> controlMode{ControlMode::VELOCITY};

        std::atomic<bool> continueAlive{false};
        std::atomic<bool> continueMessage{false};
        std::atomic<bool> continueSupervisor{false};
        std::thread aliveThread;
        std::thread messageThread;
        std::thread supervisorThread;
        int aliveWaitMs;
        bool running = false;

        // Re-send Reset/Enable when the joints are not enabled (e.g. after the emergency stop was released)
        bool autoReenable = true;
        unsigned int handledConnectionCount = 0;

        // Timeouts for bringing the robot up
        static constexpr int connectTimeoutMs = 10000;
        static constexpr int firstStatusTimeoutMs = 3000;
        static constexpr int enableTimeoutMs = 15000;
        // Enabling takes about 1 s per axis, so do not interrupt a running enable sequence
        static constexpr int reenableIntervalMs = 12000;

        int current_ccnt;
        std::mutex cntLock;

        double vel_cmd[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double pos[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double last_pos[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double vel[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double eff[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        CriMessages::Kinstate lastKinstate;
        std::array<int, 16> lastErrorJoints;
        std::string kinstateMessage;

        std::unordered_map<int, std::string> unacknowledgedCommands;
        std::mutex cmdLock;

        // ROS2 communication
        rclcpp::Service<igus_rebel_msgs::srv::SetDigitalOutput>::SharedPtr digital_output_srv_;

        // Thread functions
        void AliveThreadFunction();
        void MessageThreadFunction();
        void SupervisorThreadFunction();

        // Other functions
        int Ccnt();
        void Command(const std::string &);
        void GetConfig(const std::string &);
        void SetControlMode(const ControlMode &);
        void SendStartupSequence();
        void SendEnableSequence();
        CriMessages::Status GetLatestStatus();

        static bool JointsOk(const CriMessages::Status &);
        static bool SupplyLow(const CriMessages::Status &);
        static std::string JointErrorSummary(const CriMessages::Status &);

        // Function to react to specific status values, to display warnings, error messages, etc.
        void ProcessStatus(const CriMessages::Status &);
        void SetUpRosHardwareInterface();

    public:
        const std::vector<std::string> JOINT_NAME = {
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        // pi / 180
        const double degToRad = 0.0174532925199432957692369076848861271344287188854172545609719144;

        // IP & port
        const std::string ip = "192.168.3.11";
        const int port = 3920;

        Rebel();
        ~Rebel();

        void SetJog(const float &, const float &, const float &, const float &, const float &, const float &);
        void GetJoints(float &, float &, float &, float &, float &, float &);
        void SetDigitalOut(const int &, const bool &);

        // Interaction with hardware for ROS2
        CallbackReturn on_init(const HardwareInfo &hardware_info) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        std::vector<StateInterface> export_state_interfaces() override;
        std::vector<CommandInterface> export_command_interfaces() override;
        return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        void read();
        void write();

        void dio_callback(const std::shared_ptr<igus_rebel_msgs::srv::SetDigitalOutput::Request> request,
                          std::shared_ptr<igus_rebel_msgs::srv::SetDigitalOutput::Response> response);

        void GetReferenceInfo();

        bool Start();
        void Stop();
    };
}

#endif
