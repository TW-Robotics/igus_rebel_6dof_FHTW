#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "igus_rebel/Rebel.hpp"
#include "igus_rebel/CriKeywords.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <iostream>
#include <sstream>

namespace Igus
{

    //
    // Constructor(s) / Destructor(s)
    //
    Rebel::Rebel()
    {
    }

    Rebel::~Rebel()
    {
        Stop();
    }

    //
    // private functions
    //
    void Rebel::AliveThreadFunction()
    {
        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Starting to send ALIVEJOG");

        while (continueAlive)
        {
            std::array<float, 6> j;
            {
                std::lock_guard<std::mutex> lockGuard(jogLock);
                j = jog;
            }

            std::ostringstream msg;
            msg << std::showpoint;
            msg << std::fixed;
            msg << std::setprecision(8);
            msg << "CRISTART " << Ccnt() << " ";
            msg << "ALIVEJOG ";
            msg << j[0] << " " << j[1] << " " << j[2] << " ";
            msg << j[3] << " " << j[4] << " " << j[5] << " ";
            msg << 0.0f << " " << 0.0f << " " << 0.0f << " ";
            msg << "CRIEND" << std::endl;

            // RCLCPP_INFO(node_->get_logger(), "ALIVEJOG: %s", msg.str().c_str());
            rebelSocket->SendMessage(msg.str());

            std::this_thread::sleep_for(std::chrono::milliseconds(aliveWaitMs));
        }

        RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "Stopped to send ALIVEJOG");
    }

    void Rebel::MessageThreadFunction()
    {
        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Starting to process robot messages");

        while (continueMessage)
        {
            std::string msg;

            if (rebelSocket->WaitForMessage(msg, 100))
            {
                CriMessages::MessageType type = CriMessages::CriMessage::GetMessageType(msg);

                switch (type)
                {
                case CriMessages::MessageType::STATUS:
                {
                    CriMessages::Status status = CriMessages::Status(msg);
                    // status.Print();
                    status.Log();
                    {
                        std::lock_guard<std::mutex> lockGuard(statusLock);
                        currentStatus = status;
                    }
                    jointsReady = JointsOk(status);
                    statusCount++;
                    ProcessStatus(status);
                    break;
                }

                case CriMessages::MessageType::RUNSTATE:
                {
                    break;
                }

                case CriMessages::MessageType::MESSAGE:
                {
                    CriMessages::Message message = CriMessages::Message(msg);
                    RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Rebel MESSAGE: %s", message.message.c_str());
                    break;
                }

                case CriMessages::MessageType::CMD:
                {
                    CriMessages::Command command = CriMessages::Command(msg);

                    // Not sure if the ROS node should display these?
                    RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "CMD: %s", command.command.c_str());
                    break;
                }

                case CriMessages::MessageType::CONFIG:
                {
                    CriMessages::ConfigType configType = CriMessages::Config::GetConfigType(msg);

                    switch (configType)
                    {
                    case CriMessages::ConfigType::KINEMATICLIMITS:
                    {
                        CriMessages::KinematicLimits kinematicLimits = CriMessages::KinematicLimits(msg);
                        // kinematicLimits.Print();
                        break;
                    }
                    case CriMessages::ConfigType::UNKNOWN:
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Unknown config message: %s", msg.c_str());
                        break;
                    }
                    }

                    break;
                }

                case CriMessages::MessageType::INFO:
                {
                    CriMessages::Info info = CriMessages::Info(msg);
                    RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "INFO: %s", info.info.c_str());
                    break;
                }

                case CriMessages::MessageType::LOGMSG:
                {
                    CriMessages::LogMsg log = CriMessages::LogMsg(msg);

                    switch (log.logLevel)
                    {
                    case CriMessages::LogLevel::DEBUG:
                    {
                        RCLCPP_DEBUG(rclcpp::get_logger("igus_rebel"), "REBEL LOG: %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::APP_INFO:
                    {
                        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "REBEL LOG (APP_INFO): %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::APP_ERROR:
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "REBEL LOG (APP_ERROR): %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::INFO:
                    {
                        // The Rebel is pretty chatty with its INFO level log messages, so I've set them to output only to the ROS DEBUG level.
                        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "REBEL LOG: %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::WARN:
                    {
                        RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "REBEL LOG: %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::ERROR:
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "REBEL LOG: %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::FATAL:
                    {
                        RCLCPP_FATAL(rclcpp::get_logger("igus_rebel"), "REBEL LOG: %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }

                    case CriMessages::LogLevel::UNKNOWN:
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "REBEL LOG (UNKNOWN LOG LEVEL): %s (%ld ms)", log.logMsg.c_str(), log.timestamp);
                        break;
                    }
                    }

                    break;
                }

                case CriMessages::MessageType::VARIABLES:
                {
                    // CriMessages::Variables vars = CriMessages::Variables(msg);
                    break;
                }

                case CriMessages::MessageType::CMDERROR:
                {
                    CriMessages::CmdError error = CriMessages::CmdError(msg);
                    std::lock_guard<std::mutex> lockGuard(cmdLock);

                    try
                    {
                        std::string command = unacknowledgedCommands.at(error.recjectedCmd);
                        unacknowledgedCommands.erase(error.recjectedCmd);
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Rebel did not accept command: %s. Error message: %s", command.c_str(), error.error.c_str());
                    }
                    catch (const std::out_of_range &e)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Rebel did not accept unknown command. Error message: %s (%d)", error.error.c_str(), error.recjectedCmd);
                    }
                    break;
                }

                case CriMessages::MessageType::CMDACK:
                {
                    CriMessages::CmdAck ack = CriMessages::CmdAck(msg);
                    std::lock_guard<std::mutex> lockGuard(cmdLock);

                    try
                    {
                        std::string command = unacknowledgedCommands.at(ack.acceptedCmd);
                        unacknowledgedCommands.erase(ack.acceptedCmd);
                        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Rebel accepted command: %s", command.c_str());
                        break;
                    }
                    catch (const std::out_of_range &e)
                    {
                        RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "Rebel accepted unknown command: %d", ack.acceptedCmd);
                        break;
                    }
                    break;
                }

                case CriMessages::MessageType::CYCLESTAT:
                {
                    CriMessages::Cyclestat cyclestat = CriMessages::Cyclestat(msg);
                    // Will only output this once every 2 minutes, because this is sent every 0.5 seconds.
                    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("igus_rebel"), *node_->get_clock(), 120, "Rebel cycle statistics -- Cycletime: %d -- Workload: %d%%", cyclestat.cycletime, cyclestat.workload);
                    break;
                }

                case CriMessages::MessageType::UNKNOWN:
                {
                    RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "UNKNOW MESSAGE: %s", msg.c_str());
                    break;
                }

                case CriMessages::MessageType::OPINFO:
                {
                    break;
                }

                case CriMessages::MessageType::GSIG:
                {
                    break;
                }
                case CriMessages::MessageType::GRIPPERSTATE:
                {
                    break;
                }
                }
            }
        }

        RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "Stopped to process robot messages");
    }

    // Watches the connection and the joint state:
    // - after a reconnect the robot needs the startup sequence again
    // - if the joints are not enabled (e.g. emergency stop was pressed and released), send Reset/Enable again
    void Rebel::SupervisorThreadFunction()
    {
        auto lastEnableAttempt = std::chrono::steady_clock::now();
        bool lastReady = jointsReady;
        bool supplyLowLogged = false;

        while (continueSupervisor)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            if (!rebelSocket->IsConnected())
            {
                // Do not trust the last status while we cannot hear from the robot.
                jointsReady = false;
                continue;
            }

            unsigned int connectionCount = rebelSocket->ConnectionCount();

            if (connectionCount != handledConnectionCount)
            {
                RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "Reconnected to ReBeL, sending startup sequence again.");
                handledConnectionCount = connectionCount;
                SendStartupSequence();
                lastEnableAttempt = std::chrono::steady_clock::now();
                continue;
            }

            bool ready = jointsReady;

            if (ready != lastReady)
            {
                if (ready)
                {
                    RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "All joints enabled, ReBeL is ready.");
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "ReBeL joints are no longer enabled [%s]. Motion commands are ignored until they are enabled again.",
                                JointErrorSummary(GetLatestStatus()).c_str());
                }
                lastReady = ready;
            }

            if (ready || !autoReenable)
            {
                supplyLowLogged = false;
                continue;
            }

            CriMessages::Status status = GetLatestStatus();

            if (SupplyLow(status))
            {
                // Enabling is pointless without motor power, wait until the emergency stop is released.
                if (!supplyLowLogged)
                {
                    RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "Motor supply too low: is the emergency stop pressed? The driver enables the joints automatically once it is released.");
                    supplyLowLogged = true;
                }
                continue;
            }

            if (supplyLowLogged)
            {
                // Supply just came back, enable right away.
                supplyLowLogged = false;
                lastEnableAttempt = std::chrono::steady_clock::time_point();
            }

            if (std::chrono::steady_clock::now() - lastEnableAttempt < std::chrono::milliseconds(reenableIntervalMs))
            {
                continue;
            }

            RCLCPP_WARN(rclcpp::get_logger("igus_rebel"), "Joints not enabled [%s], sending Reset/Enable.", JointErrorSummary(status).c_str());
            SendEnableSequence();
            lastEnableAttempt = std::chrono::steady_clock::now();
        }
    }

    void Rebel::SendStartupSequence()
    {
        // Command(CriKeywords::COMMAND_CONNECT); // Gets a CMDERROR in CRI_V17
        Command(CriKeywords::COMMAND_SETACTIVE + " true");
        SendEnableSequence();
        GetConfig(CriKeywords::CONFIG_GETKINEMATICLIMITS);
    }

    void Rebel::SendEnableSequence()
    {
        Command(CriKeywords::COMMAND_RESET);
        Command(CriKeywords::COMMAND_ENABLE);
        // The jog scaling assumes the robot override is 100 %, otherwise every joint is slowed down.
        Command("Override 100");
        SetControlMode(controlMode);
    }

    CriMessages::Status Rebel::GetLatestStatus()
    {
        std::lock_guard<std::mutex> lockGuard(statusLock);
        return currentStatus;
    }

    bool Rebel::JointsOk(const CriMessages::Status &status)
    {
        for (unsigned int i = 0; i < 6; i++)
        {
            if (status.errorJoints.at(i) != 0)
            {
                return false;
            }
        }

        return true;
    }

    bool Rebel::SupplyLow(const CriMessages::Status &status)
    {
        for (unsigned int i = 0; i < 6; i++)
        {
            if (status.errorJoints.at(i) & static_cast<int>(CriMessages::ErrorJoint::ESTOP_LOWV))
            {
                return true;
            }
        }

        return false;
    }

    std::string Rebel::JointErrorSummary(const CriMessages::Status &status)
    {
        std::ostringstream summary;

        for (unsigned int i = 0; i < 6; i++)
        {
            summary << (i == 0 ? "" : " ") << "J" << i << "=0x" << std::hex << status.errorJoints.at(i) << std::dec;
        }

        return summary.str();
    }

    int Rebel::Ccnt()
    {
        std::lock_guard<std::mutex> lockGuard(cntLock);
        int current = current_ccnt;
        current_ccnt = (current_ccnt % 9999) + 1;
        return current;
    }

    void Rebel::SetDigitalOut(const int &output, const bool &is_on)
    {
        std::ostringstream cmd;
        cmd << CriKeywords::COMMAND_DOUT << " " << output << " " << (is_on ? "true" : "false");
        Command(cmd.str());
    }

    void Rebel::Command(const std::string &command)
    {
        int commandCount = Ccnt();
        std::ostringstream msg;
        msg << CriKeywords::START << " " << commandCount << " ";
        msg << CriKeywords::TYPE_CMD << " ";
        msg << command << " ";
        msg << CriKeywords::END << std::endl;

        {
            std::lock_guard<std::mutex> lockGuard(cmdLock);
            unacknowledgedCommands[commandCount] = command;
        }

        rebelSocket->SendMessage(msg.str());
    }

    void Rebel::GetConfig(const std::string &config)
    {
        std::ostringstream msg;
        msg << CriKeywords::START << " " << Ccnt() << " ";
        msg << CriKeywords::TYPE_CONFIG << " ";
        msg << config << " ";
        msg << CriKeywords::END << std::endl;

        rebelSocket->SendMessage(msg.str());
    }

    void Rebel::SetControlMode(const ControlMode &mode)
    {
        switch (mode)
        {
        case Rebel::ControlMode::POSITION:
        {
            {
                CriMessages::Status status = GetLatestStatus();
                std::lock_guard<std::mutex> lockGuard(jogLock);

                for (unsigned int i = 0; i < 6; i++)
                {
                    jog[i] = status.posJointCurrent.at(i);
                }
            }

            Command(CriKeywords::COMMAND_MOTIONTYPECARTBASE);
            controlMode = mode;
            RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Rebel now controlled by position control.");
            break;
        }

        case Rebel::ControlMode::VELOCITY:
        {
            Command(CriKeywords::COMMAND_MOTIONTYPEJOINT);
            controlMode = mode;
            RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Rebel now controlled by velocity control.");
            break;
        }
        }
    }

    void Rebel::ProcessStatus(const CriMessages::Status &status)
    {
        CriMessages::Kinstate currentKinstate = status.kinstate;
        std::array<int, 16> currentErrorJoints = status.errorJoints;

        if (lastKinstate != currentKinstate)
        {

            if (lastKinstate != CriMessages::Kinstate::NO_ERROR)
            {
                RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Kinematics error resolved [%s]", kinstateMessage.c_str());
            }

            if (currentKinstate != CriMessages::Kinstate::NO_ERROR)
            {

                switch (status.kinstate)
                {
                case CriMessages::Kinstate::JOINT_LIMIT_MIN:
                {
                    kinstateMessage = "joint at minimum limit";
                    break;
                }

                case CriMessages::Kinstate::JOINT_LIMIT_MAX:
                {
                    kinstateMessage = "joint at maximum limit";
                    break;
                }

                case CriMessages::Kinstate::CARTESIAN_SINGULARITY_CENTER:
                {
                    kinstateMessage = "cartesian singularity (center)";
                    break;
                }

                case CriMessages::Kinstate::CARTESIAN_SINGULARITY_REACH:
                {
                    kinstateMessage = "cartesian singularity (reach)";
                    break;
                }

                case CriMessages::Kinstate::CARTESIAN_SINGULARITY_WRIST:
                {
                    kinstateMessage = "cartesian singularity (wrist)";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_1:
                {
                    kinstateMessage = "tool at virtual box limit 1";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_2:
                {
                    kinstateMessage = "tool at virtual box limit 2";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_3:
                {
                    kinstateMessage = "tool at virtual box limit 3";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_4:
                {
                    kinstateMessage = "tool at virtual box limit 4";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_5:
                {
                    kinstateMessage = "tool at virtual box limit 5";
                    break;
                }

                case CriMessages::Kinstate::TOOL_AT_VIRTUAL_BOX_LIMIT_6:
                {
                    kinstateMessage = "tool at virtual box limit 6";
                    break;
                }

                case CriMessages::Kinstate::MOTION_NOT_ALLOWED:
                {
                    kinstateMessage = "motion not allowed";
                    break;
                }

                case CriMessages::Kinstate::UNKNOWN:
                {
                    kinstateMessage = "unknown error";
                    break;
                }

                case CriMessages::Kinstate::NO_ERROR:
                {
                    kinstateMessage = "no error";
                    break;
                }
                }

                RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Kinematics error [%s]", kinstateMessage.c_str());
            }
        }

        if (currentErrorJoints != lastErrorJoints)
        {

            // loop throught the 6 joint errors
            for (unsigned int i = 0; i < 6; i++)
            {
                int errorJoint = currentErrorJoints.at(i);
                std::array<int, 8> errorJointBit;

                if (errorJoint != lastErrorJoints.at(i))
                {

                    // extract bits from the error to analyze it
                    for (unsigned j = 0; j < 8; j++)
                    {
                        errorJointBit[j] = errorJoint & (int)exp2(j);
                    }

                    std::string errorMsg = "";
                    if (errorJointBit.at(0) == static_cast<int>(CriMessages::ErrorJoint::TEMP))
                    {
                        errorMsg += "'Overtemperature' ";
                    }

                    if (errorJointBit.at(1) == static_cast<int>(CriMessages::ErrorJoint::ESTOP_LOWV))
                    {
                        errorMsg += "'Supply too low: Is emergency button pressed?' ";
                    }

                    if (errorJointBit.at(2) == static_cast<int>(CriMessages::ErrorJoint::MNE))
                    {
                        errorMsg += "'Motor not enabled' ";
                    }

                    if (errorJointBit.at(3) == static_cast<int>(CriMessages::ErrorJoint::COM))
                    {
                        errorMsg += "'Communication watch dog' ";
                    }

                    if (errorJointBit.at(4) == static_cast<int>(CriMessages::ErrorJoint::POS))
                    {
                        errorMsg += "'Position lag' ";
                    }

                    if (errorJointBit.at(5) == static_cast<int>(CriMessages::ErrorJoint::ENC))
                    {
                        errorMsg += "'Encoder Error' ";
                    }

                    if (errorJointBit.at(6) == static_cast<int>(CriMessages::ErrorJoint::OC))
                    {
                        errorMsg += "'Overcurrent' ";
                    }

                    if (errorJointBit.at(7) == static_cast<int>(CriMessages::ErrorJoint::DRV))
                    {
                        errorMsg += "'DriveError/SVM' ";
                    }

                    if (errorMsg != "")
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Joint %i Error: [%s]", i, errorMsg.c_str());
                    }
                    else
                    {
                        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Joint %i Error: Cleared", i);
                    }
                }
            }
        }

        lastKinstate = currentKinstate;
        lastErrorJoints = currentErrorJoints;
    }

    //
    // public functions
    //
    void Rebel::SetJog(const float &joint1, const float &joint2, const float &joint3,
                       const float &joint4, const float &joint5, const float &joint6)
    {
        std::lock_guard<std::mutex> lockGuard(jogLock);
        jog = {joint1, joint2, joint3, joint4, joint5, joint6};
    }

    void Rebel::GetJoints(float &joint1, float &joint2, float &joint3,
                          float &joint4, float &joint5, float &joint6)
    {
        CriMessages::Status status = GetLatestStatus();
        joint1 = status.posJointCurrent.at(0);
        joint2 = status.posJointCurrent.at(1);
        joint3 = status.posJointCurrent.at(2);
        joint4 = status.posJointCurrent.at(3);
        joint5 = status.posJointCurrent.at(4);
        joint6 = status.posJointCurrent.at(5);
    }

    CallbackReturn Rebel::on_init(const HardwareInfo &hardware_info)
    {
        if (SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        }

        // Optional <param name="auto_reenable">false</param> in the URDF's <hardware> tag
        auto autoReenableParam = hardware_info.hardware_parameters.find("auto_reenable");
        autoReenable = autoReenableParam == hardware_info.hardware_parameters.end() || autoReenableParam->second != "false";

        rebelSocket = std::make_shared<RebelSocket>(ip, port, 200);
        jog.fill(0.0f);
        controlMode = Rebel::ControlMode::VELOCITY;
        current_ccnt = 1;
        continueAlive = false;
        continueMessage = false;
        aliveWaitMs = 10;
        lastKinstate = CriMessages::Kinstate::NO_ERROR;
        lastErrorJoints.fill(0);
        kinstateMessage = "";
        node_ = std::make_shared<rclcpp::Node>("igus_rebel");
        digital_output_srv_ = node_->create_service<igus_rebel_msgs::srv::SetDigitalOutput>(
            "set_digital_output", std::bind(&Rebel::dio_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Connecting to the robot happens in on_activate, so a missing robot cannot block the controller manager forever.
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Rebel::on_activate(const rclcpp_lifecycle::State &)
    {
        if (!Start())
        {
            Stop();
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Rebel::on_deactivate(const rclcpp_lifecycle::State &)
    {
        Stop();
        return CallbackReturn::SUCCESS;
    }

    std::vector<StateInterface> Rebel::export_state_interfaces()
    {
        std::vector<StateInterface> state_interfaces;

        for (int i = 0; i < 6; ++i)
        {
            state_interfaces.emplace_back(StateInterface(
                JOINT_NAME[i], hardware_interface::HW_IF_POSITION, &pos[i]));
            state_interfaces.emplace_back(StateInterface(
                JOINT_NAME[i], hardware_interface::HW_IF_VELOCITY, &vel[i]));
        }

        return state_interfaces;
    }

    std::vector<CommandInterface> Rebel::export_command_interfaces()
    {
        std::vector<CommandInterface> command_interfaces;

        for (int i = 0; i < 6; ++i)
        {
            command_interfaces.emplace_back(CommandInterface(
                JOINT_NAME[i], hardware_interface::HW_IF_VELOCITY, &vel_cmd[i]));
        }

        return command_interfaces;
    }

    return_type Rebel::read(const rclcpp::Time &, const rclcpp::Duration &period)
    {
        read();

        if (period.seconds() > 0.0)
        {
            for (unsigned int i = 0; i < 6; i++)
            {
                vel[i] = (pos[i] - last_pos[i]) / period.seconds();
            }
        }

        for (unsigned int i = 0; i < 6; i++)
        {
            last_pos[i] = pos[i];
        }
        return return_type::OK;
    }

    void Rebel::read()
    {
        std::lock_guard<std::mutex> lockGuard(statusLock);

        for (unsigned int i = 0; i < 6; i++)
        {
            pos[i] = currentStatus.posJointCurrent.at(i) * degToRad;
        }
    }

    return_type Rebel::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // Curently no use for time or period, here.
        write();
        return return_type::OK;
    }

    void Rebel::write()
    {
        // Check and call DIO callback
        if (rclcpp::ok())
        {
            rclcpp::spin_some(node_);
        }

        // Send a zero jog while the robot does not report all joints enabled.
        std::array<float, 6> j{};

        if (jointsReady)
        {
            for (unsigned int i = 0; i < 6; i++)
            {
                j[i] = (float)std::clamp(JOINT_VELOCITY_SCALE * vel_cmd[i] / degToRad, -100.0, 100.0);
            }
        }

        std::lock_guard<std::mutex> lockGuard(jogLock);
        jog = j;
    }

    void Rebel::dio_callback(
        const std::shared_ptr<igus_rebel_msgs::srv::SetDigitalOutput::Request> request,
        std::shared_ptr<igus_rebel_msgs::srv::SetDigitalOutput::Response> response)
    {
        SetDigitalOut(request->output.output, request->output.is_on);
        response->success = true;
    }

    void Rebel::GetReferenceInfo()
    {
        Command(std::string("GetReferencingInfo"));
    }

    bool Rebel::Start()
    {
        if (running)
        {
            return true;
        }

        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Connecting to ReBeL at %s:%d ...", ip.c_str(), port);

        if (!rebelSocket->Start(connectTimeoutMs))
        {
            RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"),
                         "Could not connect to ReBeL at %s:%d within %d s. Is the robot switched on, finished booting and is the network cable connected?",
                         ip.c_str(), port, connectTimeoutMs / 1000);
            return false;
        }

        running = true;
        jointsReady = false;
        statusCount = 0;
        {
            std::lock_guard<std::mutex> lockGuard(jogLock);
            jog.fill(0.0f);
        }

        continueMessage = true;
        messageThread = std::thread(&Rebel::MessageThreadFunction, this);

        continueAlive = true;
        aliveThread = std::thread(&Rebel::AliveThreadFunction, this);

        handledConnectionCount = rebelSocket->ConnectionCount();
        SendStartupSequence();

        // Wait for the first status, so the controllers start from the real joint positions.
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(firstStatusTimeoutMs);

        while (statusCount == 0)
        {
            if (std::chrono::steady_clock::now() >= deadline)
            {
                RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Connected to ReBeL, but it did not send any status within %d s.",
                             firstStatusTimeoutMs / 1000);
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        read();
        for (unsigned int i = 0; i < 6; i++)
        {
            last_pos[i] = pos[i];
            vel[i] = 0.0;
        }

        continueSupervisor = true;
        supervisorThread = std::thread(&Rebel::SupervisorThreadFunction, this);

        // The robot enables the axes one after another (about 1 s each). Wait for that, so the
        // controllers and MoveIt only start working on a robot that can actually move.
        RCLCPP_INFO(rclcpp::get_logger("igus_rebel"), "Waiting for the ReBeL to enable all joints ...");
        deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(enableTimeoutMs);

        while (!jointsReady && std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (jointsReady)
        {
            return true;
        }

        CriMessages::Status status = GetLatestStatus();
        std::string retryHint = autoReenable ? "The driver keeps trying to enable them." : "Restart the driver to try again.";

        if (SupplyLow(status))
        {
            RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Joints not enabled: motor supply too low, is the emergency stop pressed? [%s] %s",
                         JointErrorSummary(status).c_str(), retryHint.c_str());
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("igus_rebel"), "Joints not enabled after %d s [%s]. %s",
                         enableTimeoutMs / 1000, JointErrorSummary(status).c_str(), retryHint.c_str());
        }

        // Still report success: the connection works and the joints may come up later (e.g. once the
        // emergency stop is released). No motion is sent to the robot until they are enabled.
        return true;
    }

    void Rebel::Stop()
    {
        if (!running)
        {
            return;
        }
        running = false;

        continueSupervisor = false;

        if (supervisorThread.joinable())
        {
            supervisorThread.join();
        }

        {
            std::lock_guard<std::mutex> lockGuard(jogLock);
            jog.fill(0.0f);
        }

        // Give the alive thread the chance to send a zero jog before stopping it.
        std::this_thread::sleep_for(std::chrono::milliseconds(aliveWaitMs + 10));

        continueAlive = false;

        if (aliveThread.joinable())
        {
            aliveThread.join();
        }

        Command(CriKeywords::COMMAND_DISABLE);
        // Command(CriKeywords::COMMAND_DISCONNECT);
        Command(CriKeywords::COMMAND_QUIT);

        continueMessage = false;

        if (messageThread.joinable())
        {
            messageThread.join();
        }

        rebelSocket->Stop();
        jointsReady = false;
    }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    Igus::Rebel, SystemInterface);
