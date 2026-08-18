#include "unitree_interface.hpp"
#include <iostream>
#include <unistd.h>
#include <iomanip>

// MotionSwitcher RPC responder (sim only): the SDK's generic service Server base class (same one
// a real robot's motion_switcher service uses). Pulls in the reg-handler macros + Request/Response.
#include <unitree/robot/server/server.hpp>

// Constructor implementations
UnitreeInterface::UnitreeInterface(const std::string& networkInterface, RobotType robot_type, MessageType message_type)
    : config_(robot_type, message_type, GetDefaultMotorCount(robot_type), GetRobotName(robot_type, message_type)),
      mode_(PyControlMode::PR), mode_machine_(0) {
    
    InitDefaultGains();
    InitializeDDS(networkInterface);
}

UnitreeInterface::UnitreeInterface(const std::string& networkInterface, const RobotConfig& config)
    : config_(config), mode_(PyControlMode::PR), mode_machine_(0) {
    
    InitDefaultGains();
    InitializeDDS(networkInterface);
}

UnitreeInterface::UnitreeInterface(const std::string& networkInterface, RobotType robot_type, MessageType message_type, int num_motors)
    : config_(robot_type, message_type, num_motors, GetRobotName(robot_type, message_type)),
      mode_(PyControlMode::PR), mode_machine_(0) {
    
    InitDefaultGains();
    InitializeDDS(networkInterface);
}

UnitreeInterface::~UnitreeInterface() {
    if (command_writer_ptr_) {
        command_writer_ptr_.reset();
    }
    lowcmd_publisher_.reset();
    lowstate_subscriber_.reset();
    wireless_subscriber_.reset();
    odom_subscriber_.reset();
    odom_publisher_.reset();
    motion_switcher_server_.reset();
}

// Helper functions for robot configuration
int UnitreeInterface::GetDefaultMotorCount(RobotType robot_type) {
    switch (robot_type) {
        case RobotType::G1: return 29;
        case RobotType::H1: return 19;
        case RobotType::H1_2: return 29;
        case RobotType::GO2: return 12;
        default: return 12;
    }
}

std::string UnitreeInterface::GetRobotName(RobotType robot_type, MessageType message_type) {
    std::string robot_name;
    switch (robot_type) {
        case RobotType::G1: robot_name = "G1"; break;
        case RobotType::H1: robot_name = "H1"; break;
        case RobotType::H1_2: robot_name = "H1-2"; break;
        case RobotType::GO2: robot_name = "GO2"; break;
        default: robot_name = "CUSTOM"; break;
    }
    
    std::string msg_type = (message_type == MessageType::HG) ? "HG" : "GO2";
    return robot_name + "-" + msg_type;
}

void UnitreeInterface::InitDefaultGains() {
    int num_motors = config_.num_motors;
    default_kp_.resize(num_motors);
    default_kd_.resize(num_motors);
    
    // Set default gains based on robot type
    switch (config_.robot_type) {
        case RobotType::G1:
        case RobotType::H1_2:
            // G1/H1-2 gains (29 motors)
            for (int i = 0; i < 12; ++i) {  // Legs
                default_kp_[i] = 60.0f;
                default_kd_[i] = 1.0f;
            }
            for (int i = 12; i < 15; ++i) {  // Waist
                default_kp_[i] = 60.0f;
                default_kd_[i] = 1.0f;
            }
            for (int i = 15; i < num_motors; ++i) {  // Arms
                default_kp_[i] = 40.0f;
                default_kd_[i] = 1.0f;
            }
            break;
            
        case RobotType::H1:
            // H1 gains (19 motors)
            for (int i = 0; i < 12; ++i) {  // Legs
                default_kp_[i] = 60.0f;
                default_kd_[i] = 1.0f;
            }
            for (int i = 12; i < 15; ++i) {  // Waist
                default_kp_[i] = 60.0f;
                default_kd_[i] = 1.0f;
            }
            for (int i = 15; i < num_motors; ++i) {  // Arms (4 motors)
                default_kp_[i] = 40.0f;
                default_kd_[i] = 1.0f;
            }
            break;

        case RobotType::GO2:
            // GO2 gains (12 motors)
            for (int i = 0; i < num_motors; ++i) {
                default_kp_[i] = 20.0f;
                default_kd_[i] = 0.5f;
            }
            break;
            
        default:
            // Custom robot
            for (int i = 0; i < num_motors; ++i) {
                default_kp_[i] = 40.0f;
                default_kd_[i] = 1.0f;
            }
            break;
    }
}

void UnitreeInterface::InitializeDDS(const std::string& networkInterface) {
    // Initialize DDS
    ChannelFactory::Instance()->Init(0, networkInterface);
    
    // Create subscribers and publishers based on message type
    if (config_.message_type == MessageType::HG) {
        // HG message type
        lowstate_subscriber_ = std::make_shared<ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(HG_STATE_TOPIC);
        lowcmd_publisher_ = std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(HG_CMD_TOPIC);
        
        auto hg_subscriber = std::static_pointer_cast<ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(lowstate_subscriber_);
        hg_subscriber->InitChannel(std::bind(&UnitreeInterface::LowStateHandler, this, std::placeholders::_1), 1);
        
        auto hg_publisher = std::static_pointer_cast<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(lowcmd_publisher_);
        hg_publisher->InitChannel();
        
        lowstate_publisher_ = std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::LowState_>>(HG_STATE_TOPIC);
        auto hg_state_pub = std::static_pointer_cast<ChannelPublisher<unitree_hg::msg::dds_::LowState_>>(lowstate_publisher_);
        hg_state_pub->InitChannel();
        
        lowcmd_subscriber_sim_ = std::make_shared<ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>>(HG_CMD_TOPIC);
        auto hg_cmd_sub = std::static_pointer_cast<ChannelSubscriber<unitree_hg::msg::dds_::LowCmd_>>(lowcmd_subscriber_sim_);
        hg_cmd_sub->InitChannel(std::bind(&UnitreeInterface::IncomingLowCmdHandler, this, std::placeholders::_1), 1);
        
    } else {
        // GO2 message type
        lowstate_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(GO2_STATE_TOPIC);
        lowcmd_publisher_ = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(GO2_CMD_TOPIC);
        
        auto go2_subscriber = std::static_pointer_cast<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(lowstate_subscriber_);
        go2_subscriber->InitChannel(std::bind(&UnitreeInterface::LowStateHandler, this, std::placeholders::_1), 1);
        
        auto go2_publisher = std::static_pointer_cast<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(lowcmd_publisher_);
        go2_publisher->InitChannel();
        
        lowstate_publisher_ = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowState_>>(GO2_STATE_TOPIC);
        auto go2_state_pub = std::static_pointer_cast<ChannelPublisher<unitree_go::msg::dds_::LowState_>>(lowstate_publisher_);
        go2_state_pub->InitChannel();
        
        lowcmd_subscriber_sim_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>>(GO2_CMD_TOPIC);
        auto go2_cmd_sub = std::static_pointer_cast<ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>>(lowcmd_subscriber_sim_);
        go2_cmd_sub->InitChannel(std::bind(&UnitreeInterface::IncomingLowCmdHandler, this, std::placeholders::_1), 1);
    }
    
    // Wireless controller subscriber (same for both message types)
    wireless_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>>(TOPIC_JOYSTICK);
    auto wireless_sub = std::static_pointer_cast<ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>>(wireless_subscriber_);
    wireless_sub->InitChannel(std::bind(&UnitreeInterface::WirelessControllerHandler, this, std::placeholders::_1), 1);
    
    wireless_publisher_ = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::WirelessController_>>(TOPIC_JOYSTICK);
    auto wireless_pub = std::static_pointer_cast<ChannelPublisher<unitree_go::msg::dds_::WirelessController_>>(wireless_publisher_);
    wireless_pub->InitChannel();

    // Odom (sport mode) state subscriber (same for both message types)
    odom_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(TOPIC_ODOMMODESTATE);
    auto odom_sub = std::static_pointer_cast<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(odom_subscriber_);
    odom_sub->InitChannel(std::bind(&UnitreeInterface::OdomStateHandler, this, std::placeholders::_1), 1);

    odom_publisher_ = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::SportModeState_>>(TOPIC_ODOMMODESTATE);
    auto odom_pub = std::static_pointer_cast<ChannelPublisher<unitree_go::msg::dds_::SportModeState_>>(odom_publisher_);
    odom_pub->InitChannel();

    // Create command writer thread
    command_writer_ptr_ = CreateRecurrentThreadEx(
        "command_writer", UT_CPU_ID_NONE, 2000, &UnitreeInterface::LowCommandWriter, this);
    
    std::cout << "UnitreeInterface initialized: " << config_.name 
              << " (" << config_.num_motors << " motors, " 
              << (config_.message_type == MessageType::HG ? "HG" : "GO2") << " messages)"
              << " on interface: " << networkInterface << std::endl;
}

void UnitreeInterface::LowStateHandler(const void *message) {
    if (config_.message_type == MessageType::HG) {
        // Handle HG message
        unitree_hg::msg::dds_::LowState_ low_state = *(const unitree_hg::msg::dds_::LowState_ *)message;
        
        // CRC check
        if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
            std::cout << "[ERROR] low_state CRC Error (HG)" << std::endl;
            return;
        }
        
        ProcessLowState(low_state);
        
    } else {
        // Handle GO2 message
        unitree_go::msg::dds_::LowState_ low_state = *(const unitree_go::msg::dds_::LowState_ *)message;
        
        // CRC check
        if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(unitree_go::msg::dds_::LowState_) >> 2) - 1)) {
            std::cout << "[ERROR] low_state CRC Error (GO2)" << std::endl;
            return;
        }
        
        ProcessLowState(low_state);
    }
}

void UnitreeInterface::ProcessLowState(const unitree_hg::msg::dds_::LowState_& low_state) {
    // Get motor state
    MotorState ms_tmp(config_.num_motors);
    for (int i = 0; i < config_.num_motors; ++i) {
        ms_tmp.q[i] = low_state.motor_state()[i].q();
        ms_tmp.dq[i] = low_state.motor_state()[i].dq();
        ms_tmp.tau_est[i] = low_state.motor_state()[i].tau_est();
        ms_tmp.temperature[i] = low_state.motor_state()[i].temperature()[0];
        ms_tmp.voltage[i] = low_state.motor_state()[i].vol();
    }
    motor_state_buffer_.SetData(ms_tmp);
    
    // Get IMU state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    imu_tmp.quat = low_state.imu_state().quaternion();
    imu_tmp.accel = low_state.imu_state().accelerometer();
    imu_state_buffer_.SetData(imu_tmp);

    // Carry the controller tick through to Python (used for timing / staleness checks).
    tick_ = low_state.tick();

    // Update mode machine
    if (mode_machine_ != low_state.mode_machine()) {
        if (mode_machine_ == 0) {
            std::cout << config_.name << " type: " << unsigned(low_state.mode_machine()) << std::endl;
        }
        mode_machine_ = low_state.mode_machine();
    }
}

void UnitreeInterface::ProcessLowState(const unitree_go::msg::dds_::LowState_& low_state) {
    // Get motor state
    MotorState ms_tmp(config_.num_motors);
    for (int i = 0; i < config_.num_motors; ++i) {
        ms_tmp.q[i] = low_state.motor_state()[i].q();
        ms_tmp.dq[i] = low_state.motor_state()[i].dq();
        ms_tmp.tau_est[i] = low_state.motor_state()[i].tau_est();
        ms_tmp.temperature[i] = low_state.motor_state()[i].temperature(); // Single value, not array
        ms_tmp.voltage[i] = 0.0f; // GO2 doesn't have voltage field
    }
    motor_state_buffer_.SetData(ms_tmp);
    
    // Get IMU state
    ImuState imu_tmp;
    imu_tmp.omega = low_state.imu_state().gyroscope();
    imu_tmp.rpy = low_state.imu_state().rpy();
    imu_tmp.quat = low_state.imu_state().quaternion();
    imu_tmp.accel = low_state.imu_state().accelerometer();
    imu_state_buffer_.SetData(imu_tmp);

    // Carry the controller tick through to Python (used for timing / staleness checks).
    tick_ = low_state.tick();

    // GO2 doesn't have mode_machine, keep current value
    // mode_machine_ remains unchanged
}

void UnitreeInterface::WirelessControllerHandler(const void *message) {
    std::lock_guard<std::mutex> lock(wireless_mutex_);
    
    unitree_go::msg::dds_::WirelessController_ wireless_msg = *(const unitree_go::msg::dds_::WirelessController_ *)message;
    
    PyWirelessController controller_tmp;
    
    controller_tmp.lx = wireless_msg.lx();
    controller_tmp.ly = wireless_msg.ly();
    controller_tmp.rx = wireless_msg.rx();
    controller_tmp.ry = wireless_msg.ry();
    controller_tmp.keys = wireless_msg.keys();
    
    wireless_controller_buffer_.SetData(controller_tmp);
}

void UnitreeInterface::OdomStateHandler(const void *message) {
    unitree_go::msg::dds_::SportModeState_ odom_msg = *(const unitree_go::msg::dds_::SportModeState_ *)message;

    PyOdomState odom_tmp;
    odom_tmp.position = odom_msg.position();
    odom_tmp.velocity = odom_msg.velocity();
    odom_tmp.yaw_speed = odom_msg.yaw_speed();
    odom_tmp.quat = odom_msg.imu_state().quaternion();

    odom_state_buffer_.SetData(odom_tmp);
}

// SIM ONLY: a MotionSwitcher service server that impersonates a real robot's motion_switcher.
// Built on the SDK's own unitree::robot::Server (the same class a real robot service uses), so
// the request/response channels, identity echo, status wrapping, api-version handshake, and
// dispatch are handled by the SDK exactly as on hardware — we only supply the handlers.
//
// The api ids + JSON schema mirror unitree/robot/b2/motion_switcher/motion_switcher_api.hpp
// (verified against JsonizeModeName): CheckMode returns {"name","form"} where an empty "name"
// means "no high-level mode active"; a robot at rest reports exactly that. "form" is omitted
// when empty, matching JsonizeModeName::toJson. SelectMode/ReleaseMode return an empty payload
// with code 0. These constants match MOTION_SWITCHER_API_VERSION / _API_ID_* in that header.
namespace {
constexpr int32_t kMotionSwitcherCheckMode = 1001;
constexpr int32_t kMotionSwitcherSelectMode = 1002;
constexpr int32_t kMotionSwitcherReleaseMode = 1003;
constexpr int32_t kMotionSwitcherSetSilent = 1004;
constexpr int32_t kMotionSwitcherGetSilent = 1005;
const char kMotionSwitcherServiceName[] = "motion_switcher";
const char kMotionSwitcherApiVersion[] = "1.0.0.1";

class MotionSwitcherSimServer : public unitree::robot::Server {
 public:
    MotionSwitcherSimServer() : unitree::robot::Server(kMotionSwitcherServiceName) {}

    void Init() override {
        SetApiVersion(kMotionSwitcherApiVersion);
        // No lease: a real MotionSwitcher gates state-changing calls on a lease; the sim stub
        // answers unconditionally so clients never block on lease acquisition.
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(kMotionSwitcherCheckMode, &MotionSwitcherSimServer::HandleCheckMode);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(kMotionSwitcherSelectMode, &MotionSwitcherSimServer::HandleOk);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(kMotionSwitcherReleaseMode, &MotionSwitcherSimServer::HandleOk);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(kMotionSwitcherSetSilent, &MotionSwitcherSimServer::HandleOk);
        UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(kMotionSwitcherGetSilent, &MotionSwitcherSimServer::HandleGetSilent);
    }

 private:
    // CheckMode -> {"name":""} : no active high-level mode (form omitted, as the robot does when
    // empty). The "name" key is emitted explicitly because strict C++ JsonizeModeName parsing
    // requires it to be present even when the value is empty.
    int32_t HandleCheckMode(const std::string& /*parameter*/, std::string& data) {
        data = "{\"name\":\"\"}";
        return 0;
    }

    // GetSilent -> {"silent":0} : mirror a robot reporting silent-mode off.
    int32_t HandleGetSilent(const std::string& /*parameter*/, std::string& data) {
        data = "{\"silent\":0}";
        return 0;
    }

    // SelectMode / ReleaseMode / SetSilent : acknowledge success, empty payload.
    int32_t HandleOk(const std::string& /*parameter*/, std::string& data) {
        data = "";
        return 0;
    }
};
}  // namespace

void UnitreeInterface::EnableMotionSwitcherResponder() {
    if (motion_switcher_enabled_) {
        return;
    }
    std::cout << "[SIM ONLY] MotionSwitcher responder enabled (fake robot answers CheckMode/"
                 "ReleaseMode). Never enable this on a real robot." << std::endl;

    // Shares the DDS domain/participant already initialized in InitializeDDS (ChannelFactory is a
    // singleton). Server::Init registers the handlers + creates the rt/api/motion_switcher/
    // request+response channels; Start() spins the request queue thread.
    auto server = std::make_shared<MotionSwitcherSimServer>();
    server->Init();
    server->Start();
    motion_switcher_server_ = server;

    motion_switcher_enabled_ = true;
}

void UnitreeInterface::LowCommandWriter() {
    const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();
    if (!mc) {
        static int no_cmd_counter = 0;
        if (no_cmd_counter % 1000 == 0) {
            std::cout << "[DEBUG] LowCommandWriter - No motor command available!" << std::endl;
        }
        no_cmd_counter++;
        return;
    }
    
    if (config_.message_type == MessageType::HG) {
        // Write HG command
        unitree_hg::msg::dds_::LowCmd_ dds_low_command;
        dds_low_command.mode_pr() = static_cast<uint8_t>(mode_);
        dds_low_command.mode_machine() = mode_machine_;
        
        for (size_t i = 0; i < config_.num_motors; i++) {
            dds_low_command.motor_cmd().at(i).mode() = 1;
            dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff[i];
            dds_low_command.motor_cmd().at(i).q() = mc->q_target[i];
            dds_low_command.motor_cmd().at(i).dq() = mc->dq_target[i];
            dds_low_command.motor_cmd().at(i).kp() = mc->kp[i];
            dds_low_command.motor_cmd().at(i).kd() = mc->kd[i];
        }
        
        dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
        
        auto hg_publisher = std::static_pointer_cast<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(lowcmd_publisher_);
        hg_publisher->Write(dds_low_command);
        
    } else {
        // Write GO2 command
        unitree_go::msg::dds_::LowCmd_ dds_low_command;
        // dds_low_command.mode_pr() = static_cast<uint8_t>(mode_);
        // dds_low_command.mode_machine() = mode_machine_;
        
        for (size_t i = 0; i < config_.num_motors; i++) {
            dds_low_command.motor_cmd().at(i).mode() = 1;
            dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff[i];
            dds_low_command.motor_cmd().at(i).q() = mc->q_target[i];
            dds_low_command.motor_cmd().at(i).dq() = mc->dq_target[i];
            dds_low_command.motor_cmd().at(i).kp() = mc->kp[i];
            dds_low_command.motor_cmd().at(i).kd() = mc->kd[i];
        }
        
        dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
        
        auto go2_publisher = std::static_pointer_cast<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(lowcmd_publisher_);
        go2_publisher->Write(dds_low_command);
    }
}

PyLowState UnitreeInterface::ConvertToPyLowState() {
    PyLowState py_state(config_.num_motors);
    
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();
    const std::shared_ptr<const ImuState> imu = imu_state_buffer_.GetData();
    
    if (ms) {
        py_state.motor.q = ms->q;
        py_state.motor.dq = ms->dq;
        py_state.motor.tau_est = ms->tau_est;
        py_state.motor.temperature = ms->temperature;
        py_state.motor.voltage = ms->voltage;
    }
    
    if (imu) {
        py_state.imu.rpy = imu->rpy;
        py_state.imu.omega = imu->omega;
        py_state.imu.quat = imu->quat;
        py_state.imu.accel = imu->accel;
    }
    
    py_state.tick = tick_;
    py_state.mode_machine = mode_machine_;

    return py_state;
}

MotorCommand UnitreeInterface::ConvertFromPyMotorCommand(const PyMotorCommand& py_cmd) {
    MotorCommand cmd(config_.num_motors);
    cmd.q_target = py_cmd.q_target;
    cmd.dq_target = py_cmd.dq_target;
    cmd.kp = py_cmd.kp;
    cmd.kd = py_cmd.kd;
    cmd.tau_ff = py_cmd.tau_ff;
    return cmd;
}

// Python interface methods
PyLowState UnitreeInterface::ReadLowState() {
    return ConvertToPyLowState();
}

PyWirelessController UnitreeInterface::ReadWirelessController() {
    const std::shared_ptr<const PyWirelessController> controller = wireless_controller_buffer_.GetData();
    if (controller) {
        return *controller;
    } else {
        return PyWirelessController{};
    }
}

PyOdomState UnitreeInterface::ReadOdomState() {
    const std::shared_ptr<const PyOdomState> odom = odom_state_buffer_.GetData();
    if (odom) {
        return *odom;
    } else {
        return PyOdomState{};
    }
}

void UnitreeInterface::WriteLowCommand(const PyMotorCommand& command) {
    MotorCommand internal_cmd = ConvertFromPyMotorCommand(command);
    motor_command_buffer_.SetData(internal_cmd);
}

void UnitreeInterface::SetControlMode(PyControlMode mode) {
    mode_ = mode;
}

PyControlMode UnitreeInterface::GetControlMode() const {
    return mode_;
}

PyMotorCommand UnitreeInterface::CreateZeroCommand() {
    PyMotorCommand cmd(config_.num_motors);
    cmd.kp = default_kp_;
    cmd.kd = default_kd_;
    
    return cmd;
}

std::vector<float> UnitreeInterface::GetDefaultKp() const {
    return default_kp_;
}

std::vector<float> UnitreeInterface::GetDefaultKd() const {
    return default_kd_;
}

// Simulation bridge methods

void UnitreeInterface::IncomingLowCmdHandler(const void *message) {
    if (config_.message_type == MessageType::HG) {
        unitree_hg::msg::dds_::LowCmd_ low_cmd = *(const unitree_hg::msg::dds_::LowCmd_ *)message;
        
        // CRC check
        if (low_cmd.crc() != Crc32Core((uint32_t *)&low_cmd, (sizeof(low_cmd) >> 2) - 1)) {
            std::cout << "[ERROR] incoming low_cmd CRC Error (HG)" << std::endl;
            return;
        }
        
        // Convert to internal MotorCommand
        MotorCommand cmd(config_.num_motors);
        for (int i = 0; i < config_.num_motors; i++) {
            cmd.q_target[i] = low_cmd.motor_cmd().at(i).q();
            cmd.dq_target[i] = low_cmd.motor_cmd().at(i).dq();
            cmd.kp[i] = low_cmd.motor_cmd().at(i).kp();
            cmd.kd[i] = low_cmd.motor_cmd().at(i).kd();
            cmd.tau_ff[i] = low_cmd.motor_cmd().at(i).tau();
        }
        
        // Store in thread-safe buffer
        incoming_command_buffer_.SetData(cmd);
        
    } else {
        // GO2 message type
        unitree_go::msg::dds_::LowCmd_ low_cmd = *(const unitree_go::msg::dds_::LowCmd_ *)message;
        
        // CRC check
        if (low_cmd.crc() != Crc32Core((uint32_t *)&low_cmd, (sizeof(low_cmd) >> 2) - 1)) {
            std::cout << "[ERROR] incoming low_cmd CRC Error (GO2)" << std::endl;
            return;
        }
        
        // Convert to internal MotorCommand
        MotorCommand cmd(config_.num_motors);
        for (int i = 0; i < config_.num_motors; i++) {
            cmd.q_target[i] = low_cmd.motor_cmd().at(i).q();
            cmd.dq_target[i] = low_cmd.motor_cmd().at(i).dq();
            cmd.kp[i] = low_cmd.motor_cmd().at(i).kp();
            cmd.kd[i] = low_cmd.motor_cmd().at(i).kd();
            cmd.tau_ff[i] = low_cmd.motor_cmd().at(i).tau();
        }
        
        // Store in thread-safe buffer
        incoming_command_buffer_.SetData(cmd);
    }
}

PyMotorCommand UnitreeInterface::ReadIncomingCommand() {
    const std::shared_ptr<const MotorCommand> cmd = incoming_command_buffer_.GetData();
    
    PyMotorCommand py_cmd(config_.num_motors);
    if (cmd) {
        py_cmd.q_target = cmd->q_target;
        py_cmd.dq_target = cmd->dq_target;
        py_cmd.kp = cmd->kp;
        py_cmd.kd = cmd->kd;
        py_cmd.tau_ff = cmd->tau_ff;
    } else {
        // Return zero-initialized command if none available
        std::fill(py_cmd.q_target.begin(), py_cmd.q_target.end(), 0.0f);
        std::fill(py_cmd.dq_target.begin(), py_cmd.dq_target.end(), 0.0f);
        std::fill(py_cmd.kp.begin(), py_cmd.kp.end(), 0.0f);
        std::fill(py_cmd.kd.begin(), py_cmd.kd.end(), 0.0f);
        std::fill(py_cmd.tau_ff.begin(), py_cmd.tau_ff.end(), 0.0f);
    }
    
    return py_cmd;
}

void UnitreeInterface::PublishWirelessController(const PyWirelessController& controller) {
    unitree_go::msg::dds_::WirelessController_ dds_controller;
    
    dds_controller.lx(controller.lx);
    dds_controller.ly(controller.ly);
    dds_controller.rx(controller.rx);
    dds_controller.ry(controller.ry);
    dds_controller.keys(controller.keys);
    
    auto pub = std::static_pointer_cast<ChannelPublisher<unitree_go::msg::dds_::WirelessController_>>(wireless_publisher_);
    pub->Write(dds_controller);
}

void UnitreeInterface::PublishOdomState(const PyOdomState& odom) {
    unitree_go::msg::dds_::SportModeState_ dds_odom;

    dds_odom.position(odom.position);
    dds_odom.velocity(odom.velocity);
    dds_odom.yaw_speed(odom.yaw_speed);
    dds_odom.imu_state().quaternion(odom.quat);

    auto pub = std::static_pointer_cast<ChannelPublisher<unitree_go::msg::dds_::SportModeState_>>(odom_publisher_);
    pub->Write(dds_odom);
}

void UnitreeInterface::PublishLowState(const PyLowState& state) {
    if (config_.message_type == MessageType::HG) {
        // HG message type
        unitree_hg::msg::dds_::LowState_ dds_state;
        
        // Populate IMU state
        dds_state.imu_state().quaternion(state.imu.quat);
        dds_state.imu_state().gyroscope(state.imu.omega);
        dds_state.imu_state().accelerometer(state.imu.accel);
        dds_state.imu_state().rpy(state.imu.rpy);
        
        // Populate motor state
        for (int i = 0; i < config_.num_motors; i++) {
            dds_state.motor_state().at(i).q(state.motor.q[i]);
            dds_state.motor_state().at(i).dq(state.motor.dq[i]);
            dds_state.motor_state().at(i).ddq(state.motor.ddq[i]);
            dds_state.motor_state().at(i).tau_est(state.motor.tau_est[i]);
            dds_state.motor_state().at(i).temperature()[0] = state.motor.temperature[i];
            dds_state.motor_state().at(i).vol(state.motor.voltage[i]);
        }
        
        // Tick, mode and CRC
        dds_state.tick(state.tick);
        dds_state.mode_machine(state.mode_machine);
        dds_state.crc(Crc32Core((uint32_t*)&dds_state, (sizeof(dds_state) >> 2) - 1));
        
        // Publish
        auto pub = std::static_pointer_cast<ChannelPublisher<unitree_hg::msg::dds_::LowState_>>(lowstate_publisher_);
        pub->Write(dds_state);
        
    } else {
        // GO2 message type
        unitree_go::msg::dds_::LowState_ dds_state;
        
        // Populate IMU state
        dds_state.imu_state().quaternion(state.imu.quat);
        dds_state.imu_state().gyroscope(state.imu.omega);
        dds_state.imu_state().accelerometer(state.imu.accel);
        dds_state.imu_state().rpy(state.imu.rpy);
        
        // Populate motor state
        for (int i = 0; i < config_.num_motors; i++) {
            dds_state.motor_state().at(i).q(state.motor.q[i]);
            dds_state.motor_state().at(i).dq(state.motor.dq[i]);
            dds_state.motor_state().at(i).ddq(state.motor.ddq[i]);
            dds_state.motor_state().at(i).tau_est(state.motor.tau_est[i]);
            dds_state.motor_state().at(i).temperature() = state.motor.temperature[i];
            // Note: GO2 doesn't have voltage field, skip it
        }
        
        // Tick and CRC (GO2 doesn't have mode_machine)
        dds_state.tick(state.tick);
        dds_state.crc(Crc32Core((uint32_t*)&dds_state, (sizeof(dds_state) >> 2) - 1));
        
        // Publish
        auto pub = std::static_pointer_cast<ChannelPublisher<unitree_go::msg::dds_::LowState_>>(lowstate_publisher_);
        pub->Write(dds_state);
    }
}

// Static factory methods
std::shared_ptr<UnitreeInterface> UnitreeInterface::CreateG1(const std::string& networkInterface, MessageType message_type) {
    return std::make_shared<UnitreeInterface>(networkInterface, RobotType::G1, message_type);
}

std::shared_ptr<UnitreeInterface> UnitreeInterface::CreateH1(const std::string& networkInterface, MessageType message_type) {
    return std::make_shared<UnitreeInterface>(networkInterface, RobotType::H1, message_type);
}

std::shared_ptr<UnitreeInterface> UnitreeInterface::CreateH1_2(const std::string& networkInterface, MessageType message_type) {
    return std::make_shared<UnitreeInterface>(networkInterface, RobotType::H1_2, message_type);
}

std::shared_ptr<UnitreeInterface> UnitreeInterface::CreateGO2(const std::string& networkInterface, MessageType message_type) {
    return std::make_shared<UnitreeInterface>(networkInterface, RobotType::GO2, message_type);
}

std::shared_ptr<UnitreeInterface> UnitreeInterface::CreateCustom(const std::string& networkInterface, int num_motors, MessageType message_type) {
    return std::make_shared<UnitreeInterface>(networkInterface, RobotType::CUSTOM, message_type, num_motors);
}
