#include "../Inc/My_arm.hpp"
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace unitree_hardware
{
    UnitreeHardwareInterface::UnitreeHardwareInterface(int32_t usb_pid)
        : librmcs::client::CBoard(64974)
        , transmit_buffer_(*this,256)
    {
            RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
            "UnitreeHardwareInterface 构造完成");
    }
    UnitreeHardwareInterface::~UnitreeHardwareInterface()
    {
        if(event_thread_.joinable())
        {
            should_stop_ = true;
            stop_handling_events();
            event_thread_.join();
        }
        RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "UnitreeHardwareInterface 析构完成");

    }

    // ==================== 初始化 ====================
    hardware_interface::CallbackReturn UnitreeHardwareInterface::on_init(
        const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "正在初始化硬件接口...");

        // 读取硬件参数
        try
        {
            use_sim_mode_ = info_.hardware_parameters.at("use_sim_mode") == "true";
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("UnitreeHardwareInterface"), 
                "未找到 use_sim_mode 参数，默认使用仿真模式");
            use_sim_mode_ = true;
        }

        RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), 
            "运行模式: %s", use_sim_mode_ ? "仿真" : "硬件");

        // 初始化关节配置
        joints_.resize(info_.joints.size());
        
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            const auto& joint_info = info_.joints[i];
            JointConfig& joint = joints_[i];
            
            joint.name = joint_info.name;
            joint_name_to_index_[joint.name] = i;
            
            // 读取关节参数
            try
            {
                joint.motor_id = std::stoi(joint_info.parameters.at("motor_id"));
                joint.reduction_ratio = std::stod(joint_info.parameters.at("reduction_ratio"));
                joint.zero_offset = std::stod(joint_info.parameters.at("zero_offset"));
                joint.kp = std::stod(joint_info.parameters.at("kp"));
                joint.kd = std::stod(joint_info.parameters.at("kd"));
                joint.is_simulated = joint_info.parameters.at("is_simulated") == "true";
                
                // 读取电机类型
                auto motor_type_it = joint_info.parameters.find("motor_type");
                if (motor_type_it != joint_info.parameters.end()) 
                {
                    joint.motor_type = motor_type_it->second;
                } 
                else 
                {
                    joint.motor_type = (joint.name == "joint4") ? "DJI" : "GO1";
                }

                // 如果是DJI电机，走CAN
                if (joint.motor_type == "DJI") 
                {
                    joint.can_bus = std::stoi(joint_info.parameters.at("can_bus"));
                    
                    std::string can_id_str = joint_info.parameters.at("can_id");
                    if (can_id_str.substr(0, 2) == "0x" || can_id_str.substr(0, 2) == "0X") 
                    {
                        joint.can_id = std::stoi(can_id_str, nullptr, 16);
                    } 
                    else 
                    {
                        joint.can_id = std::stoi(can_id_str);
                    }
                }
            }

            catch(const std::exception& e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"),
                    "关节 %s 参数读取失败: %s", joint.name.c_str(), e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            // 初始化状态
            joint.position = 0.0;
            joint.velocity = 0.0;
            joint.effort = 0.0;
            joint.position_cmd = 0.0;
            joint.velocity_cmd = 0.0;
            joint.effort_cmd = 0.0;
            
            RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                "关节 %s: motor_id=%d, ratio=%.2f, offset=%.3f, simulated=%s",
                joint.name.c_str(), joint.motor_id, joint.reduction_ratio, 
                joint.zero_offset, joint.is_simulated ? "是" : "否");
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ==================== 配置 ====================
    hardware_interface::CallbackReturn UnitreeHardwareInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {   
        RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "配置硬件接口");
        
        if (!use_sim_mode_)
        {
            // 初始化硬件电机
            for (const auto& joint : joints_)
            {   
                //初始dji
                if (!joint.is_simulated && joint.motor_type == "DJI") 
                {
                    auto dji_motor = std::make_shared<librmcs::device::DjiMotor>();

                    librmcs::device::DjiMotor::Config config(librmcs::device::DjiMotor::Type::M2006);
                    config.enable_multi_turn_angle();

                    dji_motor->configure(config);

                    dji_motors_[joint.motor_id] = dji_motor;

                    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                        "初始化DJI电机: %s (CAN%d, ID=0x%X)", 
                        joint.name.c_str(), joint.can_bus, joint.can_id);
                }

                else if (!joint.is_simulated && joint.motor_type == "GO1") 
                {
                    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                        "初始化GO1电机: %s (ID=%d)", 
                        joint.name.c_str(), joint.motor_id);
                }
            }
             
            try
            {
                event_thread_ = std::thread([this]()
                {
                    while (!should_stop_) 
                    {
                        handle_events();
                    }
                });
            
                RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                "CAN通信初始化成功");
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("UnitreeHardwareInterface"),
                "CAN通信初始化失败 %s",e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ==================== 激活 ====================
    hardware_interface::CallbackReturn UnitreeHardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "激活硬件接口...");
        
        if (!use_sim_mode_)
        {
            for (auto& joint : joints_)
            {
                if (!joint.is_simulated && joint.motor_id >= 0)
                {
                    // 读取当前真实位置
                    update_motor_to_joint(joint);
                    
                    // 关键：将命令设置为当前位置（不移动）
                    joint.position_cmd = joint.position;
                    joint.velocity_cmd = 0.0;
                    joint.effort_cmd = 0.0;
                    
                    RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"),
                        "关节 %s 上电位置: %.3f rad，保持不动",
                        joint.name.c_str(), joint.position);
                }
            }
        }
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ==================== 停用 ====================
    hardware_interface::CallbackReturn UnitreeHardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("UnitreeHardwareInterface"), "停用硬件接口...");
        
        // 零力矩消息
        if (!use_sim_mode_)
        {
            for (auto& joint : joints_)
            {
                if (!joint.is_simulated && joint.motor_id >= 0)
                {
                    go1_manager_.set_motor(joint.motor_id, 0.0, 0.0, 0.0, 0.0, 0.0);
                }
            }
        }
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // ==================== 导出状态接口 ====================
    std::vector<hardware_interface::StateInterface> UnitreeHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        
        for (auto& joint : joints_)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.name, hardware_interface::HW_IF_POSITION, &joint.position));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort));
        }
        
        return state_interfaces;
    }

    // ==================== 导出命令接口 ====================
    std::vector<hardware_interface::CommandInterface> UnitreeHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        
        for (auto& joint : joints_)
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    joint.name, hardware_interface::HW_IF_POSITION, &joint.position_cmd));
        }
        
        return command_interfaces;
    }

    // ==================== 读取硬件状态 ====================
    hardware_interface::return_type UnitreeHardwareInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (use_sim_mode_)
        {
            // 仿真模式：直接将命令作为状态（假设完美跟踪）
            for (auto& joint : joints_)
            {
                joint.position = joint.position_cmd;
                joint.velocity = 0.0;
                joint.effort = 0.0;
            }
        }
        else
        {
            // 硬件模式：从电机读取真实状态
            for (auto& joint : joints_)
            {
                if (joint.is_simulated)
                {
                    // 仿真关节：使用命令作为状态
                    joint.position = joint.position_cmd;
                    joint.velocity = 0.0;
                    joint.effort = 0.0;
                }
                else if(joint.motor_id >= 0)
                {
                    update_motor_to_joint(joint);
                }
            }
        }
        return hardware_interface::return_type::OK;
    }

    // ==================== 写入硬件命令 ====================
    hardware_interface::return_type UnitreeHardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & )
    {
        if (!use_sim_mode_)
        {
            // 硬件模式：发送命令到电机
            for (const auto& joint : joints_)
            {
                if (!joint.is_simulated && joint.motor_id >= 0)
                {
                    update_joint_to_motor(joint);
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    // ==================== 从电机更新关节状态 ====================
    void UnitreeHardwareInterface::update_motor_to_joint(JointConfig& joint)
    {
        try
        {
            if (joint.motor_type == "DJI")
            {
                update_dji_motor_to_joint(joint);
            }
            else if (joint.motor_type == "GO1")
            {
                update_go1_motor_to_joint(joint);
            }
        }
        catch(const std::exception& e)
        {
            static auto logger = rclcpp::get_logger("UnitreeHardwareInterface");
            static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_ERROR_THROTTLE(
                logger, steady_clock, 1000,
                "读取电机 %d 数据失败: %s", joint.motor_id, e.what());
        }
    }

    // ==================== 从关节命令更新电机 ====================
    void UnitreeHardwareInterface::update_joint_to_motor(const JointConfig& joint)
    {
        try
        {
            if (joint.motor_type == "DJI")
            {
                update_joint_to_dji_motor(joint);
            }
            else if (joint.motor_type == "GO1")
            {
                update_joint_to_go1_motor(joint);
            }
        }
        catch(const std::exception& e)
        {
            static auto logger = rclcpp::get_logger("UnitreeHardwareInterface");
            static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_ERROR_THROTTLE(
                logger, steady_clock, 1000,
                "控制电机 %d 失败: %s", joint.motor_id, e.what());
        }
    }

    // ==================== 从DJI电机更新关节状态 ====================
    void UnitreeHardwareInterface::update_dji_motor_to_joint(JointConfig& joint)
    {
        auto it = dji_motors_.find(joint.motor_id);
        if (it != dji_motors_.end())
        {
            it->second->update_status();
            
            joint.position = it->second->angle() + joint.zero_offset;
            joint.velocity = it->second->velocity();
            joint.effort = it->second->torque();
        }
    }

    // ==================== 从GO1电机更新关节状态 ====================
    void UnitreeHardwareInterface::update_go1_motor_to_joint(JointConfig& joint)
    {
        try
        {
            const MotorData& motor_data = go1_manager_[joint.motor_id].get_data();
            
            if (motor_data.correct) 
            {
                motor_timeout_count_[joint.motor_id] = 0;
                
                joint.position = motor_data.q / joint.reduction_ratio + joint.zero_offset;
                joint.velocity = motor_data.dq / joint.reduction_ratio;
                joint.effort = motor_data.tau * joint.reduction_ratio;
            } 
            else 
            {
                motor_timeout_count_[joint.motor_id]++;
                
                if (motor_timeout_count_[joint.motor_id] >= MAX_TIMEOUT_COUNT) {
                    static auto logger = rclcpp::get_logger("UnitreeHardwareInterface");
                    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
                    RCLCPP_ERROR_THROTTLE(logger, steady_clock, 5000,
                        "GO1电机 %d 连续超时", joint.motor_id);
                }
            }
        }
        catch(const std::exception& e)
        {
            static auto logger = rclcpp::get_logger("UnitreeHardwareInterface");
            static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_ERROR_THROTTLE(logger, steady_clock, 1000,
                "读取GO1电机 %d 失败: %s", joint.motor_id, e.what());
        }
    }

    // ==================== 将GO1关节命令发送到电机 ====================
    void UnitreeHardwareInterface::update_joint_to_go1_motor(const JointConfig& joint)
    {
        try
        {
            double motor_pos = (joint.position_cmd - joint.zero_offset) * joint.reduction_ratio;
            double motor_vel = joint.velocity_cmd * joint.reduction_ratio;
            
            go1_manager_.set_motor(
                joint.motor_id,
                joint.effort_cmd,
                joint.kp,
                motor_pos,
                joint.kd,
                motor_vel
            );
        }
        catch(const std::exception& e)
        {
            static auto logger = rclcpp::get_logger("UnitreeHardwareInterface");
            static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
            RCLCPP_ERROR_THROTTLE(logger, steady_clock, 1000,
                "控制GO1电机 %d 失败: %s", joint.motor_id, e.what());
        }
    }

    // ==================== 将DJI关节命令发送到电机 ====================
    void UnitreeHardwareInterface::update_joint_to_dji_motor(const JointConfig& joint)
    {
        auto it = dji_motors_.find(joint.motor_id);
        if (it != dji_motors_.end())
        {
            // PD控制计算力矩
            double position_error = joint.position_cmd - joint.position;
            double velocity_error = joint.velocity_cmd - joint.velocity;
            double control_torque = joint.kp * position_error + joint.kd * velocity_error + joint.effort_cmd;
            
            // 生成控制命令
            uint16_t control_current = it->second->generate_command(control_torque);
            
            uint16_t can_commands[4] = {0,0,0,0};
            int index = joint.can_id - 0x201;
            can_commands[index] = control_current;            

            uint32_t tx_id = (joint.can_id >= 0x205) ? 0x1ff : 0x200;
            if (joint.can_bus == 1) 
            {
                transmit_buffer_.add_can1_transmission(tx_id, std::bit_cast<uint64_t>(can_commands));
            } 
            else if (joint.can_bus == 2) 
            {
                transmit_buffer_.add_can2_transmission(tx_id, std::bit_cast<uint64_t>(can_commands));
            }
            transmit_buffer_.trigger_transmission();
        }
    }
    // ==================== CAN回调 ====================
    void UnitreeHardwareInterface::can1_receive_callback(
        uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
        bool is_remote_transmission, uint8_t can_data_length)
    {
        //过滤
        if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
            return;
            
        if (can_id >= 0x201 && can_id <= 0x208)
        {
            for (const auto& joint : joints_)
            {
                if (joint.motor_type == "DJI" && static_cast<uint32_t>(joint.can_id) == can_id)
                {
                    auto it = dji_motors_.find(joint.motor_id);
                    if (it != dji_motors_.end()) 
                    {
                        // const std::byte* data_ptr = reinterpret_cast<const std::byte*>(&can_data);
                        it->second->store_status(can_data);
                    }
                    break;
                }
            }
        }
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(unitree_hardware::UnitreeHardwareInterface, hardware_interface::SystemInterface)
