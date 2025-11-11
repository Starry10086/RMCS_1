#ifndef MY_ARM_HPP
#define MY_ARM_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <map>
#include <cmath>
#include <vector>

#include <thread>                      
#include <atomic>

// ROS2 硬件接口
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <librmcs/client/cboard.hpp> 

//机械臂电机接口
#include <device/go1_motor.hpp>
#include <librmcs/device/dji_motor.hpp>

namespace unitree_hardware
{
    
    class UnitreeHardwareInterface : public hardware_interface::SystemInterface, private librmcs::client::CBoard
    {
    protected:
        void can1_receive_callback
        (
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id   ,
            bool is_remote_transmission , uint8_t can_data_length
        ) override;
        

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(UnitreeHardwareInterface)

            UnitreeHardwareInterface(int32_t usb_pid = -1);
            ~UnitreeHardwareInterface();

            // ====================管理接口====================
            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            // ==================== 接口导出 ====================
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        
            // ==================== 数据读写 ====================
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
   
        private:
            // ==================== 关节配置结构 ====================
            struct JointConfig
            {
                std::string motor_type;     //电机类型
                std::string name;           // 关节名称
                int motor_id;              
                double reduction_ratio;     
                double zero_offset;         // 零位偏移
            
                int can_bus;               //  总线编号
                int can_id;                //  can_id

                // 状态变量
                double position;        
                double velocity;        
                double effort;            
            
                // 命令变量
                double position_cmd;     
                double velocity_cmd;       
                double effort_cmd;        
            
                // 控制参数
                double kp;                  
                double kd;                  
            
                // 仿真模式变量
                bool is_simulated;          // 是否为仿真关节
            };

        // ==================== 成员变量 ====================
        std::vector<JointConfig> joints_;
        std::map<std::string, size_t> joint_name_to_index_;
        
        //电机管理器（静态单例）
        GO1 go1_manager_;
        std::map<int , std::shared_ptr<librmcs::device::DjiMotor>>dji_motors_;

        // CAN通信缓冲区
        librmcs::client::CBoard::TransmitBuffer transmit_buffer_;
        
        // 事件处理线程
        std::thread event_thread_;
        std::atomic<bool> should_stop_{false};

        // 硬件配置参数
        bool use_sim_mode_;             // 是否使用仿真模式
        int32_t usb_pid_;               // 添加：USB设备PID
        
        // 超时控制
        std::map<int, int> motor_timeout_count_;
        static const int MAX_TIMEOUT_COUNT = 5;
        
        // ==================== 辅助函数 ====================
        void update_motor_to_joint(JointConfig& joint);
        void update_joint_to_motor(const JointConfig& joint);
        void update_go1_motor_to_joint(JointConfig& joint);
        void update_dji_motor_to_joint(JointConfig& joint);
        void update_joint_to_go1_motor(const JointConfig& joint);
        void update_joint_to_dji_motor(const JointConfig& joint);
        
    };
}
#endif
