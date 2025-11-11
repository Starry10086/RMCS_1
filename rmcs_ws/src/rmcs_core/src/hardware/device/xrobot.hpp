#pragma once

#include <cstdint>
#include <cmath>
#include <eigen3/Eigen/Dense>//聚合头文件
#include <atomic>
#include <rmcs_executor/component.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/imu.hpp>
namespace rmcs_core::hardware::device {

class Xrobot {
public:
    explicit Xrobot(rclcpp::Node& node)
    : eulr_(Eigen::Vector3d::Zero())
    , quat_(Eigen::Quaterniond::Identity())
    , eulr_zero_(Eigen::Vector3d::Zero())
    , quat_zero_(Eigen::Quaterniond::Identity())
    , eulr_zero_flag(false)
    , quat_zero_flag(false){
        imu_publisher_ = node.create_publisher<sensor_msgs::msg::Imu>("/imu/data",10);
    }

    void store_state(uint32_t id,uint64_t raw_can_data)
    {
        if(id == 0x33)
        {
            eulr_raw_can_data_.store(raw_can_data,std::memory_order_relaxed);
        }
        else if(id == 0x34)
        {
            quat_raw_can_data_.store(raw_can_data,std::memory_order_relaxed);
        }
    }
    void update_status()
    {
        const double M_2PI = 2.0 * M_PI;
        CAN_PACK_QUAT can_pack_quat;
        CAN_PACK_EULR can_pack_eulr;
        can_pack_eulr.raw_can_data = eulr_raw_can_data_.load(std::memory_order_relaxed);
        can_pack_quat.raw_can_data = quat_raw_can_data_.load(std::memory_order_relaxed);

        eulr_.y() = static_cast<double>(can_pack_eulr.imu_data[0]) / 32767.0f * M_2PI;//Pitch
        eulr_.x() = static_cast<double>(can_pack_eulr.imu_data[1]) / 32767.0f * M_2PI;//Roll 
        eulr_.z() = static_cast<double>(can_pack_eulr.imu_data[2]) / 32767.0f * M_2PI;//Yaw
        if(eulr_zero_flag == false && (can_pack_eulr.imu_data[0] != 0 || can_pack_eulr.imu_data[1] != 0 || can_pack_eulr.imu_data[2] != 0))
        {
            eulr_zero_ = eulr_;
            eulr_zero_flag = true;
        }
        auto angle_diff = [](double angle_current, double angle_zero)
        {
            double diff = angle_current - angle_zero;
            
            // 添加安全检查，防止死循环
            if (std::abs(diff) > 100.0) {  // 如果差值超过合理范围
                return 0.0;  // 返回 0
            }
            
            while(diff > M_PI)
            {
                diff -= 2.0 * M_PI;
            }
            while(diff < -M_PI)
            {
                diff += 2.0 * M_PI;
            }
            return diff;
        };
        
        //基于上电位置的相对欧拉角，-180~180
        eulr_.x() = angle_diff(eulr_.x(),eulr_zero_.x()) * 180.0 /M_PI;
        eulr_.y() = angle_diff(eulr_.y(), eulr_zero_.y())* 180.0 /M_PI;
        eulr_.z() = angle_diff(eulr_.z(), eulr_zero_.z())* 180.0 /M_PI;

        double q0 = static_cast<double>(can_pack_quat.imu_data[0]) / 32767.0 * 2.0;
        double q1 = static_cast<double>(can_pack_quat.imu_data[1]) / 32767.0 * 2.0;
        double q2 = static_cast<double>(can_pack_quat.imu_data[2]) / 32767.0 * 2.0;
        double q3 = static_cast<double>(can_pack_quat.imu_data[3]) / 32767.0 * 2.0;
        quat_.w() = q0;
        quat_.x() = q2;
        quat_.y() = -q1;
        quat_.z() = q3;
        if(quat_zero_flag == false && (can_pack_quat.imu_data[0] != 0 || can_pack_quat.imu_data[1] != 0 || can_pack_quat.imu_data[2] != 0 || can_pack_quat.imu_data[3] != 0))
        {
            quat_zero_ = quat_;
            quat_zero_flag = true;
        }
        if(quat_zero_flag == true)
        {
            quat_ = quat_zero_.conjugate() * quat_;
        }

        publish_imu();

    }
    void publish_imu(){
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = rclcpp::Clock().now();
        imu_msg.header.frame_id = "imu_link";
        imu_msg.orientation.w = quat_.w();
        imu_msg.orientation.x = quat_.x();
        imu_msg.orientation.y = quat_.y();
        imu_msg.orientation.z = quat_.z();

        imu_msg.orientation_covariance[0] = -1;
        imu_publisher_->publish(imu_msg);
    }
     Eigen::Vector3d& get_eulr()  { return eulr_; }
     Eigen::Quaterniond& get_quat()  { return quat_; }
private:
    union CAN_PACK_QUAT {
        uint64_t raw_can_data;
        int16_t imu_data[4];
    };
    union CAN_PACK_EULR {
        uint64_t raw_can_data;
        int16_t imu_data[4];
    };
    std::atomic<uint64_t> eulr_raw_can_data_{0};
    std::atomic<uint64_t> quat_raw_can_data_{0};

    Eigen::Vector3d eulr_;
    Eigen::Quaterniond quat_;
    Eigen::Vector3d eulr_zero_;
    Eigen::Quaterniond quat_zero_;
    bool eulr_zero_flag;
    bool quat_zero_flag;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};
}





