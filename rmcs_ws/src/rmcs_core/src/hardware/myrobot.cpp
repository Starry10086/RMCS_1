#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <librmcs/client/dm_mc02.hpp>
#include <librmcs/client/gpio_ctrl.hpp>
#include <thread>
#include <hardware/device/xrobot.hpp>
#include <hardware/device/dm_motor.hpp>
#include <hardware/device/rs01.hpp>
#include <hardware/device/go1_motor.hpp>
#include <hardware/device/dji_motor.hpp>
#include <filter/ramp_filter.hpp>
#include <filter/MovingAverageFilter.hpp>
#include <controller/pid/matrix_pid_calculator.hpp>

namespace rmcs_core::hardware {
class Myrobot : public rmcs_executor::Component, public rclcpp::Node {
using GpioCtrl = librmcs::client::GpioCtrl;
using Gpio_Port = librmcs::client::GpioCtrl::Gpio_Port;
using Gpio_Pin = librmcs::client::GpioCtrl::Gpio_Pin;
using Gpio_State = librmcs::client::GpioCtrl::Gpio_State;
using RS01_Motor_ExtendedMode = device::Rs01::ExtendedMode;

public:
    Myrobot()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , command_component_(create_partner_component<MyrobotCommand>(get_component_name() + "_command", *this))
        , arm_ctrl_(create_partner_component<Arm_Ctrl>(get_component_name() + "_arm_ctrl", *this))
        , dm_motor_ctrl_(create_partner_component<DM_Motor_Ctrl>(get_component_name() + "_dm_motor_ctrl", *this))
        , rs01_motor_ctrl_(create_partner_component<RS01_Motor_Ctrl>(get_component_name() + "_rs01_motor_ctrl", *this))
        , chassis_ctrl_(create_partner_component<Chassis_Ctrl>(get_component_name() + "_chassis_ctrl", *this))
        {
            board_ = std::make_unique<DMH7_Board>(*this, *command_component_, static_cast<int>(get_parameter("usb_pid").as_int()));
            dm_motor_ctrl_->enable_dm_motor();
            dm_motor_ctrl_->set_zero_position(0x00);
            rs01_motor_ctrl_->enable_rs01_motor();
            rs01_motor_ctrl_ -> set_zero_position_rs01_motor(1);

            chassis_ctrl_->set_chassis_param_(
                get_parameter("wheel_base").as_double(), 
               get_parameter("track_width").as_double(), 
              get_parameter("wheel_radius").as_double(), 
             get_parameter("wheel_max_vel").as_double(),
            get_parameter("chassis_pid_kp").as_double(),
            get_parameter("chassis_pid_ki").as_double(),
            get_parameter("chassis_pid_kd").as_double(),
              get_parameter("integral_min").as_double(),
              get_parameter("integral_max").as_double(),
         get_parameter("chassis_dead_zone").as_double(),
          get_parameter("wheel_max_torque").as_double(),
              get_parameter("m2006_pid_kp").as_double(),
              get_parameter("m2006_pid_ki").as_double(),
              get_parameter("m2006_pid_kd").as_double(),
           get_parameter("m2006_dead_zone").as_double(),
        get_parameter("m2006_integral_min").as_double(),
        get_parameter("m2006_integral_max").as_double(),
             get_parameter("m2006_max_vel").as_double(),
          get_parameter("m2006_max_torque").as_double()
            );
            arm_ctrl_->set_arm_m2006_param(
                            get_parameter("arm_m2006_pos_kp").as_double(),
                            get_parameter("arm_m2006_pos_ki").as_double(),
                            get_parameter("arm_m2006_pos_kd").as_double(),
                     get_parameter("arm_m2006_pos_dead_zone").as_double(),
                  get_parameter("arm_m2006_pos_integral_min").as_double(),
                  get_parameter("arm_m2006_pos_integral_max").as_double(),
                            get_parameter("arm_m2006_vel_kp").as_double(),
                            get_parameter("arm_m2006_vel_ki").as_double(),
                            get_parameter("arm_m2006_vel_kd").as_double(),
                     get_parameter("arm_m2006_vel_dead_zone").as_double(),
                  get_parameter("arm_m2006_vel_integral_min").as_double(),
                  get_parameter("arm_m2006_vel_integral_max").as_double(),
                           get_parameter("arm_m2006_max_vel").as_double(),
                        get_parameter("arm_m2006_max_torque").as_double()
            );
        }

    ~Myrobot() override = default;


    void update() override 
    {
        board_->update();
    }

    void command_update() 
    {
        board_->command_update();
    }
    
    void publish_state()
    {  
        // auto motor_data = GO1{}[0].get_data();
    }
private:
    class MyrobotCommand : public rmcs_executor::Component {
    public:
        explicit MyrobotCommand(Myrobot& myrobot)
            : myrobot_(myrobot) {}

        void update() override 
        {
            myrobot_.command_update();
        }

    private:
        Myrobot& myrobot_;
    };

    class DM_Motor_Ctrl : public rmcs_executor::Component {
    public:
        explicit DM_Motor_Ctrl(Myrobot& myrobot)
            : myrobot_(myrobot) {
                register_output("/dm_motor/control_pos", control_pos_, false);
                register_output("/dm_motor/control_vel", control_vel_, false);
                register_output("/dm_motor/control_Kp", control_Kp_, false);
                register_output("/dm_motor/control_Kd", control_Kd_, false);
                register_output("/dm_motor/control_tff", control_tff_, false);
            }

        void update() override
        {

        }
        void set_control_pos(uint8_t id,float pos,float vel,float Kp,float Kd,float tff)
        {
            *control_pos_ = pos;
            *control_vel_ = vel;
            *control_Kp_ = Kp;
            *control_Kd_ = Kd;
            *control_tff_ = tff;
            command_ = myrobot_.board_->dm_motor_[id].generate_command(pos, vel, Kp, Kd, tff);
            myrobot_.board_->TransmitBuffer.add_can2_transmission(0x00+id, command_, true);
            myrobot_.board_->TransmitBuffer.trigger_transmission();
        }

        void enable_dm_motor()
        {
            for(size_t i = 0; i < myrobot_.board_->dm_motor_.size(); ++i)
            {
                command_ = device::DmMotor::set_control_status(device::DmMotor::ControlStatus::ENABLE);
                // myrobot_.board_->TransmitBuffer.add_can2_transmission(0x00+i, command_);
            }
            // myrobot_.board_->TransmitBuffer.trigger_transmission();
        }
        void disable_motor(uint8_t id)
        {
            command_ = device::DmMotor::set_control_status(device::DmMotor::ControlStatus::DISABLE);
            myrobot_.board_->TransmitBuffer.add_can2_transmission(0x00+id, command_);
        }
        void set_zero_position(uint8_t id)
        {
            command_ = device::DmMotor::set_control_status(device::DmMotor::ControlStatus::Set_Zero);
            myrobot_.board_->TransmitBuffer.add_can2_transmission(0x00+id, command_);
        }
    private:
        Myrobot& myrobot_;
        uint64_t command_;

        rmcs_executor::Component::OutputInterface<float> control_pos_;
        rmcs_executor::Component::OutputInterface<float> control_vel_;
        rmcs_executor::Component::OutputInterface<float> control_Kp_;
        rmcs_executor::Component::OutputInterface<float> control_Kd_;
        rmcs_executor::Component::OutputInterface<float> control_tff_;
    };
    class RS01_Motor_Ctrl : public rmcs_executor::Component {
    public:
        friend class SBUS_Motor_Ctrl;
        explicit RS01_Motor_Ctrl(Myrobot& myrobot)
            : myrobot_(myrobot) {
                register_output("/rs01_motor/control_pos", control_pos_, false);
                register_output("/rs01_motor/control_vel", control_vel_, false);
                register_output("/rs01_motor/control_Kp", control_Kp_, false);
                register_output("/rs01_motor/control_Kd", control_Kd_, false);
                register_output("/rs01_motor/control_tff", control_tff_, false);
            }

        void update() override
        {

        }
        void set_control_pos(uint8_t id,float pos,float vel,float Kp,float Kd,float tff)
        {
            *control_pos_ = pos;
            *control_vel_ = vel;
            *control_Kp_ = Kp;
            *control_Kd_ = Kd;
            *control_tff_ = tff;
            canid_command_ = myrobot_.board_->rs01_motor_[0].generate_extended_canid(RS01_Motor_ExtendedMode::operational_control, id);
            command_ = myrobot_.board_->rs01_motor_[0].generate_command(pos, vel, Kp, Kd);
            myrobot_.board_->TransmitBuffer.add_can2_transmission(canid_command_, command_, true);
            myrobot_.board_->TransmitBuffer.trigger_transmission();
        }
        void enable_rs01_motor()
        {
            for(size_t i = 0; i < myrobot_.board_->rs01_motor_.size(); ++i) 
            {
                canid_command_ = myrobot_.board_->rs01_motor_[i].generate_extended_canid(RS01_Motor_ExtendedMode::motor_enable, 0x01+i);
                myrobot_.board_->TransmitBuffer.add_can2_transmission(canid_command_, 0, true);  // true = 扩展帧
            }
            myrobot_.board_->TransmitBuffer.trigger_transmission();
        }
        void stop_rs01_motor(uint8_t id)
        {
            canid_command_ = myrobot_.board_->rs01_motor_[id].generate_extended_canid(RS01_Motor_ExtendedMode::motor_stop, 0x01+id);
            myrobot_.board_->TransmitBuffer.add_can2_transmission(canid_command_, 0, true);  
            myrobot_.board_->TransmitBuffer.trigger_transmission();
        }
        void set_zero_position_rs01_motor(uint8_t id)
        {
            canid_command_ = myrobot_.board_->rs01_motor_[id].generate_extended_canid(RS01_Motor_ExtendedMode::set_motor_zero, 0x01+id);
            myrobot_.board_->TransmitBuffer.add_can2_transmission(canid_command_, 1, true);  
            myrobot_.board_->TransmitBuffer.trigger_transmission();
        }
    private:
        Myrobot& myrobot_;
        uint32_t canid_command_;
        uint64_t command_;

        rmcs_executor::Component::OutputInterface<float> control_pos_;
        rmcs_executor::Component::OutputInterface<float> control_vel_;
        rmcs_executor::Component::OutputInterface<float> control_Kp_;
        rmcs_executor::Component::OutputInterface<float> control_Kd_;
        rmcs_executor::Component::OutputInterface<float> control_tff_;
    };

    class Chassis_Ctrl : public rmcs_executor::Component {
    public:
        friend class Myrobot;
        explicit Chassis_Ctrl(Myrobot& myrobot)
            : raw_target_vel_{0.0, 0.0, 0.0}         
            , filtered_target_vel_{0.0, 0.0, 0.0}           
            , wheel_target_vel_{0.0, 0.0, 0.0, 0.0}           
            , wheel_current_vel_{0.0, 0.0, 0.0, 0.0}           
            , filtered_current_wheel_vel_{0.0, 0.0, 0.0, 0.0}
            , wheel_vel_error_{0.0, 0.0, 0.0, 0.0}
            , m2006_raw_target_vel_{0.0, 0.0}
            , m2006_filtered_target_vel_{0.0,0.0}
            , m2006_current_vel_{0.0,0.0}
            , filtered_current_m2006_vel_{0.0,0.0}
            , m2006_vel_error_{0.0,0.0}
            , m2006_pid_calculator_(0,0,0)
            , chassis_pid_calculator_(0.0, 0.0, 0.0)
            
            {
                register_input("/chassis/left_front_wheel/velocity", left_front_wheel_vel_, false);
                register_input("/chassis/left_back_wheel/velocity", left_back_wheel_vel_, false);
                register_input("/chassis/right_back_wheel/velocity", right_back_wheel_vel_, false);
                register_input("/chassis/right_front_wheel/velocity", right_front_wheel_vel_, false);
                
                register_input("/m2006/left_motor/velocity", left_m2006_vel_, false);
                register_input("/m2006/right_motor/velocity", right_m2006_vel_, false);

                register_output("/chassis/left_front_wheel/control_torque", left_front_wheel_torque_, false);
                register_output("/chassis/left_back_wheel/control_torque", left_back_wheel_torque_, false);
                register_output("/chassis/right_back_wheel/control_torque", right_back_wheel_torque_, false);
                register_output("/chassis/right_front_wheel/control_torque", right_front_wheel_torque_, false);

                register_output("/m2006/left_motor/control_torque", left_m2006_torque_, false);
                register_output("/m2006/right_motor/control_torque", right_m2006_torque_, false);
            }

        void update() override
        {
            // RCLCPP_INFO(myrobot_.get_logger(),"1号电机速度，2号电机速度，3号电机速度，4号电机速度: %f, %f, %f, %f", *left_front_wheel_vel_, *right_front_wheel_vel_, *right_back_wheel_vel_, *left_back_wheel_vel_);
            //获取滤波后的底盘目标速度
            filtered_target_vel_[0] = ramp_filter_[0]->update(raw_target_vel_[0]);//vx
            filtered_target_vel_[1] = ramp_filter_[1]->update(raw_target_vel_[1]);//vy
            filtered_target_vel_[2] = ramp_filter_[2]->update(raw_target_vel_[2]);//omega

            m2006_filtered_target_vel_[0] = m2006_ramp_filter_[0]->update(m2006_raw_target_vel_[0]);//左轮M2006 rad/s
            m2006_filtered_target_vel_[1] = m2006_ramp_filter_[1]->update(m2006_raw_target_vel_[1]);//右轮M2006 rad/s
            //计算每个轮子目标的速度
            calculate_wheel_velocities(filtered_target_vel_[0], 
                                       filtered_target_vel_[1], 
                                    filtered_target_vel_[2]);
            //限制每个轮子的速度在最大速度范围内
            for(size_t i = 0; i < 4; ++i){
                wheel_target_vel_[i] = std::clamp(wheel_target_vel_[i], -chassis_config_.wheel_max_vel, chassis_config_.wheel_max_vel);
                if(i<=1)
                m2006_filtered_target_vel_[i] = std::clamp(m2006_filtered_target_vel_[i], -m2006_config_.m2006_max_vel, m2006_config_.m2006_max_vel);
            }
            //获取当前每个轮子的速度,并滤波
            get_wheel_current_vel();
            for(size_t i = 0; i < 4; ++i){
                filtered_current_wheel_vel_[i] = moving_average_filter_[i]->update(wheel_current_vel_[i]);
                if(i<=1)
                filtered_current_m2006_vel_[i] = m2006_moving_average_filter_[i]->update(m2006_current_vel_[i]);
            }

            //计算每个轮子的速度误差
            for(size_t i = 0; i < 4; ++i){
                wheel_vel_error_[i] = wheel_target_vel_[i] - filtered_current_wheel_vel_[i];
                if(i<=1)
                m2006_vel_error_[i] = m2006_filtered_target_vel_[i] - filtered_current_m2006_vel_[i];
            }
            
            for(int i = 0; i < 4; ++i){
                if(std::abs(wheel_vel_error_[i]) < chassis_config_.chassis_dead_zone){
                    wheel_vel_error_[i] = 0.0;  // 清零误差，防止积分累积
                }

                if(i<=1){
                    if(std::abs(m2006_vel_error_[i]) < m2006_config_.m2006_dead_zone){
                        m2006_vel_error_[i] = 0.0;  // 清零误差，防止积分累积
                    }
                }
            }
            //计算每个轮子的扭矩,把数组映射到Eigen::Vector4d,然后计算PID
            Eigen::Vector4d wheel_torque_ = chassis_pid_calculator_.update(Eigen::Map<const Eigen::Vector4d> (wheel_vel_error_.data()));
            Eigen::Vector2d m2006_torque_ = m2006_pid_calculator_.update(Eigen::Map<const Eigen::Vector2d> (m2006_vel_error_.data()));
            apply_deadzone(wheel_vel_error_,wheel_torque_);//死区内扭矩清零
            apply_deadzone(m2006_vel_error_,m2006_torque_);

           for(int i = 0; i < 4; ++i){
                wheel_torque_[i] = std::clamp(wheel_torque_[i], -chassis_config_.wheel_max_torque, chassis_config_.wheel_max_torque);
                if(i<=1)
                m2006_torque_[i] = std::clamp(m2006_torque_[i], -m2006_config_.m2006_max_torque, m2006_config_.m2006_max_torque);
           }
            
           *left_front_wheel_torque_ = wheel_torque_[0];
           *right_front_wheel_torque_ = wheel_torque_[1];
           *right_back_wheel_torque_ = wheel_torque_[2];
           *left_back_wheel_torque_ = wheel_torque_[3];

           *left_m2006_torque_ = m2006_torque_[0];
           *right_m2006_torque_ = m2006_torque_[1];
        }

        void apply_deadzone(const std::array<double,4>& wheel_vel_error_, Eigen::Vector4d& wheel_torque_) const
        {
            for(int i = 0; i < 4; ++i){
                if(std::abs(wheel_vel_error_[i]) < chassis_config_.chassis_dead_zone){
                    wheel_torque_[i] = 0.0;
                }
            }
        }
        void apply_deadzone(const std::array<double,2>& m2006_vel_error_, Eigen::Vector2d& m2006_torque_) const
        {
            for(int i = 0; i < 2; ++i){
                if(std::abs(m2006_vel_error_[i]) < m2006_config_.m2006_dead_zone){
                    m2006_torque_[i] = 0.0;
                }
            }
        }
        
        void set_chassis_param_(double wheel_base, double track_width, double wheel_radius, double wheel_max_vel, 
                                double chassis_pid_kp, double chassis_pid_ki, double chassis_pid_kd,
                                double integral_min, double integral_max, double chassis_dead_zone, double wheel_max_torque,
                                double m2006_pid_kp, double m2006_pid_ki, double m2006_pid_kd,
                                double m2006_dead_zone, double m2006_integral_min, double m2006_integral_max, double m2006_max_vel, double m2006_max_torque)
        {
            chassis_config_ = Chassis_Config(wheel_base, track_width, wheel_radius, wheel_max_vel, 
                                            chassis_pid_kp, chassis_pid_ki, chassis_pid_kd, 
                                            integral_min, integral_max, chassis_dead_zone, wheel_max_torque);
            m2006_config_ = M2006_Config(m2006_pid_kp, m2006_pid_ki, m2006_pid_kd, m2006_dead_zone,
                                         m2006_integral_min, m2006_integral_max, m2006_max_vel, m2006_max_torque);

            chassis_radius = (chassis_config_.wheel_base + chassis_config_.track_width) / 2.0;
            max_linear_vel = wheel_max_vel * chassis_config_.wheel_radius;    //m/s
            max_angular_vel = max_linear_vel / chassis_radius;  //rad/s

            linear_accel = max_linear_vel / 1.0;//1秒内达到最大速度
            angular_accel = max_angular_vel / 1.0;//1秒内达到最大角速度

            //初始化斜坡滤波器，用来滤波目标速度
            ramp_filter_[0] = std::make_unique<filter::RampFilter>(linear_accel, dt); //vx
            ramp_filter_[1] = std::make_unique<filter::RampFilter>(linear_accel, dt); //vy
            ramp_filter_[2] = std::make_unique<filter::RampFilter>(angular_accel, dt);//omega
            
            m2006_ramp_filter_[0] = std::make_unique<filter::RampFilter>(35, dt);
            m2006_ramp_filter_[1] = std::make_unique<filter::RampFilter>(35, dt);

            //初始化移动平均滤波器，用来滤波获取的实际速度
            moving_average_filter_[0] = std::make_unique<filter::MovingAverageFilter<5>>();
            moving_average_filter_[1] = std::make_unique<filter::MovingAverageFilter<5>>();
            moving_average_filter_[2] = std::make_unique<filter::MovingAverageFilter<5>>();
            moving_average_filter_[3] = std::make_unique<filter::MovingAverageFilter<5>>();

            m2006_moving_average_filter_[0] = std::make_unique<filter::MovingAverageFilter<5>>();
            m2006_moving_average_filter_[1] = std::make_unique<filter::MovingAverageFilter<5>>();

            m2006_pid_calculator_ = rmcs_core::controller::pid::MatrixPidCalculator<2>(m2006_pid_kp, m2006_pid_ki, m2006_pid_kd);
            chassis_pid_calculator_ = rmcs_core::controller::pid::MatrixPidCalculator<4>(chassis_config_.chassis_pid_kp, chassis_config_.chassis_pid_ki, chassis_config_.chassis_pid_kd);
        }
        void set_raw_target_vel_(double vx, double vy, double omega)
        {
            raw_target_vel_[0] = vx;
            raw_target_vel_[1] = vy;
            raw_target_vel_[2] = omega;
        }
        void set_m2006_raw_target_vel_(double left_m2006_vel, double right_m2006_vel)
        {
            m2006_raw_target_vel_[0] = left_m2006_vel;
            m2006_raw_target_vel_[1] = right_m2006_vel;
        }
    private:
        struct Chassis_Config{
            Chassis_Config() = default;
            Chassis_Config(double wheel_base, double track_width, double wheel_radius, double wheel_max_vel
                        , double chassis_pid_kp, double chassis_pid_ki, double chassis_pid_kd, double integral_min, double integral_max, double chassis_dead_zone, double wheel_max_torque)
            : wheel_base(wheel_base)
            , track_width(track_width)
            , wheel_radius(wheel_radius)
            , wheel_max_vel(wheel_max_vel)
            , chassis_pid_kp(chassis_pid_kp)
            , chassis_pid_ki(chassis_pid_ki)
            , chassis_pid_kd(chassis_pid_kd)
            , integral_min(integral_min)
            , integral_max(integral_max)
            , chassis_dead_zone(chassis_dead_zone)
            , wheel_max_torque(wheel_max_torque){}
            double wheel_base;
            double track_width;
            double wheel_radius;
            double wheel_max_vel;    
            double chassis_pid_kp;
            double chassis_pid_ki;
            double chassis_pid_kd;
            double integral_min;
            double integral_max;
            double chassis_dead_zone;
            double wheel_max_torque;
        };
        struct M2006_Config{
            M2006_Config() = default;
            M2006_Config(double m2006_pid_kp, double m2006_pid_ki, double m2006_pid_kd,
                         double m2006_dead_zone, double m2006_integral_min, double m2006_integral_max,
                         double m2006_max_vel, double m2006_max_torque)
            : m2006_pid_kp(m2006_pid_kp)
            , m2006_pid_ki(m2006_pid_ki)
            , m2006_pid_kd(m2006_pid_kd)
            , m2006_dead_zone(m2006_dead_zone)
            , m2006_integral_min(m2006_integral_min)
            , m2006_integral_max(m2006_integral_max)
            , m2006_max_vel(m2006_max_vel)
            , m2006_max_torque(m2006_max_torque){}
            double m2006_pid_kp;
            double m2006_pid_ki;
            double m2006_pid_kd;
            double m2006_dead_zone;
            double m2006_integral_min;
            double m2006_integral_max;
            double m2006_max_vel;
            double m2006_max_torque;
        };
        void get_wheel_current_vel()
        {
            wheel_current_vel_[0] = left_front_wheel_vel_.ready() ? *left_front_wheel_vel_ : 0.0;
            wheel_current_vel_[1] = right_front_wheel_vel_.ready() ? *right_front_wheel_vel_ : 0.0;
            wheel_current_vel_[2] = right_back_wheel_vel_.ready() ? *right_back_wheel_vel_ : 0.0;
            wheel_current_vel_[3] = left_back_wheel_vel_.ready() ? *left_back_wheel_vel_ : 0.0;

            m2006_current_vel_[0] = left_m2006_vel_.ready() ? *left_m2006_vel_ : 0.0;
            m2006_current_vel_[1] = right_m2006_vel_.ready() ? *right_m2006_vel_ : 0.0;
        }

        void calculate_wheel_velocities(double vx, double vy, double omega)
        {
            const double inv_wheel_radius_ = 1.0/chassis_config_.wheel_radius; //1/r
            const double R = chassis_radius;
            
            wheel_target_vel_[0] = (vx - vy - omega * R) * inv_wheel_radius_; // 左前
            wheel_target_vel_[1] = (vx + vy + omega * R) * inv_wheel_radius_; // 右前  
            wheel_target_vel_[2] = (vx - vy + omega * R) * inv_wheel_radius_; // 右后
            wheel_target_vel_[3] = (vx + vy - omega * R) * inv_wheel_radius_; // 左后
        }
        static constexpr double dt = 0.001; // 时间步长
        double chassis_radius;
        double max_linear_vel;
        double max_angular_vel;
        double linear_accel;
        double angular_accel;
        rmcs_executor::Component::InputInterface<double> left_front_wheel_vel_;
        rmcs_executor::Component::InputInterface<double> left_back_wheel_vel_;
        rmcs_executor::Component::InputInterface<double> right_back_wheel_vel_;
        rmcs_executor::Component::InputInterface<double> right_front_wheel_vel_;  

        rmcs_executor::Component::InputInterface<double> left_m2006_vel_;
        rmcs_executor::Component::InputInterface<double> right_m2006_vel_;

        rmcs_executor::Component::OutputInterface<double> left_front_wheel_torque_;
        rmcs_executor::Component::OutputInterface<double> left_back_wheel_torque_;
        rmcs_executor::Component::OutputInterface<double> right_back_wheel_torque_;
        rmcs_executor::Component::OutputInterface<double> right_front_wheel_torque_;

        rmcs_executor::Component::OutputInterface<double> left_m2006_torque_;
        rmcs_executor::Component::OutputInterface<double> right_m2006_torque_;

        Chassis_Config chassis_config_;
        M2006_Config m2006_config_;
        std::array<double, 3> raw_target_vel_;
        std::array<double, 3> filtered_target_vel_;
        std::array<double, 4> wheel_target_vel_;
        std::array<double, 4> wheel_current_vel_;
        std::array<double, 4> filtered_current_wheel_vel_;
        std::array<double, 4> wheel_vel_error_;

        std::array<double, 2> m2006_raw_target_vel_;
        std::array<double, 2> m2006_filtered_target_vel_;
        std::array<double, 2> m2006_current_vel_;
        std::array<double, 2> filtered_current_m2006_vel_;
        std::array<double, 2> m2006_vel_error_;

        std::array<std::unique_ptr<filter::RampFilter>, 2> m2006_ramp_filter_;
        std::array<std::unique_ptr<filter::MovingAverageFilter<5>>, 2> m2006_moving_average_filter_;
        rmcs_core::controller::pid::MatrixPidCalculator<2> m2006_pid_calculator_;

        std::array<std::unique_ptr<filter::RampFilter>, 3> ramp_filter_;
        std::array<std::unique_ptr<filter::MovingAverageFilter<5>>, 4> moving_average_filter_;
        rmcs_core::controller::pid::MatrixPidCalculator<4> chassis_pid_calculator_;
        
    };

    class Arm_Ctrl : public rmcs_executor::Component{
    public:
        explicit Arm_Ctrl(Myrobot& myrobot)
            : myrobot_(myrobot)
            , joint_pos_{0.0, 0.0, 0.0, 0.0}
            , joint_vel_{0.0, 0.0, 0.0, 0.0}
            , joint_cmd_{0.0, 0.0, 0.0, 0.0}
            , joint4_pos_pid_calculator_(0, 0, 0)  // 位置环PID
            , joint4_vel_pid_calculator_(0, 0, 0)  // 速度环PID
            {
                register_output("/myrobot/arm/joint1/pos", joint1_pos_,false);
                register_output("/myrobot/arm/joint1/vel", joint1_vel_,false);
                register_output("/myrobot/arm/joint2/pos", joint2_pos_,false);
                register_output("/myrobot/arm/joint2/vel", joint2_vel_,false);
                register_output("/myrobot/arm/joint3/pos", joint3_pos_,false);
                register_output("/myrobot/arm/joint3/vel", joint3_vel_,false);
                register_output("/myrobot/arm/joint4/pos", joint4_pos_,false);
                register_output("/myrobot/arm/joint4/vel", joint4_vel_,false);
                register_output("/arm/joint4/motor/control_torque", joint4_motor_torque_,false);

                register_input("/myrobot/arm/joint1/cmd", joint1_cmd_,false);
                register_input("/myrobot/arm/joint2/cmd", joint2_cmd_,false);
                register_input("/myrobot/arm/joint3/cmd", joint3_cmd_,false);
                register_input("/myrobot/arm/joint4/cmd", joint4_cmd_,false);
            }
            void set_arm_m2006_param(
                    double pos_kp, double pos_ki, double pos_kd, double pos_dead_zone,
                    double pos_integral_min, double pos_integral_max,
                    double vel_kp, double vel_ki, double vel_kd, double vel_dead_zone,
                    double vel_integral_min, double vel_integral_max,
                    double max_vel, double max_torque)
                {
                    arm_m2006_config_ = ArmM2006PIDConfig{
                        pos_kp, pos_ki, pos_kd, pos_dead_zone, pos_integral_min, pos_integral_max,
                        vel_kp, vel_ki, vel_kd, vel_dead_zone, vel_integral_min, vel_integral_max,
                        max_vel, max_torque
                    };
                    
                    // 初始化PID控制器
                    joint4_pos_pid_calculator_ = rmcs_core::controller::pid::MatrixPidCalculator<1>(
                        pos_kp, pos_ki, pos_kd);
                    joint4_vel_pid_calculator_ = rmcs_core::controller::pid::MatrixPidCalculator<1>(
                        vel_kp, vel_ki, vel_kd);
                        
                    // 设置积分限幅
                    joint4_pos_pid_calculator_.integral_min[0] = pos_integral_min;
                    joint4_pos_pid_calculator_.integral_max[0] = pos_integral_max;
                    joint4_vel_pid_calculator_.integral_min[0] = vel_integral_min;
                    joint4_vel_pid_calculator_.integral_max[0] = vel_integral_max;
                }
        struct GO1_Command{
            float T;
            float Kp;
            float Pos;
            float Kd;
            float V;
        };
        const std::array<GO1_Command, 3>& get_go1_command() const{return go1_command_;}
        void update() override {
            read_from_hardware();
            publish_states();
            read_command();
            control_joint();
        }
    private:
        void publish_states(){
            *joint1_pos_ = joint_pos_[0];
            *joint1_vel_ = joint_vel_[0];
            *joint2_pos_ = joint_pos_[1];
            *joint2_vel_ = joint_vel_[1];
            *joint3_pos_ = joint_pos_[2];
            *joint3_vel_ = joint_vel_[2];
            *joint4_pos_ = joint_pos_[3];
            *joint4_vel_ = joint_vel_[3];
        }
        void read_from_hardware(){
            //读取关机1-3电机信息
            for(int i = 0;i<3;i++){
                const auto& data = myrobot_.board_->arm_go1_manager_[i+1].get_data();
                if(data.correct){
                    joint_pos_[i] = data.q / 6.33;
                    joint_vel_[i] = data.dq / 6.33;
                }
            }
            //读取关节4电机信息
            myrobot_.board_->arm_m2006_motor_[0].update_status();
            joint_pos_[3] = myrobot_.board_->arm_m2006_motor_[0].angle();
            joint_vel_[3] = myrobot_.board_->arm_m2006_motor_[0].velocity();
            RCLCPP_INFO(myrobot_.get_logger(), "joint4 pos: %f, vel: %f", joint_pos_[3], joint_vel_[3]);
        }
        void read_command(){
            joint_cmd_[0] = 0;
            joint_cmd_[1] = 0;
            joint_cmd_[2] = 0;
            joint_cmd_[3] = 2;
            // // 从输入读取命令到 joint_cmd_（先检查 ready()）
            // if (joint1_cmd_.ready()) joint_cmd_[0] = *joint1_cmd_;
            // if (joint2_cmd_.ready()) joint_cmd_[1] = *joint2_cmd_;
            // if (joint3_cmd_.ready()) joint_cmd_[2] = *joint3_cmd_;
            // if (joint4_cmd_.ready()) joint_cmd_[3] = *joint4_cmd_;
        }
        void control_joint(){
            for(int i = 0;i<3;i++){
                float Pos = static_cast<float>(joint_cmd_[i] * 6.33);
                go1_command_[i].T = 0.0;
                go1_command_[i].Kp = 0.35;
                go1_command_[i].Pos = Pos;
                go1_command_[i].Kd = 0.2;
                go1_command_[i].V = 0.0;
            }
            joint4_control();
        }
        void joint4_control(){
            joint4_pos_error_ = joint_cmd_[3] - joint_pos_[3];
            if(std::abs(joint4_pos_error_) < arm_m2006_config_.pos_dead_zone){
                joint4_pos_error_ = 0.0; // 清零误差
            }
            Eigen::Matrix<double, 1, 1> pos_error_vec;
            pos_error_vec[0] = joint4_pos_error_;
            Eigen::Matrix<double,1,1> target_vel_vec;
            target_vel_vec = joint4_pos_pid_calculator_.update(pos_error_vec);\
            joint4_target_vel_ = target_vel_vec[0];
            joint4_target_vel_ = std::clamp(joint4_target_vel_, -arm_m2006_config_.max_vel, arm_m2006_config_.max_vel);
            joint4_vel_error_ = joint4_target_vel_ - joint_vel_[3];
            if(std::abs(joint4_vel_error_) < arm_m2006_config_.vel_dead_zone){
                joint4_vel_error_ = 0.0; // 清零误差
            }
            Eigen::Matrix<double,1,1> vel_error_vec;
            vel_error_vec[0] = joint4_vel_error_;
            Eigen::Matrix<double,1,1> motor_torque_vec;
            motor_torque_vec = joint4_vel_pid_calculator_.update(vel_error_vec);
            double motor_torque = motor_torque_vec[0];
            motor_torque = std::clamp(motor_torque, -arm_m2006_config_.max_torque, arm_m2006_config_.max_torque);
            *joint4_motor_torque_ = motor_torque;
        }
        Myrobot& myrobot_;

        std::array<double, 4> joint_pos_;
        std::array<double, 4> joint_vel_;
        std::array<double, 4> joint_cmd_;
        std::array<GO1_Command, 3> go1_command_;
        // 关节4串级PID相关变量
        double joint4_target_vel_;
        double joint4_pos_error_;
        double joint4_vel_error_;
        rmcs_executor::Component::OutputInterface<double> joint1_pos_,joint1_vel_;
        rmcs_executor::Component::OutputInterface<double> joint2_pos_,joint2_vel_;
        rmcs_executor::Component::OutputInterface<double> joint3_pos_,joint3_vel_;
        rmcs_executor::Component::OutputInterface<double> joint4_pos_,joint4_vel_;
        rmcs_executor::Component::OutputInterface<double> joint4_motor_torque_;

        rmcs_executor::Component::InputInterface<double> joint1_cmd_;
        rmcs_executor::Component::InputInterface<double> joint2_cmd_;
        rmcs_executor::Component::InputInterface<double> joint3_cmd_;
        rmcs_executor::Component::InputInterface<double> joint4_cmd_;
        struct ArmM2006PIDConfig {
        double pos_kp, pos_ki, pos_kd, pos_dead_zone;
        double pos_integral_min, pos_integral_max;
        double vel_kp, vel_ki, vel_kd, vel_dead_zone;
        double vel_integral_min, vel_integral_max;
        double max_vel, max_torque;
    };
    ArmM2006PIDConfig arm_m2006_config_;
    rmcs_core::controller::pid::MatrixPidCalculator<1> joint4_pos_pid_calculator_;  // 位置环
    rmcs_core::controller::pid::MatrixPidCalculator<1> joint4_vel_pid_calculator_;  // 速度环
    };

/************************************************************SBUS************************************************************************* */

    class SBUS_Motor_Ctrl {
    public:
        using Gpio_Port = librmcs::client::GpioCtrl::Gpio_Port;
        using Gpio_Pin = librmcs::client::GpioCtrl::Gpio_Pin;
        using Gpio_State = librmcs::client::GpioCtrl::Gpio_State;

        friend class DMH7_Board;
        friend class Myrobot;
        explicit SBUS_Motor_Ctrl(Myrobot& myrobot)
            : myrobot_(myrobot)
            ,remote_last_key_state(0)
            ,sa_last_key_state(0)
            ,sd_last_key_state(0)
            ,sbus_last_key_state(0){}
            
        void dbus_receive_callback(const std::byte* uart_data,
            uint8_t uart_data_length){

        // ========= sbus data printing ==========
        // ROS_INFO("SBUS data received: %d bytes, data hex: ", uart_data_length);
        // std::string hex_output = "Data (hex): ";
        // for (uint8_t j = 0; j < uart_data_length; ++j) {
        //     char hex_str[4];
        //     snprintf(hex_str, sizeof(hex_str), "%02X ", static_cast<uint8_t>(uart_data[j]));
        //     hex_output += hex_str;
        // }
        // ROS_INFO("%s", hex_output.c_str());
        // ========= sbus data printing ==========



        // SBUS 帧长度为 25 字节，以 0x0F 开头，以 0x00 结尾
        if (uart_data_length != 25 || uart_data[0] != std::byte{0x0F} || uart_data[24] != std::byte{0x00}) {
        // 如果不是有效的 SBUS 帧，则忽略
        return;
        }

        const uint8_t* payload = reinterpret_cast<const uint8_t*>(uart_data);

        auto sbus_map_channel = [](uint16_t val, uint16_t min_val, uint16_t mid_val, uint16_t max_val) -> int16_t {
        if (val < min_val || val > max_val) return 0; // 失效保护或超出范围

        // 在中点附近设置一个小的死区，以防止摇杆漂移
        if (std::abs(static_cast<int32_t>(val) - mid_val) < 20) {
        return 0;
        }

        double mapped_val;
        if (val > mid_val) {
        // 将 [mid_val, max_val] 映射到 [0, 360]
        mapped_val = (static_cast<double>(val) - mid_val) * 360.0 / (max_val - mid_val);
        } else { // val < mid_val
        // 将 [min_val, mid_val] 映射到 [-360, 0]
        mapped_val = (static_cast<double>(val) - mid_val) * 360.0 / (mid_val - min_val);
        }

        return static_cast<int16_t>(std::clamp(mapped_val, -360.0, 360.0));
        };

        // 解包通道并将其映射到我们的遥控器数据结构
        // 注意：通道映射（哪个摇杆/开关对应哪个通道）取决于发射机配置。
        // 假设 AETR 映射:
        // SBUS Ch1 (Aileron) -> 右摇杆水平 (ch0)
        // SBUS Ch2 (Elevator) -> 右摇杆垂直 (ch1)
        // SBUS Ch3 (Throttle) -> 左摇杆垂直 (ch2)
        // SBUS Ch4 (Rudder)   -> 左摇杆水平 (ch3)
        uint16_t ch1_raw = (payload[1]       | payload[2] << 8) & 0x07FF;
        uint16_t ch2_raw = (payload[2] >> 3  | payload[3] << 5) & 0x07FF;
        uint16_t ch3_raw = (payload[3] >> 6  | payload[4] << 2 | payload[5] << 10) & 0x07FF;
        uint16_t ch4_raw = (payload[5] >> 1  | payload[6] << 7) & 0x07FF;

        uint16_t ch5_raw  = (payload[6] >> 4  | payload[7] << 4) & 0x07FF;                   // SA toggle
        uint16_t ch6_raw  = (payload[7] >> 7  | payload[8] << 1 | payload[9] << 9) & 0x07FF; // SB 3pos
        uint16_t ch7_raw  = (payload[9] >> 2  | payload[10] << 6) & 0x07FF;                  // SC 3pos
        uint16_t ch8_raw  = (payload[10] >> 5 | payload[11] << 3) & 0x07FF;                  // SD toggle
        uint16_t ch9_raw  = (payload[12]      | payload[13] << 8) & 0x07FF;                  // SE Key
        uint16_t ch10_raw = (payload[13] >> 3 | payload[14] << 5) & 0x07FF;                  // SF wheel

        // RCLCPP_INFO(myrobot_.get_logger(), "SBUS: ch1_raw=%d, ch2_raw=%d, ch3_raw=%d, ch4_raw=%d", ch1_raw, ch2_raw, ch3_raw, ch4_raw); 
        SBUSData_.ch2 = -sbus_map_channel(ch1_raw, 174, 992, 1811); // 右摇杆水平
        SBUSData_.ch3 =  sbus_map_channel(ch2_raw, 174, 992, 1811); // 右摇杆垂直
        // SBUSData_.ch1 = sbus_map_channel(ch3_raw, 174, 992, 1811); // 左摇杆垂直
        SBUSData_.ch0 = sbus_map_channel(ch4_raw, 174, 992, 1811); // 左摇杆水平
        // RCLCPP_INFO(myrobot_.get_logger(), "SBUS: ch0=%d, ch1=%d, ch2=%d, ch3=%d", SBUSData_.ch0, SBUSData_.ch1, SBUSData_.ch2, SBUSData_.ch3); 


        if (ch5_raw < 500) {
        SBUSData_.sa = 0; // SA toggle
        } else if (ch5_raw > 500) {
        SBUSData_.sa = 1;
        } else {
        SBUSData_.sa = 0; 
        }

        if (ch8_raw < 500) {
        SBUSData_.sd = 0; // SD toggle
        } else if (ch8_raw > 500) {
        SBUSData_.sd = 1;
        } else {
        SBUSData_.sd = 0; 
        }

        if (ch6_raw < 500) {
        SBUSData_.sb = 0; // SB 3pos
        } else if (ch6_raw > 1300) {
        SBUSData_.sb = 2;
        } else if (ch6_raw > 500) {
        SBUSData_.sb = 1; 
        }

        if (ch7_raw < 500) {
        SBUSData_.sc = 0; // SC 3pos
        } else if (ch7_raw > 1300) {
        SBUSData_.sc = 2;
        } else if (ch7_raw > 500) {
        SBUSData_.sc = 1; 
        }

        if (ch9_raw < 500) {
        SBUSData_.se = 0; // SE Key
        } else if (ch9_raw > 500) {
        SBUSData_.se = 1;
        } else {
        SBUSData_.se = 0; 
        }

        SBUSData_.sf = ch10_raw; // SF wheel, 174-1811



        // update RemoteData_ with SBUSData_
        SBUSData_.remote = 0; // 0 for sbus remote
        RemoteData_ = SBUSData_; 
        remoteHandler(RemoteData_); // 处理遥控器数据


        // 打印解析
        // ROS_INFO( "[SBUS] ch0: %d, ch1: %d, ch2: %d, ch3: %d, sa: %d, sb: %d, sc: %d, sd: %d, se: %d, sf: %d",
        //           SBUSData_.ch0, SBUSData_.ch1, SBUSData_.ch2, SBUSData_.ch3,
        //           SBUSData_.sa, SBUSData_.sb, SBUSData_.sc, SBUSData_.sd,
        //           SBUSData_.se, SBUSData_.sf);

        // 打印raw 摇杆数据
        // ROS_INFO("[SBUS] ch1_raw: %d, ch2_raw: %d, ch3_raw: %d, ch4_raw: %d",
        //          ch1_raw, ch2_raw, ch3_raw, ch4_raw);

        // ROS_INFO("[SBUS raw] ch1: %d, ch2: %d, ch3: %d, ch4: %d, "
        //          "ch5: %d, ch6: %d, ch7: %d, ch8: %d, "
        //          "ch9: %d, ch10: %d",
        //          ch1_raw, ch2_raw, ch3_raw, ch4_raw,
        //          ch5_raw, ch6_raw, ch7_raw, ch8_raw,
        //          ch9_raw, ch10_raw);
        }
    private:
    
        struct REMOTE_Data_t{

            uint8_t remote; // 0 = SBUS, 1 = NRF24L01
            
            int16_t ch0; //left joystick
            
            int16_t ch1;
            
            int16_t ch2; //right joystick
            
            int16_t ch3;
            
            uint16_t s0; // keys
            
            uint16_t s1; // toggle switch
            
              
            
            uint16_t sa, sd; // toggle switches
            
            uint16_t sb, sc; // 3pos
            
            uint16_t se; // key
            
            uint16_t sf; // wheel
            
            REMOTE_Data_t() :remote(0), ch0(0), ch1(0) ,ch2(0), ch3(0), s0(0), s1(0), sa(0), sd(0), sb(0), sc(0), se(0), sf(0) {}
            
            };
            void remoteHandler(REMOTE_Data_t remoteData){
                // std::cout << "ch2=" << remoteData.ch2 << " ch3=" << remoteData.ch3 << " s0=" << int(remoteData.s0) << " s1=" << int(remoteData.s1) << std::endl;
                // ROS_INFO("handler ");
            
                if (remoteData.remote == 1) {      // 1 for nrf remote
                    if (remoteData.s0 != 0 && remoteData.s0 != remote_last_key_state) {
                        switch (remoteData.s0) {
                            case 1:
                                if (remoteData.s1 == 2){
                                    
                                }
                                else{
                                    
                                }
                                break;
                            case 2:
                                // increaseGivenYaw(-0.1);
                                if (remoteData.s1 == 2){
                                    
                                }
                                else {
                                    
                                }
                                break;
                            case 3:
                                
                                break;
                            case 4:
                                
                                
                                break;
                            case 5:
                              
            
                                break;
                            case 6:
                                
                                break;
                            case 7:
            
                                break;
                            case 8:
                               
                                break;
                            
            
                            default:
                                break;
                        }
                    }
                
                    remote_last_key_state = remoteData.s0;
                } 
                
                else if (remoteData.remote == 0) { // 2 for SBUS remote
                    
                    // ch3: 右摇杆垂直 -> vx（前后移动）
                    // ch2: 右摇杆水平 -> vy（左右移动）
                    double vx = remoteData.ch3 / 360.0 * 2.0;      // 归一化并限速到1m/s
                    double vy = remoteData.ch2 / 360.0 * 2.0;      // 归一化并限速到1m/s
                    double omega = 0.0;  
                    RCLCPP_INFO(myrobot_.get_logger(), "SBUS: ch2=%d, ch3=%d", remoteData.ch2, remoteData.ch3); 
                    // 控制底盘
                    myrobot_.chassis_ctrl_->set_raw_target_vel_(vx, vy, omega);

                    // double left_m2006_vel = remoteData.ch3 / 360.0 * 35.0;
                    // double right_m2006_vel = remoteData.ch2 / 360.0 * 35.0;
                    // myrobot_.chassis_ctrl_->set_m2006_raw_target_vel_(left_m2006_vel, right_m2006_vel);
                    // ================== SE 按键处理 =================
                    if (remoteData.se != remote_last_key_state) {
                        if (remoteData.se == 1) {
                            // SE 刚刚按下
                        } else {
                            // SE 刚刚松开
                        }
                    }
                
                    // SE 按下触发分支
                    if (remoteData.se != remote_last_key_state && remoteData.se == 1) { 
                        switch (remoteData.sc){
                            /* running */
                            case 0:
                                if (remoteData.sb == 0) {                                                     

                                  myrobot_.board_->gpio_ctrl_->multiple_gpio_state()
                                                                .set_gpio_config(Gpio_Port::GPIO_A, Gpio_Pin::Pin_3, Gpio_State::HIGH)
                                                                .set_gpio_config(Gpio_Port::GPIO_E, Gpio_Pin::Pin_3, Gpio_State::LOW)
                                                                .execute();
                                  
                                } else if (remoteData.sb == 1) {
                                    
                                    myrobot_.board_->gpio_ctrl_->multiple_gpio_state()
                                                               .set_gpio_config(Gpio_Port::GPIO_A, Gpio_Pin::Pin_3, Gpio_State::LOW)
                                                               .set_gpio_config(Gpio_Port::GPIO_E, Gpio_Pin::Pin_3, Gpio_State::HIGH)
                                                               .execute();
                                } 
                                else if (remoteData.sb == 2) {
                                    
                                }
                                break;
            
                            /* shooting */
                            case 1:
                                if (remoteData.sb == 0) {
                                    
                                    
                                } else if (remoteData.sb == 1) {
                                   
                                } else if (remoteData.sb == 2) {

                                }
                                   
                                break;
            
                            /* dunking */
                            case 2:
                                if (remoteData.sb == 0) {
                                    
                                } else if (remoteData.sb == 1) {
                                    
                                } else if (remoteData.sb == 2) {
                                   
                                }
                                break;
            
                        }
            
                        
                    } else 
                    // SE held dow
                    if (remoteData.se == remote_last_key_state && remoteData.se == 1) {
                        switch (remoteData.sc){
                            case 0:
                                if (remoteData.sb == 0) {
                                    // ch3: 右摇杆垂直 -> vx（前后移动）
                                    // ch2: 右摇杆水平 -> vy（左右移动）
                                    // double vx = remoteData.ch3 / 360.0 * 2.0;      // 归一化并限速到2m/s
                                    // double vy = remoteData.ch2 / 360.0 * 2.0;      // 归一化并限速到2m/s
                                    // double omega = 0.0;  
                                    // // 控制底盘
                                    // // myrobot_.chassis_ctrl_->set_raw_target_vel_(vx, vy, omega);
                                    // RCLCPP_INFO(myrobot_.get_logger(), "SBUS: vx=%f, vy=%f, omega=%f", vx, vy, omega);
                                } else if (remoteData.sb == 1) {
                                    // ch3: 右摇杆垂直 -> vx（前后移动）
                                    // ch2: 右摇杆水平 -> omega（旋转速度）
                                    // double vx = remoteData.ch3 / 360.0 * 2.0;      // 归一化并限速到2m/s
                                    // double omega = remoteData.ch2 / 360.0 * 2.0;      // 归一化并限速到2m/s
                                    // double vy = 0.0;  
                                    // // 控制底盘
                                    // myrobot_.chassis_ctrl_->set_raw_target_vel_(vx, vy, omega);
                                    // RCLCPP_INFO(myrobot_.get_logger(), "SBUS: vx=%f, vy=%f, omega=%f", vx, vy, omega);
                                } else if (remoteData.sb == 2) {
                                }
                                break;
            
                            case 1:
                                if (remoteData.sb == 0) {
                                } else if (remoteData.sb == 1) {
                                } else if (remoteData.sb == 2) {
                                }
                                break;
            
                            case 2:
                                if (remoteData.sb == 0) {
            
                                } else  {
            
                                } 
                                break;
                        }
                    }
                    remote_last_key_state = remoteData.se;
                    // ================== SE 按键处理 END =================
            
                    // ================== SA 处理 =================
                    if (remoteData.sa != sa_last_key_state) {
                        if (remoteData.sa == 1) {
                            // SA 刚刚按下
                            // AIMLOCK_MODE = true; // 切换 AIMLOCK 模式
                        } else {
                            // SA 刚刚松开
                            // AIMLOCK_MODE = false; // 关闭 AIMLOCK 模式
                        }
                        sa_last_key_state = remoteData.sa; // 更新 SA 按键状态
                    }
                    // ================ SA 处理 END =================
            
                    // ================== SD 处理 =================
                    if (remoteData.sd != sd_last_key_state) {
                        if (remoteData.sd == 1) {
                            // SD 刚刚按下
                            // AIMLOCK_MODE = true; // 切换 AIMLOCK 模式
                        } else {
                            // SD 刚刚松开
                            // AIMLOCK_MODE = false; // 关闭 AIMLOCK 模式
                        }
                        sd_last_key_state = remoteData.sd; // 更新 SD 按键状态
                    }
                } 
                
                else {
                
                }
            
                /* manual arm angle */
                // if (remoteData.s1 == 3){
                //     setArmRealRadian(map<float>(remoteData.ch1,      -360, 360, ARM_INIT_THETA, ARM_MAX_THETA));
                // }
            
            }
            
            Myrobot& myrobot_;
            REMOTE_Data_t SBUSData_;
            REMOTE_Data_t RemoteData_; // finally saved remote data unified
            uint8_t remote_last_key_state;
            uint8_t sa_last_key_state;
            uint8_t sd_last_key_state;
            uint8_t sbus_last_key_state;

            float pos = 0.0;
    };

/************************************************************SBUS************************************************************************* */
 
    class DMH7_Board : private librmcs::client::DMH7Board {
    public:
        friend class Myrobot;
        explicit DMH7_Board(Myrobot& myrobot, MyrobotCommand& myrobot_command, int usb_pid = -1)
            : librmcs::client::DMH7Board(usb_pid) 
            , dm_motor_({{myrobot, myrobot_command, "/dm_motor", device::DmMotor::Config{device::DmMotor::Type::DM_J4310_2EC}}})
            , rs01_motor_({{myrobot, myrobot_command, "/rs01_motor", device::Rs01::Config{}.set_reversed()}})
            , dji_chassis_motor_({
                {myrobot,myrobot_command,"/chassis/left_front_wheel",device::DjiMotor::Config{device::DjiMotor::Type::M3508}},
                {myrobot,myrobot_command,"/chassis/right_front_wheel",device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reversed()},
                {myrobot,myrobot_command,"/chassis/right_back_wheel",device::DjiMotor::Config{device::DjiMotor::Type::M3508}.set_reversed()},
                {myrobot,myrobot_command,"/chassis/left_back_wheel",device::DjiMotor::Config{device::DjiMotor::Type::M3508}}})
            , m2006_motor_({
                {myrobot,myrobot_command,"/m2006/left_motor",device::DjiMotor::Config{device::DjiMotor::Type::M2006}},
                {myrobot,myrobot_command,"/m2006/right_motor",device::DjiMotor::Config{device::DjiMotor::Type::M2006}.set_reversed()}})
            , arm_m2006_motor_({
                {myrobot,myrobot_command,"/arm/joint4/motor",device::DjiMotor::Config{device::DjiMotor::Type::M2006}.enable_multi_turn_angle()}})
            , sbus_motor_ctrl_(std::make_shared<SBUS_Motor_Ctrl>(myrobot))
            , myrobot(myrobot)
            , xrobot_(myrobot)
            , TransmitBuffer(*this, 32)
            , event_thread_([this]() { handle_events(); })
            {
                gpio_ctrl_ = std::make_unique<GpioCtrl>([this](uint32_t can_id, uint64_t can_data)->bool
                {
                    TransmitBuffer.add_can1_transmission(can_id, can_data);
                    return TransmitBuffer.trigger_transmission();
                });
                // 与GO1建立联系
                for(int i=0;i<3;i++)
                    arm_go1_manager_[i+1].set_motor( 0.0, 0.0, 0.0, 0.0, 0.0);

                
            }

        ~DMH7_Board() override
        {
            stop_handling_events();
            event_thread_.join();
        }

        void update() //状态更新
        {
            xrobot_.update_status();
            dm_motor_[0].update_status();
            rs01_motor_[0].update_status();
            for(int i = 0; i < 4; ++i){
                dji_chassis_motor_[i].update_status();
                if(i<=1)
                    m2006_motor_[i].update_status();
            }
            arm_m2006_motor_[0].update_status();
            myrobot.publish_state();
        }
        void command_update() //命令更新
        {
            for(int i = 0; i < 4; ++i){
                chassis_cmd[i] = dji_chassis_motor_[i].generate_command();
                if(i<=1){
                    m2006_cmd[i] = m2006_motor_[i].generate_command();
                }
            }
            const auto& go1_command = myrobot.arm_ctrl_->get_go1_command();
            m2006_cmd[2] = arm_m2006_motor_[0].generate_command();
            m2006_cmd[3] = 0;
            TransmitBuffer.add_can1_transmission(0x200, std::bit_cast<uint64_t>(chassis_cmd));
            TransmitBuffer.add_can1_transmission(0x1ff, std::bit_cast<uint64_t>(m2006_cmd));
            TransmitBuffer.trigger_transmission();

            for(int i = 0;i<3;i++){
                arm_go1_manager_[i+1].set_motor(go1_command[i].T, go1_command[i].Kp, 
                                              go1_command[i].Pos, go1_command[i].Kd, go1_command[i].V);
                                              
            }
        }
       
    private:

        void can1_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override 
        {
            if (is_extended_can_id || is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if(can_id >= 0x201 && can_id <= 0x204)
            {
                size_t index_id = can_id - 0x201;
                dji_chassis_motor_[index_id].store_status(can_data);
            }
            else if(can_id >= 0x205 && can_id <= 0x206)
            {       
                size_t index_id = can_id - 0x205;
                m2006_motor_[index_id].store_status(can_data);
            }
            else if(can_id == 0x207)
            {
                arm_m2006_motor_[0].store_status(can_data);
            }
        }

        void can2_receive_callback(
            uint32_t can_id, uint64_t can_data, bool is_extended_can_id,
            bool is_remote_transmission, uint8_t can_data_length) override 
        {
            if (is_remote_transmission || can_data_length < 8) [[unlikely]]
                return;

            if(is_extended_can_id) {
                uint8_t mode = (can_id >> 24) & 0x1F;           // bits 28-24: 通信类型
                uint8_t reset_mode = (can_id >> 22) & 0x03;     // bits 23-22: RESET模式
                uint8_t error_info = (can_id >> 16) & 0x3F;     // bits 21-16: 故障信息
                uint8_t motor_id = (can_id >> 8) & 0xFF;        // bits 15-8: 电机ID 
                uint8_t host_id = can_id & 0xFF;                // bits 7-0: 主机CAN_ID
            
                // 检查是否是RS01反馈（mode=0x2）
                if(mode == 0x2 && motor_id >= 0x01 && motor_id <= 0x04) {
                    size_t index_id = motor_id - 0x01;
                    rs01_motor_[index_id].store_status(can_data);
                }
            }
        }
        void dbus_receive_callback(const std::byte* uart_data, uint8_t uart_data_length) override {
            // RCLCPP_INFO(myrobot.get_logger(), "UART: uart_data_length=%d", uart_data_length); 
            sbus_motor_ctrl_->dbus_receive_callback(uart_data, uart_data_length);
        }
        std::array<device::DmMotor, 1> dm_motor_;
        std::array<device::Rs01, 1> rs01_motor_;
        std::array<device::DjiMotor, 4> dji_chassis_motor_;
        std::array<device::DjiMotor, 2> m2006_motor_;
        std::array<device::DjiMotor,1> arm_m2006_motor_;// joint4
        GO1 arm_go1_manager_;                           // joint1,2,3
        std::shared_ptr<SBUS_Motor_Ctrl> sbus_motor_ctrl_;
        Myrobot& myrobot;
        device::Xrobot xrobot_;
        librmcs::client::DMH7Board::TransmitBuffer TransmitBuffer;
        std::unique_ptr<GpioCtrl> gpio_ctrl_;
        std::thread event_thread_;

        std::array<uint16_t, 4> chassis_cmd;
        std::array<uint16_t, 4> m2006_cmd;
    };

    std::shared_ptr<MyrobotCommand> command_component_;
    std::unique_ptr<DMH7_Board> board_;
    std::shared_ptr<Arm_Ctrl> arm_ctrl_;
    std::shared_ptr<DM_Motor_Ctrl> dm_motor_ctrl_;
    std::shared_ptr<RS01_Motor_Ctrl> rs01_motor_ctrl_;
    std::shared_ptr<Chassis_Ctrl> chassis_ctrl_;

    
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Myrobot, rmcs_executor::Component)