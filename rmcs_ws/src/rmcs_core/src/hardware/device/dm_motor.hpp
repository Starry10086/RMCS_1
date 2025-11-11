#pragma once

#include <librmcs/device/dm_motor.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device{
class DmMotor : public librmcs::device::DmMotor{
public:
    DmMotor(rmcs_executor::Component &status_component,rmcs_executor::Component &command_component,const std::string &name_prefix)
    :librmcs::device::DmMotor(){
        status_component.register_output(name_prefix + "/pos", pos_, 0.0);
        status_component.register_output(name_prefix + "/vel", vel_, 0.0);
        status_component.register_output(name_prefix + "/tor", tor_, 0.0);
        status_component.register_output(name_prefix + "/T_mos", T_mos_, 0.0);
        status_component.register_output(name_prefix + "/T_rotor", T_rotor_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);
        
        command_component.register_input(name_prefix + "/control_pos", control_pos_, false);
        command_component.register_input(name_prefix + "/control_vel", control_vel_, false);
        command_component.register_input(name_prefix + "/control_Kp", control_Kp_, false);
        command_component.register_input(name_prefix + "/control_Kd", control_Kd_, false);
        command_component.register_input(name_prefix + "/control_tff", control_tff_, false);
    }

    DmMotor(rmcs_executor::Component &status_component,rmcs_executor::Component &command_component,const std::string &name_prefix,const librmcs::device::DmMotor::Config &config)
    :DmMotor(status_component,command_component,name_prefix){
        librmcs::device::DmMotor::configure(config);
        *max_torque_ = max_torque();
    }

    void update_status() {
        librmcs::device::DmMotor::update_status();
        *pos_ = get_pos();
        *vel_ = get_vel();
        *tor_ = get_tor();
        *T_mos_ = get_t_mos();
        *T_rotor_ = get_t_rotor();
    }

    float get_control_pos() const {
        if(control_pos_.ready()) [[likely]]
            return *control_pos_;
        else
            return 0.0;
    }

    float get_control_vel() const {
        if(control_vel_.ready()) [[likely]]
            return *control_vel_;
        else
            return 0.0;
    }

    float get_control_tff() const {
        if(control_tff_.ready()) [[likely]]
            return *control_tff_;
        else
            return 0.0;
    }

    float get_control_Kp() const {
        if(control_Kp_.ready()) [[likely]]
            return *control_Kp_;
        else
            return 0.0;
    }

    float get_control_Kd() const {
        if(control_Kd_.ready()) [[likely]]
            return *control_Kd_;
        else
            return 0.0;
    }
    
    uint64_t generate_command(float pos, float vel, float kp, float kd,float tff){
        return librmcs::device::DmMotor::generate_command(pos, vel, kp, kd, tff);
    }

    uint64_t generate_command() {
        return librmcs::device::DmMotor::generate_command(get_control_pos(), 
                                                          get_control_vel(),
                                                           get_control_Kp(), 
                                                           get_control_Kd(), 
                                                          get_control_tff());
    }

    enum class ControlStatus : uint8_t{ENABLE,DISABLE,Set_Zero};
    static uint64_t set_control_status(ControlStatus control_status) {
        switch(control_status){
            case ControlStatus::ENABLE:
                return librmcs::device::DmMotor::Enable_Motor();
            case ControlStatus::DISABLE:
                return librmcs::device::DmMotor::Disable_Motor();
            case ControlStatus::Set_Zero:
                return librmcs::device::DmMotor::Save_Zero_Position();
            default:
                return 0;
                break;
        }
    }
private:
    
    rmcs_executor::Component::OutputInterface<float> pos_;
    rmcs_executor::Component::OutputInterface<float> vel_;
    rmcs_executor::Component::OutputInterface<float> tor_;
    rmcs_executor::Component::OutputInterface<float> T_mos_;
    rmcs_executor::Component::OutputInterface<float> T_rotor_;
    rmcs_executor::Component::OutputInterface<float> max_torque_;


    rmcs_executor::Component::InputInterface<float> control_pos_;
    rmcs_executor::Component::InputInterface<float> control_vel_;
    rmcs_executor::Component::InputInterface<float> control_Kp_;
    rmcs_executor::Component::InputInterface<float> control_Kd_;
    rmcs_executor::Component::InputInterface<float> control_tff_;
};
}