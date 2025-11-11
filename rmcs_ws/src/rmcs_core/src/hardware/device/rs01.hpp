#pragma once

#include <librmcs/device/rs01.hpp>
#include <rmcs_executor/component.hpp>


namespace rmcs_core::hardware::device{
class Rs01 : public librmcs::device::Rs01{
public:
    Rs01(rmcs_executor::Component & status_component,rmcs_executor::Component & command_component,const std::string &name_prefix)
    :librmcs::device::Rs01(){
        status_component.register_output(name_prefix + "/pos", pos_, 0.0);
        status_component.register_output(name_prefix + "/vel", vel_, 0.0);
        status_component.register_output(name_prefix + "/tor", tor_, 0.0);
        status_component.register_output(name_prefix + "/temp", temp_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);

        command_component.register_input(name_prefix + "/control_pos", control_pos_, false);
        command_component.register_input(name_prefix + "/control_vel", control_vel_, false);
        command_component.register_input(name_prefix + "/control_kp", control_kp_, false);
        command_component.register_input(name_prefix + "/control_kd", control_kd_, false);
        command_component.register_input(name_prefix + "/control_tff", control_tff_, false);

        *max_torque_ = librmcs::device::Rs01::max_torque();
    }

    Rs01(rmcs_executor::Component & status_component,rmcs_executor::Component & command_component,const std::string &name_prefix,const librmcs::device::Rs01::Config &config)
    :Rs01(status_component,command_component,name_prefix){
        librmcs::device::Rs01::configure(config);
        *max_torque_ = librmcs::device::Rs01::max_torque();
    }

    void update_status(){
        librmcs::device::Rs01::update_status();
        *pos_ = librmcs::device::Rs01::get_pos();
        *vel_ = librmcs::device::Rs01::get_vel();
        *tor_ = librmcs::device::Rs01::get_tor();
        *temp_ = librmcs::device::Rs01::get_temp();
    }

    float get_control_pos() const{
        if(control_pos_.ready()) [[likely]]
            return *control_pos_;
        else
            return 0.0;
    }
    float get_control_vel() const{
        if(control_vel_.ready()) [[likely]]
        return *control_vel_;
        else
            return 0.0;
    }
    float get_control_kp() const{
        if(control_kp_.ready()) [[likely]]
            return *control_kp_;
        else
            return 0.0;
    }
    float get_control_kd() const{
        if(control_kd_.ready()) [[likely]]
            return *control_kd_;
        else
            return 0.0;
    }

    float get_control_tff() const{
        if(control_tff_.ready()) [[likely]]
            return *control_tff_;
        else
            return 0.0;
    }

    uint32_t generate_extended_canid(librmcs::device::Rs01::ExtendedMode mode,uint8_t target_id,float data = 0x127){
        if(mode == librmcs::device::Rs01::ExtendedMode::operational_control)
            return librmcs::device::Rs01::generate_extended_can_id(mode,target_id,get_control_tff());
        else
            return librmcs::device::Rs01::generate_extended_can_id(mode,target_id,data);
    }

    uint64_t generate_command(float pos, float vel, float kp, float kd){
        return librmcs::device::Rs01::generate_command(pos, vel, kp, kd);
    }
    
    uint64_t generate_command(){
        return librmcs::device::Rs01::generate_command(get_control_pos(),
                                                       get_control_vel(),
                                                        get_control_kp(),
                                                        get_control_kd());
    }
    
private:
    rmcs_executor::Component::OutputInterface<float> pos_;
    rmcs_executor::Component::OutputInterface<float> vel_;
    rmcs_executor::Component::OutputInterface<float> tor_;
    rmcs_executor::Component::OutputInterface<float> temp_;
    rmcs_executor::Component::OutputInterface<float> max_torque_;

    rmcs_executor::Component::InputInterface<float> control_pos_;
    rmcs_executor::Component::InputInterface<float> control_vel_;
    rmcs_executor::Component::InputInterface<float> control_kp_;
    rmcs_executor::Component::InputInterface<float> control_kd_;
    rmcs_executor::Component::InputInterface<float> control_tff_;
};   
}