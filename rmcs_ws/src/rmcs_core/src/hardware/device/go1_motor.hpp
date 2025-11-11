#pragma once
#include <unitreeMotor/unitreeMotor.h>
#include <serialPort/SerialPort.h>
#include <vector>
#include <map>

/**
*****************************************************************************
*  @brief   Go1_motor
*
*  @param   只有set_motor一个对外接口
*                   T+Kp*dert(Pos)+Kd*dert(V)
*                   Go1官方公式
*  @param   注意！！！只要改变电机数值，就会直接发送！！！！！！！！！！！！！！！！！
-------------------------------------------
*  @note 历史版本  修改人员      修改日期    修改内容
*  @note v1.0    lihaoyuan   2024.10.9   1.创建
*
*****************************************************************************
*/
class Go1_motor
{
public:
    explicit Go1_motor(int id)
    {
        //构造设置电机类型为GO_M8010_6,模式为FOC,id为输入id
        cmd.motorType = MotorType::GO_M8010_6;
        data.motorType = MotorType::GO_M8010_6;
        cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
        cmd.id   = id;
    }
    ~Go1_motor()
    {//析构设置电机为0位
        cmd.kp   = 0;
        cmd.kd   = 0;
        cmd.q    = 0;
        cmd.dq   = 0;
        cmd.tau  = 0;
        send();
    };
    //改变即发送
    void set_motor(float T,float Kp,float Pos,float Kd,float V)
    {
        cmd.kp   = Kp;
        cmd.kd   = Kd;
        cmd.q    = Pos;
        cmd.dq   = -V*queryGearRatio(MotorType::GO_M8010_6);
        cmd.tau  = T;
        send();        
    }
    const MotorData& get_data()
    {
        return data;
    }
private:
    MotorCmd    cmd;    
    MotorData   data;
    inline static SerialPort serial{"/dev/ttyUSB0"};    
    void send(){serial.sendRecv(&cmd,&data);}//发送命令
};
/**
*****************************************************************************
*  @brief   GO1对外接口
*
*  @param       insert(id)       
*                   加入电机并储存
*  @param       set_motor(int id,float T,float Kp,float Pos,float Kd,float V)
*                   T+Kp*dert(Pos)+Kd*dert(V)
*                   Go1官方公式
*  @param       operator[]          
*                   获取对应id的Go1_motor对象         
*----------------------------------------------------------------------------
*  @note 历史版本  修改人员      修改日期    修改内容
*  @note v1.0    lihaoyuan   2024.10.9   1.创建
*
*****************************************************************************
*/
class GO1
{
public:
    static void set_motor(int id,float T,float Kp,float Pos,float Kd,float V)
    {
        if (motor_map.find(id) == motor_map.end())
            insert(id);
        motor_list[motor_map[id]].set_motor(T, Kp, Pos, Kd, V);
    }
    Go1_motor& operator[](int id)
    {
        if (motor_map.find(id) == motor_map.end())
            insert(id);
        return motor_list[motor_map[id]];
    }
private:
    static void insert(int id)
    {
        motor_list.emplace_back(id);
        motor_map[id] = static_cast<int>(motor_list.size() - 1);
    }
    inline static std::vector<Go1_motor> motor_list;
    inline static std::map<int,int> motor_map;
};




