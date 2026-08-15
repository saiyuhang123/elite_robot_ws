#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "service_interfaces/srv/set_id.h"
#include "service_interfaces/srv/set_id.hpp"
#include "service_interfaces/srv/setopenlimit.h"
#include "service_interfaces/srv/setopenlimit.hpp"
#include "service_interfaces/srv/setclearerror.h"
#include "service_interfaces/srv/setclearerror.hpp"
#include "service_interfaces/srv/setmovetgt.h"
#include "service_interfaces/srv/setmovetgt.hpp"
#include "service_interfaces/srv/setmovemax.h"
#include "service_interfaces/srv/setmovemax.hpp"
#include "service_interfaces/srv/setmovemin.h"
#include "service_interfaces/srv/setmovemin.hpp"
#include "service_interfaces/srv/setmoveminhold.h"
#include "service_interfaces/srv/setmoveminhold.hpp"
#include "service_interfaces/srv/setestop.h"
#include "service_interfaces/srv/setestop.hpp"
#include "service_interfaces/srv/setparam.h"
#include "service_interfaces/srv/setparam.hpp"
#include "service_interfaces/srv/getopenlimit.h"
#include "service_interfaces/srv/getopenlimit.hpp"
#include "service_interfaces/srv/getcopen.h"
#include "service_interfaces/srv/getcopen.hpp"
#include "service_interfaces/srv/getstatus.h"
#include "service_interfaces/srv/getstatus.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
unsigned char send_buffer[64] = {0};
unsigned char recv_buffer[64] = {0};
serial::Serial ros_ser;//定义串口

class Gripper_control : public rclcpp::Node 
{
public:
    Gripper_control() : Node("Gripper_control")
    {
        RCLCPP_INFO(this->get_logger(), "实例化成功");
        // 实例化回调组, 作用为避免死锁(请自行百度ROS2死锁)
        //callback_group_Hand_control = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        //callback_group_getangleact = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // 实例化服务
        SetID_Server = this->create_service<service_interfaces::srv::SetID>("SetID",
                                    std::bind(&Gripper_control::setID_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setID);
        Setopenlimit_Server = this->create_service<service_interfaces::srv::Setopenlimit>("Setopenlimit",
                                    std::bind(&Gripper_control::setopenlimit_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setopenlimit);
        Setclearerror_Server = this->create_service<service_interfaces::srv::Setclearerror>("Setclearerror",
                                    std::bind(&Gripper_control::setclearerror_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setclearerror);
        Setmovetgt_Server = this->create_service<service_interfaces::srv::Setmovetgt>("Setmovetgt",
                                    std::bind(&Gripper_control::setmovetgt_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setmovetgt);
        Setmovemax_Server = this->create_service<service_interfaces::srv::Setmovemax>("Setmovemax",
                                    std::bind(&Gripper_control::setmovemax_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setmovemax);
        Setmovemin_Server = this->create_service<service_interfaces::srv::Setmovemin>("Setmovemin",
                                    std::bind(&Gripper_control::setmovemin_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setmovemin);
        Setmoveminhold_Server = this->create_service<service_interfaces::srv::Setmoveminhold>("Setmoveminhold",
                                    std::bind(&Gripper_control::setmoveminhold_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setmoveminhold);
        Setestop_Server = this->create_service<service_interfaces::srv::Setestop>("Setestop",
                                    std::bind(&Gripper_control::setestop_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setestop);
        Setparam_Server = this->create_service<service_interfaces::srv::Setparam>("Setparam",
                                    std::bind(&Gripper_control::setparam_callback,this,_1,_2),
                                    rmw_qos_profile_services_default,
                                    callback_group_setparam);
        Getopenlimit_Server = this->create_service<service_interfaces::srv::Getopenlimit>("Getopenlimit",
                                    std::bind(&Gripper_control::getopenlimit_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getopenlimit);
        Getcopen_Server = this->create_service<service_interfaces::srv::Getcopen>("Getcopen",
                                    std::bind(&Gripper_control::getcopen_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getcopen);   
        Getstatus_Server = this->create_service<service_interfaces::srv::Getstatus>("Getstatus",
                                    std::bind(&Gripper_control::getstatus_callback, this, _1, _2),
                                    rmw_qos_profile_services_default,
                                    callback_group_getstatus);                                                                                                                                                                                     
    }
private:
    // 声明服务回调组
    rclcpp::CallbackGroup::SharedPtr callback_group_setID;
    rclcpp::CallbackGroup::SharedPtr callback_group_setopenlimit;
    rclcpp::CallbackGroup::SharedPtr callback_group_setclearerror;
    rclcpp::CallbackGroup::SharedPtr callback_group_setmovetgt;
    rclcpp::CallbackGroup::SharedPtr callback_group_setmovemax;
    rclcpp::CallbackGroup::SharedPtr callback_group_setmovemin;
    rclcpp::CallbackGroup::SharedPtr callback_group_setmoveminhold;
    rclcpp::CallbackGroup::SharedPtr callback_group_setestop;
    rclcpp::CallbackGroup::SharedPtr callback_group_setparam;
    rclcpp::CallbackGroup::SharedPtr callback_group_getopenlimit;
    rclcpp::CallbackGroup::SharedPtr callback_group_getcopen;
    rclcpp::CallbackGroup::SharedPtr callback_group_getstatus;

    // 声明服务端
    rclcpp::Service<service_interfaces::srv::SetID>::SharedPtr SetID_Server;
    rclcpp::Service<service_interfaces::srv::Setopenlimit>::SharedPtr Setopenlimit_Server;
    rclcpp::Service<service_interfaces::srv::Setclearerror>::SharedPtr Setclearerror_Server;
    rclcpp::Service<service_interfaces::srv::Setmovetgt>::SharedPtr Setmovetgt_Server;
    rclcpp::Service<service_interfaces::srv::Setmovemax>::SharedPtr Setmovemax_Server;
    rclcpp::Service<service_interfaces::srv::Setmovemin>::SharedPtr Setmovemin_Server;
    rclcpp::Service<service_interfaces::srv::Setmoveminhold>::SharedPtr Setmoveminhold_Server;
    rclcpp::Service<service_interfaces::srv::Setestop>::SharedPtr Setestop_Server;
    rclcpp::Service<service_interfaces::srv::Setparam>::SharedPtr Setparam_Server;
    rclcpp::Service<service_interfaces::srv::Getopenlimit>::SharedPtr Getopenlimit_Server;
    rclcpp::Service<service_interfaces::srv::Getcopen>::SharedPtr Getcopen_Server;
    rclcpp::Service<service_interfaces::srv::Getstatus>::SharedPtr Getstatus_Server;
    // 声明回调函数，当收到要请求时调用该函数
    //设置夹爪ID
    void setID_callback(const service_interfaces::srv::SetID::Request::SharedPtr request,
                               const service_interfaces::srv::SetID::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_id")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x02;
            send_buffer[4] = 0x04;
            send_buffer[5] = request->gripper_setid;

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[6] = (check_sum & 0xFF);
            ros_ser.write(send_buffer,7);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[5] == 0x01)
                {
                    response->id_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->id_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->id_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    //设置开口限位（最大开口度和最小开口度）
    void setopenlimit_callback(const service_interfaces::srv::Setopenlimit::Request::SharedPtr request,
                               const service_interfaces::srv::Setopenlimit::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_openlimit")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x05;
            send_buffer[4] = 0x12;

            unsigned int temp_int1,temp_int2;
            temp_int1 = (unsigned int)request->openmax;
            temp_int2 = (unsigned int)request->openmin;

            send_buffer[5] = (temp_int1 & 0xFF);
            send_buffer[6] = ((temp_int1 >> 8) & 0xFF);
            send_buffer[7] = (temp_int2 & 0xFF);
            send_buffer[8] = ((temp_int2 >> 8) & 0xFF);

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[9] = (check_sum & 0xFF);
            ros_ser.write(send_buffer,10);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[5] == 0x01)
                {
                    response->openlimit_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->openlimit_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->openlimit_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    //清除故障
    void setclearerror_callback(const service_interfaces::srv::Setclearerror::Request::SharedPtr request,
                               const service_interfaces::srv::Setclearerror::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_clearerror")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x01;
            send_buffer[4] = 0x17;

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[5] = (check_sum & 0xFF);
            ros_ser.write(send_buffer,6);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[5] == 0x01)
                {
                    response->clearerror_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->clearerror_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->clearerror_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    //指定夹爪开口度
    void setmovetgt_callback(const service_interfaces::srv::Setmovetgt::Request::SharedPtr request,
                               const service_interfaces::srv::Setmovetgt::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_move_tgt")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x03;
            send_buffer[4] = 0x54;

            unsigned int temp_int1;
            temp_int1 = (unsigned int)request->movetgt;

            send_buffer[5] = (temp_int1 & 0xFF);
            send_buffer[6] = ((temp_int1 >> 8) & 0xFF);

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[7] = (check_sum & 0xFF);
            ros_ser.write(send_buffer,8);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[5] == 0x01)
                {
                    response->movetgt_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->movetgt_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->movetgt_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    //以设置的速度松开
    void setmovemax_callback(const service_interfaces::srv::Setmovemax::Request::SharedPtr request,
                               const service_interfaces::srv::Setmovemax::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_movemax")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x03;
            send_buffer[4] = 0x11;

            unsigned int temp_int1;
            temp_int1 = (unsigned int)request->speed;

            send_buffer[5] = (temp_int1 & 0xFF);
            send_buffer[6] = ((temp_int1 >> 8) & 0xFF);

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[7] = (check_sum & 0xFF);
            ros_ser.write(send_buffer,8);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[5] == 0x01)
                {
                    response->movemax_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->movemax_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->movemax_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    //以设置的速度和力控阈值夹取
    void setmovemin_callback(const service_interfaces::srv::Setmovemin::Request::SharedPtr request,
                               const service_interfaces::srv::Setmovemin::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_movemin")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x05;
            send_buffer[4] = 0x10;

            unsigned int temp_int1,temp_int2;
            temp_int1 = (unsigned int)request->speed;
            temp_int2 = (unsigned int)request->power;

            send_buffer[5] = (temp_int1 & 0xFF);
            send_buffer[6] = ((temp_int1 >> 8) & 0xFF);
            send_buffer[7] = (temp_int2 & 0xFF);
            send_buffer[8] = ((temp_int2 >> 8) & 0xFF);

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[9] = (check_sum & 0xFF);
            ros_ser.write(send_buffer,10);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[5] == 0x01)
                {
                    response->movemin_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->movemin_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->movemin_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    //以设置的速度和力控阈值持续夹取
    void setmoveminhold_callback(const service_interfaces::srv::Setmoveminhold::Request::SharedPtr request,
                               const service_interfaces::srv::Setmoveminhold::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_moveminhold")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x05;
            send_buffer[4] = 0x18;

            unsigned int temp_int1,temp_int2;
            temp_int1 = (unsigned int)request->speed;
            temp_int2 = (unsigned int)request->power;

            send_buffer[5] = (temp_int1 & 0xFF);
            send_buffer[6] = ((temp_int1 >> 8) & 0xFF);
            send_buffer[7] = (temp_int2 & 0xFF);
            send_buffer[8] = ((temp_int2 >> 8) & 0xFF);

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[9] = (check_sum & 0xFF);
            ros_ser.write(send_buffer,10);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[5] == 0x01)
                {
                    response->moveminhold_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->moveminhold_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->moveminhold_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    //急停
    void setestop_callback(const service_interfaces::srv::Setestop::Request::SharedPtr request,
                               const service_interfaces::srv::Setestop::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_estop")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x01;
            send_buffer[4] = 0x16;

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[5] = (check_sum & 0xFF);
            ros_ser.write(send_buffer,6);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[5] == 0x01)
                {
                    response->estop_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->estop_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->estop_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    //参数固化
    void setparam_callback(const service_interfaces::srv::Setparam::Request::SharedPtr request,
                               const service_interfaces::srv::Setparam::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "set_param")
        {
            // 打印指令类型
            RCLCPP_INFO(this->get_logger(), "收到%s的请求", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x01;
            send_buffer[4] = 0x01;

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[5] = (check_sum & 0xFF);
            ros_ser.write(send_buffer,7);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[5] == 0x01)
                {
                    response->param_accepted = true;
                    printf("设置指令成功\n");
                }
                else
                {
                    response->param_accepted = false;
                    printf("设置指令失败\n");
                }
            }
        }
        else
        {
            //设置指令报错
            response->param_accepted = false;
            RCLCPP_INFO(this->get_logger(), "收到一个错误请求:%s", request->status.c_str());
        }
    }
    //读取夹爪开口限位（最大开口度和最小开口度）
    void getopenlimit_callback(const service_interfaces::srv::Getopenlimit::Request::SharedPtr request,
                               const service_interfaces::srv::Getopenlimit::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_openlimit")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x01;
            send_buffer[4] = 0x13;

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[5] = check_sum;
            ros_ser.write(send_buffer,6);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x13)
                {
                    response->openmax = (recv_buffer[5] & 0xFF) + ((recv_buffer[6]<<8) & 0xFF00);
                    response->openmin = (recv_buffer[7] & 0xFF) + ((recv_buffer[8]<<8) & 0xFF00);
                    
                    RCLCPP_INFO(this->get_logger(), "夹爪最大开口限度为:%d 最小开口限度为:%d", 
                    response->openmax,response->openmin);
                }
                else
                {
                    printf("无法读取夹爪开口限度\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
    //读取夹爪当前开口度
    void getcopen_callback(const service_interfaces::srv::Getcopen::Request::SharedPtr request,
                               const service_interfaces::srv::Getcopen::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_copen")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x01;
            send_buffer[4] = 0xD9;

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[5] = check_sum;
            ros_ser.write(send_buffer,6);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0xD9)
                {
                    int temp = recv_buffer[5] + ((recv_buffer[6]<<8) & 0xFF00);
                    if(temp > 1000 && temp < 2000)
                    {
                       temp = 1000;
                    }
                    if(temp > 30000)
                    {
                       temp = 0;
                    }
                    response->copen = (float)(temp);
                    
                    RCLCPP_INFO(this->get_logger(), "夹爪当前开口度为:%d", 
                    response->copen);
                }
                else
                {
                    printf("无法读取夹爪当前开口度\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
    //读取夹爪运行状态
    void getstatus_callback(const service_interfaces::srv::Getstatus::Request::SharedPtr request,
                               const service_interfaces::srv::Getstatus::Response::SharedPtr response)
    {
        u_int8_t check_sum = 0;
        rclcpp::WallRate loop_rate(10.0);
        // 首先判断指令类型
        if(request->status == "get_status")
        {
            // 打印请求
            RCLCPP_INFO(this->get_logger(), "收到一个来自%s的指令", request->status.c_str());
            // 传递数据到数组
            send_buffer[0] = 0xEB;
            send_buffer[1] = 0x90;
            send_buffer[2] = request->gripper_id;
            send_buffer[3] = 0x01;
            send_buffer[4] = 0x41;

            int len = send_buffer[3]+5;
            for(int i = 2;i < len - 1;i++)
            {
                check_sum += send_buffer[i];
            }
            send_buffer[5] = check_sum;
            ros_ser.write(send_buffer,6);
            loop_rate.sleep();    //等待100ms接收数据
            int count = ros_ser.available(); // count读取到缓存区数据的字节数，不等于0说明缓存里面有数据可以读取
            if (count != 0)  //等待接收数据
            {   
                std::vector<unsigned char> recv_buffer(count);//开辟数据缓冲区，串口read读出的内容是无符号char类型
                count = ros_ser.read(&recv_buffer[0], count); // 读出缓存区缓存的数据，返回值为读到的数据字节数
                if(recv_buffer[4] == 0x41)
                {
                    response->status = (recv_buffer[5]);
                    response->error = (recv_buffer[6]);
                    response->temp = (recv_buffer[7]);
                    RCLCPP_INFO(this->get_logger(), "夹爪当前状态码为:%d 故障码为:%d 温度为:%d", 
                    response->status,response->error,response->temp);
                }
                else
                {
                    printf("无法读取夹爪当前状态\n");
                } 
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "收到一个非法请求,%s", request->status.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //rclcpp::WallRate loop_rate(10.0);
    ros_ser.setPort("/dev/ttyGripper");
    ros_ser.setBaudrate(115200);
    serial::Timeout to =serial::Timeout::simpleTimeout(100);
    ros_ser.setTimeout(to);

    // 开机/插拔后 USB 串口可能尚未就绪（udev 别名未建好/设备未枚举完），
    // 单次 open 失败会直接退出导致服务永不出现。这里改为最多重试 30 次，每次间隔 1s。
    const int MAX_OPEN_RETRY = 30;
    bool opened = false;
    for (int i = 1; i <= MAX_OPEN_RETRY; i++)
    {
        try
        {
            if (!ros_ser.isOpen())
            {
                ros_ser.open();
            }
        }
        catch (const std::exception &e)
        {
            // 端口未就绪，稍后重试
        }
        if (ros_ser.isOpen())
        {
            opened = true;
            break;
        }
        std::cout << "[gripper] 串口未就绪，1s 后重试 (" << i << "/"
                  << MAX_OPEN_RETRY << ")" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (opened)
    {
        std::cout<<"serial open success"<<std::endl;
    }
    else
    {
        std::cout<<"serial unable to open after "<<MAX_OPEN_RETRY
                 <<" retries"<<std::endl;
        return -1;
    }
    auto node = std::make_shared<Gripper_control>();
    // 把节点的执行器变成多线程执行器, 避免死锁
    rclcpp::executors::MultiThreadedExecutor exector;
    exector.add_node(node);
    exector.spin();
    rclcpp::shutdown();
    return 0;
}
