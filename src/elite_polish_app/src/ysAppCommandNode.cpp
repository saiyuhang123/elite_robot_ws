/***
***/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/int32.hpp"
#include <Eigen/Geometry>

using namespace std::chrono_literals;
using std::placeholders::_1;


class ysAppCommand : public rclcpp::Node
{
    public:
        ysAppCommand()
        : Node("ysAppCommand") 
        {
            sendCmd_ = true;
            cmd_ = -1;
            result_=-1;

            //
            result_sub_ = this->create_subscription<std_msgs::msg::Int32>(       
                "/elite_forceapp_cmd_result", 1, std::bind(&ysAppCommand::result_callback, this, _1)); 
           // 
            cmd_publisher = this->create_publisher<std_msgs::msg::Int32>("/elite_forceapp_cmd", 1); 
            // 
            timer_ = this->create_wall_timer(
                50ms, std::bind(&ysAppCommand::timer_callback, this));            
        }

    private:
        void timer_callback()                                                       
        {
            if (sendCmd_ == true) {
                sendCmd_ = false;
                RCLCPP_INFO(this->get_logger(),
                        "YS Robot Force App Command: \r\n"
                        "0: ROBOT GO HOME \r\n"
                        "1: AGV GO HOME \r\n"
                        "2: AGV GO POLISH \r\n"
                        "3: DO CAMERA VISION \r\n"
                        "4: DO FORCE POLISH \r\n"
                        "5: CANCEL POLISH SAFELY \r\n"
                        "51: CLOSE Polish Tool \r\n"
                        "52: OPEN Polish Tool \r\n"
                        "Current Step: %d.\r\n"
                        "Please input next command. ", cmd_);
                cmd_=-1;
                scanf("%d", &cmd_);
                if (cmd_>55 || cmd_<0)
                {
                    RCLCPP_INFO(this->get_logger(),
                        "Wrong  command: %d. ", cmd_);
                } else {
                    std_msgs::msg::Int32 msg;
                    msg.data = cmd_;
                    cmd_publisher->publish(msg);
                    RCLCPP_INFO(this->get_logger(),
                        "publish  command: %d. ", cmd_);
                }
                if (cmd_ == 1 
                // || cmd_==4
                // || cmd_==31
                || cmd_==51|| cmd_==52
                // || cmd_==53|| cmd_==54
                ) {
                    sendCmd_ = true;
                }
            } else {
                // 命令3是“视觉完成后继续自动打磨”的原子流程。103 只是中间
                // 进度，不能提前重新开放输入，否则可能在接触/打磨中插入新命令。
                const bool command_done =
                    (cmd_ == 3 && result_ == 104)
                    || (cmd_ != 3 && result_-100 == cmd_)
                    || result_ == 204 || result_ == 205;
                if (command_done)
                {
                    sendCmd_ = true;
                }

            }
        }


        void result_callback(const std_msgs::msg::Int32 msg) 
        {
            result_ = msg.data;
            RCLCPP_INFO(this->get_logger(),
                "sub the result of command: %d. ", result_);
        }

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr result_sub_;      
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cmd_publisher;  
        rclcpp::TimerBase::SharedPtr timer_;                             // 定时器指针
        int cmd_;
        int result_;
        bool sendCmd_;
                // GO_HOME = 0,
                // DO_CAPTURE = 1,
                // DO_PICK = 2,
                // DO_LEFT_FORCEGUIDE = 3,
                // DO_RIGHT_FORCEGUIDE = 4,
                // DO_PLACE = 5,
                // NOTHING = 99

};

// ROS2节点主入口main函数
int main(int argc, char * argv[])                      
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);                
    
    // 创建ROS2节点对象并进行初始化          
    rclcpp::spin(std::make_shared<ysAppCommand>());   
    
    // 关闭ROS2 C++接口
    rclcpp::shutdown();                                

    return 0;
}
