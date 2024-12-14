#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <serial/serial.h>
#include <iostream>

// 串口对象
serial::Serial stm32_serial;

// 初始化串口
void initSerial(const std::string& port, uint32_t baudrate)
{
    try
    {
        stm32_serial.setPort(port);
        stm32_serial.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        stm32_serial.setTimeout(to);
        stm32_serial.open();
        if (stm32_serial.isOpen())
        {
            ROS_INFO("Serial port initialized: %s at %d baud", port.c_str(), baudrate);
        }
        else
        {
            ROS_ERROR("Failed to open serial port!");
        }
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR("Unable to open port: %s", e.what());
    }
}

// 发送关节轨迹到串口
void sendTrajectoryToSTM32(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
    if (!stm32_serial.isOpen())
    {
        ROS_ERROR("Serial port not open. Cannot send data.");
        return;
    }

    ROS_INFO("Sending trajectory to STM32...");
    for (size_t i = 0; i < msg->goal.trajectory.points.size(); ++i)
    {
        const auto& point = msg->goal.trajectory.points[i];

        // 构造要发送的数据包（简单示例，发送关节位置）
        std::string data = "Point " + std::to_string(i) + ": ";
        for (const auto& pos : point.positions)
        {
            data += std::to_string(pos) + " ";
        }
        // data += "\n";

        // 通过串口发送数据
        stm32_serial.write(data);
        ROS_INFO("Sent: %s \n", data.c_str());

        // 模拟发送间隔
        ros::Duration(0.5).sleep();
        // stm32_serial.write("\0");
    }
    uint8_t endMarker = '\0';
    stm32_serial.write(&endMarker, 1);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "trajectory_to_serial");
    ros::NodeHandle nh("~");

    // 获取串口参数
    std::string port;
    int baudrate;
    nh.param<std::string>("port", port, "/dev/ttyUSB1");
    nh.param<int>("baudrate", baudrate, 115200);

    // 初始化串口
    initSerial(port, baudrate);

    // 订阅关节轨迹信息
    ros::Subscriber trajectory_sub = nh.subscribe(
        "/km1/arm_joint_controller/follow_joint_trajectory/goal",
        10,
        sendTrajectoryToSTM32
    );

    // 循环等待回调
    ros::spin();

    // 关闭串口
    if (stm32_serial.isOpen())
    {
        stm32_serial.close();
    }

    return 0;
}
