#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

// 回调函数，订阅 /km1/arm_joint_controller/follow_joint_trajectory/goal
void trajectoryGoalCallback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg)
{
    ROS_INFO("Received trajectory goal:");
    for (size_t i = 0; i < msg->goal.trajectory.points.size(); ++i)
    {
        const auto& point = msg->goal.trajectory.points[i];
        ROS_INFO("Point %zu:", i);
        std::cout << "  Positions: ";
        for (const auto& pos : point.positions)
            std::cout << pos << " ";
        std::cout << std::endl;

        std::cout << "  Velocities: ";
        for (const auto& vel : point.velocities)
            std::cout << vel << " ";
        std::cout << std::endl;

        std::cout << "  Accelerations: ";
        for (const auto& acc : point.accelerations)
            std::cout << acc << " ";
        std::cout << std::endl;
    }
}

// 回调函数，订阅 /joint_states
// void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
// {
//     ROS_INFO("Received joint states:");
//     std::cout << "  Positions: ";
//     for (const auto& pos : msg->position)
//         std::cout << pos << " ";
//     std::cout << std::endl;

//     std::cout << "  Velocities: ";
//     for (const auto& vel : msg->velocity)
//         std::cout << vel << " ";
//     std::cout << std::endl;

//     std::cout << "  Efforts: ";
//     for (const auto& eff : msg->effort)
//         std::cout << eff << " ";
//     std::cout << std::endl;
// }

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "trajectory_and_state_subscriber");
    ros::NodeHandle nh;

    // 订阅 /km1/arm_joint_controller/follow_joint_trajectory/goal
    ros::Subscriber trajectory_sub = nh.subscribe(
        "/km1/arm_joint_controller/follow_joint_trajectory/goal", 
        10, 
        trajectoryGoalCallback
    );

    // 订阅 /joint_states
    // ros::Subscriber joint_states_sub = nh.subscribe(
    //     "/joint_states", 
    //     10, 
    //     jointStatesCallback
    // );

    // 循环等待回调
    ros::spin();

    return 0;
}
