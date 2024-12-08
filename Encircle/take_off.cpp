#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <unistd.h>

// 声明为全局变量，确保在订阅器回调函数中能够访问
ros::Publisher multi_cmd_vel_flu_pub0, multi_cmd_vel_flu_pub1, multi_cmd_vel_flu_pub2, multi_cmd_vel_flu_pub3, multi_cmd_vel_flu_pub4, multi_cmd_vel_flu_pub5;
ros::Publisher multi_cmd_pub0, multi_cmd_pub1, multi_cmd_pub2, multi_cmd_pub3, multi_cmd_pub4, multi_cmd_pub5;

int drones_at_target_height = 0;
bool drone0_at_target = false, drone1_at_target = false, drone2_at_target = false, drone3_at_target = false, drone4_at_target = false, drone5_at_target = false;

void check_all_drones_at_target_height() {
    if (drone0_at_target && drone1_at_target && drone2_at_target && drone3_at_target && drone4_at_target && drone5_at_target) {
        ROS_INFO("All drones have reached the target height. Shutting down the node.");
        ros::shutdown();
    }
}

void poseCallback0(const geometry_msgs::PoseStamped::ConstPtr& poseMsg) {
    float currentHeight = poseMsg->pose.position.z;
    ROS_INFO("Drone 0 position: x=%f, y=%f, z=%f", poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z);

    geometry_msgs::Twist twist;
    twist.linear.z = 0.0; // 默认不上升

    if (currentHeight < 5.0) {
        twist.linear.z = 1; // 继续向上运动，调整此值为需要的上升速度,到达5m时停止上升
    } else if (currentHeight > 5.2) {
        twist.linear.z = -0.3; // 下降到5m
    } else {
        drone0_at_target = true;
        check_all_drones_at_target_height();
    }

    if (multi_cmd_vel_flu_pub0) {
        multi_cmd_vel_flu_pub0.publish(twist);
    } else {
        ROS_ERROR("Invalid velocity publisher!");
    }
}

void poseCallback1(const geometry_msgs::PoseStamped::ConstPtr& poseMsg) {
    float currentHeight = poseMsg->pose.position.z;
    ROS_INFO("Drone 1 position: x=%f, y=%f, z=%f", poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z);

    geometry_msgs::Twist twist;
    twist.linear.z = 0.0;

    if (currentHeight < 5.0) {
        twist.linear.z = 1;
    } else if (currentHeight > 5.2) {
        twist.linear.z = -0.3;
    } else {
        drone1_at_target = true;
        check_all_drones_at_target_height();
    }

    if (multi_cmd_vel_flu_pub1) {
        multi_cmd_vel_flu_pub1.publish(twist);
    } else {
        ROS_ERROR("Invalid velocity publisher!");
    }
}

void poseCallback2(const geometry_msgs::PoseStamped::ConstPtr& poseMsg) {
    float currentHeight = poseMsg->pose.position.z;
    ROS_INFO("Drone 2 position: x=%f, y=%f, z=%f", poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z);

    geometry_msgs::Twist twist;
    twist.linear.z = 0.0;

    if (currentHeight < 5.0) {
        twist.linear.z = 1;
    } else if (currentHeight > 5.2) {
        twist.linear.z = -0.3;
    } else {
        drone2_at_target = true;
        check_all_drones_at_target_height();
    }

    if (multi_cmd_vel_flu_pub2) {
        multi_cmd_vel_flu_pub2.publish(twist);
    } else {
        ROS_ERROR("Invalid velocity publisher!");
    }
}

void poseCallback3(const geometry_msgs::PoseStamped::ConstPtr& poseMsg) {
    float currentHeight = poseMsg->pose.position.z;
    ROS_INFO("Drone 3 position: x=%f, y=%f, z=%f", poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z);

    geometry_msgs::Twist twist;
    twist.linear.z = 0.0;

    if (currentHeight < 5.0) {
        twist.linear.z = 1;
    } else if (currentHeight > 5.2) {
        twist.linear.z = -0.3;
    } else {
        drone3_at_target = true;
        check_all_drones_at_target_height();
    }

    if (multi_cmd_vel_flu_pub3) {
        multi_cmd_vel_flu_pub3.publish(twist);
    } else {
        ROS_ERROR("Invalid velocity publisher!");
    }
}

void poseCallback4(const geometry_msgs::PoseStamped::ConstPtr& poseMsg) {
    float currentHeight = poseMsg->pose.position.z;
    ROS_INFO("Drone 4 position: x=%f, y=%f, z=%f", poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z);

    geometry_msgs::Twist twist;
    twist.linear.z = 0.0;

    if (currentHeight < 5.0) {
        twist.linear.z = 1;
    } else if (currentHeight > 5.2) {
        twist.linear.z = -0.3;
    } else {
        drone4_at_target = true;
        check_all_drones_at_target_height();
    }

    if (multi_cmd_vel_flu_pub4) {
        multi_cmd_vel_flu_pub4.publish(twist);
    } else {
        ROS_ERROR("Invalid velocity publisher!");
    }
}

void poseCallback5(const geometry_msgs::PoseStamped::ConstPtr& poseMsg) {
    float currentHeight = poseMsg->pose.position.z;
    ROS_INFO("Drone 5 position: x=%f, y=%f, z=%f", poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z);

    geometry_msgs::Twist twist;
    twist.linear.z = 0.0;

    if (currentHeight < 5.0) {
        twist.linear.z = 1;
    } else if (currentHeight > 5.2) {
        twist.linear.z = -0.3;
    } else {
        drone5_at_target = true;
        check_all_drones_at_target_height();
    }

    if (multi_cmd_vel_flu_pub5) {
        multi_cmd_vel_flu_pub5.publish(twist);
    } else {
        ROS_ERROR("Invalid velocity publisher!");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "takeoff_node");
    ros::NodeHandle nh;

    // 创建 ROS 发布器
    multi_cmd_vel_flu_pub0 = nh.advertise<geometry_msgs::Twist>("/xtdrone/typhoon_h480_0/cmd_vel_flu", 1);
    multi_cmd_vel_flu_pub1 = nh.advertise<geometry_msgs::Twist>("/xtdrone/typhoon_h480_1/cmd_vel_flu", 1);
    multi_cmd_vel_flu_pub2 = nh.advertise<geometry_msgs::Twist>("/xtdrone/typhoon_h480_2/cmd_vel_flu", 1);
    multi_cmd_vel_flu_pub3 = nh.advertise<geometry_msgs::Twist>("/xtdrone/typhoon_h480_3/cmd_vel_flu", 1);
    multi_cmd_vel_flu_pub4 = nh.advertise<geometry_msgs::Twist>("/xtdrone/typhoon_h480_4/cmd_vel_flu", 1);
    multi_cmd_vel_flu_pub5 = nh.advertise<geometry_msgs::Twist>("/xtdrone/typhoon_h480_5/cmd_vel_flu", 1);

    multi_cmd_pub0 = nh.advertise<std_msgs::String>("/xtdrone/typhoon_h480_0/cmd", 3);
    multi_cmd_pub1 = nh.advertise<std_msgs::String>("/xtdrone/typhoon_h480_1/cmd", 3);
    multi_cmd_pub2 = nh.advertise<std_msgs::String>("/xtdrone/typhoon_h480_2/cmd", 3);
    multi_cmd_pub3 = nh.advertise<std_msgs::String>("/xtdrone/typhoon_h480_3/cmd", 3);
    multi_cmd_pub4 = nh.advertise<std_msgs::String>("/xtdrone/typhoon_h480_4/cmd", 3);
    multi_cmd_pub5 = nh.advertise<std_msgs::String>("/xtdrone/typhoon_h480_5/cmd", 3);

    std_msgs::String cmd1, cmd2;
    cmd1.data = "OFFBOARD";
    cmd2.data = "ARM";

    ros::Subscriber poseSub0 = nh.subscribe("/typhoon_h480_0/mavros/vision_pose/pose", 1, poseCallback0);
    ros::Subscriber poseSub1 = nh.subscribe("/typhoon_h480_0/mavros/vision_pose/pose", 1, poseCallback1);
    ros::Subscriber poseSub2 = nh.subscribe("/typhoon_h480_0/mavros/vision_pose/pose", 1, poseCallback2);
    ros::Subscriber poseSub3 = nh.subscribe("/typhoon_h480_0/mavros/vision_pose/pose", 1, poseCallback3);
    ros::Subscriber poseSub4 = nh.subscribe("/typhoon_h480_0/mavros/vision_pose/pose", 1, poseCallback4);
    ros::Subscriber poseSub5 = nh.subscribe("/typhoon_h480_0/mavros/vision_pose/pose", 1, poseCallback5);

    ros::Rate rate(30);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
