#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel;

float currentX = 0.0;
float currentY = 0.0;
float currentZ = 0.0;
double roll, pitch, yaw;
std::vector<geometry_msgs::Point> target_positions;
int current_target_index = 0;

double kp_yaw=1;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg) {
    currentX = poseMsg->pose.position.x;
    currentY = poseMsg->pose.position.y;
    currentZ = poseMsg->pose.position.z;
    tf2::Quaternion q;
    tf2::fromMsg(poseMsg->pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//当前位置姿态
    ROS_INFO("Received position: x=%f, y=%f, z=%f", currentX, currentY, currentZ);
    ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
}

void moveToTargetPosition(geometry_msgs::Point& target_pos) {
    ros::Rate rate(30); // 控制循环频率为30Hz

    while (ros::ok()) {
        // 获取当前航向角
        double current_yaw = yaw;

        // 计算到目标位置的向量
        double dx = target_pos.x - currentX;
        double dy = target_pos.y - currentY;
        double dz = target_pos.z - currentZ;

        // 计算距离误差
        double distance = sqrt(dx * dx + dy * dy);
        
        // 计算航向角误差
        double target_yaw = atan2(dy, dx);
        double yaw_error = target_yaw - current_yaw;
            ROS_INFO("=================================");
            ROS_INFO("Moving to next target position");
            
            ROS_INFO("Next target position: x=%f, y=%f, z=%f", target_positions[current_target_index].x, target_positions[current_target_index].y, target_positions[current_target_index].z);
            ROS_INFO("distance=%f, yaw_error=%f", distance, yaw_error);
            ROS_INFO("=================================");
        // 对航向误差进行处理，确保在 -π 到 π 之间
        if (yaw_error > M_PI)
            yaw_error -= 2 * M_PI;
        else if (yaw_error < -M_PI)
            yaw_error += 2 * M_PI;

        // 如果航向角误差较大，则进行航向校准
        if (fabs(yaw_error) > 0.1) { // 设定航向校准阈值

        double linear_speed = 3; // 设置一个线速度值
            // 调整角速度来校准航向
            cmd_vel.angular.z = kp_yaw * yaw_error;
            cmd_vel.linear.z = linear_speed * dz / distance;
        } else {
            // 航向校准完成，开始移动到目标位置
            double linear_speed = 3; // 设置一个线速度值
            cmd_vel.linear.x = linear_speed * distance / distance;
            cmd_vel.linear.z = linear_speed * dz / distance;
        }

            

        // 发布速度控制指令
        cmd_vel_pub.publish(cmd_vel);
            

        // 判断是否到达目标位置，误差小于一定阈值则认为到达
        if (distance < 1) {
            ROS_INFO("Reached target position");

            // 停止当前的运动
            cmd_vel.linear.x = 0.0;
            cmd_vel_pub.publish(cmd_vel);

            // 切换到下一个目标位置
            current_target_index = (current_target_index + 1) % target_positions.size();
            ROS_INFO("Moving to next target position");
            ROS_INFO("=================================");
            ROS_INFO("Next target position: x=%f, y=%f, z=%f", target_positions[current_target_index].x, target_positions[current_target_index].y, target_positions[current_target_index].z);

            // 更新目标位置
            target_pos = target_positions[current_target_index];
            
            // 重新获取当前航向角
            current_yaw = yaw;
        }

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Encircle_node_1");
    ros::NodeHandle nh;

    // 创建发布器
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/xtdrone/typhoon_h480_1/cmd_vel_flu", 1);

    // 创建订阅器
    ros::Subscriber poseSub = nh.subscribe("/typhoon_h480_1/mavros/vision_pose/pose", 1, poseCallback);

    // 设置目标位置列表
    //target_pos1作为出发点
    geometry_msgs::Point target_pos1;
    target_pos1.x = -10.0;
    target_pos1.y = 0.0;
    target_pos1.z = 5.0;
    target_positions.push_back(target_pos1);

    geometry_msgs::Point target_pos2;
    target_pos2.x = -10.0;
    target_pos2.y = 50.0;
  target_pos2.z = 5.0;
    target_positions.push_back(target_pos2);

    geometry_msgs::Point target_pos3;
    target_pos3.x = 110.0;
    target_pos3.y = 50.0;
    target_pos3.z = 5.0;
    target_positions.push_back(target_pos3);

    geometry_msgs::Point target_pos4;
    target_pos4.x = 110.0;
    target_pos4.y = 0.0;
    target_pos4.z = 5.0;
    target_positions.push_back(target_pos4);

    geometry_msgs::Point target_pos5;
    target_pos5.x = -10.0;
    target_pos5.y = 0.0;
    target_pos5.z = 5.0;
    target_positions.push_back(target_pos5);

    // 开始移动到第一个目标位置
    ROS_INFO("Moving to the first target position");
    ROS_INFO("=================================");
    ROS_INFO("Next target position: x=%f, y=%f, z=%f", target_positions[current_target_index].x, target_positions[current_target_index].y, target_positions[current_target_index].z);
    moveToTargetPosition(target_positions[current_target_index]);
    ros::spin();

    return 0;
}