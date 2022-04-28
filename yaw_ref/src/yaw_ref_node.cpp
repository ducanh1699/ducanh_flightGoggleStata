#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <iostream>

int main(int argc, char** argv){
    ros::init(argc, argv, "yaw_target");
    ros::NodeHandle n;
    ros::Publisher yaw_target = n.advertise<std_msgs::Float32>("/reference/yaw", 10);
    ros::Rate loop_rate(5);
    float yaw_rd = 0.0;
    while (yaw_rd > -1.1)
    {
        std_msgs::Float32 yaw_rad;
        yaw_rad.data = yaw_rd;
        yaw_target.publish(yaw_rad);
        yaw_rd= yaw_rd -0.01;
        ros::spinOnce();
        loop_rate.sleep();
    }
}
