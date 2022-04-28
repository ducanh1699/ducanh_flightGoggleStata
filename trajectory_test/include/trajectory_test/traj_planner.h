#ifndef TRAJECTORY_TEST_TRAJECTORY_PLANNER_H
#define TRAJECTORY_TEST_TRAJECTORY_PLANNER_H

#include <ros/ros.h>
#include "mav_planning_msgs/PolynomialTrajectory.h"
#include "mav_trajectory_generation/polynomial_optimization_linear.h"
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include <vector>

class traj_planner
{
private:
    ros::Publisher pub_marker_;
    ros::Publisher pub_trajectory_;
    ros::Subscriber sub_odom_;

    ros::NodeHandle& nh_;
    ros::NodeHandle& nh_private_;

    Eigen::Affine3d current_pose_; // is a Pose type message (contrains a Vector 3d and Quaternion/RotationMaxtrix)
    Eigen::Vector3d current_vel_;
    Eigen::Vector3d current_angular_vel_;

    double max_v_;
    double max_a_;
    double max_ang_v_;
    double max_ang_a_;

public:
    traj_planner(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~traj_planner();

    void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose_msgs);

    void setMaxVel(double max_v);
    void setMaxAcc(double max_a);
    void setAngVel(double max_ang_vel);
    void setAngAcc(double max_ang_acc);

    void calculateTrajectory(const Eigen::VectorXd& goal_pos,
                             const Eigen::VectorXd& goal_vel,
                             const Eigen::VectorXd& immediatel_pos,
                            //  const Eigen::VectorXd& immediatel_vel,
                             mav_trajectory_generation::Trajectory* trajectory);
    
    void publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

    void publishMarker(const mav_trajectory_generation::Trajectory& trajectory);
};

#endif 