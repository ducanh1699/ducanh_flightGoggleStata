#ifndef TRAJECTORY_TETS_TRAJECTORY_SAMPLER_H
#define TRAJECTORY_TETS_TRAJECTORY_SAMPLER_H


#include<mav_trajectory_generation_ros/trajectory_sampler_node.h>
#include<mav_trajectory_generation/trajectory_sampling.h>
#include<mav_planning_msgs/PolynomialTrajectory.h>
#include<trajectory_test/FlatTarget.h>
#include<ros/ros.h>
#include<geometry_msgs/Vector3.h>

class traj_sampler
{
    private:
        ros::NodeHandle& nh_;
        ros::NodeHandle& nh_private_;
        ros::Subscriber traj_sub;
        ros::Timer publish_timer_;
        ros::Publisher flatreferencePub_;


        // Trajectory sampling interval.
        double dt_;
        // Time at currently published trajectory sample.
        double current_sample_time_;

        mav_trajectory_generation::Trajectory trajectory_;


        
    public:
        traj_sampler(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        ~traj_sampler();
        void TrajectoryCallback(const mav_planning_msgs::PolynomialTrajectory& segemnts);
        void TimerCallback(const ros::TimerEvent& event_);
        // void ProcessTrajectory();
};

#endif