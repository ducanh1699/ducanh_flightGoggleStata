#include <trajectory_test/traj_sampler.h>

traj_sampler::traj_sampler(ros::NodeHandle& nh, ros::NodeHandle& nh_private):nh_(nh),
nh_private_(nh_private), dt_(0.01), current_sample_time_(0.0)
{	
	ros::Duration(0.5).sleep();
    traj_sub = nh_.subscribe("/trajectory", 1, &traj_sampler::TrajectoryCallback, this);
    publish_timer_ = nh_.createTimer(ros::Duration(dt_), &traj_sampler::TimerCallback, this);
    flatreferencePub_ = nh_.advertise<trajectory_test::FlatTarget>("/reference/flatsetpoint", 1);
	nh_private_.param<double>("sample_time", dt_, 0.01);
}

traj_sampler::~traj_sampler(){};

void traj_sampler::TrajectoryCallback(const mav_planning_msgs::PolynomialTrajectory& segments){
	if (segments.segments.empty()){
		ROS_INFO("Trajectory sampler: received empty waypoint message");
		return;
	}
	else
	{
		ROS_INFO("Trajectory sampler: received %lu new waypoints", segments.segments.size());
	}
    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(segments, &trajectory_);
	if (!success)
	{
		return;
	}
	publish_timer_.start();
}

void traj_sampler::TimerCallback( const ros::TimerEvent& event_){
    if (current_sample_time_ <= trajectory_.getMaxTime())
	{
		trajectory_msgs::MultiDOFJointTrajectory msg;
		mav_msgs::EigenTrajectoryPoint trajectory_point;
		bool success = mav_trajectory_generation::sampleTrajectoryAtTime(trajectory_, current_sample_time_, &trajectory_point);
		if (!success)
		{	
			ROS_INFO("Stop line 44");
			publish_timer_.stop();
		}
		mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
		msg.points[0].time_from_start = ros::Duration(current_sample_time_);
		msg.header.stamp = ros::Time::now();
		//command_pub_.publish(msg);
		current_sample_time_ += dt_;
		// ROS_INFO_STREAM(current_sample_time_);

		trajectory_test::FlatTarget traj_msg;
		traj_msg.type_mask = 2;
		traj_msg.header = msg.header;
		traj_msg.position = msg.points[0].transforms[0].translation;
		traj_msg.velocity = msg.points[0].velocities[0].linear;
		traj_msg.acceleration = msg.points[0].accelerations[0].linear;
		flatreferencePub_.publish(traj_msg);

	// geometry_msgs::Quaternion quat = msg.points[0].transforms[0].rotation;
	// double yaw = tf::getYaw(quat);
	// std_msgs::Float32 yaw_msg;
	// yaw_msg.data = static_cast<float>(yaw);
		// ROS_INFO_STREAM("msgs: " << traj_msg);
		// flatreferencePub_.publish(traj_msg);
    }
    else
	{	
		ROS_INFO("Stop line 72");
		publish_timer_.stop();
	}
}

