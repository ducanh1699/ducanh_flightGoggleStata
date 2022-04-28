#include <trajectory_test/traj_sampler.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "trajectory_sampler");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	traj_sampler trajectory_sampler_node(nh, nh_private);
	ROS_INFO("trajectory_sampler has been initialized!");
	ros::spin();
    return 0;
}
