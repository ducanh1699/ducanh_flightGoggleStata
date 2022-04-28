#include "trajectory_test/traj_planner.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "trajectory_planner");

  ros::NodeHandle nh, nh_private;
  traj_planner planner(nh, nh_private);
  ROS_WARN_STREAM("SLEEPING FOR 5s TO WAIT FOR CLEAR CONSOLE");
  ros::Duration(5.0).sleep();
  ROS_WARN_STREAM("WARNING: CONSOLE INPUT/OUTPUT ONLY FOR DEMONSTRATION!");

  // define set point
  Eigen::Vector3d position, velocity;
  position << 18.5, 6.5, 1.0;
  velocity << 0.0, 0.0, 0.0;

  // define the middle point
  Eigen::Vector3d middle1_pos, middle1_vel;
  // middle1_pos << 20.5, -2.0, 2.0;
  middle1_pos << 22.5, -18.0, 3.0;
  //   middle1_vel << 0.0, 0.0, 0.0;

  // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
  ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
  std::cin.get();
  for (int i = 0; i < 10; i++) {
    ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
  }

  mav_trajectory_generation::Trajectory trajectory;

  int start_s=clock();
  planner.calculateTrajectory(position, velocity, middle1_pos, &trajectory);
  int stop_s=clock();
  std::cout << "optimization time: " << (stop_s-start_s)/double(CLOCKS_PER_SEC)*1000 << " ms" << std::endl;

  ROS_WARN_STREAM("PRESS ENTER TO DRAW THE TRAJECTORY");
  std::cin.get();
  planner.publishMarker(trajectory);

  ROS_WARN_STREAM("PRESS ENTER TO PUBLISH THE TRAJECTORY TO SAMPLER");
  std::cin.get();
  planner.publishTrajectory(trajectory);

  // ros::spin();
  ROS_WARN_STREAM("DONE. GOODBYE.");

  return 0;
}