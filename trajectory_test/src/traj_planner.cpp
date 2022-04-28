#include <trajectory_test/traj_planner.h>

traj_planner::traj_planner(ros::NodeHandle& nh, ros::NodeHandle& nh_private):
nh_(nh), nh_private_(nh_private), max_v_(2.0), max_a_(2.0),current_vel_(Eigen::Vector3d::Zero()), current_pose_(Eigen::Affine3d::Identity())
  {
    sub_odom_ = nh_.subscribe("/uav/odometry", 1, &traj_planner::uavOdomCallback, this);
    pub_trajectory_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory>("/trajectory", 1);
    pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("/trajectory_marker",1);

    nh_private_.param<double>("max_velocity", max_v_, 1.0);
    nh_private_.param<double>("max_acceleration", max_a_, 0.5);
    nh_private_.param<double>("max_angular_velocity", max_ang_v_, 0.1);
    nh_private_.param<double>("max_angular_acceleration", max_ang_a_, 0.1);

    ROS_INFO("trajectory_planner has been initialized!");
}

traj_planner::~traj_planner(){
    // Destructor
}

void traj_planner::setMaxVel(double max_v){
  max_v_ = max_v;
}

void traj_planner::setMaxAcc(double max_a){
  max_a_ = max_a;
}

void traj_planner::setAngAcc(double max_ang_acc){
  max_ang_a_= max_ang_acc;
}

void traj_planner::setAngVel(double max_ang_vel){
  max_ang_v_ = max_ang_vel;
}

void traj_planner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose_msgs ){
  tf::poseMsgToEigen(pose_msgs->pose.pose, current_pose_);
  tf::vectorMsgToEigen(pose_msgs->twist.twist.linear, current_vel_);
  tf::vectorMsgToEigen(pose_msgs->twist.twist.angular, current_angular_vel_);
}

void traj_planner::calculateTrajectory(const Eigen::VectorXd& goal_pos,
                         const Eigen::VectorXd& goal_vel,
                         const Eigen::VectorXd& immediatel_pos,
                         //  const Eigen::VectorXd& immediatel_vel,
                         mav_trajectory_generation::Trajectory* trajectory)
{
  const int dimension = 3;
  mav_trajectory_generation::Vertex::Vector vertices;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

  mav_trajectory_generation::Vertex start(dimension), immediatel(dimension), end(dimension);

  start.makeStartOrEnd(current_pose_.translation(), derivative_to_optimize);
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_vel_);
  vertices.push_back(start);

  immediatel.addConstraint(mav_trajectory_generation::derivative_order::POSITION, immediatel_pos);
  vertices.push_back(immediatel);

  end.makeStartOrEnd(goal_pos, derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, goal_vel);
  vertices.push_back(end);

  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
  // For Linear Optimization
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.solveLinear();
  mav_trajectory_generation::Segment::Vector segments;
  opt.getSegments(&segments);

  // For Nonlinear Optimization
  // mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  // parameters.max_iterations = 1000;
  // parameters.f_rel = 0.05;
  // parameters.x_rel = 0.1;
  // parameters.time_penalty = 500.0;
  // parameters.initial_stepsize_rel = 0.1;
  // parameters.inequality_constraint_tolerance = 0.1;

  // const int N = 10;
  // mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  // opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  // opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  // opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ANGULAR_ACCELERATION, max_a_);
  // opt.optimize();
  // mav_trajectory_generation::Segment::Vector segments;
  // opt.getPolynomialOptimizationRef().getSegments(&segments);
  
  // Get the trajectory
  opt.getTrajectory(trajectory);
  std::cout << trajectory->getMaxTime() << std::endl; 
}

void traj_planner::publishMarker(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance = 0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_marker_.publish(markers);
}

void traj_planner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  mav_planning_msgs::PolynomialTrajectory msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  ROS_INFO_STREAM("Start publish");
  pub_trajectory_.publish(msg);
}

