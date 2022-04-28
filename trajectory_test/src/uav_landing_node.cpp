#include <geometric_controller/geometric_controller.h>
#include <ros/ros.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "uav_landing_node");
    ros::NodeHandle nh, nh_private;
    geometricCtrl a(nh, nh_private);
    ros::NodeHandle n;
    // ros::ServiceClient client = n.serviceClient<geometricCtrl::
    return 0;
}
