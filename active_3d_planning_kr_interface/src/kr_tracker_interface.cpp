#include "active_3d_planning_kr_interface/active_3d_planning_kr_tracker_interface.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_3d_kr_interface");
  ros::NodeHandle n;

  // RobotNavigator robNav;
  KrTrackerInterface KrTrackerInterface;

  ros::spin();
  return 0;
}