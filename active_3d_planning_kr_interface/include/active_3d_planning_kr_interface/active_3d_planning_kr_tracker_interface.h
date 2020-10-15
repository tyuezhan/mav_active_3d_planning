#ifndef ACTIVE_3D_PLANNING_KR_TRACKER_INTERFACE_H_
#define ACTIVE_3D_PLANNING_KR_TRACKER_INTERFACE_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <kr_tracker_msgs/LineTrackerAction.h>
#include <kr_tracker_msgs/TrajectoryTrackerAction.h>
#include <kr_tracker_msgs/Transition.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class KrTrackerInterface {

public:
  KrTrackerInterface();
  ~KrTrackerInterface();

  void trajSubscribeCB(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);

  // action client done callback
  void lineTrackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::LineTrackerResultConstPtr &result);
 
  bool goTo(float x, float y, float z, float yaw, float v_des, float a_des, bool relative);
  
  // Tracker transition
  bool transition(const std::string &tracker_str);
private:
  typedef actionlib::SimpleActionClient<kr_tracker_msgs::LineTrackerAction> LineClientType;

  ros::NodeHandle nh_, pnh_;

  // Subscribers
  ros::Subscriber traj_subscriber_;

  std::string line_tracker_min_jerk_;

  // action Client: line tracker
  std::unique_ptr<LineClientType> line_tracker_min_jerk_client_ptr_;

  // Services client
  ros::ServiceClient srv_transition_;

  int line_tracker_status_;
  double server_wait_timeout_;
  bool goal_reached_;
};




#endif // ACTIVE_3D_PLANNING_KR_TRACKER_INTERFACE_H_
