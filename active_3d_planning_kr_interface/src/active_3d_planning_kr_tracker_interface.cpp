#include "active_3d_planning_kr_interface/active_3d_planning_kr_tracker_interface.h"


KrTrackerInterface::KrTrackerInterface() {

  pnh_ = ros::NodeHandle("~");

  pnh_.param<double>("server_wait_timeout", server_wait_timeout_, 10.0);
  // Planner Subscriber
  traj_subscriber_ = nh_.subscribe("/ddk/command/trajectory", 5, &KrTrackerInterface::trajSubscribeCB, this);

  // Services
  srv_transition_ = nh_.serviceClient<kr_tracker_msgs::Transition>("/ddk/trackers_manager/transition");

  // Action client
  line_tracker_min_jerk_client_ptr_.reset(new LineClientType(nh_, "/ddk/trackers_manager/line_tracker_min_jerk/LineTracker", true));
  if (!line_tracker_min_jerk_client_ptr_->waitForServer(ros::Duration(server_wait_timeout_))) {
    ROS_ERROR("LineTrackerMinJerk server not found.");
  }

  // Strings for tracker transition
  line_tracker_min_jerk_ = "kr_trackers/LineTrackerMinJerk";

  line_tracker_status_ = 0;
  goal_reached_ = false;
  ROS_INFO("Kr tracker interface initialized!");
}


KrTrackerInterface::~KrTrackerInterface() {};


void KrTrackerInterface::trajSubscribeCB(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg) {
  ROS_INFO("Received one msg, length: %d", msg->points.size());
  ros::Rate loop_rate(1);
  double yaw, _pitch, _roll;
  for (int i = 0; i < msg->points.size(); i++) {
    // ROS_INFO("Enter loop");
    float x = msg->points[i].transforms[0].translation.x;
    float y = msg->points[i].transforms[0].translation.y;
    float z = msg->points[i].transforms[0].translation.z;

    tf2::Matrix3x3(tf2::Quaternion(msg->points[i].transforms[0].rotation.x, msg->points[i].transforms[0].rotation.y,
                                 msg->points[i].transforms[0].rotation.z, msg->points[i].transforms[0].rotation.w)).getEulerYPR(yaw, _pitch, _roll);
    
    while(true) {
      // ROS_INFO("while try");
      if (line_tracker_status_ == 0) {
        goal_reached_ = false;
        ROS_INFO("Call goTo with x: %f, y: %f, z: %f, yaw_: %f", x, y, z, yaw);
        goTo(x, y, z, yaw, 0, 0, false);
        break;
      }
      ros::spinOnce(); 
      loop_rate.sleep(); 
    }

    ROS_INFO("Next goal point");
  }
}


bool KrTrackerInterface::goTo(float x, float y, float z, float yaw, float v_des, float a_des, bool relative) {
  // hack now
  double yaw_ = 0;
  
  kr_tracker_msgs::LineTrackerGoal goal;
  goal.x = x;
  goal.y = y;
  goal.relative = relative;
  // Convert relative translation in body frame to global frame
  if (relative) {
    goal.x = x * std::cos(yaw_) - y * std::sin(yaw_);
    goal.y = x * std::sin(yaw_) + y * std::cos(yaw_);
  }
  goal.z = z;
  goal.yaw = yaw;
  goal.v_des = v_des;
  goal.a_des = a_des;

  line_tracker_min_jerk_client_ptr_->sendGoal(
      goal, boost::bind(&KrTrackerInterface::lineTrackerDoneCB, this, _1, _2),
      LineClientType::SimpleActiveCallback(), LineClientType::SimpleFeedbackCallback());
  line_tracker_status_ = 1;
  return transition(line_tracker_min_jerk_);
}


void KrTrackerInterface::lineTrackerDoneCB(const actionlib::SimpleClientGoalState &state, const kr_tracker_msgs::LineTrackerResultConstPtr &result) {
  ROS_INFO("Goal reached.");
  line_tracker_status_ = 0;
  goal_reached_ = true;
}


bool KrTrackerInterface::transition(const std::string &tracker_str) {
  kr_tracker_msgs::Transition transition_cmd;
  transition_cmd.request.tracker = tracker_str;

  if (srv_transition_.call(transition_cmd) && transition_cmd.response.success) {
    ROS_INFO("Current tracker: %s", tracker_str.c_str());
    return true;
  }
  return false;
}
