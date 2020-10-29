// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
// #include <math.h>       /* cos & sin */
#include <cmath>
#include <vector>
#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2_ros/transform_broadcaster.h"


#include "nav2_behavior_tree/plugins/condition/crate_detected_condition.hpp"

// valgrind mem leaks (profiler)
// cpp gdb (debugger)

namespace nav2_behavior_tree
{

CrateDetectedCondition::CrateDetectedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
  // laser_topic_("scan"),
  // crate_measure_legnth_(0.60),
  // crate_measure_width_(0.40),
  // offset_(2)
  // tolerance_(0.02)
  // is_crate_found_(false)
{
  // getInput("laser_topic", laser_topic_);
  // getInput("crate_measure_legnth", crate_measure_legnth_);
  // getInput("crate_measure_width", crate_measure_width_);
  // getInput("offset", offset_);
  // getInput("tolerance", tolerance_);
  // bool is_crate_found_ = false;

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  // tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  // laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
  //   laser_topic_,
  //   rclcpp::SystemDefaultsQoS(),
  //   std::bind(&CrateDetectedCondition::laserCallback, this, std::placeholders::_1));
}

BT::NodeStatus CrateDetectedCondition::tick()
{
//   lock is used for thread safety
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_WARN_ONCE(node_->get_logger(), "Crate detector has been ticked");

  // CrateDetectedCondition::laserCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);
  if (true) { //is_crate_found_
    // TODO(anyone): send nav_goal to start pose
    // NOTE: already happens at the end of the callback, but is this desirable at the end of the callback?
    // CrateDetectedCondition::publishNavGoal(crate_front_pose_);
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

// // TODO(anyone): Note: maby make this a controller module or seperate thread
// // which is called from here to increase the sequence time of the BT
// void CrateDetectedCondition::laserCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   std::lock_guard<std::mutex> lock(mutex_);
//   RCLCPP_WARN_ONCE(node_->get_logger(), "Crate detector callback is activated");
//   is_crate_found_ = true;
//   msg->range_max = 0; 
//   std::vector<geometry_msgs::msg::Vector3> cartesian_sensor_data = CrateDetectedCondition::polarToCartesian(msg);

//   std::vector<geometry_msgs::msg::Vector3> corner_locations_in_laser_frame = CrateDetectedCondition::findCorners(cartesian_sensor_data);

//   geometry_msgs::msg::PoseStamped crate_front_pose_ = CrateDetectedCondition::findCrate(corner_locations_in_laser_frame);

//   if(broadcast_crate_){
//     CrateDetectedCondition::broadcastCrateToTFTree(crate_front_pose_);
//     is_crate_found_ = true;
//   }
//   // TODO(anyone): isolate logic and build a env to test and maby plot the corner locations
// }

// /* 
//   function to make ranges with certain incremental value
//    NOTE: keep in mind that the the list returned
//    could be one slot smaller than the list with ranges.
//    TODO(anyone): check if temp_value start at 0 or at the first increment value???
//    TODO(anyone): list al member function in hpp file
// */
// std::vector<float> CrateDetectedCondition::arange(float start, float end, float increment)
// {
//   double steps = ((end - start) / increment);
//   std::vector<float> temp_list;
//   float temp_value = start;
//   for(int i = 0; i <= steps; i++) {
//       // temp_list(i) = temp_value;
//       temp_list.push_back(temp_value);
//       temp_value += increment;
//   }
//   return temp_list;
// }

// std::vector<geometry_msgs::msg::Vector3> CrateDetectedCondition::polarToCartesian(sensor_msgs::msg::LaserScan::SharedPtr msg)
// {
//   std::vector<float> thetha = CrateDetectedCondition::arange(msg->angle_min, msg->angle_max, msg->angle_increment);
//   std::vector<geometry_msgs::msg::Vector3> temp_coordinates; // coordinates in meters

//   // convert polar to cartesian
//   for(long unsigned int i = 0; i <= (thetha.size()-1); i++) {
//     float xc = msg->ranges[i] * cos(thetha[i]);
//     float yc = msg->ranges[i] * sin(thetha[i]);
//     geometry_msgs::msg::Vector3 tmp;
//     tmp.set__x(xc);
//     tmp.set__y(yc);
//     tmp.set__z(0);
//     temp_coordinates.push_back(tmp);
//   }

//   return temp_coordinates;
// }

// std::vector<geometry_msgs::msg::Vector3> findCorners(std::vector<geometry_msgs::msg::Vector3> cartesian_sensor_data)
// {
//   // init lists for calculating the slope between the points
//   std::vector<float> corner_locations;
//   std::vector<float> dydxs;
//   std::vector<float> deltas;

//   // calc slope between points of laser scanner
//   for(long unsigned int i = 0; i <= (cartesian_sensor_data.size()-1); i++) {
//     float dydx = (cartesian_sensor_data[i+1].y - cartesian_sensor_data[i].y)/(cartesian_sensor_data[i+1].x - cartesian_sensor_data[i].x);
//     dydxs.push_back(dydx);

//     if(i == (cartesian_sensor_data.size()-1)) {
//       float dydx = (cartesian_sensor_data[0].y - cartesian_sensor_data[i].y)/(cartesian_sensor_data[0].x - cartesian_sensor_data[i].x);
//       dydxs.push_back(dydx);
//     }
//   }

//   // calculate difference in dslope
//   for(long unsigned int i = 0; i <= (dydxs.size()-1); i++) {
//     deltas.push_back(dydxs[i] - dydxs[i+1]);
//     if(i == dydxs.size()-1) {
//       deltas.push_back(dydxs[i] - dydxs[i-1]);
//     }
//   }

//   // add location to corner where point comes from or goes to infinity
//   for(long unsigned int i = 0; i <= (deltas.size()-1); i++) {
//     if(std::isnan(deltas[i]) & !(std::isnan(deltas[i+1]))) {
//       corner_locations.push_back(i+1);
//     } else if(!(std::isnan(deltas[i])) & std::isnan(deltas[i+1])) {
//       corner_locations.push_back(i+2);
//     } else if(deltas[i] > 40) {
//       corner_locations.push_back(i);
//     }
//   }

//   std::vector<geometry_msgs::msg::Vector3> locations_in_laser_frame;

//   // deltas[0] are made from dydxs[0] - dydxs[1]
//   // dydxs[0] are made from (y[1] - y[0])/(x[1] - x[0])
//   // that means that deltas are made from 3 x/y pairs,
//   // So the x/y pair in the middle is the absolute corner.
//   for(long unsigned int i = 0; i <= (corner_locations.size()-1); i++) {
//     if(corner_locations[i] == !0) {
//       geometry_msgs::msg::Vector3 temp;
//       temp.set__x(cartesian_sensor_data[corner_locations[i]+1].x);
//       temp.set__y(cartesian_sensor_data[corner_locations[i]+1].y);
//       temp.set__z(0);
//       locations_in_laser_frame.push_back(temp);
//     }
//   }
//   return locations_in_laser_frame;
// }

// geometry_msgs::msg::PoseStamped CrateDetectedCondition::findCrate(std::vector<geometry_msgs::msg::Vector3> corner_locations_in_laser_frame)
// {
//   double x_middle_of_crate_front = 0;
//   double y_middle_of_crate_front = 0;
//   geometry_msgs::msg::PoseStamped crate_front_plane_in_scan_frame;

//   tf2::Quaternion qt;
//   qt.setRPY(0, 0, 0);

//   // NOTE: or should it be "x_location_in_laser_frame.size() -1"
//   for(long unsigned int i = 0; i <= (corner_locations_in_laser_frame.size()-1); i++) {
//     // use pythagoras to calculate the distance between two corncers.
//     double temp = sqrt(pow((corner_locations_in_laser_frame[i+1].x - corner_locations_in_laser_frame[i].x), 2 ) +
//                        pow((corner_locations_in_laser_frame[i+1].y - corner_locations_in_laser_frame[i].y), 2 ));

//     if((crate_measure_width_ - tolerance_) >= temp && temp <= (crate_measure_width_ + tolerance_)) {
//       x_middle_of_crate_front = (corner_locations_in_laser_frame[i+1].x + corner_locations_in_laser_frame[i].x)/2;
//       y_middle_of_crate_front = (corner_locations_in_laser_frame[i+1].y + corner_locations_in_laser_frame[i].y)/2;

//       crate_front_plane_in_scan_frame.header.frame_id = "base_scan";
//       crate_front_plane_in_scan_frame.header.stamp = node_->now();                //tf2::TimePointZero);

//       crate_front_plane_in_scan_frame.pose.position.x = x_middle_of_crate_front;
//       crate_front_plane_in_scan_frame.pose.position.y = y_middle_of_crate_front;
//       crate_front_plane_in_scan_frame.pose.position.z = 0;
//       crate_front_plane_in_scan_frame.pose.orientation.w = qt.w();
//       crate_front_plane_in_scan_frame.pose.orientation.x = qt.x();
//       crate_front_plane_in_scan_frame.pose.orientation.y = qt.y();
//       crate_front_plane_in_scan_frame.pose.orientation.z = qt.z();
//       broadcast_crate_ = true;
//     } else if((crate_measure_legnth_ - tolerance_) >= temp && temp <= (crate_measure_legnth_ + tolerance_)) {
//       RCLCPP_WARN_ONCE(node_->get_logger(), "Long side of the crate found, drive around to the short side!");
//       is_crate_found_ = false;
//       broadcast_crate_ = false;
//       // TODO(Kevin): break or exit callback
//       // TODO(anyone): could calculate the front of the crate from
//       // NOTE: This will not be done within this project to keep it complexity suppresed!
//     } else {
//       RCLCPP_WARN_ONCE(node_->get_logger(), "No side of the crate found!");
//       is_crate_found_ = false;
//       broadcast_crate_ = false;
//       // TODO(Kevin): break or exit callback
//     }
//   }
//   return crate_front_plane_in_scan_frame;
// }

// void CrateDetectedCondition::broadcastCrateToTFTree(geometry_msgs::msg::PoseStamped pose)
// {
//   tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
//   geometry_msgs::msg::TransformStamped tmp_tf_stamped;

//   tmp_tf_stamped.header.frame_id = "base_scan";   // TODO(kevin): check frame name
//   tmp_tf_stamped.header.stamp = node_->now();
//   tmp_tf_stamped.child_frame_id = "crate_frame";
//   tmp_tf_stamped.transform.translation.set__x(pose.pose.position.x);
//   tmp_tf_stamped.transform.translation.set__y(pose.pose.position.y);
//   tmp_tf_stamped.transform.translation.set__z(pose.pose.position.z);
//   tmp_tf_stamped.transform.set__rotation(pose.pose.orientation);
//   tf_broadcaster_->sendTransform(tmp_tf_stamped);
// }

// void CrateDetectedCondition::publishNavGoal(geometry_msgs::msg::PoseStamped pose)
// {
// // obtain name of world frame in tf tree
//   std::vector<std::string> frames_vector = tf_->getAllFrameNames();
//   std::string world_frame_str;
//   for(long unsigned int i = 0; i < frames_vector.size(); i++){
//     if(frames_vector[i].find("map") != std::string::npos){
//       world_frame_str = frames_vector[i];
//     }
//   }
//   tf_->transform<geometry_msgs::msg::PoseStamped>(pose, nav_goal_, world_frame_str, tf2::durationFromSec(0.0));

// // TODO(Kevin): before sending the nav_goal, change the goal to be 2 meters in front of the crate!
//   auto goal_updater_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_update", 10);
//   goal_updater_pub->publish(nav_goal_);
// }  

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CrateDetectedCondition>("CrateDetected");
}