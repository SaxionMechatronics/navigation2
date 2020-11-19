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
#include <cmath>
#include <vector>
#include <memory>

#include <tf2/LinearMath/Quaternion.h>
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
: BT::ConditionNode(condition_name, conf),
  // a_(9)
  crate_measure_length_(60),
  crate_measure_width_(0.40),
  offset_(2),
  tolerance_(0.02),
  laser_topic_("scan"),
  is_crate_found_(false),
  debugging_(true)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    laser_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&CrateDetectedCondition::laserCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "[Crate detector][constructor] Constructing detection node");
  // getInput("a", a_);
  getInput("crate_measure_length", crate_measure_length_);
  getInput("crate_measure_width", crate_measure_width_);
  getInput("offset", offset_);
  getInput("tolerance", tolerance_);
  getInput("laser_topic", laser_topic_);
  is_crate_found_ = false;
  callback_count_ = 0;
  // activate_docking_ = false;
}

/* TODO(Kevin):
    - make crate found variable
      - set true when crate is found and nav_goal is published
    - when crate found == true, do not execute sensor callback anymore
      - just return status::success and set activate_docking variable high on bb.
*/

BT::NodeStatus CrateDetectedCondition::tick()
{
  //lock is used for thread safety
  std::lock_guard<std::mutex> lock(mutex_);

  // NOTE: the int in received as expecter from the blackboard, the float on the other hand is filled in by the default at the top of the file!
  // TODO(Kevin): sort out why and how to fix
  if(debugging_){
    //TODO(Kevin): use cout to check if logging goes well!
    RCLCPP_INFO(node_->get_logger(), "[Crate detector][tick] is ticked");
    // RCLCPP_WARN(node_->get_logger(), "[Crate detector][tick] A is: %i", a_);
    RCLCPP_INFO(node_->get_logger(), "[Crate detector][tick] Crate length is: %1.2f meter", crate_measure_length_);
    RCLCPP_INFO(node_->get_logger(), "[Crate detector][tick] Crate width is: %1.2f meter", crate_measure_width_);
    RCLCPP_INFO(node_->get_logger(), "[Crate detector][tick] Offset is: %1.2f meter", offset_);
    RCLCPP_INFO(node_->get_logger(), "[Crate detector][tick] Tolerance is: %1.2f meter", tolerance_);
    RCLCPP_INFO(node_->get_logger(), "[Crate detector][tick] Laser scan topic is: %s", laser_topic_.c_str());
  }
  // is_crate_found_ = true;
  if (is_crate_found_) {
    // TODO(Kevin): send nav_goal to start pose
    // NOTE: already happens at the end of the callback, but is this desirable at the end of the callback?
    // CrateDetectedCondition::publishNavGoal(crate_front_pose_);
    // setOutput("activate_docking", 1); 
    RCLCPP_INFO(node_->get_logger(), "[Crate detector][tick] SUCCESS");
    return BT::NodeStatus::SUCCESS;
  } else {
    // setOutput("activate_docking", 0);
    RCLCPP_INFO(node_->get_logger(), "[Crate detector][tick] FAILURE");
    // setOutput("activate_docking", false);
    return BT::NodeStatus::FAILURE;
  }
}

// TODO(anyone): Note: maby make this a controller module or seperate thread
// which is called from here to increase the sequence time of the BT
void CrateDetectedCondition::laserCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO_ONCE(node_->get_logger(), "[Crate detector][laserCallback] Callback is activated");
  RCLCPP_INFO(node_->get_logger(), "[Crate detector][laserCallback] callback count is: %i", callback_count_);
  callback_count_++;

  if(callback_count_ == 10){
    RCLCPP_INFO(node_->get_logger(), "[Crate detector][laserCallback] Searching for crate in laser data");
    // TODO(Kevin):Activate logic
    is_crate_found_ = true;
    // msg->range_max = 0; 

    std::vector<geometry_msgs::msg::Vector3> cartesian_sensor_data = this->polarToCartesian(msg);

    std::vector<geometry_msgs::msg::Vector3> corner_locations_in_laser_frame = this->findCorners(cartesian_sensor_data);

    geometry_msgs::msg::PoseStamped crate_front_pose_ = this->findCrate(corner_locations_in_laser_frame);

    // if(broadcast_crate_){
    //   this->broadcastCrateToTFTree(crate_front_pose_);
    //   is_crate_found_ = true;
    // }
    // TODO(anyone): isolate logic and build a env to test and maby plot the corner locations
    callback_count_ = 0;
  }
}

/* 
  function to make ranges with certain incremental value
   TODO(anyone): optimize: safe list as member variable and break out this function is list is filled already
*/
std::vector<float> CrateDetectedCondition::arange(float start, float end, float increment)
{
  float steps = ((end - start) / increment);
  std::vector<float> temp_vector;
  if(false){ //debugging_
    RCLCPP_WARN(node_->get_logger(), "[Crate detector][arange] The arguments for arrange are: start: %f, stop: %f, increments: %f", start, end, increment);
    RCLCPP_WARN(node_->get_logger(), "[Crate detector][arange] steps: %f", steps);
  }
  float temp_value = start;
  for(int i = 0; i <= steps; i++) {
    temp_vector.push_back(temp_value);
    temp_value += increment;
    if(false){ //debugging_
      RCLCPP_WARN(node_->get_logger(), "[Crate detector][arange] The vector[%i] is: %f", i, temp_value);
    }
  }
  return temp_vector;
}

std::vector<geometry_msgs::msg::Vector3> CrateDetectedCondition::polarToCartesian(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if(debugging_){
    RCLCPP_WARN(node_->get_logger(), "[Crate detector][polarToCartesian] Is called");
  }
  std::vector<float> thetha = this->arange(msg->angle_min, msg->angle_max, msg->angle_increment);
  std::vector<geometry_msgs::msg::Vector3> temp_coordinates; // coordinates in meters

  // convert polar to cartesian
  for(long unsigned int i = 0; i <= (thetha.size()-1); i++) {
    float xc = msg->ranges[i] * cos(thetha[i]);
    float yc = msg->ranges[i] * sin(thetha[i]);
    geometry_msgs::msg::Vector3 tmp;
    tmp.set__x(xc);
    tmp.set__y(yc);
    tmp.set__z(0);
    temp_coordinates.push_back(tmp);
    if(false){ //debugging_
      RCLCPP_WARN(node_->get_logger(), "[Crate detector][polarToCartesian] xc: %f, yc: %f", xc, yc);
      float size_thetha = thetha.size();
      float size_temp_coord = temp_coordinates.size();
      RCLCPP_WARN(node_->get_logger(), "[Crate detector][polarToCartesian] length of lists thetha: %f, temp_coordinates: %f", size_thetha, size_temp_coord );
    }
  }
  return temp_coordinates;
}

std::vector<geometry_msgs::msg::Vector3> CrateDetectedCondition::findCorners(std::vector<geometry_msgs::msg::Vector3> cartesian_sensor_data)
{
  if(debugging_){
    RCLCPP_WARN(node_->get_logger(), "[Crate detector][findCorners] Is called");
  }
  // init lists for calculating the slope between the points
  std::vector<float> corner_locations;
  std::vector<float> dydxs;
  std::vector<float> deltas;

  // calc slope between points of laser scanner
  for(long unsigned int i = 0; i <= (cartesian_sensor_data.size()-1); i++) {
    float dydx = (cartesian_sensor_data[i+1].y - cartesian_sensor_data[i].y)/(cartesian_sensor_data[i+1].x - cartesian_sensor_data[i].x);
    dydxs.push_back(dydx);
  }
  if(debugging_){
    RCLCPP_WARN(node_->get_logger(), "[Crate detector][findCorners][cartesian to dydxs] length of cart_sens_data: %i, dydxs: %i", cartesian_sensor_data.size(), dydxs.size());
  }

  // calculate difference in dslope
  for(long unsigned int i = 0; i <= (dydxs.size()-1); i++) {
    deltas.push_back(dydxs[i] - dydxs[i+1]);
    // if(i == dydxs.size()-1) {
    //   deltas.push_back(dydxs[i] - dydxs[i-1]);
    // }
  }
  if(debugging_){
    RCLCPP_WARN(node_->get_logger(), "[Crate detector][findCorners][dydxs to deltas] length of dydxs: %i, deltas: %i", dydxs.size(), deltas.size());
  }

  // add location to corner where point comes from or goes to infinity
  for(long unsigned int i = 0; i <= (deltas.size()-1); i++) {
    if(std::isnan(deltas[i]) & !(std::isnan(deltas[i+1]))) {
      corner_locations.push_back(i+1);
    } else if(!(std::isnan(deltas[i])) & std::isnan(deltas[i+1])) {
      corner_locations.push_back(i+2);
    } else if(deltas[i] > 40) {
      corner_locations.push_back(i);
    }
  }
  if(debugging_){
    RCLCPP_WARN(node_->get_logger(), "[Crate detector][findCorners][deltas to corner locations] length of deltas: %i, corner_locations: %i", deltas.size(), corner_locations.size());
  }

  std::vector<geometry_msgs::msg::Vector3> corn_locs_sens_frame;

  // deltas[0] are made from dydxs[0] - dydxs[1]
  // dydxs[0] are made from (y[1] - y[0])/(x[1] - x[0])
  // that means that deltas are made from 3 x/y pairs,
  // So the x/y pair in the middle is the absolute corner.
  for(long unsigned int i = 0; i <= (corner_locations.size()-1); i++) {
    // if(corner_locations[i] == !0) {
    geometry_msgs::msg::Vector3 temp;
    temp.set__x(cartesian_sensor_data[corner_locations[i]+1].x);
    temp.set__y(cartesian_sensor_data[corner_locations[i]+1].y);
    temp.set__z(0);
    corn_locs_sens_frame.push_back(temp);
    if(debugging_){
      float x = (float) temp.x;
      float y = (float) temp.y;
      float z = (float) temp.z;
      RCLCPP_WARN(node_->get_logger(), "[Crate detector][findCorners][corner locations] no. %i is %f, %f, %f", i, x, y, z);
    }
    // }
  }
  return corn_locs_sens_frame;
}

geometry_msgs::msg::PoseStamped CrateDetectedCondition::findCrate(std::vector<geometry_msgs::msg::Vector3> corner_locations_in_laser_frame)
{
  double x_middle_of_crate_front = 0;
  double y_middle_of_crate_front = 0;
  geometry_msgs::msg::PoseStamped crate_front_plane_in_scan_frame;

  tf2::Quaternion qt;
  qt.setRPY(0, 0, 0);

  // NOTE: or should it be "x_location_in_laser_frame.size() -1"
  for(long unsigned int i = 0; i <= (corner_locations_in_laser_frame.size()-1); i++) {
    // use pythagoras to calculate the distance between two corncers.
    double temp = sqrt(pow((corner_locations_in_laser_frame[i+1].x - corner_locations_in_laser_frame[i].x), 2 ) +
                       pow((corner_locations_in_laser_frame[i+1].y - corner_locations_in_laser_frame[i].y), 2 ));

    if((crate_measure_width_ - tolerance_) >= temp && temp <= (crate_measure_width_ + tolerance_)) {
      RCLCPP_WARN(node_->get_logger(), "[Crate detector][findCrate]Crate found!");
      x_middle_of_crate_front = (corner_locations_in_laser_frame[i+1].x + corner_locations_in_laser_frame[i].x)/2;
      y_middle_of_crate_front = (corner_locations_in_laser_frame[i+1].y + corner_locations_in_laser_frame[i].y)/2;
      if(debugging_){
        float x = (float) x_middle_of_crate_front;
        float y = (float) y_middle_of_crate_front;
        RCLCPP_WARN(node_->get_logger(), "[Crate detector][findCrate]Crate X: %f and Y: %f", x, y );
      }

      // crate_front_plane_in_scan_frame.header.frame_id = "base_scan";
      // crate_front_plane_in_scan_frame.header.stamp = node_->now();                //tf2::TimePointZero);

      // crate_front_plane_in_scan_frame.pose.position.x = x_middle_of_crate_front;
      // crate_front_plane_in_scan_frame.pose.position.y = y_middle_of_crate_front;
      // crate_front_plane_in_scan_frame.pose.position.z = 0;
      // crate_front_plane_in_scan_frame.pose.orientation.w = qt.w();
      // crate_front_plane_in_scan_frame.pose.orientation.x = qt.x();
      // crate_front_plane_in_scan_frame.pose.orientation.y = qt.y();
      // crate_front_plane_in_scan_frame.pose.orientation.z = qt.z();
      broadcast_crate_ = true;
      is_crate_found_ = true;
    // } else if((crate_measure_length_ - tolerance_) >= temp && temp <= (crate_measure_length_ + tolerance_)) {
    //   RCLCPP_WARN_ONCE(node_->get_logger(), "Long side of the crate found, drive around to the short side!");
    //   is_crate_found_ = false;
    //   broadcast_crate_ = false;
      // TODO(Kevin): break or exit callback
      // TODO(anyone): could calculate the front of the crate from
      // NOTE: This will not be done within this project to keep it complexity suppresed!
    } else {
      RCLCPP_WARN(node_->get_logger(), "[Crate detector][findCrate]No crate found!");
      is_crate_found_ = false;
      broadcast_crate_ = false;
      // TODO(Kevin): break or exit callback
    }
  }
  return crate_front_plane_in_scan_frame;
}

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