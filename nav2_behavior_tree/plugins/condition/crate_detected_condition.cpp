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
#include <list>
#include <memory>
#include "nav2_behavior_tree/plugins/condition/crate_detected_condition.hpp"

namespace nav2_behavior_tree
{

CrateDetectedCondition::CrateDetectedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  laser_topic_("scan"),
  crate_measure_legnth_(0.60),
  crate_measure_width_(0.40),
  offset_(2)
  // is_crate_found_(false)
{
  getInput("laser_topic", laser_topic_);
  getInput("crate_measure_legnth", crate_measure_legnth_);
  getInput("crate_measure_width", crate_measure_width_);
  getInput("offset", offset_);
  bool crate_found = false;
  // getInput("is_crate_found", is_crate_found_);
  // TODO(anyone): implement is_crate_found for faster
  // sequence times when crate is already found earlier

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    laser_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&CrateDetectedCondition::laserCallback, this, std::placeholders::_1));
}

BT::NodeStatus CrateDetectedCondition::tick()
{
//   lock is used for thread safety
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_WARN_ONCE(get_logger(), "Crate detector has been ticked");

  // CrateDetectedCondition::laserCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);
  if (crate_found) {
    // TODO(anyone): send nav to start pose
    // NOTE: already happens at the end of the callback
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

// TODO(anyone): laserCallback()
/*
    1. convert laser data (angle and distance) into x and y coordinates
    2. look for edges (edge detection)
    3. calc length of planes/ straigt lines between edges
    4. compare length of planes/ lines to measures of crate
    5. determain pose infront of crate to nav to
    6. return pose
*/
// TODO(anyone): Note: maby make this a controller module or seperate thread
// which is called from here to increase the sequence time of the BT
void CrateDetectedCondition::laserCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_WARN_ONCE(get_logger(), "Crate detector callback is activated");

  // define 360 degree in radians
  double fullcircle = 6.28319;  // rads


// msg is a shared pointer. To get data out of it use "msg->ranges" instead of the "."
  // obtain ranges from message
  std::list<float> ranges[360] = {0.0};
  ranges = msg.ranges;  // in meters

  // obtain agnles from message
  std::list<float> thetha[360] = {0.0};
  thetha = CrateDetectedCondition::arrange(0, fullcircle, msg.angle_increments);

  // init empty lists for converting from polar to cartesian
  std::list<float> x;  // in meters
  std::list<float> y;  // in meters

  // convert polar to cartesian
  for(int i = 0; i <= thetha.size(); i++) {
    float xc = ranges[i] * cos(thetha[i]);
    float yc = ranges[i] * sin(thetha[i]);
    x.push_back(xc);
    y.push_back(yc);
  }

  // TODO(anyone): fix/resize lenght of lists
  // init lists for calculating the slope between the points
  std::list<float> corner_locations[360] = {0.0};
  std::list<float> dydxs[360] = {0.0};
  std::list<float> deltas[360] = {0.0};

  // calc slope between points of laser scanner
  for(int i = 0; i <= (ranges.size()-1); i++) {
    float dydx = (y[i+1] - y[i])/(x[i+1] - x[i]);
    dydxs.push_back(dydx);

    if(i == (ranges.size()-1)) {
      float dydx = (y[0] - y[i])/(x[0] - x[i]);
      dydxs.push_back(dydx);
    }
  }

  // calculate difference in dslope
  for(int i = 0; i <= (dydxs.size()-1); i++) {
    deltas.push_back(dydxs[i] - dydxs[i+1]);
    if(i == dydxs.size()-1) {
      deltas.push_back(dydxs[i] - dydxs[i-1]);
    }
  }

  // add location to corner where point comes from or goes to infinity
  for(int i = 0; i <= (deltas.size()-1); i++) {
    if(std::isnan(deltas[i]) & !(std::isnan(deltas[i+1]))) {
      corner_locations.push_back(i+1);
    } else if(!(std::isnan(deltas[i])) & std::isnan(deltas[i+1])) {
      corner_locations.push_back(i+2);
    } else if(deltas[i] > 40) {
      corner_locations.push_back(i);
    }
  }

  std::list<float> x_locations_in_laser_frame[360] = {0.0};
  std::list<float> y_locations_in_laser_frame[360] = {0.0};

  // deltas[0] are made from dydxs[0] - dydxs[1]
  // dydxs[0] are made from (y[1] - y[0])/(x[1] - x[0])
  // that means that deltas are made from 3 x/y pairs,
  // So the x/y pair in the middle is the absolute corner.
  for(int i = 0, i <= corner_location.size(); i++) {
    if(corner_location[i] == !0) {
      x_location_in_laser_frame.push_back(x[corner_location[i]+1]);
      y_location_in_laser_frame.push_back(y[corner_location[i]+1]);
    }
  }

  double x_middle = 0;
  double y_middle = 0;
  geometry_msgs::msg::PoseStamped crate_front_plane_in_scan_frame;

  // NOTE: or should it be "x_location_in_laser_frame.size() -1"
  for(int i = 0; i <= x_location_in_laser_frame.size(); i++) {
    // use pythagoras to calculate the distance between two corncers.
    double temp = sqrt(pow((x_location_in_laser_frame[i+1]-x_location_in_laser_frame[i]), 2 ) +
                       pow((y_location_in_laser_frame[i+1]-y_location_in_laser_frame[i]), 2 ));

    if((crate_measure_width_ - 0.02) >= temp <= (crate_measure_width_ + 0.02)) {
      // plane to drive up to is found
      // store middle point between both points to use in future operations
      x_middle = (x_location_in_laser_frame[i+1]+x_location_in_laser_frame[i])/2;
      y_middle = (y_location_in_laser_frame[i+1]+y_location_in_laser_frame[i])/2;
      crate_front_plane_in_scan_frame.pose[] = [x_middle, y_middle, 0, 0, 0, 0, 0];
      // move crate_front_plane_in_scan_frame out of loop
      // check if x_middle and y_middle
    } else if((crate_measure_legnth_ - 0.02) >= temp <= (crate_measure_legnth_ + 0.02)) {
      // wrong side of crate is found
      // TODO(anyone): could calculate the front of the crate from
      // this plane, ask Wilco if this is wanted
      // NOTE: This will not be done within this project to keep it less complex!
    } else {
      // TODO(anyone): log middle of the crate not found, try again from another pose!
    }
  }

  // obtain pointer to tf_buffer object
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  std::string scan_frame = "base_scan";
  std::string world_frame = "map";
  // obtain tranfsomation matrix
  transform = tf_->lookupTransform(scan_frame, world_frame, tf2::TimePointZero);

  geometry_msgs::msg::PoseStamped crate_front_plane_in_world_frame;
  crate_front_plane_in_world_frame.pose = transform * crate_front_plane_in_scan_frame.pose;

  // TODO(anyone): calculate point approximate X meters in front of this plane
  // NOTE: what to do with multiply middle point of the
  // crate plane with by transformation matrix
  // TODO(anyone): fix matrix to fit the use of placing
  // the point X meters in front of crate no matter
  // what orientation the crate has (generate dynamic T-matrix?)
  geometry_msgs::msg::PoseStamped
  two_meters_in_front_transform.pose[] = [0, offset_, 0, 0, 0, 0, 0];

  // TODO(anyone): send nav-goal to which is in format of "geometry_msgs/PoseStamped Message"
  auto goal_updater_pub = node_->create_publisher<
  geometry_msgs::msg::PoseStamped>("goal_update", 10);

  auto start = node_->now();
  while ((node_->now() - start).seconds() < 0.5) {
    tree_->rootNode()->executeTick();
    goal_updater_pub->publish(goal_to_update);
    rclcpp::spin_some(node_);
  }
  // from: https://github.com/ros-planning/navigation2/blob/0cdf4f44c7a4171bfa4c6eb885d46c3c19f26a87/nav2_behavior_tree/test/plugins/decorator/test_goal_updater_node.cpp#L119
  crate_found = true;
  // return BT::NodeStatus::SUCCESS;

  // TODO(anyone): isolate logic and build a env to test and maby plot the corner locations
}

/* 
  function to make ranges with certain incremental value
   NOTE: keep in mind that the the list returned
   could be one slot smaller than the list with ranges.
   TODO(anyone): check if temp_value start at 0 or at the first increment value???
   TODO(anyone): list al member function in hpp file
*/
std::list<float> CrateDetectedCondition::arange(float start, float end, float increment)
{
  std::list<float> temp_list[360] = {0.0};
  float temp_value = start;
  for(int i = start; i <= 360; i++) {
      temp_list(i) = temp_value;
      temp_value += increment;
  }
  return temp_list
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::CrateDetectedCondition>("CrateDetected");
}
