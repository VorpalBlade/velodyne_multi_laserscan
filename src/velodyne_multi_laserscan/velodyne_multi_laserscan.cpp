// Velodyne Multi LaserScan - Merges multiple rings into a 2D laser scan
// Copyright (C) 2019  Arvid Norlander
// Copyright (C) 2018, 2019 Kevin Hallenbeck, Joshua Whitley
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "velodyne_multi_laserscan/velodyne_multi_laserscan.h"

#include <sensor_msgs/point_cloud2_iterator.h>

namespace velodyne_multi_laserscan {

VelodyneMultiLaserScan::VelodyneMultiLaserScan(ros::NodeHandle &nh,
                                               ros::NodeHandle &nh_priv)
  : nh_(nh)
  , srv_(nh_priv)
{
  ros::SubscriberStatusCallback connect_cb =
      boost::bind(&VelodyneMultiLaserScan::connectCb, this);
  pub_ = nh.advertise<sensor_msgs::LaserScan>(
      "multi_scan", 10, connect_cb, connect_cb);

  srv_.setCallback(
      boost::bind(&VelodyneMultiLaserScan::reconfig, this, _1, _2));
}

void VelodyneMultiLaserScan::connectCb()
{
  // This allows us to only do work if someone is subscribed to us.
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (!pub_.getNumSubscribers()) {
    sub_.shutdown();
  } else if (!sub_) {
    sub_ = nh_.subscribe(
        "velodyne_points", 10, &VelodyneMultiLaserScan::recvCallback, this);
  }
}

//! Implements the core logic, shared between fast version and iterator version.
#define CORE_LOGIC(x_expr, y_expr, z_expr, i_expr)                             \
  const float z = z_expr;                                                      \
                                                                               \
  if (z > cfg_.min_z && z < cfg_.max_z) {                                      \
    const float x = x_expr;                                                    \
    const float y = y_expr;                                                    \
                                                                               \
    const int32_t bin = static_cast<int32_t>(                                  \
        (atan2f(y, x) + static_cast<float>(M_PI)) / RESOLUTION);               \
                                                                               \
    if ((bin >= 0) && (bin < static_cast<int32_t>(SIZE))) {                    \
      auto new_range = sqrtf(x * x + y * y);                                   \
      if (new_range < scan->ranges[static_cast<size_t>(bin)]) {                \
        const float i = i_expr;                                                \
        scan->ranges[static_cast<size_t>(bin)] = new_range;                    \
        scan->intensities[static_cast<size_t>(bin)] = i;                       \
      }                                                                        \
    }                                                                          \
  }

void VelodyneMultiLaserScan::recvCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // Load structure of PointCloud2
  auto offsets = get_offsets(msg);

  if (offsets.x < 0 || offsets.y < 0 || offsets.z < 0 || offsets.i < 0) {
    ROS_ERROR(
        "VelodyneMultiLaserScan: PointCloud2 missing one or more required "
        "fields! (x,y,z,intensity)");
    return;
  }

  const float RESOLUTION = static_cast<float>(std::abs(cfg_.resolution));
  const size_t SIZE = static_cast<size_t>(2.0 * M_PI / RESOLUTION);
  sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan());
  scan->header = msg->header;
  scan->angle_increment = RESOLUTION;
  scan->angle_min =
      static_cast<decltype(sensor_msgs::LaserScan::angle_min)>(-M_PI);
  scan->angle_max =
      static_cast<decltype(sensor_msgs::LaserScan::angle_max)>(M_PI);
  scan->range_min = 0.0;
  scan->range_max = 200.0;
  scan->time_increment = 0.0;
  scan->ranges.resize(SIZE, INFINITY);

  if ((offsets.x == 0) && (offsets.y == 4) && (offsets.z == 8) &&
      (offsets.i == 16) && (offsets.r == 20)) {
    scan->intensities.resize(SIZE);

    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x");
         it != it.end();
         ++it) {
      CORE_LOGIC(it[0], it[1], it[2], it[4])
    }

  } else {
    ROS_WARN_ONCE("VelodyneMultiLaserScan: PointCloud2 fields in unexpected "
                  "order. Using slower generic method.");

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");

    scan->intensities.resize(SIZE);

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
      CORE_LOGIC(*iter_x, *iter_y, *iter_z, *iter_i)
    }
  }
  pub_.publish(scan);
}

void VelodyneMultiLaserScan::reconfig(VelodyneMultiLaserScanConfig &config,
                                      uint32_t level)
{
  cfg_ = config;
}

VelodyneMultiLaserScan::Offsets
VelodyneMultiLaserScan::get_offsets(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  Offsets offsets;

  for (size_t i = 0; i < msg->fields.size(); i++) {
    if (msg->fields[i].datatype == sensor_msgs::PointField::FLOAT32) {
      if (msg->fields[i].name == "x") {
        offsets.x = static_cast<int>(msg->fields[i].offset);
      } else if (msg->fields[i].name == "y") {
        offsets.y = static_cast<int>(msg->fields[i].offset);
      } else if (msg->fields[i].name == "z") {
        offsets.z = static_cast<int>(msg->fields[i].offset);
      } else if (msg->fields[i].name == "intensity") {
        offsets.i = static_cast<int>(msg->fields[i].offset);
      }
    } else if (msg->fields[i].datatype == sensor_msgs::PointField::UINT16) {
      if (msg->fields[i].name == "ring") {
        offsets.r = static_cast<int>(msg->fields[i].offset);
      }
    }
  }

  return offsets;
}

} // namespace velodyne_multi_laserscan
