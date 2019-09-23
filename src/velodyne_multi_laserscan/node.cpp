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

#include <ros/ros.h>
#include <velodyne_multi_laserscan/velodyne_multi_laserscan.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_laserscan_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  velodyne_multi_laserscan::VelodyneMultiLaserScan n(node, priv_nh);

  ros::spin();

  return 0;
}
