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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>

#include <dynamic_reconfigure/server.h>
#include <velodyne_multi_laserscan/VelodyneMultiLaserScanConfig.h>

namespace velodyne_multi_laserscan {

class VelodyneMultiLaserScan
{
public:
  VelodyneMultiLaserScan(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

private:
  boost::mutex connect_mutex_;
  void connectCb();
  void recvCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  VelodyneMultiLaserScanConfig cfg_;
  dynamic_reconfigure::Server<VelodyneMultiLaserScanConfig> srv_;
  void reconfig(VelodyneMultiLaserScanConfig &config, uint32_t);

  //! Offsets in PointCloud2 message
  struct Offsets
  {
    int x = -1;
    int y = -1;
    int z = -1;
    int i = -1;
    int r = -1;
  };

  static Offsets get_offsets(const sensor_msgs::PointCloud2ConstPtr &msg);
};

} // namespace velodyne_multi_laserscan
