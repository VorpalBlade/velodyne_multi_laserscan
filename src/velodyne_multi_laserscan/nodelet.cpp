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
#include <memory>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace velodyne_multi_laserscan {

class MultiLaserScanNodelet : public nodelet::Nodelet
{
public:
  MultiLaserScanNodelet() {}
  ~MultiLaserScanNodelet() {}

private:
  virtual void onInit()
  {
    node_.reset(
        new VelodyneMultiLaserScan(getNodeHandle(), getPrivateNodeHandle()));
  }

  std::shared_ptr<VelodyneMultiLaserScan> node_;
};

} // namespace velodyne_multi_laserscan

PLUGINLIB_EXPORT_CLASS(velodyne_multi_laserscan::MultiLaserScanNodelet,
                       nodelet::Nodelet);
