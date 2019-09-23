# velodyne_multi_laserscan

Node that takes multiple rings of the Velodyne scan and flattens it down,
selecting the closest point in each direction. The result is published as a
laser scan.

This operates within the same TF frame as the Velodyne, there is no support for
sensors not mounted horizontally.

You can set the minimum and maximum z values to filter out the floor and
ceiling. Dynamic reconfiguration is supported.

## Topics

Subscribed:
* `velodyne_points` (`sensor_msgs/PointCloud2`) Used as data source.

Published:
* `multi_scan` (`sensor_msgs/LaserScan`) Merged scan is published to this topic.

## Parameters

* `~min_z` (`double`) - Minimum z for obstacles (m)
* `~max_z` (`double`) - Maximum z for obstacles (m)
* `~resolution` (`double`) - Laser scan angular resolution (rad)
