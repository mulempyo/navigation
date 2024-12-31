#ifndef UPDATE_MAP_H
#define UPDATE_MAP_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace update{
    class MapUpdate{

      public:
      void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
      void updateCostmap(costmap_2d::Costmap2D* costmap);

      sensor_msgs::LaserScanConstPtr laser_scan_;
      ros::Subscriber sub;
    };
}

#endif
