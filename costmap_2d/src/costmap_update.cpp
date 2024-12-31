#include <costmap_2d/costmap_update.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace update{

void MapUpdate::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan){
   laser_scan_ = laser_scan;
}

void MapUpdate::updateCostmap(costmap_2d::Costmap2D* costmap){  //using global planner
    if (!laser_scan_) {
        ROS_WARN("No laser scan data received.");
        return;
    }

    std::vector<geometry_msgs::Point> obstacles;
    const sensor_msgs::LaserScan& scan = *laser_scan_;
    double angle = scan.angle_min;

    for (const auto& range : scan.ranges) {
        if (range >= scan.range_min && range <= scan.range_max) {
            geometry_msgs::Point obstacle;
            obstacle.x = range * std::cos(angle);
            obstacle.y = range * std::sin(angle);
            obstacle.z = 0.0;
            obstacles.push_back(obstacle);
        }
        angle += scan.angle_increment;
    }

    unsigned int size_x = costmap->getSizeInCellsX();
    unsigned int size_y = costmap->getSizeInCellsY();

    for (const auto& obs : obstacles) {
        unsigned int mx, my;
        if (costmap->worldToMap(obs.x, obs.y, mx, my)) {
            costmap->setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }

    for (unsigned int i = 0; i < size_x; ++i) {
        for (unsigned int j = 0; j < size_y; ++j) {
            if (costmap->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE) {
                bool still_obstacle = false;
                for (const auto& obs : obstacles) {
                    unsigned int mx, my;
                    if (costmap->worldToMap(obs.x, obs.y, mx, my) && mx == i && my == j) {
                        still_obstacle = true;
                        break;
                    }
                }
                if (!still_obstacle) {
                    costmap->setCost(i, j, costmap_2d::FREE_SPACE);
                }
            }
        }
    }
}

}//namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "update_map");
    ros::NodeHandle nh;
    update::MapUpdate updator;
    
    updator.sub = nh.subscribe("scan", 5, &update::MapUpdate::laserReceived, &updator);

    ros::spin();
    return 0;
}