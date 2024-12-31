#include <astar_planner/astar.h>
#include <nav_core/base_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <mutex>
#include <thread>

// Register the A* planner as a plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, nav_core::BaseGlobalPlanner)

namespace astar_planner {

    AStarPlanner::AStarPlanner() : costmap_(nullptr), initialized_(false) {}

    AStarPlanner::AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
        : costmap_(nullptr), initialized_(false) {
        initialize(name, costmap_ros);
    }

    AStarPlanner::~AStarPlanner() {}

    void AStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
        if (!initialized_) {
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("astar_plan", 1);
            costmap_ = costmap_ros->getCostmap();
            origin_x_ = costmap_->getOriginX();
            origin_y_ = costmap_->getOriginY();
            resolution_ = costmap_->getResolution();
            width_ = costmap_->getSizeInCellsX();
            height_ = costmap_->getSizeInCellsY();
            global_frame_ = costmap_ros->getGlobalFrameID();
            initialized_ = true;
        } else {
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
    }

    bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                                const geometry_msgs::PoseStamped& goal, 
                                std::vector<geometry_msgs::PoseStamped>& plan) {
        boost::mutex::scoped_lock lock(mutex_); 

        if (!initialized_) {
            ROS_ERROR("AStarPlanner has not been initialized, please call initialize() before use.");
            return false;
        }

        plan.clear();

        double wx = start.pose.position.x;
        double wy = start.pose.position.y;

        unsigned int start_x, start_y, goal_x, goal_y;
        if (!costmap_->worldToMap(wx, wy, start_x, start_y)) {
            ROS_WARN("The start is out of the map bounds.");
            return false;
        }

        wx = goal.pose.position.x;
        wy = goal.pose.position.y;

        if (!costmap_->worldToMap(wx, wy, goal_x, goal_y)) {
            ROS_WARN("The goal is out of the map bounds.");
            return false;
        }

        std::vector<unsigned int> path = aStarSearch(start_x, start_y, goal_x, goal_y);

        if (path.empty()) {
            ROS_WARN("Failed to find a valid plan.");
            return false;
        }

        if(!path.empty()){
          path.erase(path.begin());
        }

        for (unsigned int i = 0; i < path.size(); ++i) {
            unsigned int index = path[i];
            unsigned int x = index % width_;
            unsigned int y = index / width_;
            double world_x, world_y;
            mapToWorld(x, y, world_x, world_y);
            geometry_msgs::PoseStamped pose = goal;
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
            plan.push_back(pose);
         }
          publishPlan(plan);
          return true;
     }
   
    void AStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR("AStarPlanner has not been initialized, please call initialize() before use.");
            return;
        }
 
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());
        
        if (path.empty()) {
            gui_path.header.frame_id = global_frame_;
            gui_path.header.stamp = ros::Time::now();
        } else { 
            gui_path.header.frame_id = path[0].header.frame_id;
            gui_path.header.stamp = path[0].header.stamp;
        }

        for (unsigned int i = 0; i < path.size(); i++) {
            gui_path.poses[i] = path[i];
        }

        plan_pub_.publish(gui_path);
    }

    std::vector<unsigned int> AStarPlanner::aStarSearch(unsigned int start_x, unsigned int start_y, unsigned int goal_x, unsigned int goal_y) {
        unsigned int start_index = start_y * width_ + start_x;
        unsigned int goal_index = goal_y * width_ + goal_x;

        std::vector<double> g_cost(width_ * height_, std::numeric_limits<double>::infinity());
        std::vector<unsigned int> came_from(width_ * height_, 0);
        std::priority_queue<std::pair<double, unsigned int>, std::vector<std::pair<double, unsigned int>>, std::greater<std::pair<double, unsigned int>>> open_list;

        g_cost[start_index] = 0.0;
        open_list.emplace(heuristic(start_x, start_y, goal_x, goal_y), start_index);

        while (!open_list.empty()) {
            unsigned int current_index = open_list.top().second;
            unsigned int current_x = current_index % width_;
            unsigned int current_y = current_index / width_;
            open_list.pop();

            if (current_index == goal_index) {
                return reconstructPath(came_from, current_index);
            }

            for (const auto& neighbor_index : getNeighbors(current_x, current_y)) {
                unsigned int neighbor_x = neighbor_index % width_;
                unsigned int neighbor_y = neighbor_index / width_;
                double tentative_g_cost = g_cost[current_index] + distance(current_x, current_y, neighbor_x, neighbor_y);

                if (tentative_g_cost < g_cost[neighbor_index]) {
                    g_cost[neighbor_index] = tentative_g_cost;
                    double f_cost = tentative_g_cost + heuristic(neighbor_x, neighbor_y, goal_x, goal_y);
                    open_list.emplace(f_cost, neighbor_index);
                    came_from[neighbor_index] = current_index;
                }
            }
        }

        return std::vector<unsigned int>(); // Return empty path if no path found
    }

    std::vector<unsigned int> AStarPlanner::getNeighbors(unsigned int x, unsigned int y) {
        std::vector<unsigned int> neighbors;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;

                int nx = static_cast<int>(x) + dx;
                int ny = static_cast<int>(y) + dy;

                if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width_) && ny < static_cast<int>(height_) &&
                    costmap_->getCost(nx, ny) == costmap_2d::FREE_SPACE) {
                    neighbors.push_back(ny * width_ + nx);
                }
            }
        }
        return neighbors;
    }

    std::vector<unsigned int> AStarPlanner::reconstructPath(const std::vector<unsigned int>& came_from, unsigned int current_index) {
    std::vector<unsigned int> path;
    while (current_index != came_from[current_index]) { 
        path.insert(path.begin(), current_index);       
        current_index = came_from[current_index];
    }
    path.insert(path.begin(), current_index); 
    return path;
}


    double AStarPlanner::potentialFieldCost(unsigned int x, unsigned int y) const {
        double cost = 0.0;
        for (int dx = -2; dx <= 2; ++dx) {
            for (int dy = -2; dy <= 2; ++dy) {
                int nx = static_cast<int>(x) + dx;
                int ny = static_cast<int>(y) + dy;
                if (nx >= 0 && ny >= 0 && nx < static_cast<int>(width_) && ny < static_cast<int>(height_)) {
                    double distance_to_obstacle = std::hypot(dx, dy);
                    if (distance_to_obstacle > 0.0) {
                        double obstacle_cost = costmap_->getCost(nx, ny);
                        if (obstacle_cost == costmap_2d::LETHAL_OBSTACLE) {
                            cost += 1.0 / distance_to_obstacle;  
                        }
                    }
                }
            }
        }
        return cost;
    }

    double AStarPlanner::heuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const {
        double euclidean_distance = std::hypot(static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));
        double potential_cost = potentialFieldCost(x1, y1);
        return euclidean_distance + potential_cost;
    }

    double AStarPlanner::distance(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const {
        return std::hypot(static_cast<double>(x2 - x1), static_cast<double>(y2 - y1));
    }

    void AStarPlanner::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const {
        wx = origin_x_ + mx * resolution_;
        wy = origin_y_ + my * resolution_;
    }
};
