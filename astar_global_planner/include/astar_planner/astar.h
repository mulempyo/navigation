#ifndef ASTAR_GLOBAL_H
#define ASTAR_GLOBAL_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <queue>
#include <boost/thread/mutex.hpp>
#include <memory>
#include <unordered_map>
#include <thread>

namespace astar_planner {

    class AStarPlanner : public nav_core::BaseGlobalPlanner {
    public:
        AStarPlanner();
        AStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        ~AStarPlanner();

       // Node* createNode(unsigned int x, unsigned int y, double g_cost, double h_cost, Node* parent);
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
        bool makePlan(const geometry_msgs::PoseStamped& start, 
                      const geometry_msgs::PoseStamped& goal, 
                      std::vector<geometry_msgs::PoseStamped>& plan) override;
        ros::Publisher plan_pub_;
        std::string global_frame_;

    private:
        costmap_2d::Costmap2D* costmap_;
        bool initialized_;
        double origin_x_, origin_y_, resolution_;
        unsigned int width_, height_;
        unsigned int goal_x_, goal_y_;

        boost::mutex mutex_;
  
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        std::vector<unsigned int> aStarSearch(unsigned int start_x, unsigned int start_y, unsigned int goal_x, unsigned int goal_y);
        std::vector<unsigned int> getNeighbors(unsigned int x, unsigned int y);
        std::vector<unsigned int> reconstructPath(const std::vector<unsigned int>& came_from, unsigned int current_index);
        double potentialFieldCost(unsigned int x, unsigned int y) const;
        double heuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const;
        double distance(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const;
        void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;
    };
};

#endif // KWJ_GLOBAL_H