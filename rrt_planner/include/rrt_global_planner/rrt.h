#ifndef RRT_PLANNER_
#define RRT_PLANNER_

#include <ros/ros.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <boost/thread/mutex.hpp>

namespace rrt {

class RRTPlanner : public nav_core::BaseGlobalPlanner {
public:
    RRTPlanner();
    RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    ~RRTPlanner();

    ros::Publisher plan_pub_;

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool makePlan(const geometry_msgs::PoseStamped& start, 
                  const geometry_msgs::PoseStamped& goal, 
                  std::vector<geometry_msgs::PoseStamped>& plan) override;

private:

    double footprintCost(double x, double y, double th) const;
    bool isValidPose(double x, double y, double th) const;
    void createRandomValidPose(double &x, double &y, double &th) const;
    unsigned int nearestNode(double random_x, double random_y);
    void createPoseWithinRange(double start_x, double start_y, double start_th,
                                       double end_x, double end_y, double end_th,
                                       double range,
                                       double &new_x, double &new_y, double &new_th) const;
    bool isValidPathBetweenPoses(double x1, double y1, double th1,
                                         double x2, double y2, double th2) const;
    void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path) const;
    double distance(double x1, double y1, double x2, double y2);
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy);
 
    boost::mutex mutex_;
    std::vector<std::pair<unsigned int, unsigned int>> tree;
    costmap_2d::Costmap2D* costmap_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    double origin_x_, origin_y_, resolution_;
    unsigned int width_, height_;
    bool initialized_;
    double goal_threshold_;
    double step_size_;
    unsigned int max_iterations_;
    base_local_planner::CostmapModel* world_model_ = nullptr;
 };
};
#endif
