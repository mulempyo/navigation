#ifndef PURE_PURSUIT_PLANNER_H
#define PURE_PURSUIT_PLANNER_H

#include <ros/ros.h>
#include <angles/angles.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/local_planner_util.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

namespace pure_pursuit_planner {

class PurePursuitPlanner : public nav_core::BaseLocalPlanner {
public:
    PurePursuitPlanner();
    
    PurePursuitPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    ~PurePursuitPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

private:
    base_local_planner::LocalPlannerUtil planner_util_;
    bool is_initialized_;
    bool goal_reached_;
    tf2_ros::Buffer* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    double lookahead_distance_;
    double max_velocity_;
    double min_velocity_;
    double max_angular_velocity_;
    double min_angular_velocity_;
    double xy_goal_tolerance_;
    double yaw_goal_tolerance_;
    geometry_msgs::PoseStamped current_pose_;
   
    geometry_msgs::PoseStamped getLookaheadPoint(const geometry_msgs::PoseStamped& current_pose_, std::vector<geometry_msgs::PoseStamped>& transformed_plan);
    double getDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2);
    double getYaw(const geometry_msgs::PoseStamped& pose);
};

}  // namespace pure_pursuit_planner

#endif  // PURE_PURSUIT_PLANNER_H
