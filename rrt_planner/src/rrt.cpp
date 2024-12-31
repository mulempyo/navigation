#include <rrt_global_planner/rrt.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <algorithm>
#include <mutex>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(rrt::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace rrt {

RRTPlanner::RRTPlanner() : initialized_(false), goal_threshold_(0.5), step_size_(0.05) {}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    : initialized_(false), goal_threshold_(0.5), step_size_(0.05) {
  initialize(name, costmap_ros);
}

RRTPlanner::~RRTPlanner() {
  if (world_model_) { //pointer == NULL, return false;
    delete world_model_;
  }
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {

  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("rrt_plan",1);
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
    resolution_ = costmap_->getResolution();
    width_ = costmap_->getSizeInCellsX();
    height_ = costmap_->getSizeInCellsY();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
    initialized_ = true;
  } else {
    ROS_WARN("RRTPlanner has already been initialized.");
  }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                          std::vector<geometry_msgs::PoseStamped> &plan) {
  boost::mutex::scoped_lock lock(mutex_);                           

  if (!initialized_) {
    ROS_ERROR("RRTPlanner has not been initialized, please call initialize() before use.");
    return false;
  }

  plan.clear();
  tree.clear();

  tf::Stamped<tf::Pose> goal_tf;
  poseStampedMsgToTF(goal, goal_tf);
  double unuse_pitch, unuse_roll, goal_yaw;
  goal_tf.getBasis().getEulerYPR(goal_yaw, unuse_pitch, unuse_roll);

  if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
    ROS_ERROR(
        "The RRT planner can only accept goals in the %s frame, "
        "but a goal was sent in the %s frame.",
        costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    return false;
  }

  // Convert world coordinates to map coordinates
  unsigned int start_x, start_y, goal_x, goal_y;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
    ROS_WARN("The start is out of the map bounds.");
    return false;
  }

  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    ROS_WARN("The goal is out of the map bounds.");
    return false;
  }

  unsigned int start_index = start_y * width_ + start_x; //map
  unsigned int goal_index = goal_y * width_ + goal_x; //map

  tree.emplace_back(start_index, start_index);  
  unsigned int final_node_index = 0;

  int i = 0;
  int max_iterations_ = 100000000;

  while (i < max_iterations_) {
    // Generate a random valid pose
    double random_x, random_y, random_th;
    createRandomValidPose(random_x, random_y, random_th);
    unsigned int nearest_index = nearestNode(random_x, random_y);
 
    if(nearest_index == -1){
      ROS_WARN("no valid nearest node found");
      return false;
    }

    double nearest_x, nearest_y;
    costmap_->mapToWorld(nearest_index % width_,nearest_index / width_,nearest_x,nearest_y);

    double th, new_x, new_y, new_th; //th: empty theta
    createPoseWithinRange(nearest_x, nearest_y, th,
                          random_x, random_y, random_th, step_size_,
                          new_x, new_y, new_th);

   if(isValidPathBetweenPoses(nearest_x, nearest_y, th, new_x, new_y, new_th)){
      unsigned int new_x_int, new_y_int, new_index;
      costmap_->worldToMap(new_x,new_y,new_x_int,new_y_int);
      new_index = new_y_int * width_ + new_x_int;

      if(new_index != nearest_index && std::find_if(tree.begin(), tree.end(),
      [new_index](const std::pair<unsigned int, unsigned int> &node) {return node.first == new_index; }) == tree.end()){
        tree.emplace_back(new_index, nearest_index); 
      }else{
        ROS_WARN("skipping invalid or duplicate node: %d", new_index);
        continue;
      }
      

    if(isValidPathBetweenPoses(new_x, new_y, new_th, goal.pose.position.x, goal.pose.position.y, goal_yaw)){
      final_node_index = goal_index;
      tree.emplace_back(goal_index, new_index);
      break;
     }
   }
    i++;
  }

  if(final_node_index == 0){
      ROS_WARN("final_node_index == 0, failed to find a valid path");
      return false;
  }

  //ROS_WARN("start final_node_index != 0");

  if (final_node_index != 0) {
    unsigned int current_index = final_node_index;
    double wx,wy;
    unsigned int mx,my; 
    mx = current_index % width_;
    my = current_index / width_;
    costmap_->mapToWorld(mx,my,wx,wy);
    while (current_index != start_index) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = costmap_ros_->getGlobalFrameID();  
      pose.header.stamp = ros::Time::now();
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0;
      pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0,0,0,1));
      plan.push_back(pose);

      auto it = std::find_if(tree.begin(), tree.end(), // "it" type is std::vector<int>::iterator, "it" round tree.begin() ~ tree.end()
     [current_index](const std::pair<unsigned int, unsigned int> &node) { //node reference tree
     return node.first == current_index;}); // node.first == current_index is true, current_index = it->second;
     
      if(it == tree.end()){
        ROS_WARN("failed to find next node for current_index:%d",current_index);
        break;
      }
      if(it->first == it->second){
        ROS_WARN("cycle detected in tree at index:%d",it->first);
        break;
      }
      ROS_INFO("current_index:%d, next_index:%d",current_index, it->second);
      current_index = it->second; 

      mx = current_index % width_;
      my = current_index / width_;
      costmap_->mapToWorld(mx,my,wx,wy); 
    }

/* tree pair (child, parent)
tree = { {5, 3}, {3, 2}, {2, 1} }; //  5 is parent -  3 is child of 5, 3 is parent - 2 is child of 3, 2 is parent - 1 is child of 2

// first current_index is 5 (goal node)
current_index = 5;

auto it = std::find_if(tree.begin(), tree.end(),
     [current_index](const std::pair<unsigned int, unsigned int> &node) {
         return node.first == current_index; }); 
// current_index == 5 so it->second is 3 (3 is child of 5)

current_index = it->second;  // now current_index becomes 3

// if repeat it, current_index becomes 1 (start node)
*/
    std::reverse(plan.begin(), plan.end());
    publishPlan(plan);
    return true;
  } else {
    ROS_WARN("Failed to find a valid path");
    return false;
  }
}

double RRTPlanner::footprintCost(double x, double y, double th) const {
  if (!initialized_) {
    ROS_ERROR("The RRT Planner has not been initialized, you must call initialize().");
    return -1.0;
  }

  std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

  if (footprint.size() < 3) return -1.0;

  double footprint_cost = world_model_->footprintCost(x, y, th, footprint);

  return footprint_cost;
}

bool RRTPlanner::isValidPose(double x, double y, double th) const {
  double footprint_cost = footprintCost(x, y, th);
  if ((footprint_cost < 0) || (footprint_cost > 128)) {
    return false;
  }
  return true;
}

void RRTPlanner::createRandomValidPose(double &x, double &y, double &th) const {
  // get bounds of the costmap in world coordinates
  double wx_min, wy_min;
  costmap_->mapToWorld(0, 0, wx_min, wy_min);

  double wx_max, wy_max;
  unsigned int mx_max = costmap_->getSizeInCellsX();
  unsigned int my_max = costmap_->getSizeInCellsY();
  costmap_->mapToWorld(mx_max, my_max, wx_max, wy_max);

  bool found_pose = false;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  while (!found_pose) {
    double wx_rand = dis(gen);
    wx_rand = wx_min + wx_rand * (wx_max - wx_min);

    double wy_rand = dis(gen);
    wy_rand = wy_min + wy_rand * (wy_max - wy_min);

    double th_rand = dis(gen);
    th_rand = -M_PI + th_rand * (M_PI - -M_PI);

    if (isValidPose(wx_rand, wy_rand, th_rand)) {
      x = wx_rand;
      y = wy_rand;
      th = th_rand;
      found_pose = true;
    }
  }
}

unsigned int RRTPlanner::nearestNode(double random_x, double random_y) {
  unsigned int nearest_index = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto &node : tree) {
    unsigned int node_index = node.first;
    double node_x,node_y;
    costmap_->mapToWorld(node_index % width_,node_index / width_,node_x,node_y);
    double dist = distance(node_x, node_y, random_x, random_y);
    ROS_INFO("tree node:%d -> %d", node.first, node.second);
    if (dist < min_dist && dist > 0.001) { 
      min_dist = dist;
      nearest_index = node_index;
    }
  }
  return nearest_index;
}

void RRTPlanner::createPoseWithinRange(double start_x, double start_y, double start_th,
                                       double end_x, double end_y, double end_th,
                                       double range, double &new_x, double &new_y, double &new_th) const { //world

  double x_step = end_x - start_x;
  double y_step = end_y - start_y;
  double mag = sqrt((x_step * x_step) + (y_step * y_step));

  if (mag < 0.001) {
    new_x = end_x;
    new_y = end_y;
    new_th = end_th;
    return;
  }

  x_step /= mag;
  y_step /= mag;

  new_x = start_x + x_step * range;
  new_y = start_y + y_step * range;
  new_th = start_th;
}

bool RRTPlanner::isValidPathBetweenPoses(double x1, double y1, double th1,
                                         double x2, double y2, double th2) const {
  double interp_step_size = 0.05;
  double current_step = interp_step_size;

  double d = std::hypot(x2 - x1, y2 - y1);

  while (current_step < d) {
    double interp_x, interp_y, interp_th;
    createPoseWithinRange(x1, y1, th1, x2, y2, th2, current_step,
                          interp_x, interp_y, interp_th);

    if (!isValidPose(interp_x, interp_y, interp_th)) return false;
     
    current_step += interp_step_size;
    }
  
  return true;
}

void RRTPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path) const {
  if (!initialized_) {
    ROS_ERROR(
        "The RRT Planner has not been initialized, you must call "
        "initialize().");
    return;
  }

  nav_msgs::Path path_visual;
  path_visual.poses.resize(path.size());
  ROS_INFO("path size: %f",path.size());

  if (path.empty()) {
    path_visual.header.frame_id = costmap_ros_->getGlobalFrameID();
    path_visual.header.stamp = ros::Time::now();
  } else {
    path_visual.header.frame_id = path[0].header.frame_id;
    path_visual.header.stamp = path[0].header.stamp;
  }

  for (unsigned int i = 0; i < path.size(); i++) {
    path_visual.poses[i] = path[i];
  }

  plan_pub_.publish(path_visual);
}



double RRTPlanner::distance(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

void RRTPlanner::mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) {
  wx = origin_x_ + mx * resolution_;
  wy = origin_y_ + my * resolution_;
}

};  // namespace rrt

    
