
#include "mpc_planner_ros.h"
#include <pluginlib/class_list_macros.h>

using namespace std;
using namespace Eigen;

PLUGINLIB_EXPORT_CLASS(mpc_ros::MPCPlannerROS, nav_core::BaseLocalPlanner)

namespace mpc_ros{

    MPCPlannerROS::MPCPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}
	MPCPlannerROS::MPCPlannerROS(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        // initialize planner
        initialize(name, tf, costmap_ros);
    }
	MPCPlannerROS::~MPCPlannerROS() {}

	void MPCPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){

        ros::NodeHandle private_nh("~/" + name);

		tf_ = tf;
		costmap_ros_ = costmap_ros;
        //initialize the copy of the costmap the controller will use
        costmap_ = costmap_ros_->getCostmap();
        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();
        footprint_spec_ = costmap_ros_->getRobotFootprint();
        
        planner_util_.initialize(tf, costmap_, costmap_ros_->getGlobalFrameID());
        
        if( private_nh.getParam( "odom_frame", _odom_frame ))
        {
            odom_helper_.setOdomTopic( _odom_frame );
        }

        //Assuming this planner is being run within the navigation stack, we can
        //just do an upward search for the frequency at which its being run. This
        //also allows the frequency to be overwritten locally.
        ros::NodeHandle nh_;
        std::string controller_frequency_param_name;
        double controller_frequency = 0;
        if(!nh_.searchParam("move_base/controller_frequency", controller_frequency_param_name)) {
            ROS_WARN("controller_frequency_param_name doesn't exits");
        } else {
            nh_.param(controller_frequency_param_name, controller_frequency, 20.0);
            
            if(controller_frequency > 0) {
            } else {
                ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
            }
        }
        //private_nh.param("vehicle_Lf", _Lf, 0.290); // distance between the front of the vehicle and its center of gravity
        _dt = double(1.0/controller_frequency); // time step duration dt in s 

        //Parameter for topics & Frame name
        private_nh.param<std::string>("map_frame", _map_frame, "map" ); 
        private_nh.param<std::string>("odom_frame", _odom_frame, "odom");
        private_nh.param<std::string>("base_frame", _base_frame, "base_footprint");

        private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.2);


        //Publishers and Subscribers
        _sub_odom   = _nh.subscribe("odom", 1, &MPCPlannerROS::odomCB, this);
        global_plan_pub_   = _nh.advertise<nav_msgs::Path>("mpc_planner", 1);
        _pub_mpctraj = _nh.advertise<nav_msgs::Path>("mpc_trajectory",1);
        _pub_odompath = _nh.advertise<nav_msgs::Path>("mpc_reference",1);

        //Init variables
        _throttle = 0.0; 
        _w = 0.0;
        _speed = 0.0;

        //_ackermann_msg = ackermann_msgs::AckermannDriveStamped();
        _twist_msg = geometry_msgs::Twist();
        _mpc_traj = nav_msgs::Path();

        
        dsrv_ = new dynamic_reconfigure::Server<MPCPlannerConfig>(private_nh);
        dynamic_reconfigure::Server<MPCPlannerConfig>::CallbackType cb = boost::bind(&MPCPlannerROS::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    }

  void MPCPlannerROS::reconfigureCB(MPCPlannerConfig &config, uint32_t level) {
      // update generic local planner params
      base_local_planner::LocalPlannerLimits limits;
      limits.max_vel_trans = config.max_vel_trans;
      limits.min_vel_trans = config.min_vel_trans;
      limits.max_vel_x = config.max_vel_x;
      limits.min_vel_x = config.min_vel_x;
      limits.max_vel_y = config.max_vel_y;
      limits.min_vel_y = config.min_vel_y;
      limits.max_vel_theta = config.max_vel_theta;
      limits.min_vel_theta = config.min_vel_theta;
      limits.acc_lim_x = config.acc_lim_x;
      limits.acc_lim_y = config.acc_lim_y;
      limits.acc_lim_theta = config.acc_lim_theta;
      limits.acc_lim_trans = config.acc_lim_trans;
      limits.xy_goal_tolerance = config.xy_goal_tolerance;
      limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
      limits.prune_plan = config.prune_plan;
      limits.trans_stopped_vel = config.trans_stopped_vel;
      limits.theta_stopped_vel = config.theta_stopped_vel;

      //Parameter for MPC solver
      _debug_info = config.debug_info;
      _delay_mode = config.delay_mode;
      _max_speed = config.max_speed;
      _waypointsDist = config.waypoints_dist;
      _pathLength = config.path_length;
      _mpc_steps = config.steps;
      _ref_cte = config.ref_cte;
      _ref_vel = config.ref_vel;
      _ref_etheta = config.ref_etheta;
      _w_cte = config.w_cte;
      _w_etheta = config.w_etheta;
      _w_vel = config.w_vel;
      _w_angvel = config.w_angvel;
      _w_angvel_d = config.w_angvel_d;
      _w_accel_d = config.w_accel_d;
      _w_accel = config.w_accel;
      _max_angvel = config.max_angvel;
      _max_throttle = config.max_throttle;
      _bound_value = config.bound_value;

  }
  
	bool MPCPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
        if( ! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        goal_reached_ = false;
        rotate = true;

        ROS_WARN("start Plan");

        //Init parameters for MPC object
        _mpc_params["DT"] = _dt;
        //_mpc_params["LF"] = _Lf;
        _mpc_params["STEPS"]    = _mpc_steps;
        _mpc_params["REF_CTE"]  = _ref_cte;
        _mpc_params["REF_ETHETA"] = _ref_etheta;
        _mpc_params["REF_V"]    = _ref_vel;
        _mpc_params["W_CTE"]    = _w_cte;
        _mpc_params["W_EPSI"]   = _w_etheta;
        _mpc_params["W_V"]      = _w_vel;
        _mpc_params["W_ANGVEL"]  = _w_angvel;
        _mpc_params["W_A"]      = _w_accel;
        _mpc_params["W_DANGVEL"] = _w_angvel_d;
        _mpc_params["W_DA"]     = _w_accel_d;
        _mpc_params["ANGVEL"]   = _max_angvel;
        _mpc_params["MAXTHR"]   = _max_throttle;
        _mpc_params["BOUND"]    = _bound_value;
        _mpc.LoadParams(_mpc_params);
        //Display the parameters
        cout << "\n===== Parameters =====" << endl;
        cout << "debug_info: "  << _debug_info << endl;
        cout << "delay_mode: "  << _delay_mode << endl;
        //cout << "vehicle_Lf: "  << _Lf << endl;
        cout << "frequency: "   << _dt << endl;
        cout << "mpc_steps: "   << _mpc_steps << endl;
        cout << "mpc_ref_vel: " << _ref_vel << endl;
        cout << "mpc_w_cte: "   << _w_cte << endl;
        cout << "mpc_w_etheta: "  << _w_etheta << endl;
        cout << "mpc_max_angvel: "  << _max_angvel << endl;

        planner_util_.setPlan(orig_global_plan);
        
	return true;
    }

    bool MPCPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
   
    if (!costmap_ros_->getRobotPose(current_pose_)) {
        ROS_ERROR("Could not get robot pose");
        return false;
    }

    double robot_pose_x = current_pose_.pose.position.x;
    double robot_pose_y = current_pose_.pose.position.y;

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!planner_util_.getLocalPlan(current_pose_, transformed_plan)) {
        ROS_ERROR("Could not get local plan");
        return false;
    }

    if (transformed_plan.empty()) {
        ROS_WARN_NAMED("mpc_planner", "Received an empty transformed plan.");
        return false;
    }
   
    global_plan_.resize(transformed_plan.size());
        for (unsigned int i = 0; i < transformed_plan.size(); ++i) {
            global_plan_[i] = transformed_plan[i];
        }

    geometry_msgs::PoseStamped lookahead_pose = global_plan_.back(); 
    for (const auto& pose : global_plan_) {
    double dx = pose.pose.position.x - robot_pose_x;
    double dy = pose.pose.position.y - robot_pose_y;
    double distance = hypot(dx, dy);
    if (distance == 0.1) { 
      lookahead_pose = pose;
      break;
    }
    }

    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    double robot_yaw = tf2::getYaw(current_pose_.pose.orientation);

  // Calculate the goal direction from the robot's current pose to the goal pose
    double target_yaw = atan2(lookahead_pose.pose.position.y - robot_pose_y, lookahead_pose.pose.position.x - robot_pose_x); 
    double yaw_error = angles::shortest_angular_distance(robot_yaw,target_yaw);
  
    if (rotate) {
    
    if (fabs(yaw_error) > 0.2) {
      
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = yaw_error > 0 ? 0.2 : -0.2;  
      ROS_WARN("Rotating to correct yaw, yaw_error: %f", fabs(yaw_error));
      return true; 
    } else {
      ROS_WARN("Yaw aligned, proceeding to move.");
      rotate = false;  
    }
  }

    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);

    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

        
    bool success = mpcComputeVelocityCommands(current_pose_, robot_vel, drive_cmds);

         if(!success)
         {
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;
          return false;
         } else{

         cmd_vel.linear.x = drive_cmds.pose.position.x;
         cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);

         std::vector<geometry_msgs::PoseStamped> local_plan;

          for(unsigned int i = 0; i < result_traj_.getPointsSize(); ++i) {
            double p_x, p_y, p_th;
            result_traj_.getPoint(i, p_x, p_y, p_th);

            geometry_msgs::PoseStamped p;
            p.header.frame_id = costmap_ros_->getGlobalFrameID();
            p.header.stamp = ros::Time::now();
            p.pose.position.x = p_x;
            p.pose.position.y = p_y;
            p.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p_th);
            tf2::convert(q, p.pose.orientation);
            local_plan.push_back(p);
        }
         publishGlobalPlan(local_plan);
    

         geometry_msgs::PoseStamped robot_pose;
         costmap_ros_->getRobotPose(robot_pose);
         geometry_msgs::PoseStamped global_ = global_plan_.back();

         double dx = robot_pose.pose.position.x - global_.pose.position.x;
         double dy = robot_pose.pose.position.y - global_.pose.position.y;

         if (hypot(dx, dy) <= xy_goal_tolerance_) {
           cmd_vel.linear.x = 0.0;
           cmd_vel.angular.z = 0.0;
           goal_reached_ = true;
           rotate = true;
           ROS_INFO("Goal reached.");
         }
          return true;
         }
       
     }

    // Timer: Control Loop (closed loop nonlinear MPC)
    bool MPCPlannerROS::mpcComputeVelocityCommands(geometry_msgs::PoseStamped global_pose, geometry_msgs::PoseStamped& global_vel, 
    geometry_msgs::PoseStamped& drive_cmds)
    {   
        Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
        Eigen::Vector3f vel(global_vel.pose.position.x, global_vel.pose.position.y, tf2::getYaw(global_vel.pose.orientation));
        
        geometry_msgs::PoseStamped goal_pose = global_plan_.back();
        Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
        base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
         result_traj_.cost_ = 1;
   
        nav_msgs::Odometry base_odom = _odom;

        // Update system states: X=[x, y, theta, v]
        const double px = base_odom.pose.pose.position.x; //pose: odom frame
        const double py = base_odom.pose.pose.position.y;
        tf::Pose pose;
        tf::poseMsgToTF(base_odom.pose.pose, pose);
        double theta = tf::getYaw(pose.getRotation());
        const double v = base_odom.twist.twist.linear.x; //twist: body fixed frame
        // Update system inputs: U=[w, throttle]
        const double w = _w; // steering -> w
        //const double steering = _steering;  // radian
        const double throttle = _throttle; // accel: >0; brake: <0
        const double dt = _dt;

        // Update path waypoints (conversion to odom frame)
        nav_msgs::Path odom_path = nav_msgs::Path();
        try
        {
            double total_length = 0.0;
            int sampling = _downSampling;

            //find waypoints distance
            if(_waypointsDist <=0.0)
            {        
                double dx = global_plan_[1].pose.position.x - global_plan_[0].pose.position.x;
                double dy = global_plan_[1].pose.position.y - global_plan_[0].pose.position.y;
                _waypointsDist = sqrt(dx*dx + dy*dy);
                _downSampling = int(_pathLength/10.0/_waypointsDist);
            }            

            // Cut and downsampling the path
            for(int i =0; i< global_plan_.size(); i++)
            {
                if(total_length > _pathLength)
                    break;

                if(sampling == _downSampling)
                {   
                    geometry_msgs::PoseStamped tempPose;
                    tf2_ros::TransformListener tfListener(*tf_);
                    geometry_msgs::TransformStamped odom_transform;
                    odom_transform = tf_->lookupTransform(_odom_frame, _map_frame, ros::Time(0), ros::Duration(1.0) );
                    tf2::doTransform(global_plan_[i], tempPose, odom_transform); // robot_pose is the PoseStamp                     
                    odom_path.poses.push_back(tempPose);  
                    sampling = 0;
                }
                total_length = total_length + _waypointsDist; 
                sampling = sampling + 1;  
            }

           
            if (!odom_path.poses.empty()) {
            const geometry_msgs::PoseStamped& last_transformed_pose = odom_path.poses.back();
            double min_distance = std::numeric_limits<double>::max();
            int closest_index = -1;
   
            geometry_msgs::PoseStamped robot_pose;
            costmap_ros_->getRobotPose(robot_pose);
            double remaining_distance = 0.0;
            for (int i = 1; i < global_plan_.size(); ++i) {
                double dx = global_plan_[i].pose.position.x - global_plan_[i-1].pose.position.x;
                double dy = global_plan_[i].pose.position.y - global_plan_[i-1].pose.position.y;
                remaining_distance += sqrt(dx * dx + dy * dy);
            }

    
            for (int i = 0; i < global_plan_.size(); ++i) {
                double dx = global_plan_[i].pose.position.x - last_transformed_pose.pose.position.x;
                double dy = global_plan_[i].pose.position.y - last_transformed_pose.pose.position.y;
                double distance = sqrt(dx * dx + dy * dy);

                if (distance < min_distance) {
                    min_distance = distance;
                    closest_index = i;
                }
               }

         
                 if ( _pathLength >= remaining_distance) {
    
                  odom_path.poses.back() = global_plan_.back();
               } else {
        
                   if (closest_index >= 0 && closest_index < global_plan_.size()) {
                      odom_path.poses.back() = global_plan_[closest_index];
                 }
               }

   
            if(closest_index >= 0 && closest_index < global_plan_.size()){
            geometry_msgs::PoseStamped& goal_pose = odom_path.poses.back();
            goal_pose.pose.orientation = global_plan_[closest_index].pose.orientation;
          }
        }


            if(odom_path.poses.size() > 3)
            {
                // publish odom path
                odom_path.header.frame_id = "odom";
                odom_path.header.stamp = ros::Time::now();
                _pub_odompath.publish(odom_path);
            }
            else
            {
                ROS_DEBUG_NAMED("mpc_ros", "Failed to path generation since small down-sampling path.");
                _waypointsDist = -1;
                result_traj_.cost_ = -1;
                return false;
            }
            //DEBUG      
            if(_debug_info){
                cout << endl << "odom_path: " << odom_path.poses.size()
                << ", path[0]: " << odom_path.poses[0]
                << ", path[N]: " << odom_path.poses[odom_path.poses.size()-1] << endl;
            }  
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        // Waypoints related parameters
        const int N = odom_path.poses.size(); // Number of waypoints
        const double costheta = cos(theta);
        const double sintheta = sin(theta);
        
        cout << "px, py : " << px << ", "<< py << ", theta: " << theta << " , N: " << N << endl;
        // Convert to the vehicle coordinate system
        VectorXd x_veh(N);
        VectorXd y_veh(N);
        for(int i = 0; i < N; i++) 
        {
            const double dx = odom_path.poses[i].pose.position.x - px;
            const double dy = odom_path.poses[i].pose.position.y - py;
            x_veh[i] = dx * costheta + dy * sintheta;
            y_veh[i] = dy * costheta - dx * sintheta;
            //cout << "x_veh : " << x_veh[i]<< ", y_veh: " << y_veh[i] << endl;
        }
        
        // Fit waypoints
        auto coeffs = polyfit(x_veh, y_veh, 3); 
        const double cte  = polyeval(coeffs, 0.0);
        cout << "coeffs : " << coeffs[0] << endl;
        cout << "pow : " << pow(0.0 ,0) << endl;
        cout << "cte : " << cte << endl;
        double etheta = atan(coeffs[1]);

        // Global coordinate system about theta
        double gx = 0;
        double gy = 0;
        int N_sample = N * 0.3;
        for(int i = 1; i < N_sample; i++) 
        {
            gx += odom_path.poses[i].pose.position.x - odom_path.poses[i-1].pose.position.x;
            gy += odom_path.poses[i].pose.position.y - odom_path.poses[i-1].pose.position.y;
        }   

        double temp_theta = theta;
        double traj_deg = atan2(gy,gx);
        double PI = 3.141592;

         // Degree conversion -pi~pi -> 0~2pi(ccw) since need a continuity        
        if(temp_theta <= -PI + traj_deg) 
            temp_theta = temp_theta + 2 * PI;

        double theta_diff = temp_theta - traj_deg;
        double max_theta_diff = PI / 4; // 45 degrees in radians
        double min_theta_diff = -PI / 4; // -45 degrees in radians
        // Implementation about theta error more precisly
        if (theta_diff > max_theta_diff) {
        theta_diff = max_theta_diff;
        } else if (theta_diff < min_theta_diff) {
        theta_diff = min_theta_diff;
        }

        // Use the limited theta_diff as etheta
        etheta = theta_diff;  
        cout << "etheta: "<< etheta << ", atan2(gy,gx): " << atan2(gy,gx) << ", temp_theta:" << traj_deg << endl;

        // Difference bewteen current position and goal position
        const double x_err = goal_pose.pose.position.x -  base_odom.pose.pose.position.x;
        const double y_err = goal_pose.pose.position.y -  base_odom.pose.pose.position.y;
        const double goal_err = sqrt(x_err*x_err + y_err*y_err);

        cout << "x_err:"<< x_err << ", y_err:"<< y_err  << endl;
       
        VectorXd state(6); 

        if(_delay_mode)
        {
            const double theta_act = w * dt;
            const double px_act = v * dt;
            const double py_act = 0;
            const double v_act = v + throttle * dt;
            const double cte_act = cte+v*sin(etheta)*dt;
            const double etheta_act = etheta - theta_act;  
            
            state << px_act, py_act, theta_act, v_act, cte_act, etheta_act;
        }
        else
        {
            state << 0, 0, 0, v, cte, etheta;
        }


        // Solve MPC Problem
        ros::Time begin = ros::Time::now();
        vector<double> mpc_results = _mpc.Solve(state, coeffs);    
        ros::Time end = ros::Time::now();
        cout << "Duration: " << end.sec << "." << end.nsec << endl << begin.sec<< "."  << begin.nsec << endl;
            
        // MPC result (all described in car frame), output = (acceleration, w)        
        _w = mpc_results[0]; // radian/sec, angular velocity
        _throttle = mpc_results[1]; // acceleration
        
        _speed = v + _throttle * dt;  // speed
        if (_speed >= _max_speed)
            _speed = _max_speed;
        if(_speed <= 0.0)
            _speed = 0.0;

        if(_debug_info)
        {
            cout << "\n\nDEBUG" << endl;
            cout << "theta: " << theta << endl;
            cout << "V: " << v << endl;
            cout << "coeffs: \n" << coeffs << endl;
            cout << "_w: \n" << _w << endl;
            cout << "_throttle: \n" << _throttle << endl;
            cout << "_speed: \n" << _speed << endl;
        }
        // Display the MPC predicted trajectory
        _mpc_traj = nav_msgs::Path();
        _mpc_traj.header.frame_id = _base_frame; // points in car coordinate        
        _mpc_traj.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped tempPose;
        tf2::Quaternion myQuaternion;

        for(int i=0; i<_mpc.mpc_x.size(); i++)
        {
            tempPose.header = _mpc_traj.header;
            tempPose.pose.position.x = _mpc.mpc_x[i];
            tempPose.pose.position.y = _mpc.mpc_y[i];

            myQuaternion.setRPY( 0, 0, _mpc.mpc_theta[i] );  
            tempPose.pose.orientation.x = myQuaternion[0];
            tempPose.pose.orientation.y = myQuaternion[1];
            tempPose.pose.orientation.z = myQuaternion[2];
            tempPose.pose.orientation.w = myQuaternion[3];
                
            _mpc_traj.poses.push_back(tempPose); 
        }     

        if(result_traj_.cost_ < 0){
            drive_cmds.pose.position.x = 0;
            drive_cmds.pose.position.y = 0;
            drive_cmds.pose.position.z = 0;
            drive_cmds.pose.orientation.w = 1;
            drive_cmds.pose.orientation.x = 0;
            drive_cmds.pose.orientation.y = 0;
            drive_cmds.pose.orientation.z = 0;
        }
        else{
            drive_cmds.pose.position.x = _speed;
            drive_cmds.pose.position.y = 0;
            drive_cmds.pose.position.z = 0;
            tf2::Quaternion q;
            q.setRPY(0, 0, _w);
            tf2::convert(q, drive_cmds.pose.orientation);
        }
        _pub_mpctraj.publish(_mpc_traj);

        if(goal_reached_){
            odom_path.poses.clear();
            ROS_INFO("Path cleared after goal reached.");
        }
        return true;
    }


    bool MPCPlannerROS::isGoalReached()
    {
      // check if plugin is initialized
      if (!initialized_)
      {
        ROS_ERROR("mpc_local_planner has not been initialized, please call initialize() before using this planner");
        return false;
      }
        return goal_reached_;
    }

    // Evaluate a polynomial.
    double MPCPlannerROS::polyeval(Eigen::VectorXd coeffs, double x) 
    {
        double result = 0.0;
        for (int i = 0; i < coeffs.size(); i++) 
        {
            result += coeffs[i] * pow(x, i);
        }
        return result;
    }
    
    Eigen::VectorXd MPCPlannerROS::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) 
    {
        assert(xvals.size() == yvals.size());
        assert(order >= 1 && order <= xvals.size() - 1);
        Eigen::MatrixXd A(xvals.size(), order + 1);

        for (int i = 0; i < xvals.size(); i++)
            A(i, 0) = 1.0;

        for (int j = 0; j < xvals.size(); j++) 
        {
            for (int i = 0; i < order; i++) 
                A(j, i + 1) = A(j, i) * xvals(j);
        }
        auto Q = A.householderQr();
        auto result = Q.solve(yvals);
        return result;
    }

    // CallBack: Update odometry
    void MPCPlannerROS::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        _odom = *odomMsg;
    }


    void MPCPlannerROS::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
      nav_msgs::Path gui_path;
      gui_path.header.frame_id = _map_frame;
      gui_path.header.stamp = ros::Time::now();
      gui_path.poses = global_plan;
      global_plan_pub_.publish(gui_path);
    }
}
