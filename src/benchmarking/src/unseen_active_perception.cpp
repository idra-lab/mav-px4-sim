#include "benchmarking/unseen_active_perception.hpp"



UnseenActivePerception::UnseenActivePerception()
: Node("unseen_active_perception_node")
{
    getConfigParameterFile();
 
    m_planner = std::make_shared<UncertainPlanner>(m_config_parameter_file);

    this->declare_parameter("map_frame_id", "map");
    m_map_frame_id = this->get_parameter("map_frame_id").as_string();

    m_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose_with_covariance",  10, std::bind(&UnseenActivePerception::pose_with_covariance_callback, this, std::placeholders::_1));
    
    m_tracked_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/slam/tracked_pose",  10, std::bind(&UnseenActivePerception::tracked_pose_callback, this, std::placeholders::_1));
    
    m_octomap_node = std::make_shared<UncertainOctomapNode>(m_planner->getPlannerConfig());

    m_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/obstacles_markers", 10);

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/drone_path", rclcpp::QoS(1).transient_local());

    lastTime_ = this->now().seconds();

    timer_duration_ = m_planner->getPlannerConfig().smoother_horizon;

    // this->timer_callback();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_duration_), std::bind(&UnseenActivePerception::timer_callback, this));
    // rclcpp::spin(m_octomap_node);


    this->setMapFrameId(m_map_frame_id);

}

void UnseenActivePerception::getConfigParameterFile()
{
    this->declare_parameter("configuration_filename", "");

    m_config_parameter_file = this->get_parameter("configuration_filename").as_string();

    if (m_config_parameter_file.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No configuration file provided, exiting");
        return;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Configuration file: %s", m_config_parameter_file.c_str());
    }
}

void UnseenActivePerception::tracked_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    // m_planner->setTrackedPose(msg);
    std::lock_guard<std::mutex> lock(pose_mutex_);
    trackedPosition = posVec_(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    trackedRotation = rotMat_(quat_(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z));

    trackedVelocity_ = (trackedPosition - lastPositon).cast<double>() / (this->now().seconds() - lastTime_);

    next_position_ = (trackedPosition.cast<double>() + trackedVelocity_ * timer_duration_ / 1000.0 + next_waypoint_)/2.0;
    next_orientation_ = quat_(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z).cast<double>();
    

    lastPositon = trackedPosition;

    m_planner->setCurrentPosition(trackedPosition.cast<double>());
    m_planner->setCurrentVelocity(trackedVelocity_);

    tracking_began = true;
    
    double dist_to_global_goal = (trackedPosition.cast<double>() - m_planner->getGlobalGoal()).norm();
  

    if (dist_to_global_goal < m_planner->getGoalTolerance())
    {
        RCLCPP_INFO(this->get_logger(), "Global goal reached, shutting down planner");
        rclcpp::shutdown();
        return;
    }

}

void UnseenActivePerception::pose_with_covariance_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // m_planner->setPoseWithCovariance(msg);
}




void UnseenActivePerception::publishMarkers(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1> >> points)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "slam_map";
    marker.header.stamp = this->now();
    marker.ns = "drone_planner";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    for (const auto& point : points)
    {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        marker.points.push_back(p);
    }

    m_marker_pub->publish(marker);
}

void UnseenActivePerception::timer_callback()
{
    // if (!replanTrigger_)
    // {return;}

    if (!tracking_began)
    {return;}

    auto start_time = std::chrono::high_resolution_clock::now();

    m_planner->reset();

    m_planner->updateMap(m_octomap_node->getOctomap());


    RCLCPP_INFO(this->get_logger(), "Setting start at Next Position: [%f, %f, %f]", next_position_[0], next_position_[1], next_position_[2]);

    {
        // std::lock_guard<std::mutex> lock(pose_mutex_);

        // if (!m_planner->setStart(trackedPosition.cast<double>(), trackedRotation.cast<double>()))
        if (!m_planner->setStart(next_position_, next_orientation_))
        {
            RCLCPP_ERROR(this->get_logger(), "Start state invalid");
            replanTrigger_ = true;
            
            // previousGoalInitialized_ = false;
            return;

        }

    }   
    
    if (!m_planner->setGoal(m_planner->getGlobalGoal(), m_planner->getGlobalGoalQuat()))
        {
            RCLCPP_ERROR(this->get_logger(), "Goal state invalid");
            // previousGoalInitialized_ = false;
            return;
        }else if (!globalGoalSet_)
        {
                globalGoalSet_ = true;
        }
   

    if (!m_planner->plan())
    {
        RCLCPP_WARN(this->get_logger(), "\n--------------------------------------------\nPlanning failed\n--------------------------------------------\n");
        
        replanTrigger_ = true;
        return; 
    }else
    {replanTrigger_ = false;}

        
    auto path = m_planner->getSmoothPath();

    if (path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No path found");
        replanTrigger_ = true;
        return;
    }

    publishMarkers(m_planner->getObstacles());

    
    m_planner->save2File("planner_test_result.yaml");

    nav_msgs::msg::Path path_msg;


    path_msg.header.frame_id = "slam_map";
    path_msg.header.stamp = this->now();
    path_msg.poses.resize(path.size());

    // RCLCPP_INFO(this->get_logger(), "Path size: %zu", path.size());
    for (size_t i = 0; i < path.size(); ++i)
    {
        // RCLCPP_INFO(this->get_logger(), "Path point %zu: %f %f %f", i, path[i][0], path[i][1], path[i][2]);
        path_msg.poses[i].pose.position.x = path[i][0];
        path_msg.poses[i].pose.position.y = path[i][1];
        path_msg.poses[i].pose.position.z = path[i][2];
        path_msg.poses[i].pose.orientation.w = path[i][3];
        path_msg.poses[i].pose.orientation.x = path[i][4];
        path_msg.poses[i].pose.orientation.y = path[i][5];
        path_msg.poses[i].pose.orientation.z = path[i][6];
        int64_t seconds = static_cast<int64_t>(path[i][7]);
        int64_t nanoseconds = static_cast<int64_t>((path[i][7] - seconds) * 1e9);
        path_msg.poses[i].header.stamp = rclcpp::Time(seconds, nanoseconds);
        // RCLCPP_INFO(this->get_logger(), "Pose Time:  %f", path[i][7]);
        path_msg.poses[i].header.frame_id = "slam_map";
    }

    path_publisher_->publish(path_msg);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(this->get_logger(), "Planning took %ld ms", duration);

    size_t closest_idx = 0;
    double min_time_diff = std::numeric_limits<double>::max();

    double delta_time_planning = 2*duration/1000.0; // in seconds
    // double next_waypoint_time = timer_duration_ / 1000.0 + delta_time_planning; // Convert milliseconds to seconds
    double next_waypoint_time = timer_duration_ / 1000.0 - delta_time_planning; // Convert milliseconds to seconds

    for (size_t i = 0; i < path.size(); ++i) {
        double time_diff = path[i][7] - next_waypoint_time;


        if (time_diff < min_time_diff && time_diff > 0) 
        {
            min_time_diff = time_diff;

            closest_idx = i;

        }
    }

    RCLCPP_INFO(this->get_logger(), "Closest index to next waypoint time: %zu", closest_idx);
    
    auto& waypoint = path[closest_idx+1];


    // closest_idx now holds the index where path[i][7] is closest to timer_duration_ (in milliseconds)
    next_waypoint_ = (waypoint.head<3>().cast<double>()); // Average with tracked position
    // next_orientation_ = Eigen::Quaterniond(waypoint[3], waypoint[4], waypoint[5], waypoint[6]);

        

    
}
