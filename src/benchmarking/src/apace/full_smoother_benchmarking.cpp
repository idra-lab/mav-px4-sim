#include <benchmarking/apace/full_smoother_benchmarking.hpp>

namespace benchmarking {
namespace apace {


FullSmootherBenchmarkingNode::FullSmootherBenchmarkingNode() : Node("full_smoother_benchmarking_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting Full Smoother Benchmarking Node...");

    getConfigParameterFile();

    m_planner = std::make_shared<FullSmoother>(m_config_parameter_file);

    m_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/slam/pose_with_covariance",  10, std::bind(&FullSmootherBenchmarkingNode::pose_with_covariance_callback, this, std::placeholders::_1));
    m_tracked_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/slam/tracked_pose",  10, std::bind(&FullSmootherBenchmarkingNode::tracked_pose_callback, this, std::placeholders::_1));

    m_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/obstacles_markers", 10);
    m_splines_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/spline_markers", 10);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("drone_path", rclcpp::QoS(1).transient_local());
    se3_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::SE3Trajectory>("drone_se3_trajectory", rclcpp::QoS(1).transient_local());
    m_octomap_node = std::make_shared<FullUncertainOctomapNode>(m_planner->getPlannerConfig());

    m_octomap_node->setMapFrameId("slam_map");

    m_octomap_node->destroySubscribers();    
    this->declare_parameter<std::string>("map_path", "planned_map.yaml");
    
    std::string map_path = this->get_parameter("map_path").as_string();
    
    // RCLCPP_INFO(this->get_logger(), "NOT LOADING THE MAP");
    this->loadMap(map_path);
    RCLCPP_INFO(this->get_logger(), "Full Smoother Benchmarking Node started.");

    
}


void FullSmootherBenchmarkingNode::getConfigParameterFile()
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

void FullSmootherBenchmarkingNode::tracked_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

    trackedPosition = posVec_(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    trackedRotation = rotMat_(quat_(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z));

    trackedVelocity_ = (trackedPosition - lastPositon).cast<double>() / (this->now().seconds() - lastTime_);

    
    tracking_began = true;
    
    double dist_to_global_goal = (trackedPosition.cast<double>() - m_planner->getGlobalGoal()).norm();

    

    if (dist_to_global_goal < m_planner->getGoalTolerance())
    {
        RCLCPP_INFO(this->get_logger(), "Global goal reached, shutting down planner");
        rclcpp::shutdown();
        return;
    }

    if (mapLoaded_)
    {planPath();}
}

void FullSmootherBenchmarkingNode::pose_with_covariance_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // m_planner->setPoseWithCovariance(msg);
}


void FullSmootherBenchmarkingNode::publishMarkers(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1> >> points)
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

void FullSmootherBenchmarkingNode::planPath()
{
   
    

    auto start_time = std::chrono::high_resolution_clock::now();

    m_planner->reset();
    m_planner->updateMap(m_octomap_node->getOctomap());


    {

        if (!m_planner->setStart(trackedPosition.cast<double>(), trackedRotation.cast<double>()))
        {
            RCLCPP_ERROR(this->get_logger(), "Start state invalid");
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
        
        return; 
    }

    // TODO: Get B-Splines from here
    std::vector<splinePair> splines_path = m_planner->getSplines();

    if (splines_path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No path found");
        return;
    }

    
    auto end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    RCLCPP_INFO(this->get_logger(), "Planning took %ld ms", duration);
    
    publishSplines(splines_path);

    visualizeTrajectory(splines_path);

    saveSplines("planned_splines.yaml", splines_path);
    this->m_octomap_node->publishMapAsCubes(this->now());
    this->m_octomap_node->saveMapAsPcl("planned_map.pcd");

    rclcpp::shutdown();

    
}


void FullSmootherBenchmarkingNode::loadMap(std::string map_path)
{
    RCLCPP_INFO(this->get_logger(), "Loading map from file: %s", map_path.c_str());
    m_octomap_node->readFromFile(map_path);
    RCLCPP_INFO(this->get_logger(), "Map loaded.");
    mapLoaded_ = true;

}

void FullSmootherBenchmarkingNode::loadMapFromPCL(std::string map_path)
{
    RCLCPP_INFO(this->get_logger(), "Loading map from PCL file: %s", map_path.c_str());
    m_octomap_node->readFromPCLFile(map_path);
    RCLCPP_INFO(this->get_logger(), "Map loaded from PCL file.");
    mapLoaded_ = true;

}

void FullSmootherBenchmarkingNode::publishSplines(std::vector<splinePair>& splines)
{

    trajectory_msgs::msg::SE3Trajectory se3_trajectory_msg;
    se3_trajectory_msg.header.frame_id = "slam_map";
    se3_trajectory_msg.header.stamp = this->now();
    se3_trajectory_msg.num_splines = splines.size();

    float previousTime{0.0f};

    int cc{0};
    for (const auto& spline_pair : splines)
    {
        const auto& spline = spline_pair.first;
        trajectory_msgs::msg::SE3Spline se3_spline_msg;

        auto transpl = spline_pair.first; 
        auto rotspl = spline_pair.second;

        trajectory_msgs::msg::TranslationSpline translation_spline_msg;
        trajectory_msgs::msg::RotationSpline rotation_spline_msg;

        double start_time = previousTime;
        double final_time = start_time+transpl.getTime();
        translation_spline_msg.start_time = convertTime(start_time);
        rotation_spline_msg.start_time = convertTime(start_time);
        se3_spline_msg.start_time = convertTime(start_time);

        translation_spline_msg.end_time = convertTime(final_time);
        rotation_spline_msg.end_time = convertTime(final_time);
        se3_spline_msg.end_time = convertTime(final_time);

        previousTime = final_time;

        translation_spline_msg.dimensions = 3;
        translation_spline_msg.degree = transpl.getDegree_();
        translation_spline_msg.traj_id = cc;

        rotation_spline_msg.dimensions = 3;
        rotation_spline_msg.degree = rotspl.getDegree_();
        rotation_spline_msg.traj_id = cc++;

        auto transpl_knots = transpl._get_knots_();
        auto transpl_ctrl_points = transpl._getCtrlPoints_();

        auto rotspl_knots = rotspl._get_knots_();
        auto rotspl_ctrl_points = rotspl._getCtrlPoints_();

        translation_spline_msg.knots = std::vector<double>(transpl_knots.data(), transpl_knots.data() + transpl_knots.size());
        rotation_spline_msg.knots = std::vector<double>(rotspl_knots.data(), rotspl_knots.data() + rotspl_knots.size());

        
        for (unsigned int i{0}; i < transpl_ctrl_points.cols(); ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = transpl_ctrl_points(0, i);
            p.y = transpl_ctrl_points(1, i);
            p.z = transpl_ctrl_points(2, i);
            translation_spline_msg.pos_pts.push_back(p);
        }

        for (unsigned int i{0}; i < rotspl_ctrl_points.cols(); ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = rotspl_ctrl_points(0, i);
            p.y = rotspl_ctrl_points(1, i);
            p.z = rotspl_ctrl_points(2, i);
            rotation_spline_msg.rot_pts.push_back(p);
        }

        Eigen::Quaterniond start_quat(rotspl.getStartOrientation_());

        rotation_spline_msg.initial_orientation.w = start_quat.w();
        rotation_spline_msg.initial_orientation.x = start_quat.x();
        rotation_spline_msg.initial_orientation.y = start_quat.y();
        rotation_spline_msg.initial_orientation.z = start_quat.z();

        se3_spline_msg.translation_spline = translation_spline_msg;
        se3_spline_msg.rotation_spline = rotation_spline_msg;
        se3_trajectory_msg.splines.push_back(se3_spline_msg);
    }


    se3_trajectory_publisher_->publish(se3_trajectory_msg);
    

}

rclcpp::Time FullSmootherBenchmarkingNode::convertTime(double time_in_seconds)
{
    int64_t seconds = static_cast<int64_t>(time_in_seconds);
    int64_t nanoseconds = static_cast<int64_t>((time_in_seconds - seconds) * 1e9);
    return rclcpp::Time(seconds, nanoseconds);
}

void FullSmootherBenchmarkingNode::visualizeTrajectory(std::vector<splinePair>& splines)
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    int n_points{200};

    int total_id{0};

    for (const auto& spline_pair : splines)
    {
        const auto& spline = spline_pair.first;
        const auto& rotspl = spline_pair.second;

        auto waypoints = spline.sampleSpline(n_points);

        Eigen::Vector3d previous_waypoint = waypoints[0];

        double max_dist{0.0};

        auto rots = rotspl.sampleSpline(n_points);

        for (int i{1}; i < n_points; ++i)
        {
            Eigen::Vector3d p1 = waypoints[i];
            Eigen::Vector3d p0 = waypoints[i-1];
            double dist = (p1 - p0).norm();
            if (dist > max_dist)
            {
                max_dist = dist;
            }
        }

        for (int i{0}; i < n_points; ++i)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "slam_map";
            marker.header.stamp = this->now();
            marker.ns = std::string("point") + std::to_string(total_id);
            marker.id = total_id++;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.scale.x = 0.02;   // slender shaft diameter
            marker.scale.y = 0.1;   // smaller head diameter
            marker.scale.z = 0.1;    // moderate head length

            
            Eigen::Vector3d diff = waypoints[i] - previous_waypoint;
            double dist = diff.norm();
            double factor = dist/max_dist;

            previous_waypoint = waypoints[i];

            marker.color.r = factor;
            marker.color.g = 0.0f;
            marker.color.b = (1 - factor);
            marker.color.a = 1.0f;

            geometry_msgs::msg::Point p;
            p.x = waypoints[i](0);
            p.y = waypoints[i](1);
            p.z = waypoints[i](2);
            marker.points.push_back(p);

            geometry_msgs::msg::Point p2;
            Eigen::Vector3d dir = rots[i].transpose() * Eigen::Vector3d(0.0, 0.0, 0.5);
            p2.x = waypoints[i](0) + dir(0);
            p2.y = waypoints[i](1) + dir(1);
            p2.z = waypoints[i](2) + dir(2);
            marker.points.push_back(p2);

            marker_array.markers.push_back(marker);
        }
        
    }

    m_splines_pub->publish(marker_array);
}

void FullSmootherBenchmarkingNode::saveSplines(std::string filename, std::vector<splinePair>& splines)
{
    YAML::Node _baseNode;

    int i{0};

    float previousTime{0.0f};

    _baseNode["Planning_time"] = duration;

    for (const auto& spline_pair : splines)
    {
        const auto& spline = spline_pair.first;

        auto transpl = spline_pair.first; 
        auto rotspl = spline_pair.second;


        double start_time = previousTime;
        double final_time = start_time+transpl.getTime();

        _baseNode[i]["Translation"]["start_time"] = start_time;
        _baseNode[i]["Translation"]["end_time"] = final_time;
        _baseNode[i]["Rotation"]["start_time"] = start_time;
        _baseNode[i]["Rotation"]["end_time"] = final_time;

        previousTime = final_time;
        
        _baseNode[i]["Translation"]["dimensions"] = 3;
        _baseNode[i]["Translation"]["degree"] = transpl.getDegree_();
        
        
        
        _baseNode[i]["Rotation"]["dimensions"] = 3;
        _baseNode[i]["Rotation"]["degree"] = rotspl.getDegree_();

        auto transpl_knots = transpl._get_knots_();
        auto transpl_ctrl_points = transpl._getCtrlPoints_();


        _baseNode[i]["Translation"]["Knots"] = std::vector<double>(transpl_knots.data(), transpl_knots.data() + transpl_knots.size());
        _baseNode[i]["Translation"]["ControlPoints"] = std::vector<double>(transpl_ctrl_points.data(), transpl_ctrl_points.data() + transpl_ctrl_points.size());
        
        
        auto rotspl_knots = rotspl._get_knots_();
        auto rotspl_ctrl_points = rotspl._getCtrlPoints_();
        
        _baseNode[i]["Rotation"]["Knots"] = std::vector<double>(rotspl_knots.data(), rotspl_knots.data() + rotspl_knots.size());
        _baseNode[i]["Rotation"]["ControlPoints"] = std::vector<double>(rotspl_ctrl_points.data(), rotspl_ctrl_points.data() + rotspl_ctrl_points.size());


        Eigen::Quaterniond start_quat(rotspl.getStartOrientation_());

        _baseNode[i]["Rotation"]["InitialOrientation"]["w"] = start_quat.w();
        _baseNode[i]["Rotation"]["InitialOrientation"]["x"] = start_quat.x();
        _baseNode[i]["Rotation"]["InitialOrientation"]["y"] = start_quat.y();
        _baseNode[i]["Rotation"]["InitialOrientation"]["z"] = start_quat.z();

        ++i;
    }
    

    std::ofstream outFile(filename); 

    outFile << _baseNode;
}


} // namespace apace

} // namespace benchmarking