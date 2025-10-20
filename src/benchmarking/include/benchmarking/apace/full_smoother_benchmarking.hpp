#ifndef FULL_SMOOTHING_BENCHMARKING_HPP
#define FULL_SMOOTHING_BENCHMARKING_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>

#include <benchmarking/apace/full_smoother.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "octomap_msgs/msg/octomap.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <trajectory_msgs/msg/se3_trajectory.hpp>

#include "uncertain_octomap/uncertain_octomap/full_uncertain_octomap_node.hpp"

namespace benchmarking {
namespace apace {
    class FullSmootherBenchmarkingNode : public rclcpp::Node {
    public:

        FullSmootherBenchmarkingNode();
        ~FullSmootherBenchmarkingNode(){};

        void publishMarkers(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1> >> points);

        std::shared_ptr<FullUncertainOctomapNode> getOctomapNode() const { return m_octomap_node; };

        
    private:

        typedef Eigen::Matrix<float,6,6> poseCovMat_;
        typedef Eigen::Matrix<float,3,3> rotMat_;
        typedef Eigen::Vector<float,3> posVec_;
        typedef Eigen::Quaternionf quat_;

        typedef FullSmoother::splinePair splinePair;

        std::shared_ptr<FullSmoother> m_planner;

        std::shared_ptr<FullUncertainOctomapNode> m_octomap_node;
    
        void getConfigParameterFile();
        std::string m_config_parameter_file;

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_pose_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_tracked_pose_sub;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_marker_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_splines_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
        rclcpp::Publisher<trajectory_msgs::msg::SE3Trajectory>::SharedPtr se3_trajectory_publisher_;

        void pose_with_covariance_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

        void tracked_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        
        nav_msgs::msg::Path extractPath(ompl::base::PathPtr path_ptr);
        
        poseCovMat_ latestCovariance;
        posVec_ latestPosition, trackedPosition, lastPositon;
        rotMat_ latestRotation, trackedRotation;

        bool initialized_{false};
        bool globalGoalSet_{false};

        double lastTime_{0.0};
        Eigen::Vector3d trackedVelocity_{0, 0, 0};

        bool tracking_began{false};
        
        void loadMap(std::string map_path);

        void planPath();

        void publishPath();
        void publishSplines(std::vector<splinePair>& splines);
        void publishMap();

        void visualizeTrajectory(std::vector<splinePair>& splines);

        rclcpp::Time convertTime(double time_in_seconds);

    };

} // namespace apace
} // namespace benchmarking


#endif // FULL_SMOOTHING_BENCHMARKING_HPP