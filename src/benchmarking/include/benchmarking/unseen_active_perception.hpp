#ifndef __UNSEEN_ACTIVE_PERCEPTION_HPP
#define __UNSEEN_ACTIVE_PERCEPTION_HPP

#include "rclcpp/rclcpp.hpp"

#include "planner/uncertainPlanner.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "octomap_msgs/msg/octomap.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <nav_msgs/msg/path.hpp>

#include "uncertain_octomap/uncertain_octomap/uncertain_octomap_node.hpp"



class UnseenActivePerception : public rclcpp::Node
{
    public:

        UnseenActivePerception();
        UnseenActivePerception(double map_resolution);

        ~UnseenActivePerception() = default;

        std::shared_ptr<UncertainOctomapNode> getOctomapNode() const { return m_octomap_node; };

        void setMapFrameId(const std::string& map_frame_id) { m_octomap_node->setMapFrameId(map_frame_id); };

        void publishMarkers(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 1> >> points);

        void timer_callback();


    private:

        typedef Eigen::Matrix<float,6,6> poseCovMat_;
        typedef Eigen::Matrix<float,3,3> rotMat_;
        typedef Eigen::Vector<float,3> posVec_;
        typedef Eigen::Quaternionf quat_;

        std::shared_ptr<UncertainPlanner> m_planner;


        std::shared_ptr<UncertainOctomapNode> m_octomap_node;

        void getConfigParameterFile();
        std::string m_config_parameter_file;

        std::string m_map_frame_id;

        float octree_resolution{0.1};

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_pose_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_tracked_pose_sub;


        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_marker_pub;



        void pose_with_covariance_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

        void tracked_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg);



        std::mutex pose_mutex_;

        poseCovMat_ latestCovariance;
        posVec_ latestPosition, trackedPosition, lastPositon;
        rotMat_ latestRotation, trackedRotation;

        std::vector<posVec_> posLog_;
        std::vector<rotMat_> rotLog_;

    
        bool initialized_{false};
        Eigen::Vector3d previousGoal_{0,0,0}, currentGoal_{0,0,0}, previousStart_{0, 0, 0}, next_position_{0, 0, 0}, next_waypoint_{0, 0, 0};
        Eigen::Quaterniond previousGoalAtt_, currentGoalAtt_, next_orientation_{Eigen::Quaterniond::Identity()};
        Eigen::Vector3d trackedVelocity_{0, 0, 0};

        double lastTime_{0.0};

        bool previousGoalInitialized_{false};

        bool localGoalReached_{true}, globalGoalSet_{false}, replanTrigger_{true}, globalGoalReached_{false};

        int n_fails_{0}, n_max_fails_{3};

        int timer_duration_{3000}; // milliseconds

        bool tracking_began{false};


        



};
#endif // __UNSEEN_ACTIVE_PERCEPTION_HPP