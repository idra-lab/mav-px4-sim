#include <benchmarking/apace/full_smoother_benchmarking.hpp>

namespace benchmarking {
namespace apace {
FullSmootherBenchmarkingNode::FullSmootherBenchmarkingNode() : Node("full_smoother_benchmarking_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting Full Smoother Benchmarking Node...");

    // full_smoother_planner_ = std::make_shared<FullSmoother>();

    RCLCPP_INFO(this->get_logger(), "Full Smoother Benchmarking Node started.");
}


}
}