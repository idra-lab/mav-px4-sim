#include <cstdio>
#include <benchmarking/apace/full_smoother_benchmarking.hpp>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;


  rclcpp::init(argc, argv);


  std::shared_ptr<benchmarking::apace::FullSmootherBenchmarkingNode> full_smoother_planner_node = std::make_shared<benchmarking::apace::FullSmootherBenchmarkingNode>();
  rclcpp::spin(full_smoother_planner_node);
  rclcpp::shutdown();

  return 0;
}
