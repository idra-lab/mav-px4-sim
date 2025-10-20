#include <cstdio>
#include <benchmarking/apace/full_smoother_benchmarking.hpp>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::string map_frame_id = "slam_map";
  
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;

  std::shared_ptr<benchmarking::apace::FullSmootherBenchmarkingNode> full_smoother_benchmarking_node = std::make_shared<benchmarking::apace::FullSmootherBenchmarkingNode>();

  std::cout << "FULL SMOOTHER BENCHMARKING NODE RUNNING" << std::endl;

  executor.add_node(full_smoother_benchmarking_node);
  executor.add_node(full_smoother_benchmarking_node->getOctomapNode());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
