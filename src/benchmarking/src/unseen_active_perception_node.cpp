#include <cstdio>
#include <benchmarking/unseen_active_perception.hpp>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::string map_frame_id = "map";
  
  if (argc > 1)
  {
      map_frame_id = argv[1];

  }


  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;

  std::shared_ptr<UnseenActivePerception> drone_planner_node = std::make_shared<UnseenActivePerception>();

  drone_planner_node->setMapFrameId(map_frame_id);

  executor.add_node(drone_planner_node);
  executor.add_node(drone_planner_node->getOctomapNode());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
