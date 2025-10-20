Mapping Instructions

ros2 launch nav_manager sim_demo.launch.py 


ros2 run traj map_center --ros-args -p radius:=10.0 -p altitude:=3.0 -p omega:=0.2 -p center_position:='[0.0, 10.0]' 

ros2 run uncertain_octomap map_saver


Planner with loading map: 

ros2 run benchmarking full_smoother_benchmarking_node --ros-args -p configuration_filename:=src/benchmarking/config/benchmark_config.yaml -p map_path:=src/benchmarking/resource/maps/cube_benchmark/map_creator_file.yaml
