Mapping Instructions

ros2 launch nav_manager sim_demo.launch.py 


ros2 run traj map_center --ros-args -p radius:=10.0 -p altitude:=3.0 -p omega:=0.2 -p center_position:='[0.0, 10.0]' 

ros2 run uncertain_octomap map_saver


Planner with loading map: 

ros2 run benchmarking full_smoother_benchmarking_node --ros-args -p configuration_filename:=src/benchmarking/config/benchmark_config.yaml -p map_path:=src/benchmarking/resource/maps/cube_benchmark/map_creator_file.yaml

# Evo Evaluation Commands
```
cd src/benchmarking/resource/trajectories/cube_benchmark
```

```
evo_traj tum apace/est_trajectory.txt --ref=apace/gt_trajectory.txt -p --save_table apace/traj.csv --save_plot apace/traj.pdf  -c ../../../config/evo_config.json
```
```
evo_traj tum unseen_plan/est_trajectory.txt --ref=unseen_plan/gt_trajectory.txt -p --save_table unseen_plan/traj.csv --save_plot unseen_plan/traj.pdf -c ../../../config/evo_config.json
```
```
evo_ape tum apace/gt_trajectory.txt apace/est_trajectory.txt  -va --plot  --save_results apace/ape.zip -c ../../../config/evo_config.json
```
```
evo_ape tum unseen_plan/gt_trajectory.txt unseen_plan/est_trajectory.txt  -va --plot  --save_results unseen_plan/ape.zip -c ../../../config/evo_config.json
```
```
evo_rpe tum apace/gt_trajectory.txt apace/est_trajectory.txt  -va --plot  --save_results apace/rpe.zip -c ../../../config/evo_config.json
```
```
evo_rpe tum unseen_plan/gt_trajectory.txt unseen_plan/est_trajectory.txt  -va --plot  --save_results unseen_plan/rpe.zip -c ../../../config/evo_config.json
```

```
evo_res apace/ape.zip unseen_plan/ape.zip --save_table ape.csv --save_plot ape.pdf --use_filenames -c ../../../config/evo_config.json
```
```
evo_res apace/rpe.zip unseen_plan/rpe.zip --save_table rpe.csv --save_plot rpe.pdf --use_filenames -c ../../../config/evo_config.json
```
