# PX4 ROS2 SIMULATION ENVIRONMENT

## Components: 
- PX4 Autopilot

Follow installation guidelines from [PX4 Autopilot](https://docs.px4.io/main/en/dev_setup/building_px4)

- QGround Control
FOllow installation guidelines from [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

- ROS2 Humble
- Gazebo

Use gazebo version such that it uses `gz` command.

- UNSEEN ORB-SLAM3
Follow installation guidelines from [ORB-SLAM3](https://github.com/tamasso-parec/PUMA_ORB_SLAM3)

## Submodules
Update all submodules in the repository:
```bash
git submodule update --init --recursive
```

## Set global paths: 
In order to run the components, it is necessary to set global paths for PX4, QgroundControl and ORB-SLAM3. 

### PX4

Link the executable to the path [here](https://github.com/idra-lab/mav-px4-sim/blob/d965ba2ed0c13ac6f0bc79b526a841b4da07abd3/src/traj/launch/px4_sim.launch.py#L76)

### QGroundControl
Link the executable to the path [here](https://github.com/idra-lab/mav-px4-sim/blob/d965ba2ed0c13ac6f0bc79b526a841b4da07abd3/src/traj/launch/px4_sim.launch.py#L158)

### ORB-SLAM3 Node

Link the CMake configuration file to the global path of your orb slam implementation [here](https://github.com/tamasso-parec/ORB_SLAM3_ROS2/blob/acc59d7baaf6d8ae4471b16fb15cf530cb32481d/CMakeModules/FindORB_SLAM3.cmake#L9)

## Launch demo



### Build
```bash
source /opt/ros/<ROS_DISTRO>/setup.bash

colcon build

colcon build --symlink-install --packages-select orbslam3

source install/local_setup.bash
```

### Launch 

#### Terminal 1
```bash
ros2 launch uncertain_planner drone_planner.launch.py
```
#### Terminal 2
```bash
ros2 run traj offboard_takeoff --ros-args -p altitude:=4.0
```
#### Terminal 3
```bash
ros2 launch traj sim_demo.launch.py
```

## Adding models to PX4 

- Add an entry with a suitable number and name in [`~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes`](../../../PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/)
- Add the model into [`src/drone_description/models`](src/drone_description/models/)
- Update the [`CMakeLists.txt`](../../../PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt) adding the name of the airframe
- Make sure to check the appropriate flags are defined in [`~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator`](../../../PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/px4-rc.simulator)


## Launch simulation environment

Starting the simulation environment with the drone model and the world:
```bash
gz service -s /world/warehouse/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'
data: true
```


```bash
ros2 launch traj px4_sim.launch.py
```

## Takeoff

```bash
ros2 run traj offboard_takeoff --ros-args -p  altitude:=4.0
```

## OFFBOARD MODE

```bash
ros2 run traj offboard_control
```

## Land

```bash
ros2 run traj land_disarm
```

## Launch planner

```bash
ros2 run uncertain_planner drone_planner slam_map
```