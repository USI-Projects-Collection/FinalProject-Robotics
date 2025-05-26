# Clone the reopsitory:
```bash
git clone --recurse-submodules https://github.com/USI-Projects-Collection/FinalProject-Robotics.git
```

# How to run the project:
## Terminal1
```bash
pixi install 
pixi shell
colcon build --symlink-install
source install/setup.[bash|zsh]
pixi run coppelia src/orbit_tower/scenes/final_scene_without_obstacles.ttt
```
(in case the package coppeliasim_zmqremoteapi_client isn't being added correctly manually add it with pixi add)
## Terminal2
```bash
pixi shell
source install/setup.[bash|zsh]
⁠ros2 launch orbit_tower s1.launch name:=/rm0
```

## Terminal3
```bash
pixi shell
source install/setup.[bash|zsh]
⁠ros2 launch orbit_tower controller.launch name:=/rm0
```

# Maps of the topics used in the project
| Funzione     | Topic               | Type                        | Publisher                | User          |
| ------------ | ------------------- | --------------------------- | ------------------------ | ------------- |
| Mappa 2-D    | `/map`              | `nav_msgs/OccupancyGrid`    | MockMapPublisher         | Planner       |
| Pose attuale | `/odom`             | `nav_msgs/Odometry`         | Robo (sim)               | Planner       |
| Posa corrente| `/current_pose`     | `geometry_msgs/PoseStamped` | Planner                  | Robot         |
| Goal         | `/goal_pose`        | `geometry_msgs/PoseStamped` | MockMapPublisher / RViz  | Planner       |
| Traiettoria  | `/plan`             | `nav_msgs/Path`             | Planner                  | (debug)       |
| Comandi vel  | `/rm0/cmd_vel`      | `geometry_msgs/Twist`       | Planner                  | Robot         |
| Goal raggiunto| `/goal_reached`    | `std_msgs/Bool`             | Planner                  | MockMapPublisher |
| Riprendi path| `/go_again`         | `std_msgs/Bool`             | MockMapPublisher         | Planner       |
| Velocità     | `/cmd_vel`          | `geometry_msgs/Twist`       | ControllerNode           | Robo          |
| Immagini     | `/rm0/camera/image_color` | `sensor_msgs/Image`   | Robo (sim)               | ControllerNode |
| Rilevamento  | `/rm0/range_[0-3]`  | `sensor_msgs/Range`         | Robo (sim)               | ControllerNode |


## Example of publishing topics to move the robot
ros2 topic pub --once /current_pose geometry_msgs/PoseStamped \
"{header:{frame_id: map}, pose:{position:{x: -1.5, y: -1.5, z: 0.0}, orientation:{w: 1.0}}}"

ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \    
"{header:{frame_id: map}, pose:{position:{x: 3.0, y: 3.0, z: 0.0}, orientation:{w: 1.0}}}"