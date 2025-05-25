# To clone the reopsitory:
```bash
git clone --recurse-submodules https://github.com/USI-Projects-Collection/FinalProject-Robotics.git
```

# To run the project:
### Terminal1
```bash
pixi run coppelia src/assignment2/scenes/final_scene_without_obstacles.ttt
```
### Terminal2
```bash
⁠ros2 launch assignment2 s1.launch name:=/rm0
```

### Terminal3
```bash
⁠ros2 launch assignment2 controller.launch name:=/rm0
```



## Esempio di pubblicazione dei topic per direzionare il robot
ros2 topic pub --once /current_pose geometry_msgs/PoseStamped \
"{header:{frame_id: map}, pose:{position:{x: -1.5, y: -1.5, z: 0.0}, orientation:{w: 1.0}}}"

ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \    
"{header:{frame_id: map}, pose:{position:{x: 3.0, y: 3.0, z: 0.0}, orientation:{w: 1.0}}}"



## Maps of the topics used in the project
| Funzione     | Topic           | Type                        | Publisher     | User |
| ------------ | --------------- | --------------------------- | ---------------------- | ----------- |
| Mappa 2-D    | `/map`          | `nav_msgs/OccupancyGrid`    | Mapper (o finto ↘︎)    | Planner     |
| Pose attuale | `/odom` o `/tf` | `nav_msgs/Odometry` / `tf`  | Robo (sim)             | Planner     |
| Goal         | `/goal_pose`    | `geometry_msgs/PoseStamped` | Nodo “missione” o RViz | Planner     |
| Traiettoria  | `/plan`         | `nav_msgs/Path`             | Planner                | (debug)     |
| Comandi      | `/cmd_vel`      | `geometry_msgs/Twist`       | Planner                | Robot       |