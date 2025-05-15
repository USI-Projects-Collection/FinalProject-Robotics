| Funzione     | Topic           | Type                        | Publisher     | User |
| ------------ | --------------- | --------------------------- | ---------------------- | ----------- |
| Mappa 2-D    | `/map`          | `nav_msgs/OccupancyGrid`    | Mapper (o finto ↘︎)    | Planner     |
| Pose attuale | `/odom` o `/tf` | `nav_msgs/Odometry` / `tf`  | Robo (sim)             | Planner     |
| Goal         | `/goal_pose`    | `geometry_msgs/PoseStamped` | Nodo “missione” o RViz | Planner     |
| Traiettoria  | `/plan`         | `nav_msgs/Path`             | Planner                | (debug)     |
| Comandi      | `/cmd_vel`      | `geometry_msgs/Twist`       | Planner                | Robot       |








ros2 topic pub --once /start_pose geometry_msgs/PoseStamped \
"{header:{frame_id: map}, pose:{position:{x: -1.5, y: -1.5, z: 0.0}, orientation:{w: 1.0}}}"

ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \    
"{header:{frame_id: map}, pose:{position:{x: 3.0, y: 3.0, z: 0.0}, orientation:{w: 1.0}}}"