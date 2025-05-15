| Funzione     | Topic           | Type                        | Publisher     | User |
| ------------ | --------------- | --------------------------- | ---------------------- | ----------- |
| Mappa 2-D    | `/map`          | `nav_msgs/OccupancyGrid`    | Mapper (o finto ↘︎)    | Planner     |
| Pose attuale | `/odom` o `/tf` | `nav_msgs/Odometry` / `tf`  | Robo (sim)             | Planner     |
| Goal         | `/goal_pose`    | `geometry_msgs/PoseStamped` | Nodo “missione” o RViz | Planner     |
| Traiettoria  | `/plan`         | `nav_msgs/Path`             | Planner                | (debug)     |
| Comandi      | `/cmd_vel`      | `geometry_msgs/Twist`       | Planner                | Robot       |
