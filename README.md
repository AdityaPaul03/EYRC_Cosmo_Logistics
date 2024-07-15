# cosmo_logistic_bot
Cosmo Logistic Bot for E-yantra Robotics Challenge 2023-24 Team ID: CL#2736

## Task 1B

### Launch gazebo and description files
```
ros2 launch ur_description ur5_gazebo_launch.py
```

### Spawn Robot
```
ros2 launch ur5_moveit spawn_ur5_launch_moveit.launch.py
```

### Add Racks
```
ros2 run pymoveit2 ex_collision_object.py --ros-args -p action:="add" -p filepath:="~/colcon_ws/src/pymoveit2/examples/assets/rack1.stl" -p position:="[0.59, 0.055, -0.5]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]"
```
```
ros2 run pymoveit2 ex_collision_object.py --ros-args -p action:="add" -p filepath:="~/colcon_ws/src/pymoveit2/examples/assets/rack2.stl" -p position:="[0.29, -0.645, -0.5]" -p quat_xyzw:="[0.0, 0.0, 0.7071, 0.7071]"
```
```
ros2 run pymoveit2 ex_collision_object.py --ros-args -p action:="add" -p filepath:="~/colcon_ws/src/pymoveit2/examples/assets/rack3.stl" -p position:="[0.29, 0.755, -0.5]" -p quat_xyzw:="[0.0, 0.0, 0.7071, 0.7071]"
```

### Run Task 1B executor
Add task1b.py to "pymoveit2/examples" and rename the file as "ex_pose_goal.py" and then run:
```
ros2 run pymoveit2 ex_pose_goal.py
```
