# My UR MoveIt
Custom UR Description packages for MoveIt ROS 2.

## Prerequisite
You should have these package, or source them before running.
- `ur_description`
- `ur_client_library`
- `robotiq_hande_description`
- (optional, for running zed) `zed_wrapper` 

Clone this repo into your_workspace/src, then build

## Run with URSim
1. Start UR Sim Docker
```   
ros2 run ur_client_library start_ursim.sh -m ur3e -i 192.168.56.101
```
2. Start Driver
```
ros2 launch my_robot_cell_control start_robot.launch.py ur_type:=ur3e robot_ip:=192.168.56.101
```
3. Add URCap in URSim and press start

4. Launch Move group
```
ros2 launch my_robot_cell_moveit_config move_group.launch.py ur_type:=ur3e launch_rviz:=true
```
5. Launch MoveIt
```
ros2 launch my_robot_cell_moveit_config moveit_rviz.launch.py ur_type:=ur3e launch_rviz:=true
```

## Run with Real UR
1. Start real UR3e

2. Start Driver
```
ros2 launch my_robot_cell_control start_robot.launch.py ur_type:=ur3e robot_ip:=10.10.0.61
```
3. Add URCap and press start

4. Launch Move group
```
ros2 launch my_robot_cell_moveit_config move_group.launch.py ur_type:=ur3e
```
5. Launch MoveIt
```
ros2 launch my_robot_cell_moveit_config moveit_rviz.launch.py ur_type:=ur3e launch_rviz:=true
```

## Run with ZED
`zed_camera_link` is already attached in the urdf. To run ZED, run:
```
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i publish_tf:=false
```
By having `publish_tf:=false`, the base link of zed will be `zed_camera_link` making it able to mount with UR3e.

