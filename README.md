# My UR MoveIt
Custom UR Description packages for MoveIt ROS 2.

## Packages Requirement
You should have these package, or source them before running.
- `ur_description`
- `ur_client_library`
- `robotiq_hande_description`
- (optional, for running zed) `zed_wrapper` 

Clone this repo into your_workspace/src, then build

## Run with URSim
#### Use Launch
```   
ros2 launch my_robot_cell_control ursim_bringup.launch.py start_delay:={start_delay} rviz_config:={rviz_config}
```
`{rviz_config}` : File name of Rviz config to launch. The confign must be in my_robot_cell_moveit_config/config. Default is `moveit.rviz`. Main config is `main.rviz`.

`{start_delay}` : The number of seconds to delay launching control node after starting the UR Sim. Default is 12 seconds. Add more if your computer is slow.

``` 
# Example, run main:
ros2 launch my_robot_cell_control ursim_bringup.launch.py start_delay:=12 rviz_config:=main.rviz
```
#### Or Run manually
1. Start UR Sim Docker
```   
ros2 run ur_client_library start_ursim.sh -m ur3e -i 192.168.56.101
```
2. Start Driver
```
ros2 launch my_robot_cell_control start_robot.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=false
```
3. Add URCap in URSim and press start

4. Launch Move group
```
ros2 launch my_robot_cell_moveit_config move_group.launch.py ur_type:=ur3e
```
5. Launch MoveIt
```
ros2 launch my_robot_cell_moveit_config moveit_rviz.launch.py ur_type:=ur3e launch_rviz:=true
```

## Run with Real UR

#### Use Launch (start and connect LAN first.)
```   
ros2 launch my_robot_cell_control realur3e_bringup.launch.py
```

`{rviz_config}` : File name of Rviz config to launch. The confign must be in my_robot_cell_moveit_config/config. Default is `moveit.rviz`. Main config is `main.rviz`.

``` 
# Example, run main:
ros2 launch my_robot_cell_control realur3e_bringup.launch.py rviz_config:=main.rviz
```
#### Or Run manually

1. Start real UR3e

2. Start Driver
```
ros2 launch my_robot_cell_control start_robot.launch.py ur_type:=ur3e robot_ip:=10.10.0.61 launch_rviz:=false
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

