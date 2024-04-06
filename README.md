# Install

```bash
git clone https://github.com/SwatiShirke/Forward_Kinematics.git
cd Forward_Kinematics
colcon build --symlink-install
```

# Nodes 

## Open Manipulator Controller

```bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```

## Forward Kinematics

Echo the current current pose calculated from the current joint values
```bash
ros2 topic echo /calculated_pose
```

## Building Interfaces

Get the Joint values for the given Pose
```bash
ros2 service call /give_inverse_kinematics interfaces/srv/GiveInverseKinematics "{pose: {position: {x: 131.0, y: 0.0, z: 271}, orientation: {w: 0.707, x: -0.707, y: 0.0, z: 0.0}}}"❯ r.r --ros-args -p cmd:='inv_kine' -p orientation:='[0.797,-0.49,-0.5,0.0]' -p position:='[159.0,157.3,313.0]'
```

## Utils 
```bash

# show the help message
ros2 run utils utils
ros2 run utils utils --ros-args -p cmd:='help'

# set the robot to home position
ros2 run utils utils --ros-args -p cmd:='home'

# Move to a given joint position
ros2 run utils utils --ros-args -p cmd:='move' -p joint_positions:='[0.0,-0.78,0.0,0.78]'

# Return Joint values for a given end-effector pose
ros2 run utils utils --ros-args -p cmd:='inv_kine' -p orientation:='[0.797,-0.49,-0.5,0.0]' -p position:='[159.0,157.3,313.0]'
```


