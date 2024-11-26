# Robot Servoing

This package provides launch files to launch MoveIt servo for the UR5 robot. In addition a keyboard_input node is provided to test servoing (either in simulation or with real hardware) without a joy stick of other input providing delta twist commands.

## Example usage

### UR5 with fake hardware and keyboard input

Start UR robot with fake hardware:

`ros2 launch robot_servoing ur_servo.launch.py use_fake_hardware:=true`

In another terminal window switch controllers from joint_trajectory controller to forward_position_controller:

`ros2 control set_controller_state forward_position_controller active` 

Start servo server:

`ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}`

Start keyboard input node:

`ros2 run robot_servoing servo_keyboard_input`

Now you can control robot joints and end effector position using the keyboard (see node's output for specific keys)