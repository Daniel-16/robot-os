# Sonar data
rostopic echo /sonar_range

# Button press events
rostopic echo /button_event

# Turn the LED on and off
rostopic pub /set_led_state std_msgs/Bool "data: true"
rostopic pub /set_led_state std_msgs/Bool "data: false"

# Make the robot move forward (linear.x)
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# Make the robot turn in place (angular.z)
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"

# Stop the robot
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"