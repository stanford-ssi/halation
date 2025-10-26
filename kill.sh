pkill -f "ros2|rclpy|rclcpp|rosbridge|fastapi_proxy_node|motor_interface|motor_control|rosapi_node|launch|ros2|dds|gazebo|ignition"
ros2 daemon stop
ros2 daemon start
