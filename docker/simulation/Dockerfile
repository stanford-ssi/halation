FROM ros:humble-ros-base-jammy

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-humble-rosbridge-server \
        ros-humble-rosapi \
    && rm -rf /var/lib/apt/lists/*

CMD ["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml", "port:=9095", "address:=0.0.0.0", "call_services_in_new_thread:=true", "default_call_service_timeout:=5.0", "send_action_goals_in_new_thread:=true"]
