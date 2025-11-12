# Halation

The Stanford Antarctic Rover project.

### Dependencies:
- Orbstack/Docker

### Setup
1. `docker-compose up -d`
2. `docker-compose exec halation bash`
3. `colcon build --symlink-install` ALWAYS RUN FROM THE ROOT (not src!)!
4. `source install/setup.bash`     
5. On subsequent starts, run 1-2 (no need to build or source install)

To rebuild the docker image, run `docker-compose up --build -d`

### Development
- You can directly just edit the code if you have ROS2 already setup on your system, but most likely you won't be.
- If you don't have the ROS2 setup, then you won't have autocomplete/typedefs/important things that you probably want to develop with.
- Instead, we've set up two ways you can access the code.
  - (untested) There is a devcontainers.json, which should allow you to directly develop within the container using an VSCode extension 'Dev Containers' (somehow doesn't work with Hinson's machine)
  - I've additionally set up ssh, so that if devcontainers doesn't work for you, you can use 'Remote-SSH' (tested on Cursor) with `ssh root@localhost -p 2267`. The password is set in the Dockerfile as `thispasswordissecure`, although since this is just local ssh, you can do whatever you want.
- Either of these methods should get you the proper type definitions. If not, make sure the interpreter is set to `/usr/bin/python3`; if it still doesn't work, ping Hinson ;)

### Execution
- `ros2 launch rover_bringup rover_system.launch.py`

### SSHing into the system
- Install `cloudflared` (`brew install cloudflared` for Macs, follow [https://developers.cloudflare.com/cloudflare-one/networks/connectors/cloudflare-tunnel/downloads/](.deb package) + `dpkg -i ...deb` for Linux) 
- Follow `ssh-keygen` instructions to generate ssh key (give to Hinson)
- Paste this in your `~/.ssh/config`
  ```
  Host jetson.ssirovers.org
      ProxyCommand cloudflared access ssh --hostname %h
      User jetson
  ```
- Run `ssh jetson.ssirovers.org`

### LIDAR / Object Avoidance / Rerouting Execution
- ROS package for object avoidance/rerouting using LIDAR is contained within src/avoidance_rerouting
- Launch using: ros2 run avoidance_rerouting mock_lidar_publisher (this starts a node that sends mock LIDAR signals to /scan)
- To visualize LIDAR outputs, run: ros2 launch foxglove_bridge foxglove_bridge_launch.xml, then go to https://app.foxglove.dev/ssi-halation/dashboard (ask Hiroki to be invited to dashboard) and open up ws://localhost:8765 on Foxglove. 

### Dependencies
- We use `uv` to manage the python dependencies, but NOT for project running. That is, `uv add` and `./uvsync.sh`.
- Disregard the `.venv/`, ros itself has it's own types and deps we need.

### TODO:
- Figure out proper configuration for environment variables
- How to do shutdowns


### On old Ubuntu versions/Jetson
`docker compose up -d`
`docker compose exec halation bash`

