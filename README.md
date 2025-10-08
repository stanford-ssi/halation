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

### Development
- You can directly just edit the code if you have ROS2 already setup on your system, but most likely you won't be.
- If you don't have the ROS2 setup, then you won't have autocomplete/typedefs/important things that you probably want to develop with.
- Instead, we've set up two ways you can access the code.
  - (untested) There is a devcontainers.json, which should allow you to directly develop within the container using an VSCode extension 'Dev Containers' (somehow doesn't work with Hinson's machine)
  - I've additionally set up ssh, so that if devcontainers doesn't work for you, you can use 'Remote-SSH' (tested on Cursor) with `ssh root@localhost -p 2267`. The password is set in the Dockerfile as `thispasswordissecure`, although since this is just local ssh, you can do whatever you want.
- Either of these methods should get you the proper type definitions. If not, make sure the interpreter is set to `/usr/bin/python3`; if it still doesn't work, ping Hinson ;)