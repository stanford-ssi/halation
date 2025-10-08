# Halation

The Stanford Antarctic Rover project.

### Dependencies:
- Orbstack/Docker

### Setup
1. `docker-compose up -d`
2. `docker-compose exec halation bash`
3. `colcon build --symlink-install`
4. `source install/setup.bash`     
5. On subsequent starts, run 1-2 (no need to build or source install)