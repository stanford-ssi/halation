# Local ROS 2 bridge for Unity

This starts ROS 2 Humble, rosbridge, and rosapi for the Unity simulation.
Run the commands below from the Halation repository root with Docker running.

## Start

```sh
docker compose -f docker-compose.sim.yaml up -d --build --wait
```

Connect a Unity WebSocket client to `ws://127.0.0.1:9095`.
The existing web controller also needs to use this local endpoint to send commands to the simulation.
Start the web controller with the local bridge configuration:

```sh
cd web
cp -n .env.example .env.local
npm install
npm run dev -- --hostname 127.0.0.1
```

If `.env.local` already exists, set `NEXT_PUBLIC_ROSBRIDGE_URL=ws://127.0.0.1:9095` and restart the web server.
Open `http://localhost:3000` and confirm the displayed bridge address is the local endpoint.
Without this environment variable, the web app uses the existing rover endpoint.

The Unity project is in [stanford-ssi/halation-unity-simulation](https://github.com/stanford-ssi/halation-unity-simulation).
Open its `Assets/Scenes/RussellGlacier.unity` scene in Unity 6000.6.4f1, press Play, and press Tab in the Game view to select ROS control.
Move the web joystick to drive; release it to brake.

## Commands

Subscribe to `/motor_vector` and `/motor_estop` with type `std_msgs/msg/String`.
The existing web controller wraps the movement values twice: parse the ROS message's `data` as JSON, then parse that object's `data` string to obtain `x` and `y`.
`x` is steering and `y` is forward/backward input, each between -1 and 1.
The stop topic carries the web controller's stop request.
The Unity receiver preserves these topic contracts.

The bridge does not generate driving commands itself.
Motor topics are registered when a publisher or subscriber connects.

## Inspect or stop

```sh
docker compose -f docker-compose.sim.yaml ps
docker compose -f docker-compose.sim.yaml logs --tail=50 rosbridge
docker compose -f docker-compose.sim.yaml exec rosbridge /ros_entrypoint.sh bash
docker compose -f docker-compose.sim.yaml down
```

The shell command opens a terminal with ROS 2 available, where `ros2 node list` and `ros2 topic list` can inspect the local graph.

## Runtime

The container starts the upstream `rosbridge_websocket_launch.xml` with port 9095 and rosapi for topic discovery.
Docker exposes the WebSocket port only on the host's loopback address.
Native ROS discovery stays inside the container through `ROS_LOCALHOST_ONLY=1` and uses simulation domain 68.
The physical rover's existing Compose configuration uses domain 67 and remains separate.
This service does not launch the physical motor or sensor drivers.
Docker restarts the service when its engine starts unless the service was explicitly stopped.
