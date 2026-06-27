# Sample Service Clients

These examples target ROS 2 Lyrical Luth and call the Hiroz-backed DSLR capture service:

```text
/pgwaam/id2_rpi4/Canon_EOS_6D/capture
```

Build the local ROS interface package and client image:

```bash
cd sample_service_clients
docker compose build ros2-cli
```

The Dockerfile builds and sources the interface with:

```bash
source /opt/ros/lyrical/setup.bash
colcon build --packages-select pgwaam_msgs
source /ros2_ws/install/setup.bash
```

Run the Python `rclpy` client:

```bash
docker compose run --rm python-client
```

Run the plain ROS 2 CLI client:

```bash
docker compose run --rm ros2-cli
```

Both clients default to:

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/172.31.1.252:7447"]'
```

The capture service request is intentionally empty. Set camera enum indices on the running Hiroz node with ROS parameters such as `shutterspeed`, `iso`, `aperture`, and `imageformat`.
