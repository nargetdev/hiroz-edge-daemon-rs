ARG ROS_IMAGE=ros:lyrical-ros-base
FROM ${ROS_IMAGE}

SHELL ["/bin/bash", "-lc"]

ENV ROS_DISTRO=lyrical
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/172.31.1.252:7447"]'

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-lyrical-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY pgwaam_msgs src/pgwaam_msgs
COPY python_client/capture_client.py /usr/local/bin/pgwaam_capture_client.py
COPY ros2_cli/call_capture.sh /usr/local/bin/pgwaam_capture_cli.sh
COPY entrypoint.sh /entrypoint.sh

RUN chmod +x /usr/local/bin/pgwaam_capture_client.py /usr/local/bin/pgwaam_capture_cli.sh /entrypoint.sh \
    && source /opt/ros/lyrical/setup.bash \
    && colcon build --packages-select pgwaam_msgs

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
