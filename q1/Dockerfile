FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config \
    ca-certificates curl less vim-tiny procps iproute2 iputils-ping \
    python3-dev python3-pip python3-venv python3-colcon-common-extensions \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
    ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-nav-msgs \
    && rm -rf /var/lib/apt/lists/*

# Keep the compiled SDK outside /opt/q1 so the development bind mount preserves it.
COPY vendor/primebot_sdk/aimdk_msgs /opt/q1_ros/src/aimdk_msgs
WORKDIR /opt/q1_ros
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select aimdk_msgs --cmake-args \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3

COPY requirements.txt requirements-pico.txt /opt/q1/
ARG WITH_PICO=0
ARG PIP_INDEX_URL=https://pypi.org/simple
RUN python3 -m pip install --no-cache-dir -r /opt/q1/requirements.txt && \
    if [ "$WITH_PICO" = "1" ]; then \
        python3 -m pip install --no-cache-dir -r /opt/q1/requirements-pico.txt; \
    fi
RUN printf '\nsource /opt/ros/humble/setup.bash\nsource /opt/q1_ros/install/setup.bash\n' >> /root/.bashrc

COPY . /opt/q1
WORKDIR /opt/q1
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENTRYPOINT ["/bin/bash", "/opt/q1/scripts/entrypoint.sh"]
CMD ["--backend", "mock", "--host", "127.0.0.1", "--port", "8766"]
