FROM ros:humble-ros-base-jammy@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855
SHELL ["/bin/bash", "-c"]
COPY vendor/primebot_sdk/aimdk_msgs /opt/q1_ros/src/aimdk_msgs
RUN source /opt/ros/humble/setup.bash && \
    cd /opt/q1_ros && colcon build --packages-select aimdk_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    python3 -c 'import numpy'
ARG WITH_PICO=0
ARG PIP_INDEX_URL=https://pypi.org/simple
RUN if [ "$WITH_PICO" = 1 ]; then \
    apt-get update -o Dir::Etc::sourceparts=- && apt-get install -y --no-install-recommends python3-pip && \
    python3 -m pip install --no-cache-dir xrobotoolkit-sdk==1.1.0 && \
    rm -rf /var/lib/apt/lists/*; fi
COPY . /opt/q1
WORKDIR /opt/q1
ENV PYTHONDONTWRITEBYTECODE=1
ENTRYPOINT ["/bin/bash", "/opt/q1/scripts/entrypoint.sh"]
CMD ["--backend", "mock", "--host", "127.0.0.1", "--port", "8766"]
