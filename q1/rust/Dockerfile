ARG ROS_IMAGE=ros:humble-ros-base-jammy@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855
FROM ${ROS_IMAGE} AS builder
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG RUST_TOOLCHAIN=1.90.0
ARG RUSTUP_DIST_SERVER=https://static.rust-lang.org
ARG RUSTUP_UPDATE_ROOT=https://static.rust-lang.org/rustup
ENV RUSTUP_DIST_SERVER=${RUSTUP_DIST_SERVER} RUSTUP_UPDATE_ROOT=${RUSTUP_UPDATE_ROOT}
RUN apt-get -o Dir::Etc::sourceparts=- update && apt-get install -y --no-install-recommends \
    build-essential cmake curl ca-certificates pkg-config libclang-dev python3-pip \
    && rm -rf /var/lib/apt/lists/*
RUN dpkg-query -W python3-colcon-common-extensions python3-dev \
    ros-humble-rmw-fastrtps-cpp ros-humble-rosidl-default-generators \
    ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-nav-msgs
ENV CARGO_HOME=/opt/cargo RUSTUP_HOME=/opt/rustup
ENV PATH=/opt/cargo/bin:${PATH}
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs -o /tmp/rustup.sh && \
    sh /tmp/rustup.sh -y --profile minimal --default-toolchain "$RUST_TOOLCHAIN" && \
    rm /tmp/rustup.sh
COPY vendor/primebot_sdk/aimdk_msgs /opt/q1_ros/src/aimdk_msgs
WORKDIR /opt/q1_ros
RUN source /opt/ros/humble/setup.bash && colcon build --packages-select aimdk_msgs \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
COPY rust/Cargo.toml rust/Cargo.lock rust/rust-toolchain.toml /opt/q1/rust/
COPY rust/src /opt/q1/rust/src
COPY rust/web /opt/q1/rust/web
COPY config /opt/q1/config
COPY q1_edge /opt/q1/q1_edge
COPY tests /opt/q1/tests
COPY scripts /opt/q1/scripts
COPY vendor/primebot_sdk /opt/q1/vendor/primebot_sdk
COPY vendor/r2r_common_cross /opt/q1/vendor/r2r_common_cross
COPY assets/LICENSE.Apache-2.0 /opt/q1/assets/LICENSE.Apache-2.0
WORKDIR /opt/q1/rust
ARG Q1_FEATURES=ros2
RUN python3 -m pip install --no-cache-dir --no-deps --target /opt/ort-py onnxruntime==1.23.2
RUN source /opt/ros/humble/setup.bash && source /opt/q1_ros/install/setup.bash && \
    cargo build --release --locked --features "$Q1_FEATURES" && \
    /opt/q1/scripts/package-rust.sh /opt/q1/rust/target/release/q1-edge /opt/q1_ros/install/aimdk_msgs /opt/q1-release && \
    mkdir /opt/q1-runtime && tar -xzf /opt/q1-release/*.tar.gz --strip-components=1 -C /opt/q1-runtime

FROM ${ROS_IMAGE} AS runtime
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-query -W ros-humble-rmw-fastrtps-cpp ros-humble-geometry-msgs \
    ros-humble-sensor-msgs ros-humble-nav-msgs
COPY --from=builder /opt/q1-runtime /opt/q1
COPY --from=builder /opt/ort-py/onnxruntime/capi/libonnxruntime.so.1.23.2 /opt/q1/ort/libonnxruntime.so.1.23.2
COPY --from=builder /opt/ort-py/onnxruntime/capi/libonnxruntime_providers_shared.so /opt/q1/ort/libonnxruntime_providers_shared.so
ENV ORT_DYLIB_PATH=/opt/q1/ort/libonnxruntime.so.1.23.2
WORKDIR /opt/q1
ENTRYPOINT ["/opt/q1/run"]
CMD ["--help"]
