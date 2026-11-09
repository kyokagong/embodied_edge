# Q1 Rust 手柄控制服务

Rust 服务通过 r2r 接入现有 Humble/FastDDS 和固定版 `aimdk_msgs`。操作电脑继续用现有网页采集手柄，Rust 在运行主机处理输入、控制状态机和高层速度输出。默认启动为 Mock。

本轮构建面向当前 x86_64 系统，`rust-toolchain.toml` 固定 Rust 1.90.0。首版包含校准、RB 锁定使能、站立、行走、横移、整机转向、软件停止和断联处理。Y/Pico 上肢功能会明确返回未启用，完整 Pico 功能仍由 Python 入口提供。

## 构建与启动

在 `q1` 目录执行 `./scripts/build-rust.sh`，构建镜像 `embodied-q1:rust`。构建阶段固定 ROS 基础镜像摘要、Rust 工具链，并通过 `Cargo.lock` 锁定 Cargo 依赖。构建包含完整厂商消息包，编译时需要网络下载系统包及 Cargo 依赖。构建步骤只生成程序和接口产物。

依赖诊断可以在断网容器运行。

```bash
docker run --rm --network none embodied-q1:rust --diagnose
```

查看 Mock 网页可使用宿主网络，服务默认监听本机回环地址。

```bash
docker run --rm --network host embodied-q1:rust --backend mock --port 8767
```

浏览器打开 `http://127.0.0.1:8767/`。远程运行时，沿用 SSH 本地端口转发。校准前释放按键、摇杆回中，Back 校准，Start 或独立长按 RB 一秒使能，X 进入行走。默认 RB 锁定后松开仍可控制，摇杆回中归零，再独立长按 RB 解除使能，B 软件停止并清除校准。

原生 Cargo 构建需要先加载 Humble 和完整消息包。

```bash
source /opt/ros/humble/setup.bash
source /opt/q1_ros/install/setup.bash
cd rust
cargo build --release --locked --features ros2
```

纯 Mock/控制逻辑测试可通过 `cargo test --locked` 运行。开启 `ros2` 特性后需要本机 ROS 2 及消息类型支持，使用 `cargo test --locked --features ros2`。

## 发布包

在有编译产物的环境，从 `q1` 目录执行以下命令。

```bash
./scripts/package-rust.sh \
  rust/target/release/q1-edge \
  /opt/q1_ros/install/aimdk_msgs
```

输出位于 `rust/dist/`，包含可解包目录及 `.tar.gz`。包内 `run` 根据解包目录加载私有消息库，网页已嵌入二进制。使用 `./run --diagnose` 检查环境，使用 `./run --backend mock` 启动 Mock。`./run` 会加载系统 `/opt/ros/humble/setup.bash`，也可用 `Q1_ROS_SETUP` 指定系统 ROS 入口。

运行主机需要匹配的系统/glibc 和 ROS 2 Humble，包含 `rmw_fastrtps_cpp`、`geometry_msgs`、`sensor_msgs`、`nav_msgs`。包内 `manifest.json` 记录本次构建机架构、工具链、厂商 SDK 提交、Python 基线哈希及系统 ROS 包版本。`SHA256SUMS` 覆盖包内文件，解包后执行 `sha256sum -c SHA256SUMS` 校验。

## 验收

阶段划分、测试证据和剩余实板项目记录在 `../docs/rust-migration.md`。Python/Rust 控制对照由 `../scripts/check_rust_parity.py` 驱动，两者独立重放相同的带时序输入。真实 DDS 测试由 `../scripts/check_rust_dds.py` 驱动独立假 MC，须在 `--network none`、域 117、`ROS_LOCALHOST_ONLY=1` 的容器运行。

真机后端入口仍需显式选择 `--backend ros2 --enable-hardware`。启动后等待有效动作与全部十关节反馈，输入校准和显式使能后才注册控制源。正式接入前应确认本轮验收报告与部署包哈希对应，关闭另一写入实例，并保留原厂遥控器恢复路径。

## 服务超时与恢复

服务请求到期后，运动许可会撤销。r2r 会在内部保留尚无响应的请求，因此本服务会保留该请求直到迟到回包被清理，再允许同类查询；迟到回包不能授权运动。若服务永久无响应，该查询会保持占位，需要排查原厂服务后显式重启 Rust 进程。注册或删除源的结果无法确认时，会保持故障和当前注册标识，阻止重复注册。恢复运动仍需重新校准和显式使能。
