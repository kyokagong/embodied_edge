# Q1 Rust 迁移记录

范围为当前 x86_64 系统、r2r、物理手柄高层速度控制。操作电脑采集手柄，Rust 服务处理 HTTP、状态机和 MC 通信。Q1 实际只有一块承载大脑和 MC 的板，本轮按用户要求先完成当前系统功能与阶段四软件验收，目标板构建和实板负载验证留待部署阶段。

## 阶段与证据

| 阶段 | 交付内容 | 当前证据 |
|---|---|---|
| 0 | 固定 Python 源码/config 和厂商 SDK，重跑旧测试 | 基线 SHA256 见下表；隔离 Python 110 项测试与 Node 前端回归通过 |
| 1 | 当前架构构建环境、锁定 Cargo 依赖、可重定位发布包 | 当前系统 release、可重定位发布包和独立 Dockerfile 全流程构建通过，生成 amd64 运行镜像 |
| 2 | Rust SDK 服务、反馈和控制源仲裁 | 20 项 Rust 测试、ROS2 严格 Clippy 和两种关节反馈各 12 项隔离 DDS 场景通过 |
| 3 | 相同手柄轨迹对照、HTTP 及网页兼容 | 16 条轨迹、586 个事件逐帧一致；原有 Python 110 项和 Node 前端回归通过 |
| 4 | 故障注入、输出时序、干净启动、回退演练 | 发布版故障与时序报告、干净运行、同端口 Rust→Python→Rust 回退通过 |

构建入口为 `scripts/build-rust.sh`，默认输出当前宿主架构镜像 `embodied-q1:rust`；本轮实际 release 产物由独立开发容器生成。发布入口为 `scripts/package-rust.sh`，清单记录编译环境、厂商 SDK 和 Python 基线，包内运行入口使用相对安装位置加载消息库。完整镜像分为 builder 和 runtime 两阶段，runtime 使用 Humble 系统库及发布包。

完整构建已实际运行 `./scripts/build-rust.sh --build-arg RUSTUP_DIST_SERVER=https://rsproxy.cn --build-arg RUSTUP_UPDATE_ROOT=https://rsproxy.cn/rustup`，镜像 ID 为 `sha256:1dd55e4e3717e9723c23a03bcb189ee92c0a4e19e60aea2a6e0970ff75540a65`。锁定基础镜像已包含所需 ROS 开发和运行包，Dockerfile 检查包版本，仅从 Ubuntu 软件源安装缺少的编译工具。工具链与 Cargo 依赖由锁定版本重新安装及编译。

镜像内独立构建的二进制 SHA256 为 `68eb61d322a441121d633bf1704aff19fc8c3c5484e2eae8f81c893a095ac5fb`，与下述独立发布包分别记录。完整镜像已通过断网诊断、Mock HTTP 初始禁用/未校准/零速度、ROS2 无对端只读启动，以及两种模式 SIGTERM 退出码 0 检查。

从完整镜像提取的实际二进制也已在隔离 Humble 环境通过两种关节反馈各 12/12 HTTP＋DDS 场景，以及 16 条轨迹、586 个事件的 Python 行为对照。三份报告为 `rust/reports/image-dds-standard.json`、`image-dds-sdk.json` 和 `image-parity.json`，均记录镜像二进制的 SHA256。

2026-11-05 最终软件验收采用 Rust 1.90.0、ROS2 Humble、FastDDS，在当前 x86_64 主机串行运行。Rust 发布版 SHA256 为 `1130d2bb2a542a58505567df3158d5844b1701ce17d7a04adb1aee082e2d7001`，以下四份 Rust 报告均记录同一二进制哈希。测试容器均为 `--network none`、域 117、`ROS_LOCALHOST_ONLY=1`，假 MC 脚本还检查网络接口只有 lo。

| 验收 | 结果 | 正式报告（相对 q1） |
|---|---|---|
| Python/Rust 行为对照 | 16 条轨迹、586 个事件全部一致，包含接受/拒绝、模式、RB、速度、输入所有权、250/251 ms 边界 | `rust/reports/parity.json` |
| Rust 标准关节反馈 | 12/12，真实 HTTP 与 DDS | `rust/reports/rust-dds-standard.json` |
| Rust SDK 关节反馈 | 12/12，真实 HTTP 与 DDS | `rust/reports/rust-dds-sdk.json` |
| 同一套 Python 基准 | 两种关节反馈各 12/12 | `rust/reports/python-dds-standard.json`、`python-dds-sdk.json` |
| 跨运行时回退 | Rust 移动后归零释放，Python 同端口启动禁用、显式校准使能后移动并释放，再回 Rust 仍默认禁用 | `rust/reports/rollback.json` |
| 现有与新增回归 | Rust 20 项、Python 110 项、Node 界面回归和 JS 语法检查通过；ROS2 all-targets 严格 Clippy 通过 | 测试命令与复跑入口见 Rust README |

四份 DDS 报告使用同一验收脚本哈希 `dba90ab94569e0ce40e78a91c2611c1c5dfeb2ba1342d004ab1654849f628840`。12 项场景覆盖正常操控、缓存旧源名、控制权未确认、迟到确认、关节断流、关节不完整、输入断流、不完整 HTTP 请求、原厂抢占、服务挂起、短 RPC 截止时间和同运行时重启。每帧速度还检查序号与时间戳推进、pitch/level 为零，第一次 owner 确认前所有非零分量均不超过 0.2。

同一主机、同一测试脚本所得的测量如下。RSS 在正常控制场景结束时采样，CPU 为该场景内进程累计 CPU 时间。

| 实现与反馈格式 | 心跳样本数 | P99 / 最大间隔（ms） | 最后输入至收零（ms） | 慢 HTTP 至收零（ms） | RSS（KiB） | 场景 CPU（s） |
|---|---|---|---|---|---|---|
| Rust / standard | 74 | 21.13 / 21.13 | 262.66 | 260.84 | 23976 | 0.13 |
| Python / standard | 76 | 20.73 / 20.73 | 253.87 | 255.00 | 65088 | 0.60 |
| Rust / SDK | 75 | 21.15 / 21.15 | 261.50 | 256.27 | 24004 | 0.12 |
| Python / SDK | 77 | 21.27 / 21.27 | 254.23 | 254.85 | 65216 | 0.57 |

心跳采样窗口为约 1.5 秒，通过条件为至少 45 帧且最大间隔小于 150 ms；输入断流及慢 HTTP 的最后有效输入至对端收零须不超过 400 ms，首次非零仲裁至归零也须不超过 400 ms，其他故障注入至收零须不超过 500 ms。这些是隔离集成验收预算。逻辑输入超时保持 250 ms，实际对端收零约 262 ms 还包含控制循环、下一次发布和接收调度。RSS、CPU 和短窗口延迟仅描述本次测试机，实板与原厂大脑/MC 共载、无线耐久和物理制动表现留待实板验收。

复核中修复了两项实际缺陷。一次最终 SDK 格式运行在正常操控中报错，原因是控制线程先取时间、ROS 线程随后更新反馈，旧时间把新反馈判为负年龄。已先观察回归失败，再把 native 方法及服务调度的取时移入对应锁内；修复后两格式 12 项、回退和 586 事件均通过。另对 r2r 超时请求保留一个占位项，直到迟到响应清理完成，避免重复超时导致内部请求记录持续增长。150 ms RPC 截止时间、600 ms 回包延迟的验收中，首次迟到回包前 Rust 仅有一个查询，Python 使用其 pending 清理机制安全重试三次；双方均归零且不会自行复能。若旧请求永久无回包，该 Rust 查询持续占位，需显式重启服务恢复，重启后仍须重新校准与使能。

## 发布包实测（正式产物）

2026-11-05 在独立 `q1-rust-dev` 容器内完成 Rust 1.90.0 release 构建后打包，目录为 `rust/dist/q1-edge-5Mie8wm6`，压缩包为同名 `.tar.gz`。二进制 SHA256 为 `1130d2bb2a542a58505567df3158d5844b1701ce17d7a04adb1aee082e2d7001`，压缩包 SHA256 为 `b5d0276e5a61b738a6013bc7c805e92e816307b13a6e969d7a878285202237b0`。此产物包含停止/使能竞态修复，并在修复后重新执行以下干净运行与重定位检查。

使用全新 `ros:humble-ros-base-jammy` 容器，`--network none`、域 117、`ROS_LOCALHOST_ONLY=1`，仅将解包目录只读挂载至 `/opt/q1`，完成以下检查。

- 包内 `SHA256SUMS` 全部通过，`./run --diagnose` 返回 `ros2_compiled=true`、`read_only=true`、Humble 和正确 SDK 提交。
- Mock HTTP 端口 18966 成功就绪，状态为 idle、未使能、未校准、三轴命令全零。
- ROS2 HTTP 端口 18967 在没有假 MC 的情况下成功就绪，0.5 秒后仍为 idle、未使能、命令全零、反馈未连接、控制源未激活且未获取中。此检查只验证无对端启动；消息侧写入观测由隔离 DDS 测试覆盖。
- 两种后端收到 SIGTERM 后均在三秒内以退出码 0 结束。
- 同一包改挂载到 `/tmp/q1-relocated`，校验和和依赖诊断再次通过，证明运行入口支持改变解包位置。

干净运行容器没有挂载 Rust 源码、Cargo 缓存或开发消息工作区。基础镜像自带运行包版本为 `rmw_fastrtps_cpp` 6.2.10 与 `geometry_msgs`、`sensor_msgs`、`nav_msgs` 4.9.2。

## Python 基线

2026-11-05 记录当前工作目录中的文件内容，保留用户已有的未提交更改。厂商 SDK 固定提交 `7310314eee058323a2c15ab1f4f947df4112573e`。

| 文件（相对 q1） | SHA256 |
|---|---|
| `config/default.json` | `a7d3c846deb4c948d1333a6c9d12c636d2b3a8c60528f9ca06a2c567d431874e` |
| `q1_edge/core.py` | `48b31e894e186e91b8c00fa2103c57f336d47fd5a1c5e40a041c99bfd7f26d36` |
| `q1_edge/backends.py` | `bbe4f1af5d64c2e3612fc94dea5e9cd47749d126ca366466a1722e60e69d2992` |
| `q1_edge/config.py` | `f7021bad31e4b14a08dbb73532a2eb8b7e90c235b6063ce5e73d9e7b8fa96c87` |
| `q1_edge/server.py` | `62b99807a944b32bf92a3a2b06e7f2b54331cb915c6f817823852d9fe17dd05a` |
| `q1_edge/web/app.js` | `2030da5004ce16ff7e678b0312232efddda0072adfbc4d51baf4c4fdc21b1ff6` |
| `tests/test_core.py` | `ec50957a3a3c4d7a7f1c9feca8448c09717362ff1989bd8174c6516545033867` |
| `tests/test_backends.py` | `3edeccb9428f6d08f3f111b5d6540040941f80c80a4a80568d44ff8bb9b21114` |
| `scripts/check_ros2_loopback.py` | `3600e20b13377f2eac9409ee08abee1552095f7538c87a3a086df6f4d38f7a45` |

## 放行条件

1. 在干净 Humble 运行环境，发布包校验通过，动态库完整，Mock 和只读 ROS 启动成功。
2. 同一输入轨迹的请求接受/拒绝、校准、使能、模式、RB 锁定、速度和停止顺序在 Python/Rust 间一致。仅随机源名、墙钟、浮点尾差允许归一化。
3. 断网 DDS 对端验证实际收到的速度、第一次获取控制权的限幅/截止时间、停止后的零流和释放。重复序号和迟到 RPC 不能恢复运动。
4. 输入、反馈、服务挂起和原厂接管故障均有独立注入记录。输出周期报告 P99/最大间隔，归零报告最后有效输入到首次零速的耗时，明确测试机与负载。
5. 新旧服务的回退演练确认同一时刻只有一个写入实例，重启默认未使能。Pico 首版范围差异在界面和文档中明确。

## 后续实板验收

板上系统 ABI/消息版本匹配、原厂大脑与 MC 共载资源、真实无线延迟、原厂遥控器抢占、强杀进程后的固件源超时、受限实机运动仍须在部署阶段验证。本轮隔离测试的假 MC 输出记录用于软件验收。
