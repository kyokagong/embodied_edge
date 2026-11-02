# Q1 Edge：手柄控制与 Pico4Ultra 遥操

首版保留启元 Q1 原厂 MC 负责平衡、站立和行走，Pico 控制双臂与腰部。提供本地网页控制台、无设备模拟输入、真实 ROS2 SDK 适配和 Pico 采集适配。默认不会连接机器人，启动后也不会自动使能。

开发日期：2026-10-30。当前没有本体、手柄和头显在场；自动测试、浏览器模拟和真实 ROS2 DDS 回环的结论见 [verification.md](docs/verification.md)，**不能视为实机运动验收**。没有执行 git 提交。

## 现在先打开界面

```bash
cd /home/kyoka/code/embodied_edge/q1
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python -m q1_edge
```

浏览器打开 <http://127.0.0.1:8766/>。运行在远程机器时，在自己的电脑执行：

```bash
ssh -N -L 8766:127.0.0.1:8766 用户名@运行机器IP
```

然后在自己电脑打开同一个 localhost 地址。手柄插在**打开浏览器的电脑**上，使用 Chromium/Chrome，按一下手柄按钮让 Gamepad API 发现设备。先选择“模拟输入”也能完整操作：回中 → 校准 → 使能 → 行走模式 → 前推摇杆并按住 RB 保持；松开保持键速度立即归零。Pico 模拟：先释放双手 Grip、回到参考位并校准，启用后切换遥操，双手 Grip 都按住才跟随。

默认 SDK HOME 的手臂已经接近伸展，模拟演示先将左手 **Y 上移 0.035m** 或 **Z 减少 0.010m**；直接向前 0.035m 会超出臂长并停止，需要清除故障、回中、重新校准。页面显示的是输入、关节与命令反馈，不是动力学仿真或相机画面。

## 手柄默认按键

采用浏览器 `standard` 映射，可在 JSON 配置中调整索引；未知品牌首次需在界面核对按键和方向。

|按键|功能|
|---|---|
|Back / Select（8）|摇杆中位和 Pico 参考坐标校准|
|Start（9）|显式请求使能|
|A（0）|原厂平衡站立，零速度|
|X（2）|行走模式|
|Y（3）|Pico 上肢遥操模式|
|RB（5），保持按住|允许摇杆行走；松开归零|
|B（1）|软件停止；同帧停止优先|
|左摇杆|前后、横向移动|
|右摇杆水平轴|转向|

初始已按住的按钮不触发边沿操作。网页失焦/隐藏、手柄断开、输入超过250ms、机器人状态过期或控制源抢占都会停止并要求重新使能。Pico 必须同时握住两个 Grip（≥0.7），松开保持当前上肢目标，重新握住会以当前实测关节和 VR 位姿重新对齐。

速度默认上限：前后0.3m/s、横向0.2m/s、转向0.4rad/s；SDK 小非零命令存在0.2下限约束，死区外会达到该最小值，**不是从任意小速度连续起步**。初次实机在原厂要求的支撑保护与空旷平地下测试。

## Docker 开发与运行（推荐）

Linux 宿主机安装 Docker Engine 即可。镜像使用 Ubuntu22.04 / ROS2 Humble / Python3.10，自动匹配 ARM64 或 x86_64；宿主机无需安装 ROS 或 Compose。

```bash
cd /home/kyoka/code/embodied_edge/q1
sudo docker build --network host -t embodied-q1:humble .
# 先检查镜像内依赖；此命令不创建 ROS 节点。
sudo ./scripts/run-docker.sh --backend ros2 --diagnose
# 默认运行 Mock 控制台，浏览器访问 http://127.0.0.1:8766/
sudo ./scripts/run-docker.sh
```

镜像安装编译器、CMake、Git、colcon、Python开发工具、curl、ping、ip、ps、less和vi，并编译 `vendor/primebot_sdk/aimdk_msgs`。这是本项目 Python ROS2 后端使用的 SDK 消息/服务包。

启动脚本将当前 `q1` 目录读写挂载到 `/opt/q1`，使用 `--network host` 共享主机网卡、IP和UDP端口，无需 `-p` 映射。SDK编译产物位于独立的 `/opt/q1_ros/install`，不会被代码挂载覆盖；代码和配置修改直接可见，修改消息定义或镜像依赖后需重新构建镜像。FastDDS和SDK环境会自动加载。

需要开发Shell时，先退出当前Mock容器，再执行：

```bash
sudo --preserve-env=ROS_DOMAIN_ID ./scripts/run-docker.sh bash
# 以下在容器内执行：
echo "$ROS_DISTRO"                 # humble
ros2 pkg prefix aimdk_msgs
python3 -m q1_edge --backend ros2 --diagnose
```

容器名为 `embodied-q1`，同时只启动一个实例。另一个终端可用 `sudo docker exec -it embodied-q1 bash` 进入同一容器。退出启动进程会删除临时容器，镜像、宿主机代码和挂载目录中的日志保留。

连接本体前，将宿主机 `ROS_DOMAIN_ID` 设置为本体实际域号，再使用上面的 `--preserve-env=ROS_DOMAIN_ID` 命令启动；未设置时脚本使用ROS默认域0。脚本设置 `ROS_LOCALHOST_ONLY=0` 以允许跨主机DDS通信。本体网卡、防火墙、时钟和UDP缓冲区仍按[联调手册](docs/hardware-bringup.md)核对。

进入容器后，按手册完成只读检查；真实后端仍需显式授权、页面校准和使能：

```bash
# 容器内；保留已有本地配置。
test -e config/local.json || cp config/default.json config/local.json
mkdir -p logs
python3 -m q1_edge --backend ros2 --enable-hardware \
  --config config/local.json --trace logs/hardware.jsonl
```

Pico绑定为可选依赖，需要时重新构建：

```bash
sudo docker build --network host --build-arg WITH_PICO=1 -t embodied-q1:humble .
```

随后按手册先验证 `--backend mock --pico xrobotoolkit`，再进入真实遥操；绑定可导入不表示头显、PC Service及实机已经联调通过。

如果构建下载依赖需要当前终端代理，可传递Docker预定义代理参数；本机回环代理需要构建时的 `--network host`：

```bash
sudo --preserve-env=HTTP_PROXY,HTTPS_PROXY,http_proxy,https_proxy docker build \
  --network host --build-arg HTTP_PROXY --build-arg HTTPS_PROXY \
  --build-arg http_proxy --build-arg https_proxy -t embodied-q1:humble .
```

这些参数用于镜像构建，不修改Docker服务的全局代理。若Docker服务本身无法访问Docker Hub，需先将官方基础镜像导入本机镜像缓存。

## 原生部署（已有 Ubuntu22.04 / Humble 时可选）

已有 Ubuntu22.04 + ROS2 Humble + 系统 Python3.10 的控制电脑也可直接运行。以下为原生部署步骤；采用上面的Docker流程时可跳过这一节。先读 [真机联调手册](docs/hardware-bringup.md)，按“只读检查 → 原厂平衡站立 → 小幅行走 → Pico 数据 → 上肢遥操”顺序验证。

首次安装：先按ROS官方说明配置Humble软件源，在未激活Conda的终端执行。以下命令用于实际控制电脑，不要求在训练服务器安装ROS。

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-ros-base ros-humble-rosidl-default-generators \
  ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-nav-msgs \
  python3-colcon-common-extensions python3-venv python3-numpy

cd /home/kyoka/code/embodied_edge/q1
source /opt/ros/humble/setup.bash
mkdir -p .ros_ws
cd .ros_ws
/usr/bin/colcon build --base-paths ../vendor/primebot_sdk/aimdk_msgs \
  --packages-select aimdk_msgs --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
cd ..
source .ros_ws/install/setup.bash

# 继承apt安装的rclpy及ROS依赖，同时隔离本项目的Python依赖。
/usr/bin/python3 -m venv --system-site-packages .venv-ros
.venv-ros/bin/python -m pip install -r requirements.txt -r requirements-pico.txt
# 创建本地配置；如果已有文件，不覆盖现场标定值。
test -e config/local.json || cp config/default.json config/local.json
mkdir -p logs
```

`.venv`用于无设备界面，`.venv-ros`用于真实ROS环境；后者必须用系统Python3.10创建，不能从Conda Python创建。编译消息包也固定使用系统Python。首次安装完成后，每个新终端执行：

```bash
cd /home/kyoka/code/embodied_edge/q1
source /opt/ros/humble/setup.bash
source .ros_ws/install/setup.bash
.venv-ros/bin/python -m q1_edge --backend ros2 --diagnose
```

ROS_DOMAIN_ID使用本体实际配置，不要猜测域号。先启动宿主机上的官方Pico PC Service及头显端XRoboToolkit应用，再用Pico驱动mock检查数据：

```bash
.venv-ros/bin/python -m q1_edge --backend mock --pico xrobotoolkit
```

退出mock进程，确认网络、固件和配置后，再启动真实后端：

```bash
.venv-ros/bin/python -m q1_edge --backend ros2 --enable-hardware --pico xrobotoolkit \
  --config config/local.json --trace logs/pico-hardware.jsonl
```

只测试手柄时去掉`--pico xrobotoolkit`。浏览器打开 <http://127.0.0.1:8766/>，真实手柄插在浏览器所在电脑。启动真实后端仍需在界面校准、使能才会注册控制源；模拟输入被拒绝。保留原厂MC。

默认 `upper.hardware_limits_confirmed=false`，真实上肢模式会被阻止；需要确认本体关节限位、方向、零位和固件支持后修改本地配置。不要仅为了消除提示将其设为true。

## 软件自检

Python测试可在容器中执行；前端自检在安装Node.js的宿主机执行：

```bash
sudo ./scripts/run-docker.sh python3 -m unittest discover -s tests -v
node tests/check_ui.cjs
```

使用原生部署时，将第一条命令替换为 `.venv-ros/bin/python -m unittest discover -s tests -v`。

以下命令用于隔离的DDS回环自检。假MC节点不能连接本体网络，必须保留 `--network none` 和本地ROS域设置，不要通过共享主机网络的启动脚本运行此项测试。

```bash
docker run --rm --network none -e ROS_DOMAIN_ID=117 -e ROS_LOCALHOST_ONLY=1 \
  --entrypoint /bin/bash embodied-q1:humble -c \
  'source /opt/ros/humble/setup.bash && source /opt/q1_ros/install/setup.bash && python3 -m scripts.check_ros2_loopback'
```

## 文件与下一阶段

- `q1_edge/core.py`：校准、使能、按键、输入所有权、状态机和看门狗。
- `q1_edge/backends.py`：Mock 与原厂 MC 的异步 ROS2 通信。
- `q1_edge/pico.py` / `assets/q1_upper.xml`：XRoboToolkit 采集、官方上肢 FK、相对手位置 IK。
- `q1_edge/web/`：无需前端构建的操作台。
- `config/default.json`：按键、速度、时限、关节限位与控制源配置。
- [SDK 与 Pico 接线说明](docs/hardware-bringup.md)、[SuperMimic 后续接入边界](docs/supermimic-integration.md)、[接口约定](docs/interfaces.md)。

本版 Pico 是4自由度手臂的位置优先遥操，腰部小幅跟随头部 yaw/roll；没有完整手腕6D姿态跟踪、手指控制、视觉避障或全身 Pico 动捕。模型限位不能代替实机限位，IK 也不提供自碰撞保证。软件停止不等于物理急停；上肢远离 HOME 时停止会保持上肢而阻止自动切回行走，使用原厂遥控器恢复，详见手册。
