# Q1 Edge：手柄控制与 Pico4Ultra 遥操

首版保留启元 Q1 原厂 MC 负责平衡、站立和行走，Pico 控制双臂与腰部。提供本地网页控制台、无设备模拟输入、真实 ROS2 SDK 适配和 Pico 采集适配。默认不会连接机器人，启动后也不会自动使能。

开发日期：2026-10-30。当前没有本体、手柄和头显在场；自动测试、浏览器模拟和真实 ROS2 DDS 回环的结论见 [verification.md](docs/verification.md)，**不能视为实机运动验收**。没有执行 git 提交。

## 现在先打开界面

```bash
cd /data/kk/embodied_edge/q1
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

## 宿主机原生连接本体和 Pico

真机交互采用 Ubuntu22.04 + ROS2 Humble + 系统 Python3.10，直接运行在控制电脑上，不使用Docker。Q1 Edge、官方SDK和Pico PC Service都在宿主机运行。先读 [真机联调手册](docs/hardware-bringup.md)，按“只读检查 → 原厂平衡站立 → 小幅行走 → Pico 数据 → 上肢遥操”顺序验证。

首次安装：先按ROS官方说明配置Humble软件源，在未激活Conda的终端执行。以下命令用于实际控制电脑，不要求在训练服务器安装ROS。

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-ros-base ros-humble-rosidl-default-generators \
  ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-nav-msgs \
  python3-colcon-common-extensions python3-venv python3-numpy

cd /data/kk/embodied_edge/q1
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
cd /data/kk/embodied_edge/q1
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

在已配置的原生虚拟环境执行：

```bash
.venv-ros/bin/python -m unittest discover -s tests -v
node tests/check_ui.cjs
```

以下Docker命令仅保留为历史离线DDS回环自检，假MC节点不能连接本体网络；它不是本体部署或启动方式。真机流程不需要构建或启动镜像。

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
