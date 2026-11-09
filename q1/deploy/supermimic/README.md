# Rust SuperMimic 部署

当前部署已更新：Q1本地持续推理与Web服务、Mini PC Docker转发入口，真实任务操作流程已通过。[当前控制台与操作说明](../../docs/2026-11-07-q1-onboard-console.md)。以下包含此前架构／评估阶段记录。


本次真实环境是 Mini PC `192.168.54.49` → Wi-Fi → Q1 `192.168.54.41`。结果见 [真实测试记录](../../docs/2026-11-07-supermimic-deployment-results.md)。当前仅验证到单肘接管／保护及真实影子推理，全身运动未验收。

## 已运行的服务

Mini PC：Compose项目`q1-supermimic`，镜像`embodied-q1:supermimic`，控制台端口8768。Q1：独立目录下的`q1-rt`和`q1-guard-net`；最终HAL输出在本体，Mini PC容器退出不会让本体无限续期旧策略。

本次启动参数保存在本项目的`private/mini.env`，独立操作口令在`private/operator-key`；`private/network-key`仅用于机器间鉴权。不要把这两个文件放进镜像或浏览器页面。

```bash
cd /home/kyokagong/code/embodied_edge
Q1_SETTINGS_FILE="$PWD/q1/deploy/supermimic/private/mini.env" q1/deploy/supermimic/run.sh ps
Q1_SETTINGS_FILE="$PWD/q1/deploy/supermimic/private/mini.env" q1/deploy/supermimic/run.sh logs --tail=30
```

`up -d`重建Mini PC容器不会清除本体FAULT。`down`只关闭本项目的Mini PC容器，不代表关节卸力。禁止通过重启自动续走／续舞。

浏览器手柄需要HTTPS或操作电脑的localhost。通过已配置的Mini PC SSH连接建立`-L 8768:127.0.0.1:8768`转发，再访问`http://localhost:8768`。页面只接受物理标准映射手柄，先全部释放、回中并校准；未满足硬件／运动准入时启用按钮保持不可用。

## 构建

原生完整镜像可使用`rust/Dockerfile`，构建参数`Q1_FEATURES=ros2,supermimic`。本机已有Humble开发容器`q1-sm-dev`，项目`q1/`挂载到`/opt/q1`，可复用已经验证的构建环境：

```bash
docker exec q1-sm-dev bash -lc 'source /opt/ros/humble/setup.bash; source /opt/q1_ros/install/setup.bash; cd /opt/q1/rust; cargo build --release --locked --features ros2,supermimic'
docker exec q1-sm-dev bash -lc 'source /opt/ros/humble/setup.bash; source /opt/q1_ros/install/setup.bash; /opt/q1/scripts/package-supermimic-image.sh'
docker build -f q1/deploy/supermimic/Dockerfile.runtime -t embodied-q1:supermimic q1/artifacts/supermimic/mini-runtime
```

运行镜像只从校验后的运行目录构建，不包含模型、训练环境、私钥或交叉sysroot。模型通过只读卷装载。Docker默认只输出帮助，不启动Mock或自动接管。

`run.sh`优先使用系统Compose V2；本机缺少该组件，本次将Ubuntu仓库包解压到`private/tools`后使用，不修改系统安装。

## 本体 ARM64

`cross/rootfs`来自真实Q1的Humble和系统动态库，`cross/aimdk_msgs`由本项目SDK源交叉构建。r2r构建辅助补丁只区分宿主代码生成与目标链接前缀；见`vendor/r2r_common_cross/CROSS_PATCH.json`。

```bash
docker exec q1-sm-dev bash -lc '/opt/q1/scripts/build-guardian-arm64.sh'
python3 q1/scripts/validate-guardian-arm64.py --package q1/artifacts/guardian/arm64 --sysroot q1/cross/rootfs --report q1/rust/reports/guardian-arm64-validation.json
```

上传入口`q1/scripts/deploy_guardian.py`再次核对SHA256与ELF，部署到新的版本目录。配置与网络密钥分开发送；两端配置文件必须逐字节一致。`readonly`不创建HAL发布者；`air`还需要固定的小范围单左肘配置，只有显式ARM且目标通过校验后才输出。不要向正在运行的目录覆盖二进制或切换配置。

当前本体故障已锁存。现场支撑、现有输出、HAL唯一写入者和恢复路径没有重新确认前，不重启保护器来绕过故障。厂商原生watchdog、全身故障增益及IMU安装变换依然未知，不能复用单肘测试去确认全身模型配置。

## Q1 本体只读 ONNX 测量

用户已允许评估推理下沉。`q1-model-check`不创建ROS节点、控制IPC或HAL发布者；只读取固定导出包，核对68组golden并测量50 Hz周期内的推理与调度。ORT ARM64库固定1.23.2，来源和wheel SHA256保存在生成包的`notices/ort-provenance.json`。

```bash
docker exec q1-sm-dev bash -lc '/opt/q1/scripts/build-model-check-arm64.sh'
```

只有在本地依赖与包校验完成后才使用`deploy_model_check.py`。它在本体CPU4/5、nice10下分别测2线程与1线程，不接管关节。该工具使用导出golden观测做数值／性能测量，没有仿真本体，也不将结果算作真实闭环动作验收。
