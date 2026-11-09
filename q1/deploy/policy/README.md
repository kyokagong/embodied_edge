# Q1 原生策略部署

新版活动入口为 `q1-policy`，使用 BlinkMimic 原生双输入，配合 `q1-rt --policy-v2`。原 `q1-stand` 和对应脚本保留作历史版本工具，不用于本次模型部署。

2026-11-09 已在原 `/home/run/q1-supermimic` 目录完成替换部署。当前入口 <http://192.168.54.41:8768>，活动服务为 `q1-policy` 和 `q1-rt`，模型为EMA48000。当前保持 `RELEASED / DAMPING`、22轴Kp为0，没有执行站立或舞蹈。实际部署与观察记录见 [本次部署记录](../../docs/2026-11-09-blinkmimic-deployment.md)。

当前实测吊装姿态尚不满足站立参考接入：左肘约0.67rad、参考1.40rad，最大差约0.73rad超过0.5rad限制。服务部署通过不等于站立动作已具备可执行条件。趴姿起身（official_getUp）公共入口仍暂不开放。

## 当前部署包

- 运行包：`q1/artifacts/policy/arm64`，含 `q1-policy`、`q1-rt`、ARM64运行库和现有硬件配置。
- 模型包：`q1/artifacts/policy/models/817ad430b13e4a3b97082b18b882e765d92cc6aa049dc2b64bee007adaa7dd40`。
- 模型：EMA48000，`model/policy_native.onnx`，SHA256 `65fc94df9b972239d6bd46c2d147cf88174783b9de6e522e90f097407709f8f6`。
- 硬件配置 SHA256：`e33d406028847c3006779a26daf07c01ed70d3c70f957665526b116c07084057`。不改 ROS 话题、HAL槽位、厂商方向/零偏、现有PD/力矩/速度参数。
- 原生输入为 `obs_history[1,10,78]` 和 `goal[1,594]`，输出为 `actions[1,22]`。归一化和参考前馈在模型内部。

后续更新的单位是模型包：权重、契约、参考、独立golden输出和摘要。工具自动验证、提取和版本化，用户不再修改Rust或重新编译ROS。同协议、同执行配置的模型可独立更新；改变观测、动作或执行协议需明确适配，不能只按维度放行。

## 开发机复现

在项目根目录执行：

```bash
python3 q1/scripts/prepare_policy_model.py \
  --source /home/kyokagong/models/blinkmimic/q1_blinkmimic_48000_20261109

docker exec q1-sm-dev bash -lc '/opt/q1/scripts/build-policy-arm64.sh'
```

运行包的 `SHA256SUMS` 覆盖二进制、配置、依赖与工具；`notices/arm64-dependency-closure.json` 记录63个ARM64依赖对象。模型独立校验清单保留原始完整模型包清单供追溯；不上传训练checkpoint、旧接口模型或CSV。

主机离线Rust预检（实际执行ONNX，没有ROS或控制IPC）：

```bash
docker exec q1-sm-dev bash -lc '
  cd /opt/q1/rust
  ORT_DYLIB_PATH=/opt/ort/libonnxruntime.so.1.23.2 target/debug/q1-policy \
    --check-only \
    --model-dir /opt/q1/artifacts/policy/models/817ad430b13e4a3b97082b18b882e765d92cc6aa049dc2b64bee007adaa7dd40 \
    --hardware-profile /opt/q1/config/supermimic-stand-trial.json
'
```

## Q1开机后的首次部署

Q1若刚开机、项目进程尚未运行，使用下面命令。部署工具先检查没有旧项目写入者、API监听者或事务锁；不会按历史PID猜测或终止未知进程。若已有旧控制器，应先通过其UI解除接管，再用下一节的热迁移分支。

```bash
python3 q1/scripts/deploy_policy.py \
  --runtime q1/artifacts/policy/arm64 \
  --model q1/artifacts/policy/models/817ad430b13e4a3b97082b18b882e765d92cc6aa049dc2b64bee007adaa7dd40 \
  --base /home/run/q1-supermimic --cold-start --sudo \
  --host 192.168.54.41 \
  --ssh-key /home/kyokagong/code/QY01BAL2610000105/id_ed25519_QY01BAL2610000105.lf \
  --report q1/rust/reports/policy-deployment.json
```

增加 `--dry-run` 可在开发机打印包版本和目标目录，不建立SSH连接。`--sudo`仅用于本任务部署事务，复用现有免交互sudo，以处理旧版root进程和既有FIFO20调度；不修改系统服务或厂商程序。

以下是历史旧版迁移示例；本次迁移已完成。后续模型更新使用 `policy-deployment.json`，不再使用旧 `real-stand-deployment.json`。已运行旧 `q1-stand` 时，将上述 `--cold-start` 替换为：

```text
--previous-deployment q1/rust/reports/real-stand-deployment.json
```

该分支逐项核对旧进程可执行文件摘要、命令行、启动时间及PID，确认失能/解除接管，再只替换本任务进程。启动使用RT控制主线程CPU7/FIFO20（DDS、传感器与日志线程保持原CPU集合）、策略CPU4/5、50Hz实际HAL发布，保留500Hz保护调度。

部署端再次核对真实驱动与MotorModule YAML摘要，并先运行完整模型预检。只有模型身份/协议正确、反馈与推理计数推进、HAL发布推进、没有故障或准入阻塞，才写入成功报告。启动后为零刚度低阻尼、模型任务关闭，不自动起身或播放。Q1上的推理时延、真实反馈和动作效果仍需在设备启动后核验。

## 后续仅更新模型

1. 训练/导出侧提供同协议新模型包及其独立golden输出和摘要。
2. 用 `prepare_policy_model.py --source 新包目录` 得到新模型版本目录。
3. 在UI解除接管后执行：

```bash
python3 q1/scripts/update_policy_model.py \
  --model 新模型版本目录 \
  --previous-deployment q1/rust/reports/policy-deployment.json \
  --sudo --host 192.168.54.41 \
  --ssh-key /home/kyokagong/code/QY01BAL2610000105/id_ed25519_QY01BAL2610000105.lf \
  --report q1/rust/reports/policy-deployment.json
```

模型存入不可变版本目录 `/home/run/q1-supermimic/models/<摘要>`。先完成新包/ONNX/golden/执行参数预检，再替换策略进程和活动模型链接；RT的PID、程序和配置保持一致。加载/健康检查失败时恢复旧模型及策略进程，保持失能；旧模型文件保留。失败报告也会写到指定路径，需检查其结果后再操作。

## 任务语义

- 站立分为接入和保持：用户明确的起始条件为**失能、双脚着地、吊杆支撑并防倒的蹲坐姿态**。站立接入使用 `floor_fold` 的 `return_squat_start` 到 `return_end`（本包22..28秒）区间，按实际22关节及重力方向选择匹配帧，然后用同一ONNX模型和50Hz参考时钟完成蹲坐到直立，包含原参考末端保持段。不是从参考第0帧坐下，也不使用趴姿 `official_getUp`。
- 匹配误差使用现有跟踪/倾角配置边界；匹配不到则拒绝接入并报告误差，不能假定任意吊装姿态均已覆盖。双脚接触和吊杆支撑是用户确认的现场条件，软件不伪造足底接触或根部高度。设备开启后需核对实际起始姿态与参考覆盖范围。
- 实际关节/IMU达到安静直立端点后，站立保持才转为command模式的零速度目标。重复站立请求不重置参考进度。未完成首次站立时不能由失能状态直接启动其他参考动作。
- 坐下 `floor_fold`、躺下 `official_lie`、舞蹈仍经同一策略链。趴姿 `official_getUp` 是不同且尚有问题的能力，本轮公共控制入口暂不开放。
- 首次只做有界实测姿态接管，没有旧版20秒home插值，也不叠加仿真延迟、随机复用或外部参考前馈。
- 坐下在参考保持段钳制当前帧及未来10帧，收到返回请求后播放返回段；躺下保持末端；本轮站立按钮不从趴姿恢复，趴姿恢复暂不开放。
- 正常切换保留历史和物理状态，用epoch隔离旧推理；参考yaw在动作开始时一次对齐，观测和RT使用相同当前参考旋转。
- 上一动作来自RT成功HAL发布所选的clipped raw，保留复用语义，不由最终限幅位置反算。
- 页面显示站立接入帧和姿态匹配误差；没有根高度/足底接触测量时不报告其已验证。站立参考结束但实际未达安静直立端点，会进入保护保持，不能仅凭进度切到站立命令。
- “取消动作”按参考返回；“停止并保护保持”“解除接管”分别保留原独立RT语义。

输入/推理/参考/发布记录只写项目私有飞行日志，不提供张量下载API。页面显示实际加载模型摘要和资格记录。模型包和本地运行包位于忽略目录，不做git提交。

部署工具支持 `--base` 指定原项目位置。后续更新默认沿用部署报告的base，不会切换到另一目录。整机重启且本项目进程未启动时可使用上述cold-start命令；已有服务时必须依据当前部署报告替换。
