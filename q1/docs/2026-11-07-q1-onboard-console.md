# Q1 本体部署与控制台使用

日期：2026-11-07。本文记录先前的观察推理部署，已被真实本体站立服务替代。当前入口、实际关节控制和真机结果见 [本体站立链路](2026-11-07-q1-supported-stand.md)。下文保留作历史，不代表当前运行方式。

## 当前运行方式

- **Q1 本体**：Rust SuperMimic 服务、ONNX Runtime 1.23.2、参考任务和 Web/API；通过本地 Unix IPC 读取 Rust ROS2 保护器的真实22关节与IMU。
- **Mini PC Docker**：只转发控制台连接，不再运行模型推理或500 Hz网络策略发送。当前容器为`q1-supermimic-console-1`，镜像`embodied-q1:supermimic-onboard`。
- **运行模式**：真实观测推理。模型输出候选关节目标；模型执行器接管关闭。当前独立保护器仍处于此前的单左肘低增益FAULT保持，部署没有清除或重启它。

控制台入口：

- Mini PC入口：<http://192.168.54.49:8768>
- Q1直接入口：<http://192.168.54.41:8768>

两处入口连接同一Q1服务，状态中的`deployment.inference_location=Q1`、`architecture=aarch64`、`ort_threads=1`用于确认实际执行位置。

## 操作步骤

1. 打开控制台，在“操作口令”输入本项目私有文件`/home/kyokagong/code/embodied_edge/q1/deploy/supermimic/private/operator-key`中的口令。口令没有嵌入页面，也没有写入报告。
2. 点击“启动本体推理”，查看真实22关节、IMU反馈和候选目标。
3. 从8条舞蹈中选择一条，点击“运行舞蹈推理”。需要队列时先“加入队列”；重复网络请求不会重复入队。
4. 查看参考进度、模型步数和候选目标更新。偶发反馈间断时任务暂停进度，等待真实新反馈后继续；过期数据不用于推理。
5. 点击“取消舞蹈”或“停止模型任务”。实时监视继续，模型任务停止，独立本体保护输出状态仍单独显示。

“运行舞蹈推理”是当前观测模式的任务操作，不代表机器人已执行舞蹈；没有把电机接管确认字段改为已通过。实际手柄沿用标准映射：Start／RB长按启动任务、A站立策略、X行走策略、Y选中舞蹈、B／Esc停止。物理手柄操作需要浏览器HTTPS或localhost安全上下文，可使用现有Mini PC SSH连接转发本机8768端口；本次没有注入模拟手柄帧。

## 真机流程验证

实际通过Mini PC入口调用Q1服务，完成：

`启动观测推理 → 选择舞蹈 → 入队 → 相同request_id重试去重 → 运行舞蹈推理 → 参考进度推进 → 取消`

所选片段为`dance1_subject3`，参考帧从2推进到158，期间完成156次模型推理；真实关节与IMU计数均推进。模型没有发送执行器目标，本体保护会话、已采用策略序号和故障保持不变。测试结束已取消任务，控制台可直接开始新的操作。

首次测试发现反馈间断会中止观测任务，已修复为只在新反馈有效时推进。物理执行模式的反馈时限、控制间隔故障规则均未放宽。

证据：`q1/rust/reports/onboard-real-flow.json`、`onboard-edge-deployment.json`、`onboard-edge-arm64-validation.json`。物理手柄按键和机器人全身动作尚未验收，不能把本轮任务流验证写成动作质量通过。

## 文件与维护

Q1当前服务目录：`/home/run/q1-supermimic/onboard-fc1d855fb21d`。

- 可执行程序：`bin/q1-edge`；启动入口：`start`；进程PID和日志：`state/edge.pid`、`state/edge.log`。
- 模型目录：`/home/run/q1-supermimic/models/iter22100-onnx`。
- 本体Rust ROS2保护器：`/home/run/q1-supermimic/guardian-8772e2f1fb90`；其PID保持不变。
- 私有口令：服务目录内`private/operator-key`，与Mini PC项目私有口令一致。

本体程序先在项目内交叉构建、校验62个ARM64依赖对象，再放入Q1版本目录。替换仅针对明确包含`--shadow-only`的本任务旧进程，不关闭原厂服务，不覆盖原厂文件。

Mini PC服务状态／日志：

```bash
cd /home/kyokagong/code/embodied_edge
Q1_SETTINGS_FILE="$PWD/q1/deploy/supermimic/private/mini.env" q1/deploy/supermimic/run.sh ps
Q1_SETTINGS_FILE="$PWD/q1/deploy/supermimic/private/mini.env" q1/deploy/supermimic/run.sh logs --tail=30
```

构建与部署入口：`scripts/build-onboard-edge-arm64.sh`、`scripts/deploy_onboard_edge.py`。Mini PC通过`compose.onboard.yaml`启动固定目的地的Rust转发器。启动脚本保持显式启动模式，进程重启不恢复任务队列、不自动取得执行器控制权。

若Q1整机重启，需要先按现场状态启动Rust ROS2读取／保护进程，再使用服务目录的`./start 模型目录 保护器目录`启动观测服务；该入口要求本地bridge socket已就绪。本轮没有配置整机开机自启或更改厂商启动流程。

未提交Git，未使用superpowers，未进行Mock机器人操作。
