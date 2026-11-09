# SuperMimic 当前接入入口

当前部署已更新：Q1本地持续推理与Web服务、Mini PC Docker转发入口，真实任务操作流程已通过。[当前控制台与操作说明](2026-11-07-q1-onboard-console.md)。以下包含此前架构／评估阶段记录。


2026-11-07：当前任务固定使用 `q1_unified_6144_iter22100_20261107`，Rust + ONNX Runtime。已部署 Mini PC Docker 和 Q1 ARM64 保护器，处于真实联调阶段，尚未通过全身运动验收。

- [批准的设计实施方案](2026-11-07-supermimic-rust-integration-plan.md)
- [本体保护原理](2026-11-07-q1-onboard-guard-design.md)
- [真实测试、未通过项与当前执行器状态](2026-11-07-supermimic-deployment-results.md)
- [构建与运行说明](../deploy/supermimic/README.md)

下面保留旧版原厂 MC 阶段的历史说明。其中 28000 模型、BONES-SEED 及“尚无IMU”等状态不适用于本轮22100部署。

---

# 后续接入 SuperMimic 的边界

首版仅实现原厂MC站立/行走与Pico上肢遥操。页面的SuperMimic入口明确禁用，不会把仿真模型直接发送到本体。

当前仿真预览对应28,000 checkpoint：

```
/data/kk/data/supermimic_q1/releases/preview_028000_full/model.pt
SHA256 cad72984167bf95ccfadc55b9c009968656c6dd3d77db616ab11e08291911da6
```

这是已有仿真产物，不代表完成sim2real。下一阶段先在 `/data/kk/supermimic_vibed` 冻结当前模型的部署契约：50Hz actor、历史输入 `[1,10,78]`、目标输入 `[1,590]`、输出 `[1,22]`、关节顺序、`q_target=q_home+0.25*action`、全部归一化/历史初始化、动作库和控制增益。不要把SDK的上肢HOME混成训练HOME。

在增加真实执行器前须解决：

1. 获得官方可验证的IMU/基座姿态、角速度、关节角和速度接口及其坐标/时钟；当前公开SDK未核实到足够的IMU接口，不用假零值填观测。
2. 核实22关节排列、方向、偏移、硬件限位、扭矩/速度/增益、MC与HAL互斥切换及原厂恢复流程。公开SDK与模型限位存在冲突，要按实机资料解决。
3. 导出固定模型与观测/输出参考样本，做Python/部署推理的一致性测试、延迟和抖动测量，再做sim2sim闭环及不同初始姿态测试。
4. 新增单独的低层执行后端；SuperMimic、原厂MC与Pico不能同时写同一关节。切换流程、失联保持/卸力、倾倒和恢复策略先按厂商协议在支撑条件下验证。
5. 先静止参考和短小动作，再到有支撑的小幅舞蹈；全量11,006条BONES-SEED覆盖是仿真评估范围，不应自动成为真机可执行动作白名单。

现有UI、状态记录、输入校准与显式使能流程可以复用，但当前原厂MC后端的“停止=零速度保平衡”语义不能直接套到低层策略。后续部署可复用当前Humble容器，以镜像版本和模型hash固定推理环境；低层控制周期/实时性必须在实际控制器测量。
