# Q1 本体 SuperMimic 站立、舞蹈链路与真机记录

日期：2026-11-07。当前部署以本文件及 `rust/reports/real-stand-deployment.json` 为准。此前 `onboard-*` 观察推理服务和单肘保护程序已退出运行。

## 部署位置与入口

运行链路全部在 Q1：浏览器 UI → 本体 Rust `q1-stand` → 本地 Unix IPC → Rust ROS2 `q1-rt` → 原厂 device_manager → 电机。Mini PC 负责开发、交叉构建、部署和读取报告。

控制台：<http://192.168.54.41:8768>。无需操作口令，直接点击“一键站立”。站立准备完成后，选择舞蹈并点击“播放舞蹈”；点击“取消并恢复站立”会用5秒平滑返回播放前站立目标，再恢复零速度模型保持。播放到末尾也自动返回站立。停止按钮仍进入当前姿态保护保持，解除接管按钮仍进入零刚度低阻尼。

故障发生后，待实测关节静止，点击“故障复位”；复位继续保持实际姿态。然后显式点击“站立”开始新的任务。停止按钮请求保护器捕获当前实测姿态并有界保持。该软件保持不等同于独立硬件急停。

Q1 模型目录：`/home/run/q1-supermimic/models/iter22100-onnx`。服务版本目录和摘要记录在开发项目 `rust/reports/real-stand-deployment.json`。版本目录包含 `bin/`、`lib/`、`config/hardware.json`、`state/{rt,stand}.{pid,log}`。完整模型输入追踪仅保存到本体 `private/model-trace.json`，权限0600，通过已有 SSH 读取；没有开放张量下载接口。

## 本体数据到实际输出

1. `q1-rt` 订阅真实 `/aima/hal/joint/state` 全22轴位置、速度、力矩、温度和 `/aima/hal/imu/state` 四元数、角速度，核验完整性、有限数值、采样时间及消息年龄。
2. 反馈按名称转换为固定模型顺序。Rust 构造10帧×78维历史、模式、零速度命令、相位和590维目标，共1376维。严格按接入指南第4.1/5节，前一动作使用上一策略周期交给action manager的裁剪raw（±20），尚未乘0.25/加home；不把延迟、斜率或PD力矩限幅后的电机目标反算值混入历史。显式开始新任务时初始化历史，进入策略阶段保留历史。
3. Q1 上 ONNX Runtime 1.23.2 执行固定 `iter22100` 模型。观测归一化已在 ONNX 内；运行时不重复归一化。输出为22维 raw action。
4. 站立准备先匹配当前实测姿态，以模型名义 Kp/Kd 接管，再按平滑插值进入模型 home。位置有速度限制；准备完成、实测home误差处于本次0.5rad保护跟踪范围内、关节静止且倾角低于约20度后渐入零速度模型策略。该入模条件允许真实承重PD静差，不等同于最终站立验收。
5. 策略位置为 `home + 0.25 * clip(raw, -20, 20)`，本次吊装试运行额外对残差和软件 PD 力矩估算限幅。HAL 发布与约50 Hz观测/推理解耦，模型未更新时保持最近有效目标，过期则进入保护保持。本轮沿用用户已确认站立的50 Hz实际HAL发布配置；内部保护调度仍为500 Hz。
6. `standing_verified` 只由真实策略阶段的倾角、关节速度和跟踪误差连续满足判据置位，任何不稳定或故障会撤销；不能通过 UI 请求或部署配置直接置为通过。

本次是接地承重、吊带防倒条件下的站立试运行。未把 `confirmed`、驱动器独立看门狗、IMU安装变换和故障参数的验收字段伪造为已确认。IMU 到 pelvis 暂按单位变换进行本次有支撑试验。

## HAL 固定槽位：必须与模型顺序分开

本体 `device_manager release-v1.2.1.17` 的 `q1::TopicManager::JointSubscriberHandle` 直接按数组槽位读取命令，未使用 `JointCommand.name` 查找关节。必须发送完整22项，顺序为有效 MotorModule 配置中的上肢8、左下肢7、右下肢7。

对应模型索引：

```text
14 15 16 17 18 19 20 21 | 13 0 1 2 3 4 5 | 12 6 7 8 9 10 11
```

反馈和模型数组始终使用模型顺序，只有 HAL 序列化边界转换槽位。增益、位置、速度、力矩和故障保持使用同一转换。保护器检查限幅后实际下发命令的跟踪误差，避免把受限候选目标误判为电机失跟踪。实测速度上限为2.4rad/s，策略目标速度仍为0.8rad/s；故障捕获实测姿态后保留支撑增益与力矩限制。驱动内已有电气方向、零偏换算，应用层不重复施加。

最初实现误以为驱动按名称匹配，按模型顺序发送，导致“保持当前角度”也出现非预期运动。真机数据暴露问题后，通过读取本体有效 YAML 及已安装驱动回调确认固定槽位并修复。先前部分数组单肘试验也不能视作有效全身 HAL ABI 验证。证据保存在 `rust/reports/stand-hardware-map.json` 及 HAL 回调分析文件；后续换驱动版本必须重新核对。

另修复一处真实时间竞争：先读取IPC反馈、取得状态锁，再获取当前单调时钟，避免把比循环起始时刻更新的反馈误判为过期。反馈时限未因此放宽。0.03rad目标单步比较允许1e-6rad浮点舍入误差，插值后再夹紧机械范围。随后根据实际策略有效期中止，将本次静止站立试验的策略输出期限从60ms调整为120ms；本体关节/IMU反馈仍要求100ms内且源采样时间有效。起身目标速度为0.08rad/s，模型策略阶段独立允许0.8rad/s（单步仍不超过0.03rad），残差按实际关节范围约束；脚踝俯仰软件PD力矩上限从5Nm调整为8Nm，以减少实测饱和。

## 构建与验证

开发机 Docker `q1-sm-dev` 内运行 `scripts/build-stand-arm64.sh`。Rust 后端和 ROS2 SDK 均交叉构建为 ARM64；ONNX Runtime 使用 ARM64 动态库。`scripts/validate-guardian-arm64.py` 校验依赖闭包，目前63个对象通过。

`scripts/deploy_real_stand.py` 校验文件 SHA256，上传版本目录，先确认本体可加载程序，再替换本任务进程。更换UI/模型进程时可保留独立RT；修复HAL ABI或RT时用显式替换分支恢复实测姿态保持，不修改厂商配置或关闭厂商设备服务。

`scripts/check_real_stand_onboard.py` 必须在 Q1 上运行，调用本机实际控制API并采集真实ROS2编码器、IMU、HAL发布及已采用序号。它不生成机器人反馈或模拟手柄。`--takeover-only` 执行短时真实接管，完整站立验证使用 `--seconds 70 --leave-standing`，成功时才保留站立闭环。

真机原始报告位于开发项目 `rust/reports/real-*.json`。最终状态及本轮结果在下方追加；真实接管、站立准备和模型站立分别验收，HAL发布成功不能代替实际站立。

未提交 Git，未使用 superpowers，未进行 Mock 机器人操作。


## 舞蹈接入与取消返回

`reference::load` 校验并加载模型包的8条舞蹈及独立standing参考。界面默认选择 `dance2_subject1`。只有启用且完成起身、正在站立模型阶段，才能接受 `play_dance`；播放中不能重复启动。`cancel_dance` 在非播放状态下幂等，不改变站立任务。

播放时每20ms用当前真实22轴/IMU状态更新历史，将mode设为1，读取该段 `root_R[f]` 和 `goal_590[f]`，执行同一ONNX模型，输出22维动作给原有本地HAL链路。参考帧只随50Hz策略周期推进，不随浏览器刷新或保护器调度推进；切换模式保留历史。开始播放、取消和播放结束时切换epoch，旧模式未完成推理不能进入新模式。原有站立动作限幅和增益保持不变。

取消记录当时目标，用5秒smoothstep过渡到播放前保存的站立目标，经过原有速度和机械限位，再恢复mode0。返回期间历史使用当前实际发送的原始目标；不把最后舞蹈动作一直作为previous action。非有限值、真实反馈过期等故障仍终止舞蹈并进入保护保持，不自动继续。

无需口令，也不再复制或读取口令文件。Host/Origin检查、客户端控制权和请求去重仍保留；本体在浏览器关闭后继续执行已经明确启动的任务。完整输入和HAL记录仍仅保存在本体私有日志目录，通过已有SSH读取。

舞蹈验证入口 `scripts/check_real_dance_ui.cjs --execute-stand` 使用实际Chrome点击实际本体页面，先站立，再播放 `dance2_subject1` 30秒，然后点击取消并确认恢复站立。它不生成传感器数据，不模拟ROS或ONNX。结果记录在 `rust/reports/real-ui-dance2.json`，完整输入中mode1/590维goal及对应HAL记录另由本体私有飞行日志核对。30秒流程验证不等同于233.66秒整段舞蹈质量验收。


## 从失能状态重新验证：2026-11-07

用户要求暂停后，确认本体真实处于 `RELEASED / DAMPING`、`enabled=false`、零刚度。用户再次授权后，从该状态重新通过真实Chrome操作：一次点击“一键站立”，等待起身及模型站立阶段；选择 `dance2_subject1`，一次点击播放，播放30.001秒后一次点击“取消并恢复站立”。站立、播放、取消请求均返回200；浏览器无脚本异常，运行期间无保护故障。

本轮舞蹈参考推进约1500帧。取消后5秒平滑返回播放前保存的站立目标，再恢复 `POLICY_STAND`；测试结束时继续保持站立，没有自动解除位置接管。模型输入、ONNX推理与HAL发布均在Q1完成，Mini PC只运行浏览器验证和读取报告，没有远程逐帧生成动作。

记录：`rust/reports/real-ui-dance2-from-release.json`；页面截图：`rust/reports/real-ui-dance2-from-release.png`；私有本体日志核验摘要：`rust/reports/real-dance2-from-release-flight-audit.json`。当前运行目录 `/home/run/q1-supermimic/stand-2234af6dd867`，沿用已有独立RT和50Hz实际HAL输出。此次验收范围是起身、舞蹈播放和取消返回的流程；动作效果及完整舞蹈由用户后续验收。

本轮本体私有日志核验：1506个舞蹈模式输入均包含非零590维goal，1506个对应ONNX推理可匹配；输入形状及真实反馈/previous action核验无误。舞蹈期间HAL约50.001Hz、1506帧，无保护故障；返回过渡约5.014秒，最后返回目标与播放前保存目标最大差约4.71e-8rad。核验结束后再次只读确认仍为 `POLICY_STAND / ACTIVE`，舞蹈已停止、无故障，日志无丢失或写入失败。


## 速度上限调整为3.14 rad/s：2026-11-07

根据用户要求，22轴 `policy_velocity_limit` 从0.8改为3.14 rad/s，`measured_velocity_limit` 从3.0改为3.14 rad/s。策略层和保护器的参数校验同步允许该值。50Hz每周期20ms对应最大目标变化0.0628rad，`max_step_rad` 从0.03同步放宽至0.0628，避免旧单步限制使有效速度仍不超过约1.5rad/s。

起身准备的 `velocity_limit=0.08rad/s`、20秒过渡、动作幅度、机械限位和力矩限制沿用原值。模型策略、舞蹈与取消返回共用新的目标速度上限；取消返回仍执行5秒平滑过渡。实测速度保护保持连续力矩调节语义，3.14是调节阈值，不能解释为机械实际速度绝不会超过该值。

页面和 `/api/state` 显示实际加载的策略速度上限，状态也包含实测速度限制、准备速度及单步限制。开发机完成Rust检查、ARM64交叉构建和63个依赖对象校验。部署替换控制器和保护器时使用 `--resume-released`，保留用户当前零刚度低阻尼失能状态；不自动站立或播放，不执行新速度下的运动测试，由用户随后通过UI测试。部署摘要以 `rust/reports/real-stand-deployment.json` 为准，旧摘要保存在 `rust/reports/before-velocity-314-deployment.json`。

实际部署目录 `/home/run/q1-supermimic/stand-aeb26c47f2fc`。已核对控制器API和独立RT启动清单，两层加载的22轴速度限制均为3.14rad/s、单步0.0628rad。冷启动反馈过期锁存已通过保留失能的release请求清除，随后确认 `RELEASED / DAMPING`、`enabled=false`、全部Kp为0、无故障。记录为 `rust/reports/velocity-314-live-check.json`；未进行新参数运动测试。


## 可配置的软件力矩与无需编译的参数应用

本轮按用户要求仅将 `stand_trial.effort_limit` 全22轴乘以2：髋pitch/膝18→36Nm、髋roll10→20Nm、髋yaw6→12Nm、踝pitch8→16Nm、踝roll5→10Nm、腰roll10→20Nm、腰yaw6→12Nm、肩/肘4→8Nm。推理50Hz、HAL发布50Hz、目标/实测速度3.14rad/s、幅度和起身准备参数沿用原值。

参数文件是开发项目 `config/supermimic-stand-trial.json`。数组使用模型22轴顺序，与 `/api/state.joint_names` 一一对应，不能按HAL槽位排列。`effort_limit` 为软件PD估算约束；`policy_velocity_limit`、`measured_velocity_limit`、`policy_residual`、`velocity_limit`、`prepare_seconds` 和顶层 `max_step_rad` 等都由JSON加载。

原来写死的试运行力矩/速度上界移至 `stand_trial.tuning_limits`。本轮该配置的max_effort/max_velocity来自模型 `deployment_contract.json` 的36/21Nm和36/32rad/s包络，仅用于调参校验，不作为厂商硬件额定值。实际运行限制仍是effort_limit及对应速度字段。机械范围、数据有效性和温度等检查保留。

以后修改JSON后，先在UI“解除接管”，再在开发项目根目录运行：

```bash
python3 q1/scripts/apply_onboard_tuning.py --config q1/config/supermimic-stand-trial.json
```

该命令不调用Cargo，也不需要编译环境。它确认本体失能，检查本地/已部署两个程序SHA256一致，复用现有程序、暂存新配置，先在本体执行 `q1-rt --validate-only`。校验通过且本体仍失能才替换本项目两个进程，使用同一配置版本启动并保留零刚度低阻尼；由用户再显式站立。参数应用会重启进程，不是运动中无缝热改。校验失败不停止现有进程。当前UI逐关节显示实际加载的软件限矩，状态返回配置路径和SHA256。

部署记录在 `rust/reports/real-stand-deployment.json`，仅配置应用前记录在 `rust/reports/video-314/before-effort-2x-apply.json`。本轮用相同二进制先部署旧力矩配置，再通过上述工具应用加倍值，验证后续调参无需重新编译。


### 软件力矩加倍后的本体结果

本体运行目录 `/home/run/q1-supermimic/stand-bdcd910f1d6e`，配置SHA256为 `79700428ed54d205d37ca86b03286e06e55c17000a705ed5077f8878e37fc623`。配置应用前后两个程序SHA256完全相同；本体校验器拒绝负力矩参数时原RT进程保持运行，站立过程中参数应用工具拒绝重载且没有重启进程。

真实Chrome从失能状态启动站立，播放 `dance2_subject1` 约30秒，点击取消后用约5秒返回播放前站立目标并恢复 `POLICY_STAND`；新配置值在动作前由UI状态和配置SHA核对。期间无保护故障；1507个舞蹈目标包含逐层日志，1508帧实际HAL约50.001Hz。最后返回目标与保存站立目标最大误差约2.54e-8rad。测试后保留站立保持，日志无丢失/写盘失败。

新限矩实际生效：右髋roll约43.6%的发布帧估算输出超过旧10Nm限额；本轮实际所需输出峰值约12.69Nm，处于新20Nm限制内。旧“力矩”bit8同时涵盖速度减力，必须使用新增before_effort_tau/after_effort_tau及speed_scale分离统计，不能把旧bit8直接当作纯力矩裁剪比例。幅度与斜率限制仍沿用原值，因此流程跑通不代表完整舞蹈动作质量已验收，也不应与不同片段dance1的比例直接做因果比较。

报告：`rust/reports/video-314/real-ui-dance2-effort-2x.json`、`dance2-effort-2x-analysis.json`、`effort-2x-config-proof.json`、`active-config-reload-rejected.json`。当前动作效果由用户后续验收；未提交Git，未进行Mock操作。

本轮按新增阶段数组分开统计：纯软件力矩硬裁剪为0%，速度减力约4.58%；策略幅度/机械范围裁剪约99.20%，目标斜率限幅至少一轴生效约45.12%。这些是本轮dance2的实际统计，不能由此声称所有跟踪问题已经解决。


## 再次放宽：采用模型包络，2026-11-07

用户确认采用模型包络：额外的3.14rad/s目标及实测速度限制改为模型原有逐关节32/36rad/s；不是取消机械或模型包络。目标单步上限同步从0.0628改为0.72rad（36rad/s×20ms），32rad/s关节在正常20ms周期仍由策略层限制到0.64rad，避免旧斜率上限继续把速度卡住。起身准备0.08rad/s及20秒过渡沿用原值，推理50Hz、HAL发布50Hz不变。

软件力矩在上轮数值上再乘2，并与配置中36/21Nm模型包络取较小值。实际22轴值为 `[36,36,24,36,21,20,36,36,24,36,21,20,36,24,16,16,16,16,16,16,16,16]`Nm。因此髋pitch/膝仍为36Nm，髋roll为36Nm、腰roll为36Nm，踝pitch为21Nm，上肢为16Nm；并未把髋/膝放到72Nm。模型原始速度来自deployment_contract.json，随包official.urdf对腿部大关节记录effort36和velocity36，对小关节记录effort22和velocity32；训练包小关节采用21Nm包络。此处不将模型或URDF数据声称为厂商已确认的连续额定能力。

只修改JSON，通过 `scripts/apply_onboard_tuning.py` 复用既有程序应用；不重新编译。参数应用前先解除位置接管，之后保持失能，不自动站立/播放。本次没有进行更高速度/力矩下的运动测试。请求值、截限后实际值及配置摘要见 `rust/reports/tuning-model-envelope/requested-and-applied.json`；旧配置和部署摘要保存在同目录便于恢复。

实际参数目录 `/home/run/q1-supermimic/stand-4a3b3870543e`；配置SHA256 `f48206ceabaa9e2fe030b51c0416fd216ed6d8d1e1aff2b27c4ea59369857a2a`。控制器API与独立RT启动记录中的力矩/速度数组已逐轴核对，程序SHA256未变；最终为 `RELEASED / DAMPING`、Kp全0、无故障。核验报告 `rust/reports/tuning-model-envelope/live-validation.json`。


## 统一策略输出：移除站姿试运行包络

根据用户明确的单模型设计，后续以本节取代早期“站立试运行幅度/5秒姿态返回”的控制语义。`q1-stand`保留二进制名称兼容部署脚本，实际职责为本体统一策略控制器。

接管前匹配真实关节姿态，平滑准备模型home并进行短时模型接管；完成后站立使用mode0、零速度命令与空goal，由同一ONNX策略持续输出。舞蹈使用mode1、逐帧root_R/goal，仍调用同一个模式无关的 `supermimic::action::map`。取消或播放结束只切换到mode0、零速度条件；保留观测历史和上一有效策略动作，通过epoch隔离旧结果。删除原来的5秒保存姿态插值，恢复站立由模型完成，不能用切换成功代替实际站稳验收。

共用动作映射只包含模型原生raw clip、scale/home与机械位置范围，不再包含 `policy_residual`。当前JSON删除该字段；旧JSON可被读取但该字段不会参与控制。预接管home必须与固定模型契约一致。机械边界、已确认采用的力矩/速度包络、数据过期、温度和显式停止仍由独立RT保护；这些保护不使用站立home的小范围盒子限制舞蹈。

`policy_output_mode=model_native`、`cancel_behavior=zero_velocity_condition`和`policy_ready`可在状态接口核对。起身至模型接管结束前不能播放舞蹈，避免把初始接管blend误用到舞蹈。逐推理日志记录与真实控制路径共用函数产生的native_action，便于失能状态下检查模型输出转换。

Rust检查和ARM64依赖验证结果位于 `rust/reports/unified-policy/`。使用先前真实舞蹈日志回放动作映射验证，并在本体失能时验证真实ONNX输出和日志；不生成ROS反馈，不启动扩大幅度后的站立/舞蹈。新条件切换的实际站立和舞蹈恢复效果需要后续真机验收。

实际部署 `/home/run/q1-supermimic/stand-0200c7400272`。在Q1回放1721帧真实历史模型目标，统一原生映射与机械边界的最大差为0；左肘目标最低0.61839467rad，已不再被抬至旧1.15rad下限。该回放不打开硬件IPC；另以本体失能时的真实传感器/ONNX日志核对当前相同映射函数。最终为RELEASED、Kp全0、无故障。报告 `rust/reports/unified-policy/real-recorded-action-replay.json` 与 `live-validation.json`；未执行新版扩大幅度后的动作测试。


## 上一动作语义修复与真实JIT/ONNX对照

统一策略的上一动作严格采用 `clip(last issued model raw, -20, 20)`，对应训练 `action_manager.action`。`NativeAction.policy_raw`与`motor_safe_q`明确分离，机械位置、安全速度及PD力矩保护不反算回策略历史。本文早期把机械位置裁剪后反算raw作为上一动作的描述已过时，后续以本节为准。

用三轮真实本体私有输入日志导出2664组原始观测（300组mode0、2364组mode1，含切换边界），未合成或修改观测。源JIT与CPU ONNX全部通过rtol=3e-4、atol=3e-5对照；最大绝对误差6.4671e-6，RMSE约6.0321e-7。与2662组具有对应本体ARM64 ONNX结果的记录继续比较：本体/JIT最大差6.0350e-6，本体/CPU ONNX最大差1.4305e-6。两个边界输入没有匹配到时间窗口内结果，未算作通过样本。

JIT摘要 `8b9ba29c505501fbf3dfafd7cc8ceb19f331c030e8c6532a8d4f6e39cb6fddba`，ONNX摘要 `249892ded509d3f694a0053bb5de245330da606867cbff7d62a3acab78a4623f`。复用了原导出依赖，并隔离临时venv基础解释器与Conda标准库不一致的问题，没有重导出权重。数值一致性表明这批真实输入下没有明显转换偏差，不代表修复后闭环舞蹈质量已验收。

部署先在本体暂存目录用真实历史模型输出回放NativeAction映射，检查policy_raw与clip(raw)一致、motor_safe_q仍在机械范围内，校验通过才替换本项目进程。保持50Hz推理、现有模型包络及零刚度低阻尼失能，不自动站立或播放。未提交Git，未使用Mock。

报告目录 `rust/reports/previous-action-fix/`，核心数值报告 `jit-onnx-real-log-parity.json`，原始真实输入0600私有保存于 `real-observations.json`。工具 `scripts/check_real_observation_parity.py` 只离线推理、不初始化ROS或发硬件命令。未来复核应在包含原导出依赖的Python环境运行。

实际部署目录 `/home/run/q1-supermimic/stand-c824c3ca422a`。替换前ARM64回放2662帧真实输出，policy_raw与训练clip定义最大差0，机械保护目标最大差0；899帧旧算法会因机械裁剪改变回填值。替换后已用本体失能实时ONNX记录核验policy_raw/motor_safe_q分离，配置SHA保持不变，最终RELEASED、Kp全0、无故障。`live-validation.json`记录只读在线核验，本轮未启动新版闭环动作。
