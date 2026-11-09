# Q1 BlinkMimic部署记录：2026-11-09

已沿用原目录 `/home/run/q1-supermimic` 完成替换。控制台：http://192.168.54.41:8768 。本次没有执行站立、坐下、躺下或舞蹈任务，没有git提交。

## 当前部署

- 控制器：q1-policy，PID 30341。
- ROS2保护器：q1-rt --policy-v2，PID 30330。
- 运行目录：`/home/run/q1-supermimic/runtimes/e3793e81c0eeeb92ddf00a33b0a37b9bcc261312845474f8fb9734655201aa82`。
- 站立接入关节偏差限值：0.60rad；运行中的目标修正限制仍为0.50rad。
- 模型目录：`/home/run/q1-supermimic/models/817ad430b13e4a3b97082b18b882e765d92cc6aa049dc2b64bee007adaa7dd40`。
- 模型：原生双输入EMA48000，SHA256 `65fc94df9b972239d6bd46c2d147cf88174783b9de6e522e90f097407709f8f6`。
- 执行配置SHA256：`e33d406028847c3006779a26daf07c01ed70d3c70f957665526b116c07084057`，与原部署一致。
- RT主控制线程CPU7/FIFO20；DDS、ROS接收及日志线程保留原CPU集合，策略CPU4/5。

## 本体验证

本体实际执行55组golden对照，最大动作误差 2.14576721e-06。没有以仿真或替身反馈作为实机验收。新的网页已通过实际Chrome只读检查：模型48000、站立按钮为蹲坐到直立、趴姿起身入口不开放，无脚本异常，没有发送动作请求。

修正调度后连续观察120个样本（约60秒），无故障，真实关节/IMU/推理/HAL计数持续推进。实测策略调用 50.011Hz、HAL发布 50.011Hz，采样反馈年龄最大 25.012ms。该窗口不能作为长期实时性或动作质量验收。

最终为RELEASED / DAMPING、enabled=false、22轴Kp全0、低阻尼。机器人可由操作者明确选择后续操作，本次没有自动接管或启动模型动作。

## 已发现并修正的部署问题

首次观察曾出现101.16ms关节/IMU反馈断档并触发100ms保护锁存，未掩盖该记录。发现启动器在exec之前taskset，导致全部15个RT线程被限在CPU7，与旧版仅限制控制主线程不同。已用真实OS线程回归测试复现，恢复为middleware初始化后仅绑定主控制线程。100ms反馈阈值未改变，独立复查通过。修正后重新部署并完成上述观察；未通过反复清故障或放宽保护获得通过。24项部署工具测试、完整Python134项测试通过。

## 初次部署时的站立接入限制记录

只读实测姿态检查表明当前未找到合格的蹲坐站立参考接入帧。最近帧为floor_fold 23.58秒；左肘实测约0.67rad、参考1.40rad，最大关节差约0.73rad，超过0.5rad接入限制。根部重力方向差约0.11rad，处于0.45rad限制内。此处没有通过发起站立请求测试，也没有放宽限制。服务与模型部署成功不等于当前姿态可以直接开始站立；需要后续针对实测起始姿态处理接入。

## 旧版本归档

原站立、guardian、onboard版本、工具及22100模型共66项移入：`/home/run/q1-supermimic/legacy-20261109-before-blinkmimic-48000`。保留历史日志和摘要以便追溯。活动runtimes与models各只有当前一版，进程清单只有本项目q1-policy与q1-rt；没有修改厂商服务或驱动。

## 证据

- `rust/reports/policy-deployment.json`：当前部署身份、PID、预检及初始状态。
- `rust/reports/policy-live-final-20261109.json`：修正后的本体连续观测和线程配置。
- `rust/reports/policy-ui-readonly-final-20261109.json` 与同名PNG：实际Chrome只读页面检查。
- `rust/reports/policy-standing-entry-readonly-20261109.json`：真实姿态匹配诊断。
- `rust/reports/policy-affinity-diagnosis-20261109.json`：首轮调度差异及反馈超时记录。
- `rust/reports/policy-legacy-archive-20261109.json`：归档清单。

以后更新模型请使用 `update_policy_model.py --previous-deployment q1/rust/reports/policy-deployment.json ...`。工具自动沿用报告中的原目录，保持RT进程和执行配置。不要使用已归档的旧stand脚本与旧部署报告更新当前服务。

## 后续站立接入容差调整：已部署

用户反馈0.556rad偏差容易被0.50rad门槛拒绝，要求适度放宽。新版本将吊杆支撑蹲姿站立的接入偏差单独设为0.60rad；RT运行中的0.50rad目标修正限制、0.45rad倾斜限制以及速度、扭矩、温度和反馈时间检查均未改动。模型、参考动作与执行配置保持原摘要。拒绝提示新增最超限关节和实际限值，`GET /api/state`新增`standing_entry_limits`。

回归测试先复现0.556rad接入失败，再验证0.556/0.599rad可接入、0.601rad仍拒绝，倾斜、角速度、关节速度及IMU有效性检查仍生效。完整Rust56项、Python134项、实际模型55组golden对照和ARM64共63项依赖闭包检查通过。此处验证的是接入逻辑，不能作为模型实机站立成功的证据。

准备阶段的只读检查最初读到41.19ms关节/IMU时间差故障（原限值40ms）；后续40个样本中未再读到故障，采样最大时间差2.517ms。当时读取到STAND、enabled=true，用户选择继续测试，因此准备阶段没有上传或重启服务。随后用户明确授权部署；重新确认RELEASED、enabled=false、22轴刚度全0且无故障后，执行了本次替换。没有发送reset或任何站立、坐下、躺下、舞蹈请求。

准备包身份：`e3793e81c0eeeb92ddf00a33b0a37b9bcc261312845474f8fb9734655201aa82`。本地不可变目录为`q1/artifacts/policy/prepared-runtimes/`下同名子目录；压缩归档为`q1/artifacts/policy/releases/runtime-e3793e81c0eeeb92ddf00a33b0a37b9bcc261312845474f8fb9734655201aa82.tar.gz`。详细校验结果在`rust/reports/policy-standing-entry-tolerance-prepared-20261109.json`。RT源码和HAL协议没有修改；构建工具重新编译了两个程序，新的二进制摘要均已记录。

本次使用原部署报告进行受保护替换，实际执行命令如下；执行前的旧报告已另存为`rust/reports/policy-deployment-before-entry-tolerance-20261109.json`：

```bash
python3 q1/scripts/deploy_policy.py \
  --runtime q1/artifacts/policy/prepared-runtimes/e3793e81c0eeeb92ddf00a33b0a37b9bcc261312845474f8fb9734655201aa82 \
  --model q1/artifacts/policy/models/817ad430b13e4a3b97082b18b882e765d92cc6aa049dc2b64bee007adaa7dd40 \
  --base /home/run/q1-supermimic \
  --previous-deployment q1/rust/reports/policy-deployment.json \
  --host 192.168.54.41 \
  --ssh-key /home/kyokagong/code/QY01BAL2610000105/id_ed25519_QY01BAL2610000105.lf \
  --sudo --report q1/rust/reports/policy-deployment.json
```

整包更新已重启本项目RT和策略服务。新服务健康检查通过，本体API确认站立接入关节限值0.60rad、重力方向限值0.45rad。模型和执行配置与更新前摘要完全相同。本体重新执行55组golden对照，最大动作误差2.14576721e-06；未把数值验证作为闭环动作资格。新服务以RELEASED、enabled=false、22轴刚度全0启动。没有Git提交。

部署后再次连续只读观察100个样本、50.306秒：无控制器或RT故障，始终失能、释放且22轴刚度全0；实测推理49.994Hz、HAL发布50.014Hz，采样反馈年龄最大28.530ms，关节/IMU时间差最大3.144ms，采样电机温度最大51℃。RT主线程CPU7/FIFO20及middleware的多CPU调度均已核验。此窗口不代表长期实时性或物理动作成功验收。新部署汇总为`rust/reports/policy-standing-entry-tolerance-deployed-20261109.json`，原始采样为`rust/reports/policy-standing-entry-live-20261109.json`；后续更新继续使用已经刷新为本次身份的`policy-deployment.json`。
