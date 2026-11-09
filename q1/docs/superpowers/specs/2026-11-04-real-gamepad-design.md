# Q1 物理手柄接入修复方案

## 目标和现状

使用已接入本机的 MACHENIKE G3V2，通过现有浏览器 Gamepad API、HTTP 输入接口和 ROS 2 后端控制 Q1。当前容器已运行 ROS 2 后端，未使能。

只读检查确认：
- Linux 将手柄识别为 `event13/js0`；浏览器仍需要用户按键后通过 Gamepad API 暴露设备。
- 页面初始输入源为 simulator，真机后端拒绝该输入；已有 browser_gamepad 输入和按键动作链路可复用。
- Q1 `/upper_body_joint_states` 使用 `sensor_msgs/msg/JointState`，实测包含全部 10 个上肢及腰部关节，消息时间比本机约快 50 ms，位于已有 100 ms 容差内。
- 原代码仅订阅没有发布者的 `/aima/hal/joint/state` (`aimdk_msgs/msg/JointStateArray`)。
- 只读 `GetCurrentInputSource` 查询可返回：header.code=0，state.value=UNKNOWN(0)，reason=0，input_source.name 为空；当前后端要求 SUCCESS(1)，没有更新控制源反馈。之前超时记录仍留在状态中。

## 方案与取舍

推荐复用现有浏览器手柄链路，修改最少且沿用已有的输入所有权、同源校验、校准和断联停止。另起 Linux joystick 驱动需要增加设备透传和第二条输入链路，当前需求不需要。用户已授权直接实施此方案。

## 修改范围

1. 真机模式自动选择 browser_gamepad，禁用模拟输入选项，显示设备名、轴值、RB 状态和回中提示；初始按住按键不得触发动作。后端仍拒绝模拟输入。
2. 增加实际标准关节话题的适配，保留 SDK 原始话题兼容。标准消息必须具有完整、不重复的关节名和对应的有限位置值，并沿用时间戳、新鲜度及倒序检查；不得用零值补齐缺失反馈。
3. 仅针对只读 GetCurrentInputSource，兼容已观察到的 code=0、UNKNOWN、reason=0 返回，同时保持所有明确失败状态拒绝、控制源名称匹配及所有权时效检查。后续实机日志证明 SetMcInputSource 也返回 code=0 / UNKNOWN / reason=0，且官方 SDK 仅根据 header.code 判断注册/删除成功；因此源管理操作兼容这一组合，非零 RPC 错误码与明确业务失败仍拒绝，SetMcAction 的判定不放宽。
4. 保留默认按键：Back 校准；Start 显式使能；A 平衡站立；X 行走；左摇杆前后和横移；右摇杆水平转向；RB 保持运行；B 软件停止。Y 保持 Pico 功能，未接入 Pico 时不可用。
5. 用回归测试覆盖自动物理输入选择、按钮边沿、标准关节消息的有效和无效情况、只读服务返回兼容；隔离 DDS 回环覆盖手柄输入到命令输出、松开 RB 和断联归零。
6. 重建镜像、重启 ROS 2 后端，核验真机关节反馈和未使能状态。运动验收由用户在浏览器中通过真实手柄触发；不由代理在实机上生成测试运动。

## 验收和边界

- Python 全套测试、前端测试和断网 DDS 回环通过。
- 页面显示 browser_gamepad，用户按键唤醒后可看到实际设备和轴值；真机反馈包含 10 个关节且新鲜。
- 启动不注册控制源、不输出运动，用户校准并显式使能后才允许控制。
- 实机 Set 服务是否接受请求仍需操作员使能时验证；不能用服务发现或隔离测试代替实机验收。
- 保留上身硬件限位未确认锁定，不加入 Pico 绑定，不绕过原厂 MC。

## 实机使能故障复盘与 logger

第一次 ADD 实际返回 code=0/state=0，被误判失败后同名源残留，后续 ADD 返回 code=1。仅删除本程序 edge_q1 已返回 code0/state0；再次 ADD→DELETE 实机验证均返回 code0/state0，未创建速度发布者或调用动作切换。

新增异步诊断日志、可选轮转文件（5 MiB + 三个备份）、日志等级、按键/连接/动作/RPC/所有权/故障记录、每秒速度摘要和每五秒状态摘要。只读轮询及每秒输入采样归 DEBUG，HTTP 不记录 token/body。所有日志使用带 Z 的 UTC 时间。

## 实机控制权获取修复（用户已授权直接测试）

用户确认现场站稳、空旷并有人可急停，授权短时前进和后退。官方 Basic 模式保留原厂业务与安全策略，当前使用高层速度接口，不切高级模式、不关闭原厂 MC、不写 HAL。

实测 ADD→零速度不能获得 owner；显式 ENABLE 和 Reliable/BestEffort 对比仍为空。首个非零指令立即得到 owner，确认原先“先 owner 才允许任何非零”的循环等待。Get 还会保留已删除诊断源名称，不能用非空名称断定另一源仍活跃。

方案：每次 ADD 使用唯一 epoch 名；使能仅待命。明确 walk+RB+摇杆动作发起一次不超过 min(250ms,command_timeout,feedback_timeout) 的仲裁请求，各分量限0.2，仍由原厂 MC 按低于 RC 的优先级仲裁。迟到的首次确认必须拒绝；超时归零释放，不自动重试。已有 owner 丢失仍保持锁定；初始非 WALK RUNNING 明确拒绝。

两次成功的短时测试分别验证独立 SDK 和修复后的 Ros2Backend，均限制速度±0.2m/s、每方向≤0.6s、全程检查新鲜反馈/位姿/控制权，归零后 DELETE。不是精确步数控制；结果保存在 logs/bounded-motion-20261104.jsonl 与 logs/bounded-app-backend-20261104.jsonl。
