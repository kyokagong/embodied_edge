# 验证记录（2026-10-30）

以下为2026-10-30的历史验证记录，当时交付目录为 `/data/kk/embodied_edge/q1`。当前代码位于 `/home/kyoka/code/embodied_edge/q1`，容器部署方式见README。历史结果不代表当前主机或设备已经完成联调；软件、模拟输入和隔离通信验证不能代替实机测试。

## 已运行

|验证|结果与范围|
|---|---|
|Python unittest|73/73通过：ROS后端26、控制状态机19、Pico19、HTTP边界9；在交付目录运行|
|Node界面行为与语法|`node tests/check_ui.cjs` 与 `node --check q1_edge/web/app.js` 通过|
|真实 Chromium 操作链路|校准→使能→行走→RB保持→松开归零→停止→Pico校准/遥操→双Grip→手移动→Grip松开保持→失焦停；无JS错误|
|Pico模拟收敛|从SDK官方HOME左手向上35mm，最终位置误差0.000101563m（约0.102mm）；这是纯运动学与mock，不是本体误差|
|界面尺寸|1600×1100桌面和390×844手机，无横向溢出；截图见本页下方|
|官方ROS消息包|固定SDK提交7310314eee058323a2c15ab1f4f947df4112573e，完整aimdk_msgs在Ubuntu22.04 / Humble镜像中colcon编译成功|
|Pico镜像与原生绑定|`embodied-q1:pico`构建成功；Python3.10.12实际import xrobotoolkit_sdk成功，8个所需API均存在；--diagnose四项依赖全部可用。未调用SDK init或连接PC Service|
|真实ROS2 DDS|`--network none`、域117、ROS_LOCALHOST_ONLY=1，真实rclpy/generated messages与假MC服务节点回环通过|
|DDS验证内容|启动只读、显式注册源、获得控制权、50Hz速度心跳、模式确认、上肢完整10关节/20-3增益、停止保平衡并释放源、关节反馈断流看门狗|
|SDK与FK来源|vendor62文件均保留Git blob/SHA256，FK树有单独SHA256与来源许可；校验应与provenance一致|

基础镜像固定摘要 `ros:humble-ros-base-jammy@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855`。Docker产物为本机镜像，不是公网发布。

Pico首次从files.pythonhosted.org下载因网络超时失败；使用可配置公开镜像后构建通过。x86_64 wheel为`xrobotoolkit_sdk-1.1.0-cp310-abi3-manylinux_2_34_x86_64.whl`，SHA256 `ec8ce525110cfa742b901f8f43b4429692b1e825f1586903499732218fab3a89`，已与官方PyPI JSON元数据核对一致。

## 多轮检查与修正

1. **协议检查**：纠正“位置站立即平衡”的假设，采用WALK_RUN零速度；模式切换检查最终RUNNING，保持原厂MC；上肢每帧完整10关节。
2. **运动学检查**：校准使用实测关节，不跳到固定HOME；SDK正肘HOME向前35mm不可达，独立24组初值/无中立偏置参考求解和最大臂长计算均确认至少4.951mm残差，未放宽阈值掩盖问题；补正常可达方向测试。
3. **独立集成审查**：复现并修复同帧“停止+使能”导致重新使能、原厂遥控抢占后stop重新激活自动模式恢复两项问题，均先失败回归再修复，复核通过。
4. **真实浏览器检查**：修正页面刷新导致模拟Pico时间戳回退；补输入所有者更换撤销校准和HTTP操作身份校验。等待手部目标实际收敛，而不是仅检查关节开始变化。

## 可复跑

```bash
cd /data/kk/embodied_edge/q1
python3 -m unittest discover -s tests -v
node tests/check_ui.cjs
node --check q1_edge/web/app.js
# 默认mock服务运行后；Playwright是可选的验收依赖，不是程序运行依赖。
python3 scripts/check_browser.py http://127.0.0.1:8766/
```

浏览器脚本依赖 Playwright + Chromium；先安装到独立测试环境。脚本会验证backend必须是mock，输出到 `/tmp/edge-q1-browser`，不应对真实后端执行。DDS自检的隔离Docker命令见README。

![桌面Pico遥操验证](screenshots/teleop.png)

[手机尺寸截图](screenshots/mobile.png) · [浏览器结果JSON](browser-result.json)

## 未验证与明天待办

- 实际Q1固件/网卡/DDS域/时钟同步、10关节软硬限位/方向/零位、MC切换和源超时的固件行为。
- Pico4Ultra应用与PC Service兼容版本、头显遮挡/断网/跟踪丢失、真实手柄映射和无线延迟。
- 实机平衡、站立、步态、上肢20/3增益、自碰撞、物理急停与原厂接管/恢复；UI与IK不提供这些结果的保证。
- 上肢远离HOME时软件停止不会自动切回行走；保守保持和原厂遥控器接管流程需要现场验证。
- SuperMimic推理与低层真机执行尚未接入；本版本没有相机视频、手指控制或全身动捕。


## 2026-11-01：ARM64 Humble 开发容器

当前目录 `/home/kyoka/code/embodied_edge/q1`。实际构建镜像 `embodied-q1:humble`，架构arm64；Ubuntu22.04、Python3.10.12，使用官方Humble基础镜像。构建通过现有终端代理参数完成，没有修改Docker daemon配置。

|验证|本次结果|
|---|---|
|SDK构建|完整vendored aimdk_msgs编译通过，产物 `/opt/q1_ros/install/aimdk_msgs`，约1分21秒|
|完整Python测试|在镜像内、`--network none`下75/75通过；含新增Shell分发和旧CLI参数兼容测试|
|入口回归|修改前普通Bash命令被错误转成Q1参数，新增测试失败；修改后输出和退出码保持测试通过|
|SDK / ROS依赖|Python3.10.12下numpy、rclpy、aimdk_msgs可用；保留apt NumPy1.21.5；pip check、dpkg --audit通过|
|真实ROS2 DDS回环|独立`--network none`容器、域117、ROS_LOCALHOST_ONLY=1：真实DDS与生成类型支持、只读启动、控制源仲裁、速度心跳、完整上肢流、停止释放和反馈断流看门狗全部通过|
|默认启动|实际执行scripts/run-docker.sh，宿主机8766端口可读取Mock状态，enabled=false，速度为零|
|代码挂载|inspect确认当前q1目录读写挂载到`/opt/q1`，SDK产物保持可用|
|共享网络|inspect为Network=host；容器与宿主机`/proc/self/ns/net`相同|
|环境变量|传入的ROS_DOMAIN_ID=117得到保留，ROS_LOCALHOST_ONLY=0，RMW_IMPLEMENTATION=rmw_fastrtps_cpp|
|交互Shell|docker exec打开Bash自动加载Humble与SDK；从/tmp启动脚本执行Bash也通过|
|基础工具|cc/c++、cmake、git、colcon、curl、ping、ip、ps、less、vi均可调用|
|独立代码审查|未发现Critical、Important或Minor问题|

验证结束后已退出并删除本次Mock容器，保留构建镜像。未进行实机Q1/Pico联调，未在ARM64上安装可选Pico绑定，也未运行x86_64镜像。PyPI元数据确认xrobotoolkit-sdk1.1.0提供ARM64 wheel且不声明额外依赖；实际Pico安装与设备运行仍须单独验证。
