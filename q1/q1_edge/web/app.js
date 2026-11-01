(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const clientId = globalThis.crypto?.randomUUID?.() || `tab-${Date.now()}-${Math.random().toString(16).slice(2)}`;
  const token = document.querySelector('meta[name="api-token"]')?.content;
  const clockFormat = new Intl.DateTimeFormat('zh-CN',{timeZone:'Asia/Shanghai',hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false});
  const jointNames = ['waist_roll_joint','waist_yaw_joint','left_shoulder_pitch_joint','left_shoulder_roll_joint','left_shoulder_yaw_joint','left_elbow_joint','right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint','right_elbow_joint'];
  const jointLabels = ['腰部 Roll','腰部 Yaw','左肩 Pitch','左肩 Roll','左肩 Yaw','左肘','右肩 Pitch','右肩 Roll','右肩 Yaw','右肘'];
  const modeNames = {idle:'待机',stand:'平衡站立',walk:'行走',teleop:'Pico 遥操作',fault:'故障锁定'};
  const picoSliders = ['head-yaw','head-roll','left-x','left-y','left-z','right-x','right-y','right-z'];
  const jointRows = jointNames.map((name, index) => {
    const row = document.createElement('div');
    row.className = 'joint-row';
    row.innerHTML = `<span class="joint-label">${jointLabels[index]}</span><div class="joint-track"><i class="joint-actual"></i><i class="joint-target"></i></div><span class="joint-values">-- / <span>--</span></span>`;
    row.title = name;
    $('joint-list').appendChild(row);
    return {row,actual:row.querySelector('.joint-actual'),target:row.querySelector('.joint-target'),values:row.querySelector('.joint-values')};
  });
  let state = null;
  let online = false;
  let source = $('source-select').value;
  let sequence = 0;
  let lastTimestamp = 0;
  let inputBusy = false;
  let picoBusy = false;
  let pollBusy = false;
  let actionBusy = false;
  let controlEpoch = 0;
  let deadmanHeld = false;
  let suspended = document.visibilityState === 'hidden' || !document.hasFocus();
  let requirePadRelease = true;
  let padName = '';
  let padReleaseWaiting = false;
  let lastEvents = '';
  let lastInput = {connected:false,axes:[0,0,0,0],buttons:Array(17).fill(false)};
  let inputAbort = null;
  let picoAbort = null;
  const keys = new Set();

  const fixed = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '--';
  const ageText = value => Number.isFinite(value) ? `${Math.round(value)} ms` : '-- ms';
  const simulated = () => source === 'simulator' && state?.backend === 'mock';
  const visible = () => !suspended && document.visibilityState !== 'hidden';
  const neutral = () => lastInput.axes.every(value => Math.abs(value) < 0.08) && !lastInput.buttons[5];
  const nativeTracking = () => state?.pico?.source === 'xrobotoolkit' && state.pico.connected && Number.isFinite(state.pico.age_ms) && state.pico.age_ms < 250;
  const gripsReleased = () => simulated() ? !$('left-grip').checked && !$('right-grip').checked : !state?.pico?.frame || (state.pico.frame.left_grip < 0.7 && state.pico.frame.right_grip < 0.7);
  const reportError = message => {
    $('error-message').textContent = message;
    $('error-panel').hidden = !message;
  };

  async function request(path, payload, controller = new AbortController(), timeout = 900) {
    const timer = setTimeout(() => controller.abort(), timeout);
    const options = {signal:controller.signal,cache:'no-store',credentials:'same-origin'};
    if (payload !== undefined) {
      options.method = 'POST';
      options.headers = {'Content-Type':'application/json'};
      if (token) options.headers['X-Q1-Token'] = token;
      options.body = JSON.stringify(payload);
    }
    try {
      const response = await fetch(path, options);
      const data = await response.json();
      if (!response.ok) {
        if (data.state) acceptState(data.state);
        throw new Error(data.error || `HTTP ${response.status}`);
      }
      return data;
    } finally { clearTimeout(timer); }
  }

  function acceptState(next) {
    if (!next || typeof next !== 'object' || !next.robot) throw new Error('控制服务返回了无效状态');
    state = next;
    online = true;
    render();
  }

  function offline(error) {
    const wasOnline = online;
    online = false;
    if (wasOnline) resetControls();
    $('connection-status').className = 'connection offline';
    $('connection-status').innerHTML = '<i></i>服务离线';
    $('environment-banner').className = 'environment-banner offline';
    $('environment-title').textContent = '本地控制服务离线';
    $('environment-detail').textContent = '检查启动终端与依赖。输入已释放；连接恢复后需要重新使能。';
    $('environment-tag').textContent = 'OFFLINE';
    $('footer-status').textContent = `连接中断 / ${error?.message || '无法连接'}`;
    $('mode-name').textContent = '离线';
    $('mode-dot').className = 'mode-dot fault';
    document.querySelectorAll('[data-action]').forEach(button => { button.disabled = button.dataset.action !== 'stop'; });
    $('deadman-button').disabled = true;
  }

  async function pollState() {
    if (pollBusy) return;
    pollBusy = true;
    try { acceptState(await request('/api/state')); }
    catch (error) { offline(error); }
    finally { pollBusy = false; }
  }

  async function action(name) {
    if (actionBusy && name !== 'stop') return;
    if (name === 'stop' || name === 'disable') {
      controlEpoch++;
      resetControls();
      inputAbort?.abort();
      picoAbort?.abort();
    }
    const epoch = controlEpoch;
    if (name !== 'stop') actionBusy = true;
    render();
    try {
      const result = await request('/api/action', {action:name,client_id:clientId});
      if (epoch !== controlEpoch) {
        if (name !== 'stop') void action('stop');
        return;
      }
      if (result.state) acceptState(result.state);
      if (!result.ok) reportError(result.error || '操作被控制器拒绝');
      else reportError('');
    } catch (error) {
      reportError(`${name}: ${error.message}`);
      if (error.name === 'TypeError' || error.name === 'AbortError') offline(error);
    } finally {
      if (name !== 'stop') actionBusy = false;
      render();
    }
  }

  function updateSliders() {
    for (let i = 0; i < 4; i++) $(`axis-${i}-value`).textContent = fixed(Number($(`axis-${i}`).value));
    for (const id of picoSliders) $(`${id}-value`).textContent = id.startsWith('head') ? `${$(id).value}°` : fixed(Number($(id).value),3);
  }

  function setDeadman(held) {
    deadmanHeld = Boolean(held && simulated() && visible() && online);
    $('deadman-button').classList.toggle('held',deadmanHeld);
    $('deadman-button').setAttribute('aria-pressed',String(deadmanHeld));
  }

  function resetControls() {
    keys.clear();
    setDeadman(false);
    requirePadRelease = true;
    for (let i = 0; i < 4; i++) $(`axis-${i}`).value = '0';
    $('left-grip').checked = false;
    $('right-grip').checked = false;
    lastInput = {connected:false,axes:[0,0,0,0],buttons:Array(17).fill(false)};
    updateSliders();
  }

  function safetyStop() {
    void action('stop');
  }

  function readInput() {
    const empty = {axes:[0,0,0,0],buttons:Array(17).fill(false),connected:false};
    if (!visible()) return empty;
    if (source === 'simulator') {
      if (!simulated()) return null;
      const buttons = Array(17).fill(false);
      buttons[5] = deadmanHeld;
      return {axes:[0,1,2,3].map(index => Number($(`axis-${index}`).value)),buttons,connected:true};
    }
    let pad;
    try { pad = Array.from(navigator.getGamepads?.() || []).find(item => item?.connected); }
    catch (error) { reportError(`Gamepad API: ${error.message}`); }
    padName = pad?.id || '';
    if (!pad) { requirePadRelease = true; return empty; }
    const axes = [0,1,2,3].map(index => Math.max(-1,Math.min(1,Number(pad.axes[index]) || 0)));
    const buttons = Array.from({length:17},(_, index) => Boolean(pad.buttons[index]?.pressed || pad.buttons[index]?.value > 0.5));
    if (requirePadRelease && axes.every(value => Math.abs(value) < 0.08) && !buttons.some(Boolean)) requirePadRelease = false;
    padReleaseWaiting = requirePadRelease;
    return requirePadRelease ? {...empty,connected:true} : {axes,buttons,connected:true};
  }

  function picoFrame() {
    const yaw = Number($('head-yaw').value) * Math.PI / 360;
    const roll = Number($('head-roll').value) * Math.PI / 360;
    const sy = Math.sin(yaw), cy = Math.cos(yaw), sr = Math.sin(roll), cr = Math.cos(roll);
    lastTimestamp = Math.max(lastTimestamp + 1024,Math.floor((performance.timeOrigin + performance.now()) * 1e6));
    return {
      source:'simulator',timestamp_ns:lastTimestamp,
      head:[0,1.6,0,sy*sr,sy*cr,cy*sr,cy*cr],
      left:[-0.25 + Number($('left-x').value),1.2 + Number($('left-y').value),-0.35 + Number($('left-z').value),0,0,0,1],
      right:[0.25 + Number($('right-x').value),1.2 + Number($('right-y').value),-0.35 + Number($('right-z').value),0,0,0,1],
      left_grip:$('left-grip').checked ? 1 : 0,right_grip:$('right-grip').checked ? 1 : 0,valid:true
    };
  }

  async function heartbeat() {
    if (!online || !state) return;
    const input = readInput();
    if (input) lastInput = input;
    renderInput();
    if (input && !inputBusy && (input.connected || !nativeTracking())) {
      inputBusy = true;
      inputAbort = new AbortController();
      const epoch = controlEpoch;
      try {
        const result = await request('/api/input',{client_id:clientId,source,sequence:++sequence,...input},inputAbort,240);
        if (epoch === controlEpoch && result.state) acceptState(result.state);
        if (!result.ok && epoch === controlEpoch) reportError(result.error || '输入未被接受');
      } catch (error) { if (error.name !== 'AbortError') reportError(`输入: ${error.message}`); }
      finally { inputBusy = false; inputAbort = null; }
    }
    if (simulated() && visible() && online && !picoBusy) {
      picoBusy = true;
      picoAbort = new AbortController();
      const epoch = controlEpoch;
      try {
        const result = await request('/api/pico',picoFrame(),picoAbort,240);
        if (epoch === controlEpoch && result.state) acceptState(result.state);
        if (!result.ok && epoch === controlEpoch) reportError(result.error || '模拟姿态未被接受');
      } catch (error) { if (error.name !== 'AbortError') reportError(`Pico: ${error.message}`); }
      finally { picoBusy = false; picoAbort = null; }
    }
  }

  function check(id, ready, text) {
    $(`check-${id}`).classList.toggle('ready',Boolean(ready));
    $(`check-${id}-text`).classList.toggle('ready',Boolean(ready));
    $(`check-${id}-text`).textContent = text;
  }

  function renderInput() {
    const mockInput = source === 'simulator';
    $('simulator-controls').hidden = !mockInput;
    $('browser-pad-info').hidden = mockInput;
    $('input-source-badge').textContent = mockInput ? 'SIMULATED' : 'BROWSER GAMEPAD';
    $('input-source-badge').className = `badge ${mockInput ? 'badge-mock' : 'badge-outline'}`;
    $('pad-status').textContent = mockInput ? (simulated() ? '模拟手柄 · 无物理设备' : 'ROS2 禁止模拟输入') : (padName ? '物理手柄已连接' : '未检测到物理手柄');
    $('input-age').textContent = ageText(state?.input?.age_ms);
    $('gamepad-name').textContent = padName || '未检测到物理手柄';
    $('gamepad-detail').textContent = padName ? (padReleaseWaiting ? '请松开所有按键并将摇杆归中，然后显式使能。' : '标准映射：RB 按住运行，Start 显式使能。') : '连接标准映射手柄，然后按任意按钮唤醒浏览器。';
    $('pad-axes').textContent = ['LX','LY','RX','RY'].map((label,index) => `${label} ${fixed(lastInput.axes[index])}`).join(' · ');
    $('real-deadman').textContent = lastInput.buttons[5] ? 'RB HELD' : 'RB RELEASED';
    $('real-deadman').className = `badge ${lastInput.buttons[5] ? 'badge-live' : 'badge-outline'}`;
    $('deadman-button').disabled = !online || !simulated() || !visible();
    for (let i = 0; i < 4; i++) $(`axis-${i}`).disabled = !simulated();
    const mockPico = simulated();
    $('pico-simulator-controls').hidden = !mockPico;
    $('pico-source-badge').textContent = mockPico ? 'SIMULATED POSE' : 'NATIVE SOURCE';
    $('pico-source-badge').className = `badge ${mockPico ? 'badge-mock' : 'badge-outline'}`;
  }

  function render() {
    if (!state || !online) return;
    const robot = state.robot || {}, input = state.input || {}, pico = state.pico || {};
    const hardware = state.backend === 'ros2';
    const owned = !input.client_id || input.client_id === clientId;
    const fresh = Number.isFinite(input.age_ms) && input.age_ms < 250;
    const nativePico = nativeTracking();
    const inputReady = lastInput.connected && fresh && owned;
    const controlReady = inputReady || (nativePico && owned);
    $('connection-status').className = 'connection online';
    $('connection-status').innerHTML = '<i></i>服务在线';
    $('backend-badge').textContent = hardware ? 'ROS2 · HARDWARE' : 'MOCK · 仿真';
    $('backend-badge').className = `badge ${hardware ? 'badge-live' : 'badge-mock'}`;
    $('session-backend').textContent = hardware ? 'ROS2 / NATIVE MC' : 'MOCK / NO HARDWARE';
    $('environment-banner').className = `environment-banner${hardware ? ' hardware' : ''}`;
    $('environment-title').textContent = hardware ? 'ROS2 硬件模式 · 原生 MC 控制' : 'MOCK 模式 · 当前反馈与姿态均为仿真';
    $('environment-detail').textContent = hardware ? '模拟手柄与 Pico 模拟姿态已禁用。实际设备状态以下方原生反馈为准。' : '无需机器人或头显即可演练流程。模拟输入不会驱动 ROS2；真实设备尚未验证。';
    $('environment-tag').textContent = hardware ? 'LIVE BACKEND' : 'NO PHYSICAL DEVICES';
    $('mode-name').textContent = modeNames[state.mode] || state.mode || '待机';
    $('mode-dot').className = `mode-dot${state.mode === 'fault' ? ' fault' : state.enabled ? ' active' : ''}`;
    $('controller-status').textContent = state.status || robot.fault || '等待操作';
    $('enable-button').disabled = actionBusy || state.enabled || state.mode === 'fault' || !state.calibrated || !controlReady || !neutral() || !visible();
    $('calibrate-button').disabled = actionBusy || !controlReady || !neutral() || !gripsReleased() || !['idle','stand'].includes(state.mode) || !visible();
    ['stand','walk','teleop'].forEach(mode => {
      const button = $(`${mode}-button`);
      button.disabled = actionBusy || !state.enabled || !state.calibrated || !owned;
      if (mode === 'walk') button.disabled ||= !inputReady;
      if (mode === 'teleop') button.disabled ||= !pico.calibrated || !pico.connected || (hardware && !state.limits?.hardware_upper_confirmed);
      button.classList.toggle('selected',state.mode === mode);
      button.setAttribute('aria-pressed',String(state.mode === mode));
    });
    $('clear-fault-button').hidden = state.mode !== 'fault' && !robot.fault;
    $('clear-fault-button').disabled = actionBusy;
    let next = '已就绪。选择模式后，按住 RB / Space 允许运动。';
    if (!visible()) next = '窗口已失焦，控制已停止。返回后请显式使能。';
    else if (!owned) next = '另一控制端持有输入。软件停止仍可用。';
    else if (!controlReady) next = hardware && source === 'simulator' ? '请选择浏览器物理手柄；ROS2 禁止模拟输入。' : '等待新鲜手柄或原生 Pico 输入。请检查输入源与连接。';
    else if (!neutral() && !state.enabled) next = '请将摇杆归中并释放 RB，再标定或使能。';
    else if (!gripsReleased() && !state.calibrated) next = '请释放双手 Grip，然后标定中立位。';
    else if (!state.calibrated) next = '将头显、双手和摇杆归中，释放 RB，然后标定。';
    else if (!state.enabled) next = '中立位已标定。点击使能或按 Start，恢复控制。';
    else if (hardware && !state.limits?.hardware_upper_confirmed) next = '行走可用；上身硬件限位配置未确认，Pico 遥操作被锁定。';
    $('next-step').textContent = next;
    check('input',controlReady,inputReady ? `${input.source || source} · ${Math.round(input.age_ms)} ms` : nativePico ? '原生 Pico · 在线' : '等待 / 未持有');
    check('calibration',state.calibrated,state.calibrated ? '已标定' : '未标定');
    check('enable',state.enabled,state.enabled ? '已使能' : '未使能');
    check('native',robot.connected,robot.connected ? (robot.action || (hardware ? '已连接' : 'MOCK 已连接')) : '未连接');
    $('robot-connection').textContent = robot.connected ? (hardware ? 'NATIVE FEEDBACK' : 'MOCK FEEDBACK') : 'NO FEEDBACK';
    $('robot-connection').className = `badge ${robot.connected ? 'badge-live' : 'badge-outline'}`;
    $('joint-age').textContent = `反馈 ${ageText(Number.isFinite(robot.joints_age_s) ? robot.joints_age_s * 1000 : null)}`;
    const velocity = state.command?.velocity || {};
    for (const [key,unit] of [['forward','m/s'],['lateral','m/s'],['angular','rad/s']]) $(`velocity-${key}`).innerHTML = `${fixed(velocity[key] ?? 0)}<em>${unit}</em>`;
    jointRows.forEach(({row,actual,target,values},index) => {
      const a = robot.joints?.[jointNames[index]], t = state.command?.upper?.[jointNames[index]];
      const aFraction = Number.isFinite(a) ? Math.max(-0.5,Math.min(0.5,a / (2 * Math.PI))) : 0;
      actual.style.left = `${50 + Math.min(0,aFraction * 100)}%`;
      actual.style.width = `${Math.abs(aFraction * 100)}%`;
      target.style.left = `${50 + (Number.isFinite(t) ? Math.max(-50,Math.min(50,t / (2 * Math.PI) * 100)) : 0)}%`;
      actual.hidden = !Number.isFinite(a); target.hidden = !Number.isFinite(t);
      values.innerHTML = `${fixed(a)} / <span>${fixed(t)}</span>`;
      row.setAttribute('aria-label',`${jointLabels[index]}: 实际 ${fixed(a)} rad, 目标 ${fixed(t)} rad`);
    });
    $('native-pico-status').textContent = nativePico ? `原生帧在线 · ${ageText(pico.age_ms)}` : '未收到原生数据';
    $('native-pico-status').style.color = nativePico ? 'var(--cyan)' : '';
    $('pico-status').textContent = pico.connected ? `${pico.source === 'simulator' ? '模拟姿态' : pico.source || '原生姿态'} · 在线` : '未连接 / 帧过期';
    $('pico-age').innerHTML = `${Number.isFinite(pico.age_ms) ? Math.round(pico.age_ms) : '--'}<em>ms</em>`;
    $('ik-error').innerHTML = `${fixed(pico.ik_error_m,3)}<em>m</em>`;
    $('pico-calibration').textContent = pico.calibrated ? '已标定' : '未标定';
    $('pico-note').textContent = simulated() ? '以下模拟姿态仅发送到 MOCK；不会向 ROS2 发送。实际 Pico 使用官方 XRoboToolkit PC Service。' : '模拟姿态已关闭。实际 Pico 需要官方 XRoboToolkit PC Service；此处只显示服务端原生源反馈。';
    $('supermimic-reason').textContent = state.supermimic?.reason || '后续独立 HAL 后端，尚未实现。';
    const events = (state.events || []).slice(-8).reverse();
    const eventKey = JSON.stringify(events);
    if (eventKey !== lastEvents) {
      lastEvents = eventKey;
      $('event-list').replaceChildren();
      if (!events.length) {
        const item = document.createElement('li'); item.className = 'empty-event'; item.textContent = '尚无事件。先归中、标定，再使能。'; $('event-list').appendChild(item);
      }
      for (const event of events) {
        const item = document.createElement('li'), time = document.createElement('time'), text = document.createElement('span');
        const rawTime = event.time;
        time.textContent = typeof rawTime === 'number' ? (rawTime > 1e9 ? new Date(rawTime * 1000).toISOString().slice(11,19) : `${rawTime.toFixed(1)}s`) : String(rawTime || '--').slice(0,19);
        text.textContent = event.event || '';
        if (/stop|fault|error|stale|disable|lost/i.test(text.textContent)) item.className = 'event-alert';
        item.append(time,text); $('event-list').appendChild(item);
      }
    }
    $('clock').textContent = `${clockFormat.format(new Date())} 北京时间`;
    $('footer-status').textContent = `20 Hz input · 10 Hz feedback · ${hardware ? 'ROS2 / native MC' : 'MOCK / no devices'}`;
    renderInput();
  }

  document.querySelectorAll('[data-action]').forEach(button => button.addEventListener('click',() => void action(button.dataset.action)));
  $('dismiss-error').addEventListener('click',() => reportError(''));
  $('source-select').addEventListener('change',() => {
    source = $('source-select').value;
    padName = '';
    padReleaseWaiting = false;
    safetyStop();
    renderInput();
  });
  for (let i = 0; i < 4; i++) $(`axis-${i}`).addEventListener('input',updateSliders);
  for (const id of picoSliders) $(id).addEventListener('input',updateSliders);
  $('neutral-button').addEventListener('click',() => { resetControls(); renderInput(); });
  $('pico-neutral-button').addEventListener('click',() => {
    for (const id of picoSliders) $(id).value = '0';
    $('left-grip').checked = false; $('right-grip').checked = false; updateSliders();
  });
  $('deadman-button').addEventListener('pointerdown',event => {
    if (event.button !== 0) return;
    event.preventDefault();
    $('deadman-button').setPointerCapture?.(event.pointerId);
    setDeadman(true);
  });
  for (const event of ['pointerup','pointercancel','lostpointercapture']) $('deadman-button').addEventListener(event,() => setDeadman(false));
  $('deadman-button').addEventListener('keydown',event => {
    if (event.code === 'Space' || event.code === 'Enter') { event.preventDefault(); setDeadman(true); }
  });
  $('deadman-button').addEventListener('keyup',event => {
    if (event.code === 'Space' || event.code === 'Enter') { event.preventDefault(); setDeadman(false); }
  });
  $('deadman-button').addEventListener('blur',() => setDeadman(false));
  function keyboardAxes() {
    $('axis-0').value = String(Number(keys.has('KeyD')) - Number(keys.has('KeyA')));
    $('axis-1').value = String(Number(keys.has('KeyS')) - Number(keys.has('KeyW')));
    $('axis-2').value = String(Number(keys.has('KeyE')) - Number(keys.has('KeyQ')));
    updateSliders();
  }
  window.addEventListener('keydown',event => {
    if (event.code === 'Escape') { event.preventDefault(); safetyStop(); return; }
    if (!simulated() || !visible() || /^(INPUT|SELECT|TEXTAREA|BUTTON)$/.test(event.target?.tagName || '')) return;
    if (event.code === 'Space') { event.preventDefault(); setDeadman(true); }
    if (['KeyW','KeyA','KeyS','KeyD','KeyQ','KeyE'].includes(event.code)) { event.preventDefault(); keys.add(event.code); keyboardAxes(); }
  });
  window.addEventListener('keyup',event => {
    if (event.code === 'Space') setDeadman(false);
    if (keys.delete(event.code)) keyboardAxes();
  });
  window.addEventListener('blur',() => { suspended = true; safetyStop(); });
  window.addEventListener('focus',() => { suspended = document.visibilityState === 'hidden'; resetControls(); render(); });
  document.addEventListener('visibilitychange',() => {
    suspended = document.visibilityState === 'hidden' || !document.hasFocus();
    if (suspended) safetyStop();
    else { resetControls(); render(); }
  });
  window.addEventListener('gamepaddisconnected',() => { if (source === 'browser_gamepad') { padName = ''; safetyStop(); } });
  window.addEventListener('gamepadconnected',() => { requirePadRelease = true; });
  window.addEventListener('pagehide',() => {
    resetControls();
    const headers = {'Content-Type':'application/json'};
    if (token) headers['X-Q1-Token'] = token;
    void fetch('/api/action',{method:'POST',headers,body:JSON.stringify({action:'stop',client_id:clientId}),credentials:'same-origin',keepalive:true}).catch(() => {});
  });
  updateSliders();
  renderInput();
  void pollState();
  setInterval(pollState,100);
  setInterval(heartbeat,50);
})();
