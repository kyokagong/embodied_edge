(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const uuid = () => crypto.randomUUID?.() || Array.from(crypto.getRandomValues(new Uint8Array(16)), n => n.toString(16).padStart(2, '0')).join('');
  const client = uuid();
  let sequence = 0, state = null, paired = false, heartbeatBusy = false, polling = false, motions = [];
  const text = (id, value) => { $(id).textContent = value; };
  const number = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
  const describe = value => typeof value === 'string' ? value : JSON.stringify(value, null, 2);
  const modeName = {stand:'站立保持', stand_up:'吊装蹲坐到站立', dance:'舞蹈', sit:'坐姿', lie:'趴姿', getup:'起身'};
  const reasonName = {unmeasured:'等待实测反馈', lowering:'正在趴下', pose_mismatch:'实测姿态与趴姿参考不匹配', moving:'实测速度尚未稳定', settling:'正在累计稳定反馈', feedback_unavailable:'关节／IMU反馈不完整', feedback_regressed:'反馈序号或采样时间回退，需重新稳定', operator_confirmation_required:'实测趴姿稳定，仍需现场确认', missing_root_measurements:'实测趴姿稳定，完整根部测量条件不可验证'};
  async function post(action, extra = {}) {
    const controller = new AbortController(), timeout = setTimeout(() => controller.abort(), 1500);
    const payload = {action, client_id:client, sequence:++sequence, request_id:uuid(), ...extra};
    try {
      const response = await fetch('/api/action', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload), signal:controller.signal});
      const data = await response.json();
      if (!response.ok) throw Error(data.error || `请求失败 (${response.status})`);
      return data;
    } finally { clearTimeout(timeout); }
  }
  async function act(action, extra = {}) {
    try { const result = await post(action, extra); text('message', result.message || '请求已接受'); void poll(); return true; }
    catch (error) { text('message', error.name === 'AbortError' ? '请求超时，请检查服务状态后再操作。' : error.message); return false; }
  }
  $('pair').onclick = async () => { paired = await act('pair'); renderControls(); };
  $('hold').onclick = () => void act('hold');
  for (const button of document.querySelectorAll('[data-action]')) button.onclick = () => void act(button.dataset.action);
  $('play').onclick = () => void act('play_motion', {clip_id:$('motion').value});
  $('motion').onchange = renderMotion;
  function renderControls() {
    const online = state !== null, owns = paired && online;
    for (const button of document.querySelectorAll('[data-action]')) {
      const action = button.dataset.action;
      button.disabled = !online || (!owns && !['release'].includes(action));
      if (['sit','lie'].includes(action)) button.disabled ||= !state?.enabled || state?.task?.mode === 'stand_up';
      if (action === 'reset') button.disabled ||= !!state?.enabled || (state?.bridge?.released !== true && !state?.bridge?.fault);
    }
    $('play').disabled = !owns || !$('motion').value || !state?.enabled || state?.task?.mode === 'stand_up';

    text('operator', paired ? (document.hidden ? '页面处于后台，操作租约暂停' : '操作心跳已开启') : '未取得操作权');
  }
  function renderMotion() {
    const motion = motions.find(item => item.id === $('motion').value);
    if (!motion) { text('motion-note', '模型包未提供舞蹈参考'); return; }
    const metadata = motion.metadata || motion;
    const duration = metadata.duration_seconds ?? metadata.duration_s ?? (metadata.frames_50hz ? (metadata.frames_50hz - 1) / 50 : null);
    const qualification = metadata.qualification || metadata.status || (metadata.learned_policy_qualified === false ? '学习策略尚未通过物理验收' : '以模型包资格记录为准');
    text('motion-note', `${duration === null ? '时长未知' : `${number(duration, 1)} 秒`} · ${qualification}`);
    renderControls();
  }
  function renderMotions(items) {
    const next = (Array.isArray(items) ? items : []).map(item => typeof item === 'string' ? {id:item} : {...item, id:item.id || item.name || item.clip}).filter(item => item.id && ((item.metadata || item).group === 'lafan1' || /^dance[12]_subject\d+$/.test(item.id)));
    if (JSON.stringify(next) === JSON.stringify(motions)) return;
    const selected = $('motion').value; motions = next;
    $('motion').replaceChildren(...motions.map(motion => {const option = document.createElement('option'); option.value = motion.id; option.textContent = motion.id; return option;}));
    if (motions.some(item => item.id === selected)) $('motion').value = selected;
    renderMotion();
  }
  function render(next) {
    state = next;
    const bridge = state.bridge || {}, sensors = bridge.sensors || {}, task = state.task || {}, model = state.model || {};
    text('connection', `${state.controller || 'q1_policy'} / 本体 ROS2`);
    text('phase', state.phase || bridge.mode || '未知'); text('permission', state.enabled ? '运动许可开启' : '运动许可关闭');
    text('actuators', bridge.released === true ? '执行器已释放' : bridge.mode ? `桥接状态：${bridge.mode}` : '等待执行器状态');
    text('iteration', model.iteration == null ? '—' : String(model.iteration)); text('model', `模型 SHA256：${model.sha256 || '未知'}`);
    text('qualification', state.qualification == null ? '模型包未提供资格摘要；请检查部署记录。' : describe(state.qualification));
    text('blockers', [...new Set([...(Array.isArray(state.blockers) ? state.blockers : []), ...(Array.isArray(bridge.blockers) ? bridge.blockers : [])])].join('\n'));
    text('fault', [state.fault, bridge.fault].filter(Boolean).map(describe).join('\n'));
    text('inference', `${number(state.inference_ms)} ms`); text('tx', `${number(bridge.tx_hz, 1)} Hz${bridge.released ? '（低阻尼）' : ''}`);
    text('age', sensors.complete ? `${number(state.sensor_age_ms, 1)} ms` : '未收到完整反馈');
    text('inputs', state.input_count ?? state.policy_steps ?? 0); text('models', state.model_count ?? 0);
    text('gap', bridge.released ? '—' : `${number(bridge.tx_max_gap_ms)} ms`);
    renderMotions(state.motions);
    $('progress').max = Math.max(1, task.end || 0); $('progress').value = task.frame || 0;
    text('task', task.clip ? `${modeName[task.mode] || task.mode} · ${task.clip} · ${number((task.frame || 0) / 50, 2)} 秒${task.returning ? ' · 返回段' : ''}` : '站立零指令策略');
    text('task-boundary', task.clip ? `当前参考边界 ${(task.end / 50).toFixed(2)} 秒 · epoch ${task.epoch ?? '—'}` : `epoch ${task.epoch ?? '—'}`);
    const entry = task.stand_entry;
    text('readiness', entry ? `站立参考接入 ${(entry.reference_start_frame / 50).toFixed(2)} 秒处；最大关节差 ${number(entry.joint_max_rad, 3)} rad，重力方向差 ${number(entry.gravity_error_rad, 3)} rad。双脚着地与吊杆支撑为现场条件，非传感器测量。` : '站立从失能、双脚着地且吊杆支撑的蹲坐姿态开始；匹配失败会拒绝接入。趴姿起身暂不开放。');
    const names = state.joint_names || Array.from({length:22}, (_, index) => `joint ${index + 1}`);
    $('joints').replaceChildren(...names.map((name, index) => {
      const row = document.createElement('div'); row.className = 'joint-row';
      for (const value of [name, sensors.complete ? number(sensors.q?.[index], 3) : '—', number(state.target?.[index], 3), sensors.complete ? number(sensors.qd?.[index], 3) : '—', sensors.complete ? `${number(sensors.temperature?.[index], 0)}°` : '—']) {const span = document.createElement('span'); span.textContent = value; row.append(span);} return row;
    }));
    text('identity', describe({model, input_shapes:state.input_shapes, controller:state.controller}));
    $('events').replaceChildren(...(Array.isArray(state.events) ? state.events : []).map(event => {const li = document.createElement('li'); li.textContent = typeof event === 'string' ? event : event.event || describe(event); return li;}));
    renderControls();
  }
  async function poll() {
    if (polling) return; polling = true;
    try { const response = await fetch('/api/state', {signal:AbortSignal.timeout(1200)}); if (!response.ok) throw Error(response.status); render(await response.json()); }
    catch (_) { state = null; paired = false; text('connection', '控制服务离线'); renderControls(); }
    finally { polling = false; }
  }
  async function heartbeat() {
    if (!paired || document.hidden || heartbeatBusy) return;
    heartbeatBusy = true;
    try { await post('heartbeat'); }
    catch (error) { paired = false; text('message', `操作心跳断开：${error.message}`); renderControls(); }
    finally { heartbeatBusy = false; }
  }
  document.addEventListener('visibilitychange', renderControls);
  window.addEventListener('keydown', event => {if (event.key === 'Escape') void act('hold');});
  renderControls(); void poll(); setInterval(() => void poll(), 200); setInterval(() => void heartbeat(), 100);
})();
