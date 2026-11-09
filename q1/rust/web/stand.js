(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const uid = () => crypto.randomUUID?.() || Array.from(crypto.getRandomValues(new Uint8Array(16)), v => v.toString(16).padStart(2, '0')).join('');
  const client = uid();
  let paired = false, sequence = 0, heartbeatBusy = false, actionBusy = false, state = null;
  const phases = {CONNECTING:'连接本体', READY:'就绪，等待站立指令', TAKEOVER:'接管当前实测姿态', INITIALIZING:'平滑起身', POLICY_HANDOVER:'模型接管过渡', POLICY_STAND:'模型站立保持运行中', STANDING:'站姿传感器判据已满足', DANCING:'舞蹈播放中', RETURNING_STAND:'正在恢复站立姿态', WAITING_MODEL:'等待下一帧有效模型输出', FAULT:'保护保持，等待新指令', SAFE_HOLD:'保护保持，可开始站立', HOLDING:'正在进入保护保持', RESETTING:'正在复位', RELEASING:'正在解除位置接管', RELEASED:'位置接管已解除，零刚度低阻尼'};
  const n = (x, d = 3) => Number.isFinite(x) ? x.toFixed(d) : '—';
  const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
  async function post(action, extra = {}) {
    const controller = new AbortController(), timer = setTimeout(() => controller.abort(), 2000);
    try {
      const response = await fetch('/api/action', {method:'POST', headers:{'Content-Type':'application/json', 'X-Q1-Client':'console'}, body:JSON.stringify({...extra, action, client_id:client, sequence:sequence++, request_id:uid()}), signal:controller.signal});
      const value = await response.json();
      if (!response.ok) {const error = Error(value.error || response.status); error.status = response.status; throw error;}
      return value;
    } finally {clearTimeout(timer);}
  }
  async function readState() {
    const response = await fetch('/api/state', {signal:AbortSignal.timeout(2000), cache:'no-store'});
    if (!response.ok) throw Error('本体状态读取失败');
    state = await response.json();
    return state;
  }
  function buttons() {
    const ready = state && ['READY','SAFE_HOLD','RELEASED','FAULT','HOLDING'].includes(state.phase);
    $('stand').disabled = actionBusy || !ready || state.enabled;
    $('stand').textContent = actionBusy ? '正在启动站立…' : state?.enabled ? '站立运行中' : '一键站立';

    $('hold').disabled = !state?.bridge.session;
    $('release').disabled = !state || state.bridge.released;
    $('reset').disabled = !state || state.enabled || !state.bridge.fault;
    $('play-dance').disabled = actionBusy || !state || !state.enabled || !['POLICY_STAND','STANDING'].includes(state.phase) || state.dance_playing || !state.policy_ready;
    $('cancel-dance').disabled = !state?.dance_playing;
    $('dance-select').disabled = !!state?.dance_playing || !!state?.returning_to_stand;
  }
  async function connect() {
    await post('pair'); paired = true;
    $('auth').textContent = '控制器已连接；站立由本体自动执行，无需按住按钮';
  }
  $('stand').onclick = async () => {
    if (actionBusy) return;
    actionBusy = true; buttons();
    try {
      $('message').textContent = '正在连接控制器并读取当前实测姿态…';
      await connect();
      let current = await readState();
      if (current.enabled) {$('message').textContent = '站立控制已经在运行。'; return;}
      if (current.bridge.fault) {
        $('message').textContent = '正在确认反馈并复位上一次保护状态…';
        await post('reset');
        const until = performance.now() + 3000;
        do {await sleep(100); current = await readState();} while (current.bridge.fault && performance.now() < until);
        if (current.bridge.fault) throw Error(`当前保护尚未解除：${current.bridge.fault}`);
      }
      await connect();
      await post('stand');
      $('message').textContent = '一键站立已启动：接管当前姿态 → 平滑起身 → 模型保持。无需再次点击或持续按住。';
      await readState();
    } catch (e) {$('message').textContent = e.message;}
    finally {actionBusy = false; buttons();}
  };
  for (const action of ['hold','release','reset']) $(action).onclick = async () => {
    try {
      if (action === 'reset') await connect();
      const reply = await post(action);
      $('message').textContent = `指令已接受：${phases[reply.phase] || reply.phase}`;
    } catch (e) {$('message').textContent = e.message;}
  };

  $('play-dance').onclick = async () => {
    actionBusy = true; buttons();
    try {await connect(); await post('play_dance', {clip:$('dance-select').value}); $('message').textContent='舞蹈已开始；取消或播放结束会切回站立条件，模型持续控制。';}
    catch(e){$('message').textContent=e.message;} finally {actionBusy=false;buttons();}
  };
  $('cancel-dance').onclick = async () => {
    try {await connect();await post('cancel_dance');$('message').textContent='已切回零速度站立条件，由同一模型恢复站立。';} catch(e){$('message').textContent=e.message;}
  };
  async function heartbeat() {
    if (!paired || heartbeatBusy || document.hidden) return;
    heartbeatBusy = true;
    try {await post('heartbeat');}
    catch (e) {if ([401,403,409].includes(e.status)) paired = false; $('auth').textContent = `控制器心跳：${e.message}`;}
    finally {heartbeatBusy = false;}
  }
  async function poll() {
    try {
      await readState(); const b = state.bridge, s = b.sensors;
      $('phase').textContent = phases[state.phase] || state.phase;
      const elapsed = n(state.stand_elapsed_s, 0);
      $('summary').textContent = state.enabled ? `真实22关节控制进行中 · 已运行${elapsed}秒${state.standing_verified ? ' · 站姿稳定判据已满足' : ''}` : `本体保护：${b.mode} · 可从当前实测姿态开始站立`;
      const select=$('dance-select');
      if(!select.options.length) for(const clip of state.dances){const o=document.createElement('option');o.value=clip.id;o.textContent=`${clip.id.startsWith('dance2')?'舞蹈2':'舞蹈1'} · ${clip.id.split('_')[1]} · ${n(clip.duration_s,1)}秒`;select.append(o);}
      if(!select.dataset.initialized && select.options.length){select.value='dance2_subject1';select.dataset.initialized='true';}
      $('dance-progress').textContent=state.dance_playing?`${state.dance} · 已播放${n(state.dance_elapsed_s,1)}秒`:state.returning_to_stand?'恢复站立中…':'先站立，再选择舞蹈并播放';
      buttons();
      const active = b.saturation.filter(v => v !== 0).length;
      $('limits').textContent = state.protection_mode === 'continuous_saturation' ? `连续限幅 · 模型/机械裁剪${state.model_saturation.filter(Boolean).length}轴 / 执行层${active}轴 · 累计${b.saturated_frames}帧 · 倾角纠偏比例${n(b.tilt_scale,2)} · 策略目标速度上限${n(Math.max(...state.policy_velocity_limits_rad_s),2)} rad/s` : '动作阈值中止模式';
      $('fault').textContent = [state.fault,state.guard_fault,...b.blockers].filter(Boolean).join('\n');
      if (state.flight_log) {const f = state.flight_log; $('flight').textContent = `运行日志：策略${f.written}条 / HAL ${b.log_written}条 · 丢失${f.dropped+b.log_dropped}条${f.failed || b.log_failed ? ' · 写入失败' : ''}`;}
      $('input').textContent = `${state.input_count} × 1376`;
      $('model').textContent = `${state.model_count} / ${n(state.inference_ms,1)} ms`;
      $('tx').textContent = String(b.tx_count);
      $('hz').textContent = `${n(b.tx_hz,1)} Hz（配置${b.publish_hz}）`;
      $('tilt').textContent = `${n(state.tilt_rad*180/Math.PI,1)}°`;
      $('age').textContent = `${n(state.feedback_age_ms,1)} ms`;
      $('joints').replaceChildren(...state.joint_names.map((name,i) => {
        const row = document.createElement('div'); row.className = 'joint-row';
        for (const value of [name,s.complete?n(s.q[i]):'—',b.tx_count?n(b.commanded_q[i]):'—',b.tx_count?n(b.estimated_effort[i],2):'—',n(state.effort_limits_nm[i],1)]) {const span = document.createElement('span'); span.textContent = value; row.append(span);}
        return row;
      }));
      $('events').replaceChildren(...state.events.map(value => {const li = document.createElement('li'); li.textContent = value; return li;}));
    } catch (e) {$('phase').textContent = '本体控制服务离线'; $('stand').disabled = true;}
  }
  setInterval(() => void heartbeat(), 100);
  setInterval(() => void poll(), 200);
  buttons(); void poll();
})();
