(() => {
  'use strict';
  const uuid=()=>crypto.randomUUID?.()||Array.from(crypto.getRandomValues(new Uint8Array(16)),v=>v.toString(16).padStart(2,'0')).join('');
  const $=id=>document.getElementById(id), client=uuid();
  let state=null, clips=[], sequence=0, busy=false, visible=true, hadPad=false, selectedSent=null;
  const text=(id,value)=>{$(id).textContent=value;};
  async function post(path,payload){
    const abort=new AbortController(), timer=setTimeout(()=>abort.abort(),220);
    try {const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json','X-Q1-Operator':$('key').value},body:JSON.stringify({client_id:client,request_id:uuid(),...payload}),signal:abort.signal});const d=await r.json();if(!r.ok)throw new Error(d.error||r.status);return d;}finally{clearTimeout(timer);}
  }
  const act=(action,extra={})=>post('/api/action',{action,...extra}).catch(e=>text('message',e.message));
  const stop=()=>{void act('stop');};
  for(const b of document.querySelectorAll('[data-action]'))b.addEventListener('click',()=>act(b.dataset.action));
  $('stop').onclick=stop;
  $('add').onclick=()=>post('/api/dance/queue',{clip_id:$('dance').value}).catch(e=>text('message',e.message));
  $('play').onclick=()=>post('/api/dance/start',{clip_id:$('dance').value}).catch(e=>text('message',e.message));
  $('cancel').onclick=()=>post('/api/dance/cancel',{}).catch(e=>text('message',e.message));
  $('clear').onclick=()=>post('/api/dance/queue',{operation:'clear'}).catch(e=>text('message',e.message));
  $('dance').onchange=()=>{const c=clips.find(c=>c.id===$('dance').value);text('dance-note',`${c?.duration_s??0}秒 · ${c?.observation_inference_allowed?'观测推理可运行':(c?.hardware_allowed?'已允许':'尚未通过真机逐段验收')}`);selectedSent=null;};
  text('secure',window.isSecureContext?'浏览器安全上下文已就绪':'通过 HTTPS 或 SSH localhost 转发访问，以使用浏览器手柄。');
  const number=(x,n=3)=>Number.isFinite(x)?x.toFixed(n):'—';
  async function poll(){try{const r=await fetch('/api/state',{signal:AbortSignal.timeout(800)});if(!r.ok)throw Error(r.status);state=await r.json();const s=state.bridge.sensors;
    text('connection',`${state.deployment?.inference_location||'Q1'} / 真实 ROS2`);text('mode',state.shadow_only?(state.preview_running?(state.feedback_wait?'等待真实反馈':'本体模型任务运行中'):'本体实时监视'):state.execution_state||'READ_ONLY');text('permission',state.shadow_only?(state.preview_running?'观测推理任务运行中 · 电机接管关闭':'模型任务空闲 · 电机接管关闭'):(state.enabled?'运动许可开启':'运动许可关闭'));text('deployment',state.deployment?`推理位置：${state.deployment.inference_location} · ${state.deployment.architecture} · ONNX Runtime`:'');text('actuators',state.actuator_active?(state.execution_scope==='left_elbow_commissioning'?(state.bridge.mode==='FAULT'?'左肘低增益故障保持输出中':'左肘调试输出中'):(state.bridge.mode==='FAULT'?'本体故障保持输出中':'本体执行器控制输出中')):'执行器写入关闭');text('blockers',(state.shadow_only&&state.blockers.length?'电机接管条件\n':'')+[...new Set(state.blockers)].join('\n'));text('fault',[state.feedback_wait?`采集等待：${state.feedback_wait}`:'',state.model_fault?`模型：${state.model_fault}`:'',state.guard_fault?`本体保护：${state.guard_fault}`:''].filter(Boolean).join('\n')||state.fault||'');$('stop').textContent=state.shadow_only?'停止模型任务':'受控停止';for(const b of document.querySelectorAll('[data-action="enable"]'))b.textContent=state.shadow_only?'启动本体推理':'启用站立';$('play').textContent=state.shadow_only?'运行舞蹈推理':'播放选中';
    text('inference',`${number(state.inference_ms,2)} ms`);text('tx',state.actuator_active?`${number(state.bridge.tx_hz,1)} Hz`:'未发送');text('steps',state.policy_steps);text('gap',state.actuator_active?`${number(state.bridge.tx_max_gap_ms,2)} ms`:'—');
    text('age',s.complete?`${number(state.sensor_age_ms,1)} ms`:'未收到完整反馈');
    const rows=state.joint_names.map((name,i)=>{const row=document.createElement('div');row.className='joint-row';for(const value of [name,s.complete?number(s.q[i]):'—',state.policy_steps?number(state.target[i]):'—',s.complete?`${s.temperature[i]}°`:'—']){const span=document.createElement('span');span.textContent=value;row.append(span);}return row;});$('joints').replaceChildren(...rows);
    const c=clips.find(c=>c.id===state.dance.id);$('progress').max=c?.frames_50hz||1;$('progress').value=state.dance.frame;text('dance-state',state.dance.id?`${state.dance.id} · ${(state.dance.frame/50).toFixed(1)} s`:'未播放');
    for(const [id,values] of [['queue',state.dance.queue],['events',state.events.map(e=>e.event)]]){$(id).replaceChildren(...values.map((v,index)=>{const li=document.createElement('li');li.textContent=v;if(id==='queue'){const remove=document.createElement('button');remove.textContent='移除';remove.onclick=()=>post('/api/dance/queue',{operation:'remove',index}).catch(e=>text('message',e.message));li.append(remove);}return li;}));}
    for(const b of document.querySelectorAll('[data-action="enable"],[data-action="walk"]'))b.disabled=!state.shadow_only&&state.blockers.length>0;
  }catch(e){text('connection','控制服务离线');state=null;}}
  async function heartbeat(){if(busy||!visible||!$('key').value)return;if(!window.isSecureContext||typeof navigator.getGamepads!=='function'){text('pad','请通过 HTTPS 或 localhost 使用物理手柄');if(hadPad)stop();hadPad=false;return;}let pad;try{pad=Array.from(navigator.getGamepads()).find(p=>p?.connected&&p.mapping==='standard');}catch(e){text('pad',`手柄接口不可用：${e.message}`);if(hadPad)stop();hadPad=false;return;}
    if(!pad){text('pad','未检测到标准映射物理手柄');if(hadPad)stop();hadPad=false;return;}hadPad=true;text('pad',pad.id);const axes=Array.from({length:4},(_,i)=>pad.axes[i]||0),buttons=Array.from({length:17},(_,i)=>!!pad.buttons[i]?.pressed);text('axes',axes.map(x=>number(x,2)).join(' / '));busy=true;
    try{await post('/api/input',{source:'browser_gamepad',sequence:sequence++,axes,buttons});if(selectedSent!==$('dance').value){await post('/api/action',{action:'select',clip_id:$('dance').value});selectedSent=$('dance').value;}}catch(e){text('message',e.message);}finally{busy=false;}
  }
  document.addEventListener('visibilitychange',()=>{visible=!document.hidden;if(!visible)stop();});window.addEventListener('blur',()=>{visible=false;stop();});window.addEventListener('focus',()=>{visible=true;});window.addEventListener('gamepaddisconnected',stop);window.addEventListener('keydown',e=>{if(e.key==='Escape')stop();});
  fetch('/api/dances').then(r=>r.json()).then(d=>{clips=d;$('dance').replaceChildren(...d.map(c=>{const o=document.createElement('option');o.value=c.id;o.textContent=c.id;return o;}));$('dance').dispatchEvent(new Event('change'));}).catch(e=>text('message',e.message));
  setInterval(()=>void poll(),200);setInterval(()=>void heartbeat(),50);void poll();
})();
