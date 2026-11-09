const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const root = require('node:path').resolve(__dirname, '../q1_edge/web') + '/';
assert.ok(fs.existsSync(root + 'app.js'), 'dashboard behavior must exist');
const html = fs.readFileSync(root + 'index.html', 'utf8');
assert.ok(html.includes('name="api-token" content="__Q1_API_TOKEN__"'), 'server token placeholder exists');
for (const id of ['enable-button', 'calibrate-button', 'stop-button', 'source-select', 'deadman-button', 'joint-list', 'native-pico-status']) assert.ok(html.includes('id="' + id + '"'), id + ' is available to operators');
class Element {
  constructor(id, value = '') { this.id = id; this.value = value; this.listeners = {}; this.dataset = {}; this.classList = {toggle(){}, add(){}, remove(){}}; this.style = {setProperty(){}}; this.checked = false; this.disabled = false; this.hidden = false; }
  addEventListener(name, fn) { (this.listeners[name] ||= []).push(fn); }
  emit(name, data = {}) { for (const fn of this.listeners[name] || []) fn({type:name, target:this, currentTarget:this, preventDefault(){}, ...data}); }
  setAttribute(name, value) { this[name] = value; }
  appendChild() {} append() {} replaceChildren() {} querySelector() { return new Element('nested'); } querySelectorAll() { return []; }
  setPointerCapture() {} contains() { return false; }
}
const elements = new Map();
for (const match of html.matchAll(/<[^>]+\bid="([^"]+)"[^>]*>/g)) {
  const value = match[0].match(/\bvalue="([^"]*)"/);
  const el = new Element(match[1], value ? value[1] : '');
  if (match[0].includes('data-action=')) el.dataset.action = match[0].match(/data-action="([^"]+)"/)[1];
  elements.set(el.id, el);
}
elements.get('source-select').value = 'simulator';
const actions = [...elements.values()].filter(el => el.dataset.action);
const document = new Element('document');
document.visibilityState = 'visible'; document.hasFocus = () => true;
document.getElementById = id => { if (!elements.has(id)) elements.set(id, new Element(id)); return elements.get(id); };
document.querySelector = () => ({content:'test-token'});
document.querySelectorAll = selector => selector === '[data-action]' ? actions : [];
document.createElement = () => new Element('created');
const window = new Element('window');
const intervals = []; const requests = []; let pad = null; let holdInput = false; let heldResolve; let holdState=false; let heldStateResolve; let fakeNow=100; let timerId=0; const timers=new Map();
const state = {backend:'mock', enabled:false, calibrated:false, mode:'idle', status:'Ready', robot:{connected:true,joints:{},velocity:{}}, input:{source:'simulator',age_ms:0,deadman:false}, pico:{source:'simulator',connected:true,calibrated:false,age_ms:0}, command:{velocity:{},upper:{}}, events:[], supermimic:{available:false,reason:'Pending'}};
const context = {document,window,navigator:{getGamepads:() => [pad]},performance:{timeOrigin:1761807600000,now:()=>fakeNow},crypto:{randomUUID:()=>'ui-tab-test'},setInterval:(fn,ms)=>{intervals.push({fn,ms});return intervals.length;},clearInterval(){},setTimeout:(fn,ms)=>{timers.set(++timerId,{fn,ms});return timerId;},clearTimeout:id=>timers.delete(id),AbortController,console,Math,Date,Array,Number,Set,Map,JSON,fetch:async(url,options={}) => {
  const snapshot=structuredClone(state);
  requests.push({url,options,body:options.body && JSON.parse(options.body)});
  if (url === '/api/state' && holdState) await new Promise(resolve=>{heldStateResolve=resolve;});
  if (url === '/api/input' && holdInput) await new Promise((resolve,reject) => {heldResolve=resolve;options.signal.addEventListener('abort',()=>reject(Object.assign(new Error('Aborted'),{name:'AbortError'})),{once:true});});
  return {ok:true,status:200,json:async()=>url==='/api/state'?snapshot:(url==='/api/input' && options.headers['X-Q1-Compact']==='1' ? {ok:true,ack:{sequence:JSON.parse(options.body).sequence}} : {ok:true,state:structuredClone(state)})};
}};
vm.runInNewContext(fs.readFileSync(root+'app.js','utf8'),context);
const flush = async() => {for(let i=0;i<10;i++) await Promise.resolve();};
(async()=>{
 await flush();
 const beat = intervals.find(x=>x.ms===50); assert.ok(beat,'20 Hz heartbeat registered');
 await beat.fn(); await flush();
 const input=requests.filter(r=>r.url==='/api/input').at(-1).body;
 assert.equal(requests.filter(r=>r.url==='/api/input').at(-1).options.headers['X-Q1-Token'],'test-token'); assert.equal(input.source,'simulator'); assert.equal(input.client_id,'ui-tab-test'); assert.equal(input.buttons[5],false); assert.deepEqual(Array.from(input.axes),[0,0,0,0]);
 const pico=requests.filter(r=>r.url==='/api/pico').at(-1).body;
 assert.deepEqual(Array.from(pico.head),[0,1.6,0,0,0,0,1]); assert.equal(pico.source,'simulator'); assert.ok(Number.isInteger(pico.timestamp_ns) && pico.timestamp_ns > 1e18, "simulated Pico timestamps survive page reloads");
 holdInput=true; beat.fn(); await flush(); const count=requests.filter(r=>r.url==='/api/input').length; beat.fn(); beat.fn(); await flush(); assert.equal(requests.filter(r=>r.url==='/api/input').length,count,'no overlapping or queued input'); fakeNow+=120; holdInput=false; heldResolve(); await flush();
 const wake=[...timers].find(([,t])=>t.ms===0);assert.ok(wake,'delayed completion wakes newest sample immediately');
 elements.get('axis-2').value='-.6';timers.delete(wake[0]);wake[1].fn();await flush();
 const newest=requests.filter(r=>r.url==='/api/input').at(-1);assert.equal(newest.body.axes[2],-.6);assert.equal(newest.options.headers['X-Q1-Compact'],'1');
 elements.get('axis-2').value='0';
 holdInput=true; const timeoutFlight=beat.fn();await flush();fakeNow+=240;
 const timeoutTimer=[...timers].find(([,t])=>t.ms===240);assert.ok(timeoutTimer);timers.delete(timeoutTimer[0]);timeoutTimer[1].fn();await timeoutFlight;await flush();holdInput=false;
 await beat.fn();await flush();assert.equal(requests.filter(r=>r.url==='/api/input').at(-1).body.telemetry.timeouts,1,'request timeout becomes telemetry');
 const pendingWake=[...timers].filter(([,t])=>t.ms===0);
 holdInput=true;const stoppedFlight=beat.fn();await flush();window.emit('blur');await stoppedFlight;await flush();holdInput=false;const stoppedCount=requests.filter(r=>r.url==='/api/input').length;
 for(const [id,t] of pendingWake){timers.delete(id);t.fn();}await flush();
 assert.equal(requests.filter(r=>r.url==='/api/input').length,stoppedCount,'old completion cannot send after stop');window.emit('focus');await flush();
 elements.get('deadman-button').emit('pointerdown',{pointerId:1,button:0}); await beat.fn();await flush(); assert.equal(requests.filter(r=>r.url==='/api/input').at(-1).body.buttons[5],true);
 window.emit('blur'); await flush(); assert.ok(requests.some(r=>r.url==='/api/action' && r.body.action==='stop'),'blur sends stop');
 await beat.fn();await flush();assert.equal(requests.filter(r=>r.url==='/api/input').at(-1).body.buttons[5],false,'blur releases deadman');
 window.emit('focus'); await flush();
 elements.get('source-select').value='browser_gamepad';elements.get('source-select').emit('change');await flush();await beat.fn();await flush();
 assert.equal(requests.filter(r=>r.url==='/api/input').at(-1).body.connected,false,'absent pad is not connected');
 const picoCount=requests.filter(r=>r.url==='/api/pico').length;await beat.fn();await flush();assert.equal(requests.filter(r=>r.url==='/api/pico').length,picoCount,'no fake Pico frames outside simulator');
 pad={id:'Real test pad',index:0,connected:true,axes:[0,0,0,0],buttons:Array.from({length:17},()=>({pressed:false,value:0}))}; await beat.fn();await flush();pad.buttons[5].pressed=true;await beat.fn();await flush();const gamepad=requests.filter(r=>r.url==='/api/input').at(-1).body;assert.equal(gamepad.source,'browser_gamepad');assert.equal(gamepad.buttons[5],true);
 elements.get('stop-button').emit('click');await flush(); const action=requests.filter(r=>r.url==='/api/action').at(-1).body;assert.equal(action.action,'stop');assert.equal(action.client_id,'ui-tab-test');
 document.visibilityState='hidden';document.emit('visibilitychange');await flush();assert.equal(requests.filter(r=>r.url==='/api/action').at(-1).body.action,'stop');
 document.visibilityState='visible';window.emit('focus');pad=null;state.backend='ros2';state.calibrated=true;state.pico={source:'xrobotoolkit',connected:true,calibrated:true,age_ms:0,frame:{left_grip:0,right_grip:0}}; await intervals.find(x=>x.ms===200).fn();await flush();await beat.fn();await flush(); assert.equal(elements.get('enable-button').disabled,false,'native Pico standalone can enable without browser pad');
 const nativeInputCount=requests.filter(r=>r.url==='/api/input').length;await beat.fn();await flush();assert.equal(requests.filter(r=>r.url==='/api/input').length,nativeInputCount,'absent browser pad does not invalidate native Pico calibration');
 state.backend='mock';elements.get('source-select').value='simulator';elements.get('source-select').emit('change');await flush();await beat.fn();await flush();elements.get('left-grip').checked=true;await intervals.find(x=>x.ms===200).fn();await flush();assert.equal(elements.get('calibrate-button').disabled,true,'grips must be released before calibration');
 state.backend='ros2';state.pico={connected:false};
 await intervals.find(x=>x.ms===200).fn();await flush();
 assert.equal(elements.get('source-select').value,'browser_gamepad','hardware automatically chooses physical input');
 assert.equal(elements.get('simulator-option').disabled,true,'hardware disables simulator choice');
 assert.equal(elements.get('simulator-controls').hidden,true);
 assert.equal(elements.get('input-source-badge').textContent,'BROWSER GAMEPAD');
 pad={id:'MACHENIKE G3V2',connected:true,axes:[0,0,0,0],buttons:Array.from({length:17},()=>({pressed:false,value:0}))};
 await beat.fn();await flush(); pad.axes[1]=-.5;pad.buttons[5].pressed=true;
 await beat.fn();await flush();
 const realInput=requests.filter(r=>r.url==='/api/input').at(-1).body;
 assert.equal(realInput.source,'browser_gamepad');assert.equal(realInput.axes[1],-.5);assert.equal(realInput.buttons[5],true);
 assert.equal(elements.get('gamepad-name').textContent,'MACHENIKE G3V2');
 assert.equal(realInput.telemetry.timeouts,1,'intentional stop abort does not count as timeout');
 state.status='before-stop';holdState=true;const oldPoll=intervals.find(x=>x.ms===200).fn();await flush();
 state.status='after-stop';elements.get('stop-button').emit('click');await flush();
 holdState=false;heldStateResolve();await oldPoll;await flush();
 assert.equal(elements.get('controller-status').textContent,'after-stop','pre-stop state response cannot restore old UI');
 state.controls={rb_toggle_enabled:true,rb_hold_s:1};state.input.motion_latched=true;state.enabled=true;state.mode='stand';pad.axes=[0,0,0,0];pad.buttons[5].pressed=false;
 await beat.fn();await intervals.find(x=>x.ms===200).fn();await flush();
 assert.equal(elements.get('real-deadman').textContent,'使能已锁定');assert.equal(elements.get('calibrate-button').disabled,true);
 assert.ok(elements.get('gamepad-detail').textContent.includes('长按1秒'));assert.ok(elements.get('gamepad-detail').textContent.includes('左右转向'));
 state.backend='mock';state.capabilities={pico:false};state.pico={source:'simulator',connected:true,calibrated:true,age_ms:0};
 elements.get('source-select').value='simulator';elements.get('source-select').emit('change');await flush();
 await intervals.find(x=>x.ms===200).fn();await flush();
 const rustPicoCount=requests.filter(r=>r.url==='/api/pico').length;
 await beat.fn();await flush();
 assert.equal(requests.filter(r=>r.url==='/api/pico').length,rustPicoCount,'hand-only Rust capability disables simulated Pico requests');
 assert.equal(elements.get('pico-simulator-controls').hidden,true,'hand-only Rust hides Pico simulation controls');
 assert.equal(elements.get('teleop-button').disabled,true,'hand-only Rust disables upper mode even with cached Pico state');
 assert.ok(elements.get('pico-note').textContent.includes('Python'),'hand-only Rust explains how to access Pico');
 delete state.capabilities;await intervals.find(x=>x.ms===200).fn();await flush();await beat.fn();await flush();
 assert.ok(requests.filter(r=>r.url==='/api/pico').length>rustPicoCount,'legacy Python without capability field retains simulated Pico');
 console.log('PASS: compact heartbeat, latest sample wakeup, no overlapping input, timeout telemetry, stop epoch fences, 5Hz state, latch/yaw UI, legacy mock/native Pico separation, Rust hand-only capability');
})().catch(error=>{console.error(error);process.exitCode=1;});
