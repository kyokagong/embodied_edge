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
const intervals = []; const requests = []; let pad = null; let holdInput = false; let heldResolve;
const state = {backend:'mock', enabled:false, calibrated:false, mode:'idle', status:'Ready', robot:{connected:true,joints:{},velocity:{}}, input:{source:'simulator',age_ms:0,deadman:false}, pico:{source:'simulator',connected:true,calibrated:false,age_ms:0}, command:{velocity:{},upper:{}}, events:[], supermimic:{available:false,reason:'Pending'}};
const context = {document,window,navigator:{getGamepads:() => [pad]},performance:{timeOrigin:1761807600000,now:(() => {let n=100;return () => ++n;})()},crypto:{randomUUID:()=>'ui-tab-test'},setInterval:(fn,ms)=>{intervals.push({fn,ms});return intervals.length;},clearInterval(){},setTimeout:()=>1,clearTimeout(){},AbortController,console,Math,Date,Array,Number,Set,Map,JSON,fetch:async(url,options={}) => {
  requests.push({url,options,body:options.body && JSON.parse(options.body)});
  if (url === '/api/input' && holdInput) await new Promise(resolve => {heldResolve=resolve;});
  return {ok:true,status:200,json:async()=>url==='/api/state'?structuredClone(state):{ok:true,state:structuredClone(state)}};
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
 holdInput=true; beat.fn(); await flush(); const count=requests.filter(r=>r.url==='/api/input').length; beat.fn(); beat.fn(); await flush(); assert.equal(requests.filter(r=>r.url==='/api/input').length,count,'no overlapping or queued input'); holdInput=false; heldResolve(); await flush();
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
 document.visibilityState='visible';window.emit('focus');pad=null;state.backend='ros2';state.calibrated=true;state.pico={source:'xrobotoolkit',connected:true,calibrated:true,age_ms:0,frame:{left_grip:0,right_grip:0}}; await intervals.find(x=>x.ms===100).fn();await flush();await beat.fn();await flush(); assert.equal(elements.get('enable-button').disabled,false,'native Pico standalone can enable without browser pad');
 const nativeInputCount=requests.filter(r=>r.url==='/api/input').length;await beat.fn();await flush();assert.equal(requests.filter(r=>r.url==='/api/input').length,nativeInputCount,'absent browser pad does not invalidate native Pico calibration');
 state.backend='mock';elements.get('source-select').value='simulator';elements.get('source-select').emit('change');await flush();await beat.fn();await flush();elements.get('left-grip').checked=true;await intervals.find(x=>x.ms===100).fn();await flush();assert.equal(elements.get('calibrate-button').disabled,true,'grips must be released before calibration');
 console.log('PASS: neutral mock pose, heartbeat, no overlapping input, held deadman release, focus/visibility stop, real-pad presence, native/simulator separation, action client ownership');
})().catch(error=>{console.error(error);process.exitCode=1;});
