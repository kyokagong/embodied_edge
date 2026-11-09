// Actual Chrome, actual Q1 UI/API, actual actuator feedback. No mocked DOM or ROS.
const fs = require('node:fs');
const path = require('node:path');
const os = require('node:os');
const {spawn} = require('node:child_process');
const assert = require('node:assert/strict');
const args = process.argv.slice(2);
const option = (name, fallback) => {const i = args.indexOf(name); return i < 0 ? fallback : args[i+1];};
const execute = args.includes('--execute-stand');
const base = option('--url', 'http://192.168.54.41:8768');
assert.equal(base, 'http://192.168.54.41:8768', 'This check is restricted to the actual Q1');
const reportPath = path.resolve(option('--report', 'q1/rust/reports/real-ui-dance2.json'));
const screenshotPath = path.resolve(option('--screenshot', 'q1/rust/reports/real-ui-dance2.png'));
const seconds = Number(option('--seconds', '45'));
assert.ok(seconds >= 25 && seconds <= 90);
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'q1-real-ui-'));
const browser = spawn('/usr/bin/google-chrome', ['--headless=new','--no-sandbox','--no-zygote','--password-store=basic','--no-first-run','--no-default-browser-check','--disable-background-networking','--disable-sync','--disable-gpu','--disable-dev-shm-usage','--disable-background-timer-throttling','--disable-renderer-backgrounding','--no-proxy-server','--disable-features=HttpsUpgrades,NetworkServiceSandbox','--remote-debugging-pipe','--user-data-dir='+profile,'about:blank'], {stdio:['ignore','ignore','pipe','pipe','pipe']});
let nextId = 0, buffer = '', session, heartbeatCount = 0;
let browserStderr='';
const pending = new Map(), actions = [], requests = new Map(), browserErrors = [];
function rpc(method, params = {}, sessionId) {
  const id = ++nextId;
  return new Promise((resolve, reject) => {
    const timeout = setTimeout(() => {pending.delete(id); reject(Error(`Chrome ${method} timed out`));}, 8000);
    pending.set(id, {resolve, reject, timeout});
    browser.stdio[3].write(JSON.stringify({id, method, params, ...(sessionId ? {sessionId} : {})})+'\0');
  });
}
browser.stdio[4].on('data', bytes => {
  buffer += bytes;
  let separator;
  while ((separator = buffer.indexOf('\0')) >= 0) {
    const message = JSON.parse(buffer.slice(0, separator)); buffer = buffer.slice(separator+1);
    if (message.id && pending.has(message.id)) {
      const item = pending.get(message.id); pending.delete(message.id); clearTimeout(item.timeout);
      if (message.error) item.reject(Error(message.error.message)); else item.resolve(message.result);
    } else if (message.method === 'Network.requestWillBeSent' && message.params.request.url.endsWith('/api/action')) {
      const body = JSON.parse(message.params.request.postData || '{}');
      if (body.action === 'heartbeat') heartbeatCount++;
      else {const entry = {action:body.action, at:Date.now(), status:null}; actions.push(entry); requests.set(message.params.requestId, entry);}
    } else if (message.method === 'Network.responseReceived' && requests.has(message.params.requestId)) {
      requests.get(message.params.requestId).status = message.params.response.status;
    } else if (message.method === 'Runtime.exceptionThrown') browserErrors.push(message.params.exceptionDetails.text);
  }
});
browser.stderr.on('data', bytes => {browserStderr=(browserStderr+bytes).slice(-16000);fs.writeFileSync(reportPath+'.chrome.log',browserStderr,{mode:0o600});});
for (const stream of [browser.stdio[3],browser.stdio[4]]) stream.on('error',e=>browserErrors.push(e.message));
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
async function evaluate(expression) {
  const result = await rpc('Runtime.evaluate', {expression, returnByValue:true, awaitPromise:true}, session);
  if (result.exceptionDetails) throw Error(result.exceptionDetails.text);
  return result.result.value;
}
async function getState() {const response = await fetch(base+'/api/state', {signal:AbortSignal.timeout(2000)}); assert.ok(response.ok); return response.json();}
async function click(id) {
  await evaluate(`document.getElementById(${JSON.stringify(id)}).scrollIntoView({block:'center'})`);
  const rect = await evaluate(`(()=>{const r=document.getElementById(${JSON.stringify(id)}).getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2}})()`);
  await rpc('Input.dispatchMouseEvent', {type:'mousePressed',button:'left',clickCount:1,...rect}, session);
  await rpc('Input.dispatchMouseEvent', {type:'mouseReleased',button:'left',clickCount:1,...rect}, session);
}
async function screenshot() {
  await evaluate('window.scrollTo(0,0)');
  const value = await rpc('Page.captureScreenshot', {format:'png',captureBeyondViewport:true}, session);
  fs.writeFileSync(screenshotPath, Buffer.from(value.data, 'base64'), {mode:0o600});
}
(async () => {
  let before, final, error=null, ok=false, version, danceStarted, cancelAt, played, standingPose;
  const samples=[];
  async function sample() {
    const state=await getState(); samples.push({at:Date.now(),state});
    if((state.enabled||samples.some(s=>s.state.enabled))&&(state.phase==='FAULT'||state.guard_fault||state.fault)) throw Error(state.fault||state.guard_fault||'Runtime fault');
    return state;
  }
  async function waitFor(predicate, timeout, label) {
    const until=Date.now()+timeout;
    while(Date.now()<until) {const state=await sample();if(predicate(state)) return state;await sleep(100);}
    throw Error('Timeout: '+label);
  }
  try {
    assert.ok(execute,'Pass --execute-stand to authorize this real actuator test');
    version=await rpc('Browser.getVersion');
    const target=await rpc('Target.createTarget',{url:'about:blank'});
    session=(await rpc('Target.attachToTarget',{targetId:target.targetId,flatten:true})).sessionId;
    await rpc('Page.enable',{},session);await rpc('Runtime.enable',{},session);await rpc('Network.enable',{},session);
    await rpc('Page.bringToFront',{},session);await rpc('Page.navigate',{url:base+'/?real_dance_check='+Date.now()},session);
    for(let i=0;i<60;i++){if(await evaluate('!!document.getElementById("play-dance")'))break;await sleep(100);}
    assert.equal(await evaluate('!!document.querySelector("input[type=password]")'),false,'No operation password');
    before=await getState();assert.equal(before.controller,'supermimic_stand');assert.equal(before.bridge.publish_hz,50);
    const configPath=option('--expected-config',null);
    if(configPath){
      const bytes=fs.readFileSync(configPath),config=JSON.parse(bytes);
      assert.equal(before.config.sha256,require('node:crypto').createHash('sha256').update(bytes).digest('hex'));
      assert.ok(before.effort_limits_nm.every((v,i)=>Math.abs(v-config.stand_trial.effort_limit[i])<1e-5),'Actual torque caps must match the requested configuration');
    }
    assert.equal(before.enabled,false,'Deployment must wait for an explicit UI stand click');
    for(let i=0;i<40;i++){if(await evaluate('!document.getElementById("stand").disabled'))break;await sleep(100);}
    await click('stand');
    const standing=await waitFor(s=>s.enabled&&['POLICY_STAND','STANDING'].includes(s.phase)&&s.stand_elapsed_s>=30,45000,'one click standing');
    console.log(JSON.stringify({stage:'standing',phase:standing.phase,tx:standing.bridge.tx_count}));
    await evaluate('document.getElementById("dance-select").value="dance2_subject1"');
    for(let i=0;i<30;i++){if(await evaluate('!document.getElementById("play-dance").disabled'))break;await sleep(100);}
    await click('play-dance');
    const dance=await waitFor(s=>s.dance==='dance2_subject1'&&s.dance_playing&&s.dance_frame>0,4000,'dance 2 start');
    danceStarted=Date.now();standingPose=dance.stand_pose;
    while(Date.now()-danceStarted<30000) {
      played=await sample();assert.equal(played.dance,'dance2_subject1');assert.equal(played.dance_playing,true);assert.equal(played.enabled,true);
      if(samples.length%20===0) console.log(JSON.stringify({stage:'dance',frame:played.dance_frame,elapsed:played.dance_elapsed_s,hz:played.bridge.tx_hz}));
      await sleep(Math.min(150,Math.max(1,30000-(Date.now()-danceStarted))));
    }
    cancelAt=Date.now();await click('cancel-dance');
    await waitFor(s=>!s.returning_to_stand&&!s.dance_playing&&['POLICY_STAND','STANDING'].includes(s.phase),10000,'same actor resumes zero-velocity standing');
    await sleep(2000);final=await sample();
    assert.equal(final.enabled,true);assert.equal(final.bridge.mode,'ACTIVE');
    assert.ok(played.dance_frame>=1400&&played.dance_frame<=1600,'Actual reference must advance about 1500 frames in 30 seconds');
    assert.ok(final.bridge.tx_count-dance.bridge.tx_count>=1500,'Actual HAL output must continue throughout dance and condition switch');
    assert.ok(final.model_count-standing.model_count>1400,'Actual ONNX inference must advance');
    assert.equal(actions.filter(a=>a.action==='stand'&&a.status===200).length,1);
    assert.equal(actions.filter(a=>a.action==='play_dance'&&a.status===200).length,1);
    assert.equal(actions.filter(a=>a.action==='cancel_dance'&&a.status===200).length,1);
    assert.equal(browserErrors.length,0);ok=true;await screenshot();
  } catch(e) {
    error=e.message;
    try {
      final=await getState();
      if(final.dance_playing){await click('cancel-dance');await sleep(5500);final=await getState();}
      await screenshot();
    } catch {}
  } finally {
    const report={real_hardware:true,real_browser:version?.product,no_password:true,success:ok,error,dance:'dance2_subject1',requested_test_seconds:30,actual_play_wall_seconds:cancelAt&&danceStarted?(cancelAt-danceStarted)/1000:null,dance_frame:played?.dance_frame,standing_pose:standingPose,expected_config:option('--expected-config',null),cancel_behavior:final?.cancel_behavior,loaded_config:final?.config,effort_limits_nm:final?.effort_limits_nm,actions,heartbeat_count:heartbeatCount,browser_errors:browserErrors,before,samples,final,ground_support_verified:false};
    fs.writeFileSync(reportPath,JSON.stringify(report,null,2)+'\n',{mode:0o600});
    console.log(JSON.stringify({success:ok,error,actions,final_phase:final?.phase,model_enabled:final?.enabled,dance_frame:played?.dance_frame,report:reportPath}));
    try{await rpc('Browser.close');}catch{}
    await sleep(300);if(browser.exitCode===null)browser.kill('SIGTERM');
    await sleep(300);try{fs.rmSync(profile,{recursive:true,force:true,maxRetries:10,retryDelay:100});}catch{}
    process.exitCode=ok?0:2;
  }
})();
