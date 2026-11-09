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
const reportPath = path.resolve(option('--report', 'q1/rust/reports/real-ui-one-click.json'));
const screenshotPath = path.resolve(option('--screenshot', 'q1/rust/reports/real-ui-one-click.png'));
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
  let before, final, error = null, ok = false; const samples = []; let version;
  try {
    version = await rpc('Browser.getVersion');
    const target = await rpc('Target.createTarget', {url:'about:blank'});
    session = (await rpc('Target.attachToTarget', {targetId:target.targetId,flatten:true})).sessionId;
    await rpc('Page.enable', {}, session); await rpc('Runtime.enable', {}, session); await rpc('Network.enable', {}, session);
    await rpc('Page.bringToFront', {}, session);
    await rpc('Page.navigate', {url:base+'/?real_ui_check='+Date.now()}, session);
    for (let i=0;i<60;i++) {if(await evaluate('!!document.getElementById("stand")')) break; await sleep(100);}
    before = await getState();
    assert.equal(before.controller, 'supermimic_stand'); assert.equal(before.bridge.publish_hz, 50);
    if (execute) {
      assert.ok(!before.enabled, 'This check starts from the currently disabled state');
      for (let i=0;i<30;i++) {if(await evaluate('!document.getElementById("stand").disabled')) break; await sleep(100);}
      assert.ok(await evaluate('!document.getElementById("stand").disabled'), 'One-click stand must be available without a password');
      await click('stand');
      const began = Date.now();
      while (Date.now()-began < seconds*1000) {
        const state = await getState(); samples.push(state);
        if (samples.length%5===0) console.log(JSON.stringify({phase:state.phase,actual_hz:state.bridge.tx_hz,accepted:state.bridge.accepted,elapsed_s:state.stand_elapsed_s}));
        if (samples.some(s=>s.enabled) && state.phase==='FAULT') throw Error(state.fault || state.guard_fault || 'Runtime fault');
        await sleep(200);
      }
      final = await getState();
      ok = actions.filter(a=>a.action==='stand' && a.status===200).length===1 && actions.some(a=>a.action==='pair' && a.status===200) && samples.some(s=>['POLICY_STAND','STANDING'].includes(s.phase)) && final.enabled;
      assert.ok(ok, 'One click must run real takeover, preparation and model standing');
    } else {final=before;ok=true;}
    await screenshot();
  } catch (e) {error=e.message; try{final=await getState();await screenshot();}catch{}}
  finally {
    const report = {real_hardware:true,real_browser:version?.product,execute_stand:execute,success:ok,error,stand_button_clicks:actions.some(a=>a.action==='stand')?1:0,actions,heartbeat_count:heartbeatCount,browser_errors:browserErrors,browser_stderr:browserStderr,before,samples,final,standing_verified:final?.standing_verified,ground_support_verified:false};
    fs.writeFileSync(reportPath, JSON.stringify(report,null,2)+'\n', {mode:0o600});
    console.log(JSON.stringify({success:ok,error,actions,final_phase:final?.phase,model_enabled:final?.enabled,standing_verified:final?.standing_verified,report:reportPath}));
    try {await rpc('Browser.close');} catch {}
    await sleep(300); if (browser.exitCode===null) browser.kill('SIGTERM');
    await sleep(300); try {fs.rmSync(profile,{recursive:true,force:true,maxRetries:10,retryDelay:100});} catch {}
    process.exitCode=ok?0:2;
  }
})();
