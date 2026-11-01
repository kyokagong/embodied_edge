import json,time,sys
from pathlib import Path
from playwright.sync_api import sync_playwright, expect

out=Path('/tmp/edge-q1-browser');out.mkdir(exist_ok=True)
with sync_playwright() as p:
    browser=p.chromium.launch(headless=True,args=['--no-sandbox','--disable-dev-shm-usage'])
    page=browser.new_page(viewport={'width':1600,'height':1100})
    errors=[];page.on('pageerror',lambda error:errors.append(str(error)))
    r=page.goto(sys.argv[1] if len(sys.argv)>1 else 'http://127.0.0.1:8766/',wait_until='domcontentloaded')
    assert r.status==200
    page.bring_to_front();page.evaluate('window.dispatchEvent(new Event("focus"))')
    expect(page.locator('#calibrate-button')).to_be_enabled(timeout=15000)
    def state():return page.evaluate('fetch("/api/state").then(r=>r.json())')
    assert state()['backend']=='mock', 'browser test must never operate a real robot backend'
    def wait(predicate,description,timeout=8):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            s=state()
            if predicate(s):return s
            page.wait_for_timeout(100)
        raise AssertionError((description,s))
    def slider(name,value):
        page.locator('#'+name).evaluate('(el,v)=>{el.value=String(v);el.dispatchEvent(new Event("input",{bubbles:true}))}',value)
    page.screenshot(path=str(out/'initial.png'),full_page=True)
    page.locator('#calibrate-button').click();wait(lambda s:s['calibrated'],'calibrated')
    page.locator('#enable-button').click();wait(lambda s:s['enabled'],'enabled')
    page.locator('#walk-button').click();wait(lambda s:s['mode']=='walk','walkmode')
    slider('axis-1',-.7)
    rect=page.locator('#deadman-button').bounding_box();page.mouse.move(rect['x']+20,rect['y']+20);page.mouse.down()
    moving=wait(lambda s:s['command']['velocity']['forward']>.2,'forward deadman')
    page.mouse.up()
    wait(lambda s:s['command']['velocity']['forward']==0.,'release zeros')
    page.locator('#stop-button').click();wait(lambda s:not s['enabled'],'stop')
    page.locator('#neutral-button').click();page.locator('#pico-neutral-button').click()
    page.wait_for_timeout(150)
    page.locator('#calibrate-button').click();wait(lambda s:s['pico']['calibrated'],'picocalibrated')
    page.locator('#enable-button').click();wait(lambda s:s['enabled'],'reenabled')
    page.locator('#teleop-button').click();wait(lambda s:s['mode']=='teleop','teleopmode')
    page.locator('#left-grip').check();page.locator('#right-grip').check();page.wait_for_timeout(200)
    before=state()['robot']['joints']
    slider('left-y',.035)
    moved=wait(lambda s:s['enabled'] and any(abs(s['robot']['joints'][n]-before[n])>.005 for n in before),'upper moves')
    moved=wait(lambda s:s['enabled'] and s['pico']['ik_error_m']<.002,'upper converges',timeout=12)
    page.screenshot(path=str(out/'teleop.png'),full_page=True)
    page.locator('#left-grip').uncheck();page.locator('#right-grip').uncheck();page.wait_for_timeout(150)
    held=state()['robot']['joints'];slider('left-z',-.06);page.wait_for_timeout(250)
    assert all(abs(state()['robot']['joints'][n]-held[n])<1e-9 for n in held)
    page.evaluate('window.dispatchEvent(new Event("blur"))');wait(lambda s:not s['enabled'],'blur stops')
    assert not errors,errors
    page.set_viewport_size({'width':390,'height':844});page.wait_for_timeout(150)
    assert page.evaluate('document.documentElement.scrollWidth<=window.innerWidth+1'),'mobile horizontal overflow'
    page.screenshot(path=str(out/'mobile.png'),full_page=True)
    result=dict(passed=True,javascript_errors=errors,mock_backend=True,calibration_enable_walk_deadman_release=True,pico_relative_hand_ik=True,pico_grip_release_holds=True,blur_disables=True,mobile_no_horizontal_overflow=True,moving_velocity=moving['command']['velocity'],ik_error_m=moved['pico']['ik_error_m'])
    (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    browser.close()
