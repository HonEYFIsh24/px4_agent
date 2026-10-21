// Real browser verification. --flight also executes two actual Gazebo sorties.
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {mkdir, writeFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
import {resolve, sep} from 'node:path';
const require = createRequire(import.meta.url);
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = fileURLToPath(new URL('../', import.meta.url));
const output = resolve(root, process.env.UAV_TEST_OUTPUT_DIR || 'logs/web') + sep;
const base = process.env.UAV_WEB_URL || 'http://127.0.0.1:8080';
const fly = process.argv.includes('--flight');
await mkdir(output, {recursive:true});
const report = {started_at:new Date().toISOString(), url:base, actual_flight:fly, checks:[], sorties:[]};
const errors = [];
const observations = [];
let browser;
let page;
async function state() {
  const response = await fetch(base + '/api/state');
  assert.equal(response.status, 200);
  return response.json();
}
async function until(predicate, label, timeout=150000) {
  const end = Date.now()+timeout;
  while (Date.now()<end) {
    const result = await predicate();
    if (result) return result;
    await new Promise(resolve=>setTimeout(resolve,250));
  }
  throw new Error('Timed out: '+label);
}
async function submit(prompt) {
  await page.locator('#prompt-input').fill(prompt);
  const acceptedResponse = page.waitForResponse(r=>r.url()===base+'/api/commands' && r.request().method()==='POST');
  await page.locator('#submit-command').click();
  const response = await acceptedResponse;
  assert.equal(response.status(),202);
  const job = await response.json();
  assert.equal(job.status,'running');
  console.log('Submitted:', job.id, prompt);
  return job.id;
}
async function command(id) {
  const response = await fetch(base+'/api/commands/'+id);
  assert.equal(response.status,200);
  return response.json();
}
async function clickControl(action) {
  const acceptedResponse=page.waitForResponse(r=>r.url()===base+'/api/'+action && r.request().method()==='POST');
  await page.locator('#'+(action==='stop'?'stop':'land')+'-button').click();
  const response=await acceptedResponse;
  assert.equal(response.status(),200);
  return response.json();
}
try {
  browser=await chromium.launch({headless:true, ...(process.env.UAV_BROWSER_PATH ? {executablePath:process.env.UAV_BROWSER_PATH} : {})});
  page=await browser.newPage({viewport:{width:1440,height:1060}});
  page.on('pageerror',e=>errors.push(e.message));
  page.on('response',async response=>{
    if(response.url()===base+'/api/state' && response.ok()) {
      const s=await response.json().catch(()=>null);
      if(s?.gazebo?.available) observations.push({wall_ms:Date.now(),gazebo:s.gazebo,armed:s.armed,mission:s.mission});
    }
  });
  await page.goto(base,{waitUntil:'domcontentloaded'});
  await until(async()=>await page.locator('#model-name').textContent()!=='等待配置','model display');
  const initial=await state();
  report.model=initial.model;
  report.agent_architecture=initial.agent_architecture || 'dual_model';
  report.vision_model=initial.vision_model;
  if(process.env.UAV_EXPECT_ARCHITECTURE) assert.equal(report.agent_architecture,process.env.UAV_EXPECT_ARCHITECTURE);
  assert.equal(await page.locator('#model-name').textContent(),initial.model.name || '未配置模型');
  if(report.agent_architecture==='single_vlm') {
    assert.equal(initial.model?.name,initial.vision_model?.name);
    assert.equal(initial.model?.provider,initial.vision_model?.provider);
    assert.equal(await page.locator('#agent-architecture').textContent(),'单 VLM Agent');
    report.checks.push('Single VLM architecture and shared planning/image model are displayed from server metadata');
  }
  assert.equal(await page.locator('#simulation-title').textContent(),'航迹监测');
  await until(async()=>await page.evaluate(()=>['left','right'].every(side=>{
    const image=document.getElementById('camera-'+side+'-image');
    return !image.hidden && image.complete && image.naturalWidth===640 && image.naturalHeight===480;
  })),'real stereo camera previews',30000);
  const firstFrame=await page.locator('#camera-left-image').getAttribute('src');
  await until(async()=>await page.locator('#camera-left-image').getAttribute('src')!==firstFrame,'camera frame refresh',5000);
  report.cameras={};
  for(const side of ['left','right']) {
    const response=await fetch(base+'/api/camera/'+side+'.jpg');
    assert.equal(response.status,200);
    assert.equal(response.headers.get('content-type'),'image/jpeg');
    const frame=Buffer.from(await response.arrayBuffer());
    assert.equal(frame.readUInt16BE(0),0xffd8);
    await writeFile(output+'camera_'+side+'.jpg',frame);
    report.cameras[side]={bytes:frame.length,sha256:createHash('sha256').update(frame).digest('hex'),
      metadata:(await state()).camera?.[side],caption:await page.locator('#camera-'+side+'-meta').textContent()};
    assert.equal(report.cameras[side].metadata?.available,true);
    assert(await page.locator('#camera-'+side+'-overlay').isHidden());
  }
  assert.notEqual(report.cameras.left.sha256,report.cameras.right.sha256);
  report.checks.push('tracking view named correctly; real 640x480 stereo JPEGs display and refresh independently');
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
  await page.screenshot({path:output+'desktop.png',fullPage:true});
  await page.setViewportSize({width:390,height:844});
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
  await page.screenshot({path:output+'mobile.png',fullPage:true});
  await page.setViewportSize({width:1440,height:1060});
  report.checks.push('desktop and mobile render without horizontal overflow');
  if(fly) {
    assert(initial.ready && initial.gazebo?.available && !initial.armed,'Start with a ready landed Gazebo vehicle');
    await until(async()=>!(await page.locator('#map-overlay').isVisible()),'live Gazebo display');
    const id=await submit('起飞到本地高度2米，飞到本地ENU坐标（1，0，2）米，在该点悬停2秒，然后返回起飞点上方并降落。');
    await until(async()=>{const s=await state();const job=await command(id);
      assert.notEqual(job.status,'failed',job.error || job.reply || 'Agent command failed');
      return s.armed && s.gazebo?.position_m?.[2]>1.5;},'observed takeoff');
    await page.screenshot({path:output+'airborne.png',fullPage:true});
    const result=await until(async()=>{const r=await command(id);return r.status!=='running' && r;},'natural language flight result',240000);
    assert.equal(result.status,'succeeded',JSON.stringify(result));
    assert.equal(result.mission?.status,'succeeded');
    const actions=result.mission.events.map(e=>e.action);
    assert.deepEqual(actions,['takeoff','goto','hold','return_home','land']);
    const landed=await state();
    assert(!landed.armed && landed.landed_state===1);
    const goto=result.mission.events.find(e=>e.action==='goto');
    assert(Math.hypot(goto.position_enu_m[0]-1,goto.position_enu_m[1],goto.position_enu_m[2]-2)<.3);
    await until(async()=>await page.locator('#mission-status').textContent()==='已完成','mission progress in browser');
    assert.equal(await page.locator('#mission-steps li.done').count(),5);
    report.sorties.push({kind:'natural-language-waypoint-return-land',command:result,final_state:landed});
    report.checks.push('real model tool calling, Gazebo takeoff, waypoint, hold, return, landing and disarming');
    console.log('PASS: natural language flight and visible mission progress');
    await until(async()=>!(await page.locator('#prompt-input').isDisabled()),'agent ready for next command');
    const interruptedId=await submit('起飞到本地高度2米并在该位置悬停60秒。');
    const holding=await until(async()=>{const s=await state();const job=await command(interruptedId);
      assert.notEqual(job.status,'failed',job.error || job.reply || 'Agent command failed');
      return s.mission?.action==='hold' && s.mission?.status==='running' && s;},'takeoff and long hold');
    const concurrent=await page.evaluate(async()=>{
      const r=await fetch('/api/commands',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({prompt:'降落'})});return r.status;
    });
    assert.equal(concurrent,409);
    const stop=await clickControl('stop');
    assert(stop.holding);
    const cancelled=await until(async()=>{const s=await state();return s.mission?.id===holding.mission.id && s.mission.status==='cancelled' && s;},'cancelled mission');
    assert(cancelled.armed && cancelled.mode==='OFFBOARD');
    const land=await clickControl('land');
    const final=await until(async()=>{const s=await state();return s.mission?.id===land.mission.id && s.mission.status==='succeeded' && !s.armed && s.landed_state===1 && s;},'direct landing');
    const interrupted=await until(async()=>{const r=await command(interruptedId);return r.status!=='running' && r;},'interrupted agent result');
    assert.equal(interrupted.status,'failed');
    report.sorties.push({kind:'cancel-hold-immediate-land',command:interrupted,cancelled_mission:cancelled.mission,final_state:final});
    report.checks.push('concurrent command rejected, browser stop/hold, immediate landing, interrupted agent cannot resume');
    assert(observations.length>15);
    assert(Math.max(...observations.map(o=>o.gazebo.position_m[2]))>1.8);
    report.gazebo_samples=observations.length;
    await page.screenshot({path:output+'landed.png',fullPage:true});
    console.log('PASS: stop/hold and direct land buttons');
  }
  // Only camera transport is interrupted here, to verify stale frames clear and recover.
  await page.route('**/api/camera/*.jpg',route=>route.fulfill({status:404,contentType:'application/json',body:'{"error":"no fresh frame"}'}));
  await until(async()=>await page.locator('#camera-left-overlay').isVisible()
    && await page.locator('#camera-right-overlay').isVisible(),'camera stream outage',10000);
  for(const side of ['left','right']) assert.equal(await page.locator('#camera-'+side+'-image').getAttribute('src'),null);
  await page.screenshot({path:output+'camera_disconnected.png',fullPage:true});
  await page.unroute('**/api/camera/*.jpg');
  await until(async()=>await page.locator('#camera-left-overlay').isHidden()
    && await page.locator('#camera-right-overlay').isHidden(),'camera stream recovery',10000);
  report.checks.push('camera outage removes old images and previews recover automatically');
  // Test the actual page's disconnected rendering; no flight API is replaced.
  await page.route('**/api/state',route=>route.abort('connectionrefused'));
  await until(async()=>await page.locator('#connection-pill').textContent()==='服务未连接','offline indication',20000);
  assert.equal(await page.locator('#stop-button').isDisabled(),true);
  assert.equal(await page.locator('#land-button').isDisabled(),true);
  assert.equal(await page.locator('#px4-position').textContent(),'—');
  assert(await page.locator('#map-overlay').isVisible());
  assert(await page.locator('#camera-left-overlay').isVisible());
  assert(await page.locator('#camera-right-overlay').isVisible());
  await page.screenshot({path:output+'disconnected.png',fullPage:true});
  report.checks.push('disconnected page clears stale positions and disables direct controls');
  assert.deepEqual(errors,[]);
  report.checks.push('no browser JavaScript errors');
  report.validation_passed=true;
  console.log('PASS: operator page verification');
} catch(error) {
  report.validation_passed=false;
  report.error=String(error.stack || error);
  console.error(error);
  process.exitCode=1;
} finally {
  if(fly) {
    try {if((await state()).armed) await fetch(base+'/api/land',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});} catch{}
  }
  report.finished_at=new Date().toISOString();
  await writeFile(output+(fly?'validation.json':'render_validation.json'),JSON.stringify(report,null,2));
  if(fly) await writeFile(output+'gazebo_observations.json',JSON.stringify(observations));
  if(browser) await browser.close();
}
