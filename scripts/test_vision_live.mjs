// Calls the configured agent with a fresh ROS camera image and audits model provenance.
// Observation mode must leave the landed vehicle and flight mission unchanged.
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {mkdir, writeFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {join, resolve} from 'node:path';
const require = createRequire(import.meta.url);
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = fileURLToPath(new URL('../', import.meta.url));
const output = resolve(root, process.env.UAV_TEST_OUTPUT_DIR || 'logs/vision');
const base = process.env.UAV_WEB_URL || 'http://127.0.0.1:8080';
const report = {started_at:new Date().toISOString(), actual_model_call:true, actual_flight:false, checks:[]};
let browser;
const errors=[];
const samples=[];
async function state() {const r=await fetch(base+'/api/state');assert.equal(r.status,200);return r.json();}
async function until(fn,label,timeout=120000) {
  const end=Date.now()+timeout;
  while(Date.now()<end){const value=await fn();if(value)return value;await new Promise(resolve=>setTimeout(resolve,300));}
  throw new Error('Timed out: '+label);
}
await mkdir(output,{recursive:true});
try {
  report.initial_state=await state();
  report.agent_architecture=report.initial_state.agent_architecture || 'dual_model';
  if(process.env.UAV_EXPECT_ARCHITECTURE) assert.equal(report.agent_architecture,process.env.UAV_EXPECT_ARCHITECTURE);
  assert(report.initial_state.ready && !report.initial_state.armed && report.initial_state.landed_state===1);
  assert(report.initial_state.vision_model?.configured);
  if(report.agent_architecture==='single_vlm') {
    assert.equal(report.initial_state.model?.name,report.initial_state.vision_model.name);
    assert.equal(report.initial_state.model?.provider,report.initial_state.vision_model.provider);
  }
  browser=await chromium.launch({headless:true,...(process.env.UAV_BROWSER_PATH?{executablePath:process.env.UAV_BROWSER_PATH}:{})});
  const page=await browser.newPage({viewport:{width:1440,height:1060}});
  page.on('pageerror',error=>errors.push(error.message));
  await page.goto(base,{waitUntil:'domcontentloaded'});
  await until(()=>page.locator('#submit-vision').isEnabled(),'vision input ready');
  if(report.agent_architecture==='single_vlm') {
    assert.equal(await page.locator('#agent-architecture').textContent(),'单 VLM Agent');
    assert.match(await page.locator('#vision-model').textContent(),/同一 VLM 处理指令与图像/);
  }
  await page.locator('#vision-query').fill('寻找当前视野中清晰可见的绿色地面图案，报告其画面位置和周围有什么；未发现或看不清请如实说明。');
  const accepted=page.waitForResponse(r=>r.url()===base+'/api/commands' && r.request().method()==='POST');
  await page.locator('#submit-vision').click();
  const response=await accepted;
  assert.equal(response.status(),202);
  assert.equal(response.request().postDataJSON().mode,'observe');
  const job=await response.json();report.command_id=job.id;
  console.log('Submitted real observation:',job.id);
  report.command=await until(async()=>{
    const s=await state();samples.push({armed:s.armed,mission_id:s.mission?.id,stamp:s.gazebo?.stamp_sim_s});
    assert(!s.armed,'Observation must not arm or take off');
    assert.equal(s.mission?.id,report.initial_state.mission?.id,'Observation must not change a flight mission');
    const r=await fetch(base+'/api/commands/'+job.id);const c=await r.json();
    return c.status!=='running' && c;
  },'real agent + image tool result');
  assert.equal(report.command.status,'succeeded',JSON.stringify(report.command));
  const observation=report.command.perception;
  assert.equal(observation?.status,'ok');
  assert.equal(observation.model,report.initial_state.vision_model.name);
  assert(observation.captured_at_sim_s>0);
  assert(observation.camera?.image_sha256?.length===64);
  assert.equal(observation.camera.width,640);assert.equal(observation.camera.height,480);
  assert(observation.camera.age_seconds<=2 && observation.camera.measurement_age_seconds<=.5);
  assert.equal(JSON.stringify(observation).includes('image_base64'),false);
  if(report.agent_architecture==='single_vlm') {
    assert.equal(observation.architecture,'single_vlm');
    assert.equal(typeof observation.capture_id,'string');
    assert(observation.capture_id.length>0);
    assert(Array.isArray(observation.vlm_calls) && observation.vlm_calls.length>0);
    assert(Array.isArray(report.command.model_calls) && report.command.model_calls.length>0);
    assert.deepEqual(report.command.model_calls,observation.vlm_calls,
      'The final report must include the same complete model trace as the command');
    assert(observation.vlm_calls.every(call=>call.model===report.initial_state.model.name),
      'Every audited model call must use the same configured VLM');
    assert(observation.vlm_calls.some(call=>Array.isArray(call.input_modalities)
      && call.input_modalities.includes('image') && Array.isArray(call.capture_ids)
      && call.capture_ids.includes(observation.capture_id)),
      'The same VLM must receive the image belonging to this capture');
    report.model_trace=observation.vlm_calls;
    report.checks.push('One configured VLM handles planning and the current capture; audited image-call capture ID matches the report');
  }
  assert(report.command.reply.includes('只报告本次视野'));
  if(observation.targets.slice(0,3).some(t=>t.approximate_location?.method==='ground_plane_projection')) {
    assert(report.command.reply.includes('Gazebo 世界坐标'));
  }
  await until(async()=>await page.locator('#vision-status').textContent()==='分析已完成','rendered vision result',10000);
  if(report.agent_architecture==='single_vlm') assert.match(await page.locator('#vision-report-meta').textContent(),/单 VLM/);
  assert.equal(await page.locator('#vision-summary').textContent(),observation.summary);
  assert.equal(await page.locator('#vision-targets > li').count(),observation.targets.length);
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
  await page.screenshot({path:join(output,'live_desktop.png'),fullPage:true});
  await page.setViewportSize({width:390,height:844});
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
  await page.screenshot({path:join(output,'live_mobile.png'),fullPage:true});
  report.final_state=await state();
  assert(!report.final_state.armed && report.final_state.landed_state===1);
  assert.deepEqual(errors,[]);
  report.checks.push('Real agent invoked the fresh-camera tools and returned a successful audited report',
    '640x480 image provenance and freshness present; no image payload in public report',
    'Model observations, target positions and surroundings render on desktop and mobile without JavaScript errors',
    'Observation mode leaves the landed/disarmed vehicle and flight mission unchanged');
  report.validation_passed=true;
  console.log('PASS: real camera -> '+report.agent_architecture+' agent -> web report');
} catch(error) {report.validation_passed=false;report.error=String(error.stack||error);console.error(error);process.exitCode=1;}
finally {
  report.samples=samples;
  report.finished_at=new Date().toISOString();
  await writeFile(join(output,'live_validation.json'),JSON.stringify(report,null,2));
  if(browser)await browser.close();
}
