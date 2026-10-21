// Read-only verification of the real navigation pane and stereo previews.
import assert from 'node:assert/strict';
import {createRequire} from 'node:module';
import {mkdir,writeFile} from 'node:fs/promises';
import {resolve} from 'node:path';
const require=createRequire(import.meta.url);
const {chromium}=require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const output=resolve(process.env.UAV_TEST_OUTPUT_DIR || 'logs/avoidance/ui');
const base=process.env.UAV_WEB_URL || 'http://127.0.0.1:8080';
const report={started_at:new Date().toISOString(),actual_flight:false,model_calls:false,
  server_mutations:false,checks:[],screenshots:[]};
let browser;
await mkdir(output,{recursive:true});
try {
  browser=await chromium.launch({headless:true,executablePath:process.env.UAV_BROWSER_PATH || '/usr/bin/microsoft-edge'});
  const page=await browser.newPage({viewport:{width:1440,height:1060}});
  const errors=[];
  page.on('pageerror',error=>errors.push(error.message));
  await page.route('**/*',route=>route.request().method()==='GET' ? route.continue() : route.abort('blockedbyclient'));
  await page.goto(base,{waitUntil:'domcontentloaded'});
  await page.waitForFunction(()=>state?.navigation?.enabled===true && state.navigation.lidar_available===true,
    null,{timeout:30000});
  await page.waitForFunction(()=>['left','right'].every(side=>{
    const image=document.getElementById('camera-'+side+'-image');
    return !image.hidden && image.complete && image.naturalWidth===640 && image.naturalHeight===480;
  }),null,{timeout:30000});
  report.live=await page.evaluate(()=>({scene_id:state.scene?.id,scene_navigation:state.scene?.navigation,
    navigation:state.navigation,armed:state.armed,landed_state:state.landed_state,
    camera:state.camera,display:{status:document.getElementById('navigation-status').textContent,
      scans:document.getElementById('navigation-scans').textContent,
      source:document.getElementById('navigation-source').textContent,
      nearest:document.getElementById('navigation-distance').textContent,
      path:document.getElementById('navigation-path').textContent,
      map:document.getElementById('navigation-map').textContent,
      scope:document.getElementById('navigation-scope').textContent,
      detail:document.getElementById('navigation-detail').textContent}}));
  const nav=report.live.navigation;
  assert.equal(nav.enabled,true);
  assert.equal(nav.lidar_available,true);
  assert.equal(nav.lidar_sample_count,360);
  assert.equal(report.live.scene_navigation.flight_altitude_m,3.5);
  assert(report.live.display.scope.includes('3.5 m') && report.live.display.scope.includes('静态二维'));
  assert(report.live.display.scans.includes('360 束'));
  assert.equal(report.live.display.status,!nav.active ? '待起飞建图' : nav.fresh ? '建图可用' : '等待有效建图');
  report.checks.push('live LiDAR availability, 360 beams, mapping phase and fixed 3.5 m static 2-D scope agree with API');
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
  await page.screenshot({path:resolve(output,'navigation_desktop.png'),fullPage:true});
  report.screenshots.push('navigation_desktop.png');
  await page.setViewportSize({width:390,height:844});
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
  await page.screenshot({path:resolve(output,'navigation_mobile.png'),fullPage:true});
  report.screenshots.push('navigation_mobile.png');
  report.checks.push('desktop/mobile navigation pane and real stereo previews render without horizontal overflow');
  assert.deepEqual(errors,[]);
  report.checks.push('no JavaScript errors; all non-GET requests blocked inside this read-only browser');
  report.validation_passed=true;
  console.log('PASS: read-only navigation UI verification');
} catch(error) {
  report.validation_passed=false;
  report.error=String(error.stack || error);
  console.error(error);
  process.exitCode=1;
} finally {
  report.finished_at=new Date().toISOString();
  await writeFile(resolve(output,'navigation_smoke.json'),JSON.stringify(report,null,2));
  if(browser) await browser.close();
}
