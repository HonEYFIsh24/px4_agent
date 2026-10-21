// Local browser contract verification. This server cannot execute flight actions.
import assert from 'node:assert/strict';
import {createServer} from 'node:http';
import {readFile, mkdir, writeFile} from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {createRequire} from 'node:module';
const require = createRequire(import.meta.url);
const {chromium} = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = fileURLToPath(new URL('../', import.meta.url));
const output = root + 'logs/web/';
const report = {started_at: new Date().toISOString(), actual_model_call: false, actual_flight: false, checks: []};
const view = {
  ready: true, connected: true, armed: false, mode: 'POSCTL', landed_state: 1,
  position_enu_m: [0, 0, 0], target_enu_m: [0, 0, 0],
  model: {name: 'fixture-agent', provider: 'contract-test', available: true},
  vision_model: {name: 'fixture-vision', provider: 'contract-test', configured: true},
  latest_perception: null, camera: {}, gazebo: {}, vio: {}, mission: null,
};
const posts = [];
let result = {id: 'fixture-1', status: 'running', perception: null};
let browser;
const server = createServer(async (req, res) => {
  try {
    const path = new URL(req.url, 'http://localhost').pathname;
    let body;
    let contentType = 'application/json';
    if (path === '/api/state') body = JSON.stringify(view);
    else if (path === '/api/commands' && req.method === 'POST') {
      let raw = '';
      for await (const chunk of req) raw += chunk;
      posts.push(JSON.parse(raw));
      result = {id: 'fixture-' + posts.length, status: 'running', perception: null};
      res.statusCode = 202;
      body = JSON.stringify(result);
    } else if (path.startsWith('/api/commands/')) body = JSON.stringify(result);
    else if (['/', '/app.js', '/styles.css'].includes(path)) {
      body = await readFile(root + 'web/' + (path === '/' ? 'index.html' : path.slice(1)));
      contentType = path.endsWith('.js') ? 'text/javascript' : path.endsWith('.css') ? 'text/css' : 'text/html';
    } else {res.statusCode = 404; body = '{"error":"fixture route unavailable"}';}
    res.setHeader('Content-Type', contentType);
    res.end(body);
  } catch (error) {res.statusCode = 500; res.end(JSON.stringify({error: error.message}));}
});
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
await mkdir(output, {recursive: true});
try {
  browser = await chromium.launch({headless: true, ...(process.env.UAV_BROWSER_PATH ? {executablePath: process.env.UAV_BROWSER_PATH} : {})});
  const page = await browser.newPage({viewport: {width: 1440, height: 1060}});
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  await page.goto('http://127.0.0.1:' + server.address().port, {waitUntil: 'domcontentloaded'});
  await page.waitForFunction(() => !document.getElementById('submit-vision').disabled);
  assert.match(await page.locator('#vision-model').textContent(), /fixture-vision/);
  await page.locator('[data-vision-prompt^="寻找红色物体"]').click();
  await page.locator('#submit-vision').click();
  await page.waitForFunction(() => document.getElementById('submit-command').disabled);
  assert.equal(await page.locator('#vision-query').isDisabled(), true);
  assert.equal(await page.locator('#submit-vision').isDisabled(), true);
  assert.equal(posts.length, 1);
  assert.equal(posts[0].mode, 'observe');
  assert.match(posts[0].prompt, /当前机载相机图像/);
  assert.match(posts[0].prompt, /不要为此次观察起飞或移动/);
  report.checks.push('visual and flight forms share one active command lock and /api/commands');
  const perception = {
    status: 'ok', query: '寻找红色物体', model: 'fixture-vision', captured_at_sim_s: 12.34,
    summary: '<img src=x onerror="window.injected=true">可见一个红色物体。',
    targets: [{label: '红色盒子', confidence: 0.82, bbox: [0.1, 0.2, 0.3, 0.5],
      image_position: '画面左侧', approximate_location: {frame: 'camera_optical', method: 'bearing_only', bearing_right_deg: -12.4, elevation_up_deg: 3.5, reason: '目标未落在地面。'}, surroundings: '右侧有灰色墙面'}],
    limitations: ['单幅图像无法测量真实距离。'],
  };
  result = {...result, status: 'succeeded', reply: '已分析相机图像。', perception};
  view.latest_perception = perception;
  await page.waitForFunction(() => document.getElementById('vision-status').textContent === '分析已完成');
  assert.match(await page.locator('#vision-summary').textContent(), /<img src=x/);
  assert.equal(await page.locator('#vision-summary img').count(), 0);
  assert.equal(await page.evaluate(() => window.injected), undefined);
  assert.match(await page.locator('#vision-report-meta').textContent(), /12.34/);
  assert.match(await page.locator('#vision-targets').textContent(), /红色盒子/);
  assert.match(await page.locator('#vision-targets').textContent(), /-12.4°/);
  assert.match(await page.locator('#vision-targets').textContent(), /未测量距离/);
  assert.match(await page.locator('#vision-limitations').textContent(), /无法测量真实距离/);
  assert.equal(await page.locator('#submit-command').isDisabled(), false);
  report.checks.push('structured reports render snapshot time, target details and uncertainty with textContent');
  await page.locator('#vision-query').fill('描述当前场景');
  await page.locator('#submit-vision').click();
  result = {...result, status: 'failed', error: 'Vision API rejected image', reply: '视觉调用失败。',
    perception: {status: 'error', query: '描述当前场景', model: 'fixture-vision',
      captured_at_sim_s: 13.4, summary: '', targets: [], limitations: [], error: 'Vision API rejected image'}};
  view.latest_perception = result.perception;
  await page.waitForFunction(() => document.getElementById('vision-status').textContent === '分析失败');
  assert.equal(await page.locator('#vision-report-error').isVisible(), true);
  assert.match(await page.locator('#vision-report-error').textContent(), /Vision API rejected image/);
  assert.equal(await page.locator('#vision-targets li').count(), 0);
  assert.equal(await page.locator('#submit-command').isDisabled(), false);
  report.checks.push('model failures stay visible and clear earlier detections');
  view.vision_model.configured = false;
  await page.waitForFunction(() => document.getElementById('submit-vision').disabled);
  assert.match(await page.locator('#vision-model').textContent(), /尚未配置/);
  assert.equal(await page.locator('#submit-command').isDisabled(), false);
  report.checks.push('unconfigured visual model disables image analysis without disabling flight commands');
  await page.screenshot({path: output + 'vision_contract_desktop.png', fullPage: true});
  await page.setViewportSize({width: 390, height: 844});
  assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
  await page.screenshot({path: output + 'vision_contract_mobile.png', fullPage: true});
  assert.deepEqual(errors, []);
  report.checks.push('desktop/mobile layouts have no horizontal overflow or browser errors');
  report.validation_passed = true;
  console.log('PASS: vision UI contract validation (fixture only; no real flight or model request)');
} catch (error) {
  report.validation_passed = false;
  report.error = String(error.stack || error);
  process.exitCode = 1;
  console.error(error);
} finally {
  report.finished_at = new Date().toISOString();
  await writeFile(output + 'vision_contract_validation.json', JSON.stringify(report, null, 2));
  if (browser) await browser.close();
  await new Promise(resolve => server.close(resolve));
}
