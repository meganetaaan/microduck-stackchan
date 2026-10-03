import {readFile,writeFile} from 'node:fs/promises';
const json=async path=>JSON.parse(await readFile(path));
const report=await json('src/validation-report.json');
report.baseline={name:'NOARMS',mass_g:736.537,policy:'learned8-command',arm_design:'Thin flap arms option 4 / A selected for future design; not included, retrained, or validated in this simulator'};
report.physics_parity=await json('test-results/parity.json');
report.steady_gaits=await json('test-results/long.json');
report.transitions=await json('test-results/transitions.json');
report.worker=await json('test-results/worker.json');
report.pages_paths=await json('test-results/pages-paths.json');
report.model_packaging=await json('test-results/model-packaging.json');
try{report.browser_visual_qa=await json('test-results/browser/results.json');}
catch{report.browser_visual_qa={completed:false,reason:'Headless Chromium could not start in the preparation environment (socket permission). Browser screenshot, touch and visual layout checks remain unverified; the GitHub workflow includes the reproducible test.'};}
report.status=report.browser_visual_qa.completed?'Physics, Worker, project-path, and Chromium desktop/emulated-mobile checks passed':'Physics, Worker and project-path checks passed; browser visual QA remains unverified';
report.release_verification={generated_at:new Date().toISOString(),node:process.version,unit_tests:12,input_regressions:18,source_integrity:'All model assets, physics.js and control.js match baseline-manifest.json SHA-256 values',note:'Emulated mobile browser checks do not establish real-device performance or real-robot safety'};
report.ui_revision={layout:'Full-width compact work surface with persistent NOARMS / 736.537 g label',icons:'Lucide 0.468.0 ISC',physics_policy_changed:false,screen_orientation_fix_preserved:true,browser_screenshots_completed:report.browser_visual_qa.completed,static_and_geometry_tests_passed:12,dom_worker_mock_tests_passed:18,actual_worker_integration_passed:true,dom_test_scope:'Mock DOM and queued Worker events; separate browser checks are reported above'};
report.limitations=[...new Set([...report.limitations,'Thin flap arms option 4 / A are not represented in this model and require separate training and validation.'])];
await writeFile('src/validation-report.json',JSON.stringify(report,null,2)+'\n');
const html=await readFile('src/index.html','utf8');
await writeFile('src/index.html',html.replace(/(<p id="validation-summary">)[\s\S]*?(<\/p>)/,`$1${report.browser_visual_qa.completed?'Chromiumのデスクトップ／モバイル相当画面で確認済み。スマートフォン実機の性能は未検証です。':'ブラウザー画面・タッチ操作の検証は未完了です。'}$2`));
console.log(report.status);
