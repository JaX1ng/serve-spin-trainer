// 打包：src/main.js → 一个内联脚本，塞进 src/index.template.html
//   node tools/build.mjs            → docs/index.html（网站版：three.js 用仓库里 docs/vendor 下的那份，不依赖外网）
//   node tools/build.mjs --artifact → 另外再出一份 dist/artifact.html（单文件版：three.js 走 CDN）
import { build } from 'esbuild';
import fs from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const at = (...p) => path.join(root, ...p);

const r = await build({ entryPoints: [at('src/main.js')], bundle: true, format: 'iife', minify: false, write: false, target: 'es2019', legalComments: 'none' });
const js = r.outputFiles[0].text.replace(/<\/script/gi, '<\\/script');
const tpl = fs.readFileSync(at('src/index.template.html'), 'utf8');
if (!tpl.includes('/*__BUNDLE__*/')) throw new Error('模板里找不到 /*__BUNDLE__*/ 标记');
const page = tpl.replace('/*__BUNDLE__*/', () => js);

const CDN = [
  ['https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js', 'vendor/three.min.js'],
  ['https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js', 'vendor/OrbitControls.js'],
];
let site = page;
for (const [cdn, local] of CDN) {
  if (!site.includes(cdn)) throw new Error('模板里找不到 ' + cdn);
  site = site.replace(cdn, local);
}
fs.mkdirSync(at('docs/vendor'), { recursive: true });
// vendor 里的两个文件来自 three@0.128.0；装过依赖就顺手同步一次，没装也不影响（仓库里已经带着）
for (const [src, dst] of [['build/three.min.js', 'three.min.js'], ['examples/js/controls/OrbitControls.js', 'OrbitControls.js'], ['LICENSE', 'LICENSE-three.txt']]) {
  const from = at('node_modules/three', src);
  if (fs.existsSync(from)) fs.copyFileSync(from, at('docs/vendor', dst));
}
fs.writeFileSync(at('docs/index.html'), site);
console.log('docs/index.html', (fs.statSync(at('docs/index.html')).size / 1024).toFixed(1), 'KB');

if (process.argv.includes('--artifact')) {
  fs.mkdirSync(at('dist'), { recursive: true });
  fs.writeFileSync(at('dist/artifact.html'), page);
  fs.writeFileSync(at('dist/index.html'), page);
  console.log('dist/artifact.html', (fs.statSync(at('dist/artifact.html')).size / 1024).toFixed(1), 'KB');
}
