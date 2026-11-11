// 发球生成器的回归测试：每种 类型 × 左右手 × 站位 × 长短 × 意图 各生成一个，逐项核对
import { P, collide, simulate, len, sub, dot } from '../src/physics.js';
import { generateServe, mulberry32, TYPE_KEYS, STYLE_KEYS, TYPES, RPS, judge, spinParts, qRot } from '../src/serve.js';

const rng = mulberry32(12345);
let fails = 0;
const ok = (c, msg) => { console.log((c ? 'PASS ' : 'FAIL ') + msg); if (!c) { fails++; process.exitCode = 1; } };
const rows = [], nulls = [], reasons = {};
let n = 0, tsum = 0, tmax = 0;
for (const type of TYPE_KEYS) for (const hand of [1, -1]) for (const pos of [-1, 0, 1]) for (const len_ of ['short', 'long']) for (const intent of ['top', 'back', 'flat']) {
  const dbg = [];
  const style = STYLE_KEYS[n % STYLE_KEYS.length];
  const t0 = performance.now();
  const s = generateServe({ type, hand, intent, pos, len: len_, style, fake: n % 2 === 0, _dbg: dbg }, rng);
  const dt = performance.now() - t0; tsum += dt; tmax = Math.max(tmax, dt); n++;
  for (const r of dbg) { const k = `${type}:${r.split(':')[0]}`; reasons[k] = (reasons[k] || 0) + 1; }
  if (!s) { nulls.push(`${type}/${intent}/${len_}/h${hand}/p${pos}/${style}`); continue; }
  // 正向复算：同样的拍面 + 拍速再碰一次，出手状态必须一模一样
  const c = collide(s.toss.vb, [0, 0, 0], s.n, s.vr, P.rubber);
  // 画出来的挥拍在触球瞬间的速度（差分）和拍面，必须等于物理用的那组
  const e = 1e-4, pa = s.racketAt(-e).b, pb = s.racketAt(e).b;
  const vfd = [(pb[0] - pa[0]) / (2 * e), (pb[1] - pa[1]) / (2 * e), (pb[2] - pa[2]) / (2 * e)];
  const nShown = qRot(s.racketAt(0).q, [0, 0, TYPES[type].face]);
  rows.push({
    type, intent, len: len_, hand, s,
    fwdErr: len(sub(c.v, s.col.v)) + len(sub(c.w, s.col.w)) * P.R,
    kinErr: len(sub(vfd, s.vr)), faceErr: 1 - dot(nShown, s.n),
    sideOK: Math.sign(s.launch.side) === TYPES[type].side * hand,
    lenOK: len_ === 'short' ? s.verdict.b2.p[2] < 0.6 : s.verdict.b2.p[2] > 0.8,
    b1OK: s.verdict.b1.p[2] < 0 && Math.abs(s.verdict.b1.p[0]) < P.tableW / 2, b2OK: s.verdict.b2.p[2] > 0 && s.verdict.b2.p[2] < P.tableL / 2 && Math.abs(s.verdict.b2.p[0]) < P.tableW / 2,
    relOK: Math.abs(s.launch.top) < 2 * RPS || Math.sign(s.an.relAlong) === Math.sign(s.launch.top),
  });
}
console.log(`生成 ${rows.length}/${n}，平均 ${(tsum / n).toFixed(0)} ms，最慢 ${tmax.toFixed(0)} ms`);
console.log('被筛掉的原因：', Object.entries(reasons).sort((a, b) => b[1] - a[1]).map((e) => e.join('×')).join('  '));
const all = (f) => rows.every(f), mx = (f) => rows.reduce((a, r) => Math.max(a, f(r)), 0);
ok(nulls.length <= n * 0.03, `生成不出来的组合 ≤ 3%（实际 ${nulls.length} 个${nulls.length ? '：' + nulls.join(', ') : ''}）`);
ok(all((r) => r.s.legal), '全部是合法发球（先落本方台面 → 过网 → 落对方台面）');
ok(all((r) => r.s.cls === r.intent), '到接球者面前的上下旋类别 = 想要的类别');
ok(all((r) => r.sideOK), '侧旋方向与发球类型、左右手一致');
ok(all((r) => r.lenOK && r.b1OK && r.b2OK), '落点：第一跳在本方台内，第二跳在对方台内且长短符合设置');
ok(mx((r) => r.fwdErr) < 1e-9, `正向复算一致（最大差 ${mx((r) => r.fwdErr).toExponential(1)}）`);
ok(mx((r) => r.kinErr) < 0.05, `画出来的挥拍在触球瞬间的速度 = 物理用的拍速（最大差 ${mx((r) => r.kinErr).toFixed(4)} m/s）`);
ok(mx((r) => r.faceErr) < 1e-6, `画出来的拍面在触球瞬间 = 物理用的拍面（最大差 ${mx((r) => r.faceErr).toExponential(1)}）`);
ok(all((r) => r.relOK), '"顺着拍面往上蹭 → 上旋，往下蹭 → 下旋" 对每一个球都成立');
ok(all((r) => r.s.col.regime === 'grip'), '触球都处在"咬住"状态（没有超出胶皮摩擦极限）');
ok(all((r) => r.s.verdict.net.clear >= 0.004), `过网余量 ≥ 4 mm（最小 ${(Math.min(...rows.map((r) => r.s.verdict.net.clear)) * 1000).toFixed(1)} mm）`);
ok(all((r) => r.s.body.stats.maxPen <= 0.02 && r.s.body.stats.tossPen <= 0.004 && r.s.body.stats.freePen <= 0.02 && r.s.clipArm <= 0.008 && r.s.clipBall <= 0.006), '身体、球拍、球之间的穿插都在容许范围内');

// 时间步长收敛：1 ms 对比 0.1 ms
let dLand = 0, dClear = 0, dSpin = 0, m = 0;
for (let i = 0; i < rows.length; i += 6) {
  const s = rows[i].s;
  const jf = judge(simulate(s.pc, s.col.v, s.col.w, { dt: 0.0001, tMax: 2.4, maxOppBounces: 2 }));
  if (!jf.legal) { ok(false, '细步长重算后发球不合法：' + jf.why); continue; }
  m++;
  dLand = Math.max(dLand, Math.hypot(jf.b2.p[0] - s.verdict.b2.p[0], jf.b2.p[2] - s.verdict.b2.p[2]));
  dClear = Math.max(dClear, Math.abs(jf.net.clear - s.verdict.net.clear));
  dSpin = Math.max(dSpin, Math.abs(spinParts(jf.b2.wOut, jf.b2.vOut).top - s.arrive.top) / RPS);
}
ok(dLand < 0.002 && dClear < 0.001 && dSpin < 0.2, `步长收敛（${m} 个球）：落点差 ≤ ${(dLand * 1000).toFixed(2)} mm，过网余量差 ≤ ${(dClear * 1000).toFixed(2)} mm，到位上下旋差 ≤ ${dSpin.toFixed(3)} 转/秒`);

const st = (a) => { a = [...a].sort((x, y) => x - y); return `${a[0].toFixed(1)} / ${a[a.length >> 1].toFixed(1)} / ${a[a.length - 1].toFixed(1)}`; };
console.log('\n各类别的量级（最小 / 中位 / 最大）');
for (const intent of ['top', 'back', 'flat']) {
  const R = rows.filter((r) => r.intent === intent).map((r) => r.s);
  console.log(`[${intent}] n=${R.length}  离拍总转速 ${st(R.map((s) => len(s.col.w) / RPS))}  离拍上下旋 ${st(R.map((s) => s.launch.top / RPS))}  到位上下旋 ${st(R.map((s) => s.arrive.top / RPS))} 转/秒`);
  console.log(`        拍面后仰 ${st(R.map((s) => s.an.openDeg))}°  拍速 ${st(R.map((s) => s.an.vrSpeed))}  球速 ${st(R.map((s) => s.an.ballSpeed))} m/s  过网余量 ${st(R.map((s) => s.verdict.net.clear * 100))} cm`);
}
console.log(fails ? `\n${fails} 项没过` : '\n全部通过');
