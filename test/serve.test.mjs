// 发球生成器的回归测试：每种 类型 × 左右手 × 站位 × 长短 × 意图 ×（带侧旋 / 不带侧旋）各生成一个，逐项核对
import { P, collide, simulate, len, sub, dot } from '../src/physics.js';
import { generateServe, mulberry32, TYPE_KEYS, PLAYER_KEYS, PLAYERS, TYPES, RPS, TH_RPS, judge, spinParts } from '../src/serve.js';
import { LIM, GRIPS, DEG } from '../src/skeleton.js';

const rng = mulberry32(12345);
let fails = 0;
const ok = (c, msg) => { console.log((c ? 'PASS ' : 'FAIL ') + msg); if (!c) { fails++; process.exitCode = 1; } };
const rows = [], nulls = [], reasons = {};
let n = 0, tsum = 0, tmax = 0;
for (const type of TYPE_KEYS) for (const hand of [1, -1]) for (const pos of [-1, 0, 1]) for (const len_ of ['short', 'long']) for (const intent of ['top', 'back', 'flat']) for (const kind of ['side', 'pure']) {
  const dbg = [];
  const player = PLAYER_KEYS[n % PLAYER_KEYS.length];
  const t0 = performance.now();
  const s = generateServe({ type, hand, intent, kind, pos, len: len_, player, fake: n % 2 === 0, _dbg: dbg }, rng);
  const dt = performance.now() - t0; tsum += dt; tmax = Math.max(tmax, dt); n++;
  for (const r of dbg) { const k = `${type}:${r.split(':').slice(0, r.startsWith('motion') ? 2 : 1).join(':')}`; reasons[k] = (reasons[k] || 0) + 1; }
  const tag = `${type}/${intent}/${kind}/${len_}/h${hand}/p${pos}/${player}`;
  if (!s) { nulls.push(tag); continue; }
  // 正向复算：同样的拍面 + 拍速再碰一次，出手状态必须一模一样
  const c = collide(s.toss.vb, [0, 0, 0], s.n, s.vr, P.rubber);
  // 画出来的人：触球那一瞬间，拍面上碰球的那一点的位置、速度（差分），和拍面朝向，必须等于物理用的那组
  const m = s.body, e = 2e-5, ka = m.contactAt(-e), kb = m.contactAt(e), k0 = m.contactAt(0);
  const vfd = [(kb[0] - ka[0]) / (2 * e), (kb[1] - ka[1]) / (2 * e), (kb[2] - ka[2]) / (2 * e)];
  const J0 = m.at(0);
  // 整个动作：关节不出活动范围、不跳变，脚不挪
  const g = GRIPS[m.grip];
  let limBad = 0, jump = 0, vRk = 0, vBody = 0, vArm = 0, feet = 0, prev = null;
  const inR = (x, r) => x >= r[0] - 1e-6 && x <= r[1] + 1e-6;
  const tA = s.tStart, tB = Math.min(s.tEnd, m.tDone + 0.1), hs = 0.004;
  for (let t = tA; t <= tB; t += hs) {
    const J = m.at(t), q = J.q;
    if (!(inR(q[4], LIM.lean) && inR(q[5], LIM.curl) && inR(q[6], LIM.side) && inR(q[7], LIM.twist) && inR(q[10], LIM.rot) && inR(q[11], LIM.elb) && inR(q[12], g.pro) && inR(q[13], LIM.wf) && inR(q[14], g.wd) && Math.hypot(q[8], q[9]) <= LIM.elev[1] + 1e-6)) limBad++;
    if (prev) {
      for (let i = 3; i < q.length; i++) jump = Math.max(jump, Math.abs(q[i] - prev.q[i]) / hs);
      vRk = Math.max(vRk, len(sub(J.racket.c, prev.racket.c)) / hs);
      vBody = Math.max(vBody, len(sub(J.head, prev.head)) / hs, len(sub(J.pelvis, prev.pelvis)) / hs);
      vArm = Math.max(vArm, len(sub(J.wrF, prev.wrF)) / hs, len(sub(J.elF, prev.elF)) / hs, len(sub(J.elP, prev.elP)) / hs);
      feet = Math.max(feet, len(sub(J.anP, prev.anP)), len(sub(J.anF, prev.anF)));
    }
    prev = J;
  }
  const st = m.stats;
  const headEl = Math.asin(Math.max(-1, Math.min(1, s.an.head[1]))) / DEG;
  rows.push({
    tag, type, intent, kind, len: len_, hand, s,
    fwdErr: len(sub(c.v, s.col.v)) + len(sub(c.w, s.col.w)) * P.R,
    posErr: len(sub(k0, [s.pc[0] - s.n[0] * P.R, s.pc[1] - s.n[1] * P.R, s.pc[2] - s.n[2] * P.R])),
    kinErr: len(sub(vfd, s.vr)), faceErr: 1 - dot(J0.racket.n, s.n),
    sideOK: kind === 'side' ? Math.sign(s.launch.side) === TYPES[type].side * hand : Math.abs(s.launch.side) / RPS <= Math.max(6, 0.3 * len(s.col.w) / RPS),
    lenOK: len_ === 'short' ? s.verdict.b2.p[2] < 0.6 : s.verdict.b2.p[2] > 0.8,
    b1OK: s.verdict.b1.p[2] < 0 && Math.abs(s.verdict.b1.p[0]) < P.tableW / 2, b2OK: s.verdict.b2.p[2] > 0 && s.verdict.b2.p[2] < P.tableL / 2 && Math.abs(s.verdict.b2.p[0]) < P.tableW / 2,
    relOK: Math.abs(s.launch.top) < 2 * RPS || Math.sign(s.an.relAlong) === Math.sign(s.launch.top),
    timeOK: s.tStart < s.tPalm0 && s.tPalm0 < s.tRel && s.tRel < s.tSwing0 && s.tSwing0 < 0 && s.tSwingEnd > 0 && s.tEnd > s.tSwingEnd && s.arrive.t < s.tEnd,
    penOK: !st.nan && st.body <= 0.03 && st.table <= 0.015 && st.ballBody <= 0.012 && st.free <= 0.035 && st.early <= 0.006 && st.late <= 0.008 && st.leg <= 0.012,
    seeOK: st.hide <= -0.012 && st.hideT <= 0.045,
    limBad, jump, vRk, vBody, vArm, feet,
    headOK: Math.abs(s.n[1]) > 0.9 || (type === 'pendulum' || type === 'reverse' ? headEl < 30 : type === 'hook' ? headEl > -30 : true),
    deadOK: !s.dead || (kind === 'pure' ? len(s.col.w) : Math.abs(s.launch.top)) / RPS < 10,
    metaOK: s.player === player && s.kind === kind && PLAYERS[s.player] && (!s.fake || !!s.fakeKind),
  });
}
console.log(`生成 ${rows.length}/${n}，平均 ${(tsum / n).toFixed(0)} ms，最慢 ${tmax.toFixed(0)} ms`);
console.log('被筛掉重来的原因：', Object.entries(reasons).sort((a, b) => b[1] - a[1]).slice(0, 24).map((e) => e.join('×')).join('  '));
const all = (f) => rows.every(f), mx = (f) => rows.reduce((a, r) => Math.max(a, f(r)), 0);
const bad = (f) => { const b = rows.filter((r) => !f(r)); return b.length ? `（没过的 ${b.length} 个，如 ${b.slice(0, 3).map((r) => r.tag).join('、')}）` : ''; };
ok(nulls.length <= n * 0.03, `生成不出来的组合 ≤ 3%（实际 ${nulls.length} 个${nulls.length ? '：' + nulls.slice(0, 6).join(', ') : ''}）`);
ok(all((r) => r.s.legal), '全部是合法发球（先落本方台面 → 过网 → 落对方台面）');
ok(all((r) => r.s.cls === r.intent), '到接球者面前的上下旋类别 = 想要的类别（上旋 / 下旋 / 不转）');
ok(all((r) => r.sideOK), '带侧旋的：侧旋方向与发球类型、左右手一致；不带侧旋的：侧旋分量确实很小' + bad((r) => r.sideOK));
ok(all((r) => r.deadOK), '"顶出来的球"离拍时几乎不带上下旋（不带侧旋的：总转速 < 10 转/秒；带侧旋的：上下旋分量 < 10）' + bad((r) => r.deadOK));
ok(all((r) => r.lenOK && r.b1OK && r.b2OK), '落点：第一跳在本方台内，第二跳在对方台内且长短符合设置');
ok(mx((r) => r.fwdErr) < 1e-9, `正向复算一致（最大差 ${mx((r) => r.fwdErr).toExponential(1)}）`);
ok(mx((r) => r.posErr) < 1e-6, `触球瞬间，画出来的拍面正好贴在球上（最大差 ${mx((r) => r.posErr).toExponential(1)} m）`);
ok(mx((r) => r.kinErr) < 0.05, `触球瞬间，画出来的人手里拍面触球点的速度 = 物理用的拍速（最大差 ${mx((r) => r.kinErr).toFixed(4)} m/s）`);
ok(mx((r) => r.faceErr) < 1e-6, `触球瞬间，画出来的拍面朝向 = 物理用的拍面（最大差 ${mx((r) => r.faceErr).toExponential(1)}）`);
ok(all((r) => r.relOK), '"顺着拍面往上蹭 → 上旋，往下蹭 → 下旋" 对每一个球都成立');
ok(all((r) => r.s.col.regime === 'grip'), '触球都处在"咬住"状态（没有超出胶皮摩擦极限）');
ok(all((r) => r.s.verdict.net.clear >= 0.004), `过网余量 ≥ 4 mm（最小 ${(Math.min(...rows.map((r) => r.s.verdict.net.clear)) * 1000).toFixed(1)} mm）`);
ok(all((r) => r.timeOK), '时间线顺序：托球 → 抛起 → 引拍挥拍 → 触球 → 收拍 → 球到位' + bad((r) => r.timeOK));
ok(all((r) => r.limBad === 0), '整个动作里每个关节都在人的活动范围内' + bad((r) => r.limBad === 0));
ok(mx((r) => r.jump) < 70 && mx((r) => r.vRk) < 30 && mx((r) => r.vBody) < 3 && mx((r) => r.vArm) < 14, `动作连续、没有瞬移（关节最快 ${mx((r) => r.jump).toFixed(0)} 弧度/秒，球拍最快 ${mx((r) => r.vRk).toFixed(1)} m/s，头和骨盆最快 ${mx((r) => r.vBody).toFixed(1)} m/s，手肘和抛球手最快 ${mx((r) => r.vArm).toFixed(1)} m/s）`);
ok(mx((r) => r.feet) < 1e-9, '发球过程中两只脚不挪地方');
ok(all((r) => r.penOK), '身体、球拍、球、球台之间的穿插都在容许范围内' + bad((r) => r.penOK));
ok(all((r) => r.seeOK), '从抛起到触球，接球的人一直看得见球（没被发球人自己的头、身体、手臂挡住）' + bad((r) => r.seeOK));
ok(all((r) => r.headOK), '拍头指向符合发球类型：钟摆、逆旋转大体朝下，勾手朝上' + bad((r) => r.headOK));
ok(all((r) => r.metaOK), '选手、带不带侧旋、假动作的标记都对');
{ // 同一种发球，不同选手的姿势确实不一样：骨盆高度、前倾、抛球高度按选手拉开
  const by = {};
  for (const r of rows) { const a = by[r.s.player] || (by[r.s.player] = { py: [], lean: [], rise: [] }); const q = r.s.body.pose.q; a.py.push(q[1]); a.lean.push(q[4] / DEG); a.rise.push(r.s.toss.apex - r.s.toss.yRel); }
  const mean = (a) => a.reduce((x, y) => x + y, 0) / a.length;
  const M = Object.fromEntries(Object.entries(by).map(([k, a]) => [k, { py: mean(a.py), lean: mean(a.lean), rise: mean(a.rise) }]));
  console.log('各选手平均：' + Object.entries(M).map(([k, v]) => `${PLAYERS[k].name} 骨盆 ${(v.py * 100).toFixed(0)}cm 前倾 ${v.lean.toFixed(0)}° 抛高 ${(v.rise * 100).toFixed(0)}cm`).join('；'));
  ok(M.crouch.py < M.quick.py - 0.04 && M.crouch.lean > M.quick.lean + 6 && M.high.rise > 3 * M.quick.rise, '选手之间拉得开：低蹲的比站直的蹲得低、弯得深，高抛的抛球高出好几倍');
}

// 时间步长收敛：1 ms 对比 0.1 ms
let dLand = 0, dClear = 0, dSpin = 0, m = 0;
for (let i = 0; i < rows.length; i += 9) {
  const s = rows[i].s;
  const jf = judge(simulate(s.pc, s.col.v, s.col.w, { dt: 0.0001, tMax: 2.4, maxOppBounces: 2 }));
  if (!jf.legal) { ok(false, '细步长重算后发球不合法：' + jf.why); continue; }
  m++;
  dLand = Math.max(dLand, Math.hypot(jf.b2.p[0] - s.verdict.b2.p[0], jf.b2.p[2] - s.verdict.b2.p[2]));
  dClear = Math.max(dClear, Math.abs(jf.net.clear - s.verdict.net.clear));
  dSpin = Math.max(dSpin, Math.abs(spinParts(jf.b2.wOut, jf.b2.vOut).top - s.arrive.top) / RPS);
}
ok(dLand < 0.002 && dClear < 0.001 && dSpin < 0.2, `步长收敛（${m} 个球）：落点差 ≤ ${(dLand * 1000).toFixed(2)} mm，过网余量差 ≤ ${(dClear * 1000).toFixed(2)} mm，到位上下旋差 ≤ ${dSpin.toFixed(3)} 转/秒`);

const st = (a) => { a = [...a].sort((x, y) => x - y); return a.length ? `${a[0].toFixed(1)} / ${a[a.length >> 1].toFixed(1)} / ${a[a.length - 1].toFixed(1)}` : '—'; };
console.log(`\n各类别的量级（最小 / 中位 / 最大）。"不转"的界线：到位上下旋 < ${TH_RPS} 转/秒`);
for (const kind of ['side', 'pure']) for (const intent of ['top', 'back', 'flat']) {
  const R = rows.filter((r) => r.intent === intent && r.kind === kind).map((r) => r.s);
  console.log(`[${kind === 'side' ? '带侧旋' : '不带侧旋'} ${intent}] n=${R.length}  离拍总转速 ${st(R.map((s) => len(s.col.w) / RPS))}  离拍上下旋 ${st(R.map((s) => s.launch.top / RPS))}  离拍侧旋 ${st(R.map((s) => Math.abs(s.launch.side) / RPS))}  到位上下旋 ${st(R.map((s) => s.arrive.top / RPS))} 转/秒`);
  console.log(`        拍面后仰 ${st(R.map((s) => s.an.openDeg))}°  拍速 ${st(R.map((s) => s.an.vrSpeed))}  球速 ${st(R.map((s) => s.an.ballSpeed))} m/s  过网余量 ${st(R.map((s) => s.verdict.net.clear * 100))} cm`);
}
console.log(fails ? `\n${fails} 项没过` : '\n全部通过');
