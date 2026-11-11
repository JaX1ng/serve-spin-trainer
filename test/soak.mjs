// 大样本体检：随机配置生成发球，用另一套独立写的几何检查逐帧过一遍——
// 球穿头 / 身体 / 手臂，手和肘进球台，两只手和球拍打架，球被自己挡住接球的人看不见，某一帧关节突然跳走
//   node test/soak.mjs [个数=1200] [随机种子]
import { generateServe, mulberry32, TYPE_KEYS, PLAYER_KEYS } from '../src/serve.js';
import { P, sub, len, dot, addmul } from '../src/physics.js';
const N = +(process.argv[2] || 1200);
const rng = mulberry32(+(process.argv[3] || 20261111));
const pick = (a) => a[Math.floor(rng() * a.length)];
const segDist = (p, a, b) => { const ab = sub(b, a), l2 = dot(ab, ab) || 1e-9; const t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2)); return len(sub(p, addmul(a, ab, t))); };
const EYE = [0, 1.36, 2.9]; // 接球的人的眼睛
const stat = {}; const add = (k, v = 1) => { stat[k] = (stat[k] || 0) + v; };
const worst = {}; const track = (k, v, info) => { if (!(k in worst) || v > worst[k].v) worst[k] = { v, info }; };
let nullCount = 0, tGen = 0, tMax = 0, attempts = 0;
const byType = {};
for (let i = 0; i < N; i++) {
  const cfg = { type: pick(TYPE_KEYS), hand: pick([1, -1]), intent: pick(['top', 'back', 'flat']), kind: pick(['side', 'pure']), pos: pick([-1, 0, 1]), len: pick(['short', 'long']), player: rng() < 0.3 ? undefined : pick(PLAYER_KEYS), fake: rng() < 0.4 };
  const t0 = performance.now();
  const sv = generateServe(cfg, rng);
  const dt = performance.now() - t0; tGen += dt; tMax = Math.max(tMax, dt);
  const key = cfg.type;
  byType[key] = byType[key] || { n: 0, nul: 0 };
  byType[key].n++;
  if (!sv) { nullCount++; byType[key].nul++; add('null:' + cfg.type + ':' + cfg.intent + ':' + cfg.kind + ':' + cfg.len); continue; }
  attempts += sv.attempts;
  if (sv.cls !== cfg.intent) add('BAD class');
  if (!sv.legal) add('BAD illegal');
  if (cfg.fake && !sv.fakeKind) add('BAD fake missing');
  const tag = `${cfg.type}/${cfg.intent}/${cfg.kind}/${cfg.len}/h${cfg.hand}/p${cfg.pos}/${sv.player}${sv.fakeKind ? '/' + sv.fakeKind : ''}`;
  // 逐时刻检查：触球前后密一些
  const ts = [];
  const tA = sv.tStart, tB = Math.min(sv.tEnd, sv.body.tDone + 0.05);
  for (let t = tA; t < -0.3; t += 0.02) ts.push(t);
  for (let t = Math.max(tA, -0.3); t < 0.25; t += 0.006) ts.push(t);
  for (let t = 0.25; t <= tB; t += 0.02) ts.push(t);
  let headBall = 1e9, torsoBall = 1e9, armBall = 1e9, legBall = 1e9, freeArmBall = 1e9, handTable = 1e9, freeTable = 1e9, elbowTable = 1e9, elbowF = 1e9, freeRk = 1e9, freeGrip = 1e9, rkHead = 1e9;
  let hidden = 0, hiddenLate = 0, jumpE = 0, prev = null, pt = 0;
  for (const t of ts) {
    const J = sv.body.at(t);
    for (const k of Object.keys(J)) { const v = J[k]; if (Array.isArray(v) && v.some((x) => !Number.isFinite(x))) add('BAD NaN joint ' + k); }
    const b = sv.ballAt(t).p;
    const handP = addmul(J.wrP, J.handP.d, 0.05), handF = addmul(J.wrF, J.handF.d, 0.05);
    // 球出手之后：别穿过头、身体、手臂、腿
    if (t > sv.tRel + 0.03) {
      headBall = Math.min(headBall, len(sub(b, J.head)) - 0.105 - P.R);
      torsoBall = Math.min(torsoBall, segDist(b, J.pelvis, J.mid) - 0.125 - P.R, segDist(b, J.mid, J.chest) - 0.135 - P.R);
      freeArmBall = Math.min(freeArmBall, segDist(b, J.shF, J.elF) - 0.045 - P.R, segDist(b, J.elF, J.wrF) - 0.037 - P.R);
      legBall = Math.min(legBall, segDist(b, J.hipP, J.knP) - 0.075 - P.R, segDist(b, J.hipF, J.knF) - 0.075 - P.R);
      if (Math.abs(t) > 0.012) armBall = Math.min(armBall, segDist(b, J.shP, J.elP) - 0.045 - P.R, segDist(b, J.elP, J.wrP) - 0.037 - P.R);
    }
    // 接球的人看不看得见球：眼睛到球的连线不能穿过发球人的头和上身
    if (t > sv.tRel + 0.05 && t <= 0.01) {
      const hid = segDist(J.head, EYE, b) < 0.105 - 0.004 || segDist(J.chest, EYE, b) < 0.135 - 0.01 || segDist(J.mid, EYE, b) < 0.125 - 0.01 || segDist(J.elF, EYE, b) < 0.03 || segDist(handF, EYE, b) < 0.03;
      if (hid) { hidden += t - pt; if (t > -0.1) hiddenLate++; }
    }
    // 身体各处别进球台。扎进去多深 = "比台面低多少"和"离台边往里多远"里小的那个（台边外面的手肘低于台面是正常的）
    const inTable = (p, r) => Math.min(P.tableY - (p[1] - r), P.tableW / 2 + r - Math.abs(p[0]), p[2] + P.tableL / 2 + r);
    handTable = Math.min(handTable, -inTable(handP, 0.04));
    if (t > sv.tRel + 0.03) freeTable = Math.min(freeTable, -inTable(J.wrF, 0.036));
    elbowTable = Math.min(elbowTable, -inTable(J.elP, 0.042));
    elbowF = Math.min(elbowF, -inTable(J.elF, 0.042));
    // 抛球的手和球拍、握拍的手别打架（球拍按薄圆片算）；球拍别扣在脸上
    { const rl = sub(handF, J.racket.c), dn = dot(rl, J.racket.Z), rho = len(sub(rl, addmul([0, 0, 0], J.racket.Z, dn)));
      if (rho < 0.076 + 0.02) freeRk = Math.min(freeRk, Math.abs(dn) - 0.04);
      freeGrip = Math.min(freeGrip, len(sub(handF, handP)) - 0.08);
      const rh = sub(J.head, J.racket.c), hn = dot(rh, J.racket.Z), hr = len(sub(rh, addmul([0, 0, 0], J.racket.Z, hn)));
      if (hr < 0.076) rkHead = Math.min(rkHead, Math.abs(hn) - 0.105); }
    if (prev && t - pt < 0.0061) jumpE = Math.max(jumpE, len(sub(J.elF, prev.elF)) / (t - pt), len(sub(J.elP, prev.elP)) / (t - pt), len(sub(J.wrF, prev.wrF)) / (t - pt), len(sub(J.head, prev.head)) / (t - pt) * 3);
    prev = J; pt = t;
  }
  const chk = (name, v, lim) => { if (v < lim) { add('clip ' + name); track(name, -v, tag); } };
  chk('ball-head', headBall, -0.013); chk('ball-torso', torsoBall, -0.013); chk('ball-playarm', armBall, -0.013); chk('ball-freearm', freeArmBall, -0.013); chk('ball-leg', legBall, -0.013);
  chk('hand-table', handTable, -0.016); chk('freehand-table', freeTable, -0.016); chk('elbow-table', elbowTable, -0.016); chk('free-elbow-table', elbowF, -0.016);
  chk('freehand-blade', freeRk, -0.036); chk('freehand-grip', freeGrip, -0.04); chk('racket-head', rkHead, -0.031);
  if (hiddenLate > 0) { add('clip ball-hidden-at-contact'); track('ball-hidden-at-contact', hiddenLate, tag); }
  if (hidden > 0.06) { add('clip ball-hidden-in-toss'); track('ball-hidden-in-toss', hidden, tag); }
  if (jumpE > 16) { add('BAD joint jump'); track('joint-jump m/s', jumpE, tag); }
  track('fastest elbow / free hand (m/s)', jumpE, tag);
  track('attempts', sv.attempts, tag);
}
console.log(`N=${N} null=${nullCount} avg ${(tGen / N).toFixed(1)} ms, max ${tMax.toFixed(0)} ms, 平均试 ${(attempts / Math.max(1, N - nullCount)).toFixed(2)} 次出一个`);
console.log('by type', JSON.stringify(byType));
for (const k of Object.keys(stat).sort()) console.log(String(stat[k]).padStart(5), k);
console.log('最严重的一例:'); for (const k of Object.keys(worst)) console.log('  ', k, worst[k].v.toFixed(3), worst[k].info);
const clips = Object.keys(stat).filter((k) => k.startsWith('clip ')).reduce((a, k) => a + stat[k], 0);
const bad = Object.keys(stat).filter((k) => k.startsWith('BAD')).reduce((a, k) => a + stat[k], 0);
const okAll = bad === 0 && clips <= N * 0.005 && nullCount <= N * 0.03;
console.log(okAll ? `PASS 穿模 / 遮挡 ${clips} 例（容许 ≤ ${(N * 0.005).toFixed(0)}），硬错误 0，生成失败 ${nullCount} 例` : `FAIL 穿模 / 遮挡 ${clips} 例，硬错误 ${bad} 例，生成失败 ${nullCount} 例`);
if (!okAll) process.exitCode = 1;
