// 大样本体检：随机配置生成发球，逐时刻检查穿模（球穿头/身体/手臂，手和肘进球台，两只手和球拍打架）
//   node test/soak.mjs [个数=1200] [随机种子]
import { generateServe, mulberry32, TYPE_KEYS, STYLE_KEYS, RPS } from '../src/serve.js';
import { P, sub, len, dot, addmul } from '../src/physics.js';
const N = +(process.argv[2] || 1200);
const rng = mulberry32(+(process.argv[3] || 20261111));
const pick = (a) => a[Math.floor(rng() * a.length)];
const segDist = (p, a, b) => { const ab = sub(b, a), l2 = dot(ab, ab) || 1e-9; const t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2)); return len(sub(p, addmul(a, ab, t))); };
const stat = {}; const add = (k, v = 1) => { stat[k] = (stat[k] || 0) + v; };
const worst = {}; const track = (k, v, info) => { if (!(k in worst) || v > worst[k].v) worst[k] = { v, info }; };
let nullCount = 0, tGen = 0, tMax = 0;
const byType = {};
for (let i = 0; i < N; i++) {
  const cfg = { type: pick(TYPE_KEYS), hand: pick([1, -1]), intent: pick(['top', 'back', 'flat']), pos: pick([-1, 0, 1]), len: pick(['short', 'long']), style: pick(STYLE_KEYS), fake: rng() < 0.4 };
  const t0 = performance.now();
  const sv = generateServe(cfg, rng);
  const dt = performance.now() - t0; tGen += dt; tMax = Math.max(tMax, dt);
  const key = cfg.type;
  byType[key] = byType[key] || { n: 0, nul: 0 };
  byType[key].n++;
  if (!sv) { nullCount++; byType[key].nul++; add('null:' + cfg.type + ':' + cfg.intent + ':' + cfg.len); continue; }
  if (sv.cls !== cfg.intent) add('BAD class');
  if (!sv.legal) add('BAD illegal');
  const tag = `${cfg.type}/${cfg.intent}/${cfg.len}/h${cfg.hand}/p${cfg.pos}/${sv.style}${cfg.fake ? '/fake' : ''}`;
  // 逐时刻检查
  const ts = [];
  for (let k = 0; k <= 60; k++) ts.push(sv.tStart + ((Math.min(sv.tEnd, 0.5) - sv.tStart) * k) / 60);
  let elbowF = 1e9, elbowFt = 0, freeGrip = 1e9; let headBall = 1e9, torsoBall = 1e9, armBall = 1e9, legBall = 1e9, freeArmBallAfter = 1e9, rkTable = 1e9, handTable = 1e9, freeTable = 1e9, elbowTable = 1e9, freeRk = 1e9;
  for (const t of ts) {
    const J = sv.body.at(t);
    for (const k of Object.keys(J)) { const v = J[k]; if (Array.isArray(v) && v.some((x) => !Number.isFinite(x))) add('BAD NaN joint ' + k); }
    const b = sv.ballAt(t).p;
    const dHead = len(sub(b, J.head)) - 0.115 - P.R;
    const dTorso = segDist(b, J.pelvis, J.chest) - 0.14 - P.R;
    // 球出手之后：别从持拍手臂、非持拍手臂、腿里穿过去
    if (t > sv.tRel + 0.03) { headBall = Math.min(headBall, dHead); torsoBall = Math.min(torsoBall, dTorso); }
    if (t > 0.004) {
      armBall = Math.min(armBall, segDist(b, J.shP, J.elP) - 0.045 - P.R, segDist(b, J.elP, J.wrP) - 0.038 - P.R);
      freeArmBallAfter = Math.min(freeArmBallAfter, segDist(b, J.shF, J.elF) - 0.045 - P.R, segDist(b, J.elF, J.wrF) - 0.04 - P.R);
      legBall = Math.min(legBall, segDist(b, J.hipP, J.knP) - 0.07 - P.R, segDist(b, J.hipF, J.knF) - 0.07 - P.R);
    }
    // 身体各处别进球台
    const overTable = (p, m = 0) => Math.abs(p[0]) < P.tableW / 2 + m && p[2] > -P.tableL / 2 - m;
    const hp = addmul(J.racket.W, J.racket.l, 0.035);
    if (overTable(hp, 0.04)) handTable = Math.min(handTable, hp[1] - 0.04 - P.tableY);
    if (overTable(J.wrF, 0.04)) freeTable = Math.min(freeTable, J.wrF[1] - 0.012 - P.tableY);
    if (overTable(J.elP, 0.04)) elbowTable = Math.min(elbowTable, J.elP[1] - 0.042 - P.tableY);
    if (overTable(J.elF, 0.04)) { const v = J.elF[1] - 0.042 - P.tableY; if (v < elbowF) { elbowF = v; elbowFt = t; } }
    // 非持拍手和球拍别打架（球拍按薄圆片算）
    { const rl = sub(J.wrF, J.racket.c), dn = dot(rl, J.racket.n), rho = len(sub(rl, addmul([0,0,0], J.racket.n, dn)));
      if (rho < 0.076 + 0.03) freeRk = Math.min(freeRk, Math.abs(dn) - 0.03);
      freeGrip = Math.min(freeGrip, len(sub(J.wrF, addmul(J.racket.W, J.racket.l, 0.035))) - 0.08); }
  }
  const chk = (name, v, lim) => { if (v < lim) { add('clip ' + name); track(name, -v, tag); } };
  chk('ball-head', headBall, -0.008); chk('ball-torso', torsoBall, -0.008); chk('ball-playarm', armBall, -0.006); chk('ball-freearm(after hit)', freeArmBallAfter, -0.006); chk('ball-leg', legBall, -0.006);
  chk('hand-table', handTable, -0.006); chk('freehand-table', freeTable, -0.006); chk('elbow-table', elbowTable, -0.006); chk('free-elbow-table', elbowF, -0.006); chk('freehand-blade', freeRk, -0.006); chk('freehand-grip', freeGrip, -0.01);
  if (elbowF < -0.006) add(`  free elbow dip when: ${elbowFt < sv.tRel ? 'holding' : elbowFt < sv.tRel + 0.45 ? 'retreat' : 'rest'}`);
  if (sv.body.stats.maxPen > 0.02) add('BAD pen>2cm'); if (sv.body.stats.reach > 0.02) add('BAD reach>2cm');
  track('attempts', sv.attempts, tag);
}
console.log(`N=${N} null=${nullCount} avg ${(tGen / N).toFixed(1)} ms, max ${tMax.toFixed(0)} ms`);
console.log('by type', JSON.stringify(byType));
for (const k of Object.keys(stat).sort()) console.log(String(stat[k]).padStart(5), k);
console.log('最严重的一例:'); for (const k of Object.keys(worst)) console.log('  ', k, worst[k].v.toFixed(3), worst[k].info);
const clips = Object.keys(stat).filter((k) => k.startsWith('clip ')).reduce((a, k) => a + stat[k], 0);
const bad = Object.keys(stat).filter((k) => k.startsWith('BAD')).reduce((a, k) => a + stat[k], 0);
const okAll = bad === 0 && clips <= N * 0.005 && nullCount <= N * 0.03;
console.log(okAll ? `PASS 穿模 ${clips} 例（容许 ≤ ${(N * 0.005).toFixed(0)}），生成失败 ${nullCount} 例` : `FAIL 穿模 ${clips} 例，硬错误 ${bad} 例，生成失败 ${nullCount} 例`);
if (!okAll) process.exitCode = 1;
