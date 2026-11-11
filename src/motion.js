// motion.js — 发球的人怎么动。
// 物理只规定了触球那一瞬间的三样东西：球在哪儿、拍面朝哪儿、拍上触球的那一点速度是多少。
// 这里要回答的是：一个真人怎么站、怎么弯腰、手臂各个关节转到多少度，才能在那一瞬间正好做到这三样；
// 以及他是怎么从准备姿势引拍、挥到这儿，之后又怎么收拍（或者做假动作）的。
//
// 做法：
//  1. 触球姿势：在人体关节活动范围内搜索（朝向、前倾、转体、肩、肘、手腕侧偏），拍面朝向由前臂旋转和屈腕精确满足，
//     站位由"触球点必须在球那儿"精确解出。代价 = 关节别扭程度 + 穿模 + 离这位选手的习惯有多远 + 发力费不费劲。
//  2. 触球速度：把要求的拍速分摊到各个关节的转速上（加权最小范数），手腕、前臂、肘、肩、转体、重心移动都参与。
//  3. 整个动作：每个关节各自从"引拍到头"加速到触球时的转速，再减速到收拍。远端关节（手腕）起动得晚，这就是鞭打。
//     因为是在关节角度上插值的，中间每一帧都是人做得出来的姿势。
import { P, sub, dot, cross, len, norm, lerp, addmul } from './physics.js';
import { qFromBasis, smooth } from './quat.js';
import { DEG, DIM, NQ, LIM, GRIPS, fkCore, trunkLite, bladePoint, shiftState, dirsToAngles, ik2, closestOnSeg, distSeg, handPoints, pinchOf } from './skeleton.js';

const clamp = (x, a, b) => (x < a ? a : x > b ? b : x);
const sq = (x) => x * x;
const lim2 = (x, r) => (x < r[0] ? sq(r[0] - x) : x > r[1] ? sq(x - r[1]) : 0);
// 身体自己的"前、上、右" → 世界坐标
const H2W = (F, R, v) => [v[0] * F[0] + v[2] * R[0], v[1], v[0] * F[2] + v[2] * R[2]];
const mirror = (v) => [-v[0], v[1], v[2]];
// 接球的人眼睛大概在哪儿（和 main.js 里接球视角的机位一致）。规则要求发球全程让接球的人看得见球，所以姿势不能把这条视线挡住
const EYE = [0, 1.36, 2.9];
// 从接球者眼睛看过去，b 处的球被 c 处半径 r 的东西挡住多深（> 0 = 球心被挡住了）
const hideDepth = (b, c, r) => r - distSeg(c, EYE, b);

// ---------- 四种发球各自的身体模板（右手持拍；方向用身体自己的 [前, 上, 右]） ----------
// yaw：站姿朝向（-90 = 侧身，胸口朝自己右手边；0 = 正对球台）；headDir：拍头大体朝下(-1)还是朝上(+1)——这是这种发球的样子，不影响旋转
// ua / fa：触球时上臂、前臂大致指的方向；wd：手腕侧偏的习惯（负 = 拍头往小指一侧压）
// feet：两脚离骨盆中心多远（wide）、重心偏向哪只脚（shift）、前后错开多少
// bs：引拍时在"把挥拍倒放回去"的基础上再加的造型（转体、抬肘、扣腕……）
// ready：准备姿势手臂的角度，at = 球拍相对托球手的位置
// zone：球抛在身体的哪儿——抛球那会儿（上身还比较直）球相对胸口往前多远、偏右多少。球得抛在身体前面，托球的手才伸得过去；
//   （接球的人看不看得见球另外单独算：见 hideDepth）
// rest：抛完球之后那只手收到哪儿（几种里随选手习惯挑）
export const MOVES = {
  // 钟摆：侧身、弯腰，肘在身体外侧架起来，小臂吊着，拍头朝下，从外往肚子方向摆
  pendulum: {
    stance: 'side', yaw: -90, grip: 'serve', headDir: -1,
    lean: 52, curl: 10, side: 0, twist: -12,
    ua: [-0.62, 0.30, 0.72], fa: [0.25, -0.93, -0.27], wd: -25,
    feet: { wide: 0.35, shift: 0.05, fL: 0.07, fR: -0.04, toeL: 28, toeR: 10 },
    bs: { twist: -8, lean: -5, py: 0.03, wf: -10, wd: 14 }, bsD: 0.30, bsLift: 0.22, bsOff: [-0.04, 0, 0.05], fwD: 0.24,
    ready: { sF: 24, sR: 12, rot: 36, elb: 98, pro: 6, wf: -6, wd: 0, at: [-0.11, 0.07, 0.15] },
    zone: { f: [0.22, 0.50], r: [-0.28, 0.24] },
  },
  // 逆旋转：侧身，肘抬得更高、更靠前，手腕往里扣，拍头朝下，从肚子前面往外甩
  reverse: {
    stance: 'side', yaw: -90, grip: 'serve', headDir: -1,
    lean: 50, curl: 10, side: 0, twist: -4,
    ua: [0.30, 0.35, 0.89], fa: [-0.05, -0.95, -0.30], wd: -10,
    feet: { wide: 0.35, shift: 0.04, fL: 0.07, fR: -0.04, toeL: 28, toeR: 10 },
    bs: { twist: 5, lean: -4, py: 0.03, wf: 14, wd: -14 }, bsD: 0.20, bsLift: 0.16, fwD: 0.26,
    ready: { sF: 24, sR: 12, rot: 36, elb: 98, pro: 6, wf: -6, wd: 0, at: [-0.11, 0.07, 0.15] },
    zone: { f: [0.22, 0.50], r: [-0.28, 0.24] },
  },
  // 勾手：侧身，拍头朝上收在胸前，手腕内勾，再从里往外、往下勾出去
  hook: {
    stance: 'side', yaw: -90, grip: 'shake', headDir: 1,
    lean: 46, curl: 8, side: 0, twist: -6,
    ua: [0.12, -0.42, 0.90], fa: [0.60, 0.32, -0.72], wd: 6,
    feet: { wide: 0.35, shift: 0.03, fL: 0.07, fR: -0.04, toeL: 28, toeR: 10 },
    bs: { twist: -5, lean: -4, py: 0.02, wf: 20 }, bsD: 0.20, bsLift: 0.24, fwD: 0.24,
    // 勾手的引拍是固定的样子：拍子收在胸前，拍头朝上，手腕内勾，正手面朝着自己
    bsPose: { at: [-0.22, 0.20, 0.05], head: [0.15, 0.95, 0.1], face: [-0.92, 0.25, 0.1] },
    ready: { sF: 24, sR: 12, rot: 36, elb: 98, pro: 6, wf: -6, wd: 0, at: [-0.11, 0.07, 0.15] },
    zone: { f: [0.22, 0.50], r: [-0.28, 0.24] },
  },
  // 反手：正对球台，拍子从左腹前往右前方送，用反手面
  backhand: {
    stance: 'front', yaw: 24, grip: 'shake',
    lean: 40, curl: 8, side: 0, twist: 14,
    ua: [0.50, -0.82, -0.02], fa: [0.30, -0.20, -0.93], wd: -4,
    feet: { wide: 0.33, shift: 0, fL: -0.02, fR: 0.06, toeL: 16, toeR: 12 },
    bs: { twist: 12, lean: -3, py: 0.02, wf: 12 }, bsD: 0.26, bsLift: 0.05, fwD: 0.28,
    ready: { sF: 30, sR: 4, rot: 50, elb: 92, pro: 30, wf: 0, wd: 0, at: [-0.12, 0.06, 0.12] },
    zone: { f: [0.25, 0.55], r: [-0.30, 0.10] },
  },
};

// 抛完球那只手去哪儿（身体坐标 [前, 上, 右]；rel = 相对同侧肩膀 sh 还是骨盆 pel）
const REST = {
  wing: { rel: 'sh', at: [-0.41, -0.15, -0.16] },   // 往身后甩开，手臂略弯，比肩低一点（弯着腰时很自然，像滑冰的摆臂）
  back: { rel: 'sh', at: [-0.40, -0.25, -0.09] },   // 往身后收，低一些
  hip: { rel: 'pel', at: [-0.02, 0.03, -0.27] },    // 收到胯边
  tuck: { rel: 'pel', at: [0.17, 0.12, -0.10] },    // 收到小腹前
  low: { rel: 'sh', at: [-0.06, -0.44, -0.12] },    // 垂在身侧
};
// 抛球那只手的肘朝哪：手臂朝左前方略低处伸着（ARM_B）时，肘朝下、略朝外朝后（ARM_E）；手臂伸向别处时，把这个朝向顺着"最省事的转法"带过去。
// 这样手臂不管怎么挥，肘的朝向都是连着变的，不会在某一帧突然甩到另一边。
// 唯一定不下来的方向是 ARM_B 的正反面——左臂穿过自己的胸口朝右后方伸，人做不到；托球、抛球、收手用到的方向离它都很远
const ARM_B = [0.55, -0.25, -0.8], ARM_E = [-0.35, -0.85, -0.4];

// 各关节在一次发球里大致能转多快（弧度/秒；骨盆平移是 m/s）。用来把拍速分摊到关节上
const SPEED = { px: 0.5, py: 0.35, pz: 0.5, lean: 1.5, twist: 3, sF: 7, sR: 7, rot: 14, elb: 17, pro: 15, wf: 24, wd: 22 };
const RATE_IDX = [0, 1, 2, 4, 7, 8, 9, 10, 11, 12, 13, 14];
// 触球瞬间必须严格待在活动范围里面的关节（前臂旋转 12、手腕侧偏 14 的范围随握法，另查）
const HARD = [[4, LIM.lean], [5, LIM.curl], [6, LIM.side], [7, LIM.twist], [10, LIM.rot], [11, LIM.elb], [12, null], [13, LIM.wf], [14, null]];
const RATE_KEY = ['px', 'py', 'pz', 'lean', 'twist', 'sF', 'sR', 'rot', 'elb', 'pro', 'wf', 'wd'];

function solve3(A, b) {
  const [a, bb, c] = A[0], [d, e, f] = A[1], [g, h, i] = A[2];
  const det = a * (e * i - f * h) - bb * (d * i - f * g) + c * (d * h - e * g);
  if (Math.abs(det) < 1e-14) return null;
  return [
    (b[0] * (e * i - f * h) - bb * (b[1] * i - f * b[2]) + c * (b[1] * h - e * b[2])) / det,
    (a * (b[1] * i - f * b[2]) - b[0] * (d * i - f * g) + c * (d * b[2] - b[1] * g)) / det,
    (a * (e * b[2] - b[1] * h) - bb * (d * b[2] - b[1] * g) + b[0] * (d * h - e * g)) / det,
  ];
}

/** 把要求的触球点速度 vr 分摊到各关节：加权最小范数。返回各关节的速度和"费劲程度" */
function solveRates(q, g, off, face, vr, S) {
  const K0 = bladePoint(fkCore(q, g), off, face);
  const hs = 1e-5, J = [];
  for (const i of RATE_IDX) {
    if (i < 3) { J.push(i === 0 ? [1, 0, 0] : i === 1 ? [0, 1, 0] : [0, 0, 1]); continue; }
    const q2 = q.slice(); q2[i] += hs;
    const K = bladePoint(fkCore(q2, g), off, face);
    J.push([(K[0] - K0[0]) / hs, (K[1] - K0[1]) / hs, (K[2] - K0[2]) / hs]);
  }
  const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let k = 0; k < J.length; k++) { const s2 = S[k] * S[k], j = J[k]; for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) A[a][b] += s2 * j[a] * j[b]; }
  const lam = solve3(A, vr);
  if (!lam) return null;
  const u = new Array(NQ).fill(0);
  let worst = 0;
  for (let k = 0; k < J.length; k++) { const r = S[k] * S[k] * dot(J[k], lam); u[RATE_IDX[k]] = r; worst = Math.max(worst, Math.abs(r) / S[k]); }
  return { u, effort: dot(vr, lam), worst };
}

// 球台当成一整块：p 处半径 r 的球有没有扎进去
function tablePen(p, r) {
  const below = P.tableY - (p[1] - r);
  if (below <= 0) return 0;
  const ox = Math.abs(p[0]) - P.tableW / 2 - r, oz = -P.tableL / 2 - r - p[2];
  if (ox >= 0 || oz >= 0) return 0;
  return Math.min(below, -ox, -oz);
}
// 球拍（薄圆片）和一段胶囊的穿插深度
function discPen(c, nrm, a, b, rr) {
  const cp = closestOnSeg(c, a, b);
  const ux = c[0] - cp[0], uy = c[1] - cp[1], uz = c[2] - cp[2], d = Math.hypot(ux, uy, uz);
  if (d < 1e-6) return rr + DIM.blade;
  const un = (ux * nrm[0] + uy * nrm[1] + uz * nrm[2]) / d;
  const eff = DIM.blade * Math.sqrt(Math.max(0, 1 - un * un)) + 0.006;
  return Math.max(0, rr + eff - d);
}
const spherePen = (p, r, a, b, rr) => Math.max(0, r + rr - distSeg(p, a, b));
// 点到一条竖线段（球抛起落下的那条线）的距离
const distColumn = (p, x, z, y1, y2) => Math.hypot(p[0] - x, p[1] - clamp(p[1], y1, y2), p[2] - z);

function segColumn(a, b, x, z, y1, y2) {
  let m = 1e9;
  for (let k = 0; k <= 4; k++) { const f = k / 4, px = a[0] + (b[0] - a[0]) * f, py = a[1] + (b[1] - a[1]) * f, pz = a[2] + (b[2] - a[2]) * f; const d = Math.hypot(px - x, py - clamp(py, y1, y2), pz - z); if (d < m) m = d; }
  return m;
}

function headOf(A, aim) {
  let lk = norm(sub(aim, A.C));
  // 脖子有限度：视线和胸口正前方的夹角别超过 78°（再往上是眼睛在看）
  const cf = dot(lk, A.Fc), mx = Math.cos(78 * DEG);
  if (cf < mx) {
    const perp = norm(sub(lk, [A.Fc[0] * cf, A.Fc[1] * cf, A.Fc[2] * cf]));
    const s = Math.sin(78 * DEG);
    lk = [A.Fc[0] * mx + perp[0] * s, A.Fc[1] * mx + perp[1] * s, A.Fc[2] * mx + perp[2] * s];
  }
  // 脖子大体顺着脊柱，稍微跟着视线；腰弯得越深，头越要抬起来一些（不然脸就朝着地了）
  const up = 0.14 + 0.5 * Math.max(0, 0.62 - A.Uc[1]);
  const nd = norm([A.Uc[0] + 0.16 * lk[0], A.Uc[1] + 0.16 * lk[1] + up, A.Uc[2] + 0.16 * lk[2]]);
  const head = addmul(A.C, nd, DIM.neck + DIM.headR * 0.8);
  return { head, look: lk, neck: addmul(A.C, nd, 0.03) };
}

function feetOf(P0, F, R, fe, stance) {
  const w = fe.wide * stance;
  const cx = P0[0] + R[0] * fe.shift, cz = P0[2] + R[2] * fe.shift;
  const rotY = (v, a) => { const c = Math.cos(a), s = Math.sin(a); return [v[0] * c + v[2] * s, 0, -v[0] * s + v[2] * c]; };
  return {
    AL: [cx - R[0] * w + F[0] * fe.fL, DIM.ankle, cz - R[2] * w + F[2] * fe.fL],
    AR: [cx + R[0] * w + F[0] * fe.fR, DIM.ankle, cz + R[2] * w + F[2] * fe.fR],
    toeL: rotY(F, fe.toeL * DEG), toeR: rotY(F, -fe.toeR * DEG),
    poleL: norm([F[0] - R[0] * 0.4, 0, F[2] - R[2] * 0.4]), poleR: norm([F[0] + R[0] * 0.4, 0, F[2] + R[2] * 0.4]),
  };
}
const hipsOf = (Pv, R) => ({
  HL: [Pv[0] - R[0] * DIM.hipHalf, Pv[1] - DIM.hipDrop, Pv[2] - R[2] * DIM.hipHalf],
  HR: [Pv[0] + R[0] * DIM.hipHalf, Pv[1] - DIM.hipDrop, Pv[2] + R[2] * DIM.hipHalf],
});
const LEG_MAX = DIM.thigh + DIM.shin - 0.015;

function dofLim(i, g) {
  switch (i) {
    case 4: return LIM.lean; case 5: return LIM.curl; case 6: return LIM.side; case 7: return LIM.twist;
    case 10: return LIM.rot; case 11: return LIM.elb; case 12: return g.pro; case 13: return LIM.wf; case 14: return g.wd;
    default: return [-1e9, 1e9];
  }
}
function clampQ(q, g) {
  for (let i = 4; i < NQ; i++) { const r = dofLim(i, g); q[i] = clamp(q[i], r[0], r[1]); }
  const e = Math.hypot(q[8], q[9]);
  if (e > LIM.elev[1]) { q[8] *= LIM.elev[1] / e; q[9] *= LIM.elev[1] / e; }
  return q;
}

const REACH_W = { 4: 0.22, 7: 0.4, 8: 1, 9: 1, 10: 1.1, 11: 1.2 };
const ARM3 = [8, 9, 10], ARM4 = [8, 9, 10, 11];
/**
 * 让拍面中心够到 target，改动尽量小。默认动肩、肘和一点躯干；opt.idx 指定只动哪些关节；
 * opt.map：q 不是直接摆出来的姿势，而是要先经过 map（比如"手臂引到头的那一刻，上身已经压下去一截"）
 */
function reach(q0, g, target, iters = 5, opt = null) {
  const q = q0.slice();
  const IDX = (opt && opt.idx) || [8, 9, 10, 11, 4, 7], S = IDX.map((i) => REACH_W[i]);
  const bc = opt && opt.map ? (x) => fkCore(opt.map(x), g).Bc : (x) => fkCore(x, g).Bc;
  for (let it = 0; it < iters; it++) {
    const B0 = bc(q), err = sub(target, B0);
    if (len(err) < 0.004) break;
    const J = [];
    for (const i of IDX) { const q2 = q.slice(); q2[i] += 1e-5; const B = bc(q2); J.push([(B[0] - B0[0]) / 1e-5, (B[1] - B0[1]) / 1e-5, (B[2] - B0[2]) / 1e-5]); }
    const A = [[1e-3, 0, 0], [0, 1e-3, 0], [0, 0, 1e-3]];
    for (let k = 0; k < J.length; k++) for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) A[a][b] += S[k] * S[k] * J[k][a] * J[k][b];
    const lam = solve3(A, err);
    if (!lam) break;
    let mx = 0; const dq = [];
    for (let k = 0; k < J.length; k++) { const v = S[k] * S[k] * dot(J[k], lam); dq.push(v); mx = Math.max(mx, Math.abs(v)); }
    const sc = mx > 0.45 ? 0.45 / mx : 1;
    for (let k = 0; k < J.length; k++) q[IDX[k]] += dq[k] * sc;
    clampQ(q, g);
  }
  return q;
}

const hermite1 = (p0, m0, p1, m1, T, u) => { const u2 = u * u, u3 = u2 * u; return (2 * u3 - 3 * u2 + 1) * p0 + (u3 - 2 * u2 + u) * T * m0 + (-2 * u3 + 3 * u2) * p1 + (u3 - u2) * T * m1; };

// 球台顶面以下、台面范围以内：要抬多高才能回到台面上方（不在台面范围内返回 0）
function tableLift(p, r) {
  const below = P.tableY - (p[1] - r);
  if (below <= 0) return 0;
  if (Math.abs(p[0]) - P.tableW / 2 - r >= 0 || -P.tableL / 2 - r - p[2] >= 0) return 0;
  return below;
}

// 两个单位向量之间沿球面插值
function slerp(a, b, t) {
  const c = clamp(dot(a, b), -1, 1);
  if (c > 0.99999) return a;
  if (c < -0.9995) { // 正好相反：随便挑一个垂直的方向绕过去（朝下优先）
    let ax = cross(a, [0, -1, 0]); if (len(ax) < 1e-3) ax = cross(a, [1, 0, 0]); ax = norm(ax);
    const m = cross(ax, a), an = Math.PI * t; return [a[0] * Math.cos(an) + m[0] * Math.sin(an), a[1] * Math.cos(an) + m[1] * Math.sin(an), a[2] * Math.cos(an) + m[2] * Math.sin(an)];
  }
  const th = Math.acos(c), s0 = Math.sin((1 - t) * th), s1 = Math.sin(t * th), sn = Math.sin(th);
  return [(a[0] * s0 + b[0] * s1) / sn, (a[1] * s0 + b[1] * s1) / sn, (a[2] * s0 + b[2] * s1) / sn];
}

// 同上，但在台边是渐变的（给逐帧动的东西用：手从台外挪到台面上方时不会突然跳一下）
function tableLiftSoft(p, r) {
  const below = P.tableY - (p[1] - r);
  if (below <= 0) return 0;
  const din = Math.min(P.tableW / 2 + r - Math.abs(p[0]), p[2] + P.tableL / 2 + r) + 0.04; // 离台边还有 4 厘米就开始抬
  return din <= 0 ? 0 : below * smooth(din / 0.07);
}

/**
 * o: { type, hand, face, pc, n, vr, vOut(球离拍速度), off, tRel, tPalm0, tossY:[低, 高], ballAt(t)→{p}, pl(选手习惯), fake('lift'|'chop'|'tuck'|null), rng }
 * 返回 { ok:false, why } 或 { ok:true, at(t, full), racketAt(t), contactAt(t), tReady, tDone, stats, ... }
 */
export function buildMotion(o) {
  const mv = MOVES[o.type], pl = o.pl, rng = o.rng;
  const g = GRIPS[pl.grip && pl.grip[o.type] ? pl.grip[o.type] : mv.grip];
  const hand = o.hand, face = o.face;
  const mir = hand < 0;
  const mx = (v) => (mir ? mirror(v) : v);
  // 左手持拍：整件事放到镜像世界里按右手算，最后再镜像回来
  const pc = mx(o.pc), n = mx(o.n), vr = mx(o.vr), vOut = o.vOut ? mx(o.vOut) : null;
  const ballAt = (t) => mx(o.ballAt(t).p);
  const pcb = addmul(pc, n, -P.R);           // 拍面上的触球点
  const Zt = [n[0] * face, n[1] * face, n[2] * face];
  const off = o.off || [0, 0];
  const tRel = o.tRel;
  const palmLo = (() => { const b = ballAt(-1e6); return [b[0], b[1] - P.R - 0.012, b[2]]; })();
  const palmHi = (() => { const b = ballAt(tRel); return [b[0], b[1] - P.R - 0.012, b[2]]; })();
  const colY1 = pc[1] + 0.06, colY2 = o.tossY[1];
  const stanceW = pl.stance || 1;
  const followUp = pl.followUp === undefined ? 0.1 : pl.followUp;
  // 球离拍后头 0.25 秒走过的几个点：身体不能挡在这条路上
  const outPts = [];
  if (vOut) for (const t of [0.03, 0.06, 0.1, 0.14, 0.19, 0.25]) outPts.push([pc[0] + vOut[0] * t, pc[1] + vOut[1] * t - 0.5 * P.g * t * t, pc[2] + vOut[2] * t]);

  const recv = [-0.1, 1.25, 1.7]; // 发完球看向接球的人
  // 各关节的速度能力：手腕型的人手腕快、大臂慢；大臂型反过来
  const wr = pl.wrist === undefined ? 1 : pl.wrist;
  const S = RATE_KEY.map((k) => SPEED[k] * (k === 'wf' || k === 'wd' || k === 'pro' ? wr : k === 'sF' || k === 'sR' || k === 'rot' || k === 'elb' ? 1.25 - 0.25 * wr : 1) * (k === 'wd' && g.key === 'serve' ? 1.25 : 1));

  // ---------- 1. 触球姿势 ----------
  // 搜索变量：朝向、前倾、含胸、侧弯、转体、上臂(2)、上臂自转、屈肘、手腕侧偏
  const v0 = (() => {
    const q = new Array(NQ).fill(0);
    // 侧身站的发球：站在自己反手位一角时最侧；越往正手位那边站，身子越要转开一些朝着球台，不然自己的头和肩会把球挡住
    const open = mv.stance === 'side' ? 40 * clamp((0.45 - pc[0]) / 0.9, 0, 1) : 0;
    q[3] = (mv.yaw + open + (pl.yaw || 0)) * DEG; q[4] = clamp((mv.lean + (pl.lean || 0)) * DEG, LIM.lean[0], LIM.lean[1]); q[5] = mv.curl * DEG; q[6] = mv.side * DEG; q[7] = mv.twist * DEG;
    const A = fkCore(q, g);
    const an = dirsToAngles(A, norm(H2W(A.F, A.R, mv.ua)), norm(H2W(A.F, A.R, mv.fa)));
    return [q[3], q[4], q[5], q[6], q[7], an.sF, an.sR, an.rot, an.elb, clamp((mv.wd + (pl.wd || 0)) * DEG, g.wd[0], g.wd[1])];
  })();
  const RNG = [30 * DEG, 12 * DEG, 8 * DEG, 8 * DEG, 18 * DEG, 0.5, 0.5, 0.6, 0.5, 0.4];
  const LO = [v0[0] - 48 * DEG, LIM.lean[0], 0, -15 * DEG, -42 * DEG, -2.3, -2.3, LIM.rot[0], LIM.elb[0], g.wd[0]];
  const HI = [v0[0] + 48 * DEG, LIM.lean[1], 28 * DEG, 15 * DEG, 42 * DEG, 2.3, 2.3, LIM.rot[1], LIM.elb[1], g.wd[1]];
  const WPREF = [0.5, 0.7, 0.2, 0.6, 0.3, 0.3, 0.3, 0.15, 0.25, 0.35];
  // 不带侧旋的球是直着往上 / 往下蹭的，逆旋转、勾手那种"横着甩"的架势用不上：手臂的摆法放宽，只留个大概的样子
  if (o.kind === 'pure' && (o.type === 'reverse' || o.type === 'hook')) for (const i of [5, 6, 7, 8, 9]) WPREF[i] *= 0.4;
  const py0 = pl.py || 0.8;
  const riseR = (pl.rise === undefined ? 28 : pl.rise) * DEG; // 球抛起来之后上身比触球时直多少：真人是抛球时站得比较直，球落下来再弯腰压下去
  // 引拍多大、挥多久（由"从静止加速到触球速度要多长"定）。弯腰下沉比挥臂慢，得早点开始（但要等抛球的手送完球）
  const V = len(vr);
  const bsAmp = pl.bs === undefined ? 1 : pl.bs;
  const Db = mv.bsD * bsAmp * (0.8 + 0.035 * V);
  const T1max = clamp(Math.max(1.6 * Db / Math.max(V, 1), (pl.tempo || 0.25) * 0.62), 0.13, 0.34);
  const T1body = clamp(-tRel - 0.12, T1max, 0.4);
  const tb0 = Math.min(Math.max(tRel - 0.03, -T1max - (pl.bsT || 0.42)), -T1max - 0.14, -T1body - 0.02); // 什么时候开始引拍
  // 上身起落得有时间：抛得低、出手快的时候来不及先直起来再压下去，起伏就小
  const riseH = Math.min(riseR * 0.45, 110 * DEG * T1body / 1.5);                                        // 托着球准备时：比触球时直一些，但还弯着腰
  const riseT = Math.min(riseR, riseH + 70 * DEG * Math.max(0, -T1body - tb0) / 1.5, 110 * DEG * T1body / 1.5); // 球在空中时：最直
  const bsTw = (mv.bs.twist || 0) * DEG * bsAmp;
  // 球离手之后到触球这一路取几个时刻：那会儿球在哪儿、上身大概是什么样子。
  // 上身有三个样子：托球准备（略弯腰）→ 球抛起来之后直起来 → 球落下来时压到触球姿势。
  // 每个时刻记：比触球时直多少(rise)、骨盆高多少(py)、转体 = ta × 触球时的转体 + tb
  const seeT = [];
  { const HOLD = [riseH, 0.03, 0.4, 0], TALL = [riseT, 0.05, 1, bsTw], HIT = [0, 0, 1, 0];
    const mix = (a, b, k) => a.map((x, i) => x + (b[i] - x) * k);
    for (const t of [-0.015, -0.045, -0.08, -0.12, -0.17, -0.23, -0.3, -0.38, -0.47, -0.57, -0.68, -0.8, -0.95]) {
      if (t < tRel + 0.05) break;
      const ps = t >= -T1body ? mix(TALL, HIT, smooth((t + T1body) / T1body)) : t > tb0 ? mix(HOLD, TALL, smooth((t - tb0) / (-T1body - tb0))) : HOLD;
      seeT.push({ t, b: ballAt(t), hit: t >= -T1body && ps[0] < 0.06 * riseT + 1e-9, rise: ps[0], py: ps[1], ta: ps[2], tb: ps[3] });
    } }

  const evalPose = (v, bound) => {
    const q = new Array(NQ).fill(0);
    q[3] = v[0]; q[4] = v[1]; q[5] = v[2]; q[6] = v[3]; q[7] = v[4]; q[8] = v[5]; q[9] = v[6]; q[10] = v[7]; q[11] = v[8]; q[14] = v[9];
    const A = fkCore(q, g, Zt);
    if (!A) return null;
    if (q[12] < g.pro[0] || q[12] > g.pro[1] || q[13] < LIM.wf[0] || q[13] > LIM.wf[1] || A.elev > LIM.elev[1]) return null; // 手腕、前臂转不到这个拍面
    const s = sub(pcb, bladePoint(A, off, face));
    q[0] = s[0]; q[1] = s[1]; q[2] = s[2];
    shiftState(A, s);
    let c = 0;
    // 关节范围 + 别扭程度
    c += 260 * (lim2(q[12], g.pro) + lim2(q[13], LIM.wf) + lim2(q[10], LIM.rot) + lim2(q[11], LIM.elb) + lim2(A.elev, LIM.elev) + lim2(q[14], g.wd));
    c += 0.35 * sq(q[13]) + 0.2 * sq(q[12] / 1.3) + 0.1 * sq((q[10] - 0.2) / 1.2);
    c += 0.8 * sq(Math.max(0, q[11] - 126 * DEG) / 0.3) + 0.4 * sq(Math.max(0, 55 * DEG - q[11]) / 0.4); // 肘折得太死或伸得太直都不自然
    c += 2.5 * sq(Math.max(0, q[4] - 60 * DEG) / (8 * DEG));                                              // 腰弯到 60° 以上就很吃力了
    // 肩：上臂伸不到背后太多，也横不过胸口太多
    const ub = -dot(A.ua, A.Fc), uc = -dot(A.ua, A.Rc), uu = dot(A.ua, A.Uc);
    c += 260 * (sq(Math.max(0, ub - 0.6)) + sq(Math.max(0, uc - 0.5)) + sq(4 * Math.max(0, ub - 0.25) * Math.max(0, uu - 0.25)));
    const cLim = c;
    for (let i = 0; i < 10; i++) c += WPREF[i] * sq((v[i] - v0[i]) / RNG[i]);
    // 拍头的指向（在拍面里转，不改变旋转）：钟摆、逆旋转大体朝下，勾手朝上，左右偏多少随人。拍面接近放平时无所谓上下
    // cosd = 拍头和"拍面里最朝下的那个方向"夹角的余弦
    if (mv.headDir) {
      const k = 1 - Zt[1] * Zt[1];
      if (k > 0.1) { const kl = Math.sqrt(k), cosd = (A.l[0] * Zt[0] * Zt[1] + A.l[1] * (Zt[1] * Zt[1] - 1) + A.l[2] * Zt[2] * Zt[1]) / kl; c += 2.5 * k * sq(Math.max(0, 0.3 + mv.headDir * cosd) / 0.5); }
    }
    const cPref = c - cLim;
    if (c >= bound) return null;
    // 骨盆高度、腿够不够长
    c += 300 * lim2(q[1], [0.56, 0.93]) + 220 * sq(q[1] - py0);
    const ft = feetOf(A.P, A.F, A.R, mv.feet, stanceW), hp = hipsOf(A.P, A.R);
    const dL = len(sub(hp.HL, ft.AL)), dR = len(sub(hp.HR, ft.AR));
    c += 400 * (sq(Math.max(0, dL - LEG_MAX + 0.03)) + sq(Math.max(0, dR - LEG_MAX + 0.03)));
    const cLeg = c - cLim - cPref;
    if (c >= bound) return null;
    const KL = ik2(hp.HL, ft.AL, ft.poleL, DIM.thigh, DIM.shin).E, KR = ik2(hp.HR, ft.AR, ft.poleR, DIM.thigh, DIM.shin).E;
    // 人要站在球台外面
    let tp = sq(tablePen(A.P, 0.175)) + sq(tablePen(A.M, 0.14)) + sq(tablePen(A.C, 0.145)) + sq(tablePen(A.E, 0.06)) + sq(tablePen(A.W, 0.05));
    tp += sq(tablePen(hp.HL, 0.105)) + sq(tablePen(hp.HR, 0.105)) + sq(tablePen(KL, 0.085)) + sq(tablePen(KR, 0.085));
    tp += sq(tablePen([ft.AL[0] + ft.toeL[0] * 0.1, 0.05, ft.AL[2] + ft.toeL[2] * 0.1], 0.04)) + sq(tablePen([ft.AR[0] + ft.toeR[0] * 0.1, 0.05, ft.AR[2] + ft.toeR[2] * 0.1], 0.04));
    tp += sq(tablePen(lerp(hp.HL, KL, 0.5), 0.095)) + sq(tablePen(lerp(hp.HR, KR, 0.5), 0.095));
    c += 900 * tp;
    // 球拍、手、小臂别和自己的身体打架
    const hd = headOf(A, pc).head;
    const caps = [[A.P, A.M, DIM.rLo], [A.M, A.C, DIM.rHi], [hd, hd, DIM.headR], [hp.HL, KL, DIM.rThigh], [hp.HR, KR, DIM.rThigh]];
    const hnd = addmul(A.W, A.dd, 0.05), fm = lerp(A.E, A.W, 0.55);
    let sp = 0;
    for (const [a, b, rr] of caps) {
      sp += sq(discPen(A.Bc, A.Zf, a, b, rr + 0.025)) + sq(spherePen(hnd, 0.045, a, b, rr + 0.02)) + sq(spherePen(fm, DIM.rFore, a, b, rr + 0.015));
    }
    sp += sq(spherePen(A.E, 0.046, A.P, A.M, DIM.rLo)) + sq(spherePen(A.E, 0.046, A.M, A.C, DIM.rHi - 0.02)) + sq(spherePen(A.E, 0.046, hd, hd, DIM.headR));
    c += 1600 * sp;
    // 抛起来的球走一条竖线：头、身体、肩、上臂得让开（头和胸口还要给托球的手留出位置）
    let cp = sq(Math.max(0, DIM.headR + P.R + 0.04 - distColumn(hd, pc[0], pc[2], colY1, colY2)));
    cp += 0.2 * sq(Math.max(0, 0.34 - len(sub(hd, pc)))) + 0.4 * sq(Math.max(0, pc[1] + 0.16 - hd[1])); // 触球点别贴在脸跟前：眼睛离球总得有一尺来远，而且是从上面往下看球
    for (const [pt, rr] of [[A.M, DIM.rLo], [lerp(A.M, A.C, 0.5), DIM.rHi], [A.C, DIM.rHi], [A.SL, 0.065]]) cp += sq(Math.max(0, rr + P.R + 0.025 - distColumn(pt, pc[0], pc[2], colY1, colY2)));
    cp += sq(Math.max(0, DIM.rUpper + 0.012 + P.R + 0.03 - segColumn(A.SR, A.E, pc[0], pc[2], colY1, colY2)));
    const hd2 = headOf(A, recv).head;
    // 球打出去的那条路也得让开
    for (const bp of outPts) {
      cp += sq(Math.max(0, DIM.headR + P.R + 0.045 - len(sub(bp, hd)))) + sq(Math.max(0, DIM.headR + P.R + 0.045 - len(sub(bp, hd2)))) + sq(Math.max(0, DIM.rHi + P.R + 0.03 - distSeg(bp, A.M, A.C))) + sq(Math.max(0, DIM.rLo + P.R + 0.03 - distSeg(bp, A.P, A.M)));
      cp += sq(Math.max(0, 0.07 + P.R + 0.03 - len(sub(bp, A.SL)))) + sq(Math.max(0, DIM.rThigh + P.R + 0.02 - distSeg(bp, hp.HL, KL)));
    }
    c += 500 * cp;
    // 抛球那会儿上身比触球时直：托着球时略弯腰（riseH），球抛起来后人跟着起来一些（riseR），球落下来再压下去。
    // （起伏的幅度受时间限制，见 riseH / riseT）这两个姿势下：托球的手要够得着，而且是从身体前面伸过去的；头和胸口还得给往上送的手让出位置
    const TRh = trunkLite(q[0], q[1] + 0.03, q[2], q[3], Math.max(LIM.lean[0], q[4] - riseH), q[5], q[6], q[7] * 0.4);
    const TRt = trunkLite(q[0], q[1] + 0.05, q[2], q[3], Math.max(LIM.lean[0], q[4] - riseT), q[5], q[6], q[7] + bsTw);
    let fr = 0;
    const yTop = palmHi[1] + followUp + 0.08;
    { const zf = (pc[0] - TRt.C[0]) * A.F[0] + (pc[2] - TRt.C[2]) * A.F[2], zr = (pc[0] - TRt.C[0]) * A.R[0] + (pc[2] - TRt.C[2]) * A.R[2]; fr += 0.25 * (lim2(zf, mv.zone.f) + lim2(zr, mv.zone.r)); }
    for (const TR of [TRh, TRt]) {
      fr += sq(Math.max(0, DIM.headR + 0.085 - distColumn(TR.head, pc[0], pc[2], palmLo[1], yTop)));
      fr += sq(Math.max(0, DIM.rHi + 0.08 - distSeg([pc[0], clamp(lerp(TR.M, TR.C, 0.5)[1], palmLo[1], yTop), pc[2]], TR.M, TR.C)));
    }
    for (const [TR, pm] of [[TRh, palmLo], [TRt, [palmHi[0], palmHi[1] + followUp, palmHi[2]]]]) {
      const d = len(sub(pm, TR.SL)); fr += 0.6 * (sq(Math.max(0, d - 0.5)) + sq(Math.max(0, 0.2 - d)));
      for (const f of [0.75, 1]) { const pt = lerp(TR.SL, pm, f); fr += sq(Math.max(0, DIM.rHi + 0.07 - distSeg(pt, TR.M, TR.C))) + sq(Math.max(0, DIM.rLo + 0.07 - distSeg(pt, TR.P, TR.M))) + sq(Math.max(0, DIM.headR + 0.065 - len(sub(pt, TR.head)))); }
    }
    c += 500 * fr;
    // 接球的人得一直看得见球：从他的眼睛到球的这条视线上，不能有发球人自己的头、躯干、肩膀、上臂
    let see = 0;
    const blk = (b, pt, rr) => sq(Math.max(0, hideDepth(b, pt, rr) + P.R + 0.022));
    for (const sT of seeT) {
      const b = sT.b;
      if (sT.hit) {
        see += blk(b, headOf(A, b).head, DIM.headR) + blk(b, A.C, DIM.rHi) + blk(b, lerp(A.M, A.C, 0.5), DIM.rHi) + blk(b, A.M, DIM.rLo) + blk(b, A.SL, 0.066);
        see += blk(b, A.SR, 0.066) + blk(b, lerp(A.SR, A.E, 0.5), DIM.rUpper + 0.004) + blk(b, A.E, 0.046);
      } else {
        const TR = trunkLite(q[0], q[1] + sT.py, q[2], q[3], Math.max(LIM.lean[0], q[4] - sT.rise), q[5], q[6], sT.ta * q[7] + sT.tb);
        see += blk(b, headOf(TR, b).head, DIM.headR) + blk(b, TR.C, DIM.rHi) + blk(b, lerp(TR.M, TR.C, 0.5), DIM.rHi) + blk(b, TR.M, DIM.rLo) + blk(b, TR.SL, 0.066);
      }
    }
    c += (globalThis.__seeW === undefined ? 2500 : globalThis.__seeW) * see;
    if (c >= bound) return null;
    // 发这个拍速费不费劲
    const rt = solveRates(q, g, off, face, vr, S);
    if (!rt) return null;
    // 触球前后各 5 毫秒里，每个关节都还得在活动范围内、腿也够得着地——
    // 不然关节顶到头被卡住，画出来的拍子就不是物理算的那个位置、那个速度了
    { const u = rt.u, h = 0.005;
      for (const [i, r] of HARD) { const lo = r || (i === 12 ? g.pro : g.wd), a = q[i] - u[i] * h, b = q[i] + u[i] * h; if (Math.min(a, b) < lo[0] || Math.max(a, b) > lo[1]) return null; }
      if (Math.max(Math.hypot(q[8] - u[8] * h, q[9] - u[9] * h), Math.hypot(q[8] + u[8] * h, q[9] + u[9] * h)) > LIM.elev[1]) return null;
      if (Math.max(dL, dR) > LEG_MAX - 0.006) return null; }
    c += 0.45 * rt.effort + 4 * sq(Math.max(0, rt.worst - 1.15));
    if (c >= bound) return null;
    return { v: v.slice(), q, A, cost: c, rt, ft, effort: rt.effort, worst: rt.worst, parts: { lim: cLim, pref: cPref, leg: cLeg, table: 900 * tp, self: 1600 * sp, col: 500 * cp, free: 500 * fr, see: 2500 * see, effort: 0.45 * rt.effort + 4 * sq(Math.max(0, rt.worst - 1.15)) } };
  };

  let best = null;
  // 先粗找：换几个朝向、前倾、肘的弯度起步，再撒一把随机点。留下朝向各不相同的几个起点
  const starts = [];
  const seed = (v) => {
    const r = evalPose(v, starts.length >= 3 ? starts[starts.length - 1].cost : 1e9); if (!r) return;
    const near = starts.findIndex((s2) => Math.abs(s2.v[0] - r.v[0]) < 16 * DEG);
    if (near >= 0) { if (r.cost < starts[near].cost) starts[near] = r; else return; } else starts.push(r);
    starts.sort((a, b) => a.cost - b.cost); if (starts.length > 3) starts.length = 3;
  };
  for (const dy of [0, -22, 22, -40, 40]) for (const dl of [0, -9, 9]) for (const de of [0, -0.35, 0.35]) {
    const v = v0.slice(); v[0] += dy * DEG; v[1] = clamp(v[1] + dl * DEG, LO[1], HI[1]); v[8] = clamp(v[8] + de, LO[8], HI[8]);
    seed(v);
  }
  for (let k = 0; k < 90; k++) { const v = v0.map((x, i) => clamp(x + (rng() * 2 - 1) * RNG[i] * 1.2, LO[i], HI[i])); seed(v); }
  // 一个能用的起点都没撒到（手腕够不着这个拍面的情况居多）：范围放宽再撒
  for (let k = 0; k < 500 && !starts.length; k++) { const v = v0.map((x, i) => clamp(x + (rng() * 2 - 1) * RNG[i] * 2.2, LO[i], HI[i])); seed(v); }
  if (!starts.length) return { ok: false, why: 'pose' };
  const step = (sc) => {
    const v = best.v.slice();
    const m = 1 + Math.floor(rng() * 4);
    for (let k = 0; k < m; k++) { const i = Math.floor(rng() * 10); v[i] = clamp(v[i] + (rng() * 2 - 1) * sc * RNG[i], LO[i], HI[i]); }
    const r = evalPose(v, best.cost); if (r) best = r;
  };
  // 每个起点先各自往下走一小段，再从最好的那个接着细找：免得一开始就掉进"背对着球台"这种死胡同
  { let top = null;
    for (const s0 of starts) { best = s0; for (let it = 0; it < 55; it++) step(0.55); if (!top || best.cost < top.cost) top = best; }
    best = top; }
  const N = 200;
  for (let it = 0; it < N; it++) step(it < N * 0.3 ? 0.55 : it < N * 0.75 ? 0.22 : 0.08);
  if (o.dbg) o.dbg(best);
  if (best.cost > 14) return { ok: false, why: 'pose-cost' };
  const qC = best.q, uC = best.rt.u, ft = best.ft, AC = best.A;

  // ---------- 2. 关键姿势 ----------
  const hpC = hipsOf(AC.P, AC.R);
  const legFix = (q) => { // 腿够不着就把骨盆放低
    for (let k = 0; k < 3; k++) {
      const sy = Math.sin(q[3]), cy = Math.cos(q[3]), hp = hipsOf(q, [-cy, 0, sy]);
      const ex = Math.max(len(sub(hp.HL, ft.AL)), len(sub(hp.HR, ft.AR))) - LEG_MAX;
      if (ex <= 0) break;
      q[1] -= ex * 1.3 + 0.002;
    }
    return q;
  };
  const outOfTable = (q) => { // 骨盆被挪进球台范围了就水平推出来
    for (let k = 0; k < 3; k++) {
      const pen = tablePen(q, 0.17); if (pen <= 0) break;
      const ox = P.tableW / 2 + 0.17 - Math.abs(q[0]), oz = q[2] + P.tableL / 2 + 0.17;
      if (ox < oz) q[0] += Math.sign(q[0] || 1) * (ox + 0.005); else q[2] -= oz + 0.005;
    }
    legFix(q);
    for (let k = 0; k < 3; k++) { // 膝盖
      const sy = Math.sin(q[3]), cy = Math.cos(q[3]), hp = hipsOf(q, [-cy, 0, sy]);
      let moved = false;
      for (const [H, A_, pole] of [[hp.HL, ft.AL, ft.poleL], [hp.HR, ft.AR, ft.poleR]]) {
        const K = ik2(H, A_, pole, DIM.thigh, DIM.shin).E;
        if (tablePen(K, 0.078) <= 0) continue;
        const ox = P.tableW / 2 + 0.078 - Math.abs(K[0]), oz = K[2] + P.tableL / 2 + 0.078;
        if (ox < oz) q[0] += Math.sign(K[0] || 1) * (ox * 1.6 + 0.004); else q[2] -= oz * 1.6 + 0.004;
        moved = true;
      }
      if (!moved) break;
      legFix(q);
    }
    return q;
  };
  // 一个姿势"打不打架"：球拍 / 手 / 小臂 / 肘 对 身体、球台；col = 还要让开球抛起落下的那条线（并给托球的手留位置；col = 2 时只管手臂和球拍）
  const posePen = (q, col, target) => {
    const A = fkCore(q, g);
    const hp = hipsOf(A.P, A.R);
    const KL = ik2(hp.HL, ft.AL, ft.poleL, DIM.thigh, DIM.shin).E, KR = ik2(hp.HR, ft.AR, ft.poleR, DIM.thigh, DIM.shin).E;
    const hd = headOf(A, pc).head;
    const hnd = addmul(A.W, A.dd, 0.05), fm = lerp(A.E, A.W, 0.55);
    let pen = 0;
    for (const [a, b, rr] of [[A.P, A.M, DIM.rLo], [A.M, A.C, DIM.rHi], [hd, hd, DIM.headR], [hp.HL, KL, DIM.rThigh], [hp.HR, KR, DIM.rThigh]]) {
      pen += sq(discPen(A.Bc, A.Zf, a, b, rr + 0.02)) + sq(spherePen(hnd, 0.045, a, b, rr + 0.015)) + sq(spherePen(fm, DIM.rFore, a, b, rr + 0.012));
    }
    pen += sq(spherePen(A.E, 0.046, A.P, A.M, DIM.rLo)) + sq(spherePen(A.E, 0.046, hd, hd, DIM.headR));
    const drop = DIM.blade * Math.sqrt(Math.max(0, 1 - A.Zf[1] * A.Zf[1]));
    pen += sq(tablePen([A.Bc[0], A.Bc[1] - drop + 0.02, A.Bc[2]], 0.03)) + sq(tablePen(hnd, 0.05)) + sq(tablePen(A.E, 0.055)) + sq(tablePen(A.W, 0.045));
    pen += sq(tablePen(A.P, 0.16)) + sq(tablePen(KL, 0.075)) + sq(tablePen(KR, 0.075));
    if (col) {
      const y2 = Math.max(colY2, palmHi[1] + followUp + 0.1);
      const dc = (p) => distColumn(p, pc[0], pc[2], palmLo[1], y2);
      if (col !== 2) {
        pen += sq(Math.max(0, DIM.headR + 0.085 - dc(hd))) + sq(Math.max(0, DIM.rHi + 0.08 - distSeg([pc[0], clamp(lerp(A.M, A.C, 0.5)[1], palmLo[1], y2), pc[2]], A.M, A.C)));
        pen += sq(Math.max(0, DIM.rLo + 0.07 - dc(A.M)));
      }
      for (const [pt, rr] of [[A.Bc, DIM.blade + 0.01], [hnd, 0.05]]) pen += sq(Math.max(0, rr + P.R + 0.03 - distColumn(pt, pc[0], pc[2], colY1 - 0.1, y2)));
      pen += sq(Math.max(0, DIM.rUpper + 0.012 + P.R + 0.03 - segColumn(A.SR, A.E, pc[0], pc[2], colY1 - 0.1, y2))) + sq(Math.max(0, DIM.rFore + 0.01 + P.R + 0.03 - segColumn(A.E, A.W, pc[0], pc[2], colY1 - 0.1, y2)));
    }
    const hard = pen;
    if (target) pen += 0.6 * (sq(A.Bc[0] - target[0]) + sq(A.Bc[1] - target[1]) + sq(A.Bc[2] - target[2]));
    return { pen, hard };
  };
  const REP = [[8, 0.3], [9, 0.3], [10, 0.4], [11, 0.3], [7, 0.2], [4, 0.12], [13, 0.3], [14, 0.3], [12, 0.3]];
  const REP_ARM = REP.filter((r) => r[0] >= 8);
  // opt.arm：只动手臂（上身的姿势是定好的，不让它为了迁就手臂而变）；opt.map：同 reach
  const repair = (q0, col, target, iters = 70, extra = null, opt = null) => {
    const REPL = opt && opt.arm ? REP_ARM : REP, map = opt && opt.map;
    const pen = (q) => { const r = posePen(map ? map(q) : q, col, target); if (extra) { const e = extra(q); r.pen += e; r.hard += e; } return r; };
    let q = outOfTable(q0.slice()), cur = pen(q);
    if (cur.hard < 1e-7) return { q, hard: 0 };
    for (let it = 0; it < iters && cur.hard > 1e-7; it++) {
      const sc = it < iters * 0.5 ? 1 : 0.4, q2 = q.slice();
      const m = 1 + Math.floor(rng() * 3);
      for (let k = 0; k < m; k++) { const [i, r] = REPL[Math.floor(rng() * REPL.length)]; q2[i] += (rng() * 2 - 1) * r * sc; }
      clampQ(q2, g); outOfTable(q2);
      const c2 = pen(q2);
      if (c2.pen < cur.pen) { q = q2; cur = c2; }
    }
    return { q, hard: cur.hard };
  };
  // 把一个"球拍要去的位置"从身体、球台、抛球线里推出来
  const KL0 = ik2(hpC.HL, ft.AL, ft.poleL, DIM.thigh, DIM.shin).E, KR0 = ik2(hpC.HR, ft.AR, ft.poleR, DIM.thigh, DIM.shin).E;
  const hdC = headOf(AC, pc).head;
  const avoid = (tg, colClear) => {
    const p = tg.slice();
    for (let it = 0; it < 3; it++) {
      for (const [a, b2, rr] of [[AC.P, AC.M, DIM.rLo], [AC.M, AC.C, DIM.rHi], [hpC.HL, KL0, DIM.rThigh], [hpC.HR, KR0, DIM.rThigh], [hdC, hdC, DIM.headR]]) {
        const cp = closestOnSeg(p, a, b2), d = Math.hypot(p[0] - cp[0], p[1] - cp[1], p[2] - cp[2]), need = rr + 0.115;
        if (d < need && d > 1e-6) { const k = (need - d) / d; p[0] += (p[0] - cp[0]) * k; p[1] += (p[1] - cp[1]) * k; p[2] += (p[2] - cp[2]) * k; }
      }
      p[1] += tableLift(p, 0.1);
      if (colClear) { const dx = p[0] - pc[0], dz = p[2] - pc[2], dc = Math.hypot(dx, dz); if (dc < colClear && dc > 1e-6 && p[1] > pc[1] - 0.05) { p[0] += (dx / dc) * (colClear - dc); p[2] += (dz / dc) * (colClear - dc); } }
    }
    return p;
  };
  const hv = Math.hypot(vr[0], vr[2]);
  const dh = hv > 0.8 ? [vr[0] / hv, 0, vr[2] / hv] : norm(H2W(AC.F, AC.R, mv.swingDir || [0, 0, -1]));
  const vd = V > 0.3 ? [vr[0] / V, vr[1] / V, vr[2] / V] : dh;

  // 准备姿势：上身直一点，拍子收在托球手旁边；头和胸口要给抛球的手让出位置
  const QR = qC.slice();
  QR[1] = qC[1] + 0.03; QR[4] = clamp(qC[4] - riseH, LIM.lean[0], LIM.lean[1]); QR[7] = qC[7] * 0.4;
  { const rd = mv.ready; QR[8] = rd.sF * DEG; QR[9] = rd.sR * DEG; QR[10] = rd.rot * DEG; QR[11] = rd.elb * DEG; QR[12] = rd.pro * DEG; QR[13] = rd.wf * DEG; QR[14] = rd.wd * DEG; }
  legFix(QR);
  { // 上身离抛球线太近、或者挡住了接球人看球的视线，就再直起来一点、往后坐一点
    const early = seeT.filter((sT) => sT.t <= tb0 + 0.05);
    for (let k = 0; k < 7; k++) {
      const A = fkCore(QR, g), hd = headOf(A, [pc[0], palmHi[1] + 0.3, pc[2]]).head, y2 = palmHi[1] + followUp + 0.1;
      const dH = distColumn(hd, pc[0], pc[2], palmLo[1], y2) - DIM.headR - 0.09;
      const dC = Math.min(distSeg([pc[0], clamp(A.C[1], palmLo[1], y2), pc[2]], A.M, A.C), distSeg([pc[0], clamp(A.M[1], palmLo[1], y2), pc[2]], A.M, A.C)) - DIM.rHi - 0.085;
      let dS = 1;
      for (const sT of early) dS = Math.min(dS, -hideDepth(sT.b, headOf(A, sT.b).head, DIM.headR) - P.R - 0.015, -hideDepth(sT.b, A.C, DIM.rHi) - P.R - 0.015, -hideDepth(sT.b, A.SL, 0.066) - P.R - 0.015);
      if (dH >= 0 && dC >= 0 && dS >= 0) break;
      QR[4] = Math.max(LIM.lean[0], QR[4] - 4 * DEG); QR[0] -= A.F[0] * 0.015; QR[2] -= A.F[2] * 0.015;
      legFix(QR);
    }
    // 拍子收在托球手旁边、靠持拍这一侧肩膀的方向：两条小臂从两边肩膀伸过来，在手这儿碰头，不会交叉
    const A = fkCore(QR, g), hx = A.SR[0] - palmLo[0], hz = A.SR[2] - palmLo[2], hl = Math.hypot(hx, hz) || 1, rd = pl.readyD || mv.ready.d || 0.19;
    const tg = [palmLo[0] + hx / hl * rd, palmLo[1] + (mv.ready.up === undefined ? 0.06 : mv.ready.up), palmLo[2] + hz / hl * rd];
    // 准备时手臂是垂着的：大臂挂在肩膀下面，小臂往前伸，把拍子递到托球的手旁边。肘不该架到肩膀那么高
    { const an = dirsToAngles(A, norm(H2W(A.F, A.R, [0.06, -0.95, 0.3])), norm(sub(tg, addmul(A.SR, norm(H2W(A.F, A.R, [0.06, -0.95, 0.3])), DIM.upper))));
      QR[8] = an.sF; QR[9] = an.sR; QR[10] = clamp(an.rot, LIM.rot[0], LIM.rot[1]); QR[11] = clamp(an.elb, 60 * DEG, 125 * DEG); }
    // 肘别架高；拍子、持拍的手和小臂离托球的手、托球那条小臂远一点（两只手是并排的，不是叠在一起的）
    const fMid = lerp(A.SL, palmLo, 0.7);
    const readyPen = (q) => {
      const B = fkCore(q, g), hnd = addmul(B.W, B.dd, 0.05);
      let p = 6 * sq(Math.max(0, B.E[1] - B.SR[1] + 0.13));
      for (const pt of [palmLo, fMid]) p += sq(Math.max(0, 0.115 - distSeg(pt, B.E, B.W))) + sq(Math.max(0, 0.125 - len(sub(pt, hnd)))) + sq(discPen(B.Bc, B.Zf, pt, pt, 0.07));
      return p;
    };
    const rp = repair(reach(QR, g, tg, 5, { idx: ARM4 }), 2, tg, 70, readyPen, { arm: true });
    for (let i = 0; i < NQ; i++) QR[i] = rp.q[i]; }
  // 引拍：球拍引到"挥拍方向的反面、再高一些"，距离 Db
  // 远端关节起动得晚：躯干和肩先动，然后肘，最后前臂旋转和手腕
  const LATE = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.85, 0.62, 0.5, 0.5];
  const QB = qC.slice(), T1 = LATE.map((k) => T1max * k);
  for (const i of [0, 1, 2, 4, 5, 6, 7]) T1[i] = T1body;
  // 收拍：匀减速停下来。往球台方向挥的时候，得在碰到台子之前停住
  let Df = (mv.fwD || 0.26) * (pl.follow || 1) * (0.75 + 0.04 * V);
  { // 沿挥拍方向往前看：拍子最低点低于台面高度 + 余量的地方，不能进到台面范围里
    const lowC = AC.Bc[1] - DIM.blade * Math.sqrt(Math.max(0, 1 - AC.Zf[1] * AC.Zf[1]));
    for (let d = 0.03; d <= Df + 1e-9; d += 0.015) {
      const p = [AC.Bc[0] + vd[0] * d, lowC + Math.min(0, vd[1] * d), AC.Bc[2] + vd[2] * d];
      if (tableLift(p, 0.05) > 0) { Df = Math.max(0.05, d - 0.04); break; }
      // 同样，朝自己身上挥的（钟摆就是往肚子方向摆）得在碰到身体之前停住
      const pb = [AC.Bc[0] + vd[0] * d, AC.Bc[1] + vd[1] * d, AC.Bc[2] + vd[2] * d];
      if (distSeg(pb, AC.P, AC.M) < DIM.rLo + 0.1 || distSeg(pb, AC.M, AC.C) < DIM.rHi + 0.1 || len(sub(pb, hdC)) < DIM.headR + 0.11 || distSeg(pb, hpC.HL, KL0) < DIM.rThigh + 0.09 || distSeg(pb, hpC.HR, KR0) < DIM.rThigh + 0.09) { Df = Math.max(0.05, d - 0.03); break; }
    }
  }
  let T2max = clamp(2 * Df / Math.max(V, 0.8), 0.07, 0.3);
  const QF = qC.slice(), T2 = LATE.map((k) => T2max * (0.6 + 0.4 * k));
  // 远端关节（前臂旋转、手腕）：直接把触球时的转速倒放回去 / 顺延下去，受活动范围限制
  for (const i of [12, 13, 14]) {
    const v = uC[i], av = Math.abs(v), sg = Math.sign(v), r = dofLim(i, g), mg = 3 * DEG;
    if (av < 1e-3) continue;
    const availB = Math.max(0, sg > 0 ? qC[i] - r[0] - mg : r[1] - mg - qC[i]);
    const dB = Math.min(0.6 * av * T1[i] * Math.min(1.2, bsAmp), availB);
    T1[i] = clamp(dB / (0.6 * av), 0.05, T1[i]);
    QB[i] = qC[i] - sg * dB;
    const availF = Math.max(0, sg > 0 ? r[1] - mg - qC[i] : qC[i] - r[0] - mg);
    const dF = Math.min(0.5 * av * T2[i], availF);
    T2[i] = clamp(dF / (0.5 * av), 0.05, T2[i]);
    QF[i] = qC[i] + sg * dF;
  }
  // 引拍的造型（转体、重心、扣腕）叠加上去
  const bsKey = { px: 0, py: 1, pz: 2, lean: 4, curl: 5, side: 6, twist: 7, sF: 8, sR: 9, rot: 10, elb: 11, pro: 12, wf: 13, wd: 14 };
  for (const k of Object.keys(mv.bs)) { const i = bsKey[k]; QB[i] += mv.bs[k] * (k === 'py' ? 1 : DEG) * bsAmp; }
  // 引拍时上身是直起来的：人跟着抛球起来一点，球还在头顶上，抛球的手也才刚收走。弯腰、下沉是球往下落的那段时间里完成的
  for (const i of [0, 2, 5, 6]) QB[i] = QR[i];
  QB[1] = qC[1] + 0.05; QB[4] = clamp(qC[4] - riseT, LIM.lean[0], LIM.lean[1]);
  clampQ(QB, g); legFix(QB);
  // 手臂引到头、开始往前挥的那一刻（-T1max），上身已经从最直的样子往下压了一截。那一刻整个人的姿势：
  // 触球之前任意时刻的姿势（假如引拍到头的姿势是 qb）：准备 → 引拍 → 往前挥。各关节起动的时刻不一样（T1[i]）
  // DT：从准备姿势到引拍到头，各关节默认是直着转过去的；要是那样半路上手肘会穿过身体，就让手臂绕一下——
  // DT[i] 是半路上多转的量，两头都是零，所以起点和终点不变、过渡是平滑的
  const DT = new Array(NQ).fill(0);
  const preAt = (qb, t) => {
    const q = new Array(NQ);
    for (let i = 0; i < NQ; i++) {
      const t1 = T1[i];
      if (t <= tb0) q[i] = QR[i];
      else if (t < -t1) { const u = (t - tb0) / (-t1 - tb0); q[i] = QR[i] + (qb[i] - QR[i]) * smooth(u) + (DT[i] ? DT[i] * sq(4 * u * (1 - u)) : 0); }
      else q[i] = hermite1(qb[i], 0, qC[i], uC[i], t1, (t + t1) / t1);
    }
    return legFix(clampQ(q, g));
  };
  const atSwing0 = (qb) => preAt(qb, -T1max);
  // 从准备姿势引拍、再往前挥的一路上：手臂别和身体打架，别扫到正在落下来的球，别挡住接球人的视线
  const SW_T = [tb0 + 0.4 * (-T1max - tb0), tb0 + 0.75 * (-T1max - tb0), ...[0, 0.18, 0.35, 0.6, 0.78, 0.9, 0.96].map((f) => -T1max * (1 - f))];
  const swingPen = (qb) => {
    let pen = 0;
    for (const t of SW_T) {
      const q = preAt(qb, t);
      pen += posePen(q, false, null).hard;
      if (t < Math.max(tRel + 0.03, -T1max - 1e-9)) continue; // 球还在手里 / 刚离手：下面这些"别碰球"的检查还用不上
      const A = fkCore(q, g), b = ballAt(t);
      pen += sq(Math.max(0, DIM.rUpper + P.R + 0.03 - distSeg(b, A.SR, A.E))) + sq(Math.max(0, DIM.rFore + P.R + 0.025 - distSeg(b, A.E, A.W)));
      const hd = headOf(A, b).head;
      pen += sq(Math.max(0, DIM.headR + P.R + 0.03 - len(sub(b, hd)))) + sq(Math.max(0, DIM.rHi + P.R + 0.02 - distSeg(b, A.M, A.C)));
      pen += sq(discPen(A.Bc, A.Zf, hd, hd, DIM.headR + 0.06)) * 0.5; // 拍子别贴着脸
      // 持拍的手臂、球拍也别挡住接球人看球的视线
      for (const [pt, rr] of [[lerp(A.SR, A.E, 0.5), DIM.rUpper], [A.E, 0.044], [lerp(A.E, A.W, 0.5), DIM.rFore]]) pen += sq(Math.max(0, hideDepth(b, pt, rr) + P.R + 0.006));
      // 球拍不能提前碰到球：球心在拍面圆片范围内时，离拍面至少一个球半径多一点
      { const rl = sub(b, A.Bc), dn = dot(rl, A.Zf), rho = len(sub(rl, [A.Zf[0] * dn, A.Zf[1] * dn, A.Zf[2] * dn]));
        if (rho < DIM.blade + 0.01) pen += sq(Math.max(0, P.R + DIM.rubber + 0.006 - Math.abs(dn))) * 4; }
    }
    return pen;
  };
  { const bo = H2W(AC.F, AC.R, mv.bsOff || [0, 0, 0]);
    const tg = avoid([AC.Bc[0] - dh[0] * Db + bo[0], AC.Bc[1] + mv.bsLift * bsAmp + bo[1], AC.Bc[2] - dh[2] * Db + bo[2]], 0.19);
    // 引拍时肘保持弯着（靠肩和转体把拍子带过去），实在够不到再伸肘
    QB[11] = clamp(qC[11] + 8 * DEG, 88 * DEG, 126 * DEG);
    let q2 = reach(QB, g, tg, 7, { idx: ARM3, map: atSwing0 });
    if (len(sub(fkCore(atSwing0(q2), g).Bc, tg)) > 0.07) { q2 = reach(q2, g, tg, 4, { idx: ARM4, map: atSwing0 }); q2[11] = Math.max(q2[11], 62 * DEG); }
    const rp = repair(legFix(q2), 2, tg, 80, swingPen, { arm: true, map: atSwing0 });
    for (let i = 0; i < NQ; i++) QB[i] = rp.q[i]; }
  if (mv.bsPose) {
    // 有固定引拍造型的发球：球拍放到胸前指定的位置，拍面、拍头按造型摆，再检查不打架
    // 位置按"球落下来的那条线"定：在线和身体之间、比触球点高一些
    const o3 = H2W(AC.F, AC.R, mv.bsPose.at), tg = [pc[0] + o3[0], pc[1] + o3[1] * Math.min(1.2, bsAmp), pc[2] + o3[2]];
    const Zb = norm(H2W(AC.F, AC.R, mv.bsPose.face)), hb = norm(H2W(AC.F, AC.R, mv.bsPose.head));
    let q = QB.slice();
    for (let it = 0; it < 4; it++) {
      q = reach(q, g, tg, 3, { idx: ARM3, map: atSwing0 });
      const qs = atSwing0(q);
      if (!fkCore(qs, g, Zb)) break;                         // 反算前臂旋转和屈腕，让正手面朝 Zb
      q[12] = qs[12]; q[13] = qs[13];
      let bw = q[14], bd = -2;                               // 手腕侧偏：让拍头尽量指向 hb
      for (let w = g.wd[0]; w <= g.wd[1] + 1e-9; w += 6 * DEG) { qs[14] = w; const d = dot(fkCore(qs, g).l, hb); if (d > bd) { bd = d; bw = w; } }
      q[14] = bw; clampQ(q, g);
    }
    for (const i of [12, 13, 14]) T1[i] = T1max * 0.8; // 拍面要早点转过来，不然会在半路上撞到落下来的球
    const rp = repair(legFix(q), 2, tg, 110, swingPen, { arm: true, map: atSwing0 });
    for (let i = 0; i < NQ; i++) QB[i] = rp.q[i];
  }
  { // 准备 → 引拍的半路上要是手臂会和身体打架，就给它找一条绕开的路（见 DT）
    const tsD = [0.2, 0.35, 0.5, 0.65, 0.8].map((u) => tb0 + u * (-T1max - tb0));
    const hard = () => { let p = 0; for (const t of tsD) p += posePen(preAt(QB, t), false, null).hard; return p; };
    let cur = hard();
    if (cur > 1e-7) {
      const DD = [[8, 0.5], [9, 0.5], [10, 0.6], [11, 0.45]];
      let obj = cur;
      for (let it = 0; it < 80 && cur > 1e-7; it++) {
        const save = DT.slice(), sc = it < 40 ? 1 : 0.4, m = 1 + Math.floor(rng() * 2);
        for (let k = 0; k < m; k++) { const [i, r] = DD[Math.floor(rng() * DD.length)]; DT[i] += (rng() * 2 - 1) * r * sc; }
        const h2 = hard(), o2 = h2 + 2e-5 * (sq(DT[8]) + sq(DT[9]) + sq(DT[10]) + sq(DT[11]));
        if (o2 < obj) { obj = o2; cur = h2; } else for (let i = 0; i < NQ; i++) DT[i] = save[i];
      }
    } }
  // 顺势收拍：触球之后每个关节顺着自己正在转的方向匀减速停下来——球拍走的是绕着肘和肩的弧线，不是直线
  // （钟摆发球拍子是朝自己肚子挥的：手臂是贴着身体前面裹过去的，不是直着捅过去）。
  // 一路上和到位时都不能和身体、球台打架；会打架就收短一点，再不行就调整停下来的姿势
  let Tf = 0;
  const QFd = QF.slice(), T2d = T2.slice(); // 手腕和前臂旋转的收拍（上面算好的）
  const PROX = [0, 1, 2, 4, 7, 8, 9, 10, 11];
  const pathBad = (qf, T2x, Tm, fs) => {
    let bad = 0;
    for (const f of fs) {
      const q = new Array(NQ), tt = f * Tm;
      for (let i = 0; i < NQ; i++) q[i] = tt < T2x[i] ? hermite1(qC[i], uC[i], qf[i], 0, T2x[i], tt / T2x[i]) : qf[i];
      bad += posePen(legFix(clampQ(q, g)), false, null).hard;
    }
    return bad;
  };
  { let ok = false;
    for (const scale of [1, 0.7, 0.45, 0.28]) {
      const Tm = clamp(2 * Df * scale / Math.max(V, 0.8), 0.06, 0.3);
      for (let i = 0; i < NQ; i++) { QF[i] = QFd[i]; T2[i] = T2d[i]; }
      for (const i of PROX) { T2[i] = Tm * (0.6 + 0.4 * LATE[i]); QF[i] = qC[i] + uC[i] * T2[i] * 0.5; }
      for (const i of [12, 13, 14]) if (T2d[i] > Tm) { T2[i] = Tm; const d = QFd[i] - qC[i]; QF[i] = qC[i] + Math.sign(d) * Math.min(Math.abs(d), 0.5 * Math.abs(uC[i]) * Tm); }
      legFix(clampQ(QF, g));
      T2max = Tm;
      if (pathBad(QF, T2, Tm, [0.25, 0.5, 0.75, 1]) < 2e-5) { ok = true; break; }
    }
    if (!ok) { // 最短的收拍还是会碰到：调整停下来的姿势，让整条路让开
      const Tm = T2max, rp = repair(QF, false, null, 80, (q) => pathBad(q, T2, Tm, [0.3, 0.6, 0.85]), { arm: true }); // 只动手臂：上身在这么短的时间里改不了多少
      for (let i = 0; i < NQ; i++) QF[i] = rp.q[i];
    } }
  // 假动作：触球后球拍不顺着走，而是拐去别的方向
  if (o.fake) {
    const amp = pl.fakeAmp || 1;
    let D;
    if (o.fake === 'lift') D = [dh[0] * 0.10, 0.24 * amp, dh[2] * 0.10];
    else if (o.fake === 'chop') { const room = clamp(AC.Bc[1] - P.tableY - 0.13, 0, 0.14 * amp); D = [dh[0] * 0.15, -room, dh[2] * 0.15]; }
    else { const toC = norm(sub(lerp(AC.C, AC.M, 0.8), AC.Bc)); D = [toC[0] * 0.16, 0.03 + toC[1] * 0.1, toC[2] * 0.16]; } // 往肚子前面收，不是往脸上收
    Tf = clamp(0.17 * (pl.fakeT || 1) * (0.85 + 0.15 * amp), 0.13, 0.26);
    const tg = avoid([AC.Bc[0] + D[0], AC.Bc[1] + D[1], AC.Bc[2] + D[2]], 0);
    if (o.fake === 'chop') tg[1] = Math.max(tg[1], P.tableY + 0.11);
    const seed = qC.map((x, i) => x + uC[i] * Tf * 0.18);
    for (let i = 0; i < NQ; i++) T2[i] = Tf;
    let rp = repair(legFix(reach(clampQ(seed, g), g, tg, 7, { idx: ARM4 })), false, tg, 70, null, { arm: true });
    if (pathBad(rp.q, T2, Tf, [0.3, 0.6, 0.85]) > 2e-5) rp = repair(rp.q, false, tg, 70, (q) => pathBad(q, T2, Tf, [0.3, 0.6, 0.85]), { arm: true }); // 拐过去的路上别蹭到身体
    for (let i = 0; i < NQ; i++) QF[i] = rp.q[i];
    // 假动作顺带把拍面翻一下：假装上提时合一点，假装下切时亮一点
    if (o.fake !== 'tuck') QF[12] = clamp(QF[12] + (o.fake === 'lift' ? 1 : -1) * face * 22 * DEG, g.pro[0], g.pro[1]);
  }

  // 收势：直起身，胸口转向球台，拍子往身前收。脚不动
  const QE = QF.slice();
  { const NEU = { 8: 28 * DEG, 9: 14 * DEG, 10: 8 * DEG, 11: 98 * DEG, 12: 25 * DEG, 13: 0, 14: -4 * DEG };
    for (const w of [0.6, 0.3, 0.08]) {
      for (let i = 0; i < NQ; i++) QE[i] = QF[i];
      QE[3] = qC[3]; QE[4] = Math.min(qC[4], 30 * DEG); QE[5] = 8 * DEG; QE[6] = 0; QE[7] = mv.stance === 'side' ? 24 * DEG : 0; QE[1] = qC[1] + 0.05; QE[0] = qC[0]; QE[2] = qC[2];
      for (const i of [8, 9, 10, 11, 12, 13, 14]) QE[i] = QF[i] + (NEU[i] - QF[i]) * w;
      const rp = repair(outOfTable(clampQ(QE, g)), false, null, 30);
      for (let i = 0; i < NQ; i++) QE[i] = rp.q[i];
      let bad = rp.hard;
      for (const f of [0.3, 0.55, 0.8]) { const q = QF.map((x, i) => x + (QE[i] - x) * smooth(f)); bad += posePen(legFix(clampQ(q, g)), false, null).hard; }
      if (bad < 2e-5) break;
    } }

  // ---------- 3. 时间线 ----------
  const T2end = Math.max(...T2);
  const te0 = T2end + (pl.hold || 0.1), De = 0.5;
  const track = (t) => {
    if (t <= 0) return preAt(QB, t);
    const q = new Array(NQ);
    for (let i = 0; i < NQ; i++) {
      if (t < T2[i]) q[i] = hermite1(qC[i], uC[i], QF[i], 0, T2[i], t / T2[i]);
      else if (t < te0) q[i] = QF[i];
      else q[i] = QF[i] + (QE[i] - QF[i]) * smooth((t - te0) / De);
    }
    return legFix(clampQ(q, g));
  };
  const coreAt = (t) => {
    const q = track(t), A = fkCore(q, g);
    const bp = ballAt(Math.min(t, 0.03));
    const aim = t < 0.03 ? bp : lerp(bp, recv, smooth((t - 0.03) / 0.32)); // 头：触球前盯球，之后看向对面
    const hd = headOf(A, aim), hp = hipsOf(A.P, A.R);
    return { t, q, A, hd, hp, KL: ik2(hp.HL, ft.AL, ft.poleL, DIM.thigh, DIM.shin).E, KR: ik2(hp.HR, ft.AR, ft.poleR, DIM.thigh, DIM.shin).E };
  };

  // ---------- 4. 抛球的那只手 ----------
  // 托球 → 往上送 → 球离手后顺势再送一小段 → 收走。收到哪儿、肘朝哪、走哪条路：先按这位选手的习惯，打架就换一种
  const freeT = clamp(-tRel - T1max * 0.5 - 0.1, 0.2, pl.freeT || 0.42), freeDelay = Math.min(pl.freeDelay || 0.1, Math.max(0.03, -tRel - T1max - freeT));
  let eDir = null, eT0 = 0; const eDt = 0.008; // 在时间上再抹平一遍之后的"肘朝哪"（见下面）
  const armB = norm(H2W(AC.F, AC.R, ARM_B)), armE = (() => { const e = H2W(AC.F, AC.R, ARM_E), d = dot(e, armB); return norm([e[0] - armB[0] * d, e[1] - armB[1] * d, e[2] - armB[2] * d]); })();
  const poleOf = (dn, sw) => {
    const c = dot(armB, dn), ax = cross(armB, dn);
    let e = armE;
    if (c > -0.995) { const k = dot(ax, armE) / (1 + c), x = cross(ax, armE); e = [armE[0] * c + x[0] + ax[0] * k, armE[1] * c + x[1] + ax[1] * k, armE[2] * c + x[2] + ax[2] * k]; }
    if (sw) { const x = cross(dn, e), cs = Math.cos(sw), sn = Math.sin(sw); e = [e[0] * cs + x[0] * sn, e[1] * cs + x[1] * sn, e[2] * cs + x[2] * sn]; }
    return e;
  };
  const freeArm = (core, plan, useSmooth) => {
    const { A, t } = core, F = A.F, R = A.R;
    const tau = t - tRel;
    const dirH = (() => { const v = [palmLo[0] - A.SL[0], 0, palmLo[2] - A.SL[2]]; const l = Math.hypot(v[0], v[2]); return l > 0.05 ? [v[0] / l, 0, v[2] / l] : F; })();
    const wristOf = (palm) => [palm[0] - dirH[0] * 0.05, palm[1] - 0.014, palm[2] - dirH[2] * 0.05];
    let Wf, relax = 0, d = dirH, p = [0, 1, 0];
    if (tau <= 0) { const b = ballAt(t); Wf = wristOf([b[0], b[1] - P.R - 0.012, b[2]]); }
    else {
      // 以肩为中心：手往哪个方向伸、伸多远分开算（方向从 pA 的方向经过 via 转到 pB 的方向，远近匀着变）。
      // 这样手是绕着肩膀挥过去的，不会从肩膀跟前擦过去——那样手臂得缩成一团，肘也会乱甩
      const S0 = A.SL;
      const sph = (pA, pB, via, u) => {
        const ax = pA[0] - S0[0], ay = pA[1] - S0[1], az = pA[2] - S0[2], bx = pB[0] - S0[0], by = pB[1] - S0[1], bz = pB[2] - S0[2];
        const rA = Math.hypot(ax, ay, az) || 1e-6, rB = Math.hypot(bx, by, bz) || 1e-6;
        // 方向：在球面上从 A 的方向朝着 via 拐向 B 的方向（球面上的二次贝塞尔：两两球面插值再插一次），转得匀、不会突然掉头
        const dA = [ax / rA, ay / rA, az / rA], dB = [bx / rB, by / rB, bz / rB];
        const dv = slerp(slerp(dA, via, u), slerp(via, dB, u), u);
        const lo = Math.min(rA, 0.27), r = Math.max(rA + (rB - rA) * u, lo + (0.27 - lo) * smooth(u / 0.3));
        return [S0[0] + dv[0] * r, S0[1] + dv[1] * r, S0[2] + dv[2] * r];
      };
      const up = followUp * (1 - Math.exp(-tau / 0.09));
      const A0 = wristOf([palmHi[0], palmHi[1] + up, palmHi[2]]);
      const ue = smooth((t - te0) / De); // 收势：这只手也回到身前
      const ro = H2W(F, R, plan.rest.at), rb = plan.rest.rel === 'pel' ? A.P : A.SL, ge = H2W(F, R, [0.20, -0.33, -0.12]);
      let rs = [rb[0] + ro[0], rb[1] + ro[1], rb[2] + ro[2]];
      if (ue > 0) rs = sph(rs, [A.SL[0] + ge[0], A.SL[1] + ge[1], A.SL[2] + ge[2]], norm(H2W(F, R, [-0.1, -0.95, -0.28])), ue); // 从身后回到身前：手是垂着荡过去的
      const u = smooth((tau - freeDelay) / plan.freeT);
      if (u <= 0) Wf = A0;
      else {
        // 收手的路：默认从外侧偏低的地方绕过去；plan.via 可以换成别的绕法
        let via = plan.via ? norm(H2W(F, R, plan.via)) : null;
        if (!via) { const a = norm(sub(A0, S0)), b = norm(sub(rs, S0)), ob = H2W(F, R, [0.1, -0.5, -0.85]); via = norm([a[0] + b[0] + ob[0], a[1] + b[1] + ob[1], a[2] + b[2] + ob[2]]); }
        Wf = sph(A0, rs, via, u);
      }
      Wf[1] += tableLiftSoft(Wf, 0.055);
      relax = smooth(tau / 0.3);
    }
    const u2 = tau <= 0 ? 0 : smooth((tau - freeDelay) / plan.freeT);
    let pole = poleOf(norm(sub(Wf, A.SL)), plan.sw * (1 - u2) + plan.swR * u2);
    if (useSmooth && eDir) { const x = clamp((t - eT0) / eDt, 0, eDir.length - 1.001), k = Math.floor(x); pole = lerp(eDir[k], eDir[k + 1], x - k); }
    let kf = ik2(A.SL, Wf, pole, DIM.upper, DIM.fore);
    // 肘要是贴进自己身体里了，就绕着"肩—手"这条线往外转一点；低到台面以下就往上转。
    // 转多少都跟着"进去了多少"连续地变，不会一帧跳过去
    { let px = 0, py = 0, pz = 0, dep = 0;
      for (const [a, b, rr] of [[A.P, A.M, DIM.rLo + 0.046], [A.M, A.C, DIM.rHi + 0.04]]) {
        const cp = closestOnSeg(kf.E, a, b), ex = kf.E[0] - cp[0], ey = kf.E[1] - cp[1], ez = kf.E[2] - cp[2], d = Math.hypot(ex, ey, ez);
        if (d < rr && d > 1e-6) { const w = (rr - d) / d; px += ex * w; py += ey * w; pz += ez * w; dep += rr - d; }
      }
      if (dep > 0) {
        // 正对着身体顶进去的时候往哪边转都行：一律往自己这一侧的外面转，免得两边来回跳
        px -= A.Rc[0] * dep * 0.7; py -= A.Rc[1] * dep * 0.7; pz -= A.Rc[2] * dep * 0.7;
        const dn = norm(sub(Wf, kf.S)), ev = sub(kf.E, kf.S), dd = dot(ev, dn), e = norm([ev[0] - dn[0] * dd, ev[1] - dn[1] * dd, ev[2] - dn[2] * dd]), tg = cross(dn, e);
        const del = 0.75 * Math.tanh((px * tg[0] + py * tg[1] + pz * tg[2]) * 12), cs = Math.cos(del), sn = Math.sin(del);
        pole = [e[0] * cs + tg[0] * sn, e[1] * cs + tg[1] * sn, e[2] * cs + tg[2] * sn];
        kf = ik2(A.SL, Wf, pole, DIM.upper, DIM.fore);
      } }
    for (let k = 0, add = 0; k < 3; k++) { const d = tableLiftSoft(kf.E, 0.05); if (d < 2e-4) break; add += d * 10; kf = ik2(A.SL, Wf, [pole[0], pole[1] + add, pole[2]], DIM.upper, DIM.fore); }
    if (relax > 0) {
      const fd = norm(sub(Wf, kf.E));
      let pr = [R[0] * 0.55, -0.65, R[2] * 0.55]; // 放松时掌心朝身体内侧偏下
      pr = sub(pr, [fd[0] * dot(pr, fd), fd[1] * dot(pr, fd), fd[2] * dot(pr, fd)]);
      pr = len(pr) > 1e-3 ? norm(pr) : [0, -1, 0];
      d = norm(lerp(dirH, fd, relax)); let pm = lerp([0, 1, 0], pr, relax);
      pm = sub(pm, [d[0] * dot(pm, d), d[1] * dot(pm, d), d[2] * dot(pm, d)]);
      p = len(pm) > 1e-3 ? norm(pm) : pr;
    }
    return { S: kf.S, E: kf.E, W: Wf, d, p, r: cross(p, d), relax, tau };
  };
  const SWIVEL = [[0, 0], [0.5, 0], [-0.45, 0], [0, 0.5], [0, -0.5], [0.5, 0.5]]; // 托球时 / 收走之后，肘在自然朝向上再往外（正）或往里拧多少
  const VIAS = [null, [0.3, -0.5, -0.8], [-0.1, -0.15, -0.98], [0.15, 0.3, -0.94], [0.5, -0.8, -0.35]]; // 收手半路上手臂朝哪儿（身体坐标 [前, 上, 右]）
  const restOrder = (() => {
    const first = (pl.rest && pl.rest[mv.stance]) || (mv.stance === 'side' ? 'wing' : 'low');
    return [first, ...Object.keys(REST).filter((k) => k !== first)];
  })();
  const samp = [];
  { const tA = Math.min(tb0, tRel) - 0.12; for (let t = tA; t < te0 + De; t += 0.03) samp.push(coreAt(t)); samp.push(coreAt(te0 + De));
    for (const t of [0.015, 0.045, 0.075, 0.105, 0.135, 0.165]) { const c = coreAt(t); c.ballOnly = true; samp.push(c); } }
  const planPen = (plan) => {
    let pen = 0;
    for (const c of samp) {
      const fa = freeArm(c, plan), A = c.A, t = c.t;
      const hf = addmul(fa.W, fa.d, 0.05), ff = lerp(fa.E, fa.W, 0.5), hnd = addmul(A.W, A.dd, 0.05);
      if (c.ballOnly) { const b = ballAt(t); pen += sq(Math.max(0, P.R + 0.065 - len(sub(b, hf)))) + sq(Math.max(0, P.R + DIM.rFore + 0.025 - distSeg(b, fa.E, fa.W))) + sq(Math.max(0, P.R + DIM.rUpper + 0.025 - distSeg(b, fa.S, fa.E))); continue; }
      const pts = [[hf, 0.042, 1], [ff, 0.038, 0], [fa.E, 0.044, 0]];
      for (const [pt, rr, isHand] of pts) {
        const rl = sub(pt, A.Bc), dn = dot(rl, A.Zf), rho = len(sub(rl, [A.Zf[0] * dn, A.Zf[1] * dn, A.Zf[2] * dn]));
        if (rho < DIM.blade + rr) pen += sq(Math.max(0, rr + 0.02 - Math.abs(dn)));
        pen += sq(Math.max(0, rr + 0.05 - distSeg(pt, A.E, A.W))) + sq(Math.max(0, rr + 0.055 - len(sub(pt, hnd))));
        if (!isHand || fa.tau > 0.05) pen += sq(Math.max(0, DIM.rLo + rr - distSeg(pt, A.P, A.M))) + sq(Math.max(0, DIM.rHi + rr - 0.01 - distSeg(pt, A.M, A.C))) + sq(Math.max(0, DIM.headR + rr - len(sub(pt, c.hd.head))));
        pen += sq(Math.max(0, DIM.rThigh + rr - 0.01 - distSeg(pt, c.hp.HL, c.KL)));
        if (!isHand || fa.tau > 0.02) pen += sq(tablePen(pt, rr + 0.008));
        if (fa.tau > 0.25) pen += 0.25 * sq(tablePen([pt[0], pt[1] - 0.22, pt[2]], rr)); // 收走之后别悬在台面上方，像撑着球台
        if (fa.tau > 0.07 && t < 0.2) pen += sq(Math.max(0, P.R + rr + 0.02 - len(sub(pt, ballAt(t)))));
      }
      if (fa.tau > 0.07 && t < 0.2) pen += sq(Math.max(0, P.R + DIM.rUpper + 0.02 - distSeg(ballAt(t), fa.S, fa.E)));
      // 规则：球一离手，这只手和手臂就得从"球和接球人之间"让开
      if (fa.tau > 0.1 && t < 0.012) { const b = ballAt(t); for (const [pt, rr] of [[hf, 0.045], [ff, 0.04], [fa.E, 0.046], [lerp(fa.S, fa.E, 0.5), DIM.rUpper + 0.004]]) pen += sq(Math.max(0, hideDepth(b, pt, rr) + P.R + 0.01)); }
      pen += 4 * sq(Math.max(0, 0.3 - len(sub(fa.W, fa.S)))); // 手别缩到自己肩膀跟前
    }
    return pen;
  };
  let plan = null, planBest = 1e9;
  search: for (let ri = 0; ri < restOrder.length; ri++) for (let hi = 0; hi < SWIVEL.length; hi++) for (let ci = 0; ci < VIAS.length; ci++) {
    const pln = { rest: REST[restOrder[ri]], restKey: restOrder[ri], sw: SWIVEL[hi][0], swR: SWIVEL[hi][1], via: VIAS[ci] };
    { // 收手要多久：绕的路越远越久，手最快别超过 6 m/s 左右（抛得低、出手快的时候，手还没收完拍子就挥出来了，也没关系）
      const ro = H2W(AC.F, AC.R, pln.rest.at), rb = pln.rest.rel === 'pel' ? AC.P : AC.SL;
      const vA = [palmHi[0] - AC.SL[0], palmHi[1] + followUp - AC.SL[1], palmHi[2] - AC.SL[2]], vB = [rb[0] + ro[0] - AC.SL[0], rb[1] + ro[1] - AC.SL[1], rb[2] + ro[2] - AC.SL[2]];
      const rA = len(vA), rB = len(vB), dA = norm(vA), dB = norm(vB);
      let via = pln.via ? norm(H2W(AC.F, AC.R, pln.via)) : null;
      if (!via) { const ob = H2W(AC.F, AC.R, [0.1, -0.5, -0.85]); via = norm([dA[0] + dB[0] + ob[0], dA[1] + dB[1] + ob[1], dA[2] + dB[2] + ob[2]]); }
      const arc = 0.8 * (Math.acos(clamp(dot(dA, via), -1, 1)) + Math.acos(clamp(dot(via, dB), -1, 1))) * (rA + rB) / 2 + Math.abs(rA - rB);
      pln.freeT = clamp(Math.max(freeT, 1.5 * arc / 6), 0.2, 0.55); }
    const pn = planPen(pln) + 2e-5 * ri + 1e-5 * ci + 1e-5 * hi;
    if (pn < planBest) { planBest = pn; plan = pln; }
    if (pn < 1e-4) break search;
  }

  // 抛球手的肘：逐帧按"肘朝哪"摆出来之后，再在时间上抹平一遍。
  // 不然手臂伸的方向和"肘朝哪"快要重合的那一下，肘会在一两帧里甩到另一边去
  { eT0 = Math.min(tb0, tRel, o.tPalm0 === undefined ? tRel : o.tPalm0) - 0.2;
    const N = Math.ceil((te0 + De + 0.1 - eT0) / eDt) + 2, dir = new Array(N), axis = new Array(N);
    for (let k = 0; k < N; k++) {
      const fa = freeArm(coreAt(eT0 + k * eDt), plan), dn = norm(sub(fa.W, fa.S)), v = sub(fa.E, fa.S), d = dot(v, dn);
      const pp = [v[0] - dn[0] * d, v[1] - dn[1] * d, v[2] - dn[2] * d], l = len(pp);
      dir[k] = l > 1e-6 ? [pp[0] / l, pp[1] / l, pp[2] / l] : [0, -1, 0]; axis[k] = dn;
    }
    for (let pass = 0; pass < 24; pass++) {
      let a = dir[0];
      for (let k = 1; k < N - 1; k++) {
        const b = dir[k], c2 = dir[k + 1], dn = axis[k];
        let v = [a[0] + 2 * b[0] + c2[0], a[1] + 2 * b[1] + c2[1], a[2] + 2 * b[2] + c2[2]];
        const d = dot(v, dn); v = [v[0] - dn[0] * d, v[1] - dn[1] * d, v[2] - dn[2] * d];
        const l = len(v); a = b;
        if (l > 1e-3) dir[k] = [v[0] / l, v[1] / l, v[2] / l];
      }
    }
    eDir = dir; }

  const at = (t, full) => {
    const c = coreAt(t), A = c.A, F = A.F, R = A.R, fa = freeArm(c, plan, true);
    const J = {
      pelvis: A.P, mid: A.M, chest: A.C, neck: c.hd.neck, head: c.hd.head, look: c.hd.look, F, R, Uc: A.Uc, Fc: A.Fc, Rc: A.Rc, sd1: A.sd1,
      shP: A.SR, elP: A.E, wrP: A.W, shF: fa.S, elF: fa.E, wrF: fa.W,
      hipP: c.hp.HR, hipF: c.hp.HL, knP: c.KR, knF: c.KL, anP: ft.AR, anF: ft.AL, toeP: ft.toeR, toeF: ft.toeL,
      handP: { d: A.dd, p: A.pp, r: A.rr }, handF: { d: fa.d, p: fa.p, r: fa.r },
      racket: { c: A.Bc, l: A.l, Z: A.Zf, n: [A.Zf[0] * face, A.Zf[1] * face, A.Zf[2] * face], K: bladePoint(A, off, face) },
      q: c.q,
    };
    if (full) {
      J.fingP = handPoints(A.W, A.dd, A.pp, A.rr, g.key, null, 0, g.key === 'serve' ? pinchOf(A) : null);
      J.fingF = handPoints(fa.W, fa.d, fa.p, fa.r, 'flat', 'relax', fa.relax);
    }
    if (mir) {
      for (const k of ['pelvis', 'mid', 'chest', 'neck', 'head', 'look', 'F', 'R', 'Uc', 'Fc', 'Rc', 'sd1', 'shP', 'elP', 'wrP', 'shF', 'elF', 'wrF', 'hipP', 'hipF', 'knP', 'knF', 'anP', 'anF', 'toeP', 'toeF']) J[k] = mirror(J[k]);
      for (const hk of ['handP', 'handF']) J[hk] = { d: mirror(J[hk].d), p: mirror(J[hk].p), r: mirror(J[hk].r) };
      const rk = J.racket; J.racket = { c: mirror(rk.c), l: mirror(rk.l), Z: mirror(rk.Z), n: mirror(rk.n), K: mirror(rk.K) };
      if (full) { J.fingP = J.fingP.map((f) => f.map(mirror)); J.fingF = J.fingF.map((f) => f.map(mirror)); }
    }
    J.racket.q = qFromBasis(cross(J.racket.l, J.racket.Z), J.racket.l, J.racket.Z);
    return J;
  };

  const tReady = Math.min(tb0, o.tPalm0 === undefined ? tb0 : o.tPalm0);
  const tDone = te0 + De;
  const out = {
    ok: true, at, tReady, tDone, tSwing0: -T1max, tSwingEnd: T2end, T1max, grip: g.key, restKey: plan.restKey, planPen: planBest,
    racketAt: (t) => { const r = at(t).racket; return { c: r.c, q: r.q }; },
    contactAt: (t) => at(t).racket.K,
    pose: { cost: best.cost, effort: best.effort, worst: best.worst, q: qC.slice(), u: uC.slice() },
  };
  out.stats = validate(out, o);
  return out;
}

// 躯干的截面是扁的（左右宽 hw、前后薄 hd，和画出来的一样）：点 p 离躯干某一段的表面多远（负 = 在里面）
function slabDist(p, a, b, right, hw, hd) {
  const cp = closestOnSeg(p, a, b), vx = p[0] - cp[0], vy = p[1] - cp[1], vz = p[2] - cp[2];
  const lat = vx * right[0] + vy * right[1] + vz * right[2];
  const sag = Math.sqrt(Math.max(0, vx * vx + vy * vy + vz * vz - lat * lat));
  return (Math.hypot(lat / hw, sag / hd) - 1) * hd;
}

/** 逐时刻检查：穿模、球提前碰拍、手进球台…… 返回各项最大穿插深度（米）；where 记下最严重的那一处是什么、在什么时刻 */
function validate(m, o) {
  const st = { body: 0, table: 0, ballBody: 0, early: 0, late: 0, free: 0, leg: 0, nan: 0, hide: -1, hideT: 0, where: {} };
  const up = (k, v, t, tag) => { if (v > st[k]) { st[k] = v; st.where[k] = tag + '@' + t.toFixed(2); } };
  const ts = [];
  const t0 = Math.min(m.tReady, o.tRel) - 0.05, t1 = Math.min(m.tDone, 0.9);
  for (let t = t0; t < -0.42; t += 0.04) ts.push(t);
  for (let t = Math.max(t0, -0.42); t < 0.3; t += 0.012) ts.push(t);
  for (let t = 0.3; t <= t1; t += 0.04) ts.push(t);
  ts.push(0);
  const R = P.R;
  ts.sort((a, b) => a - b);
  for (let ti = 0; ti < ts.length; ti++) {
    const t = ts[ti];
    const J = m.at(t);
    for (const k of ['pelvis', 'chest', 'head', 'elP', 'wrP', 'elF', 'wrF', 'knP', 'knF']) if (!J[k].every(Number.isFinite)) st.nan++;
    const rk = J.racket;
    // 腰腹、胸廓按扁的截面算（前后只有十来厘米厚：钟摆发球小臂就是贴着肚子前面过去的），其余按圆的算
    const Rm = norm(lerp(J.R, J.Rc, 0.45)), TW = [0.15, 0.108], CW = [0.172, 0.114];
    const caps = [['torso', J.pelvis, J.mid, TW[1] + 0.008, Rm, TW], ['chest', J.mid, J.chest, CW[1] + 0.008, J.Rc, CW], ['head', J.head, J.head, DIM.headR], ['thighP', J.hipP, J.knP, DIM.rThigh], ['thighF', J.hipF, J.knF, DIM.rThigh], ['shinP', J.knP, J.anP, DIM.rShin], ['shinF', J.knF, J.anF, DIM.rShin]];
    const hnd = addmul(J.wrP, J.handP.d, 0.05), fm = lerp(J.elP, J.wrP, 0.55);
    for (const [nm, a, b, rr, rt, wd] of caps) {
      up('body', discPen(rk.c, rk.Z, a, b, rr), t, 'racket-' + nm);
      up('body', rt ? 0.042 - slabDist(hnd, a, b, rt, wd[0], wd[1]) : spherePen(hnd, 0.042, a, b, rr), t, 'hand-' + nm);
      up('body', rt ? DIM.rFore - slabDist(fm, a, b, rt, wd[0], wd[1]) : spherePen(fm, DIM.rFore, a, b, rr), t, 'forearm-' + nm);
    }
    up('body', 0.044 - slabDist(J.elP, J.pelvis, J.mid, Rm, TW[0], TW[1]), t, 'elbow-torso'); up('body', spherePen(J.elP, 0.044, J.head, J.head, DIM.headR), t, 'elbow-head');
    // 球台
    { const nz = rk.Z, drop = DIM.blade * Math.sqrt(Math.max(0, 1 - nz[1] * nz[1]));
      up('table', tablePen([rk.c[0], rk.c[1] - drop + 0.012, rk.c[2]], 0.012), t, 'racket');
      up('table', tablePen(hnd, 0.04), t, 'hand'); up('table', tablePen(J.elP, 0.042), t, 'elbow'); up('table', tablePen(J.wrP, 0.036), t, 'wrist');
      up('table', tablePen(J.elF, 0.042), t, 'freeElbow'); up('table', tablePen(J.knP, 0.06), t, 'knee'); up('table', tablePen(J.knF, 0.06), t, 'kneeF'); up('table', tablePen(J.pelvis, 0.14), t, 'pelvis');
      if (t > o.tRel + 0.02) up('table', tablePen(J.wrF, 0.036), t, 'freeWrist'); }
    // 抛球手：别碰球拍、持拍的手臂、自己的身体
    { const hf = addmul(J.wrF, J.handF.d, 0.05), ff = lerp(J.elF, J.wrF, 0.5);
      for (const [nm, pt, rr] of [['hand', hf, 0.04], ['forearm', ff, 0.036], ['elbow', J.elF, 0.042]]) {
        const rl = sub(pt, rk.c), dn = dot(rl, rk.Z), rho = len(sub(rl, [rk.Z[0] * dn, rk.Z[1] * dn, rk.Z[2] * dn]));
        if (rho < DIM.blade + rr * 0.5) up('free', rr + 0.006 - Math.abs(dn), t, nm + '-blade');
        up('free', rr + 0.036 - distSeg(pt, J.elP, J.wrP), t, nm + '-forearm');
        up('free', rr + 0.04 - len(sub(pt, hnd)), t, nm + '-hand');
      }
      if (t > o.tRel + 0.05) { up('free', 0.04 - slabDist(hf, J.pelvis, J.mid, Rm, TW[0], TW[1]), t, 'hand-torso'); up('free', 0.04 - slabDist(hf, J.mid, J.chest, J.Rc, CW[0], CW[1]), t, 'hand-chest'); up('free', DIM.headR + 0.04 - len(sub(hf, J.head)), t, 'hand-head'); }
      up('free', 0.042 - slabDist(J.elF, J.pelvis, J.mid, Rm, TW[0], TW[1]), t, 'elbow-torso'); }
    // 腿有没有被拉直过头
    up('leg', Math.max(len(sub(J.hipP, J.anP)), len(sub(J.hipF, J.anF))) - (DIM.thigh + DIM.shin), t, 'leg');
    // 球
    const b = o.ballAt(t).p;
    if (t > o.tRel + 0.03) {
      up('ballBody', DIM.headR + R - len(sub(b, J.head)), t, 'head');
      up('ballBody', DIM.rLo + R - distSeg(b, J.pelvis, J.mid), t, 'torso'); up('ballBody', DIM.rHi + R - distSeg(b, J.mid, J.chest), t, 'chest');
      up('ballBody', DIM.rUpper + R - distSeg(b, J.shP, J.elP), t, 'upperP'); up('ballBody', DIM.rUpper + R - distSeg(b, J.shF, J.elF), t, 'upperF');
      up('ballBody', DIM.rFore + R - distSeg(b, J.elF, J.wrF), t, 'foreF');
      if (Math.abs(t) > 0.012) up('ballBody', DIM.rFore + R - distSeg(b, J.elP, J.wrP), t, 'foreP');
      if (t > o.tRel + 0.08) up('ballBody', 0.04 + R - len(sub(b, addmul(J.wrF, J.handF.d, 0.05))), t, 'handF');
      if (Math.abs(t) > 0.012) up('ballBody', 0.038 + R - len(sub(b, hnd)), t, 'handP');
      up('ballBody', DIM.rThigh + R - distSeg(b, J.hipP, J.knP), t, 'thigh'); up('ballBody', DIM.rThigh + R - distSeg(b, J.hipF, J.knF), t, 'thighF');
    }
    // 接球的人看不看得见球（抛起之后到触球）：hide = 触球前 0.1 秒内球心被挡住最深的那一下；hideT = 一共有多久看不见球心
    if (t > o.tRel + 0.05 && t <= 0.012) {
      const hf = addmul(J.wrF, J.handF.d, 0.05);
      let worst = -1, tag = '';
      for (const [nm, c, r] of [['head', J.head, DIM.headR], ['chest', J.chest, DIM.rHi], ['chest', lerp(J.mid, J.chest, 0.5), DIM.rHi], ['torso', J.mid, DIM.rLo], ['torso', lerp(J.pelvis, J.mid, 0.5), DIM.rLo],
        ['shF', J.shF, 0.06], ['shP', J.shP, 0.06], ['upperF', lerp(J.shF, J.elF, 0.5), DIM.rUpper], ['elF', J.elF, 0.042], ['foreF', lerp(J.elF, J.wrF, 0.5), DIM.rFore], ['handF', hf, 0.04],
        ['upperP', lerp(J.shP, J.elP, 0.5), DIM.rUpper], ['elP', J.elP, 0.042], ['foreP', lerp(J.elP, J.wrP, 0.5), DIM.rFore]]) {
        const d = hideDepth(b, c, r); if (d > worst) { worst = d; tag = nm; }
      }
      if (worst > 0) { st.hideT += ti > 0 ? Math.min(0.04, t - ts[ti - 1]) : 0.012; if (!st.where.hideT) st.where.hideT = tag + '@' + t.toFixed(2); }
      if (t > -0.1) up('hide', worst, t, tag);
    }
    // 球和球拍：触球前不能提前碰上，触球后不能再碰一次
    if (t > o.tRel + 0.03 && Math.abs(t) > 0.004) {
      const rl = sub(b, rk.c), dn = dot(rl, rk.Z), rho = len(sub(rl, [rk.Z[0] * dn, rk.Z[1] * dn, rk.Z[2] * dn]));
      if (rho < DIM.blade) up(t < 0 ? 'early' : 'late', R + DIM.rubber - Math.abs(dn), t, 'blade');
    }
  }
  return st;
}
