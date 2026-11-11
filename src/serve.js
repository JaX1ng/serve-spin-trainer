// serve.js — 发球生成器：由物理引擎反解"合法且打到指定落点"的发球，再正向验证。
// 旋转不是预设的：先定拍面朝向(相对于球速改变量 Δv 的夹角)，由碰撞的角动量关系
//   ω = (κ/R) · Δv × n   （κ = mR²/I = 3/2，n 为拍面法线）
// 得到旋转；球拍速度由碰撞模型反解；最后把"球拍速度 + 拍面"喂回正向碰撞和飞行引擎，
// 以正向结果为准（落点、过网、两跳后的旋转、侧上/侧下的判定全部来自正向模拟）。

import { P, collide, simulate, add, sub, mul, dot, cross, len, norm, lerp, addmul } from './physics.js';
import { qMul, qAxis, qRot, qFromBasis, qSlerp, qFromTo, smooth } from './quat.js';
import { makeBody } from './body.js';
export { qRot } from './quat.js';

export const RPS = 2 * Math.PI;
const DEG = Math.PI / 180;
export const TH_RPS = 5; // 到你面前上下旋分量小于这个值(转/秒)就算"基本不带上下旋"

// ---------- 随机数 ----------
export function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const U = (rng, r) => r[0] + (r[1] - r[0]) * rng();

// ---------- 发球类型（以右手选手为模板；左手 = X 镜像） ----------
// side: 侧旋方向（ω_y 的符号，右手选手）。-1 = 俯视顺时针（碰你的拍子往你右边蹦）
// face: +1 用正手面(红)，-1 用反手面(黑)
// head: 拍头大致指向；pivot: 挥拍圆弧的圆心方向（肘/腕在哪）；elbow: 肘尖大致朝哪
// pre/post: 触球前/后挥多远的倍率；ready: 准备姿势在引拍弧线上的位置（1 = 已经引到头）
// 站姿 side = 侧身（胸口朝自己的右手边，抛球手那一肩对着球台），front = 正对球台
export const TYPES = {
  // 钟摆：小臂吊着，拍头朝下，从身体外侧往肚子方向摆
  pendulum: { name: '正手顺旋转（钟摆）', short: '顺旋转', side: -1, face: 1, stance: 'side', head: [-0.25, -0.95, 0.1], pivot: [0.2, 1, -0.2], elbow: [0.1, 0.55, -0.8], arcScale: 1, pre: 1, post: 0.6, ready: 0.6 },
  // 逆旋转：抬肘，拍头朝下往里扣，从肚子前面往外甩
  reverse: { name: '正手逆旋转', short: '逆旋转', side: 1, face: 1, stance: 'side', head: [0.3, -0.9, -0.25], pivot: [0, 1, 0.15], elbow: [-0.4, 0.5, -0.6], arcScale: 0.85, pre: 0.75, post: 1, ready: 0.8, wrist: 0.8 },
  // 勾手：抬肘，手腕内勾，拍头朝上，拍子先收在胸前，再从里往外、往下勾出去
  hook: { name: '勾手发球', short: '勾手', side: 1, face: 1, stance: 'side', head: [0.15, 0.75, 0.6], pivot: [0.1, 1, -0.25], elbow: [0.1, 0.15, -1], arcScale: 1.0, pre: 1.0, post: 0.5, ready: 0.97, rHead: [0.25, 0.9, 0.3], rFace: [0.87, 0.1, -0.5], back: 0.13, lift: 0.04 },
  // 反手：正对球台，拍子从左腹前往右前方送
  backhand: { name: '反手发球', short: '反手', side: 1, face: -1, stance: 'front', head: [0.9, -0.25, 0.2], pivot: [-0.6, 0.35, -0.7], elbow: [-0.7, -0.1, 0.7], arcScale: 1, pre: 0.85, post: 1, ready: 0.75, back: 0.05, lift: 0.025 },
};
export const TYPE_KEYS = ['pendulum', 'reverse', 'hook', 'backhand'];

// ---------- 选手风格 ----------
// rise: 抛球高度(离手后上升, m)；kappa: 摩擦"薄"的程度(°)，越大越转；arcR: 挥拍圆弧半径(手腕小动作 ↔ 大臂)；
// pre/post: 触球前/后挥过的弧度；clear: 过网高度余量(m)；wMax: 这名选手能发出的最大转速(转/秒)
export const STYLES = {
  standard: { name: '标准中抛', wMax: [40, 54], rise: [0.30, 0.55], kappa: [31, 42], arcR: [0.22, 0.30], pre: [1.2, 1.5], post: [1.0, 1.3], clear: [0.012, 0.04] },
  wrist: { name: '手腕小动作', wMax: [36, 50], rise: [0.22, 0.40], kappa: [29, 41], arcR: [0.13, 0.17], pre: [1.0, 1.3], post: [0.8, 1.1], clear: [0.012, 0.035] },
  arm: { name: '大臂钟摆', wMax: [44, 58], rise: [0.35, 0.65], kappa: [33, 43], arcR: [0.33, 0.42], pre: [1.1, 1.4], post: [1.1, 1.4], clear: [0.015, 0.05] },
  high: { name: '高抛', wMax: [46, 62], rise: [1.0, 1.7], kappa: [35, 43], arcR: [0.20, 0.30], pre: [1.2, 1.5], post: [1.0, 1.3], clear: [0.015, 0.05] },
  low: { name: '低抛快节奏', wMax: [34, 48], rise: [0.17, 0.24], kappa: [27, 40], arcR: [0.16, 0.26], pre: [1.0, 1.3], post: [0.9, 1.2], clear: [0.010, 0.03] },
};
export const STYLE_KEYS = ['standard', 'wrist', 'arm', 'high', 'low'];

// ---------- 旋转分解（相对球的飞行方向） ----------
/** top>0 上旋；side = ω_y (>0 俯视逆时针)；cork = 沿飞行方向的"螺旋"分量 */
export function spinParts(w, v) {
  const hl = Math.hypot(v[0], v[2]) || 1;
  const f = [v[0] / hl, 0, v[2] / hl];
  const h = [f[2], 0, -f[0]]; // y × f
  return { top: dot(w, h), side: w[1], cork: dot(w, f), h, f };
}

// ---------- 拍面规则：由 Δv 和 (chi, kappa) 定出拍面法线与旋转 ----------
// chi: 旋转轴在"垂直于 Δv 的平面"里的方位，>0 偏上旋，<0 偏下旋，0 = 纯侧旋
// kappa: 拍面法线与 Δv 的夹角（越大摩擦越薄、越转；tan(kappa) 不能超过胶皮摩擦系数）
function spinRule(vOut, vb, chi, kappa, s, wMax) {
  const dv = sub(vOut, vb);
  const D = len(dv);
  const d = mul(dv, 1 / D);
  const hl = Math.hypot(d[0], d[2]) || 1e-9;
  const h = [d[2] / hl, 0, -d[0] / hl];
  const e = cross(d, h);
  const cc = Math.cos(chi) * s, sc = Math.sin(chi);
  const wh = [e[0] * cc + h[0] * sc, e[1] * cc + h[1] * sc, e[2] * cc + h[2] * sc];
  // 转速 = (κ/R)·|Δv|·sin(kappa)，但不超过这名选手的上限 wMax（超过就等于"摩擦得厚一点"）
  let sk = Math.sin(kappa);
  const kR = P.kappa / P.R;
  if (wMax && kR * D * sk > wMax) sk = wMax / (kR * D);
  const ck = Math.sqrt(1 - sk * sk);
  const t = cross(wh, d);
  const n = [d[0] * ck + t[0] * sk, d[1] * ck + t[1] * sk, d[2] * ck + t[2] * sk];
  return { w: mul(wh, kR * D * sk), n, dv, D };
}

/** 由想要的球速改变量 Δv 和拍面法线 n，反解球拍速度（咬住不打滑的情形，闭式解） */
export function inverseRacket(vb, dv, n, surf) {
  const dn = dot(dv, n);
  const dt_ = sub(dv, mul(n, dn));
  const dtm = len(dt_);
  const a = surf.e1, b = 1 + surf.e0;
  const x = Math.abs(a) < 1e-9 ? dn / b : (-b + Math.sqrt(Math.max(0, b * b + 4 * a * dn))) / (2 * a);
  const a2 = surf.et1, b2 = 1 + surf.et0, c2 = (1 + P.kappa) * dtm;
  const y = Math.abs(a2) < 1e-9 ? c2 / b2 : (-b2 + Math.sqrt(Math.max(0, b2 * b2 + 4 * a2 * c2))) / (2 * a2);
  return add(vb, add(mul(n, x), mul(dt_, dtm > 1e-9 ? y / dtm : 0)));
}

function solve3(A, b) {
  const [a, bb, c] = A[0], [d, e, f] = A[1], [g, h, i] = A[2];
  const det = a * (e * i - f * h) - bb * (d * i - f * g) + c * (d * h - e * g);
  if (Math.abs(det) < 1e-12) return null;
  const x = (b[0] * (e * i - f * h) - bb * (b[1] * i - f * b[2]) + c * (b[1] * h - e * b[2])) / det;
  const y = (a * (b[1] * i - f * b[2]) - b[0] * (d * i - f * g) + c * (d * b[2] - b[1] * g)) / det;
  const z = (a * (e * b[2] - b[1] * h) - bb * (d * b[2] - b[1] * g) + b[0] * (d * h - e * g)) / det;
  return [x, y, z];
}

// 粗略初值：给定出手的竖直速度 vy0，估一个能打到落点的水平速度（之后交给牛顿法在完整物理下修正）
function guessVxz(pc, tgt, vy0) {
  const h = pc[1] - (P.tableY + P.R);
  const vi = Math.sqrt(vy0 * vy0 + 2 * P.g * h);
  const t1 = (vy0 + vi) / P.g, T2 = (2 * 0.93 * vi) / P.g;
  const dist = tgt.z - pc[2];
  const vz = (dist / (t1 + 0.75 * T2)) * 1.12;
  return [((tgt.x - pc[0]) / dist) * vz, vz];
}

function evalLaunch(pc, v, rule) {
  const sr = rule(v);
  const sim = simulate(pc, v, sr.w, { dt: 0.002, ideal: true, maxBounces: 2, tMax: 2.4, zStop: 1e9 });
  let b1 = null, b2 = null, net = null;
  for (const e of sim.ev) {
    if (e.type === 'bounce') { if (!b1) b1 = e; else if (!b2) b2 = e; }
    else if (e.type === 'net' && !net) net = e;
  }
  if (!b1 || !b2) return null;
  return { b1, b2, net, sr };
}

/** 牛顿法：出手竖直速度固定为 vy0，求水平速度 (vx, vz) 使对方台面的第一跳落在 tgt */
function solveLaunch(pc, rule, tgt, vy0, g0) {
  const resid = (vx, vz) => {
    const r = evalLaunch(pc, [vx, vy0, vz], rule);
    if (!r) return null;
    return { r: [r.b2.p[0] - tgt.x, r.b2.p[2] - tgt.z], info: r };
  };
  let vx = g0[0], vz = g0[1];
  let R0 = resid(vx, vz);
  if (!R0) return null;
  for (let it = 0; it < 14; it++) {
    const rn = Math.hypot(R0.r[0], R0.r[1]);
    if (rn < 3e-4) break;
    const hh = 1e-3;
    const Rx = resid(vx + hh, vz), Rz = resid(vx, vz + hh);
    if (!Rx || !Rz) return null;
    const a = (Rx.r[0] - R0.r[0]) / hh, b = (Rz.r[0] - R0.r[0]) / hh, c = (Rx.r[1] - R0.r[1]) / hh, d = (Rz.r[1] - R0.r[1]) / hh;
    const det = a * d - b * c;
    if (Math.abs(det) < 1e-10) return null;
    let dx = (-R0.r[0] * d + R0.r[1] * b) / det, dz = (-a * R0.r[1] + c * R0.r[0]) / det;
    let sc = 1; const dl = Math.hypot(dx, dz); if (dl > 1.5) sc = 1.5 / dl;
    let okStep = false;
    for (let k = 0; k < 5; k++) {
      const Rn = resid(vx + dx * sc, vz + dz * sc);
      if (Rn && Math.hypot(Rn.r[0], Rn.r[1]) < rn) { vx += dx * sc; vz += dz * sc; R0 = Rn; okStep = true; break; }
      sc *= 0.5;
    }
    if (!okStep) break;
  }
  if (Math.hypot(R0.r[0], R0.r[1]) > 2.5e-3) return null;
  if (R0.info.b1.p[2] > -0.06 || !R0.info.net) return null; // 第一跳必须落在本方台面
  return { v: [vx, vy0, vz], vy0, clear: R0.info.net.clear, info: R0.info };
}

// 出手竖直速度的两族解：'slow' 略向上送出（短球，第一跳靠近球网）；'fast' 向下压（急球，第一跳靠近本方端线）
const VY_SLOW = [0.1, 0.4, 0.7, 1.0, 1.3, 1.6, 1.9];
const VY_FAST = [-0.3, -0.7, -1.1, -1.5, -2.0, -2.5, -3.1, -3.8];

/** 找过网余量 ≈ cT 的发球。先试 branch 指定的那一族，不行再试另一族 */
function findLaunch(pc, rule, tgt, branch, cT) {
  const pick = (list) => {
    const res = [];
    let warm = null;
    for (const vy of list) {
      let r = solveLaunch(pc, rule, tgt, vy, warm || guessVxz(pc, tgt, vy));
      if (!r && warm) r = solveLaunch(pc, rule, tgt, vy, guessVxz(pc, tgt, vy));
      if (r) warm = [r.v[0], r.v[2]];
      res.push(r);
    }
    for (let i = 0; i < res.length; i++) {
      if (!res[i] || res[i].clear < cT) continue;
      if (i === 0 || !res[i - 1]) return res[i].clear <= cT + 0.03 ? res[i] : null;
      let a = res[i - 1], b = res[i]; // a.clear < cT <= b.clear
      for (let k = 0; k < 6; k++) {
        const vm = (a.vy0 + b.vy0) / 2;
        const m = solveLaunch(pc, rule, tgt, vm, [a.v[0], a.v[2]]) || solveLaunch(pc, rule, tgt, vm, [b.v[0], b.v[2]]);
        if (!m) break;
        if (m.clear < cT) a = m; else b = m;
      }
      return b.clear <= cT + 0.03 ? b : null;
    }
    // 这一族到不了 cT：取余量最大的（至少 6 mm）
    let best = null;
    for (const r of res) if (r && r.clear >= 0.006 && (!best || r.clear > best.clear)) best = r;
    return best;
  };
  return branch === 'slow' ? pick(VY_SLOW) || pick(VY_FAST) : pick(VY_FAST) || pick(VY_SLOW);
}

/** 判罚：先落本方台面 → 过网 → 落对方台面 */
export function judge(sim) {
  const out = { legal: false, why: '', b1: null, b2: null, net: null, arrive: null };
  let stage = 0;
  for (const e of sim.ev) {
    if (stage === 0) {
      if (e.type === 'bounce' && e.side === 'own') { out.b1 = e; stage = 1; }
      else if (e.type === 'bounce') { out.why = '没先落本方台面，直接打到了对面'; return out; }
      else if (e.type === 'net') { out.why = e.hit ? '没落台就下网了' : '没先落本方台面就过网了'; return out; }
      else if (e.type === 'floor') { out.why = '球没上台'; return out; }
    } else if (stage === 1) {
      if (e.type === 'net') { out.net = e; if (e.hit) { out.why = '下网'; return out; } stage = 2; }
      else if (e.type === 'bounce') { out.why = '在本方台面弹了两次'; return out; }
      else if (e.type === 'floor') { out.why = '第一跳后没过网，出界'; return out; }
    } else if (stage === 2) {
      if (e.type === 'bounce' && e.side === 'opp') { out.b2 = e; out.legal = true; out.why = '合法发球'; out.arrive = e; return out; }
      else if (e.type === 'floor') { out.why = '过网后出界'; return out; }
    }
  }
  if (!out.why) out.why = stage === 0 ? '球没上台' : stage === 1 ? '没过网' : '过网后没落台';
  return out;
}

export function classOf(topRad) {
  const r = topRad / RPS;
  return r >= TH_RPS ? 'top' : r <= -TH_RPS ? 'back' : 'flat';
}

// ---------- 抛球 ----------
function simToss(xc, zc, yStart, rise, yc) {
  // 离手速度 u0：让离手后上升 rise（含空气阻力修正）
  let u0 = Math.sqrt(2 * P.g * rise);
  const Ta = Math.min(0.22, Math.max(0.12, 0.09 + 0.022 * u0));
  for (let k = 0; k < 2; k++) {
    const up = simulate([xc, 2, zc], [0, u0, 0], [0, 0, 0], { dt: 0.002, tMax: u0 / P.g + 0.06, rec: true, zStop: 1e9 });
    let top = 2;
    for (let i = 1; i < up.S.n * 3; i += 3) if (up.S.p[i] > top) top = up.S.p[i];
    const got = top - 2;
    if (got > 0.01) u0 *= Math.sqrt(rise / got);
  }
  const yRel = yStart + (u0 * Ta) / 2;
  if (yRel + rise < yc + 0.04) return null;
  const sim = simulate([xc, yRel, zc], [0, u0, 0], [0, 0, 0], { dt: 0.001, tMax: 4, rec: true, stopY: yc, zStop: 1e9 });
  if (sim.stop !== 'stopY') return null;
  return { u0, Ta, yStart, yRel, T: sim.end.t, vb: sim.end.v.slice(), S: sim.S, apex: yRel + rise, pc: sim.end.p.slice() };
}

// 从静止高度落下（实验台用）：给定触球瞬间下落速度
function simDrop(xc, zc, vfall, yc) {
  let h = (vfall * vfall) / (2 * P.g);
  if (h < 1e-4) return { u0: 0, Ta: 0.15, yStart: yc, yRel: yc, T: 0, vb: [0, 0, 0], S: null, apex: yc, pc: [xc, yc, zc], drop: true };
  const run = () => simulate([xc, yc + h, zc], [0, 0, 0], [0, 0, 0], { dt: 0.001, tMax: 4, rec: true, stopY: yc, zStop: 1e9 });
  for (let k = 0; k < 3; k++) {
    const got = -run().end.v[1];
    if (got > 1e-3) h *= (vfall / got) ** 2;
  }
  const sim = run();
  return { u0: 0, Ta: 0.15, yStart: yc + h, yRel: yc + h, T: sim.end.t, vb: sim.end.v.slice(), S: sim.S, apex: yc + h, pc: sim.end.p.slice(), drop: true };
}

// ---------- 挥拍运动学 ----------
// 拍面中心沿一段圆弧（绕肘/腕的"钟摆"）运动：圆弧在触球点与球拍速度相切，
// 触球瞬间的拍速、拍面法线严格等于物理反解出来的值。所以"看到的动作"就是"物理用的动作"。
function hermite(p0, m0, p1, m1, T, u) {
  const u2 = u * u, u3 = u2 * u;
  const h00 = 2 * u3 - 3 * u2 + 1, h10 = (u3 - 2 * u2 + u) * T, h01 = -2 * u3 + 3 * u2, h11 = (u3 - u2) * T;
  return [
    h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0],
    h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1],
    h00 * p0[2] + h10 * m0[2] + h01 * p1[2] + h11 * m1[2],
  ];
}

function buildSwing(pc, n, vr, T, hand, st, fake, cls, rng) {
  const V = len(vr);
  const bc = addmul(pc, n, -P.R); // 拍面上的触球点
  const head0 = [T.head[0] * hand, T.head[1], T.head[2]];
  let l = sub(head0, mul(n, dot(head0, n)));
  if (len(l) < 0.15) l = sub([hand * -0.5, -0.5, 0.5], mul(n, dot([hand * -0.5, -0.5, 0.5], n)));
  l = qRot(qAxis(n, (rng() * 20 - 10) * DEG), norm(l));
  const Z = mul(n, T.face);
  const X = cross(l, Z);
  const qc = qFromBasis(X, l, Z);
  if (V < 0.25) {
    const st0 = { b: bc, q: qc };
    return { pose: () => st0, T1: 0.2, T2: 0.2, Tb: 0.2, bc, qc, fake: false, tReady: -0.5, tDone: 0.3, clipTable: false, clipBall: 0 };
  }
  const vh = mul(vr, 1 / V);
  const ph = [T.pivot[0] * hand, T.pivot[1], T.pivot[2]];
  let c = sub(ph, mul(vh, dot(ph, vh)));
  if (len(c) < 0.25) c = sub([0, 1, 0], mul(vh, vh[1]));
  if (len(c) < 0.25) c = sub([0, 0, -1], mul(vh, -vh[2]));
  c = norm(c);
  const L = U(rng, st.arcR) * T.arcScale;
  const Pv = addmul(bc, c, L);
  const ax = norm(cross(vh, c));
  const Om = V / L;
  const pw_ = 2.3;
  let phiS = -U(rng, st.pre) * (T.pre || 1), phiE = U(rng, st.post) * (T.post || 1);
  let T1 = (-phiS * pw_) / Om, T2 = (phiE * pw_) / Om;
  if (T1 > 0.42) { T1 = 0.42; phiS = (-T1 * Om) / pw_; }
  if (T2 > 0.42) { T2 = 0.42; phiE = (T2 * Om) / pw_; }
  const Tb = 0.42;
  // 手腕：引拍时拍面往身体一侧扣（勾手、逆旋转的"内勾"），触球那一刻正好转回到物理要求的拍面，之后顺势展开
  const bodyDir = T.stance === 'side' ? [hand, 0, 0] : [0, 0, -1];
  const wr = (T.wrist || 0) * (Math.sign(dot(X, bodyDir)) || 1) * T.face;
  // 有些发球的准备姿势是固定的（比如勾手：拍头朝上收在胸前、手腕内勾）。引拍从那个姿势出发，
  // 一边沿圆弧下来一边"解开手腕"，到触球那一刻正好转成物理要求的拍面。
  let exAxis = null, exAng = 0;
  if (T.rHead) {
    const lr = norm([T.rHead[0] * hand, T.rHead[1], T.rHead[2]]);
    let nr = [T.rFace[0] * hand, T.rFace[1], T.rFace[2]];
    nr = norm(sub(nr, mul(lr, dot(nr, lr))));
    const Zr = mul(nr, T.face);
    const qd = qFromBasis(cross(lr, Zr), lr, Zr);
    const qs = qMul(qAxis(ax, phiS), qc);
    let qr = qMul([-qs[0], -qs[1], -qs[2], qs[3]], qd);
    if (qr[3] < 0) qr = [-qr[0], -qr[1], -qr[2], -qr[3]];
    const sh = Math.hypot(qr[0], qr[1], qr[2]);
    if (sh > 1e-6) { exAxis = [qr[0] / sh, qr[1] / sh, qr[2] / sh]; exAng = 2 * Math.atan2(sh, qr[3]); }
  }
  // g：1 = 准备姿势，0 = 触球瞬间；触球前后转得最慢，所以触球那一下看到的拍面就是真实拍面
  const cock = (phi) => (phi < 0 ? Math.pow(phi / phiS, 1.5) : -0.3 * Math.pow(phi / phiE, 1.5));
  const arc = (phi) => {
    const r = qAxis(ax, phi);
    let q = qMul(r, qc);
    if (exAxis || wr) {
      const g = cock(phi);
      if (exAxis) q = qMul(q, qAxis(exAxis, exAng * g));
      if (wr) q = qMul(q, qAxis([0, 1, 0], wr * g));
    }
    return { b: add(Pv, qRot(r, sub(bc, Pv))), q };
  };
  const start = arc(phiS);
  // 准备姿势：拍子已经收在引拍位置附近，并且离"球抛起落下的那条竖线"足够远，不会碰到手里的球
  const r0 = arc(phiS * (T.ready || 0.6));
  const rb = r0.b.slice();
  { const dx = rb[0] - pc[0], dz = rb[2] - pc[2], dh = Math.hypot(dx, dz), need = 0.2;
    if (dh < need) { const ux = dh > 1e-3 ? dx / dh : -vh[0], uz = dh > 1e-3 ? dz / dh : -vh[2]; const ul = Math.hypot(ux, uz) || 1; rb[0] = pc[0] + (ux / ul) * need; rb[2] = pc[2] + (uz / ul) * need; } }
  const ready = { b: rb, q: r0.q };
  let fk = null;
  const Tf = 0.14;
  if (fake) {
    const up = cls === 'top' ? -1 : 1; // 真上旋 → 触球后假装下切；真下旋/不转 → 触球后假装上提
    let dh = [vr[0], 0, vr[2]];
    dh = len(dh) > 0.3 ? norm(dh) : [0, 0, 1];
    const room = Math.max(0, Math.min(0.12, bc[1] - P.tableY - 0.1)); // 假装下切也不能把拍子切进台面
    const fb = up > 0 ? add(bc, add(mul(dh, 0.13), [0, 0.22, 0])) : add(bc, add(mul(dh, 0.14), [0, -room, 0.02]));
    let nh = [n[0], 0, n[2]];
    nh = len(nh) > 0.2 ? norm(nh) : [0, 0, 1];
    const el = up > 0 ? -10 * DEG : 55 * DEG;
    const nf = [nh[0] * Math.cos(el), Math.sin(el), nh[2] * Math.cos(el)];
    fk = { b: fb, q: qMul(qFromTo(n, nf), qc) };
  }
  const pose = (t) => {
    if (t <= -T1 - Tb) return ready;
    if (t < -T1) { const u = smooth((t + T1 + Tb) / Tb); return { b: lerp(ready.b, start.b, u), q: qSlerp(ready.q, start.q, u) }; }
    if (t <= 0) return arc(phiS * (1 - Math.pow((t + T1) / T1, pw_)));
    const hon = t < T2 ? arc(phiE * (1 - Math.pow(1 - t / T2, pw_))) : arc(phiE);
    if (!fk) return hon;
    const u = Math.min(1, t / Tf);
    return { b: hermite(bc, vr, fk.b, [0, 0, 0], Tf, u), q: qSlerp(hon.q, fk.q, smooth(u)) };
  };
  const tDone = Math.max(T2, fk ? Tf : 0);
  // 球拍会不会切进球台：拍面是半径 7.6cm 的圆片，看它最低的那一点
  let clipTable = false;
  for (let i = 0; i <= 40; i++) {
    const t = -T1 - Tb + ((tDone + T1 + Tb) * i) / 40;
    const pz = pose(t), nz = qRot(pz.q, [0, 0, 1]);
    const drop = 0.078 * Math.sqrt(Math.max(0, 1 - nz[1] * nz[1])), reach = 0.078;
    if (pz.b[1] - drop < P.tableY + 0.004 && pz.b[2] + reach > -P.tableL / 2 && Math.abs(pz.b[0]) < P.tableW / 2 + reach) { clipTable = true; break; }
    // 握拍的手和手腕（拍头朝上时，手在拍面下面）
    const lz = qRot(pz.q, [0, 1, 0]);
    for (const [dd, rr] of [[0.13, 0.042], [0.175, 0.036]]) {
      const hp = addmul(pz.b, lz, -dd);
      if (hp[1] - rr < P.tableY + 0.004 && hp[2] + rr > -P.tableL / 2 && Math.abs(hp[0]) < P.tableW / 2 + rr) { clipTable = true; break; }
    }
    if (clipTable) break;
  }
  return { pose, T1, T2, Tb, bc, qc, pivot: Pv, L, axis: ax, c, phiS, phiE, fake: !!fk, tReady: -T1 - Tb, tDone, clipTable };
}

// ---------- 采样 ----------
function sampleS(S, t) {
  const f = t / S.dt;
  let i = Math.floor(f);
  if (i < 0) i = 0;
  const last = S.n - 1;
  if (i >= last) {
    const j = last * 3, k = last * 4;
    return { p: [S.p[j], S.p[j + 1], S.p[j + 2]], v: [S.v[j], S.v[j + 1], S.v[j + 2]], w: [S.w[j], S.w[j + 1], S.w[j + 2]], q: [S.q[k], S.q[k + 1], S.q[k + 2], S.q[k + 3]] };
  }
  const a = f - i, j = i * 3, k = i * 4;
  const q = [S.q[k] + (S.q[k + 4] - S.q[k]) * a, S.q[k + 1] + (S.q[k + 5] - S.q[k + 1]) * a, S.q[k + 2] + (S.q[k + 6] - S.q[k + 2]) * a, S.q[k + 3] + (S.q[k + 7] - S.q[k + 3]) * a];
  const ql = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return {
    p: [S.p[j] + (S.p[j + 3] - S.p[j]) * a, S.p[j + 1] + (S.p[j + 4] - S.p[j + 1]) * a, S.p[j + 2] + (S.p[j + 5] - S.p[j + 2]) * a],
    v: [S.v[j], S.v[j + 1], S.v[j + 2]],
    w: [S.w[j], S.w[j + 1], S.w[j + 2]],
    q: [q[0] / ql, q[1] / ql, q[2] / ql, q[3] / ql],
  };
}

/** 把抛球、触球、飞行、挥拍拼成一个可按时间采样的发球记录（t=0 为触球瞬间） */
function assemble(base) {
  const { toss, pc, n, vr, col, sim, T, hand, swing } = base;
  const verdict = judge(sim);
  const launch = col ? spinParts(col.w, col.v) : null;
  let arrive = null;
  if (verdict.arrive) { const a = verdict.arrive; arrive = { ...spinParts(a.wOut, a.vOut), w: a.wOut.slice(), v: a.vOut.slice(), t: a.t, p: a.p.slice() }; }
  const cls = arrive ? classOf(arrive.top) : null;
  const tRel = -toss.T;
  const tPalm0 = tRel - toss.Ta;
  const tStart = Math.min(tPalm0, swing.tReady) - 0.45;
  const tEnd = Math.max(sim.end.t, swing.tDone) + 0.05;
  const q0 = [0, 0, 0, 1];
  const ballAt = (t) => {
    if (t >= 0 && col) { const s = sampleS(sim.S, t); s.phase = 'flight'; return s; }
    if (t >= 0) { // 没碰到拍：球继续下落（实验台）
      const y = pc[1] + toss.vb[1] * t - 0.5 * P.g * t * t;
      return { p: [pc[0], Math.max(P.R, y), pc[2]], v: [0, toss.vb[1] - P.g * t, 0], w: [0, 0, 0], q: q0, phase: 'miss' };
    }
    if (t >= tRel && toss.S) { const s = sampleS(toss.S, t - tRel); s.phase = 'toss'; return s; }
    if (toss.drop || t <= tPalm0) return { p: [pc[0], toss.yStart, pc[2]], v: [0, 0, 0], w: [0, 0, 0], q: q0, phase: 'hand' };
    const tau = t - tPalm0, acc = toss.u0 / toss.Ta;
    return { p: [pc[0], toss.yStart + 0.5 * acc * tau * tau, pc[2]], v: [0, acc * tau, 0], w: [0, 0, 0], q: q0, phase: 'hand' };
  };
  // 抛球手的手掌位置：托着球 → 跟着往上送 → 球离手
  const palmAt = (t) => {
    const b = ballAt(Math.min(t, tRel)).p;
    return { p: [b[0], b[1] - P.R - 0.02, b[2]], after: t - tRel };
  };
  let body = null, clipBall = 0;
  if (!base.lab) {
    body = makeBody({ T, hand, pc, pose: swing.pose, face: T.face, tReady: swing.tReady, tDone: swing.tDone, palmAt, ballAt, tossY: [toss.yStart, toss.apex] });
    // 触球前球有没有提前"穿过"拍面
    for (let t = -swing.T1; t < -0.004; t += 0.004) {
      const pz = swing.pose(t), nz = qRot(pz.q, [0, 0, T.face]), bp = ballAt(t).p;
      const rel = sub(bp, pz.b), dn = dot(rel, nz), rho = len(sub(rel, mul(nz, dn)));
      if (rho < 0.078 && Math.abs(dn) < P.R - 0.004) clipBall = Math.max(clipBall, P.R - Math.abs(dn));
    }
  }
  // 球离拍后别从持拍的手臂里穿过去
  let clipArm = 0;
  if (body && col) {
    for (let t = 0.006; t < 0.16; t += 0.01) {
      const J = body.at(t), bp = ballAt(t).p;
      for (const [a, b, rr] of [[J.shP, J.elP, 0.045], [J.elP, J.wrP, 0.038], [J.shF, J.elF, 0.045], [J.elF, J.wrF, 0.04]]) {
        const ab = sub(b, a), l2 = dot(ab, ab) || 1e-9, u = Math.max(0, Math.min(1, dot(sub(bp, a), ab) / l2));
        const d = len(sub(bp, addmul(a, ab, u)));
        if (d < rr + P.R) clipArm = Math.max(clipArm, rr + P.R - d);
      }
    }
  }
  // 触球分析量
  const an = {};
  if (col) {
    const dv = sub(col.v, toss.vb);
    const sp = spinParts(col.w, col.v);
    const h = sp.h;
    let uf = cross(n, h); uf = norm(uf); // 沿拍面"往上蹭"的方向：拍子相对球沿这个方向走 → 上旋
    const rel = sub(vr, toss.vb);
    an.openDeg = Math.asin(Math.max(-1, Math.min(1, n[1]))) / DEG; // 拍面后仰角：0=立着，90=完全放平朝天，<0 前倾
    an.yawDeg = Math.atan2(n[0], n[2]) / DEG;
    an.epsDeg = Math.atan2(dv[1], Math.hypot(dv[0], dv[2])) / DEG; // Δv 的仰角
    an.kappaDeg = Math.acos(Math.max(-1, Math.min(1, dot(norm(dv), n)))) / DEG;
    an.dv = dv; an.uf = uf;
    an.racketAlong = dot(vr, uf);        // 拍子自己沿拍面向上的速度
    an.ballAlong = -dot(toss.vb, uf);    // 球下落折算成"拍子相对球向上"的速度
    an.relAlong = dot(rel, uf);          // 合计：>0 → 离拍上旋
    an.vrSpeed = len(vr); an.vfall = -toss.vb[1];
    an.regime = col.regime;
    an.ballSpeed = len(col.v);
    // 拍面贴着球"蹭"的方向 = 球拍相对球的速度里，沿拍面的那部分
    const rn = dot(rel, n);
    an.brush = sub(rel, mul(n, rn));
    an.push = rn;                        // 垂直拍面"顶"球的速度
    // 触到球的哪儿：接触点在球心的 -n 方向。side 用接球者看到的左右（+1 = 画面右侧）
    an.spotUp = -n[1];                   // >0 触球的上半部，<0 触球的下半部
    an.spotSide = -dot(n, h);
  }
  return {
    ...base, verdict, legal: verdict.legal, launch, arrive, cls, tRel, tPalm0, tStart, tEnd, ballAt, body, an, clipBall, clipArm,
    racketAt: swing.pose,
    events: sim.ev,
  };
}

function forward(pc, vb, n, vr, surf) {
  const col = collide(vb, [0, 0, 0], n, vr, surf);
  if (!col) return { col: null, sim: { ev: [], S: null, end: { t: 0.7 }, fault: 'miss' } };
  const sim = simulate(pc, col.v, col.w, { dt: 0.001, rec: true, tMax: 2.4, maxOppBounces: 2 });
  return { col, sim };
}

/**
 * 生成一个发球。cfg: { type, hand(+1 右手/-1 左手), intent('top'|'back'|'flat'), pos(-1/0/1), len('short'|'long'), style, fake }
 * 失败返回 null（极少数落点/风格组合在物理上发不出来）。
 */
export function generateServe(cfg, rng = Math.random) {
  for (let attempt = 0; attempt < 16; attempt++) {
    const s = tryGenerate(cfg, rng, attempt);
    if (s) { s.attempts = attempt + 1; return s; }
  }
  return null;
}

function tryGenerate(cfg, rng, attempt) {
  const fail = (why) => { if (cfg._dbg) cfg._dbg.push(why); return null; };
  const T = TYPES[cfg.type], hand = cfg.hand || 1;
  const styleKey = cfg.style && STYLES[cfg.style] ? cfg.style : STYLE_KEYS[Math.floor(rng() * STYLE_KEYS.length)];
  const st = STYLES[styleKey];
  const side = T.side * hand;
  // 位置：pos = 对手站在画面的左(-1)/中(0)/右(+1)（接球者视角，画面右 = +X）；len = 来球近(short)/远(long)
  let tgt;
  if (cfg.target) {
    const jit = cfg.jitter === undefined ? 1 : cfg.jitter;
    tgt = { x: Math.max(-0.62, Math.min(0.62, cfg.target.x + (rng() * 2 - 1) * 0.11 * jit)), z: Math.max(0.28, Math.min(1.27, cfg.target.z + (rng() * 2 - 1) * 0.07 * jit)) };
  } else {
    tgt = { x: (rng() * 2 - 1) * 0.5, z: cfg.len === 'short' ? U(rng, [0.34, 0.52]) : U(rng, [0.85, 1.22]) };
  }
  const lenClass = tgt.z < 0.55 ? 'short' : tgt.z < 0.9 ? 'half' : 'long';
  const xc = cfg.pos === undefined ? hand * U(rng, [0.25, 0.55]) : cfg.pos * 0.42 + (rng() * 2 - 1) * 0.1;
  const hc = (lenClass === 'long' ? U(rng, [0.09, 0.15]) : lenClass === 'half' ? U(rng, [0.10, 0.18]) : U(rng, [0.13, 0.22])) + (T.lift || 0);
  // 长球触球点低，离端线远一点，拍子挥到最低点时还在台外
  const zc = -P.tableL / 2 - (lenClass === 'short' ? U(rng, [0.1, 0.2]) : U(rng, [0.17, 0.28])) - (T.back || 0);
  const yc = P.tableY + hc;
  let rise = U(rng, st.rise);
  if (cfg.intent === 'back' && attempt > 5) rise = Math.max(rise, 0.4 + 0.05 * attempt); // 下旋不够就抛高一点（球落得快，能摩擦得更狠）
  const toss = simToss(xc, zc, P.tableY + U(rng, [0.07, 0.13]), rise, yc);
  if (!toss) return fail('toss');
  const pc = toss.pc, vb = toss.vb;
  const cap = Math.atan(P.rubber.mu) - 2 * DEG;
  let kappa = Math.min(U(rng, st.kappa) * DEG, cap);
  let wMax = U(rng, st.wMax) * RPS;
  let chi;
  if (cfg.intent === 'top') {
    // 偶尔离拍时几乎没有上下旋（"不转"发球）：两跳台面摩擦会让它到位时带上旋，所以仍归在"侧上"这一类
    chi = (rng() < 0.2 ? U(rng, [-6, 10]) : U(rng, [12, 62])) * DEG;
    // 又短又强的上旋几乎发不出来（一落台就往前窜）：短球的上旋成分自然要轻一些
    if (lenClass === 'short') { chi = Math.min(chi, U(rng, [10, 38]) * DEG); wMax *= 0.8; }
  } else if (cfg.intent === 'back') { chi = -U(rng, [58, 80]) * DEG; kappa = Math.min(cap, kappa + (4 + attempt) * DEG); wMax *= 1.1; }
  else chi = -U(rng, [25, 45]) * DEG;
  const mkRule = (ch) => (v) => spinRule(v, vb, ch, kappa, side, wMax);
  // 长球多为"急长"(第一跳靠近本方端线)，偶尔是弧线高一点的慢长球；短球第一跳靠近球网
  const branch = lenClass === 'long' ? 'fast' : lenClass === 'short' ? (rng() < 0.9 ? 'slow' : 'fast') : rng() < 0.6 ? 'slow' : 'fast';
  const cT = U(rng, st.clear);
  let Ln = findLaunch(pc, mkRule(chi), tgt, branch, cT);
  if (!Ln) return fail('launch');

  // 下旋/不转：按"两跳之后到接球者面前"的上下旋分量来调 chi（台面摩擦每跳都会吃掉一部分下旋）
  if (cfg.intent !== 'top') {
    const want = cfg.intent === 'back' ? -U(rng, [6.5, 20]) * RPS : U(rng, [-2.5, 2.5]) * RPS;
    const arrTop = (ch, warm) => {
      const r = solveLaunch(pc, mkRule(ch), tgt, Ln.vy0, [warm[0], warm[2]]);
      if (!r) return null;
      return { top: spinParts(r.info.b2.wOut, r.info.b2.vOut).top, chi: ch, r };
    };
    // 从当前 chi 出发一步步挪（连续延拓），直到把目标夹住，再二分
    let cur = arrTop(chi, Ln.v);
    if (!cur) return fail('chi0');
    const dir = cur.top > want ? -1 : 1; // 上下旋分量随 chi 单调增
    let other = null;
    for (let k = 0; k < 14 && !other; k++) {
      const nx = Math.max(-88 * DEG, Math.min(30 * DEG, cur.chi + dir * 9 * DEG));
      if (nx === cur.chi) break;
      const f = arrTop(nx, cur.r.v);
      if (!f) break;
      if ((f.top - want) * dir >= 0) other = f; else cur = f;
    }
    if (!other) {
      // 夹不住：下旋已经加到头了。只要到位时还算下旋就接受，否则这次作废
      if (cfg.intent === 'back' && cur.top < -(TH_RPS + 1) * RPS) { chi = cur.chi; Ln = cur.r; }
      else return fail('not-enough-back');
    } else {
      let lo = dir < 0 ? other : cur, hi = dir < 0 ? cur : other; // lo.top <= want <= hi.top
      for (let k = 0; k < 7; k++) {
        const fm = arrTop((lo.chi + hi.chi) / 2, lo.r.v);
        if (!fm) break;
        if (fm.top > want) hi = fm; else lo = fm;
      }
      const best = Math.abs(lo.top - want) < Math.abs(hi.top - want) ? lo : hi;
      chi = best.chi; Ln = best.r;
    }
    if (Ln.clear < 0.006 || Ln.clear > 0.075) {
      Ln = findLaunch(pc, mkRule(chi), tgt, branch, cT);
      if (!Ln) return fail('relaunch');
    }
  }

  const sr = Ln.info.sr;
  const n = norm(sr.n);
  const vr = inverseRacket(vb, sr.dv, n, P.rubber);
  // —— 正向：球拍速度 + 拍面 → 碰撞 → 飞行。以下一切以正向结果为准 ——
  const { col, sim } = forward(pc, vb, n, vr, P.rubber);
  if (!col || col.regime !== 'grip') return fail('slip');
  const swing = buildSwing(pc, n, vr, T, hand, st, !!cfg.fake, cfg.intent, rng);
  if (swing.clipTable) return fail('clip-table');
  const s = assemble({ type: cfg.type, hand, style: styleKey, intent: cfg.intent, target: tgt, lenClass, toss, pc, n, vr, col, sim, T, swing, chi, kappa, fake: !!cfg.fake });
  if (!s.legal) return fail('illegal:' + s.verdict.why);
  if (s.verdict.net.clear < 0.004) return fail('net-margin');
  const b1 = s.verdict.b1.p, b2 = s.verdict.b2.p;
  if (Math.abs(b1[0]) > P.tableW / 2 - 0.02 || b1[2] > -0.04) return fail('b1-edge');
  if (Math.abs(b2[0]) > P.tableW / 2 - 0.02 || b2[2] > P.tableL / 2 - 0.02) return fail('b2-edge');
  if (s.cls !== cfg.intent) return fail('class:' + s.cls);
  if (s.clipBall > 0.006) return fail('clip-ball');
  if (s.body.stats.maxPen > 0.02) return fail('clip-body');
  if (s.body.stats.reach > 0.02) return fail('reach');
  if (s.body.stats.tossPen > 0.004) return fail('toss-head');
  if (s.body.stats.tossReach > 0.003) return fail('toss-reach');
  if (s.body.stats.freePen > 0.02) return fail('free-hand');
  if (s.clipArm > 0.008) return fail('clip-arm');
  return s;
}

/** 实验台：手动给拍面和拍速，纯正向模拟（可能不合法——那正是要看的） */
export function manualServe(lab, rng = Math.random) {
  const T = TYPES[lab.type || 'pendulum'];
  const hand = lab.hand || 1;
  const yc = P.tableY + lab.hc;
  const toss = simDrop(lab.xc, lab.zc === undefined ? -1.5 : lab.zc, lab.vfall, yc);
  const pc = toss.pc, vb = toss.vb;
  const a = lab.openDeg * DEG, g = lab.yawDeg * DEG;
  const n = [Math.sin(g) * Math.cos(a), Math.sin(a), Math.cos(g) * Math.cos(a)];
  const vr = lab.vr.slice();
  const surf = lab.rubber || P.rubber;
  const { col, sim } = forward(pc, vb, n, vr, surf);
  const st = { arcR: [0.26, 0.26], pre: [1.25, 1.25], post: [1.1, 1.1] };
  const fixed = () => 0.5;
  const swing = buildSwing(pc, n, vr, T, hand, st, false, 'flat', fixed);
  const s = assemble({ type: lab.type || 'pendulum', hand, style: 'lab', intent: null, target: null, toss, pc, n, vr, col, sim, T, swing, lab: true });
  if (!col) s.verdict.why = '拍子没碰到球（拍面是背着球走的）';
  return s;
}

/** 把一个生成好的发球转成实验台参数（"载入示例"） */
export function toLab(s) {
  return { type: s.type, hand: s.hand, xc: s.pc[0], zc: s.pc[2], hc: s.pc[1] - P.tableY, vfall: -s.toss.vb[1], openDeg: s.an.openDeg, yawDeg: s.an.yawDeg, vr: s.vr.slice() };
}
