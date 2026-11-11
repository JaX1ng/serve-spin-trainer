// physics.js — 乒乓球物理引擎：球拍/台面碰撞 + 空气阻力 + 马格努斯力 + 台面反弹
// 纯数值，不依赖任何渲染库。坐标：X = 横（平行球网，接球者右手边为 +X），
// Y = 竖（向上），Z = 前后（沿球台长边，指向接球者为 +Z）。球台中心在原点上方。
//
// 参数出处：
//  [Conti 2026] Conti et al., "Physics Models for Sim-to-Real Transfer in Professional-Level
//               Robot Table Tennis", arXiv:2606.28805 —— 阻力/马格努斯系数表、台面与球拍接触模型
//  [Nakashima 2010] 台面库仑摩擦滑动/滚动判别、胶皮切向弹性模型
//  胶皮摩擦上限 mu=1.0 为本模型的假设值（文献多按"始终咬住"处理），实验台里可调。

export const P = {
  m: 0.0027,            // 球质量 kg
  R: 0.02,              // 球半径 m (40mm 球)
  g: 9.81,
  rho: 1.2,             // 空气密度 kg/m^3
  kappa: 1.5,           // m R^2 / I，薄球壳 I = 2/3 m R^2
  tableY: 0.76, tableL: 2.74, tableW: 1.525,
  netH: 0.1525, netHalfW: 0.915,
  // 接触面参数：e = e0 + e1*|法向速度|；切向恢复 et = et0 + et1*|切向接触速度|；es = 绕法线自旋保留率
  table:  { mu: 0.25, e0: 0.98,  e1: -0.020, et0: 0,     et1: 0,      es: 1 },
  rubber: { mu: 1.0,  e0: 0.878, e1: -0.020, et0: 0.819, et1: -0.010, es: 0.805 },
};

const AREA = Math.PI * P.R * P.R;
const VOL = (4 / 3) * Math.PI * P.R ** 3;

// ---------- 小向量工具（数组 [x,y,z]） ----------
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const len = (a) => Math.hypot(a[0], a[1], a[2]);
export const norm = (a) => { const l = len(a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
export const lerp = (a, b, f) => [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
export const addmul = (a, b, s) => [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s];

// ---------- 空气动力系数 [Conti 2026] ----------
function pw(xs, ys, x) {
  const n = xs.length;
  if (x <= xs[0]) return ys[0];
  if (x >= xs[n - 1]) return ys[n - 1];
  for (let i = 0; i < n - 1; i++) {
    if (x <= xs[i + 1]) return ys[i] + ((x - xs[i]) / (xs[i + 1] - xs[i])) * (ys[i + 1] - ys[i]);
  }
  return ys[n - 1];
}
const CD_V = [2.5, 7.5, 12.5, 17.5];
const CD_SP = [[0, 0.3, 0.7, 0.95, 1.5, 2.0], [0, 0.4, 0.75, 1.1, 1.3, 2.0], [0, 0.4, 0.62, 0.95, 1.3, 2.0], [0, 0.4, 0.5, 0.84, 1.2, 2.0]];
const CD_C = [[0.55, 0.55, 0.55, 0.55, 0.55, 0.55], [0.49, 0.49, 0.55, 0.48, 0.53, 0.53], [0.47, 0.47, 0.53, 0.41, 0.48, 0.48], [0.47, 0.47, 0.51, 0.37, 0.45, 0.45]];
/** 阻力系数 C_D(速度, 旋转比 Sp = Rω/v) */
export function cdOf(v, sp) {
  if (v <= CD_V[0]) return pw(CD_SP[0], CD_C[0], sp);
  if (v >= CD_V[3]) return pw(CD_SP[3], CD_C[3], sp);
  for (let i = 0; i < 3; i++) {
    if (v <= CD_V[i + 1]) {
      const a = pw(CD_SP[i], CD_C[i], sp), b = pw(CD_SP[i + 1], CD_C[i + 1], sp);
      return a + ((v - CD_V[i]) / (CD_V[i + 1] - CD_V[i])) * (b - a);
    }
  }
  return 0.5;
}
// 每行: v_eff, m1, s, ω_b, a, b, c   ——  ω<=ω_b: C_M = m1 ω + s； 否则 C_M = a ω² + b ω + c；C_M >= 0
const CM = [
  [2.0, 0, 0.08, 150, -1.852e-7, -1.296e-4, 0.0983],
  [3.5, -1.1e-3, 0.31, 200, -1.667e-7, -3.333e-5, 0.1],
  [7.5, -8.0e-4, 0.37, 350, -2.0e-7, 1.7e-4, 0.0587],
  [10.5, -6.58e-4, 0.375, 440, -2.604e-7, 3.646e-4, -0.0225],
  [13.5, -5.6e-4, 0.383, 550, -3.571e-7, 5.357e-4, -0.0893],
  [17.0, -4.48e-4, 0.371, 650, -1.0e-7, 2.3e-4, -0.0375],
];
function cmRow(r, w) {
  const c = w <= r[3] ? r[1] * w + r[2] : r[4] * w * w + r[5] * w + r[6];
  return c > 0 ? c : 0;
}
/** 马格努斯系数 C_M(速度, 转速 rad/s)，力 F = C_M ρ V_球 (ω × v) */
export function cmOf(v, w) {
  if (v <= CM[0][0]) return cmRow(CM[0], w);
  const n = CM.length;
  if (v >= CM[n - 1][0]) return cmRow(CM[n - 1], w);
  for (let i = 0; i < n - 1; i++) {
    if (v <= CM[i + 1][0]) {
      const a = cmRow(CM[i], w), b = cmRow(CM[i + 1], w);
      return a + ((v - CM[i][0]) / (CM[i + 1][0] - CM[i][0])) * (b - a);
    }
  }
  return 0.07;
}

/** 飞行中的加速度：重力 + 阻力 + 马格努斯。out 为长度 3 的数组 */
export function accel(vx, vy, vz, wx, wy, wz, out) {
  const v = Math.hypot(vx, vy, vz);
  const w = Math.hypot(wx, wy, wz);
  const sp = v > 1e-6 ? (P.R * w) / v : 0;
  const kd = (0.5 * P.rho * AREA * cdOf(v, sp)) / P.m;
  const km = (cmOf(v, w) * P.rho * VOL) / P.m;
  out[0] = -kd * v * vx + km * (wy * vz - wz * vy);
  out[1] = -P.g - kd * v * vy + km * (wz * vx - wx * vz);
  out[2] = -kd * v * vz + km * (wx * vy - wy * vx);
  return out;
}

const _a1 = [0, 0, 0], _a2 = [0, 0, 0], _a3 = [0, 0, 0], _a4 = [0, 0, 0];
/** 四阶 Runge-Kutta 走一步，原地更新 p、v（飞行中 ω 视为不变） */
export function stepRK4(p, v, w, dt) {
  const h = dt / 2;
  accel(v[0], v[1], v[2], w[0], w[1], w[2], _a1);
  const v2x = v[0] + _a1[0] * h, v2y = v[1] + _a1[1] * h, v2z = v[2] + _a1[2] * h;
  accel(v2x, v2y, v2z, w[0], w[1], w[2], _a2);
  const v3x = v[0] + _a2[0] * h, v3y = v[1] + _a2[1] * h, v3z = v[2] + _a2[2] * h;
  accel(v3x, v3y, v3z, w[0], w[1], w[2], _a3);
  const v4x = v[0] + _a3[0] * dt, v4y = v[1] + _a3[1] * dt, v4z = v[2] + _a3[2] * dt;
  accel(v4x, v4y, v4z, w[0], w[1], w[2], _a4);
  const s = dt / 6;
  p[0] += s * (v[0] + 2 * v2x + 2 * v3x + v4x);
  p[1] += s * (v[1] + 2 * v2y + 2 * v3y + v4y);
  p[2] += s * (v[2] + 2 * v2z + 2 * v3z + v4z);
  v[0] += s * (_a1[0] + 2 * _a2[0] + 2 * _a3[0] + _a4[0]);
  v[1] += s * (_a1[1] + 2 * _a2[1] + 2 * _a3[1] + _a4[1]);
  v[2] += s * (_a1[2] + 2 * _a2[2] + 2 * _a3[2] + _a4[2]);
}

/**
 * 球与一个"很重的表面"（球拍 / 台面）的瞬时碰撞。冲量-动量模型：
 *   法向：J_n = -(1+e) w_n
 *   切向：咬住时 J_t = -α w_t，α = (1+e_t)/(1+κ)；超过库仑上限 μ J_n 则打滑，J_t = μ J_n
 *   自旋：Δω = (m/I) r × J，r = -R n
 * @param v 球速（碰前） @param w 球角速度（碰前） @param n 表面单位法线（指向球心）
 * @param u 表面在接触点的速度 @param surf 接触面参数
 * @returns null 表示没有相向运动（碰不上）
 */
export function collide(v, w, n, u, surf) {
  const R = P.R, k = P.kappa;
  // 球上接触点相对表面的速度：v + ω × (-R n) - u
  const cx = v[0] - R * (w[1] * n[2] - w[2] * n[1]) - u[0];
  const cy = v[1] - R * (w[2] * n[0] - w[0] * n[2]) - u[1];
  const cz = v[2] - R * (w[0] * n[1] - w[1] * n[0]) - u[2];
  const wn = cx * n[0] + cy * n[1] + cz * n[2];
  if (wn >= 0) return null;
  const tx = cx - wn * n[0], ty = cy - wn * n[1], tz = cz - wn * n[2];
  const vt = Math.hypot(tx, ty, tz);
  const e = Math.min(1, Math.max(0.3, surf.e0 + surf.e1 * -wn));
  const jn = -(1 + e) * wn; // 单位质量的法向冲量
  const et = Math.min(1, Math.max(0, surf.et0 + surf.et1 * vt));
  const alpha = (1 + et) / (1 + k);
  let jt = alpha * vt;
  let regime = 'grip';
  if (jt > surf.mu * jn) { jt = surf.mu * jn; regime = 'slip'; }
  const s = vt > 1e-9 ? -jt / vt : 0;
  const Jtx = s * tx, Jty = s * ty, Jtz = s * tz;
  const v2 = [v[0] + jn * n[0] + Jtx, v[1] + jn * n[1] + Jty, v[2] + jn * n[2] + Jtz];
  // Δω = -(κ/R) n × J_t
  const f = -k / R;
  const w2 = [
    w[0] + f * (n[1] * Jtz - n[2] * Jty),
    w[1] + f * (n[2] * Jtx - n[0] * Jtz),
    w[2] + f * (n[0] * Jty - n[1] * Jtx),
  ];
  if (surf.es !== 1) {
    const sn = (w2[0] * n[0] + w2[1] * n[1] + w2[2] * n[2]) * (surf.es - 1);
    w2[0] += sn * n[0]; w2[1] += sn * n[1]; w2[2] += sn * n[2];
  }
  return { v: v2, w: w2, jn, jt, regime, e, et, alpha, wn, vt, vtDir: vt > 1e-9 ? [tx / vt, ty / vt, tz / vt] : [0, 0, 0] };
}

// 四元数 [x,y,z,w]：把绕 ω 的转动累积到姿态上
export function spinQuat(q, w, dt) {
  const wl = Math.hypot(w[0], w[1], w[2]);
  const ang = wl * dt;
  if (ang < 1e-12) return q;
  const s = Math.sin(ang / 2) / wl, c = Math.cos(ang / 2);
  const ax = w[0] * s, ay = w[1] * s, az = w[2] * s;
  // dq ⊗ q
  return [
    c * q[0] + ax * q[3] + ay * q[2] - az * q[1],
    c * q[1] - ax * q[2] + ay * q[3] + az * q[0],
    c * q[2] + ax * q[1] - ay * q[0] + az * q[3],
    c * q[3] - ax * q[0] - ay * q[1] - az * q[2],
  ];
}

/**
 * 从 (p0, v0, w0) 开始积分飞行，处理台面反弹、球网、落地。
 * o: { dt, tMax, ideal(无限大台面且无网，用于反解), rec(记录轨迹), maxBounces, maxOppBounces, zStop, stopY, q0 }
 */
export function simulate(p0, v0, w0, o = {}) {
  const dt = o.dt || 0.001, tMax = o.tMax || 2.5;
  const ideal = !!o.ideal, rec = !!o.rec;
  const maxOpp = o.maxOppBounces || 2;
  const zStop = o.zStop === undefined ? P.tableL / 2 + 0.9 : o.zStop;
  const yb = P.tableY + P.R;
  const hw = P.tableW / 2, hl = P.tableL / 2;
  let p = [p0[0], p0[1], p0[2]], v = [v0[0], v0[1], v0[2]], w = [w0[0], w0[1], w0[2]];
  let q = o.q0 ? o.q0.slice() : [0, 0, 0, 1];
  let t = 0, bounces = 0, oppB = 0, netDone = false, fault = null, stop = null;
  const ev = [];
  const S = rec ? { dt, t0: 0, n: 0, p: [], v: [], w: [], q: [] } : null;
  const push = () => { if (S) { S.p.push(p[0], p[1], p[2]); S.v.push(v[0], v[1], v[2]); S.w.push(w[0], w[1], w[2]); S.q.push(q[0], q[1], q[2], q[3]); S.n++; } };
  push();
  const maxSteps = Math.ceil(tMax / dt);
  for (let i = 0; i < maxSteps; i++) {
    const px = p[0], py = p[1], pz = p[2], vx = v[0], vy = v[1], vz = v[2];
    stepRK4(p, v, w, dt);
    // 抛球阶段：下落到指定高度就停
    if (o.stopY !== undefined && v[1] < 0 && p[1] <= o.stopY && py > o.stopY) {
      const f = (py - o.stopY) / (py - p[1]);
      p = [px + (p[0] - px) * f, o.stopY, pz + (p[2] - pz) * f];
      v = [vx + (v[0] - vx) * f, vy + (v[1] - vy) * f, vz + (v[2] - vz) * f];
      q = spinQuat(q, w, dt * f);
      t += dt * f; stop = 'stopY';
      break;
    }
    // 过网平面 z = 0
    if (!netDone && pz < 0 && p[2] >= 0) {
      const f = pz / (pz - p[2]);
      const yc = py + (p[1] - py) * f, xc = px + (p[0] - px) * f;
      const clear = yc - P.R - (P.tableY + P.netH);
      const hit = !ideal && Math.abs(xc) <= P.netHalfW && clear < 0 && yc + P.R > P.tableY;
      ev.push({ type: 'net', t: t + dt * f, x: xc, y: yc, clear, hit });
      if (hit) {
        fault = fault || 'net';
        p = [xc, yc, -P.R * 0.5];
        v = [v[0] * 0.3, Math.min(v[1], 0) * 0.5, -Math.abs(v[2]) * 0.08];
        w = [w[0] * 0.3, w[1] * 0.3, w[2] * 0.3];
      } else netDone = true;
    }
    // 台面
    if (p[1] <= yb && py >= yb - 1e-9 && v[1] < 0 && py > p[1]) {
      const f = (py - yb) / (py - p[1]);
      const xi = px + (p[0] - px) * f, zi = pz + (p[2] - pz) * f;
      if (ideal || (Math.abs(xi) <= hw && Math.abs(zi) <= hl)) {
        const vi = [vx + (v[0] - vx) * f, vy + (v[1] - vy) * f, vz + (v[2] - vz) * f];
        const res = collide(vi, w, [0, 1, 0], [0, 0, 0], P.table);
        if (res) {
          const side = zi < 0 ? 'own' : 'opp';
          ev.push({ type: 'bounce', t: t + dt * f, p: [xi, yb, zi], side, vIn: vi, wIn: w.slice(), vOut: res.v.slice(), wOut: res.w.slice(), regime: res.regime, n: bounces });
          bounces++; if (side === 'opp') oppB++;
          p = [xi, yb, zi]; v = res.v; w = res.w;
          const rem = dt * (1 - f);
          if (rem > 1e-9) stepRK4(p, v, w, rem);
          if (p[1] < yb) p[1] = yb;
          if (res.v[1] < 0.12) { stop = 'rest'; }
        }
      }
    }
    q = spinQuat(q, w, dt);
    t += dt;
    push();
    if (stop) break;
    if (p[1] - P.R <= 0) { ev.push({ type: 'floor', t, p: p.slice() }); stop = 'floor'; break; }
    if (o.maxBounces && bounces >= o.maxBounces) { stop = 'bounces'; break; }
    if (oppB >= maxOpp) { stop = 'opp'; break; }
    if (p[2] > zStop) { stop = 'z'; break; }
  }
  if (S) { S.p = Float32Array.from(S.p); S.v = Float32Array.from(S.v); S.w = Float32Array.from(S.w); S.q = Float32Array.from(S.q); }
  return { ev, S, end: { t, p, v, w, q }, fault, stop };
}
