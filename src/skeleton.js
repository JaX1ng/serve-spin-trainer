// skeleton.js — 人体骨架：尺寸、关节活动范围、正向运动学。
// 链条：骨盆 → 腰 → 胸 → 肩 → 上臂 → 肘 → 前臂旋转 → 手腕 → 手 → 球拍。
// 每个关节只做它在人身上做得到的转动，所以手肘、手腕不会拧出人做不到的角度。
//
// 约定：这里全部按"右手持拍"的人来算；左手持拍 = 整个画面按 X 镜像（镜像在 motion.js 里做）。
// 坐标同 physics.js：X 横，Y 竖，Z 指向接球的人。
import { sub, dot, cross, len, norm, addmul } from './physics.js';

export const DEG = Math.PI / 180;

export const DIM = {
  hipHalf: 0.10, hipDrop: 0.03,
  thigh: 0.43, shin: 0.42, ankle: 0.085, foot: 0.25,
  lumbar: 0.24, thorax: 0.27, shHalf: 0.185, shDrop: 0.03,
  neck: 0.085, headR: 0.105,
  upper: 0.285, fore: 0.255,
  rLo: 0.125, rHi: 0.135, rUpper: 0.046, rFore: 0.037, rThigh: 0.075, rShin: 0.05, rHand: 0.042,
  blade: 0.079,   // 拍面半径（比实际的 76.5 mm 略大，留一点余量）
  rubber: 0.0055, // 拍面中面到胶皮表面
};

// 关节变量（一个姿势 = 一个长度 NQ 的数组）
//  px py pz : 骨盆位置          yaw : 身体朝向（0 = 正对接球的人）
//  lean : 上身前倾   curl : 含胸   side : 侧弯（朝持拍手一侧为正）   twist : 转体（胸口朝非持拍手一侧转为正）
//  sF sR : 上臂抬起的方向和幅度（朝前 / 朝外侧的分量，合起来就是抬了多少）
//  rot : 上臂自转（内旋为正）   elb : 屈肘   pro : 前臂旋前（掌心朝下的方向为正）
//  wf : 屈腕（朝掌心为正）      wd : 手腕侧偏（朝拇指一侧为正）
export const QI = { px: 0, py: 1, pz: 2, yaw: 3, lean: 4, curl: 5, side: 6, twist: 7, sF: 8, sR: 9, rot: 10, elb: 11, pro: 12, wf: 13, wd: 14 };
export const NQ = 15;

const d2r = (a) => [a[0] * DEG, a[1] * DEG];
// 活动范围（弧度）。数值取常见的人体关节活动度，略收紧一点
export const LIM = {
  lean: d2r([6, 74]), curl: d2r([-4, 30]), side: d2r([-18, 18]), twist: d2r([-50, 50]),
  elev: d2r([0, 142]), rot: d2r([-82, 92]), elb: d2r([8, 146]), pro: d2r([-88, 88]), wf: d2r([-68, 78]),
};

// 握法。gamma：拍柄在掌心里斜着放的角度（拍头偏向拇指一侧）；G：拇指和食指捏住的那一点（拍肩）相对手腕的位置 [d, p, r]；
// wd：拍子在掌面里相对前臂能偏多少（负 = 往小指一侧，也就是"拍头往下掉"）；wk：其中手腕自己承担的比例。
// 发球握法后三指松开，球拍可以绕着拇指和食指捏住的那一点转，所以范围大得多——钟摆发球拍头能垂下来靠的就是这个。
// pro：拍面绕前臂方向能转多少；pk：其中前臂自己旋转承担的比例（发球握法里，拍子还能在拇指和食指之间再拧一点）
export const GRIPS = {
  shake: { key: 'shake', gamma: 36 * DEG, G: [0.0856, 0.024, 0.0328], wd: d2r([-38, 24]), wk: 1, pro: d2r([-88, 88]), pk: 1 },
  serve: { key: 'serve', gamma: 22 * DEG, G: [0.086, 0.023, 0.034], wd: d2r([-104, 40]), wk: 0.38, pro: d2r([-114, 100]), pk: 0.77 },
};
export const NECK = 0.085; // 拍肩到拍面中心

const clamp = (x, a, b) => (x < a ? a : x > b ? b : x);

/**
 * 正向运动学。q：关节变量；g：握法；Zt（可选）：要求掌心（= 正手面）朝这个方向，
 * 这时由它反算出 pro 和 wf 并写回 q（返回 null 表示这个朝向手腕做不到）。
 */
export function fkCore(q, g, Zt) {
  const sy = Math.sin(q[3]), cy = Math.cos(q[3]);
  const F = [sy, 0, cy], R = [-cy, 0, sy];
  const st = Math.sin(q[4]), ct = Math.cos(q[4]), sc = Math.sin(q[5]), cc = Math.cos(q[5]);
  const ss = Math.sin(q[6]), cs = Math.cos(q[6]), sw = Math.sin(q[7]), cw = Math.cos(q[7]);
  const u1x = ss * R[0], u1y = cs, u1z = ss * R[2];
  const sd1 = [ct * u1x + st * F[0], ct * u1y, ct * u1z + st * F[2]];       // 腰椎方向
  const f1 = [-st * u1x + ct * F[0], -st * u1y, -st * u1z + ct * F[2]];     // 与它垂直、朝前
  const r1 = cross(f1, sd1);
  const P = [q[0], q[1], q[2]];
  const M = addmul(P, sd1, DIM.lumbar);
  const Uc = [cc * sd1[0] + sc * f1[0], cc * sd1[1] + sc * f1[1], cc * sd1[2] + sc * f1[2]];
  const Fc0 = [-sc * sd1[0] + cc * f1[0], -sc * sd1[1] + cc * f1[1], -sc * sd1[2] + cc * f1[2]];
  const Fc = [cw * Fc0[0] - sw * r1[0], cw * Fc0[1] - sw * r1[1], cw * Fc0[2] - sw * r1[2]];
  const Rc = [cw * r1[0] + sw * Fc0[0], cw * r1[1] + sw * Fc0[1], cw * r1[2] + sw * Fc0[2]];
  const C = addmul(M, Uc, DIM.thorax);

  // 上臂
  const sF = q[8], sR = q[9];
  const e = Math.hypot(sF, sR);
  const ca = e > 1e-7 ? sF / e : 1, sa = e > 1e-7 ? sR / e : 0;
  const se = Math.sin(e), ce = Math.cos(e);
  const ua = [0, 0, 0], xu0 = [0, 0, 0];
  const k1 = -sa * (1 - ce);
  for (let i = 0; i < 3; i++) {
    ua[i] = -ce * Uc[i] + se * (ca * Fc[i] + sa * Rc[i]);
    xu0[i] = ce * Fc[i] + se * ca * Uc[i] + (ca * Rc[i] - sa * Fc[i]) * k1; // 不自转时，屈肘后前臂指的方向
  }
  // 抬肘时肩膀跟着耸一点、往前送一点（肩胛骨的活动）
  const shUp = 0.045 * Math.max(0, -ce), shFw = 0.035 * se * ca;
  const SR = [0, 0, 0], SL = [0, 0, 0];
  for (let i = 0; i < 3; i++) {
    SR[i] = C[i] - DIM.shDrop * Uc[i] + DIM.shHalf * Rc[i] + shUp * Uc[i] + shFw * Fc[i];
    SL[i] = C[i] - DIM.shDrop * Uc[i] - DIM.shHalf * Rc[i];
  }
  const yu0 = cross(ua, xu0);
  const sr = Math.sin(q[10]), cr = Math.cos(q[10]);
  const xu = [cr * xu0[0] - sr * yu0[0], cr * xu0[1] - sr * yu0[1], cr * xu0[2] - sr * yu0[2]];
  const sb = Math.sin(q[11]), cb = Math.cos(q[11]);
  const fa = [cb * ua[0] + sb * xu[0], cb * ua[1] + sb * xu[1], cb * ua[2] + sb * xu[2]];
  const h = cross(ua, xu);                         // 肘的转轴
  const E = addmul(SR, ua, DIM.upper), W = addmul(E, fa, DIM.fore);
  const p0 = [-h[0], -h[1], -h[2]];                // 前臂不旋转时掌心的朝向
  const k = cross(fa, p0);

  // 手：前臂旋转 → 屈腕 → 侧偏
  if (Zt) {
    const zf = clamp(dot(Zt, fa), -1, 1);
    const wf = -Math.asin(zf), cwf = Math.cos(wf);
    if (cwf < 0.05) return null;
    const p1x = (Zt[0] - zf * fa[0]) / cwf, p1y = (Zt[1] - zf * fa[1]) / cwf, p1z = (Zt[2] - zf * fa[2]) / cwf;
    q[12] = Math.atan2(-(p1x * k[0] + p1y * k[1] + p1z * k[2]), p1x * p0[0] + p1y * p0[1] + p1z * p0[2]);
    q[13] = wf;
  }
  // 球拍的朝向按"整个转动"算；手本身只转前臂承担的那部分（pk）和手腕承担的那部分（wk），其余是球拍在手指间的转动
  const frame = (pro, wf, wdev) => {
    const sp = Math.sin(pro), cp = Math.cos(pro);
    const p1 = [cp * p0[0] - sp * k[0], cp * p0[1] - sp * k[1], cp * p0[2] - sp * k[2]];
    const r1h = cross(fa, p1);
    const sf = Math.sin(wf), cf = Math.cos(wf);
    const d = [cf * fa[0] + sf * p1[0], cf * fa[1] + sf * p1[1], cf * fa[2] + sf * p1[2]];
    const pp = [cf * p1[0] - sf * fa[0], cf * p1[1] - sf * fa[1], cf * p1[2] - sf * fa[2]];
    const sd = Math.sin(wdev), cd = Math.cos(wdev);
    return { dd: [cd * d[0] + sd * r1h[0], cd * d[1] + sd * r1h[1], cd * d[2] + sd * r1h[2]], pp, rr: [cd * r1h[0] - sd * d[0], cd * r1h[1] - sd * d[1], cd * r1h[2] - sd * d[2]] };
  };
  const wdev = q[14] * g.wk, piv = q[14] - wdev;
  const Vf = frame(q[12], q[13], wdev);                                   // 球拍跟的那个坐标架
  const Hf = g.pk === 1 ? Vf : frame(q[12] * g.pk, q[13], wdev);          // 手实际的坐标架
  const dd = Hf.dd, pp = Hf.pp, rr = Hf.rr;
  const sg = Math.sin(g.gamma + piv), cg = Math.cos(g.gamma + piv);
  const l = [cg * Vf.dd[0] + sg * Vf.rr[0], cg * Vf.dd[1] + sg * Vf.rr[1], cg * Vf.dd[2] + sg * Vf.rr[2]];
  const Zf = Vf.pp;
  const Xr = cross(l, Zf);
  // 拍肩捏在拇指和食指之间：位置跟着手，朝向跟着球拍自己
  const Bc = [0, 0, 0];
  for (let i = 0; i < 3; i++) Bc[i] = W[i] + g.G[0] * dd[i] + g.G[1] * pp[i] + g.G[2] * rr[i] + NECK * l[i];
  return { F, R, P, M, C, Uc, Fc, Rc, SR, SL, ua, fa, h, E, W, dd, pp, rr, l, Zf, Xr, Bc, elev: e, sd1 };
}

/** 只算躯干：腰、胸口、非持拍手一侧的肩、头的大致位置（搜索姿势时用来估"抛球那会儿上身在哪"） */
export function trunkLite(px, py, pz, yaw, lean, curl, side, twist) {
  const sy = Math.sin(yaw), cy = Math.cos(yaw), F = [sy, 0, cy], R = [-cy, 0, sy];
  const st = Math.sin(lean), ct = Math.cos(lean), sc = Math.sin(curl), cc = Math.cos(curl), ss = Math.sin(side), cs = Math.cos(side), sw = Math.sin(twist), cw = Math.cos(twist);
  const u1x = ss * R[0], u1y = cs, u1z = ss * R[2];
  const sd1 = [ct * u1x + st * F[0], ct * u1y, ct * u1z + st * F[2]], f1 = [-st * u1x + ct * F[0], -st * u1y, -st * u1z + ct * F[2]];
  const r1 = cross(f1, sd1);
  const M = [px + sd1[0] * DIM.lumbar, py + sd1[1] * DIM.lumbar, pz + sd1[2] * DIM.lumbar];
  const Uc = [cc * sd1[0] + sc * f1[0], cc * sd1[1] + sc * f1[1], cc * sd1[2] + sc * f1[2]];
  const Fc0 = [-sc * sd1[0] + cc * f1[0], -sc * sd1[1] + cc * f1[1], -sc * sd1[2] + cc * f1[2]];
  const Rc = [cw * r1[0] + sw * Fc0[0], cw * r1[1] + sw * Fc0[1], cw * r1[2] + sw * Fc0[2]];
  const C = [M[0] + Uc[0] * DIM.thorax, M[1] + Uc[1] * DIM.thorax, M[2] + Uc[2] * DIM.thorax];
  const SL = [C[0] - DIM.shDrop * Uc[0] - DIM.shHalf * Rc[0], C[1] - DIM.shDrop * Uc[1] - DIM.shHalf * Rc[1], C[2] - DIM.shDrop * Uc[2] - DIM.shHalf * Rc[2]];
  const k = DIM.neck + DIM.headR * 0.8;
  const Fc = [cw * Fc0[0] - sw * r1[0], cw * Fc0[1] - sw * r1[1], cw * Fc0[2] - sw * r1[2]];
  return { P: [px, py, pz], M, C, SL, Uc, Fc, head: [C[0] + Uc[0] * k * 0.92, C[1] + Uc[1] * k * 0.92 + 0.02, C[2] + Uc[2] * k * 0.92] };
}

/** 拍面上某个点（相对拍面中心：沿拍柄方向 cy、横向 cx）在胶皮表面上的位置。face = +1 正手面，-1 反手面 */
export function bladePoint(A, off, face) {
  const s = face * DIM.rubber;
  return [
    A.Bc[0] + off[0] * A.Xr[0] + off[1] * A.l[0] + s * A.Zf[0],
    A.Bc[1] + off[0] * A.Xr[1] + off[1] * A.l[1] + s * A.Zf[1],
    A.Bc[2] + off[0] * A.Xr[2] + off[1] * A.l[2] + s * A.Zf[2],
  ];
}

/** 把一组算好的点整体平移（解出站位之后用） */
export function shiftState(A, s) {
  for (const key of ['P', 'M', 'C', 'SR', 'SL', 'E', 'W', 'Bc']) { const v = A[key]; v[0] += s[0]; v[1] += s[1]; v[2] += s[2]; }
}

/** 已知躯干朝向和"上臂、前臂各指哪儿"，反算肩和肘的四个角度 */
export function dirsToAngles(A, ua, fa) {
  const { Uc, Fc, Rc } = A;
  const e = Math.acos(clamp(-dot(ua, Uc), -1, 1));
  const a = e > 1e-6 ? Math.atan2(dot(ua, Rc), dot(ua, Fc)) : 0;
  const ca = Math.cos(a), sa = Math.sin(a), se = Math.sin(e), ce = Math.cos(e), k1 = -sa * (1 - ce);
  const xu0 = [0, 0, 0];
  for (let i = 0; i < 3; i++) xu0[i] = ce * Fc[i] + se * ca * Uc[i] + (ca * Rc[i] - sa * Fc[i]) * k1;
  const yu0 = cross(ua, xu0);
  let xu = sub(fa, [ua[0] * dot(fa, ua), ua[1] * dot(fa, ua), ua[2] * dot(fa, ua)]);
  if (len(xu) < 1e-6) xu = xu0; else xu = norm(xu);
  return { sF: e * ca, sR: e * sa, rot: Math.atan2(-dot(xu, yu0), dot(xu, xu0)), elb: Math.acos(clamp(dot(ua, fa), -1, 1)) };
}

/** 两段骨骼的反向运动学：根 S、末端 H、关节大致朝 pole。够不着时把根往末端拉一点 */
export function ik2(S, H, pole, a, b) {
  let dv = sub(H, S);
  let L = len(dv);
  const mx = a + b - 0.004;
  let S2 = S;
  if (L > mx) { S2 = addmul(H, dv, -mx / L); dv = sub(H, S2); L = mx; }
  const mn = Math.abs(a - b) + 0.01;
  if (L < mn) L = mn;
  const dn = norm(dv);
  const ca = clamp((a * a + L * L - b * b) / (2 * a * L), -1, 1), sa = Math.sqrt(1 - ca * ca);
  let pp = addmul(pole, dn, -dot(pole, dn));
  if (len(pp) < 1e-4) pp = addmul([0, -1, 0], dn, dn[1]);
  pp = norm(pp);
  return { S: S2, E: [S2[0] + dn[0] * a * ca + pp[0] * a * sa, S2[1] + dn[1] * a * ca + pp[1] * a * sa, S2[2] + dn[2] * a * ca + pp[2] * a * sa] };
}

export function closestOnSeg(p, a, b) {
  const abx = b[0] - a[0], aby = b[1] - a[1], abz = b[2] - a[2];
  const l2 = abx * abx + aby * aby + abz * abz || 1e-9;
  const t = clamp(((p[0] - a[0]) * abx + (p[1] - a[1]) * aby + (p[2] - a[2]) * abz) / l2, 0, 1);
  return [a[0] + abx * t, a[1] + aby * t, a[2] + abz * t];
}
export const distSeg = (p, a, b) => { const c = closestOnSeg(p, a, b); return Math.hypot(p[0] - c[0], p[1] - c[1], p[2] - c[2]); };

// ---------- 手指 ----------
// 手的局部坐标：d = 指尖方向，p = 掌心朝向，r = 拇指一侧。原点在手腕。
// 每根手指：掌指关节的位置 mcp[d, p, r]，三节指骨的长度，粗细
const FINGERS = [
  { mcp: [0.088, -0.002, 0.027], len: [0.040, 0.024, 0.020], rad: 0.0086 },  // 食指
  { mcp: [0.092, -0.002, 0.008], len: [0.044, 0.027, 0.021], rad: 0.0088 },  // 中指
  { mcp: [0.088, -0.002, -0.011], len: [0.041, 0.025, 0.020], rad: 0.0082 }, // 无名指
  { mcp: [0.080, -0.002, -0.029], len: [0.032, 0.019, 0.018], rad: 0.0073 }, // 小指
];
const THUMB = { base: [0.022, 0.006, 0.034], len: [0.040, 0.030, 0.026], rad: 0.0098 };
export const FINGER_RAD = [THUMB.rad, ...FINGERS.map((f) => f.rad)];

// 手型：ab = 每根手指朝拇指一侧张开的角度；curl = 三个指关节各弯多少；thumb = 拇指三节各指的方向 [d, p, r]
const HANDS = {
  // 横拍握法：食指贴在反手面下沿，另外三指绕过拍柄，拇指压在正手面
  shake: {
    ab: [3, 4, -2, -8],
    curl: [[10, 4, 2], [86, 96, 42], [88, 98, 44], [90, 100, 46]],
    thumb: [[0.45, 0.80, 0.40], [0.90, 0.02, -0.44], [0.95, -0.02, -0.30]],
  },
  // 发球握法：拇指和食指捏住拍肩，另外三指松开半握
  serve: {
    ab: [16, 3, -2, -8],
    curl: [[24, 16, 10], [52, 56, 30], [58, 60, 32], [62, 62, 34]],
    thumb: [[0.52, 0.76, 0.40], [0.93, 0.02, -0.36], [0.96, -0.04, -0.26]],
  },
  // 托球：手掌摊平，四指并拢，拇指自然张开（规则要求球放在不握拢的手掌上）
  flat: {
    ab: [5, 1, -3, -8],
    curl: [[-4, 3, 2], [-5, 3, 2], [-4, 3, 2], [-2, 4, 3]],
    thumb: [[0.55, 0.08, 0.83], [0.72, 0.03, 0.69], [0.80, 0.0, 0.60]],
  },
  // 放松：手指自然半弯
  relax: {
    ab: [6, 1, -3, -8],
    curl: [[22, 30, 18], [26, 34, 20], [30, 36, 22], [34, 38, 24]],
    thumb: [[0.62, 0.38, 0.69], [0.82, 0.30, 0.49], [0.88, 0.28, 0.38]],
  },
};
const lerpN = (a, b, f) => a + (b - a) * f;
function handPose(a, b, f) {
  const A = HANDS[a];
  if (!b || f <= 0) return A;
  const B = HANDS[b];
  if (f >= 1) return B;
  return {
    ab: A.ab.map((x, i) => lerpN(x, B.ab[i], f)),
    curl: A.curl.map((c, i) => c.map((x, j) => lerpN(x, B.curl[i][j], f))),
    thumb: A.thumb.map((c, i) => c.map((x, j) => lerpN(x, B.thumb[i][j], f))),
  };
}

/**
 * 一只手的全部指节位置。W：手腕；d/p/r：手的三个方向；pose/pose2/f：手型（可在两种之间过渡）
 * pinch（可选）：{ thumb, index, n }——拇指尖、食指尖要按在球拍上的位置（世界坐标），n = 正手面朝向。
 *   发球握法里球拍会在手指间转动，拇指和食指得跟着球拍走，不然会从拍面里穿出来
 * 返回 5 根手指（拇指在前），每根 4 个点
 */
export function handPoints(W, d, p, r, pose, pose2, f, pinch) {
  const H = handPose(pose, pose2, f || 0);
  const at = (c) => [W[0] + c[0] * d[0] + c[1] * p[0] + c[2] * r[0], W[1] + c[0] * d[1] + c[1] * p[1] + c[2] * r[1], W[2] + c[0] * d[2] + c[1] * p[2] + c[2] * r[2]];
  // 指根 → 指尖之间拱起来的三节（指尖贴在拍面上，指节朝拍面外侧拱）
  const arch = (base, tip, nx, ny, nz, h) => [base,
    [base[0] + (tip[0] - base[0]) * 0.42 + nx * h, base[1] + (tip[1] - base[1]) * 0.42 + ny * h, base[2] + (tip[2] - base[2]) * 0.42 + nz * h],
    [base[0] + (tip[0] - base[0]) * 0.76 + nx * h * 0.7, base[1] + (tip[1] - base[1]) * 0.76 + ny * h * 0.7, base[2] + (tip[2] - base[2]) * 0.76 + nz * h * 0.7], tip];
  const out = [];
  // 拇指
  if (pinch) out.push(arch(at(THUMB.base), pinch.thumb, pinch.n[0], pinch.n[1], pinch.n[2], 0.014));
  else { let c = THUMB.base.slice(); const pts = [at(c)];
    for (let k = 0; k < 3; k++) { const v = H.thumb[k], vl = Math.hypot(v[0], v[1], v[2]) || 1, L = THUMB.len[k] / vl; c = [c[0] + v[0] * L, c[1] + v[1] * L, c[2] + v[2] * L]; pts.push(at(c)); }
    out.push(pts); }
  for (let i = 0; i < 4; i++) {
    const fg = FINGERS[i];
    if (i === 0 && pinch) { out.push(arch(at(fg.mcp), pinch.index, -pinch.n[0], -pinch.n[1], -pinch.n[2], 0.012)); continue; }
    const ab = H.ab[i] * DEG, ca = Math.cos(ab), sa = Math.sin(ab);
    let c = fg.mcp.slice(); const pts = [at(c)];
    let ang = 0;
    for (let k = 0; k < 3; k++) {
      ang += H.curl[i][k] * DEG;
      const cu = Math.cos(ang), su = Math.sin(ang), L = fg.len[k];
      c = [c[0] + cu * ca * L, c[1] + su * L, c[2] + cu * sa * L];
      pts.push(at(c));
    }
    out.push(pts);
  }
  return out;
}

/** 发球握法：拇指尖按在正手面靠拍肩的地方，食指尖勾在反手面。A = fkCore 的结果 */
export function pinchOf(A) {
  const pt = (x, y, z) => [A.Bc[0] + x * A.Xr[0] + y * A.l[0] + z * A.Zf[0], A.Bc[1] + x * A.Xr[1] + y * A.l[1] + z * A.Zf[1], A.Bc[2] + x * A.Xr[2] + y * A.l[2] + z * A.Zf[2]];
  return { thumb: pt(0.006, -0.050, DIM.rubber + 0.0085), index: pt(-0.012, -0.046, -(DIM.rubber + 0.008)), n: A.Zf };
}
