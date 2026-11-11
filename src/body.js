// body.js — 对手的骨架。球拍每一时刻的位置由挥拍运动学给出（那是物理用到的量），
// 这里只负责"人怎么站、手臂怎么够到球拍"：两段骨骼的反向运动学 + 把身体摆到不和球拍打架的位置。
import { P, add, sub, mul, dot, len, norm, lerp, addmul } from './physics.js';
import { qRot, smooth } from './quat.js';

export const BODY = {
  upper: 0.29, fore: 0.265,        // 上臂、前臂
  thigh: 0.42, shin: 0.43,         // 大腿、小腿
  shHalf: 0.19, hipHalf: 0.095,    // 半肩宽、半髋宽
  spine: 0.46, pelvisY: 0.84, ankleY: 0.075,
  grip: 0.165,                     // 拍面中心到手腕的距离（沿拍柄）
  rTorso: 0.14, rHead: 0.115,
};
const DEG = Math.PI / 180;

/** 两段骨骼 IK：肩 S、手腕 H、肘大致朝 pole 方向。够不着时肩膀跟过去一点。 */
export function ik2(S, H, pole, a, b) {
  let d = sub(H, S);
  let L = len(d);
  const mx = a + b - 0.004;
  let S2 = S;
  if (L > mx) { S2 = addmul(H, d, -mx / L); d = sub(H, S2); L = mx; }
  const mn = Math.abs(a - b) + 0.01;
  if (L < mn) L = mn;
  const dn = norm(d);
  const ca = Math.max(-1, Math.min(1, (a * a + L * L - b * b) / (2 * a * L))), sa = Math.sqrt(1 - ca * ca);
  let pp = addmul(pole, dn, -dot(pole, dn));
  if (len(pp) < 1e-4) pp = addmul([0, -1, 0], dn, dn[1]);
  pp = norm(pp);
  return { S: S2, E: add(addmul(S2, dn, a * ca), mul(pp, a * sa)) };
}

// 点到线段的最近点
function closestOnSeg(p, a, b) {
  const ab = sub(b, a), l2 = dot(ab, ab) || 1e-9;
  const t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2));
  return addmul(a, ab, t);
}
const rotY = (v, a) => { const c = Math.cos(a), s = Math.sin(a); return [v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c]; };

/**
 * o: { T(发球类型模板), hand, pc(触球时球心), pose(t)→{b,q} 球拍姿态, face, tReady, tDone, palmAt(t) 抛球手位置, ballAt(t) }
 * 返回 { at(t) → 各关节坐标, stats }
 */
export function makeBody(o) {
  const { T, hand, pc, pose, face } = o;
  const side = T.stance === 'side';
  const F = side ? [-hand, 0, 0] : [0, 0, 1];            // 身体朝向
  const pl = side ? [0, 0, -1] : [-hand, 0, 0];          // 持拍手一侧
  const fl = [-pl[0], 0, -pl[2]];                        // 抛球手一侧
  const rk = (t) => {
    const p = pose(t);
    const n = qRot(p.q, [0, 0, face]), l = qRot(p.q, [0, 1, 0]);
    const c = addmul(p.b, n, -0.0055);
    return { c, l, n, W: addmul(c, l, -BODY.grip) };
  };
  const ts = [];
  const t0 = o.tReady - 0.02, t1 = o.tDone + 0.03;
  for (let i = 0; i <= 26; i++) ts.push(t0 + ((t1 - t0) * i) / 26);
  for (const t of [-0.04, -0.02, 0, 0.02, 0.04]) ts.push(t);
  const RK = ts.map(rk);
  const W0 = rk(0).W;

  const build = (dF, dl, th) => {
    const off = (side ? 0.10 : 0.02) + dl;
    // 弯腰越深，膝盖弯得越多、重心越低
    const py = BODY.pelvisY + 0.03 - 0.09 * Math.max(0, Math.min(1, (th / DEG - 32) / 21));
    const Pv = [pc[0] - F[0] * dF + pl[0] * off, py, pc[2] - F[2] * dF + pl[2] * off];
    const sd = [F[0] * Math.sin(th), Math.cos(th), F[2] * Math.sin(th)];
    const C = addmul(Pv, sd, BODY.spine);
    return { P: Pv, sd, C, Sp: addmul(C, pl, BODY.shHalf), Sf: addmul(C, fl, BODY.shHalf), th, dF, dl };
  };
  // 肩膀跟着手走一点（转肩）
  const shoulderAt = (L, W) => {
    const d = sub(W, W0), dl = len(d);
    const k = dl > 0.3 ? 0.3 / dl : 1;
    return addmul(L.Sp, d, 0.22 * k);
  };
  const headAt = (L, C) => add(addmul(C, L.sd, 0.175), [0, 0.035, 0]);

  const evalCand = (L, pole) => {
    let clip = 0, reach = 0, maxPen = 0;
    const pen = (p, r, a, b, rr) => {
      const cp = closestOnSeg(p, a, b);
      const d = len(sub(p, cp));
      const need = r + rr + 0.012;
      if (d < need) { clip += (need - d) ** 2; if (need - d - 0.012 > maxPen) maxPen = need - d - 0.012; }
    };
    for (let i = 0; i < RK.length; i++) {
      const r = RK[i];
      const Sp = shoulderAt(L, r.W);
      const k = ik2(Sp, r.W, pole, BODY.upper, BODY.fore);
      const C = lerp(k.S, L.Sf, 0.5);
      const hd = headAt(L, C);
      const d = len(sub(r.W, Sp));
      if (d > 0.53) reach += (d - 0.53) ** 2;
      if (d < 0.2) reach += (0.2 - d) ** 2;
      // 拍面是个圆片：朝着身体"侧着"时才按整个半径算
      for (const [pt, rr] of [[k.E, 0.045], [lerp(k.E, r.W, 0.5), 0.04]]) { // 肘和小臂别扎进球台
        const dip = P.tableY + 0.006 - (pt[1] - rr);
        if (dip > 0 && Math.abs(pt[0]) < P.tableW / 2 + rr && pt[2] > -P.tableL / 2 - rr) { clip += dip * dip; if (dip - 0.006 > maxPen) maxPen = dip - 0.006; }
      }
      for (const ob of [[L.P, C, BODY.rTorso], [hd, hd, BODY.rHead]]) {
        const cp = closestOnSeg(r.c, ob[0], ob[1]);
        const u = norm(sub(r.c, cp));
        const eff = 0.076 * Math.sqrt(Math.max(0, 1 - dot(u, r.n) ** 2)) + 0.006;
        pen(r.c, eff, ob[0], ob[1], ob[2]);
        pen(addmul(r.W, r.l, 0.045), 0.045, ob[0], ob[1], ob[2]);
        pen(k.E, 0.045, ob[0], ob[1], ob[2]);
        pen(lerp(k.E, r.W, 0.5), 0.04, ob[0], ob[1], ob[2]);
      }
    }
    return { clip, reach, maxPen };
  };

  // 抛起来的球走的是过 pc 的一条竖线：头和身体得让开；托球、抛球时那只手也得够得着
  const palmLo = o.palmAt(-1e9).p, palmHi = o.palmAt(1e9).p;
  const tossY = o.tossY || [palmLo[1], palmHi[1]];
  const tossEval = (L) => {
    let clip = 0, maxPen = 0, reach = 0;
    const hd = headAt(L, L.C);
    const yn = Math.max(tossY[0], Math.min(tossY[1], hd[1]));
    const dH = Math.hypot(hd[0] - pc[0], hd[1] - yn, hd[2] - pc[2]), needH = BODY.rHead + P.R + 0.05;
    if (dH < needH) { clip += (needH - dH) ** 2; maxPen = Math.max(maxPen, needH - 0.05 - dH); }
    const needT = BODY.rTorso + P.R + 0.03;
    for (let y = tossY[0]; y <= tossY[1] + 1e-9; y += 0.04) {
      const q = [pc[0], y, pc[2]];
      const d = len(sub(q, closestOnSeg(q, L.P, L.C)));
      if (d < needT) { clip += 0.3 * (needT - d) ** 2; maxPen = Math.max(maxPen, needT - 0.03 - d); }
    }
    // 托球的手够不着时，那一侧的肩膀可以往前送几厘米（人本来就会这样），再远就算够不着
    for (const pm of [palmLo, palmHi]) { const d = len(sub(pm, L.Sf)); if (d > 0.6) reach += (d - 0.6) ** 2; if (d < 0.16) reach += (0.16 - d) ** 2; }
    return { clip, maxPen, reach };
  };

  const pole0 = norm([T.elbow[0] * hand, T.elbow[1], T.elbow[2]]);
  const dFs = side ? [0.38, 0.45, 0.52, 0.59, 0.66, 0.73] : [0.36, 0.42, 0.48, 0.54, 0.60, 0.66];
  const dF0 = side ? 0.55 : 0.48, th0 = (side ? 40 : 34) * DEG;
  const cands = [];
  for (const dF of dFs) for (const dl of (side ? [-0.2, -0.1, -0.05, 0, 0.05, 0.1] : [-0.1, -0.05, 0, 0.05, 0.1])) for (const thd of [32, 39, 46, 53]) {
    const L = build(dF, dl, thd * DEG);
    const e = evalCand(L, pole0);
    const te = tossEval(L); L.toss = te;
    // 弯腰太深不好看也不真实：除非不这样就会穿模，否则尽量站得舒展些
    let cost = 400 * e.clip + 300 * e.reach + 400 * te.clip + 300 * te.reach + 1.5 * (dF - dF0) ** 2 + 1.5 * dl * dl + 2.5 * (L.th - th0) ** 2;
    // 人要站在球台外面：骨盆、大腿不能进到台面底下
    if (Math.abs(L.P[0]) < P.tableW / 2 + 0.16 && L.P[2] > -P.tableL / 2 - 0.2) cost += 60 * (L.P[2] + P.tableL / 2 + 0.2) ** 2 + 0.5;
    cands.push({ L, cost, e });
  }
  cands.sort((a, b) => a.cost - b.cost);
  const poles = [pole0, norm(rotY(pole0, 0.6)), norm(rotY(pole0, -0.6)), norm(add(pole0, [0, 0.6, 0])), norm(add(pole0, [0, -0.6, 0])), norm(rotY(pole0, 1.2)), norm(rotY(pole0, -1.2))];
  let best = null;
  for (let i = 0; i < Math.min(6, cands.length); i++) {
    for (let j = 0; j < poles.length; j++) {
      const e = j === 0 ? cands[i].e : evalCand(cands[i].L, poles[j]);
      const cost = cands[i].cost - 400 * cands[i].e.clip - 300 * cands[i].e.reach + 400 * e.clip + 300 * e.reach + 0.03 * (1 - dot(poles[j], pole0));
      if (!best || cost < best.cost) best = { L: cands[i].L, pole: poles[j], cost, e };
    }
  }
  const L = best.L, pole = best.pole;

  // 腿：脚踩实，膝盖朝前弯
  const hipF = addmul(L.P, fl, BODY.hipHalf), hipP = addmul(L.P, pl, BODY.hipHalf);
  const anF = [L.P[0] + fl[0] * 0.27 + F[0] * 0.05, BODY.ankleY, L.P[2] + fl[2] * 0.27 + F[2] * 0.05];
  const anP = [L.P[0] + pl[0] * 0.25 - F[0] * 0.02, BODY.ankleY, L.P[2] + pl[2] * 0.25 - F[2] * 0.02];
  const knF = ik2(hipF, anF, add(F, mul(fl, 0.25)), BODY.thigh, BODY.shin).E;
  const knP = ik2(hipP, anP, add(F, mul(pl, 0.25)), BODY.thigh, BODY.shin).E;
  // 抛完球，这只手收回到身侧（规则也要求它马上离开球和球网之间），肘往后带，不是叉腰。
  // 收到哪儿、走哪条路：按"自然"的先后顺序列几种，挑一个不挡球、不碰球拍和持拍手臂、不进球台的
  const freeAt = (plan, palm) => {
    if (palm.after <= 0) return palm.p;
    const u = smooth(palm.after / 0.42), v = 1 - u;
    const A = add(palm.p, [0, 0.05 * Math.sin(Math.PI * Math.min(1, palm.after / 0.22)), 0]);
    const c = plan.ctrl, r = plan.rest;
    return [v * v * A[0] + 2 * u * v * c[0] + u * u * r[0], v * v * A[1] + 2 * u * v * c[1] + u * u * r[1], v * v * A[2] + 2 * u * v * c[2] + u * u * r[2]];
  };
  const tsF = [];
  { const a = -o.palmAt(0).after; // 球离手的时刻
    for (const t of [-1e6, a - 0.12, a - 0.06, a]) tsF.push(t); // 托球不动、往上送、离手
    for (let i = 1; i <= 14; i++) tsF.push(a + (0.5 * i) / 14);
    for (let i = 0; i <= 12; i++) tsF.push(Math.min(-0.3, a + 0.5) + ((o.tDone + 0.3) * i) / 12); }
  const rel = (f, l, y) => add(add(addmul(L.Sf, F, f), mul(fl, l)), [0, y, 0]);
  const rests = [
    rel(-0.1, 0.04, -0.37),    // 松松地垂在肩膀下面
    rel(-0.2, 0.1, -0.34),     // 往后、往外带一点
    rel(0.0, 0.17, -0.35),     // 垂在外侧
    rel(-0.3, 0.03, -0.26),    // 收到身后
    rel(-0.12, 0.12, -0.22),   // 抬高一点收在腰侧
    rel(0.08, 0.26, -0.28),    // 往外撇开
    rel(-0.32, 0.14, -0.12),   // 肘往后拉，手收到肋边
  ];
  const polesF = [
    norm(add(add(mul(F, -0.9), mul(fl, 0.25)), [0, -0.3, 0])),   // 肘朝后下
    norm(add(add(mul(F, -0.5), mul(fl, 0.8)), [0, -0.1, 0])),    // 肘朝外
    norm(add(add(mul(F, -0.7), mul(fl, 0.3)), [0, 0.5, 0])),     // 肘朝后上
    norm(add(add(mul(F, -0.2), mul(fl, 0.6)), [0, 0.8, 0])),     // 肘朝外上（架起来）
  ];
  const evalPlan = (plan) => {
    let pen = 0, maxPen = 0;
    const why = {};
    let cat = '';
    const hit = (d, need) => { if (d < need) { pen += (need - d) ** 2; if (need - d > maxPen) maxPen = need - d; if (!(need - d < (why[cat] || 0))) why[cat] = need - d; } };
    for (const t of tsF) {
      const palm = o.palmAt(t);
      const Wf = freeAt(plan, palm);
      const kf = ik2(L.Sf, Wf, plan.pole, BODY.upper, BODY.fore);
      const r = rk(t);
      const kp = ik2(shoulderAt(L, r.W), r.W, pole, BODY.upper, BODY.fore);
      const mid = lerp(kf.E, Wf, 0.5);
      for (const [pt, rr, isHand] of [[Wf, 0.045, 1], [mid, 0.04, 0], [kf.E, 0.042, 0]]) {
        if (palm.after > 0 || !isHand) { // 托着球的时候手就在该在的地方，只查肘和小臂
          const rl = sub(pt, r.c), dn = dot(rl, r.n), rho = len(sub(rl, mul(r.n, dn)));
          cat = 'blade'; if (rho < 0.078 + rr) hit(Math.abs(dn), rr + 0.012);
          cat = 'grip'; hit(len(sub(pt, addmul(r.W, r.l, 0.035))), rr + 0.05);
          cat = 'forearm'; hit(len(sub(pt, closestOnSeg(pt, kp.E, r.W))), rr + 0.045);
        }
        const dip = P.tableY + 0.008 - (pt[1] - rr);
        if (!isHand || palm.after > 0) if (dip > 0 && Math.abs(pt[0]) < P.tableW / 2 + rr && pt[2] > -P.tableL / 2 - rr) { pen += dip * dip; if (dip > maxPen) maxPen = dip; why.table = Math.max(why.table || 0, dip); }
      }
      cat = 'fold'; hit(len(sub(Wf, L.Sf)), 0.2); // 手别缩到自己肩膀跟前，那样胳膊会整个折起来
      cat = 'torso';
      if (palm.after > 0) hit(len(sub(Wf, closestOnSeg(Wf, L.P, L.C))), BODY.rTorso + 0.03);
      hit(len(sub(kf.E, closestOnSeg(kf.E, L.P, L.C))), BODY.rTorso + 0.01);
      // 还在空中的球：手和小臂别挡在它落下来的路上
      if (palm.after > 0.06 && t < 0.2) {
        const bp = o.ballAt(t).p;
        cat = 'ball'; hit(len(sub(bp, closestOnSeg(bp, kf.E, Wf))), P.R + 0.05);
        hit(len(sub(bp, closestOnSeg(bp, kf.S, kf.E))), P.R + 0.055);
      }
    }
    return { pen, maxPen, why };
  };
  let plan = null, freeE = null;
  const relPt = o.palmAt(1e9).p;
  search: for (let j = 0; j < polesF.length; j++) for (let i = 0; i < rests.length; i++) {
    const mid = lerp(relPt, rests[i], 0.5);
    // 走直线；或者先往外侧绕一下；或者往外、往上绕
    const ctrls = [mid, add(mid, mul(fl, 0.2)), add(add(mid, mul(fl, 0.24)), [0, 0.12, 0]), add(add(mid, mul(F, 0.12)), mul(fl, 0.28))];
    for (let c = 0; c < ctrls.length; c++) {
      const pl_ = { rest: rests[i], ctrl: ctrls[c], pole: polesF[j] };
      const e = evalPlan(pl_);
      const score = e.pen + 2e-5 * i + 1e-5 * c + 4e-5 * j;
      if (!freeE || score < freeE.score) { freeE = { score, maxPen: e.maxPen, i, c, j, why: e.why }; plan = pl_; }
      if (e.pen < 1e-9) break search;
    }
  }
  const poleFree = plan.pole;

  const at = (t) => {
    const r = rk(t);
    const k = ik2(shoulderAt(L, r.W), r.W, pole, BODY.upper, BODY.fore);
    // 抛球手：托球 → 抛出 → 收回到身侧
    const palm = o.palmAt(t);
    const Wf = freeAt(plan, palm);
    // 兜底：收手的路上如果擦着台面边缘，手抬一点、肘架高一点，不往球台里扎
    if (palm.after > 0) {
      const ox = Math.abs(Wf[0]) - P.tableW / 2, oz = -P.tableL / 2 - Wf[2], out = Math.max(ox, oz);
      const w = 1 - smooth((out - 0.03) / 0.06), need = P.tableY + 0.034 - Wf[1];
      if (w > 0 && need > 0) Wf[1] += need * w;
    }
    let kf = ik2(L.Sf, Wf, poleFree, BODY.upper, BODY.fore);
    for (let k = 1; k <= 3; k++) {
      const E = kf.E;
      if (!(E[1] - 0.042 < P.tableY + 0.004 && Math.abs(E[0]) < P.tableW / 2 + 0.042 && E[2] > -P.tableL / 2 - 0.042)) break;
      kf = ik2(L.Sf, Wf, add(poleFree, [0, 0.7 * k, 0]), BODY.upper, BODY.fore);
    }
    const C = lerp(k.S, kf.S, 0.5);
    const neck = addmul(C, L.sd, 0.07);
    const head = headAt(L, C);
    // 眼睛：触球前盯球，之后看向对面
    const bp = o.ballAt(Math.min(t, 0.04)).p;
    const aim = t < 0.04 ? bp : lerp(bp, [0, 1.0, 1.4], smooth((t - 0.04) / 0.35));
    // 头跟着球转，但脖子有限度：最多低头 50°、仰头 22°（再往上是眼睛在看）
    const lk = sub(aim, head);
    const hx = lk[0] + F[0] * 0.08, hz = lk[2] + F[2] * 0.08, hh = Math.hypot(hx, hz) || 1e-6;
    const pit = Math.max(-50 * DEG, Math.min(22 * DEG, Math.atan2(lk[1], hh)));
    const look = [head[0] + (hx / hh) * Math.cos(pit), head[1] + Math.sin(pit), head[2] + (hz / hh) * Math.cos(pit)];
    return {
      pelvis: L.P, chest: C, neck, head, look, F, pl, fl, sd: L.sd,
      shP: k.S, elP: k.E, wrP: r.W, shF: kf.S, elF: kf.E, wrF: Wf,
      hipP, hipF, knP, knF, anP, anF,
      racket: r,
    };
  };
  return { at, layout: L, pole, stats: { maxPen: best.e.maxPen, reach: best.e.reach, cost: best.cost, tossPen: L.toss.maxPen, tossReach: L.toss.reach, freePen: freeE.maxPen, freePlan: `${freeE.i}${freeE.c}${freeE.j}`, freeWhy: freeE.why } };
}
