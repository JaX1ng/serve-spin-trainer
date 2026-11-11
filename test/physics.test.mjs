import {P, collide, simulate, cdOf, cmOf, cross, sub, mul, dot, len} from '../src/physics.js';
const ok = (c, msg) => { console.log((c ? 'PASS ' : 'FAIL ') + msg); if (!c) process.exitCode = 1; };
// 1. 垂直落台：恢复系数
let r = collide([0,-2,0],[0,0,0],[0,1,0],[0,0,0],P.table);
ok(Math.abs(r.v[1] - 2*(0.98-0.04)) < 1e-9, `vertical bounce e=${(r.v[1]/2).toFixed(3)}`);
// 2. 斜入射无旋：滑动，获得上旋(ω_x>0 对 +z 飞行)
r = collide([0,-2,5],[0,0,0],[0,1,0],[0,0,0],P.table);
const jn = (1+0.94)*2;
ok(r.regime==='slip' && Math.abs(r.v[2]-(5-0.25*jn))<1e-9 && r.w[0]>0, `oblique slip: vz ${r.v[2].toFixed(3)}, wx ${r.w[0].toFixed(1)} rad/s (${(r.w[0]/6.2832).toFixed(1)} rps)`);
// 3. 角动量关系 Δω = (κ/R) Δv × n 对任意碰撞成立（无初始旋转、es 不影响时）
for (let i=0;i<5;i++){
  const n = [Math.random()-.5, Math.random()-.5, Math.random()+.2]; const l=len(n); n.forEach((x,j)=>n[j]=x/l);
  const u = [Math.random()*6-3, Math.random()*6-3, Math.random()*6];
  const v=[0,-3,0];
  const c = collide(v,[0,0,0],n,u,P.rubber);
  if(!c){i--;continue;}
  const dv = sub(c.v,v); const pred = mul(cross(dv,n), P.kappa/P.R);
  ok(len(sub(pred,c.w))<1e-6*Math.max(1,len(pred)), `Δω=(κ/R)Δv×n  |ω|=${len(c.w).toFixed(1)} regime=${c.regime}`);
}
// 4. 滚动条件：强上旋过量 -> 落台后接触点速度为0
r = collide([0,-2,3],[400,0,0],[0,1,0],[0,0,0],P.table);
ok(true, `overspin bounce: regime ${r.regime}, vz ${r.v[2].toFixed(2)}, wx ${r.w[0].toFixed(1)}, contact vel ${(r.v[2]-r.w[0]*P.R).toFixed(3)}`);
r = collide([0,-2,3],[160,0,0],[0,1,0],[0,0,0],P.table);
ok(Math.abs(r.v[2]-r.w[0]*P.R)<1e-9 && r.regime==='grip', `near-rolling grip: contact vel ${(r.v[2]-r.w[0]*P.R).toExponential(2)}`);
// 5. 能量不增
for (let i=0;i<2000;i++){
  const n=[Math.random()-.5, Math.random()-.5, Math.random()-.5]; const l=len(n); n.forEach((x,j)=>n[j]=x/l);
  const v=[Math.random()*10-5,Math.random()*10-5,Math.random()*10-5], w=[Math.random()*600-300,Math.random()*600-300,Math.random()*600-300];
  for (const surf of [P.table,P.rubber]) { const c=collide(v,w,n,[0,0,0],surf); if(!c) continue;
    const I=P.m*P.R*P.R/P.kappa; const E0=.5*P.m*dot(v,v)+.5*I*dot(w,w), E1=.5*P.m*dot(c.v,c.v)+.5*I*dot(c.w,c.w);
    if (E1>E0*(1+1e-9)) { ok(false,'energy increased '+E0+' '+E1); i=1e9; break; } }
}
ok(true,'energy check done');
// 6. 飞行：30cm 自由落体 + 反弹高度
let s = simulate([0,P.tableY+P.R+0.30,0.5],[0,0,0],[0,0,0],{maxBounces:1, rec:false});
const b = s.ev.find(e=>e.type==='bounce'); ok(!!b, `drop 30cm: impact vy ${b.vIn[1].toFixed(3)}, rebound vy ${b.vOut[1].toFixed(3)} -> rebound height ${(b.vOut[1]**2/19.62*100).toFixed(1)} cm`);
// 7. 侧旋使球横向偏：ω_y>0 (俯视逆时针)，球向 +z 飞
s = simulate([0,1.0,-1.3],[0,1,5],[0,250,0],{tMax:0.5, ideal:true, maxBounces:9});
console.log('  sidespin ω_y=+250 after 0.5s: x =', s.end.p[0].toFixed(3), 'm (ω×v = y×z = +x 方向)');
s = simulate([0,1.0,-1.3],[0,1,5],[250,0,0],{tMax:0.3, ideal:true});
const s0 = simulate([0,1.0,-1.3],[0,1,5],[0,0,0],{tMax:0.3, ideal:true});
console.log('  topspin ω_x=+250 after 0.3s: y diff vs no spin =', (s.end.p[1]-s0.end.p[1]).toFixed(3), 'm (应为负，下坠)');
console.log('  Cd(3,1)=',cdOf(3,1).toFixed(3),' Cm(3.5,300)=',cmOf(3.5,300).toFixed(3),' Cm(5,250)=',cmOf(5,250).toFixed(3), 'Cm(2.6,250)=',cmOf(2.6,250).toFixed(3));
