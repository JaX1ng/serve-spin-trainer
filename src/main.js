// main.js — 三维场景、练习流程、触球讲解、实验台
import { P } from './physics.js';
import { generateServe, manualServe, toLab, TYPES, TYPE_KEYS, PLAYERS, PLAYER_KEYS, RPS, TH_RPS, spinParts } from './serve.js';
import { FINGER_RAD } from './skeleton.js';

const T3 = window.THREE;
const $ = (s, r = document) => r.querySelector(s);
const $$ = (s, r = document) => Array.from(r.querySelectorAll(s));
const DEG = Math.PI / 180;
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const smooth = (u) => (u <= 0 ? 0 : u >= 1 ? 1 : u * u * (3 - 2 * u));
const pick = (a) => a[Math.floor(Math.random() * a.length)];

const PRESETS = {
  basic: { kinds: { side: true, pure: false }, fake: 'none', choices: 2, speed: 0.5, watch: 'land', player: 'std', note: '半速，没有假动作，同一个人发，只分侧上 / 侧下。先把拍型看明白。' },
  mid: { kinds: { side: true, pure: true }, fake: 'some', choices: 3, speed: 1, watch: 'land', player: 'random', note: '原速，每球换一个人发，有时触球后做假动作；加上不带侧旋的"转与不转"，要分出不转。' },
  match: { kinds: { side: true, pure: true }, fake: 'all', choices: 3, speed: 1, watch: 'net', player: 'random', note: '原速，每球都有假动作，球一过网就看不到了：只能靠拍型和前半段球路。' },
};
const STORE = 'pp-spin-v3';

const S = {
  mode: 'practice', phase: 'ready', serve: null, t: 0, lastT: null, playing: false, rate: 1, stopAt: null, auto: null,
  answer: null, ansWhen: '', view: 'recv', home: 'recv', aids: true, trail: true, iso: 'all', spinView: false, spinRate: 0.45, dirty: false, live: null,
  set: { preset: 'basic', types: { pendulum: true, reverse: true, hook: true, backhand: true }, kinds: { side: true, pure: false }, player: 'std', hand: 1, pos: [true, true, true], len: [true, true], fake: 'none', choices: 2, speed: 0.5, watch: 'land' },
  lab: { type: 'pendulum', hand: 1, xc: 0.35, zc: -1.5, hc: 0.16, vfall: 3.0, openDeg: 62, yawDeg: -14, vr: [1.6, -0.6, 4.4], mu: P.rubber.mu, et: P.rubber.et0 },
};

function load(extra) {
  try {
    const d = extra && extra.set ? extra : JSON.parse(localStorage.getItem(STORE) || 'null');
    if (!d || !d.set) return;
    Object.assign(S.set, d.set);
    if (!Array.isArray(S.set.pos) || S.set.pos.length !== 3) S.set.pos = [true, true, true];
    if (!Array.isArray(S.set.len) || S.set.len.length !== 2) S.set.len = [true, true];
    if (!S.set.kinds || (!S.set.kinds.side && !S.set.kinds.pure)) S.set.kinds = { side: true, pure: false };
    if (S.set.player !== 'random' && !PLAYERS[S.set.player]) S.set.player = 'std';
    if (!['land', 'net', 'racket'].includes(S.set.watch)) S.set.watch = 'land';
  } catch (e) { /* 没有存储也照常用 */ }
}
function save() {
  try { localStorage.setItem(STORE, JSON.stringify({ set: S.set })); } catch (e) { /* 忽略 */ }
}

function start(hot) {
  if (!T3 || !T3.OrbitControls) { $('#err').style.display = 'grid'; return; }
  load(hot);
  if (window.claude && window.claude.hot && window.claude.hot.snapshot) window.claude.hot.snapshot(() => ({ set: S.set }));

  // ================= 场景 =================
  const canvas = $('#gl');
  const renderer = new T3.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputEncoding = T3.sRGBEncoding;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = T3.PCFSoftShadowMap;
  const C = (hex) => new T3.Color(hex).convertSRGBToLinear(); // r128 里材质颜色按线性空间解释
  const scene = new T3.Scene();
  scene.background = new T3.Color(0x0b111b);
  scene.fog = new T3.Fog(C(0x0b111b), 11, 30);
  const camera = new T3.PerspectiveCamera(36, 1, 0.02, 60);
  const controls = new T3.OrbitControls(camera, canvas);
  controls.enableDamping = true; controls.dampingFactor = 0.14;
  controls.minDistance = 0.08; controls.maxDistance = 14; controls.maxPolarAngle = Math.PI * 0.96; controls.zoomSpeed = 0.9;

  scene.add(new T3.HemisphereLight(0xe6eeff, 0x5a4640, 0.95));
  const sun = new T3.DirectionalLight(0xffffff, 1.05);
  sun.position.set(1.2, 5.5, 1.8); sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  const sc = sun.shadow.camera; sc.left = -2.8; sc.right = 2.8; sc.top = 3.6; sc.bottom = -3.6; sc.near = 0.5; sc.far = 12;
  sun.shadow.bias = -0.0004;
  scene.add(sun);
  const fill = new T3.DirectionalLight(0xbfd4ff, 0.35); fill.position.set(-2, 2, 4); scene.add(fill);

  const UP = new T3.Vector3(0, 1, 0), ZAX = new T3.Vector3(0, 0, 1);
  const V = (a) => new T3.Vector3(a[0], a[1], a[2]);
  const _v = new T3.Vector3(), _v2 = new T3.Vector3(), _c = new T3.Vector3(), _c2 = new T3.Vector3(), _q = new T3.Quaternion(), _m = new T3.Matrix4();
  const _a = new T3.Vector3(), _b = new T3.Vector3(), _x = new T3.Vector3(), _y = new T3.Vector3(), _z = new T3.Vector3();
  const std = (hex, rough = 0.8) => new T3.MeshStandardMaterial({ color: C(hex), roughness: rough });

  // --- 地面、挡板、球台、球网 ---
  (function buildArena() {
    const floor = new T3.Mesh(new T3.PlaneGeometry(26, 22), new T3.MeshStandardMaterial({ color: C(0x7c2f2b), roughness: 0.95, side: T3.DoubleSide }));
    floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
    const bm = std(0x15233a, 0.9);
    for (const [w, d, x, z] of [[14, 0.05, 0, -6.2], [14, 0.05, 0, 7], [0.05, 13.2, -7, 0], [0.05, 13.2, 7, 0]]) {
      const b = new T3.Mesh(new T3.BoxGeometry(w, 0.75, d), bm); b.position.set(x, 0.375, z); scene.add(b);
    }
    const top = new T3.Mesh(new T3.BoxGeometry(P.tableW, 0.03, P.tableL), std(0x1d5fae, 0.55));
    top.position.y = P.tableY - 0.015; top.receiveShadow = true; scene.add(top);
    const lm = new T3.MeshBasicMaterial({ color: C(0xf2f5fa) });
    const line = (w, l, x, z) => { const m = new T3.Mesh(new T3.PlaneGeometry(w, l), lm); m.rotation.x = -Math.PI / 2; m.position.set(x, P.tableY + 0.0006, z); scene.add(m); };
    line(0.02, P.tableL, -P.tableW / 2 + 0.01, 0); line(0.02, P.tableL, P.tableW / 2 - 0.01, 0);
    line(P.tableW, 0.02, 0, -P.tableL / 2 + 0.01); line(P.tableW, 0.02, 0, P.tableL / 2 - 0.01);
    line(0.006, P.tableL, 0, 0);
    const dm = std(0x10161f, 0.8);
    const ap = new T3.Mesh(new T3.BoxGeometry(P.tableW - 0.16, 0.06, P.tableL - 0.3), dm); ap.position.y = P.tableY - 0.06; scene.add(ap);
    for (const x of [-0.55, 0.55]) for (const z of [-1.0, 1.0]) { const leg = new T3.Mesh(new T3.BoxGeometry(0.05, P.tableY - 0.09, 0.05), dm); leg.position.set(x, (P.tableY - 0.09) / 2, z); leg.castShadow = true; scene.add(leg); }
    const c = document.createElement('canvas'); c.width = 512; c.height = 48;
    const g = c.getContext('2d'); g.clearRect(0, 0, 512, 48); g.strokeStyle = 'rgba(20,24,30,.9)'; g.lineWidth = 1.4;
    for (let x = 0; x <= 512; x += 6) { g.beginPath(); g.moveTo(x, 0); g.lineTo(x, 48); g.stroke(); }
    for (let y = 0; y <= 48; y += 6) { g.beginPath(); g.moveTo(0, y); g.lineTo(512, y); g.stroke(); }
    const nt = new T3.CanvasTexture(c); nt.encoding = T3.sRGBEncoding;
    const net = new T3.Mesh(new T3.PlaneGeometry(P.netHalfW * 2, P.netH), new T3.MeshBasicMaterial({ map: nt, transparent: true, side: T3.DoubleSide, depthWrite: false }));
    net.position.set(0, P.tableY + P.netH / 2, 0); scene.add(net);
    const tape = new T3.Mesh(new T3.BoxGeometry(P.netHalfW * 2, 0.014, 0.005), lm); tape.position.set(0, P.tableY + P.netH - 0.007, 0); scene.add(tape);
    for (const x of [-P.netHalfW, P.netHalfW]) { const post = new T3.Mesh(new T3.CylinderGeometry(0.009, 0.009, P.netH + 0.02, 10), dm); post.position.set(x, P.tableY + P.netH / 2 - 0.01, 0); scene.add(post); }
  })();

  // --- 球：练习时是一颗素球（真实比赛里也看不清球在怎么转）；答完之后换成带标记的球 ---
  const ballPivot = new T3.Group(); scene.add(ballPivot);
  const markTex = (() => {
    const c = document.createElement('canvas'); c.width = 512; c.height = 256;
    const g = c.getContext('2d');
    g.fillStyle = '#f6f3ea'; g.fillRect(0, 0, 512, 256);
    g.fillStyle = '#ff8f1f'; g.fillRect(0, 112, 512, 32);
    g.fillStyle = '#1f6fd6'; g.fillRect(0, 0, 512, 30);
    g.fillStyle = '#d7263d'; g.fillRect(0, 226, 512, 30);
    g.fillStyle = '#12161d'; g.fillRect(0, 30, 16, 196); g.fillRect(256, 60, 10, 52);
    g.font = 'bold 34px Arial'; g.fillText('★★★ 40+', 70, 92);
    g.beginPath(); g.arc(400, 185, 16, 0, 7); g.fill();
    const t = new T3.CanvasTexture(c); t.encoding = T3.sRGBEncoding; t.anisotropy = 4;
    return t;
  })();
  const matPlain = new T3.MeshStandardMaterial({ color: C(0xf6f3ea), roughness: 0.55 });
  const matMark = new T3.MeshStandardMaterial({ map: markTex, roughness: 0.55 });
  const ballMesh = new T3.Mesh(new T3.SphereGeometry(P.R, 40, 28), matPlain);
  ballMesh.castShadow = true; ballPivot.add(ballMesh);
  const ballBlur = new T3.Mesh(new T3.SphereGeometry(P.R * 1.012, 28, 20), new T3.MeshStandardMaterial({ color: C(0xf3ead8), roughness: 0.6, transparent: true, opacity: 0 }));
  ballPivot.add(ballBlur);
  const spot = new T3.Mesh(new T3.SphereGeometry(0.0045, 14, 10), new T3.MeshBasicMaterial({ color: C(0xffe14d) })); spot.visible = false; scene.add(spot);

  // --- 球拍 ---
  const racket = new T3.Group(); scene.add(racket);
  (function buildRacket() {
    const disc = (r, h, color, z) => { const gm = new T3.CylinderGeometry(r, r, h, 40); gm.rotateX(Math.PI / 2); const m = new T3.Mesh(gm, std(color, 0.7)); m.position.z = z; m.scale.y = 1.05; m.castShadow = true; racket.add(m); };
    disc(0.0765, 0.006, 0xc7a36d, 0);
    disc(0.0755, 0.0024, 0xc8102e, 0.0042);   // 正手面：红
    disc(0.0755, 0.0024, 0x14171c, -0.0042);  // 反手面：黑
    const hm = std(0x8a5a2b, 0.7);
    const neck = new T3.Mesh(new T3.BoxGeometry(0.036, 0.03, 0.014), hm); neck.position.y = -0.088; racket.add(neck);
    const h = new T3.Mesh(new T3.BoxGeometry(0.029, 0.1, 0.023), hm); h.position.y = -0.135; h.castShadow = true; racket.add(h);
  })();

  // --- 对手：按关节摆出来的人形。骨架和每一帧的关节位置来自 motion.js / skeleton.js ---
  const body = new T3.Group(); scene.add(body);
  const mJersey = std(0x2c9c8c, 0.85), mShorts = std(0x1a2231, 0.9), mSkin = std(0xe0b48f, 0.75), mHair = std(0x17120f, 0.9), mShoe = std(0xe9ecf1, 0.7), mEye = std(0x1a1410, 0.5), mSock = std(0xf2f2f2, 0.85);
  const addMesh = (geo, mat, parent = body) => { const m = new T3.Mesh(geo, mat); m.castShadow = true; parent.add(m); return m; };
  const limb = (rb, rt, mat, seg = 16) => { const g = new T3.CylinderGeometry(rt, rb, 1, seg); g.translate(0, 0.5, 0); return addMesh(g, mat); };
  const ball = (r, mat, parent, ws = 18, hs = 14) => addMesh(new T3.SphereGeometry(r, ws, hs), mat, parent);
  const setLimb = (m, a, b) => { m.position.copy(a); _v.copy(b).sub(a); const L = _v.length() || 1e-4; m.scale.set(1, L, 1); m.quaternion.setFromUnitVectors(UP, _v.multiplyScalar(1 / L)); };
  // 截面是椭圆的那几段（躯干、手掌、骨盆）：纵轴沿 a→b，横轴大致朝 right
  const setSlab = (m, a, b, right, sx, sz, len) => {
    _y.copy(b).sub(a); const L = _y.length() || 1e-4; _y.multiplyScalar(1 / L);
    _x.copy(right).addScaledVector(_y, -right.dot(_y)); if (_x.lengthSq() < 1e-8) _x.set(1, 0, 0); _x.normalize();
    _z.crossVectors(_x, _y);
    _m.makeBasis(_x, _y, _z); m.quaternion.setFromRotationMatrix(_m); m.position.copy(a); m.scale.set(sx, len === undefined ? L : len, sz);
  };
  const B = {};
  (function buildBody() {
    B.belly = limb(1, 0.95, mJersey, 24);
    B.ribs = limb(0.88, 1, mJersey, 24);
    B.waist = ball(1, mJersey); B.yoke = limb(0.062, 0.062, mJersey); B.top = ball(1, mJersey);
    B.shorts = ball(1, mShorts);
    B.neck = limb(0.05, 0.045, mSkin);
    B.head = new T3.Group(); body.add(B.head);
    const skull = ball(0.1, mSkin, B.head); skull.scale.set(0.9, 1.1, 1.0);
    const hair = addMesh(new T3.SphereGeometry(0.105, 20, 14, 0, Math.PI * 2, 0, Math.PI * 0.56), mHair, B.head); hair.scale.set(0.92, 1.1, 1.02); hair.rotation.x = -0.42; hair.position.y = 0.006;
    for (const sx of [-1, 1]) {
      const e = ball(0.011, mEye, B.head, 10, 8); e.position.set(0.034 * sx, 0.012, 0.089); e.castShadow = false;
      const ear = ball(0.018, mSkin, B.head, 10, 8); ear.scale.set(0.5, 1, 0.8); ear.position.set(0.09 * sx, 0, 0.0);
    }
    const nose = ball(0.012, mSkin, B.head, 10, 8); nose.position.set(0, -0.012, 0.099);
    for (const k of ['P', 'F']) {
      B['sh' + k] = ball(0.062, mJersey);
      B['up' + k] = limb(0.048, 0.04, mSkin);
      B['sl' + k] = limb(0.061, 0.054, mJersey);   // 短袖
      B['el' + k] = ball(0.041, mSkin);
      B['fo' + k] = limb(0.04, 0.029, mSkin);
      B['wr' + k] = ball(0.03, mSkin, body, 12, 10);
      B['palm' + k] = ball(1, mSkin);
      B['fg' + k] = [];
      for (let f = 0; f < 5; f++) {
        const r = FINGER_RAD[f], segs = [];
        for (let s2 = 0; s2 < 3; s2++) segs.push({ bone: limb(r * (1 - 0.08 * s2), r * (0.92 - 0.08 * s2), mSkin, 8), tip: ball(r * (0.92 - 0.08 * s2), mSkin, body, 8, 6) });
        B['fg' + k].push(segs);
      }
      B['hip' + k] = ball(0.088, mShorts);
      B['th' + k] = limb(0.07, 0.053, mSkin);
      B['sh2' + k] = limb(0.092, 0.076, mShorts);  // 短裤腿
      B['kn' + k] = ball(0.055, mSkin);
      B['sn' + k] = limb(0.053, 0.037, mSkin);
      B['sock' + k] = limb(0.041, 0.043, mSock);
      B['shoe' + k] = ball(1, mShoe); B['shoe' + k].scale.set(0.05, 0.045, 0.132); // 鞋：一个压扁拉长的椭球，下半截埋在地面以下
    }
  })();
  let shirtOf = null;
  function updateBody(sv, t) {
    const J = sv.body.at(t, true);
    const pel = V(J.pelvis), mid = V(J.mid), chest = V(J.chest), Rh = V(J.R), Rc = V(J.Rc), Uc = V(J.Uc);
    const Rm = Rh.clone().lerp(Rc, 0.45).normalize();
    // 躯干：腰腹一段、胸廓一段，各自顺着脊柱；截面是扁的
    setSlab(B.belly, pel.clone().addScaledVector(V(J.sd1), -0.02), mid, Rm, 0.15, 0.108);
    setSlab(B.ribs, mid, chest.clone().addScaledVector(Uc, -0.035), Rc, 0.172, 0.114);
    setSlab(B.waist, mid.clone().addScaledVector(Uc, -0.07), mid.clone().addScaledVector(Uc, 0.07), Rm, 0.1445, 0.1, 0.075);
    setSlab(B.top, chest.clone().addScaledVector(Uc, -0.095), chest, Rc, 0.172, 0.114, 0.075);
    setSlab(B.shorts, pel.clone().addScaledVector(UP, -0.06), pel.clone().addScaledVector(V(J.sd1), 0.06).addScaledVector(UP, 0.02), Rh, 0.174, 0.128, 0.115);
    const shP = V(J.shP), shF = V(J.shF);
    setLimb(B.yoke, shP, shF);
    const head = V(J.head);
    setLimb(B.neck, V(J.neck), head);
    B.head.position.copy(head); B.head.lookAt(_v.copy(head).add(_v2.set(J.look[0], J.look[1], J.look[2])));
    for (const k of ['P', 'F']) {
      const sh = k === 'P' ? shP : shF, el = V(J['el' + k]), wr = V(J['wr' + k]);
      B['sh' + k].position.copy(sh);
      setLimb(B['up' + k], sh, el);
      setLimb(B['sl' + k], sh, _a.copy(sh).lerp(el, 0.52));
      B['el' + k].position.copy(el);
      setLimb(B['fo' + k], el, wr);
      B['wr' + k].position.copy(wr);
      // 手掌和手指
      const H = J['hand' + k], d = V(H.d), pn = V(H.p);
      const right = _b.crossVectors(d, pn);
      setSlab(B['palm' + k], _a.copy(wr).addScaledVector(d, 0.004).addScaledVector(pn, -0.002), _c.copy(wr).addScaledVector(d, 0.094).addScaledVector(pn, -0.002), right, 0.041, 0.0135, 0.047);
      B['palm' + k].position.copy(wr).addScaledVector(d, 0.049).addScaledVector(pn, -0.002);
      const fg = J['fing' + k];
      for (let f = 0; f < 5; f++) for (let s2 = 0; s2 < 3; s2++) {
        const a = fg[f][s2], b = fg[f][s2 + 1], seg = B['fg' + k][f][s2];
        setLimb(seg.bone, _a.set(a[0], a[1], a[2]), _c.set(b[0], b[1], b[2]));
        seg.tip.position.set(b[0], b[1], b[2]);
      }
      const hip = V(J['hip' + k]), kn = V(J['kn' + k]), an = V(J['an' + k]), toe = J['toe' + k];
      B['hip' + k].position.copy(hip);
      setLimb(B['th' + k], hip, kn);
      setLimb(B['sh2' + k], hip, _a.copy(hip).lerp(kn, 0.55));
      B['kn' + k].position.copy(kn);
      setLimb(B['sn' + k], kn, an);
      setLimb(B['sock' + k], _a.copy(kn).lerp(an, 0.8), _c.copy(an).setY(0.05));
      const shoe = B['shoe' + k];
      shoe.position.set(an.x + toe[0] * 0.055, 0.032, an.z + toe[2] * 0.055);
      shoe.rotation.set(0, Math.atan2(toe[0], toe[2]), 0);
    }
    if (shirtOf !== sv.player && PLAYERS[sv.player]) { shirtOf = sv.player; mJersey.color.copy(C(PLAYERS[sv.player].shirt)); }
  }

  // --- 箭头、转向环、标签 ---
  function makeArrow(color) {
    const mat = new T3.MeshBasicMaterial({ color: C(color) });
    const g = new T3.Group();
    const sg = new T3.CylinderGeometry(1, 1, 1, 14); sg.translate(0, 0.5, 0);
    const hg = new T3.ConeGeometry(1, 1, 18); hg.translate(0, 0.5, 0);
    const shaft = new T3.Mesh(sg, mat), head = new T3.Mesh(hg, mat);
    // 被球台或身体挡住的那一段，用半透明的影子补出来
    const gm = new T3.MeshBasicMaterial({ color: C(color), transparent: true, opacity: 0.26, depthTest: false, depthWrite: false });
    const shaftG = new T3.Mesh(sg, gm), headG = new T3.Mesh(hg, gm);
    shaftG.renderOrder = headG.renderOrder = 6;
    g.add(shaft, head, shaftG, headG); g.visible = false; scene.add(g);
    g.tip = new T3.Vector3();
    g.put = (o, vec, thick) => {
      const L = vec.length();
      if (L < 1e-5) { g.visible = false; return; }
      g.visible = true;
      const hl = Math.min(L * 0.42, thick * 6);
      shaft.scale.set(thick, Math.max(1e-5, L - hl), thick);
      head.position.y = L - hl; head.scale.set(thick * 2.5, hl, thick * 2.5);
      shaftG.scale.copy(shaft.scale); headG.position.copy(head.position); headG.scale.copy(head.scale);
      g.position.copy(o);
      g.quaternion.setFromUnitVectors(UP, _v2.copy(vec).multiplyScalar(1 / L));
      g.tip.copy(o).add(vec);
    };
    return g;
  }
  function makeRing(color) {
    const mat = new T3.MeshBasicMaterial({ color: C(color) });
    const g = new T3.Group();
    g.add(new T3.Mesh(new T3.TorusGeometry(1, 0.03, 8, 72, 5.25), mat));
    for (const th of [1.1, 3.1, 5.25]) { const c = new T3.Mesh(new T3.ConeGeometry(0.1, 0.3, 14), mat); c.position.set(Math.cos(th), Math.sin(th), 0); c.rotation.z = th; g.add(c); }
    g.visible = false; scene.add(g);
    g.put = (center, axis, radius) => { g.visible = true; g.position.copy(center); g.scale.setScalar(radius); g.quaternion.setFromUnitVectors(ZAX, axis); };
    return g;
  }
  const COL = { x: 0xff6257, y: 0x35d08b, z: 0x4ea9ff, t: 0xf4f6fa, n: 0xe9eef6, br: 0xffe14d };
  // bu / bs：把"蹭"拆成顺着拍面上下的那一份（决定上下旋）和横着的那一份（决定侧旋）
  const arr = { x: makeArrow(COL.x), y: makeArrow(COL.y), z: makeArrow(COL.z), t: makeArrow(COL.t), n: makeArrow(COL.n), br: makeArrow(COL.br), bu: makeArrow(COL.x), bs: makeArrow(COL.y) };
  const ring = { x: makeRing(COL.x), y: makeRing(COL.y), z: makeRing(COL.z), t: makeRing(COL.t) };
  const labelsEl = $('#labels');
  const mkLabel = (cssColor, cls = '') => { const el = document.createElement('div'); el.className = 'lbl ' + cls; el.style.color = cssColor; el.style.display = 'none'; labelsEl.appendChild(el); return el; };
  const lab3 = { x: mkLabel('var(--x)'), y: mkLabel('var(--y)'), z: mkLabel('var(--z)'), t: mkLabel('#f4f6fa'), n: mkLabel('#e9eef6', 'sm'), br: mkLabel('#ffe14d', 'sm') };
  // 标签贴在箭头尖外侧；箭头在屏幕上太短时，改成围着中心排开，免得叠在一起
  const LOFF = { x: [70, 0], y: [0, -34], z: [-70, 0], t: [0, 36], n: [0, -34], br: [-86, 6] };
  let ballPx = 100;
  function placeLabel(el, tip, on, key, center) {
    if (!on) { el.style.display = 'none'; return; }
    const w = canvas.clientWidth, h = canvas.clientHeight;
    _v.copy(tip).project(camera);
    if (_v.z > 1 || _v.z < -1) { el.style.display = 'none'; return; }
    let x = (_v.x * 0.5 + 0.5) * w, y = (-_v.y * 0.5 + 0.5) * h;
    _c.copy(center).project(camera);
    const cx = (_c.x * 0.5 + 0.5) * w, cy = (-_c.y * 0.5 + 0.5) * h, d = Math.hypot(x - cx, y - cy);
    const near = key === 'n' || key === 'br';
    const radial = near ? d < ballPx + 30 : ballPx < 56;
    if (radial || d < 1) {
      const k = near ? Math.max(1, (ballPx + 24) / 34) : 1; // 镜头凑得近时球很大，标签要让到球外面去
      x = cx + LOFF[key][0] * (key === 'br' ? Math.max(1, k * 0.7) : 1); y = cy + LOFF[key][1] * (key === 'n' ? k : 1);
    } else { x += ((x - cx) / d) * 16; y += ((y - cy) / d) * 16; }
    // 别钻到两边的面板和上下的栏底下
    const wide = w > 1020;
    x = clamp(x, (wide ? 296 : 0) + 64, w - (wide ? 310 : 0) - 64); y = clamp(y, 112, h - 150);
    el.style.display = 'block';
    el.style.left = x.toFixed(1) + 'px';
    el.style.top = y.toFixed(1) + 'px';
  }

  // --- 轨迹线、落点标记、球拍走过的弧线 ---
  const TR_MAX = 900;
  const trailGeo = new T3.BufferGeometry(); trailGeo.setAttribute('position', new T3.BufferAttribute(new Float32Array(TR_MAX * 3), 3));
  const trail = new T3.Line(trailGeo, new T3.LineBasicMaterial({ color: C(0xffb35c), transparent: true, opacity: 0.9 })); trail.frustumCulled = false; scene.add(trail);
  let trailT = [], trailN = 0;
  const PATH_N = 140;
  const pathGeo = new T3.BufferGeometry(); pathGeo.setAttribute('position', new T3.BufferAttribute(new Float32Array(PATH_N * 3), 3));
  const swingPath = new T3.Line(pathGeo, new T3.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.45 })); swingPath.frustumCulled = false; scene.add(swingPath);
  const marks = [];
  for (let i = 0; i < 4; i++) { const m = new T3.Mesh(new T3.RingGeometry(0.02, 0.031, 28), new T3.MeshBasicMaterial({ color: C(0xffb35c), side: T3.DoubleSide, transparent: true, opacity: 0.95 })); m.rotation.x = -Math.PI / 2; m.visible = false; scene.add(m); marks.push(m); }

  function prepServeVisuals(sv) {
    const pa = trailGeo.attributes.position.array; trailT = []; trailN = 0;
    if (sv.col && sv.sim.S) {
      const Sm = sv.sim.S, stepN = Math.max(1, Math.ceil(Sm.n / TR_MAX));
      for (let i = 0; i < Sm.n && trailN < TR_MAX; i += stepN) { pa[trailN * 3] = Sm.p[i * 3]; pa[trailN * 3 + 1] = Sm.p[i * 3 + 1]; pa[trailN * 3 + 2] = Sm.p[i * 3 + 2]; trailT.push(i * Sm.dt); trailN++; }
    }
    trailGeo.attributes.position.needsUpdate = true;
    // 拍面中心从引拍到收拍走过的路
    const pp = pathGeo.attributes.position.array; const t0 = sv.tSwing0 - 0.02, t1 = sv.tSwingEnd + 0.02;
    for (let i = 0; i < PATH_N; i++) { const c = sv.racketAt(t0 + ((t1 - t0) * i) / (PATH_N - 1)).c; pp[i * 3] = c[0]; pp[i * 3 + 1] = c[1]; pp[i * 3 + 2] = c[2]; }
    pathGeo.attributes.position.needsUpdate = true;
    const bs = sv.events.filter((e) => e.type === 'bounce');
    marks.forEach((m, i) => { if (bs[i]) { m.position.set(bs[i].p[0], P.tableY + 0.0012, bs[i].p[2]); m.userData.t = bs[i].t; } else m.userData.t = Infinity; });
    sv.pel0 = sv.body ? sv.body.at(0).pelvis : sv.pc; // 人站在哪儿（镜头取景用）
  }

  // ================= 视角 =================
  const EYE = new T3.Vector3(0, 1.36, 2.9); // 接球者眼睛的位置
  let camAnim = null;
  controls.addEventListener('start', () => { camAnim = null; }); // 用户自己动了镜头：停掉正在进行的镜头移动
  function flyTo(pos, tgt, dur = 0.55) { camAnim = { p0: camera.position.clone(), t0: controls.target.clone(), p1: pos, t1: tgt, u: 0, dur }; }
  const ballPos = new T3.Vector3(), prevBall = new T3.Vector3();
  // 接球视角：屏幕窄、或者两边面板挡住了发球的人时，像真的接发球那样把头转向他
  let recvAx = 0;
  function recvAim(sv) {
    if (!sv) return 0;
    const w = canvas.clientWidth || window.innerWidth, h = canvas.clientHeight || window.innerHeight;
    const wide = w > 1020, free = (w - (wide ? 296 + 310 : 0)) / 2 - (wide ? 120 : 145); // 没被挡住的半宽，留出一个人的余量
    const f = h / 2 / Math.tan((camera.fov * DEG) / 2);
    const xs = (sv.pc[0] + (sv.pel0 ? sv.pel0[0] : sv.pc[0])) / 2;
    const off = (Math.abs(xs) / 4.5) * f;
    return off > free ? clamp((Math.sign(xs) * (off - free) * 4.1) / f, -0.9, 0.9) : 0;
  }
  function setView(name, instant, wide) {
    S.view = name;
    const sv = S.serve;
    const pc = sv ? V(sv.pc) : new T3.Vector3(0.35, 0.92, -1.5);
    const pel = sv && sv.pel0 ? sv.pel0 : null;
    const toEye = EYE.clone().sub(pc).normalize();
    let pos, tgt;
    if (name === 'recv') { pos = EYE.clone(); recvAx = recvAim(sv); tgt = new T3.Vector3(recvAx, 0.95, -1.2); }
    else if (name === 'close') { // 还是从你这边看过去，只是走近了
      const bx = pel ? pel[0] - pc.x : 0;
      tgt = pc.clone().add(new T3.Vector3(bx * 0.35, 0.2, 0)); pos = tgt.clone().addScaledVector(toEye, 1.95); pos.y += 0.05;
    } else if (name === 'contact') { // 触球特写：还在你这一侧，挑一个"拍面朝向"和"蹭球方向"两根箭头都不正对镜头的角度
      const nn = sv ? V(sv.n) : new T3.Vector3(0, 0, 1);
      const bb = sv && sv.an && sv.an.brush ? V(sv.an.brush) : null;
      if (bb && bb.length() > 0.2) bb.normalize();
      const bd = pel ? new T3.Vector3(pel[0] - pc.x, 0, pel[2] - pc.z).normalize() : null;
      const eh = new T3.Vector3(toEye.x, 0, toEye.z).normalize();
      // wide：慢放整个挥拍时退远一点，把引拍、收拍和上半身都框进来
      const dist = wide ? 1.8 : 1.0;
      tgt = pc.clone();
      const J0 = sv && sv.body ? sv.body.at(0) : null;
      if (wide && J0) tgt.lerp(V(J0.chest), 0.22);
      // 他自己的头、肩、手臂别挡在镜头和触球点之间，也别怼到镜头跟前
      const blockers = J0 ? [[J0.head, 0.13], [J0.chest, 0.16], [J0.shF, 0.08], [J0.shP, 0.08], [J0.elF, 0.06], [J0.wrF, 0.06], [J0.elP, 0.06], [J0.mid, 0.15]] : [];
      let best = null;
      for (let az = -80; az <= 80; az += 10) for (const el of [0.04, 0.2, 0.4, 0.65]) {
        const d = eh.clone().applyAxisAngle(UP, az * DEG); d.y = el; d.normalize();
        let score = Math.max(Math.abs(d.dot(nn)), bb && bb.lengthSq() > 0.5 ? Math.abs(d.dot(bb)) : 0) + 0.004 * Math.abs(az) + 0.15 * el;
        if (bd && d.dot(bd) > 0.35) score += 1; // 别把镜头架到他身体那一侧去
        _a.copy(tgt).addScaledVector(d, dist); if (_a.y < P.tableY + 0.16) _a.y = P.tableY + 0.16;
        for (const [p, r] of blockers) {
          _b.set(p[0], p[1], p[2]);
          _x.copy(pc).sub(_a); const L2 = _x.lengthSq(), u = clamp(_y.copy(_b).sub(_a).dot(_x) / L2, 0, 1);
          const off = _y.copy(_a).addScaledVector(_x, u).distanceTo(_b);
          if (off < r + 0.05 && u < 0.97) score += 0.8 + 2 * (r + 0.05 - off) / (r + 0.05);
          const near = _b.distanceTo(_a); if (near < 0.5 * dist) score += 2 * (0.5 * dist - near) / dist;
        }
        if (!best || score < best.score) best = { d, score };
      }
      pos = tgt.clone().addScaledVector(best.d, dist);
      if (pos.y < P.tableY + 0.16) pos.y = P.tableY + 0.16;
    }
    else if (name === 'side') { const sgn = pel ? Math.sign(pel[0] - pc.x) || 1 : 1; pos = new T3.Vector3(-5.6 * sgn, 1.5, -0.2); tgt = new T3.Vector3(0, 0.9, -0.2); }
    else if (name === 'top') { tgt = new T3.Vector3(0, 0.76, -0.4); pos = new T3.Vector3(0, 8.9, -0.38); } // 把站在端线后面的人也框进来
    else { // 凑近看球：还是从你这一侧看，球停在画面正中
      _v.copy(EYE).sub(ballPos); _v.y = Math.max(_v.y, 0.12); _v.normalize();
      pos = ballPos.clone().addScaledVector(_v, 0.3); tgt = ballPos.clone();
    }
    if (instant) { camera.position.copy(pos); controls.target.copy(tgt); camAnim = null; } else flyTo(pos, tgt);
    $$('#views button').forEach((b) => b.classList.toggle('on', b.dataset.view === name));
  }
  // 画面中心挪到没被面板挡住的那块区域的中心。窄屏上底栏的高度会变，按实际高度算
  function inset() {
    const w = canvas.clientWidth || window.innerWidth, h = canvas.clientHeight || window.innerHeight;
    const wide = w > 1020;
    const L = wide ? 296 : 0, R = wide ? 310 : 0, T = wide ? 104 : 92;
    const Bm = wide ? 128 : Math.max(96, h - $('#dock').getBoundingClientRect().top + 8);
    camera.setViewOffset(w, h, -(L - R) / 2, (Bm - T) / 2, w, h);
    camera.updateProjectionMatrix();
  }
  function resize() {
    const w = canvas.clientWidth || window.innerWidth, h = canvas.clientHeight || window.innerHeight;
    renderer.setSize(w, h, false);
    camera.aspect = w / h; camera.fov = clamp((36 * 1.55) / camera.aspect, 36, 50);
    inset();
    // 窗口变窄 / 转屏之后，接球视角要重新对一下发球的人
    if (S.view === 'recv' && S.serve && !camAnim && Math.abs(recvAim(S.serve) - recvAx) > 0.03) setView('recv', true);
  }
  window.addEventListener('resize', resize);
  if (window.ResizeObserver) new ResizeObserver(resize).observe($('#app'));

  // ================= 每帧把"时间 t 的发球状态"画出来 =================
  const qShow = new T3.Quaternion();
  const reviewing = () => S.mode === 'lab' || S.phase === 'review';
  // 练习时球看到哪一刻为止。落到你这边台面的那一下就消失：落台之后那段弧线太容易看出上下旋，实战里那时候早该出手了
  function ballGoneAt(sv) {
    const w = S.set.watch;
    if (w === 'racket') return 0.03;
    if (w === 'net' && sv.verdict.net) return sv.verdict.net.t + 0.01;
    return sv.arrive ? sv.arrive.t + 0.004 : sv.tEnd;
  }
  // 练习时动作放到哪儿为止：球消失之后再留一小会儿，把收拍看完
  const playEndOf = (sv) => Math.min(sv.tEnd, Math.max(ballGoneAt(sv) + 0.2, sv.tSwingEnd + 0.4));
  // 旋转按球自己的方向分解：X = 横轴（垂直于前进方向，水平），Y = 竖轴，Z = 前后轴（前进方向）
  function axesFor(b) {
    if (Math.hypot(b.v[0], b.v[2]) > 0.15) { const sp = spinParts(b.w, b.v); return { ax: [sp.h, [0, 1, 0], sp.f], c: [sp.top, sp.side, sp.cork] }; }
    return { ax: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], c: [b.w[0], b.w[1], b.w[2]] };
  }
  function apply(dt) {
    const sv = S.serve, t = S.t;
    const b = sv.ballAt(t);
    const rv = reviewing();
    let hideBall = false;
    if (!rv && t > ballGoneAt(sv)) hideBall = true;
    ballPos.set(b.p[0], b.p[1], b.p[2]);
    ballPivot.position.copy(ballPos); ballPivot.visible = !hideBall;
    ballMesh.material = rv ? matMark : matPlain;
    const rd = P.R;
    const wl = Math.hypot(b.w[0], b.w[1], b.w[2]);
    const A = axesFor(b);
    // 姿态：播放时用物理积分出来的姿态。只有"看球怎么转"时，球才停在原地慢慢转
    if (S.playing || S.lastT !== t || !S.spinView) qShow.set(b.q[0], b.q[1], b.q[2], b.q[3]);
    let blur = 0;
    if (!S.playing && S.spinView && rv) {
      let wd = b.w;
      if (S.iso !== 'all') { const i = 'xyz'.indexOf(S.iso); wd = [A.ax[i][0] * A.c[i], A.ax[i][1] * A.c[i], A.ax[i][2] * A.c[i]]; }
      const m = Math.hypot(wd[0], wd[1], wd[2]);
      const slow = Math.pow(S.spinRate, 2.2) * 0.12;
      if (m > 1e-6 && slow > 0) { _q.setFromAxisAngle(_v.set(wd[0] / m, wd[1] / m, wd[2] / m), m * dt * slow); qShow.premultiply(_q); }
    } else if (S.playing && rv) blur = clamp((wl * dt * S.rate - 0.7) / 1.3, 0, 0.82);
    ballMesh.quaternion.copy(qShow);
    ballBlur.material.opacity = blur; ballBlur.visible = blur > 0.01;
    S.lastT = t;

    // 球拍与对手
    const pz = sv.racketAt(t);
    racket.quaternion.set(pz.q[0], pz.q[1], pz.q[2], pz.q[3]);
    racket.position.set(pz.c[0], pz.c[1], pz.c[2]);
    body.visible = !!sv.body;
    if (sv.body) updateBody(sv, t);

    // 轨迹与落点
    const showTr = rv && S.trail && trailN > 1;
    const whole = S.mode === 'lab' && !S.playing && t <= 0.002;
    trail.visible = showTr;
    if (showTr) { let n = 0; if (whole) n = trailN; else { while (n < trailN && trailT[n] <= t) n++; } trailGeo.setDrawRange(0, n); }
    marks.forEach((m) => { m.visible = rv && S.trail && !S.spinView && (t >= m.userData.t || (whole && isFinite(m.userData.t))); });

    // 触球讲解：拍面朝向、拍面蹭球的方向、蹭到球的哪儿、球被带着怎么转
    const hit = rv && S.aids && sv.col && !S.spinView && t > -0.02 && t < 0.045;
    swingPath.visible = rv && S.aids && !S.spinView;
    const bc = _c2.set(sv.pc[0] - sv.n[0] * P.R, sv.pc[1] - sv.n[1] * P.R, sv.pc[2] - sv.n[2] * P.R);
    if (hit) {
      const o = bc.clone(), k = 0.026, an = sv.an;
      arr.n.put(o, V(sv.n).multiplyScalar(0.11), 0.003);
      const br = V(an.brush), up = V(an.uf).multiplyScalar(an.relAlong), sd = br.clone().sub(up);
      arr.br.put(o, br.clone().multiplyScalar(k), 0.0042);
      const bl = br.length();
      if (Math.abs(an.relAlong) > 0.1 * bl && sd.length() > 0.1 * bl) { arr.bu.put(o, up.multiplyScalar(k), 0.0026); arr.bs.put(o, sd.multiplyScalar(k), 0.0026); }
      else arr.bu.visible = arr.bs.visible = false; // 几乎全是一个方向时，黄箭头自己就够了
      spot.visible = true; spot.position.copy(o);
      lab3.n.textContent = '拍面朝向';
      lab3.br.textContent = '拍子蹭球的方向';
    } else { arr.n.visible = arr.br.visible = arr.bu.visible = arr.bs.visible = false; spot.visible = false; }
    placeLabel(lab3.n, arr.n.tip, hit && arr.n.visible, 'n', bc);
    placeLabel(lab3.br, arr.br.tip, hit && arr.br.visible, 'br', bc);

    // 旋转：合成转轴 + 三个分量
    ballPx = (rd / Math.max(0.05, camera.position.distanceTo(ballPos))) * (canvas.clientHeight / (2 * Math.tan((camera.fov * DEG) / 2)));
    const spinOn = rv && S.aids && wl > 2 && t >= 0 && ballPivot.visible;
    const comps = spinOn && (S.spinView || t >= 0.045); // 触球特写时不画分量箭头，免得和蹭球的箭头挤在一起
    const ref = Math.max(wl, 15 * RPS);
    for (const key of ['x', 'y', 'z']) {
      const i = 'xyz'.indexOf(key), c = A.c[i], on = comps && Math.abs(c) > 0.04 * ref && (S.iso === 'all' || S.iso === key);
      if (on) {
        const dir = _v.set(A.ax[i][0], A.ax[i][1], A.ax[i][2]).multiplyScalar(Math.sign(c)).clone();
        arr[key].put(ballPos.clone().addScaledVector(dir, rd * 1.04), dir.clone().multiplyScalar(rd * 2.0 * Math.abs(c) / ref), rd * 0.065);
        if (S.iso === key) ring[key].put(ballPos, dir, rd * 1.24); else ring[key].visible = false;
        lab3[key].innerHTML = `${key.toUpperCase()} ${compName(i, c)} <b>${Math.abs(c / RPS).toFixed(0)}</b>`;
      } else { arr[key].visible = false; ring[key].visible = false; }
      placeLabel(lab3[key], arr[key].tip, on, key, ballPos);
    }
    const totOn = spinOn && S.iso === 'all';
    if (totOn) {
      const dir = new T3.Vector3(b.w[0] / wl, b.w[1] / wl, b.w[2] / wl);
      ring.t.put(ballPos, dir, rd * 1.24);
      if (comps) { arr.t.put(ballPos.clone().addScaledVector(dir, rd * 1.04), dir.clone().multiplyScalar(rd * 2.3 * wl / ref), rd * 0.045); lab3.t.innerHTML = `转轴 <b>${(wl / RPS).toFixed(0)}</b> 转/秒`; }
      else { arr.t.visible = false; }
    } else { arr.t.visible = false; ring.t.visible = false; }
    placeLabel(lab3.t, arr.t.tip, totOn && comps, 't', ballPos);
    updateReadout(b, A, wl);
  }
  function compName(i, c) {
    if (i === 0) return c > 0 ? '上旋' : '下旋';
    if (i === 1) return '侧旋';
    return '螺旋';
  }

  // ================= 右侧：这一球的讲解 + 三根轴的转速 =================
  let lastReadKey = '';
  function updateReadout(b, A, wl) {
    const sv = S.serve;
    const key = `${S.t.toFixed(3)}|${sv.id}`;
    if (key === lastReadKey) return; lastReadKey = key;
    const set = (id, c, pos, neg) => {
      const r = c / RPS;
      $('#val' + id).innerHTML = S.t < 0 ? '—' : `${Math.abs(r).toFixed(1)}<small>转/秒</small>`;
      const w = clamp(Math.abs(r) / 60, 0, 1) * 50;
      const el = $('#bar' + id); el.style.width = (S.t < 0 ? 0 : w) + '%'; el.style.left = r >= 0 ? '50%' : 50 - w + '%';
      $('#desc' + id).textContent = S.t < 0 ? '还没触球' : Math.abs(r) < 0.5 ? '几乎没有' : r > 0 ? pos : neg;
    };
    set('X', A.c[0], '上旋：碰你的拍子往上蹦', '下旋：碰你的拍子往下掉');
    set('Y', A.c[1], '侧旋：碰你的拍子往你左边蹦', '侧旋：碰你的拍子往你右边蹦');
    set('Z', A.c[2], '螺旋：落台后往你左边拐', '螺旋：落台后往你右边拐');
    $('#valT').textContent = S.t < 0 ? '—' : (wl / RPS).toFixed(1);
    let st = '触球前';
    if (S.t >= 0) { st = '刚离拍'; for (const e of sv.events) if (e.type === 'bounce' && S.t >= e.t) st = e.side === 'own' ? '第一跳后' : '到你面前'; }
    $('#stageName').textContent = st;
  }
  // 把一个方向说成人话。左右按你（接球的人）看到的算
  const dirWord = (v) => {
    const l = Math.hypot(v[0], v[1], v[2]) || 1, x = v[0] / l, y = v[1] / l, z = v[2] / l;
    const out = [];
    if (Math.abs(x) > 0.33) out.push(x > 0 ? '往你的右边' : '往你的左边');
    if (Math.abs(y) > 0.33) out.push(y > 0 ? '往上' : '往下');
    if (Math.abs(z) > 0.33) out.push(z > 0 ? '朝你这边' : '往他自己那边');
    return out.join('、') || '几乎没动';
  };
  // 到你面前算哪一类。带侧旋的叫侧上 / 侧下，不带侧旋的叫上旋 / 下旋
  const clsName = (sv, cls = sv.cls) => (sv.kind === 'pure' ? { top: '上旋', back: '下旋', flat: '不转' } : { top: '侧上', back: '侧下', flat: '不转' })[cls];
  const FAKE_TXT = { lift: '触球之后拍子往上一提，是装出来的"往上蹭"', chop: '触球之后拍子往下一切，是装出来的"往下蹭"', tuck: '触球之后马上把拍子收回身前，不让你多看' };
  function explainHtml(sv) {
    if (!sv.col) return '<li><b>没碰到球</b>拍面是背着球走的，拍子和球错开了。</li>';
    const a = sv.an, out = [];
    // 1. 拍面朝哪儿、碰到球的哪儿
    const o = a.openDeg;
    let face = o >= 75 ? '几乎放平、朝天' : o >= 45 ? '后仰得很多' : o >= 20 ? '明显后仰' : o >= 6 ? '稍微后仰' : o > -6 ? '立着' : '前倾（压着拍）';
    face += `（${Math.abs(o).toFixed(0)}°）`;
    if (Math.abs(a.yawDeg) >= 12 && o < 75) face += `、偏向你的${a.yawDeg > 0 ? '右' : '左'}边`;
    const u = a.spotUp;
    let sp = u < -0.87 ? '球的底部' : u < -0.5 ? '球的后下方' : u < -0.17 ? '球的后面偏下' : u < 0.17 ? '球的正后面' : '球的后上方';
    if (a.spotSide > 0.25) sp += '偏你的右手边'; else if (a.spotSide < -0.25) sp += '偏你的左手边';
    let zone = '';
    if (sv.body) zone = a.off[1] > 0.02 ? '，用的是<b class="in">拍头那一端</b>（挥起来那儿走得最快）' : a.off[1] < -0.012 ? '，用的是<b class="in">靠近拍柄的地方</b>（那儿走得慢，蹭出来的旋转少）' : '';
    let head = '';
    if (a.head) { const y = a.head[1]; head = y < -0.6 ? '拍头朝下' : y < -0.25 ? '拍头斜朝下' : y < 0.25 ? '拍头横着' : y < 0.6 ? '拍头斜朝上' : '拍头朝上'; }
    out.push(`<li><b>拍面</b>${face}，贴在${sp}${zone}。${head ? `<i>这一球${head}。拍头指哪儿不影响旋转，只看拍面朝哪、往哪蹭。</i>` : ''}</li>`);
    // 2. 怎么蹭的：整体方向，再拆成"顺着拍面上下"（只管上下旋）和"横着"（只管侧旋）两份
    const bs = Math.hypot(a.brush[0], a.brush[1], a.brush[2]);
    const thick = bs > 1.7 * a.push ? '蹭得薄，主要靠摩擦。' : bs < 0.75 * a.push ? '顶得多、蹭得少，球主要是被撞出去的。' : '';
    const L = sv.launch, tp = L.top / RPS, sd = L.side / RPS, ck = L.cork / RPS;
    const va = a.relAlong, vs = Math.sqrt(Math.max(0, bs * bs - va * va));
    const split = Math.abs(va) > 0.1 * bs && vs > 0.1 * bs; // 画面里只有两份都够大时才画红、绿两根箭头
    const red = split ? '<span class="cx">（红）</span>' : '', green = split ? '<span class="cy">（绿）</span>' : '';
    let upTxt, note = '';
    if (Math.abs(tp) < 6) upTxt = `上下方向只蹭了 ${Math.abs(va).toFixed(1)}，离拍时<span class="cx">${Math.abs(tp) < 1.5 ? '没有上下旋' : `只有一点${tp > 0 ? '上' : '下'}旋（${Math.abs(tp).toFixed(0)} 转/秒）`}</span>`;
    else if (tp > 0) upTxt = `顺着拍面<b class="in">往上</b>蹭 ${va.toFixed(1)}${red} → <span class="cx">上旋 ${tp.toFixed(0)} 转/秒</span>`;
    else upTxt = `顺着拍面<b class="in">往下</b>蹭 ${Math.abs(va).toFixed(1)}${red} → <span class="cx">下旋 ${Math.abs(tp).toFixed(0)} 转/秒</span>`;
    if (tp <= -6 && o >= 55) note = '拍面很平，"往下蹭"看上去就是朝你这边送。';
    else if (a.racketAlong < -0.3 && va > 0.3 && tp >= 6) note = '拍子自己在往下走，但球落得更快，相对球还是往上蹭。';
    let sideTxt = '';
    if (Math.abs(sd) >= 3 || Math.abs(ck) >= 6) {
      sideTxt = `；横着蹭 ${vs.toFixed(1)}${green} → <span class="cy">侧旋 ${Math.abs(sd).toFixed(0)}</span>`;
      if (Math.abs(ck) >= 6) sideTxt += `、<span class="cz">螺旋 ${Math.abs(ck).toFixed(0)}</span>`;
    }
    out.push(`<li><b>怎么蹭的</b>贴着球${dirWord(a.brush)}蹭 ${bs.toFixed(1)} m/s（<span class="cb">黄箭头</span>），同时朝球里顶 ${a.push.toFixed(1)} m/s。${thick}<br>${upTxt}${sideTxt}。${note ? `<i>${note}</i>` : ''}</li>`);
    // 3. 到你面前
    if (sv.arrive) {
      const R = sv.arrive.top / RPS, d = R - tp;
      const res = Math.abs(R) < TH_RPS ? '基本不转' : (R > 0 ? '上旋 ' : '下旋 ') + Math.abs(R).toFixed(0);
      const tbl = d > 1 ? `落两次台，台面把它往上旋方向带了 ${d.toFixed(0)}` : d < -1 ? `上旋太强，落台时被台面磨掉 ${(-d).toFixed(0)}` : '落两次台，上下旋几乎没变';
      let extra = '';
      if (sv.dead) extra = '这是"顶"出来的球：离拍几乎不转，落台后自己往前滚。拿搓下旋的手型去碰会冒高。';
      else if (sv.cls === 'flat') extra = '往下蹭了，但不够多，下旋在台面上磨光了。不到 5 转/秒就算不转。';
      out.push(`<li><b>到你面前</b>${tbl}，到你面前是<em class="${R >= TH_RPS ? 'up' : R <= -TH_RPS ? 'dn' : ''}">${res}</em>。${extra}</li>`);
    } else out.push(`<li><b>这球</b>${sv.verdict.why}。</li>`);
    if (sv.fakeKind) out.push(`<li><b>假动作</b>${FAKE_TXT[sv.fakeKind]}。那是球离拍之后的事，不算数。</li>`);
    return out.join('');
  }

  // ================= 练习流程 =================
  let serveId = 0;
  function adopt(sv) {
    sv.id = ++serveId; S.serve = sv; lastReadKey = '';
    prepServeVisuals(sv); $('#explain').innerHTML = explainHtml(sv);
  }
  function makeServe() {
    const s = S.set;
    const types = TYPE_KEYS.filter((k) => s.types[k]);
    const kinds = ['side', 'pure'].filter((k) => s.kinds[k]);
    const ps = [-1, 0, 1].filter((_, i) => s.pos[i]), ls = ['short', 'long'].filter((_, i) => s.len[i]);
    for (let k = 0; k < 10; k++) {
      const r = Math.random();
      const intent = s.choices === 3 ? (r < 0.38 ? 'top' : r < 0.76 ? 'back' : 'flat') : r < 0.5 ? 'top' : 'back';
      const cfg = { type: pick(types.length ? types : TYPE_KEYS), kind: pick(kinds.length ? kinds : ['side']), hand: s.hand, intent, pos: pick(ps.length ? ps : [-1, 0, 1]), len: pick(ls.length ? ls : ['short', 'long']),
        player: s.player === 'random' ? undefined : s.player, fake: s.fake === 'all' || (s.fake === 'some' && Math.random() < 0.45) };
      const sv = generateServe(cfg);
      if (sv) { sv.cfg = cfg; return sv; }
    }
    return null;
  }
  // 下一球提前备好：算一个发球要几十到几百毫秒（手机上更久），趁画面停着等你作答、或者慢放放完的时候先算，
  // 按"下一球"时就不用等。设置在这之后改过的话，备好的这个作废，按新设置现算
  let spare = null, spareTimer = 0;
  const setKey = () => { const s = S.set; return JSON.stringify([s.types, s.kinds, s.player, s.hand, s.pos, s.len, s.fake, s.choices]); };
  function prepareSpare() {
    clearTimeout(spareTimer);
    spareTimer = setTimeout(() => {
      if (S.mode !== 'practice' || S.playing || (spare && spare.key === setKey())) return;
      const key = setKey(), sv = makeServe();
      spare = sv ? { key, sv } : null;
    }, 120);
  }
  function nextServe() {
    const sv = spare && spare.key === setKey() ? spare.sv : makeServe();
    spare = null;
    if (!sv) { S.serve = null; renderStage(); return false; }
    adopt(sv);
    S.dirty = false; S.phase = 'ready'; S.answer = null; S.t = sv.tStart; S.playing = false; S.stopAt = null; S.auto = null; S.iso = 'all'; S.spinView = false;
    backToHome(true);
    syncUI(); renderStage();
    return true;
  }
  // 讲解时镜头会凑到拍子或球跟前；回到练习时，回到你自己选的那个视角
  function backToHome(newServe) {
    const special = S.view === 'ball' || S.view === 'contact';
    const v = special ? S.home : S.view;
    if (special || (newServe && (v === 'close' || v === 'side' || (v === 'recv' && Math.abs(recvAim(S.serve) - recvAx) > 0.03)))) setView(v);
  }
  function runPlay() { // 按练习的方式把这一球放一遍
    const sv = S.serve;
    S.phase = 'play'; S.t = sv.tStart + 0.25; S.rate = S.set.speed; S.playing = true; S.stopAt = playEndOf(sv); S.auto = null; S.live = null; S.spinView = false;
    backToHome(false);
    syncUI(); renderStage();
  }
  function playServe() {
    if (S.dirty || !S.serve) { if (!nextServe()) return; } // 设置改过：换一个符合新设置的球再发
    S.answer = null;
    runPlay();
  }
  function onPlayEnd() {
    if (S.mode === 'practice' && S.phase === 'play') { S.phase = 'ask'; renderStage(); }
    syncTransport();
    prepareSpare();
  }
  // 慢放整个动作：引拍 → 触球（放得最慢）→ 收拍 / 假动作 → 球飞完全程。中途不停
  function autoRate(sv, t) {
    if (t < -0.07) return 0.2;
    if (t < 0.05) return 0.05;
    if (t < sv.tSwingEnd + 0.1) return 0.2;
    return 0.5;
  }
  function showContact(replay) {
    const sv = S.serve; if (!sv) return;
    S.spinView = false; S.iso = 'all';
    if (replay) {
      setView('contact', false, true);
      S.t = Math.max(sv.tStart, sv.tSwing0 - 0.2); S.playing = true; S.stopAt = null;
      S.auto = { cam: true }; S.rate = autoRate(sv, S.t);
    } else { setView('contact'); S.t = 0; S.playing = false; S.stopAt = null; S.auto = null; }
    syncUI();
  }
  function showSpin(at) { // 凑近看球：默认停在到你面前的那一刻，球在原地慢慢转
    const sv = S.serve; if (!sv) return;
    S.playing = false; S.stopAt = null; S.auto = null;
    if (at !== undefined) S.t = Math.min(sv.tEnd, at);
    else if (sv.arrive) S.t = Math.min(sv.tEnd, sv.arrive.t + 0.05); else if (S.t < 0.05) S.t = Math.min(sv.tEnd, 0.08);
    S.spinView = true; $('#spinMore').open = true;
    const b = sv.ballAt(S.t); ballPos.set(b.p[0], b.p[1], b.p[2]);
    setView('ball');
    syncUI();
  }
  function answer(a) {
    const sv = S.serve; if (!sv || S.answer || (S.phase !== 'play' && S.phase !== 'ask') || S.t < 0) return;
    // 你是在球飞到哪儿的时候下的判断
    const tn = sv.verdict.net ? sv.verdict.net.t : 1e9, ta = sv.arrive ? sv.arrive.t : 1e9;
    S.ansWhen = S.phase === 'ask' || S.t >= ta ? '球落到你这边之后才判断的，实战里这时候该出手了' : S.t < tn ? '球过网之前就判断了' : '球过了网、还没落台时判断的';
    S.answer = a; S.phase = 'review';
    renderStage(); showContact(true);
  }
  function describe(sv) {
    const c = sv.cfg || {}, pl = PLAYERS[sv.player];
    return `${pl ? pl.name + ' · ' : ''}${TYPES[sv.type].short}${sv.hand < 0 ? '（左手）' : ''} · ${c.len === 'short' ? '近' : '远'}${sv.fake ? ' · 有假动作' : ''}`;
  }
  function brief(sv) { // 一句话：这球为什么是这个旋转
    const o = sv.an.openDeg, tp = sv.launch.top / RPS;
    const face = o >= 6 ? `拍面后仰 ${o.toFixed(0)}°` : o > -6 ? '拍面立着' : `拍面前倾 ${(-o).toFixed(0)}°`;
    if (sv.dead) return `${face}，是顶出来的：上下方向几乎没蹭，落台后自己往前滚`;
    if (Math.abs(tp) < 6) return `${face}，上下方向几乎没蹭${sv.cls === 'top' ? '，落台后才带上上旋' : ''}`;
    if (sv.cls === 'flat') return `${face}，往下蹭了但不够多，到你面前已经磨没了`;
    return `${face}，顺着拍面往${tp > 0 ? '上' : '下'}蹭`;
  }
  function renderStage() {
    renderStageInner(); inset();
  }
  function renderStageInner() {
    const st = $('#stage'), sv = S.serve;
    if (S.mode === 'lab') {
      st.innerHTML = `<div class="row"><button type="button" class="btn primary" data-act="labplay">播放这一球</button><button type="button" class="btn" data-act="hit">看触球瞬间</button><button type="button" class="btn" data-act="spin">看球怎么转</button><p class="hint" style="flex:1">拖左边的滑杆，球路和右边的讲解会跟着重算。</p></div>`;
      return;
    }
    if (!sv) { st.innerHTML = '<p class="hint">这组设置下没算出能上台的发球，换个设置再试。</p>'; return; }
    if (S.phase === 'ready') {
      st.innerHTML = `<div class="row"><button type="button" class="btn primary" data-act="serve">发球<kbd>空格</kbd></button>
        <p class="hint" style="flex:1">盯住触球那一下的拍型：拍面仰多少，拍子贴着球是往上蹭还是往下蹭。球一落到你这边就看不到了。</p></div>`;
    } else if (S.phase === 'play' || S.phase === 'ask') {
      const three = S.set.choices === 3, pure = S.set.kinds.pure, side = S.set.kinds.side;
      const up = pure && side ? ['上旋', '侧上也算'] : pure ? ['上旋', ''] : ['侧上', ''], dn = pure && side ? ['下旋', '侧下也算'] : pure ? ['下旋', ''] : ['侧下', ''];
      st.innerHTML = `<div class="row"><div class="answers">
          <button type="button" class="ans up" data-ans="top" disabled><b>${up[0]}</b><small>${up[1] ? up[1] + ' · ' : ''}键盘 ↑</small></button>
          ${three ? '<button type="button" class="ans" data-ans="flat" disabled><b>不转</b><small>到你面前基本不转 · 键盘 →</small></button>' : ''}
          <button type="button" class="ans down" data-ans="back" disabled><b>${dn[0]}</b><small>${dn[1] ? dn[1] + ' · ' : ''}键盘 ↓</small></button>
        </div><button type="button" class="btn" data-act="again">再看一遍</button></div>`;
      S.live = null;
    } else if (S.phase === 'review') {
      const ok = S.answer === sv.cls;
      st.innerHTML = `<div class="row"><div class="verdict ${ok ? 'ok' : 'no'}"><b>${ok ? '对了' : '看错了'}，是${clsName(sv)}</b><span>${brief(sv)}<i>${describe(sv)}</i><i>${S.ansWhen}</i></span></div>
        <button type="button" class="btn" data-act="replay">慢放重看</button><button type="button" class="btn" data-act="spin">看球怎么转</button><button type="button" class="btn primary" data-act="next">下一球<kbd>空格</kbd></button></div>`;
    }
  }
  $('#stage').addEventListener('click', (e) => {
    const b = e.target.closest('button'); if (!b || b.disabled) return;
    if (b.dataset.ans) return answer(b.dataset.ans);
    const act = b.dataset.act;
    if (act === 'serve') playServe();
    else if (act === 'again') runPlay();
    else if (act === 'next') { S.dirty = true; playServe(); }
    else if (act === 'replay') showContact(true);
    else if (act === 'hit') showContact(false);
    else if (act === 'spin') showSpin();
    else if (act === 'labplay') { S.spinView = false; S.auto = null; S.t = S.serve.tStart + 0.3; S.rate = 0.25; S.playing = true; S.stopAt = null; if (S.view === 'ball' || S.view === 'contact') setView(S.home === 'recv' ? 'side' : S.home); syncTransport(); }
  });

  // ================= 进度条 =================
  const scrub = $('#scrub');
  function syncTransport() {
    const was = $('#transport').hidden;
    $('#transport').hidden = !reviewing();
    if (was !== $('#transport').hidden) inset();
    $('#btnPlay').textContent = S.playing ? '暂停' : '播放';
    $$('#rateSeg button').forEach((b) => b.classList.toggle('on', !S.auto && Math.abs(parseFloat(b.dataset.rate) - S.rate) < 1e-6));
    $('#tgAids').classList.toggle('on', S.aids); $('#tgTrail').classList.toggle('on', S.trail);
  }
  function tickTransport() {
    const sv = S.serve; if (!sv || $('#transport').hidden) return;
    if (document.activeElement !== scrub || S.playing) scrub.value = String(Math.round(((S.t - sv.tStart) / (sv.tEnd - sv.tStart)) * 1000));
    $('#timeLbl').textContent = (S.t >= 0 ? '+' : '−') + Math.abs(S.t).toFixed(3) + ' s';
  }
  scrub.addEventListener('input', () => { const sv = S.serve; if (!sv) return; S.playing = false; S.stopAt = null; S.auto = null; S.t = sv.tStart + (parseFloat(scrub.value) / 1000) * (sv.tEnd - sv.tStart); syncTransport(); });
  $('#btnPlay').addEventListener('click', () => {
    const sv = S.serve; if (!sv) return;
    S.stopAt = null;
    if (!S.playing && S.t >= sv.tEnd - 1e-3) S.t = Math.max(sv.tStart, sv.tSwing0 - 0.2);
    S.playing = !S.playing;
    // 从触球特写接着往下放：球马上就飞出画面了，换回能看到整条球路的视角
    if (S.playing && S.view === 'contact' && S.t >= sv.tSwingEnd && !S.auto) setView(S.mode === 'lab' && S.home === 'recv' ? 'side' : S.home);
    syncTransport();
  });
  $('#rateSeg').addEventListener('click', (e) => { const b = e.target.closest('button'); if (!b) return; S.auto = null; S.rate = parseFloat(b.dataset.rate); syncTransport(); });
  function jump(where) { // 右侧三个时刻：镜头跟到那个时刻球所在的地方
    const sv = S.serve; if (!sv) return;
    if (where === 'hit') return showContact(false);
    if (where === 'arrive') return showSpin();
    const b1 = sv.verdict.b1;
    showSpin(clamp(b1 ? b1.t * 0.5 : 0.06, 0.012, 0.07)); // 刚离拍：还没落第一跳
  }
  $('#stageChips').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b && reviewing()) jump(b.dataset.stage); });
  $('#tgAids').addEventListener('click', () => { S.aids = !S.aids; syncTransport(); });
  $('#tgTrail').addEventListener('click', () => { S.trail = !S.trail; syncTransport(); });
  $('#spinRate').addEventListener('input', (e) => { S.spinRate = parseFloat(e.target.value) / 100; });

  // ================= 设置面板 =================
  // 改设置不会马上换掉眼前这个球，下一次发球才生效
  const touch = () => { S.dirty = true; save(); };
  const custom = () => { S.set.preset = 'custom'; };
  const segBind = (sel, attr, get, setv) => {
    const root = $(sel);
    root.addEventListener('click', (e) => { const b = e.target.closest('button'); if (!b) return; setv(b.dataset[attr]); syncUI(); });
    return () => $$('button', root).forEach((b) => b.classList.toggle('on', String(get()) === b.dataset[attr]));
  };
  const multiBind = (sel, attr, arrOrObj, keyOf, after) => {
    const root = $(sel);
    root.addEventListener('click', (e) => {
      const b = e.target.closest('button'); if (!b) return;
      const k = keyOf(b.dataset[attr]); const o = arrOrObj();
      o[k] = !o[k];
      if (!Object.values(o).some(Boolean)) o[k] = true; // 至少留一个
      if (after) after();
      touch(); syncUI();
    });
    return () => $$('button', root).forEach((b) => b.classList.toggle('on', !!arrOrObj()[keyOf(b.dataset[attr])]));
  };
  const syncers = [
    multiBind('#typeChips', 'type', () => S.set.types, (v) => v),
    multiBind('#kindChips', 'kind', () => S.set.kinds, (v) => v, () => { custom(); if (S.set.kinds.pure) S.set.choices = 3; }), // "转与不转"就得分出不转
    multiBind('#posChips', 'pos', () => S.set.pos, (v) => parseInt(v, 10)),
    multiBind('#lenChips', 'len', () => S.set.len, (v) => parseInt(v, 10)),
    segBind('#handSeg', 'hand', () => S.set.hand, (v) => { S.set.hand = parseInt(v, 10); touch(); }),
    segBind('#fakeSeg', 'fake', () => S.set.fake, (v) => { S.set.fake = v; custom(); touch(); }),
    segBind('#choiceSeg', 'ch', () => S.set.choices, (v) => { S.set.choices = parseInt(v, 10); custom(); touch(); }),
    segBind('#speedSeg', 'sp', () => S.set.speed, (v) => { S.set.speed = parseFloat(v); custom(); save(); }),
    segBind('#watchSeg', 'watch', () => S.set.watch, (v) => { S.set.watch = v; custom(); save(); }),
    segBind('#isoSeg', 'iso', () => S.iso, (v) => { S.iso = v; }),
  ];
  $('#presetChips').addEventListener('click', (e) => {
    const b = e.target.closest('button'); if (!b) return;
    const p = PRESETS[b.dataset.preset];
    Object.assign(S.set, { preset: b.dataset.preset, kinds: { ...p.kinds }, fake: p.fake, choices: p.choices, speed: p.speed, watch: p.watch, player: p.player });
    touch(); syncUI();
  });
  $('#playerSel').innerHTML = '<option value="random">每球随机换一个人</option>' + PLAYER_KEYS.map((k) => `<option value="${k}">${PLAYERS[k].name}</option>`).join('');
  $('#playerSel').addEventListener('change', (e) => { S.set.player = e.target.value; custom(); touch(); syncUI(); });
  $('#btnLeft').addEventListener('click', () => { $('#left').classList.toggle('open'); $('#right').classList.remove('open'); });
  $('#btnRight').addEventListener('click', () => { $('#right').classList.toggle('open'); $('#left').classList.remove('open'); });
  $('#views').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b) { S.spinView = false; S.home = b.dataset.view; setView(b.dataset.view); } });

  function syncUI() {
    syncers.forEach((f) => f());
    $$('#presetChips button').forEach((b) => b.classList.toggle('on', b.dataset.preset === S.set.preset));
    $('#presetNote').textContent = PRESETS[S.set.preset] ? PRESETS[S.set.preset].note : '自定义：下面每一项都按你选的来。';
    $('#playerSel').value = S.set.player;
    $('#playerNote').textContent = S.set.player === 'random' ? '站姿、抛球高度、引拍大小、收拍方式每球都换。' : PLAYERS[S.set.player].note + '。';
    $$('.tabs button').forEach((b) => b.classList.toggle('on', b.dataset.mode === S.mode));
    $('#panelPractice').hidden = S.mode !== 'practice'; $('#panelLab').hidden = S.mode !== 'lab';
    $('#readBody').hidden = !reviewing(); $('#readLock').hidden = reviewing(); $('#right').classList.toggle('locked', !reviewing());
    $('#spinCtl').hidden = !S.spinView;
    $('#pending').hidden = !(S.mode === 'practice' && S.dirty && S.phase !== 'review');
    syncTransport();
  }

  // ================= 实验台 =================
  const LAB_SL = [
    ['openDeg', '拍面后仰角（0 立着，90 放平）', -40, 110, 1, '°'],
    ['yawDeg', '拍面左右偏（正 = 朝你的右边）', -180, 180, 1, '°'], // 拍面接近放平时，这个角可以是任意值
    ['vf', '拍速 · 向前（朝你）', -3, 12, 0.1, ' m/s'],
    ['vu', '拍速 · 向上', -8, 8, 0.1, ' m/s'],
    ['vs', '拍速 · 朝你的右边', -10, 10, 0.1, ' m/s'],
    ['vfall', '触球时球的下落速度', 0, 6, 0.1, ' m/s'],
    ['hc', '触球高度（离台面）', 5, 35, 1, ' cm'],
    ['xc', '发球位置（左右）', -60, 60, 1, ' cm'],
    ['mu', '胶皮摩擦系数', 0.2, 2, 0.05, ''],
    ['et', '胶皮切向弹性（0 不回弹）', 0, 0.95, 0.01, ''],
  ];
  const labGet = (k) => { const L = S.lab; return k === 'vf' ? L.vr[2] : k === 'vu' ? L.vr[1] : k === 'vs' ? L.vr[0] : k === 'hc' ? L.hc * 100 : k === 'xc' ? L.xc * 100 : L[k]; };
  const labSet = (k, v) => { const L = S.lab; if (k === 'vf') L.vr[2] = v; else if (k === 'vu') L.vr[1] = v; else if (k === 'vs') L.vr[0] = v; else if (k === 'hc') L.hc = v / 100; else if (k === 'xc') L.xc = v / 100; else L[k] = v; };
  $('#labSliders').innerHTML = LAB_SL.map(([k, name, mn, mx, stp]) => `<div class="sl"><label for="lab_${k}">${name}</label><output id="out_${k}"></output><input type="range" id="lab_${k}" min="${mn}" max="${mx}" step="${stp}"></div>`).join('');
  function syncLabSliders() {
    for (const [k, , mn, mx, stp, unit] of LAB_SL) { const v = clamp(labGet(k), mn, mx); labSet(k, v); $('#lab_' + k).value = v; $('#out_' + k).textContent = (stp >= 1 ? v.toFixed(0) : stp >= 0.1 ? v.toFixed(1) : v.toFixed(2)) + unit; }
    $('#labType').value = S.lab.type;
  }
  function rebuildLab() {
    const L = S.lab;
    const sv = manualServe({ ...L, vr: L.vr.slice(), rubber: { ...P.rubber, mu: L.mu, et0: L.et } });
    adopt(sv);
    S.phase = 'lab'; S.playing = false; S.stopAt = null; S.auto = null; S.t = 0; S.spinView = false;
    const el = $('#labStatus');
    el.className = sv.legal ? 'ok' : 'bad';
    el.textContent = sv.legal ? `合法发球：落在你这边离网 ${sv.verdict.b2.p[2].toFixed(2)} m，过网余量 ${(sv.verdict.net.clear * 100).toFixed(1)} cm，到你面前是${{ top: '上旋', back: '下旋', flat: '不转' }[sv.cls]}` : `不合法：${sv.verdict.why}`;
    renderStage(); syncUI();
  }
  $('#labSliders').addEventListener('input', (e) => { const k = e.target.id.replace('lab_', ''); labSet(k, parseFloat(e.target.value)); syncLabSliders(); rebuildLab(); });
  $('#labType').addEventListener('change', (e) => { S.lab.type = e.target.value; rebuildLab(); });
  $$('.labEx').forEach((b) => b.addEventListener('click', () => {
    let sv = null;
    for (let k = 0; k < 8 && !sv; k++) sv = generateServe({ type: S.lab.type, hand: 1, intent: b.dataset.ex, pos: 1, len: pick(['short', 'long']), player: 'std', fake: false });
    if (!sv) return;
    Object.assign(S.lab, toLab(sv), { mu: P.rubber.mu, et: P.rubber.et0 });
    syncLabSliders(); rebuildLab(); setView('side');
  }));
  let labSeeded = false;
  function seedLab() {
    if (labSeeded) return; labSeeded = true;
    for (let k = 0; k < 8; k++) {
      const sv = generateServe({ type: S.lab.type, hand: 1, intent: 'back', pos: 1, len: 'long', player: 'std', fake: false });
      if (sv) { Object.assign(S.lab, toLab(sv)); break; }
    }
  }
  function setMode(m) {
    S.mode = m; S.spinView = false; S.auto = null;
    $('#spinMore').open = m === 'lab'; // 实验台里转速读数是主角，练习时先收着
    if (m === 'lab') { seedLab(); syncLabSliders(); rebuildLab(); setView('side'); }
    else { nextServe(); setView('recv'); }
    try { if (location.hash !== '#' + m) history.replaceState(null, '', '#' + m); } catch (e) { /* 忽略 */ }
    syncUI(); renderStage();
  }
  $('.tabs').addEventListener('click', (e) => { const b = e.target.closest('button'); if (b && b.dataset.mode !== S.mode) setMode(b.dataset.mode); });
  window.addEventListener('hashchange', () => { const m = location.hash === '#lab' ? 'lab' : 'practice'; if (m !== S.mode) setMode(m); });

  // ================= 键盘 =================
  window.addEventListener('keydown', (e) => {
    if (e.target && /INPUT|SELECT|TEXTAREA/.test(e.target.tagName)) return;
    if (e.code === 'Space') {
      e.preventDefault();
      if (S.mode === 'lab') $('#btnPlay').click();
      else if (S.phase === 'ready') playServe();
      else if (S.phase === 'review') { S.dirty = true; playServe(); }
    } else if (S.mode === 'practice' && (S.phase === 'play' || S.phase === 'ask')) {
      if (e.key === 'ArrowUp') { e.preventDefault(); answer('top'); }
      else if (e.key === 'ArrowDown') { e.preventDefault(); answer('back'); }
      else if (e.key === 'ArrowRight' && $('#stage [data-ans="flat"]')) { e.preventDefault(); answer('flat'); }
    } else if (reviewing() && (e.key === 'ArrowLeft' || e.key === 'ArrowRight')) {
      e.preventDefault(); S.playing = false; S.stopAt = null; S.auto = null; S.t = clamp(S.t + (e.key === 'ArrowLeft' ? -1 : 1) * (e.shiftKey ? 0.02 : 0.002), S.serve.tStart, S.serve.tEnd); syncTransport();
    }
  });

  // ================= 主循环 =================
  let last = performance.now();
  function frame(now) {
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    const sv = S.serve;
    if (sv) {
      if (S.playing) {
        if (S.auto) {
          S.rate = autoRate(sv, S.t);
          // 收拍看完了：镜头退回你选的视角，接着看球飞过来
          if (S.auto.cam && S.t > sv.tSwingEnd + 0.1 && !S.spinView) { S.auto.cam = false; setView(S.mode === 'lab' && S.home === 'recv' ? 'side' : S.home); }
        }
        S.t += dt * S.rate;
        if (S.stopAt !== null && S.t >= S.stopAt) { S.t = S.stopAt; S.playing = false; S.stopAt = null; onPlayEnd(); }
        else if (S.t >= sv.tEnd) { S.t = sv.tEnd; S.playing = false; S.auto = null; onPlayEnd(); }
      }
      if (S.mode === 'practice' && (S.phase === 'play' || S.phase === 'ask')) { // 触球之后才能作答
        const live = S.t >= 0 || S.phase === 'ask';
        if (live !== S.live) { S.live = live; $$('#stage .ans').forEach((b) => { b.disabled = !live; }); }
      }
      prevBall.copy(ballPos);
      apply(dt);
      if (S.view === 'ball' && !camAnim) { _v.copy(ballPos).sub(prevBall); camera.position.add(_v); controls.target.add(_v); } // 凑近看球时镜头跟着球
      tickTransport();
    }
    if (camAnim) {
      camAnim.u += dt / camAnim.dur; const e = smooth(Math.min(1, camAnim.u));
      camera.position.lerpVectors(camAnim.p0, camAnim.p1, e); controls.target.lerpVectors(camAnim.t0, camAnim.t1, e);
      if (camAnim.u >= 1) camAnim = null;
    }
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(frame);
  }

  resize();
  const m0 = location.hash === '#lab' ? 'lab' : 'practice';
  S.mode = m0;
  $('#spinMore').open = m0 === 'lab';
  if (m0 === 'lab') { seedLab(); syncLabSliders(); rebuildLab(); setView('side', true); } else { nextServe(); setView('recv', true); }
  syncUI(); renderStage();
  window.__pp = { S, setView, nextServe, playServe, answer, jump, setMode, showContact, showSpin, camera, controls, ballVisible: () => ballPivot.visible,
    cam: (p, t) => { camAnim = null; camera.position.set(p[0], p[1], p[2]); controls.target.set(t[0], t[1], t[2]); },
    force: (cfg) => { const sv = generateServe(cfg); if (!sv) return null; sv.cfg = cfg; adopt(sv); S.dirty = false; S.phase = 'ready'; S.answer = null; S.t = sv.tStart; S.playing = false; S.auto = null; S.spinView = false; syncUI(); renderStage(); return sv.cls; } };
  requestAnimationFrame(frame);
}

if (window.claude && window.claude.hot && window.claude.hot.ready) window.claude.hot.ready(start);
else start((window.claude && window.claude.hot && window.claude.hot.data) || {});
