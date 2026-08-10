// 3D Map — three.js 배관 맵.
//
// ━━ 데이터 소스 — 전부 동적이다. 여기 좌표를 하드코딩하지 않는다 ━━━━━
//   코스 중심선  ← `course` 토픽(latched). 시연(real_map_demo --ros)의
//                  CenterLine 이 단일 출처다. 표본 [[s,x,y,z],...] (m).
//   CAD 메시     ← GET /mesh (.webmesh — tools/usd_to_webmesh.py 로 굽는다)
//   로봇·결함    ← drive_state 의 pos_m (코스와 같은 월드 m 좌표)
//
// three.js 는 /static 벤더 사본(오프라인 동작). Z-up 으로 쓴다 — Isaac/ROS
// 규약과 좌표를 맞추기 위해 DEFAULT_UP 을 바꾼다(회전 컨트롤도 Z 축 기준).
//
// 🔑 장면은 **모듈 수준에 한 번만** 만든다. 페이지를 오갈 때마다 WebGL 컨텍스트
//    와 메시를 새로 굽는 것은 비싸고, 브라우저의 컨텍스트 개수 한도(보통 16)에
//    걸리면 조용히 렌더가 죽는다. mount 는 캔버스를 다시 붙이고 rAF 만 켠다.

import * as THREE from 'three';
import {OrbitControls} from '/static/OrbitControls.js';
import {store, bus} from '/static/app.js';

const W3 = 960, H3 = 460;

// ── 코스 기하 헬퍼 (store.course 를 읽는다) ─────────────────
let PTS = null, IR = 0.05, S_TOTAL = 0;
const V = {
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  mul: (a, k) => [a[0] * k, a[1] * k, a[2] * k],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0]],
  norm: a => { const l = Math.hypot(a[0], a[1], a[2]) || 1;
               return [a[0] / l, a[1] / l, a[2] / l]; },
};

function sToXyz(s) {         // 호길이 → 3D 좌표 (표본 사이는 선형 보간)
  s = Math.max(PTS[0].s, Math.min(s, PTS[PTS.length - 1].s));
  let lo = 0, hi = PTS.length - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; PTS[m].s <= s ? lo = m : hi = m; }
  const A = PTS[lo], B = PTS[hi], t = (s - A.s) / Math.max(B.s - A.s, 1e-9);
  return [0, 1, 2].map(k => A.p[k] + (B.p[k] - A.p[k]) * t);
}
function wallP(s, aRad, r = IR) {
  // 호길이 s + 시계각(라디안, 규약: 0=천장, π=바닥) → 관 **벽면** 좌표
  const c = sToXyz(s);
  const t = V.norm(V.sub(sToXyz(s + 0.005), sToXyz(s - 0.005)));
  const up = Math.abs(t[2]) > 0.9 ? [1, 0, 0] : [0, 0, 1];
  const u = V.norm(V.cross(t, up)), v = V.norm(V.cross(u, t));  // v = 천장
  // 🔑 sin 항은 -u — Isaac 쪽 규약(atan2(Δy,Δz): +90° = +Y)과 맞춘다.
  //    u = t×up 는 입구 직관에서 -Y 라 부호를 안 뒤집으면 거울상이 된다.
  return V.add(c, V.add(V.mul(v, r * Math.cos(aRad)),
                        V.mul(u, -r * Math.sin(aRad))));
}
const wallXyz = (s, clock) =>
  clock == null ? sToXyz(s) : wallP(s, clock * Math.PI / 180);

function snapCourse(pos) {   // 월드 좌표 → 중심선에서 가장 가까운 {s, p}
  let best = null;
  for (let i = 0; i + 1 < PTS.length; i++) {
    const A = PTS[i].p, B = PTS[i + 1].p, AB = V.sub(B, A), AP = V.sub(pos, A);
    const L2 = AB[0] ** 2 + AB[1] ** 2 + AB[2] ** 2;
    const t = L2 > 0 ? Math.max(0, Math.min(1,
      (AP[0] * AB[0] + AP[1] * AB[1] + AP[2] * AB[2]) / L2)) : 0;
    const Q = V.add(A, V.mul(AB, t));
    const d2 = (pos[0] - Q[0]) ** 2 + (pos[1] - Q[1]) ** 2
             + (pos[2] - Q[2]) ** 2;
    if (best === null || d2 < best.d2)
      best = {d2, p: Q, s: PTS[i].s + (PTS[i + 1].s - PTS[i].s) * t};
  }
  return best;
}

// ── 장면 (모듈에 한 번) ──────────────────────────────────────
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);          // Z-up (Isaac/ROS 규약)
let renderer = null, glError = null;
try {
  renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
  renderer.setPixelRatio(window.devicePixelRatio || 1);
  renderer.setSize(W3, H3, false);
  renderer.setClearColor(0x000000, 0);
  renderer.autoClear = false;
  renderer.domElement.style.width = '100%';
} catch (e) {
  glError = 'WebGL 사용 불가 — 브라우저 하드웨어 가속을 켤 것';
}

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(50, W3 / H3, 0.01, 100);
camera.position.set(-1, -2, 1.5);
scene.add(new THREE.AmbientLight(0xffffff, 0.9));
{
  const d1 = new THREE.DirectionalLight(0xffffff, 1.6);
  d1.position.set(0.4, 0.3, 0.85); scene.add(d1);
  const d2 = new THREE.DirectionalLight(0xffffff, 0.5);
  d2.position.set(-0.7, 0.5, -0.2); scene.add(d2);
}
const meshGroup = new THREE.Group(); scene.add(meshGroup);      // CAD 메시
const courseGroup = new THREE.Group(); scene.add(courseGroup);  // 코스 튜브
const markerGroup = new THREE.Group(); scene.add(markerGroup);  // 결함·스티커
let progressMesh = null, gridHelper = null;

// 🔴 로봇 점. 스프라이트로 두되 **월드 크기(sizeAttenuation:true)** 라
//    확대하면 커지고 축소하면 작아진다. 다만 전체 맵(≈5m)까지 줌아웃하면
//    관 내반경 50mm 짜리 점이 1px 밑으로 사라지므로, 매 프레임 화면상
//    크기를 재서 **최소 픽셀로 바닥을 받친다**(fitRobotDot). 옛 2D 판의
//    `r = max(2, 0.01*k)` 와 같은 규칙이다.
//
// 🚨 `transparent: true` 가 **반드시** 있어야 한다. three 는 불투명/반투명을
//    두 목록으로 갈라 불투명을 먼저 그리고, renderOrder 는 목록 **안에서만**
//    순서를 정한다. 이게 없으면 점은 불투명 목록에서 먼저 그려지고, 그 뒤에
//    오는 반투명 CAD 메시(배관 0.80 / 벽 0.22)가 그 위를 덮어 버린다 —
//    라벨만 뜨고 점은 안 보이는 증상이 정확히 이것이었다.
function dotSprite(draw) {
  const cv = document.createElement('canvas');
  cv.width = cv.height = 64;
  draw(cv.getContext('2d'));
  return new THREE.Sprite(new THREE.SpriteMaterial({
    map: new THREE.CanvasTexture(cv), transparent: true,
    depthTest: false}));
}
const robotMesh = dotSprite(g => {
  g.beginPath(); g.arc(32, 32, 26, 0, 7);
  g.fillStyle = '#ff3333'; g.fill();
  g.lineWidth = 6; g.strokeStyle = '#ffffff'; g.stroke();
});
robotMesh.renderOrder = 11;              // 결함 마커(9)보다도 위
robotMesh.visible = false;
scene.add(robotMesh);

// 점의 실제 크기 = 관 내반경 × 1.4 (내경 100mm 관이면 70mm — 관을 거의 채워
// 로봇이 거기 있다는 게 읽힌다). 최소 픽셀은 **안전망일 뿐**이다: 이 조합이면
// 카메라가 8.6m 보다 멀 때만 걸리는데, 기본 시점이 전체 맵에서도 7.2m 라
// 실사용 줌 구간에서는 항상 배율대로 커지고 작아진다.
const DOT_WORLD_K = 1.4;
const DOT_MIN_PX = 5;
/** 로봇 점 크기를 정한다 — 관 크기에 비례하되 화면에서 사라지지는 않게.
 *
 * 원근 투영에서 월드 크기 s 인 스프라이트의 화면 높이(px)는
 *     px = P₁₁ · s / dist · H / 2      (P₁₁ = 1/tan(fov/2), H = 표시 높이)
 * 이므로, 최소 픽셀을 보장하는 월드 크기는 s = 2·px·dist / (P₁₁·H) 다.
 */
function fitRobotDot(viewH) {
  const world = IR * DOT_WORLD_K;
  const dist = camera.position.distanceTo(robotMesh.position);
  const p11 = camera.projectionMatrix.elements[5];
  const floor = 2 * DOT_MIN_PX * dist / Math.max(p11 * viewH, 1e-6);
  robotMesh.scale.setScalar(Math.max(world, floor));
  robotMesh.scale.z = 1;
}

const axScene = new THREE.Scene();
axScene.add(new THREE.AxesHelper(1));            // X빨강 Y초록 Z파랑
const axCam = new THREE.PerspectiveCamera(50, 1, 0.1, 10);
axCam.up.set(0, 0, 1);

// OrbitControls 는 mount 마다 새로 만든다 — r160 에는 connect/disconnect 가
// 없고 dispose 뿐이라, 안 보이는 페이지에서 드래그를 먹지 않으려면 떼는 수밖에
// 없다. 카메라는 모듈에 살아 있고 target 만 여기 받아 두면 시점이 유지된다.
let controls = null;
const camTarget = new THREE.Vector3();
let framedBy = null;      // 'mesh' > 'course'
let meshLoaded = false, meshOn = true, meshNote = '';
let entryPos = null, courseBuilt = -1, progressBuilt = -1;

class ArcCurve extends THREE.Curve {   // 호길이 파라미터 곡선 (선형 보간)
  constructor(sMax) { super(); this.sMax = sMax; }
  getPoint(t) { return new THREE.Vector3(...sToXyz(t * this.sMax)); }
}

function frame(lo, hi, by) {
  if (framedBy === 'mesh' && by !== 'mesh') return;
  framedBy = by;
  const c = new THREE.Vector3((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2,
                              (lo[2] + hi[2]) / 2);
  const dim = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2], 0.5);
  camera.position.set(c.x - 0.5 * dim, c.y - 1.1 * dim, c.z + 0.8 * dim);
  camera.near = dim / 100; camera.far = dim * 30;
  camera.updateProjectionMatrix();
  camTarget.copy(c);
  if (controls) { controls.target.copy(c); controls.update(); }
  else camera.lookAt(c);
  if (gridHelper) { scene.remove(gridHelper); gridHelper.geometry.dispose(); }
  const size = Math.ceil((dim + 0.6) * 10) / 10;      // 바닥 격자 0.1m
  gridHelper = new THREE.GridHelper(size, Math.round(size / 0.1),
                                    0x333333, 0x222222);
  gridHelper.rotation.x = Math.PI / 2;                // XZ 평면 → XY 로
  gridHelper.position.set(c.x, c.y, lo[2] - IR * 1.5);
  scene.add(gridHelper);
}

function buildCourse() {
  const msg = store.course;
  if (!msg || courseBuilt === msg.stamp) return;
  courseBuilt = msg.stamp;
  PTS = msg.pts.map(q => ({s: q[0], p: [q[1], q[2], q[3]]}));
  IR = msg.ir_m || 0.05;
  S_TOTAL = msg.s_total_m || PTS[PTS.length - 1].s;
  for (const o of courseGroup.children) if (o.geometry) o.geometry.dispose();
  courseGroup.clear();
  courseGroup.add(new THREE.Mesh(          // 관 튜브 (반투명)
    new THREE.TubeGeometry(new ArcCurve(S_TOTAL),
                           Math.max(96, PTS.length * 2), IR, 14, false),
    new THREE.MeshLambertMaterial({color: 0x4a7aa8, transparent: true,
      opacity: 0.35, side: THREE.DoubleSide, depthWrite: false})));
  courseGroup.add(new THREE.Line(          // 중심선
    new THREE.BufferGeometry().setFromPoints(
      PTS.map(o => new THREE.Vector3(...o.p))),
    new THREE.LineBasicMaterial({color: 0x3a5a78})));
  entryPos = PTS[0].p;
  const bb = PTS.reduce((m, o) => ({
    lo: m.lo.map((x, k) => Math.min(x, o.p[k])),
    hi: m.hi.map((x, k) => Math.max(x, o.p[k]))}),
    {lo: [1e9, 1e9, 1e9], hi: [-1e9, -1e9, -1e9]});
  frame(bb.lo.map(x => x - 2 * IR), bb.hi.map(x => x + 2 * IR), 'course');
  progressBuilt = -1;
  buildProgress(); buildMarks();
}

function clearProgress() {   // 재시작 — 지나온 초록선을 걷어낸다
  // 🚨 `buildProgress()` 만으로는 안 지워진다 — maxS 가 0 이면 맨 위에서
  //    바로 돌아가 **옛 메시가 장면에 그대로 남는다.** 지우는 길을 따로 둔다.
  if (progressMesh) {
    scene.remove(progressMesh);
    progressMesh.geometry.dispose();
    progressMesh = null;
  }
  progressBuilt = -1;
}

function buildProgress() {   // 지나온 구간 (초록 굵은 선)
  if (!PTS) return;
  // 🔑 답파거리가 **뒤로 갔다** = 시연 재시작(app.js 가 store.maxS 를 0 으로
  //    되돌린다). 3D 맵을 안 보고 있는 사이에 재시작이 나도 여기서 걸린다 —
  //    mount 가 이 함수를 다시 부르기 때문이다.
  if (progressMesh && store.maxS < progressBuilt - 0.001) clearProgress();
  if (store.maxS <= 0.01) return;
  if (progressMesh && store.maxS - progressBuilt < 0.02) return;
  progressBuilt = store.maxS;
  if (progressMesh) { scene.remove(progressMesh); progressMesh.geometry.dispose(); }
  progressMesh = new THREE.Mesh(
    new THREE.TubeGeometry(new ArcCurve(Math.min(store.maxS, S_TOTAL)),
                           128, 0.007, 8, false),
    // transparent 가 있어야 반투명 CAD 메시보다 **뒤에** 그려진다(위 참고)
    new THREE.MeshBasicMaterial({color: 0x2f8f5a, transparent: true,
                                 depthTest: false}));
  progressMesh.renderOrder = 8;
  scene.add(progressMesh);
}

function spriteMat(draw) {
  const cv = document.createElement('canvas'); cv.width = cv.height = 64;
  draw(cv.getContext('2d'));
  return new THREE.SpriteMaterial({map: new THREE.CanvasTexture(cv),
    transparent: true, depthTest: false});
}
const xMat = spriteMat(g => {
  g.strokeStyle = '#f96'; g.lineWidth = 10; g.lineCap = 'round';
  g.beginPath(); g.moveTo(14, 14); g.lineTo(50, 50);
  g.moveTo(50, 14); g.lineTo(14, 50); g.stroke();
});
const sqMat = spriteMat(g => {
  g.fillStyle = '#fd4'; g.fillRect(12, 12, 40, 40);
  g.strokeStyle = '#a80'; g.lineWidth = 6; g.strokeRect(12, 12, 40, 40);
});

function buildMarks() {
  if (!PTS) return;
  // 🚨 Sprite 의 geometry 는 three 가 **모든 스프라이트끼리 공유**하는 하나다.
  //    여기서 dispose 하면 다른 마커까지 같이 날린다 — 스프라이트는 건너뛴다.
  for (const o of markerGroup.children)
    if (o.geometry && !o.isSprite) o.geometry.dispose();
  markerGroup.clear();
  for (const d of store.marks) {
    if (d.repaired && d.clock != null) {
      // 수리 스티커 = 관 벽면에 밀착한 곡면 패치 (축 ±16mm × 원주 ±25°).
      // 벽면보다 1mm 안쪽으로 띄워 튜브 표면과의 z-fight 를 피한다.
      const a0 = d.clock * Math.PI / 180, dS = 0.016,
            dA = 25 * Math.PI / 180, N = 6, pos = [], idx = [];
      for (const s of [d.s - dS, d.s + dS])
        for (let i = 0; i <= N; i++)
          pos.push(...wallP(s, a0 - dA + 2 * dA * i / N, IR - 0.001));
      for (let i = 0; i < N; i++)
        idx.push(i, i + 1, N + 1 + i, i + 1, N + 2 + i, N + 1 + i);
      const g = new THREE.BufferGeometry();
      g.setAttribute('position',
        new THREE.BufferAttribute(new Float32Array(pos), 3));
      g.setIndex(idx); g.computeVertexNormals();
      const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({
        color: 0xffdd44, side: THREE.DoubleSide, transparent: true,
        opacity: 0.9, depthTest: false}));
      m.renderOrder = 9;
      markerGroup.add(m);
    } else {
      // ✕(미수리) / ■(시계각 미상) 은 항상 카메라를 보는 스프라이트
      const sp = new THREE.Sprite(d.repaired ? sqMat : xMat);
      sp.position.set(...wallXyz(d.s, d.clock));
      sp.scale.set(0.05, 0.05, 1);
      sp.renderOrder = 9;
      markerGroup.add(sp);
    }
  }
}

function updateRobot() {
  // 🔴 pos_m(월드 좌표)이 오면 중심선에 스냅한다. 없으면(구판) s 로.
  const d = store.state;
  if (!PTS || !d) { robotMesh.visible = false; return null; }
  const pos = (Array.isArray(d.pos_m) && d.pos_m.length === 3) ? d.pos_m : null;
  let p3, sLab;
  if (pos) { const sn = snapCourse(pos); p3 = sn.p; sLab = sn.s; }
  else { p3 = sToXyz((d.s_mm || 0) / 1000); sLab = (d.s_mm || 0) / 1000; }
  robotMesh.position.set(...p3);
  robotMesh.visible = true;
  return `s=${(sLab * 1000).toFixed(0)}mm`;
}

// ── CAD 메시 — .webmesh 를 한 번만 fetch 한다 ────────────────
// 포맷·좌표계는 tools/usd_to_webmesh.py 참고. 파트별로 Mesh 를 갈라 두면
// three.js 가 반투명 정렬(뒤→앞)을 파트 단위로 해 준다. BufferAttribute 는
// 전 파트가 공유한다(업로드 1회).
const GROUP_COLOR = {floor2: 0x5c9ee0, floor1: 0x54b38c,
                     aisle: 0x9999a8, etc: 0xb39966};
let meshPromise = null;
function loadMesh() {
  // 실패해도 코스 튜브로 동작하지만, 왜 메시가 안 떴는지는 화면에 반드시
  // 적는다 — 조용한 실패는 "파이프만 나온다" 문의가 된다(실화).
  if (meshPromise) return meshPromise;
  meshPromise = (async () => {
    if (!renderer) { meshNote = glError; return; }
    let buf;
    try {
      const r = await fetch('/mesh');
      if (!r.ok) { meshNote = '메시 파일 없음 — usd_to_webmesh.py 로 구울 것';
                   return; }
      buf = await r.arrayBuffer();
    } catch (e) { meshNote = '메시 fetch 실패: ' + e; return; }
    const L = new DataView(buf).getUint32(0, true);
    const hdr = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, L)));
    let off = 4 + L; off += (4 - off % 4) % 4;
    const nv = hdr.vtx_count, nt = hdr.tri_count;
    const pos = new Float32Array(buf, off, nv * 3);
    const aP = new THREE.BufferAttribute(pos, 3);
    const aN = new THREE.BufferAttribute(
      new Float32Array(buf, off + nv * 12, nv * 3), 3);
    const aI = new THREE.BufferAttribute(
      new Uint32Array(buf, off + nv * 24, nt * 3), 1);
    for (const p of hdr.parts) {
      // 벽/배관 분류 — 두 번째로 긴 축이 1m 를 넘는 파트는 방 껍데기(벽)다
      // (이 맵 실측: 벽 3개 1.5×1.8m+, 배관은 전부 0.7m 이하). 벽은 아주
      // 옅게 깔아 안의 배관·로봇이 비쳐 보이게 한다.
      const mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9];
      for (let i = p.vtx_start * 3; i < (p.vtx_start + p.vtx_count) * 3; i += 3)
        for (let k = 0; k < 3; k++) {
          const v = pos[i + k];
          if (v < mn[k]) mn[k] = v;
          if (v > mx[k]) mx[k] = v;
        }
      const ext = mx.map((x, k) => x - mn[k]).sort((a, b) => b - a);
      const shell = ext[1] > 1.0;
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', aP); g.setAttribute('normal', aN);
      g.setIndex(aI); g.setDrawRange(p.idx_start, p.idx_count);
      // 공유 어트리뷰트라 자동 bounding 계산이 전체 맵을 잡는다 — 파트 bbox 로
      // 직접 넣어 준다(반투명 뒤→앞 정렬이 이 구를 쓴다).
      g.boundingSphere = new THREE.Sphere(
        new THREE.Vector3((mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2,
                          (mn[2] + mx[2]) / 2),
        Math.hypot(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]) / 2);
      const m = new THREE.Mesh(g, new THREE.MeshLambertMaterial({
        color: shell ? 0x9fa6b8 : (GROUP_COLOR[p.group] || GROUP_COLOR.etc),
        transparent: true,
        opacity: shell ? 0.22 : (p.group === 'floor2' ? 0.80 : 0.50),
        side: THREE.DoubleSide, depthWrite: false}));
      m.renderOrder = shell ? 0 : 1;       // 벽(모든 걸 감싼다)을 먼저
      meshGroup.add(m);
    }
    meshLoaded = true;
    frame(hdr.bbox[0], hdr.bbox[1], 'mesh');
    meshNote = `${hdr.source} · ${(nt / 1000).toFixed(1)}k tri`;
  })();
  return meshPromise;
}

// ── 페이지 ───────────────────────────────────────────────────
export function mount(el) {
  el.innerHTML = `
   <div class="legend">
    <span><b style="color:#f33">●</b> 로봇</span>
    <span><b style="color:#f96">✕</b> 결함</span>
    <span><b style="color:#fd4">■</b> 수리 완료</span>
    <span class="muted">우클릭 회전 · 휠 눌러 이동 · 휠 굴려 확대</span>
    <label style="margin-left:auto"><input type="checkbox" id="m-ck" checked>
     CAD 메시</label>
    <span class="muted" id="m-note"></span>
   </div>
   <div id="map3d"><div id="labels">
    <div class="lab" id="lab-robot" style="color:#f88;display:none"></div>
    <div class="lab" id="lab-entry" style="color:#888;display:none">입구</div>
   </div></div>
   <div class="legend" id="m-course"></div>`;

  const wrap = el.querySelector('#map3d');
  const note = el.querySelector('#m-note');
  const ck = el.querySelector('#m-ck');
  const labRobot = el.querySelector('#lab-robot');
  const labEntry = el.querySelector('#lab-entry');
  const courseNote = el.querySelector('#m-course');

  if (!renderer) {
    wrap.innerHTML = `<div class="empty" style="padding:60px;
      text-align:center">${glError}</div>`;
    return null;
  }
  wrap.insertBefore(renderer.domElement, wrap.firstChild);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.15;
  // 마우스 배정: 좌 = **아무것도 안 함**, 휠클릭 = 이동, 우 = 회전.
  // (기본값은 좌=회전, 휠클릭=줌, 우=이동이라 셋 다 바꾼 것이다.) null 로
  // 두면 onMouseDown 의 switch 가 default 로 떨어져 state=NONE 이 된다.
  controls.mouseButtons = {LEFT: null,
                           MIDDLE: THREE.MOUSE.PAN,
                           RIGHT: THREE.MOUSE.ROTATE};
  controls.target.copy(camTarget);         // 지난 시점을 이어받는다
  controls.update();
  // 🚨 OrbitControls 의 pointerdown 은 기본 동작을 막지 않는다 — 그대로 두면
  //    휠 클릭에 브라우저 **자동 스크롤**(닻 아이콘)이 떠서 이동과 겹친다.
  //    가운데·오른쪽 버튼의 기본 동작만 여기서 막는다(왼쪽은 그대로).
  const noDefault = e => { if (e.button === 1 || e.button === 2)
                             e.preventDefault(); };
  renderer.domElement.addEventListener('mousedown', noDefault);
  renderer.domElement.addEventListener('pointerdown', noDefault);
  ck.checked = meshOn;
  ck.onchange = () => { meshOn = ck.checked; meshGroup.visible = meshOn; };
  meshGroup.visible = meshOn;

  function placeLabel(elm, p3, dx) {
    const v = new THREE.Vector3(...p3).project(camera);
    if (v.z > 1 || Math.abs(v.x) > 1.1 || Math.abs(v.y) > 1.1) {
      elm.style.display = 'none'; return;
    }
    elm.style.display = 'block';
    elm.style.left = ((v.x * 0.5 + 0.5) * wrap.clientWidth + dx) + 'px';
    elm.style.top = ((-v.y * 0.5 + 0.5) * wrap.clientHeight) + 'px';
  }

  function drawCourseNote() {
    const c = store.course;
    courseNote.innerHTML = c
      ? `<span class="muted">코스 ${(c.s_total_m * 1000).toFixed(0)}mm · `
        + `표본 ${c.pts.length}점 · 답파 `
        + `${(store.maxS * 1000).toFixed(0)}mm</span>`
      : '<span class="muted">코스 수신 대기 — 시연을 <code>--ros</code> 로 '
        + '띄울 것</span>';
    note.textContent = meshNote ? `(${meshNote})` : '';
  }

  // 🚨 buildProgress 를 따로 부른다 — buildCourse 는 코스가 그대로면 바로
  //    돌아가서, 화면을 떠나 있는 동안 일어난 재시작을 못 반영한다.
  buildCourse(); buildProgress(); buildMarks(); drawCourseNote();
  loadMesh().then(drawCourseNote);

  let raf = 0;
  function tick() {
    raf = requestAnimationFrame(tick);
    controls.update();
    courseGroup.visible = !(meshLoaded && meshOn);
    // 줌·회전이 매 프레임 바뀌므로 점 크기도 매 프레임 다시 잰다.
    // 화면에 보이는 높이(CSS px)로 재야 캔버스가 가로 100% 로 늘어난 배율까지
    // 반영된다 — 캔버스 백버퍼(H3)로 재면 창 크기에 따라 어긋난다.
    if (robotMesh.visible) fitRobotDot(wrap.clientHeight || H3);
    renderer.clear();
    renderer.render(scene, camera);
    // 좌표계 게이지 (왼쪽 아래 인셋) — Z 가 위 (Isaac/ROS 규약).
    // autoClear=false 라 본 장면의 깊이가 남아 있다 — 이 칸만 깊이를 지운다.
    const s = 76, m = 6;
    renderer.setScissorTest(true);
    renderer.setViewport(m, m, s, s); renderer.setScissor(m, m, s, s);
    renderer.clearDepth();
    axCam.position.copy(camera.position).sub(controls.target).setLength(2.8);
    axCam.lookAt(0, 0, 0);
    renderer.render(axScene, axCam);
    renderer.setScissorTest(false);
    renderer.setViewport(0, 0, W3, H3);
    if (robotMesh.visible) placeLabel(labRobot, robotMesh.position.toArray(), 10);
    else labRobot.style.display = 'none';
    if (entryPos) placeLabel(labEntry, entryPos, 8);
  }
  labRobot.textContent = updateRobot() || '';
  tick();

  const off = [
    bus.on('state', () => {
      const lab = updateRobot();
      if (lab) labRobot.textContent = lab;
      buildProgress(); drawCourseNote();
    }),
    bus.on('course', () => { buildCourse(); drawCourseNote(); }),
    bus.on('event', buildMarks),
    bus.on('defect', buildMarks),
    // 시연 재시작 — 초록선과 마커는 지난 판의 것이다 (app.js 의 onState).
    bus.on('reset', (run) => {
      clearProgress(); buildMarks(); drawCourseNote();
      console.log('[map3d] 시연 재시작 — 답파 구간 초기화 (run', run, ')');
    }),
  ];

  return () => {
    cancelAnimationFrame(raf);
    off.forEach(f => f());
    // 🔑 캔버스는 버리지 않고 떼기만 한다 — 다음 mount 가 그대로 다시 쓴다
    //    (WebGL 컨텍스트·업로드된 메시를 살려 둔다). 컨트롤만 dispose 해서
    //    전역 리스너를 떼어야 안 보이는 페이지에서 드래그를 먹지 않는다.
    camTarget.copy(controls.target);
    controls.dispose();
    controls = null;
    renderer.domElement.removeEventListener('mousedown', noDefault);
    renderer.domElement.removeEventListener('pointerdown', noDefault);
    renderer.domElement.remove();
  };
}
