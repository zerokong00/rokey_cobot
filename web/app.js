// pipe_comm 관제 패널 — 셸: 라우터 + 웹소켓 + 공유 상태 + 지령.
//
// ━━ 구조 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
//   app.js         이 파일 — 사이드바/라우팅, WS 수신, store, cmd()
//   views/*.js     페이지 하나당 모듈. `mount(el)` 을 export 하고,
//                  정리할 게 있으면 unmount 함수를 돌려준다.
//
// 🔑 **웹소켓은 하나이고 페이지 전환에도 끊기지 않는다.** 라우팅을 서버가
//    아니라 브라우저(History API)가 하는 이유가 이것이다 — 전환마다 재접속하면
//    카메라가 매번 끊기고 지난 결함·사건을 다시 받아야 한다. 서버는 어느
//    경로로 들어와도 같은 셸을 준다(web_panel.py 의 경로 목록과 맞출 것).
//
// 🔑 화면에 없는 페이지도 **상태는 계속 쌓인다** — store 가 받아 두고,
//    페이지는 mount 될 때 store 를 통째로 읽어 그린 뒤 bus 를 구독한다.

import * as home from '/static/views/home.js';
import * as camera from '/static/views/camera.js';
import * as handling from '/static/views/handling.js';
import * as map3d from '/static/views/map3d.js';
import * as detect from '/static/views/detect.js';
import * as events from '/static/views/events.js';

// ── 아이콘 (인라인 SVG — 외부 폰트/CDN 없이) ──────────────────
const ico = {
  home: '<path d="M3 9.5 10 3l7 6.5"/><path d="M5 8.5V17h10V8.5"/>',
  cam: '<rect x="2" y="5.5" width="13" height="9.5" rx="2"/>'
     + '<path d="M15 9.5 18.5 7v6.5L15 11"/>',
  robot: '<rect x="4" y="7" width="12" height="9" rx="2"/>'
       + '<path d="M10 7V4"/><circle cx="7.5" cy="11" r="1"/>'
       + '<circle cx="12.5" cy="11" r="1"/>',
  map: '<path d="M2 5.5 7.3 3l5.4 2.5L18 3v11.5L12.7 17 7.3 14.5 2 17z"/>'
     + '<path d="M7.3 3v11.5M12.7 5.5V17"/>',
  list: '<path d="M7 5.5h11M7 10h11M7 14.5h11"/><circle cx="3.4" cy="5.5" r="1"/>'
      + '<circle cx="3.4" cy="10" r="1"/><circle cx="3.4" cy="14.5" r="1"/>',
  log: '<path d="M4.5 3h7l4 4v10a1 1 0 0 1-1 1h-10a1 1 0 0 1-1-1V4a1 1 0 0 1 1-1z"/>'
     + '<path d="M11 3v4.5h4.5M7 11h6M7 14h4"/>',
};

// 🔑 경로 목록의 단일 출처. 서버(web_panel.py)의 라우트 목록과 **같아야**
//    직접 주소를 치거나 새로고침해도 404 가 안 난다.
export const ROUTES = [
  {path: '/home', label: 'Home', title: 'Home', icon: ico.home, view: home},
  {path: '/handling', label: 'Robot Handling', title: 'Robot Handling',
   icon: ico.robot, view: handling},
  {path: '/map', label: '3D Map', title: '3D Map', icon: ico.map,
   view: map3d},
  {path: '/detect', label: 'Detect List', title: 'Detect List',
   icon: ico.list, view: detect, badge: () => store.defectRows.size},
];

// ── 아주 작은 이벤트 버스 ────────────────────────────────────
// 종류: 'state' 'course' 'event' 'defect' 'cmd' 'frame'(role) 'conn' 'reset'(run)
function makeBus() {
  const m = new Map();
  return {
    on(evt, fn) {
      if (!m.has(evt)) m.set(evt, new Set());
      m.get(evt).add(fn);
      return () => m.get(evt).delete(fn);      // 해제 함수를 돌려준다
    },
    emit(evt, data) {
      const s = m.get(evt);
      if (s) for (const fn of [...s]) {
        try { fn(data); } catch (e) { console.error(evt, e); }
      }
    },
  };
}
export const bus = makeBus();

// ── 공유 상태 ────────────────────────────────────────────────
// 🚨 카메라는 **Blob 으로만** 들고 있는다. objectURL 을 여기서 만들면 화면에
//    없는 페이지 몫까지 10Hz 로 URL 이 쌓여 새는데, Blob 은 다음 프레임이
//    덮어쓰면 그대로 회수된다. URL 은 보는 쪽이 showFrame() 으로 만든다.
export const store = {
  state: null,            // drive_state (+roll/odom/max_s 보강)
  course: null,           // {pts:[[s,x,y,z]], ir_m, s_total_m}
  events: [],             // 사건 (오래된 것부터)
  cmds: [],               // 이 패널이 보낸 지령 이력 (서버가 기록해 흘려 준다)
  defectRows: new Map(),  // defect_id → 리포트 dict
  marks: [],              // 3D 맵 마커 {id, s, clock, repaired}
  maxS: 0,                // 지나간 최대 호길이 (m)
  run: null,              // 시연 판 번호 (서버가 센다). 바뀌면 = 재시작
  cam: {front: null, rear: null, torch: null},   // {blob, t, fps}
  connected: false,
};

const CH = {1: 'front', 2: 'rear', 3: 'torch'};   // web_panel.py 의 CAM_CH

/** 지금 켜진 역할만 남기고 나머지 카메라의 Blob 을 놓아 준다.
 *
 * 🔑 시연이 한 번에 한 대만 발행하므로, 안 켜진 역할의 마지막 프레임은 아무도
 *    안 보면서 메모리만 붙들고 있다. 역할이 바뀌는 순간 버린다. */
export function dropOtherCams(active) {
  if (!active) return;
  for (const r of Object.keys(store.cam))
    if (r !== active) store.cam[r] = null;
}

/** <img> 하나에 최신 프레임을 걸어 준다. 이전 URL 은 회수한다. */
export function showFrame(img, role) {
  const c = store.cam[role];
  if (!c || !c.blob) return false;
  const url = URL.createObjectURL(c.blob);
  const old = img.dataset.url;
  // 🔑 회수를 못 한 URL 을 `prev` 에 적어 둔다 — 로드가 끝나기 전에 이 <img>
  //    를 비우면(카메라 분기 전환) onload 가 안 불려서 그 하나가 샌다.
  if (old) img.dataset.prev = old;
  img.onload = () => {
    if (old) URL.revokeObjectURL(old);
    if (img.dataset.prev === old) delete img.dataset.prev;
  };
  img.dataset.url = url;
  img.src = url;
  return true;
}

// ── 웹소켓 ───────────────────────────────────────────────────
const connEl = document.getElementById('conn');
function setConn(ok, msg) {
  store.connected = ok;
  connEl.className = 'conn ' + (ok ? 'on' : 'off');
  connEl.textContent = msg;
  bus.emit('conn', ok);
}

function onState(d) {
  // 🔑 시연 재시작(Isaac GUI 의 Stop→Play) — 서버가 run 번호를 올려서 알린다.
  //    지나온 초록선과 수리 마커는 **그 판에서만 참인 것**이라 같이 지운다.
  //    사건 로그는 남긴다 — 로그는 지난 판까지 포함해서 읽는 것이다.
  if (d.run !== undefined && store.run !== null && d.run !== store.run) {
    store.maxS = 0;
    store.marks.length = 0;
    bus.emit('reset', d.run);
  }
  if (d.run !== undefined) store.run = d.run;
  // 카메라 분기가 바뀌면 지난 역할의 프레임은 바로 놓는다 — 화면(카메라 칸)이
  // 떠 있지 않아도 store 는 계속 받으므로, 여기서 버려야 실제로 안 쌓인다.
  if (d.cam && d.cam !== (store.state || {}).cam) dropOtherCams(d.cam);
  store.state = d;
  store.maxS = Math.max(store.maxS, (d.max_s_mm || d.s_mm || 0) / 1000);
  bus.emit('state', d);
}

function onEvent(d) {
  // 용접 사건은 3D 맵 마커로도 남는다 — 맵 페이지가 떠 있지 않아도 쌓아 둔다.
  if (d.event === 'WELD_BEGIN' && typeof d.defect_s_mm === 'number') {
    const s = d.defect_s_mm / 1000;
    if (!store.marks.some(m => Math.abs(m.s - s) < 0.02))
      store.marks.push({id: 'defect', s,
        clock: (typeof d.clock_deg === 'number') ? d.clock_deg : null});
  }
  if (d.event === 'WELD_DONE') {
    // defect_s_mm/clock_deg(결함 자체의 좌표)가 있으면 그대로 쓰고, 없으면
    // (구판 브리지) 로봇이 물러난 시점의 s 로 근사한다 — 가장 가까운 미수리
    // 결함(±100mm)에 붙이고 없으면 새로 찍는다.
    const s = (typeof d.defect_s_mm === 'number' ? d.defect_s_mm
                                                 : d.s_mm || 0) / 1000;
    const ck = (typeof d.clock_deg === 'number') ? d.clock_deg : null;
    let best = null;
    for (const m of store.marks)
      if (!m.repaired && Math.abs(m.s - s) < 0.1
          && (best === null || Math.abs(m.s - s) < Math.abs(best.s - s)))
        best = m;
    if (best) { best.repaired = true; if (ck !== null) best.clock = ck; }
    else store.marks.push({id: 'weld', s, clock: ck, repaired: true});
  }
  store.events.push(d);
  bus.emit('event', d);
}

function onDefect(d) {
  const id = d.defect_id || '?';
  store.defectRows.set(id, d);
  const s = (d.defect || {}).axial_position_from_entry_m;
  if (typeof s === 'number') {
    const ck = (typeof d.defect.clock_angle_deg === 'number')
      ? d.defect.clock_angle_deg : null;
    const m = store.marks.find(x => x.id === id);
    if (m) { m.s = s; if (ck !== null) m.clock = ck; }
    else store.marks.push({id, s, clock: ck});
  }
  bus.emit('defect', d);
}

function connect() {
  const ws = new WebSocket(
    (location.protocol === 'https:' ? 'wss://' : 'ws://')
    + location.host + '/ws');
  ws.binaryType = 'arraybuffer';
  ws.onopen = () => setConn(true, '● 서버 연결됨');
  ws.onmessage = (e) => {
    if (typeof e.data === 'string') {
      const m = JSON.parse(e.data);
      if (m.type === 'state') onState(m.data);
      else if (m.type === 'event') onEvent(m.data);
      else if (m.type === 'defect') onDefect(m.data);
      else if (m.type === 'cmd') {
        // 서버가 기록해 흘려 주는 지령 이력(다른 브라우저·새로고침 전 것 포함).
        // 🚨 지금은 그리는 화면이 없다(Robot Handling 을 비웠다) — 그래도
        //    받아는 두되 **상한을 건다.** 아무도 안 보는 배열이 무한히 자라는
        //    것이 제일 조용한 누수다.
        store.cmds.push(m.data);
        if (store.cmds.length > 200) store.cmds.splice(0, 100);
        bus.emit('cmd', m.data);
      }
      else if (m.type === 'course') {
        store.course = m.data;
        bus.emit('course', m.data);
      }
      return;
    }
    const v = new Uint8Array(e.data);
    const role = CH[v[0]];
    if (!role) return;
    const prev = store.cam[role];
    const t = performance.now();
    store.cam[role] = {
      blob: new Blob([v.subarray(1)], {type: 'image/jpeg'}), t,
      // 프레임 간격의 지수이동평균 → 초당 장수. 튀는 값을 눌러 준다.
      fps: prev ? (prev.fps || 0) * 0.8 + 0.2 * (1000 / Math.max(t - prev.t, 1))
                : 0,
    };
    bus.emit('frame', role);
  };
  ws.onclose = () => {
    setConn(false, '○ 연결 끊김 — 3초 후 재접속');
    setTimeout(connect, 3000);
  };
  ws.onerror = () => ws.close();
}

// ── 지령 ─────────────────────────────────────────────────────
export async function cmd(c, extra) {
  const r = await fetch('/cmd', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(Object.assign({cmd: c}, extra || {})),
  });
  return r.json();
}

// ── 라우터 ───────────────────────────────────────────────────
const navEl = document.getElementById('nav');
const viewEl = document.getElementById('view');
const titleEl = document.getElementById('page-title');
let unmount = null;

for (const r of ROUTES) {
  const li = document.createElement('li');
  li.innerHTML = `<a href="${r.path}" data-path="${r.path}">`
    + `<svg viewBox="0 0 20 20">${r.icon}</svg><span>${r.label}</span>`
    + `<span class="badge" hidden></span></a>`;
  li.querySelector('a').onclick = (e) => {
    if (e.metaKey || e.ctrlKey || e.shiftKey) return;   // 새 탭은 그대로
    e.preventDefault();
    go(r.path);
  };
  navEl.appendChild(li);
}

function refreshBadges() {
  for (const r of ROUTES) {
    if (!r.badge) continue;
    const el = navEl.querySelector(`a[data-path="${r.path}"] .badge`);
    const n = r.badge();
    el.hidden = !n;
    el.textContent = n;
  }
}
bus.on('event', refreshBadges);
bus.on('defect', refreshBadges);

function render(path) {
  const r = ROUTES.find(x => x.path === path) || ROUTES[0];
  if (unmount) { try { unmount(); } catch (e) { console.error(e); } }
  unmount = null;
  viewEl.replaceChildren();
  titleEl.textContent = r.title;
  document.title = `${r.title} — pipe_comm 관제`;
  for (const a of navEl.querySelectorAll('a'))
    a.classList.toggle('on', a.dataset.path === r.path);
  unmount = r.view.mount(viewEl) || null;
}

export function go(path) {
  if (location.pathname !== path) history.pushState({}, '', path);
  render(path);
}
window.addEventListener('popstate', () => render(location.pathname));

// ── 머리말의 상태 한 줄 (모든 페이지 공통) ──────────────────
const headEl = document.getElementById('head-state');
export function stateLine(d) {
  if (!d) return '상태 수신 대기…';
  const tot = d.s_total_mm || 0;
  const pct = tot ? (100 * d.s_mm / tot).toFixed(0) + '%' : '-';
  return `<span class="k">${d.moving ? '주행' : '정지'}</span> · `
    + `<span class="k">${d.state || '?'}</span> · `
    + `s <span class="k">${(d.s_mm || 0).toFixed(0)}mm</span> `
    + `${d.dir > 0 ? '→' : '←'} ${pct}`
    + (d.reason ? ` · ${d.reason}` : '');
}
bus.on('state', d => { headEl.innerHTML = stateLine(d); });

// ── 기동 ─────────────────────────────────────────────────────
setConn(false, '○ 연결 대기…');
connect();
// `/` 로 들어오면 홈으로 주소를 바꿔 준다 — 사이드바 표시와 URL 을 맞춘다.
render(ROUTES.some(r => r.path === location.pathname)
  ? location.pathname : (history.replaceState({}, '', '/home'), '/home'));
