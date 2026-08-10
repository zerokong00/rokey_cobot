// Robot Handling — 조종석. 지령 버튼 + 지금 켜진 카메라 + 상태 + 사건 로그.
//
// 🔑 이 화면 하나만 보고 시연을 몰 수 있어야 한다(사용자 요청으로 Camera·
//    Event Log 를 여기로 합쳤다). 그래서 카메라 칸과 로그 칸은 **각 페이지의
//    모듈을 그대로 끼워 쓴다** — 같은 것을 두 벌 만들면 한쪽만 고쳐진다.
//
// 🚨 발행은 성공해도 **구독자가 0 이면 지령은 조용히 사라진다**(RELIABLE 이지
//    latch 가 아니다). 그래서 서버가 구독자 수를 세어 돌려주고, 여기서 그걸
//    그대로 띄운다 — "눌렀는데 아무 일도 없다" 를 진단으로 바꾼다.
import {store, bus, cmd} from '/static/app.js';
import {mountCam} from '/static/views/camera.js';
import {mountLog} from '/static/views/events.js';

export function mount(el) {
  el.innerHTML = `
   <div class="cards">
    <div class="card wide">
     <h2>임무 지령</h2>
     <div class="btns">
      <button id="b-run" class="go">▶ 시작</button>
      <button id="b-forward">⏩ 전진 점검</button>
      <button id="b-recall">↩ 복귀</button>
      <button id="b-estop">⛔ 비상정지</button>
      <button id="b-release">🔓 비상정지 해제</button>
     </div>
     <div id="toast"></div>
    </div>

    <div class="card wide">
     <h2>카메라 <span class="muted" id="h-camwhen"></span></h2>
     <div id="h-cam"></div>
    </div>

    <div class="split">
     <div class="card">
      <h2>현재 상태</h2>
      <div class="rowlist">
       <div class="row"><span class="l">FSM</span>
        <span class="v" id="k-state">—</span></div>
       <div class="row"><span class="l">주행</span>
        <span class="v" id="k-moving">—</span></div>
       <div class="row"><span class="l">방향</span>
        <span class="v" id="k-dir">—</span></div>
       <div class="row"><span class="l">진행</span>
        <span class="v" id="k-s">—</span></div>
       <div class="row"><span class="l">지령 속도</span>
        <span class="v" id="k-speed">—</span></div>
       <div class="row"><span class="l">사유</span>
        <span class="v" id="k-reason">—</span></div>
      </div>
      <div class="bar" id="k-bar"><i style="width:0"></i></div>
     </div>

     <div class="card">
      <h2>사건 로그 <span class="muted">event</span></h2>
      <div id="h-log"></div>
     </div>
    </div>
   </div>`;

  const $ = id => el.querySelector('#' + id);
  const toast = $('toast');

  async function send(c, extra) {
    toast.textContent = '전송 중…';
    toast.style.color = '';
    try {
      const j = await cmd(c, extra);
      toast.textContent = j.ok ? `✓ ${c} 전송 (구독자 ${j.subscribers})`
                               : (j.warn || j.error || '실패');
      toast.style.color = j.ok ? 'var(--ok)' : 'var(--warn)';
    } catch (e) {
      toast.textContent = '서버에 못 보냈다: ' + e;
      toast.style.color = 'var(--bad)';
    }
  }

  // ── ▶ 시작 / ⏸ 정지 토글 ────────────────────────────────────
  // 🔑 버튼의 모양은 **로봇이 실제로 보내오는 상태**를 따른다(내가 뭘 눌렀는지가
  //    아니라). 시연이 지령을 미룰 수 있으므로 — 용접 중 정지는 시퀀스가 끝나야
  //    걸린다 — 누르자마자 뒤집으면 버튼이 거짓말을 한다.
  // 🚨 그래서 "보냈는데 아직 안 걸린" 구간(pend)을 따로 표시한다. 이게 없으면
  //    눌러도 글자가 안 변해서 안 먹은 줄 알고 계속 누르게 된다.
  const runBtn = $('b-run');
  let pend = null;                 // 'STOP' | 'START' — 상태가 따라오길 기다림

  function isHeld(d) {
    // HOLD = 지령 정지, DEAD = 비상정지. 둘 다 "지금 서 있다".
    return !d || d.state === 'HOLD' || d.state === 'DEAD';
  }

  function syncRun() {
    const d = store.state, held = isHeld(d);
    const estop = d && d.state === 'DEAD', done = d && d.state === 'DONE';
    // 임무가 끝나면 기다리던 지령도 없던 일이 된다 — 안 그러면 "정지 대기…"
    // 가 영영 안 풀린다(끝난 로봇은 HOLD 로 가지 않는다).
    if (done || (pend && ((pend === 'STOP' && held)
                          || (pend === 'START' && !held))))
      pend = null;
    runBtn.classList.toggle('go', held && !done);
    runBtn.disabled = !!(estop || done);
    runBtn.textContent =
      estop ? '⛔ 비상정지 중' :
      done ? '✔ 임무 완료' :
      pend === 'STOP' ? '⏸ 정지 대기…' :
      pend === 'START' ? '▶ 시작 대기…' :
      held ? '▶ 시작' : '⏸ 정지';
  }

  runBtn.onclick = () => {
    const c = isHeld(store.state) ? 'START' : 'STOP';
    pend = c;
    syncRun();
    send(c);
  };

  $('b-forward').onclick = () => send('FORWARD');
  $('b-recall').onclick = () => send('RECALL');
  $('b-estop').onclick = () => {
    if (confirm('비상정지는 지금 있는 자리에서 그대로 언다 (아크도 끈다).\n'
                + '풀려면 "🔓 비상정지 해제" 를 따로 눌러야 한다. 실행할까?'))
      send('ESTOP');
  };
  // 🔑 해제는 **START 에 reason 을 실어** 보낸다 — 규약에 지령을 새로 만들지
  //    않으면서도, 실수로 ▶ 시작을 눌러 비상정지가 풀리는 일은 막는다.
  //    풀어도 바로 안 간다(정지 상태로 돌아온다) — 다시 ▶ 시작을 눌러야 한다.
  $('b-release').onclick = () => {
    if (confirm('비상정지를 해제한다. 로봇 주변이 안전한지 확인했는가?\n'
                + '(해제해도 바로 움직이지 않는다 — ▶ 시작을 다시 눌러야 한다)'))
      send('START', {reason: 'ESTOP_RELEASE'});
  };

  // ── 카메라 · 사건 로그 (각 페이지 모듈을 그대로 끼운다) ────────
  const CAM_WHEN = {front: '전진 — 전방', rear: '후진·복귀 — 후방',
                    torch: '정렬~용접 — 토치'};
  const camWhen = $('h-camwhen');
  const offCam = mountCam($('h-cam'), {bare: true});
  const log = mountLog($('h-log'), {limit: 60});

  function draw() {
    syncRun();                       // 상태가 아직 없어도 버튼은 맞춰 둔다
    const d = store.state;
    if (!d) return;
    camWhen.textContent = CAM_WHEN[d.cam] || '';
    $('k-state').textContent = d.state || '—';
    $('k-moving').textContent = d.moving ? '주행' : '정지';
    $('k-dir').textContent = d.dir > 0 ? '전진 →' : '후진 ←';
    const tot = d.s_total_mm || 0;
    $('k-s').textContent = `${(d.s_mm || 0).toFixed(0)} / ${tot.toFixed(0)} mm`;
    $('k-speed').textContent = `${((d.speed_mps || 0) * 1000).toFixed(0)} mm/s`;
    // 🔑 지령으로 서 있으면 시연이 `HOLD(CRUISE)` 처럼 **원래 FSM 상태를
    //    괄호에 넣어** 보낸다 — 무엇을 하다 멈췄는지가 여기서만 보인다.
    $('k-reason').textContent = d.reason || '—';
    const bar = $('k-bar');
    bar.firstElementChild.style.width =
      tot ? `${Math.min(100, 100 * (d.s_mm || 0) / tot).toFixed(1)}%` : '0';
    bar.classList.toggle('done', d.state === 'DONE');
  }

  draw();
  const off = bus.on('state', draw);
  return () => { off(); offCam(); log.off(); };
}
