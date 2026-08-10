// Camera — 지금 켜져 있는 카메라 **한 대만** 본다.
//
// 🚨 시연은 한 번에 한 대만 발행한다: 전진이면 전방, 후진(RETURN·RECOVER)이면
//    후방, 정렬~아크(ALIGN·EXTEND·ARC)면 토치. 분기 규칙의 단일 출처는 시연의
//    `active_camera_name()` 이고, 켜진 역할은 `drive_state.cam` 으로 온다.
//
// 🔑 **화면도 한 칸만 만든다.** 예전에는 3칸을 나란히 띄워 놓고 둘은 늘 "대기"
//    였는데, 안 쓰는 칸이 마지막 프레임을 디코딩된 채로 붙들고 있었다(브라우저는
//    <img> 에 걸린 그림을 화면 밖이라고 버리지 않는다). 분기가 바뀌면 쓰던 칸을
//    비우고(objectURL 회수 + src 제거) 새 역할로 갈아 끼운다.
import {store, bus, showFrame, dropOtherCams} from '/static/app.js';

const CAM = {
  front: {name: '전방 카메라', topic: 'rgb/compressed', when: '전진 주행'},
  rear: {name: '후방 카메라', topic: 'rear/rgb/compressed', when: '후진·복귀'},
  torch: {name: '토치 카메라', topic: 'torch/rgb/compressed', when: '정렬~용접'},
};

/** 카메라 한 칸을 el 안에 만든다. 해제 함수를 돌려준다.
 *  opts.bare — 카드 안에 넣을 때(테두리·배경 없이). */
export function mountCam(el, opts = {}) {
  el.innerHTML = `
   <div class="cam${opts.bare ? ' bare' : ''}">
    <div class="capt"><b id="cm-name">—</b>
     <span class="live" id="cm-live" hidden>● LIVE</span>
     <span class="t" id="cm-when"></span>
     <span class="fps" id="cm-fps">—</span></div>
    <div class="fr"><img id="cm-img" alt="카메라">
     <div class="nosig" id="cm-nosig">프레임 대기 —
      시연을 <code>--ros</code> 로 띄웠는지 확인</div></div>
   </div>`;

  const $ = id => el.querySelector('#' + id);
  const img = $('cm-img'), nosig = $('cm-nosig'), fps = $('cm-fps');
  const name = $('cm-name'), when = $('cm-when'), live = $('cm-live');
  let role = null;                 // 지금 이 칸이 보여 주는 역할

  /** 지금 켜진 역할. 구판 시연(cam 필드 없음)이면 전방으로 본다. */
  const active = () => (store.state || {}).cam || 'front';

  /** 칸을 비운다 — objectURL 을 회수하고 그림을 떼어 낸다.
   *  🚨 src 를 그냥 두면 역할이 바뀌어도 **앞 카메라의 마지막 그림**이 남는다
   *     (제목만 바뀌어 화면이 거짓말을 한다). 메모리도 그만큼 안 풀린다. */
  function clearFrame() {
    for (const k of ['url', 'prev']) {
      if (img.dataset[k]) {
        URL.revokeObjectURL(img.dataset[k]);
        delete img.dataset[k];
      }
    }
    img.removeAttribute('src');
    img.onload = null;
    fps.textContent = '—';
    nosig.hidden = false;
  }

  function setRole(r) {
    if (r === role) return;
    role = r;
    clearFrame();
    const c = CAM[r] || {name: r, when: ''};
    name.textContent = c.name;
    when.textContent = c.when || '';
    nosig.textContent = `${c.name} 프레임 대기…`;
    live.hidden = true;
    // 🔑 다른 역할이 붙들고 있던 프레임(Blob)을 놓아 준다 — 안 보는 카메라의
    //    그림을 들고 있을 이유가 없다.
    dropOtherCams(r);
    draw(r);
  }

  function draw(r) {
    if (r !== role) return;        // 지금 보는 역할의 프레임만 그린다
    if (showFrame(img, r)) {
      nosig.hidden = true;
      live.hidden = false;
      const c = store.cam[r];
      fps.textContent = c.fps ? `${c.fps.toFixed(1)} fps` : '';
    }
  }

  setRole(active());

  // 프레임이 끊기면 fps 가 옛날 값에 얼어붙는다 — 1초마다 늙히고, 오래 끊기면
  // 덮개를 다시 덮는다. 지금 켜진 카메라가 끊긴 것은 **정상이 아니다**.
  const timer = setInterval(() => {
    const s = store.cam[role];
    if (!s) return;
    const age = (performance.now() - s.t) / 1000;
    if (age > 2) fps.textContent = `${age.toFixed(0)}초 전`;
    if (age > 3) {
      nosig.textContent = `${age.toFixed(0)}초째 프레임 없음`;
      nosig.hidden = false;
      live.hidden = true;
    }
  }, 1000);

  const off = [
    bus.on('frame', draw),
    bus.on('state', () => setRole(active())),
  ];
  return () => {
    off.forEach(f => f());
    clearInterval(timer);
    clearFrame();                  // 페이지를 떠날 때도 그림을 놓는다
  };
}

export function mount(el) {
  const wrap = document.createElement('div');
  wrap.className = 'cams one';
  el.appendChild(wrap);
  const legend = document.createElement('div');
  legend.className = 'legend';
  legend.innerHTML = '어안 140° · 10Hz JPEG · <b>한 번에 한 대만</b> 켜진다 '
    + '(전진 전방 / 후진 후방 / 정렬~아크 토치). 화면도 한 칸만 만든다 — '
    + '분기가 바뀌면 이 칸이 새 카메라로 갈아 끼워진다.';
  el.appendChild(legend);
  return mountCam(wrap);
}
