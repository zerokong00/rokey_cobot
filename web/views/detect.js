// Detect List — dongyeon 검출 노드의 결함 리포트(/defect/report_json).
import {store, bus} from '/static/app.js';

function row(d) {
  const def = d.defect || {}, m = d.measurement || {};
  const s = def.axial_position_from_entry_m;
  const size = (m.length_mm != null && m.width_mm != null)
    ? `${m.length_mm.toFixed(1)}×${m.width_mm.toFixed(1)}mm` : '-';
  const repaired = store.marks.some(
    x => x.id === (d.defect_id || '?') && x.repaired);
  return `<td>${d.defect_id || '?'}</td><td>${d.class || '?'}</td>`
    + `<td class="num">${((d.confidence || 0) * 100).toFixed(0)}%</td>`
    + `<td class="num">${typeof s === 'number'
        ? (s * 1000).toFixed(0) + 'mm' : '-'}</td>`
    + `<td class="num">${def.clock_angle_deg != null
        ? def.clock_angle_deg.toFixed(0) + '° (' + def.clock_hour + '시)'
        : '-'}</td>`
    + `<td class="num">${size}</td><td>${d.registration || '-'}</td>`
    + `<td>${repaired ? '<span class="pill run">수리 완료</span>'
                      : '<span class="pill warn">대기</span>'}</td>`;
}

export function mount(el) {
  el.innerHTML = `
   <div class="card wide">
    <h2>결함 목록 <span class="muted">/defect/report_json</span></h2>
    <div class="scroll">
     <table><thead><tr><th>ID</th><th>종류</th><th>신뢰도</th>
      <th>입구에서</th><th>시계각</th><th>크기(길이×폭)</th><th>등록</th>
      <th>상태</th></tr></thead>
     <tbody id="d-body"></tbody></table>
    </div>
    <div class="empty" id="d-empty">수신 대기 — 검출 노드가 떠 있는가?</div>
   </div>`;

  const body = el.querySelector('#d-body'), empty = el.querySelector('#d-empty');

  function draw() {
    const rows = [...store.defectRows.values()];
    empty.hidden = rows.length > 0;
    // 최신이 위 — 규약에 순번이 없으니 들어온 순서를 뒤집어 쓴다
    body.replaceChildren(...rows.reverse().map(d => {
      const tr = document.createElement('tr');
      tr.innerHTML = row(d);
      return tr;
    }));
  }
  draw();

  // 수리 완료 표시는 `event`(WELD_DONE)로 바뀐다 — 그때도 다시 그린다
  const off = [bus.on('defect', draw), bus.on('event', draw)];
  return () => off.forEach(f => f());
}
