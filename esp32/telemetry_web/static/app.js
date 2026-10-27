const $ = id => document.getElementById(id);
const DEG = 180 / Math.PI;
const colors = ['#58e0b2', '#ffbd72', '#72bfff', '#cba2ff'];
let state = null, lastMessage = 0, packetTimes = [], freshTimes = [], lastSequence = -1;
let samples = [], group = 'control', fieldNodes = new Map(), fieldKey = '';
const fmt = (v, digits = 3) => Number.isFinite(v) ? v.toFixed(digits) : '—';
const put = (id, value) => { const e = $(id); if (e.textContent !== value) e.textContent = value; };
const faults = ['无故障', '传感器初始化', '传感器超时', '母线电压', '母线过期', 'ADC 读取', '过流', 'FOC 采样超时', 'PWM 更新', '电机校准', '任务超时'];
const stream = new EventSource('/events');
stream.onmessage = event => {
  state = JSON.parse(event.data);
  lastMessage = performance.now();
  packetTimes.push(lastMessage);
  const c = state.control;
  if (c && c.sequence !== lastSequence) {
    lastSequence = c.sequence;
    freshTimes.push(lastMessage);
    const tor = state.torque, yaw = state.yaw;
    samples.push({t: c.sample_us / 1e6, pitch: c.pitch * DEG, targetPitch: c.target_pitch * DEG,
      speed: c.speed, targetSpeed: c.target_speed, yaw: c.yaw * DEG,
      targetYaw: (yaw?.target || 0) * DEG,
      left: tor?.count_a ? -tor.mean_net_iq_a : null,
      right: tor?.count_b ? tor.mean_net_iq_b : null,
      commandLeft: c.command_a, commandRight: c.command_b});
    while (samples.length > 1000 || (samples.length && samples[0].t < samples.at(-1).t - 10)) samples.shift();
  }
};
stream.onerror = () => { put('status', '连接中断 · 正在重连'); $('dot').className = 'dot bad'; };

function pose(c) {
  // Match balance::observe: convert IMU right/forward/up Euler to URDF forward/left/up.
  const sy = Math.sin(c.roll), cy = Math.cos(c.roll);
  const sx = Math.max(-1, Math.min(1, -Math.sin(c.pitch) / cy));
  const cx = Math.sqrt(Math.max(0, 1 - sx * sx));
  return {roll: Math.atan2(sy, cy * cx), pitch: c.pitch, yaw: c.yaw};
}

function metrics(now) {
  while (packetTimes.length && packetTimes[0] < now - 1000) packetTimes.shift();
  while (freshTimes.length && freshTimes[0] < now - 1000) freshTimes.shift();
  const c = state?.control, y = state?.yaw;
  const age = state?.link.age_ms == null ? Infinity : state.link.age_ms + now - lastMessage;
  const stale = age > 250;
  document.body.classList.toggle('stale', stale);
  $('dot').className = 'dot ' + (stale || c?.fault ? 'bad' : 'good');
  put('status', stale ? '数据中断' : c?.fault ? '故障停机' : (c?.flags & 6) === 6 ? '持续平衡中' : '已连接 · 停机');
  put('receive-hz', `${packetTimes.length} Hz`);
  put('fresh-hz', `${freshTimes.length} Hz`);
  put('age', Number.isFinite(age) ? `${fmt(age, 0)} ms` : '—');
  if (!c) return;
  const p = pose(c);
  put('pitch', `${fmt(c.pitch * DEG, 2)} °`); put('target-pitch', `${fmt(c.target_pitch * DEG, 2)}°`);
  put('speed', `${fmt(c.speed)} m/s`); put('target-speed', `${fmt(c.target_speed)} m/s`);
  put('yaw', `${fmt(c.yaw * DEG, 2)} °`); put('yaw-rate', `${fmt(c.yaw_rate * DEG, 2)}°/s`);
  put('distance', `${fmt(c.distance * 100, 2)} cm`); put('bus', `${fmt(c.bus, 2)} V`);
  put('current-limit', `${fmt(c.current_limit_A, 1)} A`);
  put('roll', `${fmt(p.roll * DEG, 2)}°`); put('pose-pitch', `${fmt(p.pitch * DEG, 2)}°`); put('pose-yaw', `${fmt(p.yaw * DEG, 2)}°`);
  put('mode', ['停机', '影子', '平衡', '探测'][c.mode] || String(c.mode));
  put('fault-badge', c.fault ? `FAULT ${c.fault} · ${faults[c.fault] || '未知'}` : 'FAULT 0 · 正常');
  $('fault-badge').className = `badge ${c.fault ? 'bad' : 'good'}`;
  put('flags', [1, 2, 4].map(bit => c.flags & bit ? '✓' : '—').join(' / '));
  put('saturation', c.saturated ? '是' : '否');
  put('dt', `${fmt(c.dt_us / 1000, 3)} ms`); put('calculate', `${c.calculate_us} μs`);
  put('imu-age', `${c.imu_age_us} μs`); put('enc-age', `${c.encoder_age_us} μs`);
  put('iq-age', `${c.current_age_a_us} / ${c.current_age_b_us} μs`);
  put('learning', !y ? '等待数据' : y.flags & 4 ? '正在学习零偏' : y.flags & 2 ? '静止候选' : '冻结');
  put('bias', y ? `${fmt(y.bias * DEG, 5)} °/s` : '—');
  put('quiet', y ? `${fmt(y.quiet_time, 2)} s` : '—');
  put('rates', y ? `${fmt(y.gyro_rate * DEG, 3)} / ${fmt(y.encoder_rate * DEG, 3)} °/s` : '—');
  put('sequence', `源 ${state.link.source_hz} Hz · 帧 ${c.sequence} · 设备运行 ${fmt(c.sample_us / 1e6, 1)} s`);
}

function fields() {
  const data = state?.[group];
  if (!data) return;
  const query = $('filter').value.trim().toLowerCase();
  const keys = Object.keys(data).filter(k => !['magic', 'crc'].includes(k) && k.toLowerCase().includes(query));
  const key = group + ':' + keys.join(',');
  if (key !== fieldKey) {
    fieldKey = key; fieldNodes.clear(); $('fields').replaceChildren();
    for (const k of keys) {
      const row = document.createElement('div'); row.className = 'field';
      const name = document.createElement('span'); name.textContent = k; name.title = k;
      const value = document.createElement('b'); row.append(name, value); $('fields').append(row); fieldNodes.set(k, value);
    }
  }
  for (const [k, node] of fieldNodes) {
    const v = data[k];
    node.textContent = Number.isInteger(v) ? String(v) : Math.abs(v) < .00001 && v !== 0 ? v.toExponential(3) : fmt(v, 6);
  }
}
document.querySelectorAll('[data-group]').forEach(button => button.onclick = () => {
  group = button.dataset.group;
  document.querySelectorAll('[data-group]').forEach(b => b.classList.toggle('selected', b === button)); fields();
});
$('filter').oninput = fields;

const plots = [
  {id: 'pitch-chart', keys: ['pitch', 'targetPitch'], span: 2},
  {id: 'speed-chart', keys: ['speed', 'targetSpeed'], span: .04},
  {id: 'current-chart', keys: ['left', 'right', 'commandLeft', 'commandRight'], colors: [colors[0], colors[2], colors[1], colors[1]], span: .4},
  {id: 'yaw-chart', keys: ['yaw', 'targetYaw'], span: 2}
];
function chart(plot) {
  const canvas = $(plot.id), box = canvas.getBoundingClientRect(), dpr = Math.min(devicePixelRatio, 2);
  const w = Math.round(box.width), h = Math.round(box.height);
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, w, h);
  const left = 49, top = 12, right = w - 12, bottom = h - 25;
  let lo = 0, hi = 0;
  for (const s of samples) for (const k of plot.keys) if (s[k] != null) { lo = Math.min(lo, s[k]); hi = Math.max(hi, s[k]); }
  const range = Math.max(plot.span, hi - lo), mid = (lo + hi) / 2;
  lo = mid - range * .6; hi = mid + range * .6;
  ctx.font = '10px ui-monospace,monospace'; ctx.lineWidth = 1;
  for (let i = 0; i < 5; i++) {
    const y = top + (bottom - top) * i / 4, v = hi - (hi - lo) * i / 4;
    ctx.strokeStyle = '#263740'; ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(right, y); ctx.stroke();
    ctx.fillStyle = '#8399a8'; ctx.textAlign = 'right'; ctx.fillText(fmt(v, range < .2 ? 3 : 2), left - 8, y + 3);
  }
  ctx.textAlign = 'center';
  for (let i = 0; i <= 5; i++) ctx.fillText(i === 5 ? '现在' : `${-10 + i * 2}s`, left + (right - left) * i / 5, h - 5);
  if (!samples.length) return;
  const end = samples.at(-1).t;
  plot.keys.forEach((key, index) => {
    const dashed = plot.colors ? index >= 2 : index === 1;
    ctx.strokeStyle = (plot.colors || colors)[index]; ctx.lineWidth = 1.5; ctx.setLineDash(dashed ? [4, 4] : []); ctx.beginPath();
    let previous = null;
    for (const s of samples) {
      if (s[key] == null) { previous = null; continue; }
      const x = left + (s.t - end + 10) / 10 * (right - left);
      const y = bottom - (s[key] - lo) / (hi - lo) * (bottom - top);
      if (previous === null || s.t - previous > .1) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      previous = s.t;
    }
    ctx.stroke(); ctx.setLineDash([]);
  });
}

let renderModel = () => {};
async function setupModel() {
  const THREE = await import('three');
  const {STLLoader} = await import('/vendor/STLLoader.js');
  const {OrbitControls} = await import('/vendor/OrbitControls.js');
  const container = $('model'), renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.outputColorSpace = THREE.SRGBColorSpace;
  container.append(renderer.domElement);
  const scene = new THREE.Scene(), camera = new THREE.PerspectiveCamera(36, 1, .005, 10);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true; controls.minDistance = .13; controls.maxDistance = 1.2;
  function reset() { camera.position.set(.27, .19, .30); controls.target.set(0, .065, 0); controls.update(); }
  $('camera-reset').onclick = reset; reset();
  scene.add(new THREE.HemisphereLight(0xcce9ff, 0x263c43, 3));
  const light = new THREE.DirectionalLight(0xffffff, 3); light.position.set(.3, .6, .2); scene.add(light);
  const grid = new THREE.GridHelper(.65, 26, 0x416270, 0x233e4a); scene.add(grid);
  const zup = new THREE.Group(); zup.rotation.x = -Math.PI / 2; scene.add(zup);
  const car = new THREE.Group(); car.position.z = .0325; zup.add(car);
  const loader = new STLLoader();
  const parts = await Promise.all(['body', 'left_wheel', 'right_wheel'].map(name => loader.loadAsync(`/meshes/${name}.stl?v=urdf1`)));
  const body = new THREE.Mesh(parts[0], new THREE.MeshStandardMaterial({color: 0x359bad, roughness: .45, metalness: .3})); car.add(body);
  const wheels = parts.slice(1).map((geometry, index) => {
    const wheel = new THREE.Group(); wheel.position.y = index === 0 ? .0662682 : -.0662682;
    const tire = new THREE.Mesh(geometry, new THREE.MeshStandardMaterial({color: 0x27333c, roughness: .85})); wheel.add(tire);
    const stripe = new THREE.Mesh(new THREE.BoxGeometry(.006, .001, .025), new THREE.MeshStandardMaterial({color: 0x8badb7}));
    stripe.position.set(0, index === 0 ? .012 : -.012, .015); wheel.add(stripe); car.add(wheel); return wheel;
  });
  const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(.045, 0, 0), .055, 0x58e0b2, .012, .008); zup.add(arrow);
  const observer = new ResizeObserver(() => {
    renderer.setSize(container.clientWidth, container.clientHeight); camera.aspect = container.clientWidth / container.clientHeight; camera.updateProjectionMatrix();
  }); observer.observe(container);
  $('model-loading').remove();
  renderModel = () => {
    const c = state?.control;
    if (c) {
      const p = pose(c); car.quaternion.setFromEuler(new THREE.Euler(p.roll, p.pitch, p.yaw, 'ZYX'));
      wheels[0].rotation.y = -c.angle_a; wheels[1].rotation.y = c.angle_b;
    }
    controls.update(); renderer.render(scene, camera);
  };
}
setupModel().catch(error => { $('model-loading').textContent = `3D 模型加载失败：${error.message}`; console.error(error); });
let lastFields = 0;
function frame(now) {
  metrics(now); plots.forEach(chart); renderModel();
  if (now - lastFields > 100) { fields(); lastFields = now; }
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
