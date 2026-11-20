// 제본 코일 3D 모형 — 사진 대신 쓰는 시뮬레이션 그림. THREE(r128) 전역 필요.
// 단위 mm. X = 코일 길이, Y = 위, Z = 깊이.
window.CoilSim = (() => {
  // 치수 = 2026-11-14 자와 함께 찍은 사진 42장(docs/spring-media/코일모양-2026-11-14/1114 - 코일모양 상세 촬영.zip)에서 읽음: 열린 코일 겉 폭 15.5 × 높이 9.5mm(±5%),
  // 쌍선 간격 2.3mm. 철사 굵기 0.7mm는 사용자 캘리퍼 실측(흰색·검은색 같음, 코팅 포함 — 사진으로는 0.9로 읽혔으나 캘리퍼를 따른다). 피치 8.47은 규격(3:1)이고 사진 축척을 맞추는 데 썼다.
  const P = 8.47, N = 34, L = P * (N - 1), W = 15.5, H = 9.5, R = 0.35;
  const TRAY = { slots: 5, gap: 24, r: 10.5, len: L + 30 };   // 칸 간격 24.0±0.7(11/13 사진 보정). 칸 모양은 단순화 — 실제는 돌기(리브) 사이 칸이다(tools/spring_tray_sim.py)
  let _loop, _curve, _mats;
  const mats = () => _mats || (_mats = {
    wire: new THREE.MeshStandardMaterial({ color: 0xf6f8f9, roughness: 0.45, metalness: 0.05 }),
    wireB: new THREE.MeshStandardMaterial({ color: 0x8fd0e6, roughness: 0.45, metalness: 0.05 }),
    table: new THREE.MeshStandardMaterial({ color: 0x26323a, roughness: 0.95 }),
    dark: new THREE.MeshStandardMaterial({ color: 0x1b2328, roughness: 0.6 }),
    steel: new THREE.MeshStandardMaterial({ color: 0x8d9aa1, roughness: 0.4, metalness: 0.5 }),
    accent: new THREE.MeshStandardMaterial({ color: 0x1d6b86, roughness: 0.6 }),
    foam: new THREE.MeshStandardMaterial({ color: 0xfbf6e6, roughness: 0.9 }),
    print: new THREE.MeshStandardMaterial({ color: 0xe0863a, roughness: 0.65 }),
    tray: new THREE.MeshStandardMaterial({ color: 0xd8f1f8, roughness: 0.15, transparent: true, opacity: 0.42, side: THREE.DoubleSide }),
  });
  // ── 고리 모양: 끝에서 본 사진(19번)과 위에서 본 사진(16번)에서 옮김.
  // 등뼈 막대가 없다. 철사 한 가닥이 빗처럼 이어진다: 쌍선 갈래(큰 고리, −Z 입술에서 좁게 U턴) ↔ 이웃 갈래를 잇는 이음(작은 고리, +Z 입술을 따라 축 방향으로 달린다).
  // 단면은 좌우가 거의 같은 넓은 U: 바닥 가운데가 0.7mm 솟고, 옆벽이 불룩하고, 입술이 2mm쯤 안으로 말린다(입 폭 10.4 < 몸통 폭 14.6, 중심선 기준).
  const G = 2.3, HW = W / 2 - R, HT = H - 2 * R;   // 쌍선 간격(중심-중심) · 중심선 반폭 · 중심선 높이
  // 단면 반쪽 [z ÷ HW, y ÷ HT]: 입술 끝 → 옆벽 → 바닥 가운데
  const HALF = [[0.71, 1], [0.81, 0.93], [0.92, 0.82], [0.985, 0.66], [1, 0.5], [0.98, 0.35], [0.9, 0.2], [0.75, 0.055], [0.55, 0], [0.27, 0.045], [0, 0.085]];
  function loopGeo() {
    if (_loop) return _loop;
    const pts = [], v = (x, h, s) => new THREE.Vector3(x, h[1] * HT, s * h[0] * HW), n = HALF.length - 1, lip = HALF[0];
    const side = x => { for (let i = 0; i <= n; i++) pts.push(v(x, HALF[i], 1)); for (let i = n - 1; i >= 0; i--) pts.push(v(x, HALF[i], -1)); };   // +Z 입술 → 바닥 → −Z 입술
    pts.push(v(-P / 2, lip, 1), v(-G / 2 - 1.4, lip, 1));                       // 이음(작은 고리) 절반
    side(-G / 2);
    pts.push(new THREE.Vector3(0, HT + 0.25, -lip[0] * HW + 0.9));              // 큰 고리 끝 U턴(입술이 말린 방향으로 조금 더 나간다)
    const back = pts.length; side(G / 2); pts.splice(back, pts.length - back, ...pts.slice(back).reverse());
    pts.push(v(G / 2 + 1.4, lip, 1), v(P / 2, lip, 1));
    _curve = new THREE.CatmullRomCurve3(pts, false, "centripetal");
    return (_loop = new THREE.TubeGeometry(_curve, 220, R, 6, false));
  }
  // 코일 하나: 피치 34개. 열린 쪽이 위, 고리 끝 입술(높다)이 −Z, 이음 입술(낮다)이 +Z.
  function coil(mat) {
    const g = new THREE.Group(), im = new THREE.InstancedMesh(loopGeo(), mat || mats().wire, N), m = new THREE.Matrix4();
    for (let i = 0; i < N; i++) { m.makeTranslation(-L / 2 + i * P, R, 0); im.setMatrixAt(i, m); }
    im.castShadow = true; im.receiveShadow = true; g.add(im);
    return g;
  }
  // ── 포갬 자세는 손으로 맞추지 않는다: tools/coil_fit_check.html(철사 중심선 사이 거리 ≥ 철사 굵기)로 찾은 값이다.
  // 2개 세트 = 두 코일을 **같은 방향으로** 포갠 것(2026-11-14 사용자 확인). 입(10.4)이 몸통(14.6)보다 좁아 위에서는 못 내려앉는다 —
  // 축 방향으로 밀어 넣어 만든다. 겹친 자리에서 안 뚫는 가장 가까운 자세 = 반 피치 + 철사 한 가닥(0.7mm)만큼 비켜 앉음.
  const SETP = { dx: P / 2, dy: 0.5, dz: 0.5 };
  // 4개 묶음 = 세트 둘을 위아래로 포갠 닫힌 관. 맞는 모양 "작은 고리끼리": 엎은 세트를 옆으로 BZ만큼 밀어야 내려간다 —
  // 작은 고리(이음) 쪽은 아래 세트 벽 **바깥**으로 걸치고, 큰 고리 끝은 아래 세트 입 **안**으로 들어간다. 그렇게 내려앉은 높이가 BH.
  // 검산: 계산 20.0 × 10.7mm ↔ 사진(0~2번·33~38번) 17.3~18.5 × 11.8~12.9mm. 폭은 8%쯤 크고 높이는 12%쯤 낮다 — 강체 가정과 단면 읽기 오차.
  // (철사를 0.9로 두면 높이 12.4로 맞는다 — 굵기가 0.7~0.9 사이일 수 있다.)
  // ⚠ 버니어 실측(11/14)은 묶음 폭 16.0(검정)·18.0(흰색) × 높이 12.7mm다. 이 강체 계산(20 × 10.7)은 폭이 2~4mm 넓고 높이가 2mm 낮다 —
  //   치수가 필요하면 버니어 값을 쓴다. 이 모형은 '철사가 안 뚫는 포갬 모양'을 보여 주는 용도다.
  const SW = W + SETP.dz, SH = H + SETP.dy, BZ = 4, BH = 10.7, BW = SW + BZ;
  function set(mat) {
    const g = new THREE.Group(), a = coil(mat), b = coil(mat);
    b.position.set(SETP.dx, SETP.dy, SETP.dz); g.add(a, b); return g;
  }
  // 휜 코일: pts = 중심선 [[x,y],...](mm, 실측 — tools/spring_sag_measure.py). 고리를 호 길이 따라 P 간격으로 놓고
  // 접선만큼 기울인다. hidden = [x0,x1] 구간(사진에서 손에 가려 이은 곳)은 파랗게 칠한다. off = 호 방향 밀기, mate = true면 세트의 둘째 코일(뒤집혀 덮는 쪽).
  function bent(pts, hidden, off, mate) {
    const cum = [0]; for (let i = 1; i < pts.length; i++) cum.push(cum[i - 1] + Math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]));
    const total = cum[cum.length - 1], n = Math.min(N, Math.floor((total - 1.7) / P) + 1), s0 = (total - (n - 1) * P) / 2;
    const at = s => { let i = 1; while (i < cum.length - 1 && cum[i] < s) i++; const t = Math.min(1, Math.max(0, (s - cum[i - 1]) / (cum[i] - cum[i - 1]))), a = pts[i - 1], b = pts[i];
      return { x: a[0] + (b[0] - a[0]) * t, y: a[1] + (b[1] - a[1]) * t, a: Math.atan2(b[1] - a[1], b[0] - a[0]) }; };
    const g = new THREE.Group(), im = new THREE.InstancedMesh(loopGeo(), mats().wire, n), m = new THREE.Matrix4(), q = new THREE.Quaternion(), z = new THREE.Vector3(0, 0, 1);
    const seen = new THREE.Color(0xffffff), gues = new THREE.Color(0x58b7e0), rz = new THREE.Matrix4();
    // 고리 원점(바닥)을 중심선에 맞추는 자리 옮김. 둘째 코일은 같은 방향으로 SETP만큼 얹는다.
    const local = new THREE.Matrix4().makeTranslation(0, -H / 2 + (mate ? SETP.dy : 0), mate ? SETP.dz : 0);
    for (let i = 0; i < n; i++) {
      const p = at(Math.min(total, Math.max(0, s0 + i * P + (off || 0))));
      m.makeTranslation(p.x, p.y, 0).multiply(rz.makeRotationZ(p.a)).multiply(local);
      im.setMatrixAt(i, m); im.setColorAt(i, hidden && p.x > hidden[0] && p.x < hidden[1] ? gues : seen);
    }
    im.castShadow = true; g.add(im);
    return g;
  }
  // 세트를 엎는다(열린 쪽이 아래). 작은 고리 쪽은 그대로 같은 쪽에 둔다 — 그대로 얹으면 "작은 고리끼리"가 된다.
  function flipped(obj) { const o = new THREE.Group(); obj.position.y = -SH / 2; o.add(obj); o.rotation.z = Math.PI; o.position.y = SH / 2; const w = new THREE.Group(); w.add(o); return w; }
  // 4개 묶음: 세트(열린 쪽 위) + 엎은 세트를 옆으로 BZ 밀어 덮은 관. 가운데 정렬.
  function bundle() { const g = new THREE.Group(), bot = set(), top = set(mats().wireB); bot.position.z = -(BZ + SETP.dz) / 2;
    top.rotation.z = Math.PI; top.position.set(P / 2, BH, bot.position.z + BZ); g.add(bot, top); return g; }
  function halfPipe(len, r) {
    const geo = new THREE.PlaneGeometry(len, Math.PI * r, 1, 18), p = geo.attributes.position;
    for (let i = 0; i < p.count; i++) { const a = p.getY(i) / r; p.setXYZ(i, p.getX(i), r - r * Math.cos(a), r * Math.sin(a)); }
    geo.computeVertexNormals(); return geo;
  }
  function tray() {
    const g = new THREE.Group(), M = mats(), depth = TRAY.slots * TRAY.gap + 22;
    const base = new THREE.Mesh(new THREE.BoxGeometry(TRAY.len + 40, 1.2, depth), M.tray); base.position.y = 0.6; base.receiveShadow = true; g.add(base);
    for (let i = 0; i < TRAY.slots; i++) { const m = new THREE.Mesh(halfPipe(TRAY.len, TRAY.r), M.tray); m.position.set(0, 1.3, slotZ(i)); g.add(m); }
    const rim = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(TRAY.len + 40, 6, depth)), new THREE.LineBasicMaterial({ color: 0x6f8f9b })); rim.position.y = 3; g.add(rim);
    return g;
  }
  const slotZ = i => (i - (TRAY.slots - 1) / 2) * TRAY.gap;
  function box(w, h, d, mat, x, y, z) { const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat); m.position.set(x, y, z); m.castShadow = true; m.receiveShadow = true; return m; }
  function stage() {
    const scene = new THREE.Scene(); scene.background = new THREE.Color(0x1c262c); scene.fog = new THREE.Fog(0x1c262c, 700, 1500);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x55636a, 0.5));
    const d = new THREE.DirectionalLight(0xffffff, 0.62); d.position.set(-140, 320, 180); d.castShadow = true;
    d.shadow.mapSize.set(2048, 2048); Object.assign(d.shadow.camera, { left: -280, right: 280, top: 220, bottom: -220, near: 50, far: 800 }); d.shadow.bias = -0.0015; scene.add(d);
    const t = new THREE.Mesh(new THREE.PlaneGeometry(1600, 1600), mats().table); t.rotation.x = -Math.PI / 2; t.receiveShadow = true; scene.add(t);
    return scene;
  }
  // 단계 1~11 = 방문자가 책상에서 재현한 사진 1~11과 같은 장면
  function step(n) {
    const s = stage(), M = mats(); let cam = [-185, 125, 215], tgt = [0, 0, 0];
    const put = (o, x, y, z) => { o.position.set(x || 0, y || 0, z || 0); s.add(o); return o; };
    if (n === 1) { put(coil(), 14, 0, -17); put(coil(), 14, 0, 17); s.add(box(70, 46, 110, M.dark, -L / 2 - 18, 23, 0)); s.add(box(8, 14, 24, M.steel, -L / 2 + 18, 30, -17)); s.add(box(8, 14, 24, M.steel, -L / 2 + 18, 30, 17)); }
    if (n === 2) { put(coil(), 0, 0, 0); const pv = new THREE.Group(), c = coil(); c.position.x = -L / 2 + P / 2; pv.add(c); pv.position.set(L / 2, 0.4, 0.5); pv.rotation.y = 0.075; s.add(pv);
      [-1, 1].forEach(k => { const r = new THREE.Mesh(new THREE.CylinderGeometry(6, 6, 16, 20), M.steel); r.position.set(L / 2 - 55, 8, k * 12.5); r.castShadow = true; s.add(r); });
      s.add(box(30, 4, 40, M.steel, L / 2 - 55, 18, 0)); cam = [215, 120, 200]; tgt = [30, 0, 0]; }
    if (n === 3) { put(set()); cam = [-150, 95, 170]; }
    if (n === 4) { put(set(), 0, 0, -20); put(set(), 0, 0, 20); }
    if (n === 5) { put(set(), 0, 0, -20); put(flipped(set(M.wireB)), 0, 0, 20); }
    if (n === 6) { put(set(), 0, 0, -6); const b = flipped(set(M.wireB)); b.rotation.x = 0.14; put(b, 0, 24, 4); const g = gripperBars(); put(g, 0, 24, 4); cam = [-190, 130, 230]; tgt = [0, 14, 0]; }
    if (n === 7) { put(bundle()); cam = [L / 2 + 62, BH / 2 + 4, 0.01]; tgt = [L / 2 - 40, BH / 2, 0]; }
    if (n === 8) { put(bundle()); cam = [L / 2 + 70, 42, 58]; tgt = [L / 2 - 50, 3, 0]; }
    if (n === 9) { put(bundle()); cam = [-150, 95, 170]; }
    if (n === 10) { put(tray(), 0, 0, -40); put(bundle(), 0, 0, 80); cam = [-170, 250, 330]; tgt = [0, 0, 20]; }
    if (n === 11) { put(tray()); put(bundle(), 0, 2.5, slotZ(1)); cam = [-150, 230, 290]; }
    return { scene: s, cam, tgt };
  }
  // 단순 그리퍼(긴 받침 두 쌍) — 6번 장면용
  function gripperBars() {
    const g = new THREE.Group(), M = mats(); g.add(box(L * 0.72, 5, 7, M.dark, 0, SH + 26, 0));
    [-1, 1].forEach(k => { g.add(box(5, 22, 5, M.dark, k * L * 0.3, SH + 14, 0)); [-1, 1].forEach(j => g.add(box(26, 12, 1.6, M.accent, k * L * 0.3, SH / 2 + 2, j * (SW / 2 + 1.2)))); });
    return g;
  }
  function camera(aspect, cam, tgt, fov) { const c = new THREE.PerspectiveCamera(fov || 30, aspect, 1, 3000); c.position.set(...cam); c.lookAt(...tgt); return c; }
  // 한 번에 여러 장면을 그림(JPEG data URL)으로 뽑는다.
  function renderSteps(steps, w, h, builder) {
    const r = new THREE.WebGLRenderer({ antialias: true }); r.setPixelRatio(1); r.setSize(w, h); r.outputEncoding = THREE.sRGBEncoding; r.shadowMap.enabled = true; r.shadowMap.type = THREE.PCFSoftShadowMap;
    const out = {};
    steps.forEach(n => { const st = (builder || step)(n); r.render(st.scene, camera(w / h, st.cam, st.tgt, st.fov)); out[n] = r.domElement.toDataURL("image/jpeg", 0.86); });
    r.dispose(); if (r.forceContextLoss) r.forceContextLoss(); return out;
  }
  // 돌려 볼 수 있는 뷰어
  function viewer(canvas, builder) {
    const r = new THREE.WebGLRenderer({ canvas, antialias: true }); r.outputEncoding = THREE.sRGBEncoding; r.shadowMap.enabled = true; r.shadowMap.type = THREE.PCFSoftShadowMap;
    let st, cam, ctl, tick = null;
    const fit = () => { const w = canvas.clientWidth, h = canvas.clientHeight; r.setPixelRatio(Math.min(2, window.devicePixelRatio || 1)); r.setSize(w, h, false); if (cam) { cam.aspect = w / h; cam.updateProjectionMatrix(); } draw(); };
    const draw = () => { if (st) { if (tick) tick(st); r.render(st.scene, cam); } };
    function show(n, arg) {
      st = (builder || step)(n, arg); cam = camera(canvas.clientWidth / canvas.clientHeight, st.cam, st.tgt, st.fov);
      if (ctl) ctl.dispose();
      if (THREE.OrbitControls) { ctl = new THREE.OrbitControls(cam, canvas); ctl.target.set(...st.tgt); ctl.maxPolarAngle = Math.PI / 2 - 0.02; ctl.update(); ctl.addEventListener("change", draw); }
      fit();
    }
    window.addEventListener("resize", fit);
    return { show, draw, get state() { return st; }, onTick(f) { tick = f; } };
  }
  const centerline = n => (loopGeo(), _curve.getSpacedPoints(n));   // 한 피치 철사 중심선(간섭 검사용)
  return { centerline, P, N, L, W, H, R, SETP, SW, SH, BW, BH, TRAY, mats, coil, bent, set, flipped, bundle, tray, slotZ, box, stage, step, camera, renderSteps, viewer };
})();
