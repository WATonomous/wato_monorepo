// driveView.js
// Self-contained 3D drive view (chase cam behind EVE).
// - Owns: three.js scene/renderer/camera, chase-cam logic, drag-to-look,
//   ego car (STL), tracked-object boxes + predictions + labels, lanelet ribbons,
//   route ribbon, ground + grid, and its own requestAnimationFrame render loop.
// - Does NOT own: status pills (speed/connection/autonomy/behaviour) — shell's job.
// - Is WebSocket-agnostic: the shell calls update(msg) with each parsed message.
//
// Public interface (mirrors MapView):
//   DriveView.init({ canvas, three, STLLoader, modelUrl })  -> set up + start loop
//   DriveView.update(msg)                                    -> feed one WS message
//   DriveView.resize()                                       -> re-fit after size change
//
// Note: three.js and STLLoader are passed IN (dependency injection) so this module
// doesn't hard-code import paths — the test page and the shell both provide them.

export const DriveView = (() => {
  let THREE, canvas, renderer, scene, camera;
  let car, ground, grid;
  let objectsGroup, laneletGroup, routeGroup;
  const objectPool = new Map();

  // chase-cam working values
  let UP, camOffset, camTarget, camFocus;
  let camYaw = 0;

  // drag-to-look state
  let dragging = false;
  let lastX = 0, lastY = 0;
  let lookYaw = 0, lookPitch = 0;

  const ROAD_H = 0.03;
  const CLASS_COLORS = { person: 0x00e676, car: 0x2196f3 };
  const CLASS_COLOR_DEFAULT = 0x888888;

  let rafId = null;

  // ── geometry helpers ──
  function makeMatcap(base = '#c4c4c4') {
    const s = 256;
    const cv = document.createElement('canvas');
    cv.width = cv.height = s;
    const ctx = cv.getContext('2d');
    const g = ctx.createRadialGradient(s * 0.38, s * 0.32, s * 0.04, s * 0.5, s * 0.5, s * 0.5);
    g.addColorStop(0.0, '#e2e2e2');
    g.addColorStop(0.45, base);
    g.addColorStop(1.0, '#242424');
    ctx.fillStyle = g;
    ctx.beginPath();
    ctx.arc(s / 2, s / 2, s / 2, 0, Math.PI * 2);
    ctx.fill();
    return new THREE.CanvasTexture(cv);
  }

  function makeLaneLine(pts, color, dashed = false) {
    const geo = new THREE.BufferGeometry().setFromPoints(
      pts.map((p) => new THREE.Vector3(p.x, ROAD_H, -p.y))
    );
    const mat = dashed
      ? new THREE.LineDashedMaterial({ color, dashSize: 1, gapSize: 1 })
      : new THREE.LineBasicMaterial({ color });
    const line = new THREE.Line(geo, mat);
    if (dashed) line.computeLineDistances();
    return line;
  }

  function makeRoadRibbon(centerline, halfWidth) {
    const positions = [];
    const n = centerline.length;
    for (let i = 0; i < n; i++) {
      const a = centerline[Math.max(0, i - 1)];
      const b = centerline[Math.min(n - 1, i + 1)];
      let tx = b.x - a.x, ty = b.y - a.y;
      const len = Math.hypot(tx, ty) || 1;
      tx /= len; ty /= len;
      const c = centerline[i];
      const lx = c.x - ty * halfWidth, ly = c.y + tx * halfWidth;
      const rx = c.x + ty * halfWidth, ry = c.y - tx * halfWidth;
      positions.push(lx, 0, -ly, rx, 0, -ry);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    const idx = [];
    for (let i = 0; i < n - 1; i++) {
      const o = i * 2;
      idx.push(o, o + 1, o + 2, o + 2, o + 1, o + 3);
    }
    geo.setIndex(idx);
    geo.computeVertexNormals();
    const mat = new THREE.MeshStandardMaterial({
      color: 0x2a2a30, side: THREE.DoubleSide, roughness: 1.0, metalness: 0.0,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.y = 0.02;
    return mesh;
  }

  function laneHalfWidth(L) {
    if (L.left.length && L.right.length) {
      const dx = L.left[0].x - L.right[0].x;
      const dy = L.left[0].y - L.right[0].y;
      const w = Math.hypot(dx, dy) / 2;
      if (w > 0.5 && w < 6) return w * 1.025;
    }
    return 2;
  }

  function makeThickLine(pts, width, color, height = 0.05) {
    const positions = [];
    const n = pts.length;
    const half = width / 2;
    for (let i = 0; i < n; i++) {
      const a = pts[Math.max(0, i - 1)];
      const b = pts[Math.min(n - 1, i + 1)];
      let tx = b.x - a.x, ty = b.y - a.y;
      const len = Math.hypot(tx, ty) || 1;
      tx /= len; ty /= len;
      const c = pts[i];
      positions.push(
        c.x - ty * half, height, -(c.y + tx * half),
        c.x + ty * half, height, -(c.y - tx * half)
      );
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    const idx = [];
    for (let i = 0; i < n - 1; i++) {
      const o = i * 2;
      idx.push(o, o + 1, o + 2, o + 2, o + 1, o + 3);
    }
    geo.setIndex(idx);
    geo.computeVertexNormals();
    const mat = new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide });
    return new THREE.Mesh(geo, mat);
  }

  function makePredLine(color) {
    const geo = new THREE.BufferGeometry();
    const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.8 });
    return new THREE.Line(geo, mat);
  }

  function makeLabel(text, color = '#ffffff') {
    const cv = document.createElement('canvas');
    const ctx = cv.getContext('2d');
    const font = 48;
    ctx.font = `bold ${font}px sans-serif`;
    const w = ctx.measureText(text).width;
    cv.width = w + 20;
    cv.height = font + 20;
    ctx.font = `bold ${font}px sans-serif`;
    ctx.fillStyle = 'rgba(0,0,0,0.5)';
    ctx.fillRect(0, 0, cv.width, cv.height);
    ctx.fillStyle = color;
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 10, cv.height / 2);
    const tex = new THREE.CanvasTexture(cv);
    const mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false });
    const sprite = new THREE.Sprite(mat);
    const h = 0.5;
    sprite.scale.set(h * cv.width / cv.height, h, 1);
    return sprite;
  }

  // ── render loop ──
  function animate() {
    rafId = requestAnimationFrame(animate);

    let d = car.rotation.y - camYaw;
    d = Math.atan2(Math.sin(d), Math.cos(d));
    camYaw += d * 0.1;
    camFocus.lerp(car.position, 0.15);

    ground.position.x = car.position.x;
    ground.position.z = car.position.z;
    const cell = 200 / 40;
    grid.position.x = Math.round(car.position.x / cell) * cell;
    grid.position.z = Math.round(car.position.z / cell) * cell;

    if (!dragging) { lookYaw *= 0.85; lookPitch *= 0.85; }

    camTarget.copy(camOffset)
      .applyAxisAngle(new THREE.Vector3(1, 0, 0), lookPitch)
      .applyAxisAngle(UP, camYaw + lookYaw)
      .add(camFocus);
    camera.position.copy(camTarget);
    camera.lookAt(camFocus);
    renderer.render(scene, camera);

    for (const { mesh, target, label } of objectPool.values()) {
      mesh.position.x += (target.x - mesh.position.x) * 0.2;
      mesh.position.z += (target.z - mesh.position.z) * 0.2;
      let dy = target.yaw - mesh.rotation.y;
      dy = Math.atan2(Math.sin(dy), Math.cos(dy));
      mesh.rotation.y += dy * 0.2;
      if (label) label.position.set(mesh.position.x, mesh.position.y + 1.5, mesh.position.z);
    }
  }

  // ── drag handlers ──
  function onMouseDown(e) { dragging = true; lastX = e.clientX; lastY = e.clientY; }
  function onMouseUp() { dragging = false; }
  function onMouseMove(e) {
    if (!dragging) return;
    lookYaw   -= (e.clientX - lastX) * 0.005;
    lookPitch += (e.clientY - lastY) * 0.005;
    lookPitch = Math.max(-0.4, Math.min(1.2, lookPitch));
    lastX = e.clientX; lastY = e.clientY;
  }

  // ── public interface ──
  function init({ canvas: c, three, STLLoader, modelUrl }) {
    THREE = three;
    canvas = c;

    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0f);

    camera = new THREE.PerspectiveCamera(60, 1, 0.1, 1000);
    camera.position.set(0, 6, -12);
    camera.lookAt(0, 0, 10);

    UP        = new THREE.Vector3(0, 1, 0);
    camOffset = new THREE.Vector3(0, 6, -12);
    camTarget = new THREE.Vector3();
    camFocus  = new THREE.Vector3();

    car = new THREE.Group();
    scene.add(car);

    const loader = new STLLoader();
    loader.load(modelUrl, (geometry) => {
      const mesh = new THREE.Mesh(
        geometry,
        new THREE.MeshMatcapMaterial({ matcap: makeMatcap() })
      );
      mesh.scale.setScalar(0.01);
      mesh.rotation.x = -Math.PI / 2;
      car.add(mesh);
    }, undefined, (err) => console.error('STL load failed:', err));

    const ambient = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambient);
    const dir = new THREE.DirectionalLight(0xffffff, 0.8);
    dir.position.set(5, 10, 5);
    scene.add(dir);

    ground = new THREE.Mesh(
      new THREE.PlaneGeometry(200, 200),
      new THREE.MeshStandardMaterial({ color: 0x1a1a22 })
    );
    ground.rotation.x = -Math.PI / 2;
    scene.add(ground);

    grid = new THREE.GridHelper(200, 40, 0x333344, 0x222230);
    scene.add(grid);

    objectsGroup = new THREE.Group(); scene.add(objectsGroup);
    laneletGroup = new THREE.Group(); scene.add(laneletGroup);
    routeGroup   = new THREE.Group(); scene.add(routeGroup);

    canvas.addEventListener('mousedown', onMouseDown);
    window.addEventListener('mouseup', onMouseUp);
    window.addEventListener('mousemove', onMouseMove);

    resize();     // size to container before first frame
    animate();
  }

  function update(msg) {
    if (msg.type === 'ego') {
      car.position.x =  msg.x;
      car.position.z = -msg.y;
      car.rotation.y =  msg.yaw + Math.PI / 2;
    } else if (msg.type === 'objects') {
      for (const entry of objectPool.values()) entry.seen = false;
      for (const o of msg.objects) {
        const tx = o.x, tz = -o.y, tyaw = o.yaw;
        let entry = objectPool.get(o.id);
        if (!entry) {
          const geo = new THREE.BoxGeometry(o.sx, o.sz, o.sy);
          const color = CLASS_COLORS[o.cls] ?? CLASS_COLOR_DEFAULT;
          const box = new THREE.Mesh(geo,
            new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.55 }));
          box.position.set(tx, o.z, tz);
          box.rotation.y = tyaw;
          objectsGroup.add(box);
          const predLine = makePredLine(color);
          objectsGroup.add(predLine);
          const label = makeLabel(String(o.id));
          objectsGroup.add(label);
          entry = { mesh: box, line: predLine, label, target: { x: tx, z: tz, yaw: tyaw }, seen: true };
          objectPool.set(o.id, entry);
        } else {
          entry.target.x = tx; entry.target.z = tz; entry.target.yaw = tyaw;
          entry.mesh.position.y = o.z;
          entry.seen = true;
        }
        const pts = (o.pred ?? []).map((p) => new THREE.Vector3(p.x, o.z, -p.y));
        entry.line.geometry.setFromPoints(pts);
      }
      for (const [id, entry] of objectPool) {
        if (!entry.seen) {
          objectsGroup.remove(entry.mesh);
          entry.mesh.geometry.dispose(); entry.mesh.material.dispose();
          objectsGroup.remove(entry.line);
          entry.line.geometry.dispose(); entry.line.material.dispose();
          objectsGroup.remove(entry.label);
          entry.label.material.map.dispose(); entry.label.material.dispose();
          objectPool.delete(id);
        }
      }
    } else if (msg.type === 'lanelet') {
      for (const c of laneletGroup.children) { c.geometry.dispose(); c.material.dispose(); }
      laneletGroup.clear();
      for (const L of msg.lanelets) {
        if (L.centerline.length > 1) laneletGroup.add(makeRoadRibbon(L.centerline, laneHalfWidth(L)));
      }
      for (const L of msg.lanelets) {
        if (L.left.length  > 1) laneletGroup.add(makeThickLine(L.left,  0.05, 0xffffff));
        if (L.right.length > 1) laneletGroup.add(makeThickLine(L.right, 0.05, 0xffffff));
      }
      for (const L of msg.lanelets) {
        if (L.centerline.length > 1) laneletGroup.add(makeLaneLine(L.centerline, 0xaaaaaa, true));
      }
    } else if (msg.type === 'route') {
      for (const c of routeGroup.children) { c.geometry.dispose(); c.material.dispose(); }
      routeGroup.clear();
      // NOTE: drive's route handler expects lanelets[]; the current server sends
      // route as points[]. Left as-is to match the original drive.html behavior.
      for (const L of (msg.lanelets ?? [])) {
        if (L.centerline.length > 1) {
          const ribbon = makeRoadRibbon(L.centerline, laneHalfWidth(L) * 0.6);
          ribbon.material.color.set(0x2196f3);
          ribbon.material.transparent = true;
          ribbon.material.opacity = 0.55;
          ribbon.position.y = 0.06;
          routeGroup.add(ribbon);
        }
      }
    }
  }

  function resize() {
    if (!canvas || !renderer) return;
    const rect = canvas.getBoundingClientRect();
    const w = Math.max(1, Math.round(rect.width));
    const h = Math.max(1, Math.round(rect.height));
    renderer.setSize(w, h, false);       // false: don't override canvas CSS size
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }

  return { init, update, resize };
})();