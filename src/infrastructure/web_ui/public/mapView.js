// mapView.js
// Self-contained 2D map view 
// - Owns: the map canvas, client-side .osm parsing, ego arrow, route line,
//   destination pin, and the routing UI (infoBar: GO / CANCEL / EXIT).
// - Does NOT own: speed / connection / autonomy / behaviour status pills.
//   Those live on the app shell.
// - Is WebSocket-agnostic: it never opens or touches a socket.
//     * outbound: call onDestinationClick(cb); the shell's cb sends over its WS.
//     * inbound:  the shell calls update(msg) with each parsed WS message.
//
// Public interface:
//   MapView.init({ canvas, infoBar, mapUrl })  -> set up and start
//   MapView.update(msg)                         -> feed one WS message
//   MapView.resize()                            -> re-fit after canvas size change
//   MapView.onDestinationClick(cb)              -> cb({x, y}) when user hits GO

export const MapView = (() => {
  // ---- private state ----
  let canvas, ctx;
  let infoBarEl;                 // container with routeStatus / destInfo / btnRow / exitBtn
  let els = {};                  // cached infoBar child elements

  let transform = null;
  let clickMarker = null;
  let pendingDest = null;
  let ego = null;
  let route = [];
  let rawWays = null, bounds = null;

  let destClickCb = null;        // shell-provided: called on GO with {x, y}

  // ---- transform / projection ----
  function buildTransform() {
    const margin = 40;
    const scale = Math.min(
      (canvas.width  - 2 * margin) / (bounds.maxX - bounds.minX),
      (canvas.height - 2 * margin) / (bounds.maxY - bounds.minY)
    );
    const offsetX = (canvas.width  - (bounds.maxX - bounds.minX) * scale) / 2;
    const offsetY = (canvas.height - (bounds.maxY - bounds.minY) * scale) / 2;
    transform = { scale, offsetX, offsetY, minX: bounds.minX, minY: bounds.minY };
  }

  function mapToPixel(x, y) {
    const t = transform;
    return {
      px: t.offsetX + (x - t.minX) * t.scale,
      py: canvas.height - t.offsetY - (y - t.minY) * t.scale,
    };
  }
  function pixelToMap(px, py) {
    const t = transform;
    return {
      x: (px - t.offsetX) / t.scale + t.minX,
      y: (canvas.height - t.offsetY - py) / t.scale + t.minY,
    };
  }

  // ---- map loading ----
  async function loadMap(mapUrl) {
    const text = (await fetch(mapUrl).then((r) => r.text())).trim();
    const xml = new DOMParser().parseFromString(text, 'text/xml');

    const rawNodes = {};
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const node of xml.getElementsByTagName('node')) {
      const id = node.getAttribute('id');
      let x = null, y = null;
      for (const tag of node.getElementsByTagName('tag')) {
        if (tag.getAttribute('k') === 'local_x') x = parseFloat(tag.getAttribute('v'));
        if (tag.getAttribute('k') === 'local_y') y = parseFloat(tag.getAttribute('v'));
      }
      if (x === null || y === null) continue;
      rawNodes[id] = { x, y };
      minX = Math.min(minX, x); maxX = Math.max(maxX, x);
      minY = Math.min(minY, y); maxY = Math.max(maxY, y);
    }
    bounds = { minX, minY, maxX, maxY };

    rawWays = [];
    for (const way of xml.getElementsByTagName('way')) {
      const refs = [];
      for (const nd of way.getElementsByTagName('nd')) {
        const n = rawNodes[nd.getAttribute('ref')];
        if (n) refs.push(n);
      }
      if (refs.length) rawWays.push(refs);
    }

    buildTransform();
    draw();
  }

  // ---- drawing ----
  function draw() {
    if (!ctx) return;
    ctx.fillStyle = '#141418';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    if (rawWays) {
      for (const refs of rawWays) {
        const pts = refs.map((n) => mapToPixel(n.x, n.y));
        ctx.strokeStyle = 'rgba(80,80,85,0.35)';
        ctx.lineWidth = 5; strokePath(pts);
        ctx.strokeStyle = 'rgba(120,120,128,0.9)';
        ctx.lineWidth = 1.5; strokePath(pts);
      }
    }

    if (route.length > 1) {
      const pts = route.map((p) => mapToPixel(p.x, p.y))
                       .filter((p) => isFinite(p.px) && isFinite(p.py));
      if (pts.length > 1) {
        ctx.strokeStyle = 'rgba(77,163,255,0.3)';
        ctx.lineWidth = 10; ctx.lineCap = 'round'; ctx.lineJoin = 'round';
        strokePath(pts);
        ctx.strokeStyle = '#4da3ff';
        ctx.lineWidth = 4;
        strokePath(pts);
      }
    }

    if (clickMarker) {
      const { px, py } = mapToPixel(clickMarker.x, clickMarker.y);
      drawPin(px, py);
    }

    if (ego) {
      const { px, py } = mapToPixel(ego.x, ego.y);
      drawEgo(px, py, ego.yaw);
    }
  }

  function drawPin(px, py) {
    ctx.fillStyle = '#ff3b30';
    ctx.beginPath();
    ctx.arc(px, py - 18, 9, Math.PI, 0);
    ctx.lineTo(px, py);
    ctx.closePath();
    ctx.fill();
    ctx.beginPath();
    ctx.arc(px, py - 18, 9, 0, 2 * Math.PI);
    ctx.fill();
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.arc(px, py - 18, 3.5, 0, 2 * Math.PI);
    ctx.fill();
  }

  function drawEgo(px, py, yaw) {
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(-yaw + Math.PI / 2);
    ctx.beginPath();
    ctx.moveTo(0, -14);
    ctx.lineTo(10, 10);
    ctx.lineTo(0, 5);
    ctx.lineTo(-10, 10);
    ctx.closePath();
    ctx.fillStyle = '#ff3b30';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2.5;
    ctx.lineJoin = 'round';
    ctx.stroke();
    ctx.restore();
  }

  function strokePath(pts) {
    ctx.beginPath();
    pts.forEach((p, i) => (i ? ctx.lineTo(p.px, p.py) : ctx.moveTo(p.px, p.py)));
    ctx.stroke();
  }

  // ---- routing UI (infoBar) ----
  function resetToStart() {
    pendingDest = null;
    clickMarker = null;
    route = [];
    draw();
    els.destInfo.style.display = 'none';
    els.btnRow.style.display = 'none';
    els.exitBtn.style.display = 'none';
    els.routeStatus.textContent = 'Tap the map to set a destination';
    els.routeStatus.style.color = '#fff';
  }

  function onCanvasClick(event) {
    if (!transform) return;
    const rect = canvas.getBoundingClientRect();
    const px = event.clientX - rect.left;
    const py = event.clientY - rect.top;
    const m = pixelToMap(px, py);
    clickMarker = { x: m.x, y: m.y };
    pendingDest = { x: m.x, y: m.y };
    draw();

    let distText = '';
    if (ego) {
      const dist = Math.hypot(m.x - ego.x, m.y - ego.y);
      distText = ` \u00b7 ${dist.toFixed(1)} m away`;
    }
    els.destInfo.textContent = `(${m.x.toFixed(1)}, ${m.y.toFixed(1)}) m${distText}`;
    els.destInfo.style.display = 'block';
    els.btnRow.style.display = 'flex';
    els.exitBtn.style.display = 'none';

    els.routeStatus.textContent = '';
    els.routeStatus.style.color = '#fff';
  }

  function onGo() {
    if (!pendingDest) return;
    if (destClickCb) destClickCb({ x: pendingDest.x, y: pendingDest.y });
    els.btnRow.style.display = 'none';
    els.routeStatus.textContent = 'Routing\u2026';
    els.routeStatus.style.color = '#fff';
  }

  // ---- public interface ----
  function init({ canvas: c, infoBar, mapUrl }) {
    canvas = c;
    ctx = canvas.getContext('2d');
    infoBarEl = infoBar;

    els = {
      routeStatus: infoBarEl.querySelector('#routeStatus'),
      destInfo:    infoBarEl.querySelector('#destInfo'),
      btnRow:      infoBarEl.querySelector('#btnRow'),
      goBtn:       infoBarEl.querySelector('#goBtn'),
      cancelBtn:   infoBarEl.querySelector('#cancelBtn'),
      exitBtn:     infoBarEl.querySelector('#exitBtn'),
    };

    canvas.addEventListener('click', onCanvasClick);
    els.goBtn.addEventListener('click', onGo);
    els.cancelBtn.addEventListener('click', resetToStart);
    els.exitBtn.addEventListener('click', resetToStart);

    resize();          // size canvas to its container before first draw
    loadMap(mapUrl);
  }

  function update(msg) {
    if (msg.type === 'ego') {
      ego = { x: msg.x, y: msg.y, yaw: msg.yaw };
      draw();
    } else if (msg.type === 'route') {
      route = msg.points || [];
      draw();
    } else if (msg.type === 'routeResult') {
      if (msg.success) {
        els.routeStatus.textContent = `Route set \u2192 lanelet ${msg.goalLanelet}`;
        els.routeStatus.style.color = '#3ddc84';
        els.exitBtn.style.display = 'inline-block';
      } else {
        els.routeStatus.textContent = `Route failed: ${msg.error}`;
        els.routeStatus.style.color = '#ff5a5a';
      }
    }
    // ego/route/routeResult are the only messages the map view consumes.
    // objects / lanelet / behaviour etc. are ignored here (drive view / shell).
  }

  function resize() {
    if (!canvas) return;
    // size the drawing buffer to the canvas's actual displayed size
    const rect = canvas.getBoundingClientRect();
    canvas.width  = Math.max(1, Math.round(rect.width));
    canvas.height = Math.max(1, Math.round(rect.height));
    if (bounds) { buildTransform(); draw(); }
  }

  function onDestinationClick(cb) { destClickCb = cb; }

  return { init, update, resize, onDestinationClick };
})();