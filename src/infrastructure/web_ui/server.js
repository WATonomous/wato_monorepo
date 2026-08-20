const express = require('express');
const http = require('http');
const { WebSocketServer, WebSocket } = require('ws');
const rclnodejs = require('rclnodejs');

const app = express();
app.use(express.static('public'));

const server = http.createServer(app);
const wss = new WebSocketServer({ server });

function broadcast(obj) {
  const data = JSON.stringify(obj);
  wss.clients.forEach((c) => {
    if (c.readyState === WebSocket.OPEN) c.send(data);
  });
}

rclnodejs.init().then(() => {
  const node = new rclnodejs.Node('ui_node');
  let lastEgo = {x: 0, y: 0};

  // push a topic value out to the browser
  let lastSent = 0;
  node.createSubscription('std_msgs/msg/Float64', '/interfacing/can_state_estimator/body_velocity', (msg) => {
    const now = Date.now();
    if (now - lastSent < 100) return;
    lastSent = now;
    broadcast({ speed: msg.data });
  });

  // service client for SetRoute
  const setRouteClient = node.createClient('lanelet_msgs/srv/SetRoute', '/world_modeling/set_route');
  const getRouteClient = node.createClient('lanelet_msgs/srv/GetShortestRoute', '/world_modeling/get_shortest_route');

/*
  // route ahead → browser
  node.createSubscription(
    'lanelet_msgs/msg/RouteAhead',
    '/world_modeling/lanelet/route_ahead',
    (msg) => {
      if (!msg.has_active_route) { broadcast({ type: 'route', lanelets: [] }); return; }
      const lanelets = msg.lanelets.map((L) => ({
        centerline: L.centerline.map((p) => ({ x: p.x, y: p.y })),
        left:  L.left_boundary.map((p) => ({ x: p.x, y: p.y })),
        right: L.right_boundary.map((p) => ({ x: p.x, y: p.y })),
      }));
      broadcast({ type: 'route', lanelets });
    }
  );
*/ 

  // ego pose → browser 
  node.createSubscription(
    'nav_msgs/msg/Odometry',
    '/world_modeling/liso/odometry',
    (msg) => {
      const p = msg.pose.pose.position;
      const q = msg.pose.pose.orientation;
      const yaw = Math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
      );
      broadcast({ type: 'ego', x: p.x, y: p.y, yaw });
      lastEgo = { x: p.x, y: p.y }; 
    }
  );

  // tracked objects → browser
  node.createSubscription(
    'world_model_msgs/msg/WorldObjectArray',
    '/world_modeling/world_objects_enriched',          
    (msg) => {
      const objects = msg.objects.map((o) => {
        const c = o.detection.bbox.center;
        const s = o.detection.bbox.size;
        const q = c.orientation;
        const yaw = Math.atan2(
          2 * (q.w * q.z + q.x * q.y),
          1 - 2 * (q.y * q.y + q.z * q.z)
        );
        const cls = (o.detection.results ?? [])
          .map(r => r.hypothesis?.class_id ?? '')
          .find(c => c && c !== 'linear_velocity' && !c.startsWith('behavior:')) ?? '';

        const pred = o.predictions?.[0]?.poses?.map(p => ({
          x: p.pose.position.x,
          y: p.pose.position.y,
        })) ?? [];

        return {
          x: c.position.x, y: c.position.y, z: c.position.z,
          yaw,
          sx: s.x, sy: s.y, sz: s.z,       
          id: o.detection.id, 
          cls,
          pred,
        };
      });
      broadcast({ type: 'objects', objects });
    }
  );

  // lane geometry ahead → browser (map frame, same as ego/objects)
  node.createSubscription(
    //'lanelet_msgs/msg/LaneletAhead',
    //'/world_modeling/lanelet/lanelet_ahead',
    'lanelet_msgs/msg/MapVisualization',          
    '/world_modeling/lanelet/map_visualization',
    (msg) => {
      const RADIUS = 50;   // metres — only keep lanelet within this of ego
      const R2 = RADIUS * RADIUS;
      const near = (L) => L.centerline.some((p) => {
        const dx = p.x - lastEgo.x, dy = p.y - lastEgo.y;
        return dx * dx + dy * dy < R2;     
      });
      const lanelets = msg.lanelets
        .filter(near)
        .map((L) => ({
        centerline: L.centerline.map((p) => ({ x: p.x, y: p.y })),
        left:  L.left_boundary.map((p) => ({ x: p.x, y: p.y })),
        right: L.right_boundary.map((p) => ({ x: p.x, y: p.y })),
      }));
      broadcast({ type: 'lanelet', lanelets });
    }
  );

  // behaviour → browser
  node.createSubscription(
    'behaviour_msgs/msg/ExecuteBehaviour',
    '/behaviour/execute_behaviour',
    (msg) => {
      broadcast({ type: 'behaviour', behaviour: msg.behaviour });
    }
  );

  // listen for messages coming from the browser
  wss.on('connection', (socket) => {
    socket.on('message', async (raw) => {
      const msg = JSON.parse(raw);
      if (msg.type === 'setDestination') {
        console.log(`Destination clicked at x=${msg.x}, y=${msg.y}`);

        const ready = await setRouteClient.waitForService(2000);
        if (!ready) {
          console.log('SetRoute service not available');
          return;
        }

        //test point
        //const request = { goal_point: { x: 10.0, y: 5.0, z: 0.0 } };
        const request = { goal_point: { x: msg.x, y: msg.y, z: 0.0 } };

        setRouteClient.sendRequest(request, async (response) => {
          console.log('SetRoute response:', response);
          broadcast({
            type: 'routeResult',
            success: response.success,
            error: response.error_message,
            goalLanelet: Number(response.goal_lanelet_id)
          });

          // on success, fetch the FULL route to draw
          if (response.success) {
            const ready = await getRouteClient.waitForService(2000);
            if (ready) {
              getRouteClient.sendRequest({}, (routeResp) => {
                if (routeResp.success) {
                  const points = [];
                  for (const lanelet of routeResp.lanelets) {
                    for (const pt of lanelet.centerline) {
                      points.push({ x: pt.x, y: pt.y });
                    }
                  }
                  console.log('full route points:', points.length);
                  broadcast({ type: 'route', points });
                } else {
                  console.log('GetShortestRoute failed:', routeResp.error_message);
                }
              });
            }
          }
        });
      }
    });
  });

  server.listen(3000, () => console.log('UI running at http://localhost:3000'));
  rclnodejs.spin(node);
});