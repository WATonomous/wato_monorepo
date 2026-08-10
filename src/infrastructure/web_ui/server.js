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

  // push a topic value out to the browser
  let lastSent = 0;
  node.createSubscription('std_msgs/msg/Float64', '/test_speed', (msg) => {
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
      if (!msg.has_active_route) {
        broadcast({ type: 'route', points: [] });
        return;
      }
      const points = [];
      for (const lanelet of msg.lanelets) {
        for (const pt of lanelet.centerline) {
          points.push({ x: pt.x, y: pt.y });   // was pt.local_x / pt.local_y
        }
      }
      broadcast({ type: 'route', points });
    }
  );
*/
  // ego pose → browser 
  node.createSubscription(
    'geometry_msgs/msg/PoseStamped',
    '/world_modeling/slam/pose',
    (msg) => {
      const p = msg.pose.position;
      const q = msg.pose.orientation;
      const yaw = Math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
      );
      broadcast({ type: 'ego', x: p.x, y: p.y, yaw });
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
        const cls = o.detection.results?.[0]?.hypothesis?.class_id ?? '';
        return {
          x: c.position.x, y: c.position.y, z: c.position.z,
          yaw,
          sx: s.x, sy: s.y, sz: s.z,        // ROS-frame extents
          id: o.detection.id, cls
        };
      });
      broadcast({ type: 'objects', objects });
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

  rclnodejs.spin(node);
  server.listen(3000, () => console.log('UI running at http://localhost:3000'));
});