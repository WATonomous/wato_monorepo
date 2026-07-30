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

        setRouteClient.sendRequest(request, (response) => {
          console.log('SetRoute response:', response);

          //send the result back to the browser
          broadcast({ 
            type: 'routeResult',
            success: response.success,
            error: response.error_message,
            goalLanelet: Number(response.goal_lanelet_id)
          });
        });
      }
    });
  });

  rclnodejs.spin(node);
  server.listen(3000, () => console.log('UI running at http://localhost:3000'));
});