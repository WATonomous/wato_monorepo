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
  let lastSent = 0;
  node.createSubscription('std_msgs/msg/Float64', '/test_speed', (msg) => {
    const now = Date.now();
    if (now - lastSent < 100) return;
    lastSent = now;
    broadcast({ speed: msg.data });
  });
  rclnodejs.spin(node);
  server.listen(3000, () => console.log('UI running at http://localhost:3000'));
});
