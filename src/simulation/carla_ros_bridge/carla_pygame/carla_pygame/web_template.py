# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""HTML template for the web-based CARLA map visualizer."""

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>CARLA Map Visualizer</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: #1a1a1a;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 100vh;
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
        }
        .container {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
        }
        h1 {
            color: #eee;
            font-size: 1.5em;
            font-weight: normal;
        }
        #status {
            color: #888;
            font-size: 0.9em;
        }
        #status.connected { color: #8ae234; }
        #status.disconnected { color: #ef2929; }
        #canvas-container {
            position: relative;
            border: 2px solid #333;
            border-radius: 4px;
            overflow: hidden;
        }
        #mapCanvas {
            display: block;
            cursor: grab;
        }
        #mapCanvas:active {
            cursor: grabbing;
        }
        .controls {
            color: #888;
            font-size: 0.85em;
            text-align: center;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>CARLA Map Visualizer</h1>
        <div id="status" class="disconnected">Connecting...</div>
        <div id="canvas-container">
            <canvas id="mapCanvas" width="{{ width }}" height="{{ height }}"></canvas>
        </div>
        <div class="controls">
            Scroll to zoom | Drag to pan
        </div>
    </div>

    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <script>
        const canvas = document.getElementById('mapCanvas');
        const ctx = canvas.getContext('2d');
        const status = document.getElementById('status');

        const socket = io();

        let isDragging = false;
        let lastMousePos = { x: 0, y: 0 };

        socket.on('connect', () => {
            status.textContent = 'Connected';
            status.className = 'connected';
        });

        socket.on('disconnect', () => {
            status.textContent = 'Disconnected';
            status.className = 'disconnected';
        });

        socket.on('frame', (data) => {
            const img = new Image();
            img.onload = () => {
                ctx.drawImage(img, 0, 0);
            };
            img.src = 'data:image/png;base64,' + data;
        });

        canvas.addEventListener('mousedown', (e) => {
            isDragging = true;
            lastMousePos = { x: e.clientX, y: e.clientY };
            canvas.style.cursor = 'grabbing';
        });

        canvas.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const dx = e.clientX - lastMousePos.x;
                const dy = e.clientY - lastMousePos.y;
                lastMousePos = { x: e.clientX, y: e.clientY };
                socket.emit('pan', { dx: dx, dy: dy });
            }
        });

        canvas.addEventListener('mouseup', () => {
            isDragging = false;
            canvas.style.cursor = 'grab';
        });

        canvas.addEventListener('mouseleave', () => {
            isDragging = false;
            canvas.style.cursor = 'grab';
        });

        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const delta = e.deltaY > 0 ? -1 : 1;
            const rect = canvas.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            socket.emit('zoom', { delta: delta, x: x, y: y });
        });

        canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    </script>
</body>
</html>
"""
