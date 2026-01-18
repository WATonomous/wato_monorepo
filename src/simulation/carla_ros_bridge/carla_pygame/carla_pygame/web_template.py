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
        #stream-container {
            position: relative;
            border: 2px solid #333;
            border-radius: 4px;
            overflow: hidden;
            cursor: grab;
        }
        #stream-container:active {
            cursor: grabbing;
        }
        #stream {
            display: block;
            user-select: none;
            -webkit-user-drag: none;
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
        <div id="stream-container">
            <img id="stream" src="/stream" width="{{ width }}" height="{{ height }}" draggable="false" />
        </div>
        <div class="controls">
            Scroll to zoom | Drag to pan
        </div>
    </div>

    <script>
        const container = document.getElementById('stream-container');
        let isDragging = false;
        let lastMousePos = { x: 0, y: 0 };

        container.addEventListener('mousedown', (e) => {
            isDragging = true;
            lastMousePos = { x: e.clientX, y: e.clientY };
            container.style.cursor = 'grabbing';
        });

        container.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const dx = e.clientX - lastMousePos.x;
                const dy = e.clientY - lastMousePos.y;
                lastMousePos = { x: e.clientX, y: e.clientY };
                fetch('/pan', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ dx: dx, dy: dy })
                });
            }
        });

        container.addEventListener('mouseup', () => {
            isDragging = false;
            container.style.cursor = 'grab';
        });

        container.addEventListener('mouseleave', () => {
            isDragging = false;
            container.style.cursor = 'grab';
        });

        container.addEventListener('wheel', (e) => {
            e.preventDefault();
            const delta = e.deltaY > 0 ? -1 : 1;
            const rect = container.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            fetch('/zoom', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ delta: delta, x: x, y: y })
            });
        });

        container.addEventListener('contextmenu', (e) => e.preventDefault());
    </script>
</body>
</html>
"""
