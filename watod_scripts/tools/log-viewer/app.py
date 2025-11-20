#!/usr/bin/env python3
# Copyright (c) 2025-present WATonomous. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

from flask import Flask, render_template, jsonify, request
import docker
import os
from datetime import datetime, timezone

app = Flask(__name__)

# Initialize Docker client with explicit socket path
def get_docker_client():
    try:
        return docker.DockerClient(base_url='unix:///var/run/docker.sock')
    except Exception as e:
        print(f"Error connecting to Docker: {e}")
        raise

client = get_docker_client()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/containers')
def get_containers():
    """Get list of all containers, filtering out those older than 1 hour"""
    try:
        containers = client.containers.list(all=True)
        container_list = []
        now = datetime.now(timezone.utc)

        for container in containers:
            # Get container details
            container.reload()

            # For exited containers, check if they exited more than 1 hour ago
            if container.status == 'exited':
                finished_at_str = container.attrs['State']['FinishedAt']
                # Parse the timestamp (format: 2025-11-19T20:00:00.123456789Z)
                finished_at = datetime.fromisoformat(finished_at_str.replace('Z', '+00:00'))
                time_diff = (now - finished_at).total_seconds()

                # Skip containers that exited more than 1 hour ago
                if time_diff > 3600:
                    continue

                # Calculate human-readable time ago
                time_ago = format_time_ago(time_diff)
            else:
                time_ago = None

            container_list.append({
                'id': container.id[:12],
                'name': container.name,
                'status': container.status,
                'image': container.image.tags[0] if container.image.tags else container.image.id[:12],
                'time_ago': time_ago
            })
        return jsonify(container_list)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

def format_time_ago(seconds):
    """Format seconds into human-readable time ago string"""
    if seconds < 60:
        return f"{int(seconds)}s ago"
    elif seconds < 3600:
        minutes = int(seconds / 60)
        return f"{minutes}m ago"
    else:
        hours = int(seconds / 3600)
        return f"{hours}h ago"

@app.route('/api/logs/<container_id>')
def get_logs(container_id):
    """Get logs for a specific container"""
    try:
        tail = request.args.get('tail', '500')
        timestamps = request.args.get('timestamps', 'false') == 'true'

        container = client.containers.get(container_id)
        logs = container.logs(
            tail=int(tail),
            timestamps=timestamps,
            stdout=True,
            stderr=True
        ).decode('utf-8', errors='replace')

        return jsonify({
            'container_name': container.name,
            'logs': logs
        })
    except docker.errors.NotFound:
        return jsonify({'error': 'Container not found'}), 404
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/logs/<container_id>/stream')
def stream_logs(container_id):
    """Stream logs for a specific container"""
    try:
        container = client.containers.get(container_id)

        def generate():
            for log in container.logs(stream=True, follow=True, tail=100):
                yield f"data: {log.decode('utf-8', errors='replace')}\n\n"

        return generate(), {'Content-Type': 'text/event-stream'}
    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8888, debug=True)
