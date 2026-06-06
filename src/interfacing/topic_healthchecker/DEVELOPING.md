# Developing topic_healthchecker

## Topics

The node has no fixed subscriptions. Topics are declared in the `topics` parameter and subscribed dynamically when they appear in the ROS graph.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `topics` | string[] | — | List of topic names to monitor |
| `http_port` | int | `8080` | Port for the HTTP health endpoint |
| `stale_timeout` | double | `5.0` | Seconds after which a topic is considered stale |
| `window_size` | int | `50` | Rolling window size for message rate calculation |
| `check_period` | double | `2.0` | How often to scan the graph for new topic publishers (seconds) |

## Build & Launch

```bash
colcon build --packages-select topic_healthchecker
# Launched by interfacing_bringup/interfacing.launch.yaml
```

## Internal Architecture

**Dynamic subscriptions:** A wall timer fires every `check_period` seconds. It queries the ROS graph for publishers on each monitored topic. If a publisher is found and no subscription exists yet, a generic subscription is created using the `rclcpp::GenericSubscription` API (type-erased, so no message type knowledge is needed).

**Rate calculation:** Each subscription callback records the arrival time in a circular buffer of size `window_size`. Rate is estimated as `(window_size - 1) / (newest_time - oldest_time)`.

**HTTP server:** Runs in a separate thread started at node construction. On each `GET /` request it serializes the current per-topic state (last age, rate, status) to JSON and returns it. The per-topic state map is protected by a mutex shared with the subscription callbacks.

**Rate smoothing:** A larger `window_size` reduces rate noise but takes longer to react to actual rate changes. If a topic's reported rate looks wrong despite healthy message delivery, lower `window_size` for less-averaged estimates.

## After Launching

1. **Verify the HTTP server started** — check the node log for:

   ```
   Health HTTP server listening on port 8080
   Topic healthchecker monitoring N topics, HTTP on port 8080
   ```

2. **Poll the endpoint:**

```bash
   curl http://localhost:8080
   ```

1. **Watch for subscriptions** — as monitored topics come online, the node logs:

   ```
Subscribed to /some/topic [sensor_msgs/msg/PointCloud2]

   ```

## Definition of Good Result

A healthy system returns HTTP 200 with an overall `{"status":"ok"}` and all per-topic statuses as `"healthy"`:

```json
{
  "status": "ok",
  "topics": {
    "/lidar_cc/velodyne_points": {"status": "healthy", "age_sec": 0.08, "rate_hz": 10.1},
    "/novatel/oem7/bestpos":     {"status": "healthy", "age_sec": 0.21, "rate_hz": 4.9}
  }
}
```

Possible per-topic status values:

| Status | Meaning |
|--------|---------|
| `"healthy"` | Message received within `stale_timeout` seconds |
| `"stale"` | Last message older than `stale_timeout` (default 5 s) |
| `"no_publishers"` | Topic exists in graph but no node is publishing |
| `"not_found"` | Topic has never appeared in the ROS graph |

Any status other than `"healthy"` for an expected topic indicates a driver or node problem.

## Adding Monitored Topics

Add topic names to the `topics` parameter in `config/interfacing.yaml` under the `topic_healthchecker` key. No code changes needed.
