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

## Adding Monitored Topics

Add topic names to the `topics` parameter in `config/interfacing.yaml` under the `topic_healthchecker` key. No code changes needed.
