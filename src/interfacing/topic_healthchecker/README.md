# topic_healthchecker

Monitors liveness of a configurable set of ROS topics and exposes the health status over an HTTP endpoint.

## Overview

During vehicle operation, a large number of sensor and control topics must be publishing at expected rates. `topic_healthchecker` subscribes dynamically to a configured list of topics, tracks the age of the most recent message on each, and serves a JSON health summary over HTTP. This lets external monitoring tools (dashboards, scripts) poll vehicle health without being ROS-aware.

## Architecture

```
ROS topic graph
     │
[topic_healthchecker] ─── wall timer ───► check staleness ───► JSON cache
                                                                     │
                                                              HTTP server thread
                                                                     │
                                                          GET http://localhost:8080
```

The node periodically scans the ROS graph for any new instances of the monitored topics and subscribes to them on first appearance. Each incoming message updates the last-seen timestamp. The HTTP server thread serves the latest cached JSON on every request.

## HTTP Response Format

```json
{
  "topics": {
    "/some/topic": {
      "status": "ok",
      "age_sec": 0.12,
      "rate_hz": 9.8
    },
    "/stale/topic": {
      "status": "stale",
      "age_sec": 7.3,
      "rate_hz": 0.0
    }
  }
}
```

`status` is `"ok"` when the last message arrived within `stale_timeout` seconds, `"stale"` otherwise. `rate_hz` is computed over a rolling window of the last `window_size` messages.
