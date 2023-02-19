# Sample Python Project
A sample ROS2 project used as a reference for migrating ROS Python nodes to ROS2. Highlights coding conventions and testing practices in ROS2.

![Architecture](https://user-images.githubusercontent.com/65926174/213855531-8a663eac-8637-4d8a-adb3-9982d140f7b8.png)
Each ROS2 node is containerized([Producer](../../../docker/samples/python/producer.Dockerfile), [Transformer](../../../docker/samples/python/transformer.Dockerfile), [Aggregator](../../../docker/samples/python/aggregator.Dockerfile)) and communicate with each other using ROS2 publishers and subscribers.

## Producer
Produces [unfiltered](../../ros_msgs/sample_msgs/msg/Unfiltered.msg) coordinate data at 500ms intervals and publishes data to the [Transformer](#transformer) and [Aggregator](#aggregator) nodes. The coordinate data will be incremented according to the 'velocity' parameter. This can be dynamically adjusted with, `ros2 param set /producer velocity <some value>`.

## Transformer
Collects [unfiltered](../../ros_msgs/sample_msgs/msg/Unfiltered.msg) coordinate data from the [Producer](#producer) at regular intervals and filters out messages containing invalid fields. Once 10 messages are collected they are packed into a [filtered_array](../../ros_msgs/sample_msgs/msg/FilteredArray.msg) message and published to the [Aggregator](#aggregator).

## Aggregator
Listens to messages from the Producer and Transfomer nodes and logs the frequency of messages to the console.

## Usage
**Before proceding ensure that you have followed the setup guide([setup](../../../docs/setup.md))**

To configure watod2, update `dev_config.local.sh` to include the samples profile.
```bash
#!/bin/bash
from dev_config.sh

ACTIVE_PROFILES="samples"
```

Then bring up the containers with,
```
watod2 up
```

## Development
The development workflow in ROS2 is similar to ROS, however, it uses a different set
of tools. For developing and testing ROS2 nodes the process is as follows.
1. Start the service and open a shell into a running Docker container
```
watod2 up <SERVICE_NAME>
watod2 -t <SERVICE_NAME>
```
2. After making changes to source code rebuild the ROS2 package
```
colcon build
```
3. Test that no breaking changes were introduced and that ROS2 coding conventions were followed
```
colcon test
colcon test-result --verbose // Get detailed information about failures
```
