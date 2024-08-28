import rclpy
from rclpy.node import Node

from carla_msgs.msg import CarlaTrafficLightInfoList, CarlaBoundingBox
from geometry_msgs.msg import Pose, Vector3

from vision_msgs.msg import Detection3D, Detection3DArray, BoundingBox3D

import os


class Datalogger(Node):
    def __init__(self):
        super().__init__('datalogger')
        self.get_logger().info("TRAFFIC LIGHT NODE STARTED")
        
        self.trafficLights = None
        
        self.trafficLightSubscription = self.create_subscription(
            CarlaTrafficLightInfoList,
            '/carla/traffic_lights/info',
            self.trafficLightCallback,
            10
        )
        
        self.trafficLightPublisher = self.create_publisher(
            Detection3DArray,
            '/carla/traffic_lights/detection3d_array',
            10
        )
        
        # Publish last traffic message every 15 seconds
        self.timer = self.create_timer(15, self.timer_callback)

    def trafficLightCallback(self, msg):
        self.get_logger().info(f"UPDATED {len(msg.traffic_lights)} LIGHTS")
        self.trafficLights = msg.traffic_lights
        self.timer_callback()

    def timer_callback(self):
        if (self.trafficLights is None):
            self.get_logger().info("traffic light sensor not yet initiated")
            return
        
        detectionsArray = Detection3DArray()
        detectionsArray.detections = []
        
        self.get_logger().info(f"PUBLISHING {len(self.trafficLights)} LIGHTS")
        for i in range(len(self.trafficLights)):
            # light: [CarlaTrafficLightInfo]
            light = self.trafficLights[i]
                                    
            detection = Detection3D()
            detection.bbox = BoundingBox3D()
            detection.bbox.center = light.transform
            detection.bbox.size = light.trigger_volume.size
            
            # add traffic light detection3d to array
            detectionsArray.detections.append(detection)
        
        self.trafficLightPublisher.publish(detectionsArray)


def main(args=None):
    rclpy.init(args=args)

    datalogger = Datalogger()

    rclpy.spin(datalogger)

    datalogger.destroy_node()
    rclpy.shutdown()
    return


if __name__ == '__main__':
    main()
