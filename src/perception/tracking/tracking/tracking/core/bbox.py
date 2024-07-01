from typing import Optional

class Bbox3d:
    def __init__(self, x, y, z, rz, w, l, h, cls="UNKNOWN"):
        """
        Dimensions are in the odom frame.
        The relationships between these measurements are:
        x, w <-> east, y, l <-> north, z, h <-> up

        x: x-coordinate of the center of the bounding box
        y: y-coordinate of the center of the bounding box
        z: z-coordinate of the center of the bounding box
        rz: Rotation about the z-axis. This is yaw.
        w: Width of the bounding box
        l: Length of the bounding box
        h: Height of the bounding box
        cls: The class of the obstacle. Ex: "PEDESTRIAN", "CYCLIST".
        """
        self.x = x
        self.y = y
        self.z = z
        self.rz = rz
        self.w = w
        self.l = l
        self.h = h
        self.cls = cls


class Bbox2d:
    def __init__(self, x, y, w, l, cls="UNKNOWN"):
        """
        x, y are in the camera frame, with (x, y) = (0, 0) representing the top-left of the frame.

        x: x-coordinate of the top left corner of the bounding box
        y: y-coordinate of the top left corner of the bounding box
        w: Width of the bounding box
        l: Length of the bounding box
        cls: The class of the obstacle. Ex: "PEDESTRIAN", "CYCLIST".
        """
        self.x = x
        self.y = y
        self.w = w
        self.l = l
        self.cls = cls


class BboxRadar:
    def __init__(self, x, y, rz, w, l, vel_x, vel_y):
        """
        Dimensions are in the odom frame.
        The relationships between these measurements are:
        x, w, vel_x <-> east, y, l, vel_y <-> north

        x: x=coordinate of the center of the bounding box
        y: y-coordinate of the center of the bounding box
        rz: Rotation about the z-axis. This is yaw.
        vel_x: x-component of the velocity observed by the radar
        vel_y: y-component of the velocity observed by the radar
        """
        self.x = x
        self.y = y
        self.rz = rz
        self.w = w
        self.l = l
        self.vel_x = vel_x
        self.vel_y = vel_y 


class FusedBbox:
    def __init__(self,
            bbox_3d: Optional[Bbox3d] = None,
            bbox_2d: Optional[Bbox2d] = None,
            bbox_radar: Optional[BboxRadar] = None):
        """
        bbox_3d: Optional 3d bounding box
        bbox_2d: Optional 2d bounding box
        bbox_radar: Optional bounding box from radar detections
        """
        self.bbox_3d = bbox_3d
        self.bbox_2d = bbox_2d
        self.bbox_radar = bbox_radar
