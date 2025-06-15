from numba import jit
from scipy.spatial import ConvexHull
import copy
import numpy as np
import tf_transformations as tr


@jit
def poly_area(x, y):
    """
    Compute area of a polygon.

    x: np.array of x coordinates
    y: np.array of y coordinates
    """
    # https://stackoverflow.com/a/49129646
    correction = x[-1] * y[0] - y[-1] * x[0]
    main_area = np.dot(x[:-1], y[1:]) - np.dot(y[:-1], x[1:])
    return 0.5*np.abs(main_area + correction)


@jit
def box3d_vol(corners):
    """
    Compute the volume of a 3d bounding box from it's corners.

    corners: (8, 3) no assumption on axis direction.
    """
    a = np.sqrt(np.sum((corners[0, :] - corners[1, :])**2))
    b = np.sqrt(np.sum((corners[1, :] - corners[2, :])**2))
    c = np.sqrt(np.sum((corners[0, :] - corners[4, :])**2))
    return a*b*c


def convex_hull_intersection(p1, p2):
    """
    Compute area of two convex hull's intersection area.

    p1, p2 are a list of (x, y) tuples of hull vertices.
    return a list of (x, y) for the intersection and its volume.
    """
    inter_p = polygon_clip(p1, p2)
    if inter_p is not None:
        hull_inter = ConvexHull(inter_p)
        return inter_p, hull_inter.volume
    else:
        return None, 0.0


def polygon_clip(subjectPolygon, clipPolygon):
    """
    Clip a polygon with another polygon.

    Args:
    ----
        subjectPolygon: a list of (x, y) 2d points, any polygon.
        clipPolygon: a list of (x, y) 2d points, has to be *convex*
    Note:
        **points have to be counter-clockwise ordered**

    Return:
    ------
        a list of (x, y) vertex point for the intersection polygon.

    """
    def inside(p):
        return(cp2[0]-cp1[0])*(p[1]-cp1[1]) > (cp2[1]-cp1[1])*(p[0]-cp1[0])

    def computeIntersection():
        dc = [cp1[0] - cp2[0], cp1[1] - cp2[1]]
        dp = [s[0] - e[0], s[1] - e[1]]
        n1 = cp1[0] * cp2[1] - cp1[1] * cp2[0]
        n2 = s[0] * e[1] - s[1] * e[0]
        n3 = 1.0 / (dc[0] * dp[1] - dc[1] * dp[0])
        return [(n1*dp[0] - n2*dc[0]) * n3, (n1*dp[1] - n2*dc[1]) * n3]

    outputList = subjectPolygon
    cp1 = clipPolygon[-1]

    for clipVertex in clipPolygon:
        cp2 = clipVertex
        inputList = outputList
        outputList = []
        s = inputList[-1]

        for subjectVertex in inputList:
            e = subjectVertex
            if inside(e):
                if not inside(s):
                    outputList.append(computeIntersection())
                outputList.append(e)
            elif inside(s):
                outputList.append(computeIntersection())
            s = e
        cp1 = cp2
        if len(outputList) == 0:
            return None
    return(outputList)


def iou3d(corners1, corners2):
    """
    Compute 3D bounding box IoU.

    Input:
        corners1: numpy array (8, 3), assume up direction is Z
        corners2: numpy array (8, 3), assume up direction is Z
    Output:
        iou: 3D bounding box IoU
        iou_2d: bird's eye view 2D bounding box IoU
    """
    # corner points are in counter clockwise order
    rect1 = [(corners1[i, 0], corners1[i, 1]) for i in range(3, -1, -1)]
    rect2 = [(corners2[i, 0], corners2[i, 1]) for i in range(3, -1, -1)]
    area1 = poly_area(np.array(rect1)[:, 0], np.array(rect1)[:, 1])
    area2 = poly_area(np.array(rect2)[:, 0], np.array(rect2)[:, 1])
    inter, inter_area = convex_hull_intersection(rect1, rect2)
    iou_2d = inter_area/(area1+area2-inter_area)
    zmax = min(corners1[0, 2], corners2[0, 2])
    zmin = max(corners1[4, 2], corners2[4, 2])
    inter_vol = inter_area * max(0.0, zmax-zmin)
    vol1 = box3d_vol(corners1)
    vol2 = box3d_vol(corners2)
    iou = inter_vol / (vol1 + vol2 - inter_vol)
    return iou, iou_2d


@jit
def roty(t):
    """Rotation about the y-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c,  0.0,  s],
                     [0.0, 1.0, 0.0],
                     [-s, 0.0,  c]])


@jit
def rotz(t):
    """Rotation about the z-axis."""
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s,  0.0],
                     [s,  c,  0.0],
                     [0.0, 0.0, 1.0]])


def convert_3dbox_to_8corner(bbox3d_input):
    """
    Take an object and a projection matrix (P) and projects the 3d bounding box into the image plane.

    Returns
    -------
        corners_2d: (8, 2) array in left image coord.
        corners_3d: (8, 3) array in in rect camera coord.

    Note: the output of this function will be passed to the funciton iou3d
        for calculating the 3D-IOU. But the function iou3d was written for
        kitti, so the caller needs to set nuscenes_to_kitti to True if
        the input bbox3d_input is in nuscenes format.

    """  # noqa: E501
    # compute rotational matrix around yaw axis
    bbox3d = copy.copy(bbox3d_input)

    R = rotz(bbox3d[3])

    # 3d bounding box dimensions
    w = bbox3d[4]
    l = bbox3d[5]  # noqa: E741  # noqa: E741
    h = bbox3d[6]

    # 3d bounding box corners
    x_corners = [w/2, w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2]
    y_corners = [l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2, l/2]
    z_corners = [h/2, h/2, h/2, h/2, -h/2, -h/2, -h/2, -h/2]

    # rotate and translate 3d bounding box
    corners_3d = np.dot(R, np.vstack([x_corners, y_corners, z_corners]))
    corners_3d[0, :] = corners_3d[0, :] + bbox3d[0]
    corners_3d[1, :] = corners_3d[1, :] + bbox3d[1]
    corners_3d[2, :] = corners_3d[2, :] + bbox3d[2]

    return np.transpose(corners_3d)


def angle_in_range(angle):
    """
    Get angle in range.

    Input angle: -2pi ~ 2pi
    Output angle: -pi ~ pi
    """
    if angle > np.pi:
        angle -= 2 * np.pi
    if angle < -np.pi:
        angle += 2 * np.pi
    return angle


def diff_orientation_correction(det, trk):
    """
    Return the angle diff = det - trk.

    if angle diff > 90 or < -90, rotate trk and update the angle diff.
    """
    diff = det - trk
    diff = angle_in_range(diff)
    if diff > np.pi / 2:
        diff -= np.pi
    if diff < -np.pi / 2:
        diff += np.pi
    diff = angle_in_range(diff)
    return diff


class ClippingError(Exception):
    pass


def project_bbox_2d(bbox, to_camera_transform, intrinsic_matrix):
    """
    Project bbox to 2d.

    Inputs:
        - bbox: [x y z rz w l h]
        - to_camera_transform: geometry_msgs/Transform
    Outputs:
        - bbox_2d: [x1 y1 w l] top left corner coordinates + width (x) and height (y)
    """
    points = convert_3dbox_to_8corner(bbox)  # Convert to tuple of 8 corners [[xyz], [xyz], ...]
    points = transform_points(points, to_camera_transform)  # Transform corners to camera frame
    # print("Camera", points)
    if (np.any(points[:, 2] <= 0)):  # Reject boxes behind the camera
        raise ClippingError()

    points_cam = np.dot(intrinsic_matrix, points.T).T  # Project viewing frustum to 2d
    points_cam_norm = points_cam[:, :2] / points_cam[:, 2].reshape(-1, 1)  # Normalize by z coord

    min_xy = np.min(points_cam_norm, axis=0)
    max_xy = np.max(points_cam_norm, axis=0)
    dimensions = max_xy - min_xy
    # Clip boxes outside the camera image
    if (np.any(dimensions == 0) or np.any(points_cam[:, 2] > 100)):
        raise ClippingError()

    return np.hstack((min_xy, dimensions))


def transform_points(points, transform):
    """
    Transform a series of points based on a transformationStamped.

    points: [Nx3] Array of points xyz
    transform: Transform ROS message, eg StampedTransform.Transform
    """
    if len(points) == 0:
        return points

    translation = tr.translation_matrix(
        [transform.translation.x, transform.translation.y, transform.translation.z]
    )
    rotation = tr.quaternion_matrix([transform.rotation.x, transform.rotation.y,
                                     transform.rotation.z, transform.rotation.w])
    combined_hom_transform = np.dot(translation, rotation)

    points_hom = np.hstack((points[:, :3], np.ones((points.shape[0], 1), dtype=np.float32)))
    points_transformed = np.dot(combined_hom_transform, points_hom.T).T

    points[:, :3] = points_transformed[:, :3]

    return points


def transform_boxes(boxes, transform):
    """
    Transform boxes.

    Boxes: N * [x y z rz w l h]
    Transform: Transformation, ie TransformStamped.Transform
    """
    if len(boxes) == 0:
        return boxes

    boxes = transform_points(boxes, transform)

    tf_yaw = tr.euler_from_quaternion([transform.rotation.x, transform.rotation.y,
                                       transform.rotation.z, transform.rotation.w])[2]
    boxes[:, 3] += tf_yaw

    return boxes
