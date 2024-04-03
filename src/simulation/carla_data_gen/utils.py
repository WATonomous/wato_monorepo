import numpy as np
import cv2
import carla


BBOX_EDGES = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K


def get_image_point(loc, K, w2c):
        # Calculate 2D projection of 3D coordinate

        # Format the input coordinate (loc is a carla.Position object)
        point = np.array([loc.x, loc.y, loc.z, 1])
        # transform to camera coordinates
        point_camera = np.dot(w2c, point)

        # New we must change from UE4's coordinate system to an "standard"
        # (x, y ,z) -> (y, -z, x)
        # and we remove the fourth componebonent also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # now project 3D->2D using the camera matrix
        point_img = np.dot(K, point_camera)
        # normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

def draw_bbox(img, bb, camera_matrix, vehicle, K):
    # Calculate the dot product between the forward vector
    # of the vehicle and the vector between the vehicle
    # and the bounding box. We threshold this dot product
    # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
    forward_vec = vehicle.get_transform().get_forward_vector()
    ray = bb.location - vehicle.get_transform().location
    edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]
    if forward_vec.dot(ray) > 1:
        # Cycle through the vertices
        verts = [v for v in bb.get_world_vertices(carla.Transform())]
        for edge in edges:
            # Join the vertices into edges
            p1 = get_image_point(verts[edge[0]], K, camera_matrix)
            p2 = get_image_point(verts[edge[1]],  K, camera_matrix)
            # Draw the edges into the camera output
            cv2.line(img, (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1])), (0,0,255, 255), 1)

    return img

def project_to_image_plane(point_3d, world_2_camera, image_width, image_height):
    """
    Project a 3D point in world coordinates into the 2D image plane of the camera.
    """
    # Convert to homogeneous coordinates
    point_3d_hom = np.append(point_3d, 1)
    # Project onto camera
    point_cam = np.dot(world_2_camera, point_3d_hom)
    # Perspective division
    point_cam /= point_cam[2]
    # Convert to pixel coordinates
    point_img = np.array([point_cam[0] * image_width / 2 + image_width / 2,
                          -point_cam[1] * image_height / 2 + image_height / 2])
    return point_img

# https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/client_bounding_boxes.py
def get_matrix(transform):
    """
    Creates matrix from carla transform.
    """

    rotation = transform.rotation
    location = transform.location
    c_y = np.cos(np.radians(rotation.yaw))
    s_y = np.sin(np.radians(rotation.yaw))
    c_r = np.cos(np.radians(rotation.roll))
    s_r = np.sin(np.radians(rotation.roll))
    c_p = np.cos(np.radians(rotation.pitch))
    s_p = np.sin(np.radians(rotation.pitch))
    matrix = np.matrix(np.identity(4))
    matrix[0, 3] = location.x
    matrix[1, 3] = location.y
    matrix[2, 3] = location.z
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r
    return matrix

def bounding_box_to_world(bbox):
    extent = bbox.extent

    cords = np.zeros((8, 4))
    cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
    cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
    cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
    cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
    cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
    cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
    cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
    cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])

    world_matrix = get_matrix(bbox)

    world_cords = np.dot(world_matrix, np.transpose(cords))

    return world_cords

def world_points_to_camera(world_points_locations, camera_sensor: carla.Sensor, K):
    """
    params:
    world_points_locations: list of carla.Location objects
    camera_sensor: carla.Sensor object

    return:
    camera_points: np.array of shape (num_points, 3)
    """

    # Extract the x, y, and z coordinates from each carla.Location object
    world_points = np.array([[location.x, location.y, location.z] for location in world_points_locations])

    # Convert to homogeneous coordinates by adding a ones column
    homogeneous_world_points = np.hstack((world_points, np.ones((world_points.shape[0], 1))))

    # Assuming you have the world_2_camera transformation matrix from the camera sensor
    world_2_camera = np.array(camera_sensor.get_transform().get_inverse_matrix())

    # Transform the points from world space to camera space
    sensor_points_homogeneous = np.dot(world_2_camera, homogeneous_world_points.T)

    # Convert back to standard 3D coordinates from homogeneous coordinates
    sensor_points = sensor_points_homogeneous[:3, :] / sensor_points_homogeneous[3, :]

    # Corrected: Adjust the coordinate system if necessary. This depends on your specific setup in CARLA and how you want to map the coordinates.
    # This step was previously done as: point_in_camera_coords = np.array([sensor_points[1], sensor_points[2] * -1, sensor_points[0]]).T
    # Ensure this aligns with your camera's coordinate system conventions.
    point_in_camera_coords = sensor_points.T  # This assumes the transformed points are already in the correct coordinate system for projection.

    # Assuming K is the camera's intrinsic matrix
    # K = [...]

    # Project the 3D points to 2D using the intrinsic matrix K
    points_2d_homogeneous = np.dot(K, point_in_camera_coords.T)

    # Normalize the x, y values by the 3rd value to convert from homogeneous to 2D coordinates
    points_2d = points_2d_homogeneous[:2, :] / points_2d_homogeneous[2, :]

    # If you need to adjust the resulting 2D points (e.g., for display purposes), do so here
    points_2d_adjusted = points_2d.T  # This transpose back to shape (N, 2) where N is the number of points

    # If the subtraction of 10 was intended to adjust y-values in the final 2D coordinates
    points_2d_adjusted[:, 1] -= 10

    return points_2d
    # Return or use points_2d_adjusted as needed
    # return points_2d_adjusted
    # Return or use points_2d_adjusted as needed
    # return points_2d_adjusted
# https://github.com/carla-simulator/carla/discussions/5229


def transform_location_to_relative(vehicle_transform, location):
    """
    Transforms a location from the world coordinate system to the vehicle's local coordinate system.
    """
    vehicle_matrix = np.array(vehicle_transform.get_matrix())
    inv_vehicle_matrix = np.linalg.inv(vehicle_matrix)
    location_hom = np.array([location.x, location.y, location.z, 1])
    location_relative_hom = np.dot(inv_vehicle_matrix, location_hom)
    location_relative = carla.Location(x=location_relative_hom[0], y=location_relative_hom[1], z=location_relative_hom[2])
    return location_relative

def transform_rotation_to_relative(vehicle_rotation, actor_rotation):
    """
    Adjusts actor's rotation to be relative to the vehicle's rotation.
    This simplistic approach subtracts the Euler angles, which might not be ideal for all applications.
    """
    # Convert CARLA rotations (pitch, yaw, roll) to numpy arrays for subtraction
    vehicle_rot = np.array([vehicle_rotation.pitch, vehicle_rotation.yaw, vehicle_rotation.roll])
    actor_rot = np.array([actor_rotation.pitch, actor_rotation.yaw, actor_rotation.roll])

    # Calculate relative rotation
    rotation_relative = actor_rot - vehicle_rot

    return rotation_relative

def get_actor_bb_info(actor, vehicle_transform):
    """
    Gathers bounding box information and transforms its location and rotation to the vehicle's local coordinate system.
    """
    actor_transform = actor.get_transform()
    bb_world_loc = actor_transform.location
    bb_loc_relative = transform_location_to_relative(vehicle_transform, bb_world_loc)
    bb_extent = actor.bounding_box.extent

    # Handling rotation
    bb_rot_relative = transform_rotation_to_relative(vehicle_transform.rotation, actor_transform.rotation)

    # Structured array to hold the data
    data = [bb_loc_relative.x, bb_loc_relative.y, bb_loc_relative.z,
                      bb_rot_relative[0], bb_rot_relative[1], bb_rot_relative[2],
                      bb_extent.x, bb_extent.y, bb_extent.z]
    return data
