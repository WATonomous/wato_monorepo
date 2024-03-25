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
