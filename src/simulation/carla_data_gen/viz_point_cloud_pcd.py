import open3d as o3d
import numpy as np
import argparse

def create_bbox_geometry(center, extent, pitch, yaw, roll):
    """
    Creates an Open3D bounding box geometry given the center, extent, and Euler angles (roll, pitch, yaw).
    """
    R = rotation_matrix(pitch, yaw, roll)
    bbox = o3d.geometry.OrientedBoundingBox(center=center, R=R, extent=2*extent)
    return bbox

def rotation_matrix(pitch, yaw, roll):
    # Convert angles from degrees to radians
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)
    
    # Rotation matrix around the x-axis (roll)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    
    # Rotation matrix around the y-axis (pitch)
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    
    # Rotation matrix around the z-axis (yaw)
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    
    # The final rotation matrix is obtained by the matrix multiplication of Rx, Ry, and Rz
    # Note: The order of multiplication depends on the convention you are following. Here, it's Rz * Ry * Rx
    R = np.dot(Rz, np.dot(Ry, Rx))
    
    return R


def visualize_pcd(pcd_file, bbox_file, output_png):
    # Load the PCD file
    pcd = o3d.io.read_point_cloud(pcd_file)
    # Load the bounding box data
    bbox_data = np.load(bbox_file)

    # Initialize offscreen renderer
    renderer = o3d.visualization.rendering.OffscreenRenderer(800, 600)
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "defaultUnlit"

    # Add point cloud to the scene
    renderer.scene.add_geometry("point_cloud", pcd, mat)

    # Create and add bounding box geometries
    for i in range(bbox_data.shape[0]):
        center, extent, pitch, yaw, roll = bbox_data[i, :3], bbox_data[i, 6:], bbox_data[i, 3], bbox_data[i, 4], bbox_data[i, 5]
        bbox = create_bbox_geometry(center, extent, pitch, yaw, roll)
        renderer.scene.add_geometry(f"bbox_{i}", bbox, mat)

    # Adjust camera and render the scene
    # Example to set a default view point, can be adjusted as needed
    camera = renderer.scene.camera
    camera.set_projection(field_of_view=60, aspect_ratio=800 / 600, near_plane=0.01, far_plane=1000, field_of_view_type=o3d.visualization.rendering.Camera.FovType.Horizontal)

    camera.look_at([0, 0, 0], [0, 0, 200], [0, -1, 0])
    img = renderer.render_to_image()
    o3d.io.write_image(output_png, img)

def parse_arguments():
    parser = argparse.ArgumentParser(description="Visualize PCD with bounding boxes and save as PNG in headless mode.")
    parser.add_argument("filename", type=str, help="Path to the PCD file.")
    parser.add_argument("bbox_filename", type=str, help="Path to the bounding box file (npy).")
    parser.add_argument("output_png", type=str, help="Path to save the output PNG file.")
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_arguments()
    visualize_pcd(args.filename, args.bbox_filename, args.output_png)
