import numpy as np
import cv2
import matplotlib.pyplot as plt

# Camera settings
FOV = 120  # Field of view in degrees
W = 800  # Image width in pixels
H = 600  # Image height in pixels

# Calculate focal length (assuming square pixels)
f = W / (2 * np.tan(np.deg2rad(FOV) / 2))

# Calculate the principal point
cx = W / 2
cy = H / 2

# fx, fy are the same as f due to square pixels assumption
fx = fy = f

print(f"fx = {fx}, fy = {fy}, cx = {cx}, cy = {cy}")

def depth_map_from_carla_depth(depth_image):
    # Normalized depth map calculation
    depth_image = depth_image.astype(np.float32)
    normalized_depth = (depth_image[:, :, 2] + depth_image[:, :, 1] * 256 + depth_image[:, :, 0] * 256 * 256) / (256 * 256 * 256 - 1)
    # Convert normalized depth to meters
    depth_in_meters = 1000 * normalized_depth

    # # Convert to logarithmic depth.
    # logdepth = np.ones(normalized_depth.shape) + \
    #     (np.log(normalized_depth) / 5.70378)
    # logdepth = np.clip(logdepth, 0.0, 1.0)
    # logdepth *= 255.0
    # # Expand to three colors.
    # grayscale_image = np.repeat(logdepth[:, :, np.newaxis], 3, axis=2)
    
    # Scale values to the range 0 to 255 and convert to 8-bit unsigned integers
    # grayscale_image = (normalized_depth * 255).astype(np.uint8)
    # print("grayscale", grayscale_image)


    # cv2.imwrite('my_depth_map.png', grayscale_image )
    return depth_in_meters
 
def generate_point_cloud(depth_image_path, fx, fy, cx, cy):
    """
    Generate a point cloud from a depth image.
    
    :param depth_image_path: Path to the depth image file.
    :param fx: Focal length of the camera in the x-axis in pixel units.
    :param fy: Focal length of the camera in the y-axis in pixel units.
    :param cx: The x coordinate of the camera's principal point.
    :param cy: The y coordinate of the camera's principal point.
    :return: A NumPy array containing the 3D coordinates of the point cloud.
    """
    # Load the depth image, rgb format
    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)
    if depth_image is None:
        raise ValueError("Could not load depth image")
    # Get rid of the alpha channel if it exists
    # Assuming the depth image is a single-channel image where each pixel represents depth in meters.
    # print(depth_image)
    # Get the shape of the depth image
    height, width, channels = depth_image.shape
    
    # Generate a grid of coordinates corresponding to the image pixel indices
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    
    z = depth_map_from_carla_depth(depth_image)
    print(z)
    # Convert pixel indices to camera coordinates
    x = (u - cx) * z / fx
    # x = u
    y = (v - cy) * z / fy
    # y = v
    # z = depth_image
    
    # Stack the coordinates into a point cloud
    points = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    
    return points

# Example usage:
depth_image_path = 'images/000010_depth.png'
point_cloud = generate_point_cloud(depth_image_path, fx, fy, cx, cy)

# Now `point_cloud` contains the 3D coordinates of the points in the point cloud.

np.save('point_cloud.npy', point_cloud)

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2], marker='o', s=0.5)

ax.set_xlabel('R')
ax.set_ylabel('G')
ax.set_zlabel('B')
ax.set_title('RGB Sphere Point Cloud')
ax.set_aspect('auto')  # 'auto' lets the axes scale independently, ensuring points are accurately represented
plt.savefig('point_cloud_xy_projection.png', bbox_inches='tight', pad_inches=0)
plt.close()  # Close the figure to free memory
