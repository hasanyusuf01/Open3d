
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

def np_arrayTOimage(depth_grid):
        # Assuming depth_grid is obtained from project_slice_to_2d function
        # Visualize the depth grid as an image
        plt.imshow(depth_grid, cmap='gray')
        plt.colorbar()
        plt.title('Depth Image')
        plt.xlabel('Horizontal Pixels')
        plt.ylabel('Vertical Pixels')
        plt.show()

# Definayion of extract_slices function 
def extract_slices(point_cloud, origin, slice_angle, threshold_radius):
    """
    Extract slices from a 3D point cloud.

    Parameters:
        - point_cloud (numpy.ndarray): Array containing the 3D point cloud data.
        - origin (tuple): Origin point in Cartesian coordinates (x, y, z).
        - slice_angle (int): Angle of the slice to extract (default is 120 degrees).
        - threshold_radius (float): Threshold radius to bound the slice (default is 70).

    Returns:
        - slices (list of numpy.ndarray): List of slices extracted from the point cloud.
    """
    slices = []
    num_slices = int(360 / slice_angle)
    
    # Conversion of Cartesian to Spherical coordinates
    X, Y, Z = point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2]
    r = np.sqrt((X - origin[0])**2 + (Y - origin[1])**2 + (Z - origin[2])**2)
    theta = np.arctan2(np.sqrt(X**2 + Y**2), Z)
    phi = np.arctan2(X, Y)

    print("x",X)
    print("y",Y)
    print("z",Z)
    print("r",r)
    print("theta",theta)
    print("pi",phi)
    print("max",np.max(phi))
    for i in range(num_slices):
        # Define angle for slicing
        slice_start = i * slice_angle
        slice_end = (i + 1) * slice_angle
 
        phi_wrapped = phi.copy()
        phi_wrapped[phi_wrapped < 0] += 2 * np.pi
        print("min..phi",np.min(phi_wrapped))
        print("max..phi",np.max(phi_wrapped))
        # Extract points within the slice
        mask = ((phi_wrapped >= np.radians(slice_start)) & (phi_wrapped < np.radians(slice_end)) & (r < threshold_radius)) 
        # print("mask",mask)
        sliced_points = point_cloud[mask] #conditional selection of elements from an array
        # print(sliced_points)
        slices.append(sliced_points)
    
    return slices
def project_slice_to_2d(slice_points, resolution):
    """
    Project a 3D slice onto a 2D grid.

    Parameters:
        - slice_points (numpy.ndarray): Array containing the points of the 3D slice.
        - resolution (tuple): Resolution of the 2D grid (vertical_units, horizontal_units).

    Returns:
        - depth_grid (numpy.ndarray): 2D grid containing depth values.
    """
    if slice_points.size:  # Check if slice_points is not empty


    # Extract x, y, z values from the slice points
        x, y, z = slice_points[:, 0], slice_points[:, 1], slice_points[:, 2]
        # Define grid coordinates
        vertical_units, horizontal_units = resolution
        y_grid, x_grid = np.mgrid[0:vertical_units, 0:horizontal_units]
        # Interpolate depth values onto the grid using nearest neighbor interpolation
        depth_grid = griddata((y, x), z, (y_grid, x_grid), method='nearest')
        
        return depth_grid
    else:
    # Handle the case when slice_points is empty, for example:
        print("No slice points available to process.")
        return 


# Read point cloud
pcd = o3d.io.read_point_cloud("/path/to/point cloud.ply")

point_cloud_np = np.asarray(pcd.points)

# Perform slicing operation
slice_angle = 72  # Example azimuth angle
threshold_radius = 100000  # Example threshold radius
resolution=(600, 600)
origin_point = [0, 0, 0]

sliced_points = extract_slices(point_cloud_np, origin_point, slice_angle=slice_angle, threshold_radius=threshold_radius)
i =0 
# Convert sliced points back to Open3D point cloud

for slice_points in sliced_points:
    sliced_pcd = o3d.geometry.PointCloud()
    sliced_pcd.points = o3d.utility.Vector3dVector(slice_points)
    print("slicepoints",i)
    i=i+1
    print(slice_points)
    print("slice_points")
    print(sliced_pcd)

# Visualize the sliced point cloud
    o3d.visualization.draw_geometries([sliced_pcd], zoom=0.4, front=[0.4257, -0.2125, -0.8795], lookat=[0, 0, 0], up=[0, 0, 1])


 



