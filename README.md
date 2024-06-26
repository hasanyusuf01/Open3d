# Open3d-Point Cloud Slicing


**This code demonstrates how to extract and visualize slices from a 3D point cloud using Python and the Open3D library.**

**Key steps and features:**

1. **Importing Necessary Libraries:**
   - `open3d`: For working with 3D point clouds.
   - `numpy`: For numerical computations and array manipulation.

2. **Extracting Slices Function:**
   - `extract_slices(point_cloud, origin, slice_angle, threshold_radius)`
     - Takes a point cloud, origin point, slice angle, and threshold radius as input.
     - Converts Cartesian coordinates to spherical coordinates.
     - Divides the point cloud into slices based on the specified slice angle.
     - Extracts points within each slice, falling within the threshold radius.
     - Returns a list of extracted slices as NumPy arrays.

3. **Point Cloud Loading and Slicing:**
   - Reads a point cloud from a PLY file using Open3D.
   - Converts the point cloud to a NumPy array for manipulation.
   - Calls the `extract_slices` function with example parameters to extract slices.

4. **Visualizing Sliced Point Clouds:**
   - Iterates through the extracted slices.
   - Creates Open3D point clouds from the extracted slice points.
   - Visualizes each sliced point cloud using Open3D's visualization functions.

5.**Output:**
- The video below shows the slices of a 3D pointcloud of a sphere, for a slice angle of 72 degrees cutting the sphere in 5 slices.
- The voxel size was increased with the help of `open3d` so the points are vissible easily

  
https://github.com/hasanyusuf01/Open3d/assets/96037958/8ff5ce40-d2f7-452b-9a02-8b89bc12915e


