import open3d as o3d
import numpy as np
import pandas as pd
from pyproj import Transformer
import matplotlib.pyplot as plt
import folium
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d

# === Step 1: Load PCD file ===
pcd = o3d.io.read_point_cloud("map.pcd")
points = np.asarray(pcd.points)

print("🔍 PCD File Info")
print(f"- Number of points: {points.shape[0]}")
min_bounds = points.min(axis=0)
max_bounds = points.max(axis=0)
print(f"- X range: {min_bounds[0]:.2f} to {max_bounds[0]:.2f} meters")
print(f"- Y range: {min_bounds[1]:.2f} to {max_bounds[1]:.2f} meters")
print(f"- Z range: {min_bounds[2]:.2f} to {max_bounds[2]:.2f} meters")

# Apply fixed rotation to match SLAM (X forward, Y left, Z up) → ENU
T_fix = np.array([
    [0, 1, 0],   # X ← Y
    [1, 0, 0],   # Y ← X
    [0, 0, -1]   # Z ← -Z
])
points = points @ T_fix.T

# Save local SLAM coordinates to CSV
df_local = pd.DataFrame(points, columns=["x", "y", "z"])
df_local.to_csv("slam_map_local_coords.csv", index=False)
print("✅ Saved local SLAM coordinates to 'slam_map_local_coords.csv'")

# === Step 2: Define SLAM origin (known GPS) ===
origin_lat = 15.671407
origin_lon = 74.457667
origin_alt = 0.0

# === Step 3: Coordinate transformations ===
wgs84_to_ecef = Transformer.from_crs("epsg:4326", "epsg:4978", always_xy=True)
ecef_to_wgs84 = Transformer.from_crs("epsg:4978", "epsg:4326", always_xy=True)
origin_ecef = wgs84_to_ecef.transform(origin_lon, origin_lat, origin_alt)
print("SLAM Origin in ECEF:", origin_ecef)

# === Step 4: Load poses and GPS trajectory ===
def load_kitti_poses(file_path):
    poses = []
    with open(file_path, 'r') as f:
        for line in f:
            values = list(map(float, line.strip().split()))
            pose = np.array(values).reshape(3, 4)
            pose = np.vstack((pose, [0, 0, 0, 1]))
            poses.append(pose)
    return poses

poses = load_kitti_poses("output_120_d_poses_kitti.txt")
trajectory_xyz = np.array([pose[:3, 3] for pose in poses])
trajectory_xyz = trajectory_xyz @ T_fix.T

gps_trajectory = pd.read_csv("gnss_data_20250324_103623.csv")
print("Columns in GPS trajectory CSV:", gps_trajectory.columns)
gps_trajectory.rename(columns={'Latitude': 'lat', 'Longitude': 'lon'}, inplace=True)
gps_trajectory['alt'] = 0.0
gps_trajectory = gps_trajectory[['lon', 'lat', 'alt']]
lonlatalt_traj = gps_trajectory.to_numpy()

slam_indices = np.linspace(0, len(lonlatalt_traj) - 1, len(trajectory_xyz)).astype(int)
gps_interpolated = lonlatalt_traj[slam_indices]
gps_trajectory_ecef = np.array([wgs84_to_ecef.transform(lon, lat, alt) for lon, lat, alt in gps_interpolated])

# === Step 5: Align using first pose instead of centroid ===
first_slam = trajectory_xyz[0]
first_gps = gps_trajectory_ecef[0]
translation = first_gps - first_slam
slam_centered = trajectory_xyz - first_slam
gps_centered = gps_trajectory_ecef - first_gps
rotation, _ = R.align_vectors(gps_centered, slam_centered)

print("Rotation Matrix (from SLAM to GPS):")
print(rotation.as_matrix())
print("Translation Vector:", translation)

slam_map_centered = points - first_slam
#slam_map_transformed = points - first_slam + translation
slam_map_transformed = rotation.apply(slam_map_centered) + translation
slam_map_global = np.array([ecef_to_wgs84.transform(*pt) for pt in slam_map_transformed])
df_global = pd.DataFrame(slam_map_global, columns=["lon", "lat", "alt"])
df_global.to_csv("slam_map_global_coords_corrected.csv", index=False)

trajectory_transformed = rotation.apply(slam_centered) + translation
trajectory_global = np.array([ecef_to_wgs84.transform(*pt) for pt in trajectory_transformed])
df_traj = pd.DataFrame(trajectory_global, columns=["lon", "lat", "alt"])
df_traj.to_csv("trajectory_global_coords_transformed.csv", index=False)

'''
# === Step 5: Align using two points instead of full vectors ===

# Select two points from SLAM and GPS trajectories
slam_point1 = trajectory_xyz[0]  # First SLAM point
slam_point2 = trajectory_xyz[10]  # Use a point farther away to ensure a valid vector
gps_point1 = gps_trajectory_ecef[0]  # First GPS point
gps_point2 = gps_trajectory_ecef[10]  # Use a point farther away to ensure a valid vector

# Compute vectors between the two points
slam_vector = slam_point2 - slam_point1
gps_vector = gps_point2 - gps_point1

# Check for zero-length vectors
if np.linalg.norm(slam_vector) == 0 or np.linalg.norm(gps_vector) == 0:
    raise ValueError("SLAM or GPS points are identical, resulting in zero-length vectors.")

# Normalize the vectors
slam_vector_normalized = slam_vector / np.linalg.norm(slam_vector)
gps_vector_normalized = gps_vector / np.linalg.norm(gps_vector)

# Compute the rotation matrix to align the SLAM vector with the GPS vector
rotation, _ = R.align_vectors([gps_vector_normalized], [slam_vector_normalized])

print("Rotation Matrix (from SLAM to GPS):")
print(rotation.as_matrix())

# Compute translation using the first points
translation = gps_point1 - slam_point1
print("Translation Vector:", translation)

# Apply the rotation and translation to the SLAM map
slam_map_centered = points - slam_point1
slam_map_transformed = rotation.apply(slam_map_centered) + translation
slam_map_global = np.array([ecef_to_wgs84.transform(*pt) for pt in slam_map_transformed])
df_global = pd.DataFrame(slam_map_global, columns=["lon", "lat", "alt"])
df_global.to_csv("slam_map_global_coords_corrected.csv", index=False)

# Apply the rotation and translation to the SLAM trajectory
trajectory_centered = trajectory_xyz - slam_point1
trajectory_transformed = rotation.apply(trajectory_centered) + translation
trajectory_global = np.array([ecef_to_wgs84.transform(*pt) for pt in trajectory_transformed])
df_traj = pd.DataFrame(trajectory_global, columns=["lon", "lat", "alt"])
df_traj.to_csv("trajectory_global_coords_transformed.csv", index=False)

'''
# === Step 6: Interactive Map ===
start_lat = df_global['lat'].iloc[0]
start_lon = df_global['lon'].iloc[0]
m = folium.Map(location=[start_lat, start_lon], zoom_start=18, tiles=None)
folium.TileLayer(
    tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attr="Esri",
    name="Esri Satellite",
    overlay=False,
    control=True
).add_to(m)

for lat, lon in zip(df_global['lat'][::100], df_global['lon'][::100]):
    folium.CircleMarker(location=[lat, lon], radius=2, color="blue", fill=True, fill_color="blue", fill_opacity=0.6).add_to(m)

for lon, lat, alt in gps_interpolated:
    folium.CircleMarker(location=[lat, lon], radius=3, color="green", fill=True, fill_color="green", fill_opacity=0.6).add_to(m)

trajectory = list(zip(df_traj['lat'], df_traj['lon']))
folium.PolyLine(trajectory, color="yellow", weight=4).add_to(m)
folium.Marker([origin_lat, origin_lon], tooltip="SLAM Origin").add_to(m)
m.save("trajectory_with_slam_and_interpolated_gps_subset.html")

# === Step 7: Matplotlib Visual ===
plt.figure(figsize=(10, 10))
plt.scatter(slam_map_global[::100, 1], slam_map_global[::100, 0], s=1, c='blue', label='Corrected Map Points')
plt.plot(trajectory_global[:, 1], trajectory_global[:, 0], c='red', linewidth=2, label='GPS Trajectory')
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("Corrected SLAM Map and GPS Trajectory")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.show()
