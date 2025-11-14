import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Load the datasets
data_df = pd.read_csv(r'EuRoc/MH01/mav0/state_groundtruth_estimate0/data.csv')
# Clean up column names by removing leading/trailing spaces
data_df.columns = data_df.columns.str.strip()

camera_df = pd.read_csv(r'CameraTrajectory.txt', sep=' ', header=None)
camera_df.columns = ['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']

# 3D Plotting
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot data.csv trajectory
ax.plot(data_df['p_RS_R_x [m]'], data_df['p_RS_R_y [m]'], data_df['p_RS_R_z [m]'], label='data.csv Trajectory')

# Plot CameraTrajectory.txt trajectory
ax.plot(camera_df['tx'], camera_df['ty'], camera_df['tz'], label='CameraTrajectory.txt Trajectory')

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('3D Trajectory Comparison')
ax.legend()
plt.savefig('trajectory_comparison.png')

# Trajectory Matching
# Normalize timestamps to seconds for easier comparison
data_timestamps = data_df['#timestamp'].to_numpy() / 1e9
camera_timestamps = camera_df['timestamp'].to_numpy() / 1e9

# Find corresponding points by finding the closest timestamp
corresponding_points_data = []
corresponding_points_camera = []

for i in range(len(data_timestamps)):
    time_diffs = np.abs(camera_timestamps - data_timestamps[i])
    closest_index = np.argmin(time_diffs)

    # Only consider matches within a small time threshold (e.g., 0.05 seconds)
    if time_diffs[closest_index] < 0.05:
        corresponding_points_data.append(data_df.iloc[i][['p_RS_R_x [m]', 'p_RS_R_y [m]', 'p_RS_R_z [m]']].to_numpy())
        corresponding_points_camera.append(camera_df.iloc[closest_index][['tx', 'ty', 'tz']].to_numpy())

corresponding_points_data = np.array(corresponding_points_data)
corresponding_points_camera = np.array(corresponding_points_camera)

# Calculate Euclidean distances
if len(corresponding_points_data) > 0 and len(corresponding_points_camera) > 0:
    distances = np.linalg.norm(corresponding_points_data - corresponding_points_camera, axis=1)

    # Measures of matching
    mae = np.mean(distances)
    rmse = np.sqrt(np.mean(distances**2))

    print(f"Mean Absolute Error (MAE): {mae:.4f} meters")
    print(f"Root Mean Squared Error (RMSE): {rmse:.4f} meters")
else:
    print("No corresponding points found within the time threshold.")