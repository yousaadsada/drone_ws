
import pandas as pd
import numpy as np
from sklearn.linear_model import LinearRegression
import matplotlib
matplotlib.use('Agg')  # Use the Agg backend for non-GUI environments
import matplotlib.pyplot as plt
import csv
import os

# Load CSV file

data_dir = os.path.join(os.path.expanduser("~"), 'drone_ws', 'data', 'drone_state_function')
save_dir = os.path.join(data_dir, "figures")
os.makedirs(save_dir, exist_ok=True)
read_file_roll = os.path.join(data_dir, "state_data_y.csv")
read_file_pitch = os.path.join(data_dir, "state_data_x.csv")
read_file_yaw = os.path.join(data_dir, "state_data_yaw.csv")
read_file_z = os.path.join(data_dir, "state_data_z.csv")
save_file = os.path.join(data_dir, "state_function_matrix","state_function_matrix.csv")

data = pd.read_csv(read_file_roll)

# Extract columns
timestamps = data['elapsed_time'].values
positions = data['roll'].values
inputs = data['control_y'].values

# Prepare training data
v_k = positions[:-1]  # v(k)
v_k1 = positions[1:]   # v(k+1)
u_k = inputs[:-1]       # u(k)

a = None
b = None

with open(save_file, mode="r") as file:
    reader = csv.DictReader(file)
    for row in reader:
        if row["Model"] == "Roll":
            a = float(row["a"])
            b = float(row["b"])
            break  # no need to continue once "Roll" is found

# Estimate all velocities using trained a and b
estimated_velocities = np.zeros_like(positions)
estimated_velocities[0] = positions[0]  # Initial velocity

# Calculate estimated velocities iteratively
for i in range(1, len(positions)):
    estimated_velocities[i] = a * estimated_velocities[i - 1] + b * inputs[i - 1]

# Calculate Mean Squared Error (MSE)
mse = np.mean((positions - estimated_velocities) ** 2)
print(f"Mean Squared Error (MSE): {mse:.4f}")

# Plot real vs estimated velocities
plt.figure(figsize=(10, 6))
plt.plot(timestamps, positions, label='Measured by Sensors', linestyle='-', marker='o')
plt.plot(timestamps, estimated_velocities, label='Estimated by System State', linestyle='--', marker='x')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
# plt.title(f'Real vs Estimated Velocity\nMSE = {mse:.4f}')
plt.legend()
plt.grid(False)

plt.show()

# Save the plot as an image file
save_fig_path = os.path.join(save_dir, "roll.png")
plt.savefig(save_fig_path)




data = pd.read_csv(read_file_pitch)

# Extract columns
timestamps = data['elapsed_time'].values
positions = data['pitch'].values
inputs = data['control_x'].values

# Prepare training data
v_k = positions[:-1]  # v(k)
v_k1 = positions[1:]   # v(k+1)
u_k = inputs[:-1]       # u(k)

a = None
b = None

with open(save_file, mode="r") as file:
    reader = csv.DictReader(file)
    for row in reader:
        if row["Model"] == "Pitch":
            a = float(row["a"])
            b = float(row["b"])
            break  # no need to continue once "Roll" is found

# Estimate all velocities using trained a and b
estimated_velocities = np.zeros_like(positions)
estimated_velocities[0] = positions[0]  # Initial velocity

# Calculate estimated velocities iteratively
for i in range(1, len(positions)):
    estimated_velocities[i] = a * estimated_velocities[i - 1] + b * inputs[i - 1]

# Calculate Mean Squared Error (MSE)
mse = np.mean((positions - estimated_velocities) ** 2)
print(f"Mean Squared Error (MSE): {mse:.4f}")

# Plot real vs estimated velocities
plt.figure(figsize=(10, 6))
plt.plot(timestamps, positions, label='Measured by Sensors', linestyle='-', marker='o')
plt.plot(timestamps, estimated_velocities, label='Estimated by System State', linestyle='--', marker='x')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
# plt.title(f'Real vs Estimated Velocity\nMSE = {mse:.4f}')
plt.legend()
plt.grid(False)

plt.show()

# Save the plot as an image file
save_fig_path = os.path.join(save_dir, "pitch.png")
plt.savefig(save_fig_path)




data = pd.read_csv(read_file_yaw)

# Extract columns
timestamps = data['elapsed_time'].values
speeds = data['yaw_speed'].values
inputs = data['control_yaw'].values

# Prepare training data
v_k = speeds[:-1]  # v(k)
v_k1 = speeds[1:]   # v(k+1)
u_k = inputs[:-1]       # u(k)

a = None
b = None

with open(save_file, mode="r") as file:
    reader = csv.DictReader(file)
    for row in reader:
        if row["Model"] == "Yaw":
            a = float(row["a"])
            b = float(row["b"])
            break  # no need to continue once "Roll" is found

# Estimate all velocities using trained a and b
estimated_velocities = np.zeros_like(speeds)
estimated_velocities[0] = speeds[0]  # Initial velocity

# Calculate estimated velocities iteratively
for i in range(1, len(speeds)):
    estimated_velocities[i] = a * estimated_velocities[i - 1] + b * inputs[i - 1]

# Calculate Mean Squared Error (MSE)
mse = np.mean((speeds - estimated_velocities) ** 2)
print(f"Mean Squared Error (MSE): {mse:.4f}")

# Plot real vs estimated velocities
plt.figure(figsize=(10, 6))
plt.plot(timestamps, speeds, label='Measured by Sensors', linestyle='-', marker='o')
plt.plot(timestamps, estimated_velocities, label='Estimated by System State', linestyle='--', marker='x')
plt.xlabel('Time (s)')
plt.ylabel('Speed (m)')
# plt.title(f'Real vs Estimated Velocity\nMSE = {mse:.4f}')
plt.legend()
plt.grid(False)

plt.show()

# Save the plot as an image file
save_fig_path = os.path.join(save_dir, "yaw.png")
plt.savefig(save_fig_path)




data = pd.read_csv(read_file_z)

# Extract columns
timestamps = data['elapsed_time'].values
speeds = data['z_speed'].values
inputs = data['control_z'].values

# Prepare training data
v_k = speeds[:-1]  # v(k)
v_k1 = speeds[1:]   # v(k+1)
u_k = inputs[:-1]       # u(k)

a = None
b = None

with open(save_file, mode="r") as file:
    reader = csv.DictReader(file)
    for row in reader:
        if row["Model"] == "Z":
            a = float(row["a"])
            b = float(row["b"])
            break  # no need to continue once "Roll" is found

# Estimate all velocities using trained a and b
estimated_velocities = np.zeros_like(speeds)
estimated_velocities[0] = speeds[0]  # Initial velocity

# Calculate estimated velocities iteratively
for i in range(1, len(speeds)):
    estimated_velocities[i] = a * estimated_velocities[i - 1] + b * inputs[i - 1]

# Calculate Mean Squared Error (MSE)
mse = np.mean((speeds - estimated_velocities) ** 2)
print(f"Mean Squared Error (MSE): {mse:.4f}")

# Plot real vs estimated velocities
plt.figure(figsize=(10, 6))
plt.plot(timestamps, speeds, label='Measured by Sensors', linestyle='-', marker='o')
plt.plot(timestamps, estimated_velocities, label='Estimated by System State', linestyle='--', marker='x')
plt.xlabel('Time (s)')
plt.ylabel('Speed (m)')
# plt.title(f'Real vs Estimated Velocity\nMSE = {mse:.4f}')
plt.legend()
plt.grid(False)

plt.show()

# Save the plot as an image file
save_fig_path = os.path.join(save_dir, "z.png")
plt.savefig(save_fig_path)
