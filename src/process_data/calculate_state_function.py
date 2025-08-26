import numpy as np
import csv
import os
from sklearn.linear_model import LinearRegression
import pandas as pd



def save_matrices(a_roll, b_roll, a_pitch, b_pitch, a_yaw, b_yaw, a_z, b_z, csv_file):

    with open(csv_file, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Model", "a", "b"])
        writer.writerow(["Roll", a_roll, b_roll])
        writer.writerow(["Pitch", a_pitch, b_pitch])
        writer.writerow(["Yaw", a_yaw, b_yaw])
        writer.writerow(["Z", a_z, b_z])

data_dir = os.path.join(os.path.expanduser("~"), 'drone_ws', 'data', 'drone_state_function')

read_file_roll = os.path.join(data_dir, "state_data_y.csv")
read_file_pitch = os.path.join(data_dir, "state_data_x.csv")
read_file_yaw = os.path.join(data_dir, "state_data_yaw.csv")
read_file_z = os.path.join(data_dir, "state_data_z.csv")
save_file_path = os.path.join(data_dir, "state_function_matrix")
os.makedirs(save_file_path, exist_ok=True)
save_file = os.path.join(save_file_path, "state_function_matrix.csv")

data = pd.read_csv(read_file_roll)

timestamps = data['elapsed_time'].values
positions = data['roll'].values
inputs = data['control_y'].values

# Prepare training data
x_k = positions[:-1]  # v(k)
x_k1 = positions[1:]   # v(k+1)
u_k = inputs[:-1]       # u(k)

# Prepare X (features) and y (target)
X = np.vstack((x_k, u_k)).T  # Features: [v(k), u(k)]
y = x_k1                     # Target: v(k+1)

# Train model to find a and b
model = LinearRegression()
model.fit(X, y)

# Extract a and b
a_roll, b_roll = model.coef_




data = pd.read_csv(read_file_pitch)

timestamps = data['elapsed_time'].values
positions = data['pitch'].values
inputs = data['control_x'].values

# Prepare training data
x_k = positions[:-1]  # v(k)
x_k1 = positions[1:]   # v(k+1)
u_k = inputs[:-1]       # u(k)

# Prepare X (features) and y (target)
X = np.vstack((x_k, u_k)).T  # Features: [v(k), u(k)]
y = x_k1                     # Target: v(k+1)

# Train model to find a and b
model = LinearRegression()
model.fit(X, y)

# Extract a and b
a_pitch, b_pitch = model.coef_




data = pd.read_csv(read_file_yaw)

timestamps = data['elapsed_time'].values
positions = data['yaw_speed'].values
inputs = data['control_yaw'].values

# Prepare training data
x_k = positions[:-1]  # v(k)
x_k1 = positions[1:]   # v(k+1)
u_k = inputs[:-1]       # u(k)

# Prepare X (features) and y (target)
X = np.vstack((x_k, u_k)).T  # Features: [v(k), u(k)]
y = x_k1                     # Target: v(k+1)

# Train model to find a and b
model = LinearRegression()
model.fit(X, y)

# Extract a and b
a_yaw, b_yaw = model.coef_




data = pd.read_csv(read_file_z)

timestamps = data['elapsed_time'].values
positions = data['z_speed'].values
inputs = data['control_z'].values

# Prepare training data
x_k = positions[:-1]  # v(k)
x_k1 = positions[1:]   # v(k+1)
u_k = inputs[:-1]       # u(k)

# Prepare X (features) and y (target)
X = np.vstack((x_k, u_k)).T  # Features: [v(k), u(k)]
y = x_k1                     # Target: v(k+1)

# Train model to find a and b
model = LinearRegression()
model.fit(X, y)

# Extract a and b
a_z, b_z = model.coef_



save_matrices(a_roll, b_roll, a_pitch, b_pitch, a_yaw, b_yaw, a_z, b_z, save_file)