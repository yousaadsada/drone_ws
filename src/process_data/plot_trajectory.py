# plot_trajectory.py
import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_trajectory(data_dir, csv_file):
    # Load CSV
    df = pd.read_csv(csv_file)

    time = df["elapsed_time"]

    # Plot positions (x, y, z, yaw)
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    axs[0].plot(time, df["ref_x"], label="ref_x", linestyle="--")
    axs[0].plot(time, df["current_x"], label="current_x")
    axs[0].set_ylabel("X [m]")
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(time, df["ref_y"], label="ref_y", linestyle="--")
    axs[1].plot(time, df["current_y"], label="current_y")
    axs[1].set_ylabel("Y [m]")
    axs[1].legend()
    axs[1].grid(True)

    axs[2].plot(time, df["ref_z"], label="ref_z", linestyle="--")
    axs[2].plot(time, df["current_z"], label="current_z")
    axs[2].set_ylabel("Z [m]")
    axs[2].legend()
    axs[2].grid(True)

    axs[3].plot(time, df["ref_yaw"], label="ref_yaw", linestyle="--")
    axs[3].plot(time, df["current_yaw"], label="current_yaw")
    axs[3].set_ylabel("Yaw [rad]")
    axs[3].set_xlabel("Time [s]")
    axs[3].legend()
    axs[3].grid(True)

    plt.suptitle("Reference vs Current Trajectory")
    plt.tight_layout()
    plt.savefig(os.path.join(data_dir, 'drone_trajectory_2.0_2.0.png'))
    plt.show()


if __name__ == "__main__":
    data_dir = os.path.join(os.path.expanduser("~"), 'drone_ws', 'data', 'tracking_trajectory')

    read_file = os.path.join(data_dir, "state_data_2.0_2.0.csv")

    plot_trajectory(data_dir, read_file)
