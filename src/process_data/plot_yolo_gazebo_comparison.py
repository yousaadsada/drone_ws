import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_jackal_tracking(data_dir, csv_file):
    # Load CSV
    df = pd.read_csv(csv_file)
    t = df["elapsed_time"]

    # Plot positions
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # X position
    axs[0].plot(t, df["jackal_x_gazebo"], label="Gazebo", linewidth=2)
    axs[0].plot(t, df["jackal_x_yolo_filted"], label="YOLO Filtered", linestyle="--")
    axs[0].plot(t, df["jackal_x_yolo_raw"], label="YOLO Raw", linestyle=":")
    axs[0].set_ylabel("X [m]")
    axs[0].legend()
    axs[0].grid(True)

    # Y position
    axs[1].plot(t, df["jackal_y_gazebo"], label="Gazebo", linewidth=2)
    axs[1].plot(t, df["jackal_y_yolo_filted"], label="YOLO Filtered", linestyle="--")
    axs[1].plot(t, df["jackal_y_yolo_raw"], label="YOLO Raw", linestyle=":")
    axs[1].set_ylabel("Y [m]")
    axs[1].legend()
    axs[1].grid(True)

    # Z position
    axs[2].plot(t, df["jackal_z_gazebo"], label="Gazebo", linewidth=2)
    axs[2].plot(t, df["jackal_z_yolo_filted"], label="YOLO Filtered", linestyle="--")
    axs[2].plot(t, df["jackal_z_yolo_raw"], label="YOLO Raw", linestyle=":")
    axs[2].set_ylabel("Z [m]")
    axs[2].set_xlabel("Time [s]")
    axs[2].legend()
    axs[2].grid(True)

    plt.suptitle("Jackal Tracking: Gazebo vs YOLO Filtered vs YOLO Raw")
    plt.tight_layout()
    plt.savefig(os.path.join(data_dir, 'yolo_gazebo_pos_est.png'))
    plt.show()

    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # X speed
    axs[0].plot(t, df["jackal_x_speed_gazebo"], label="Gazebo", linewidth=2)
    axs[0].plot(t, df["jackal_x_speed_yolo_filted"], label="YOLO Filtered", linestyle="--")
    axs[0].plot(t, df["jackal_x_speed_yolo_raw"], label="YOLO Raw", linestyle=":")
    axs[0].set_ylabel("Vx [m/s]")
    axs[0].legend()
    axs[0].grid(True)

    # Y speed
    axs[1].plot(t, df["jackal_y_speed_gazebo"], label="Gazebo", linewidth=2)
    axs[1].plot(t, df["jackal_y_speed_yolo_filted"], label="YOLO Filtered", linestyle="--")
    axs[1].plot(t, df["jackal_y_speed_yolo_raw"], label="YOLO Raw", linestyle=":")
    axs[1].set_ylabel("Vy [m/s]")
    axs[1].legend()
    axs[1].grid(True)

    # Z speed
    axs[2].plot(t, df["jackal_z_speed_gazebo"], label="Gazebo", linewidth=2)
    axs[2].plot(t, df["jackal_z_speed_yolo_filted"], label="YOLO Filtered", linestyle="--")
    axs[2].plot(t, df["jackal_z_speed_yolo_raw"], label="YOLO Raw", linestyle=":")
    axs[2].set_ylabel("Vz [m/s]")
    axs[2].set_xlabel("Time [s]")
    axs[2].legend()
    axs[2].grid(True)

    plt.suptitle("Jackal Speed: Gazebo vs YOLO Filtered vs YOLO Raw")
    plt.tight_layout()
    plt.savefig(os.path.join(data_dir, 'yolo_gazebo_speed_est.png'))
    plt.show()


if __name__ == "__main__":
    data_dir = os.path.join(os.path.expanduser("~"), 'drone_ws', 'data', 'yolo_gazebo_pos_est')

    read_file = os.path.join(data_dir, "yolo_gazebo_pos_est.csv")

    plot_jackal_tracking(data_dir, read_file)