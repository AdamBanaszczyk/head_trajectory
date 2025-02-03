import numpy as np
import os
from datetime import datetime
import matplotlib.pyplot as plt
import csv
from matplotlib.dates import date2num, DateFormatter
import rosbag


def read_rosbag_topic(bag_pth, topic_name):
    # Lists to store the extracted values
    x, y, z = [], [], []
    qx, qy, qz, qw = [], [], [], []
    t_timestamp = []

    # Open the bag file and read messages
    with rosbag.Bag(bag_pth, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            # Access timestamps
            curr_time = t.to_sec()
            t_timestamp.append(curr_time)

            # Access position and orientation data
            x.append(msg.pose.position.x)
            y.append(msg.pose.position.y)
            z.append(msg.pose.position.z)
            qx.append(msg.pose.orientation.x)
            qy.append(msg.pose.orientation.y)
            qz.append(msg.pose.orientation.z)
            qw.append(msg.pose.orientation.w)

    return t_timestamp, x, y, z, qx, qy, qz, qw


def read_unity_log_data(log_path):
    # Don't read last line - could be corrupted due to application shutdown
    data = np.loadtxt(open(log_path).readlines()[:-1], delimiter=",")

    # Extract relevant columns and apply adjustments
    year, month, day = data[:, 2:5].astype(int).T
    hour = data[:, 5].astype(int)
    minute, second, milisecond = data[:, 6:9].astype(int).T

    # Adjust hour based on timezone. In our case it was GMT1 (+1)
    date_stamp_vec = []
    for i in range(len(hour)):
        date_stamp = datetime(year[i], month[i], day[i], hour[i] + 1, minute[i], second[i], milisecond[i] * 1_000)
        date_stamp_vec.append(date_stamp)

    # Position and orientation data
    x, y, z = data[:, 9:12].T
    qx, qy, qz, qw = data[:, 12:16].T

    return date_stamp_vec, x, y, z, qx, qy, qz, qw


def save_poses_to_csv(out_file, column_names, pose_data):
    with open(out_file, 'w') as myfile:
        wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
        wr.writerow(column_names)
        for w in range(len(pose_data[0])):
            wr.writerow(
                [pose_data[0][w], pose_data[1][w], pose_data[2][w], pose_data[3][w],
                 pose_data[4][w], pose_data[5][w], pose_data[6][w], pose_data[7][w]])
        print(f'Successfully saved poses to {out_file}')


def main():
    fname = 'calib'
    t_offset = -0.27
    dir_pth = 'data/experiments/quest2/'

    topic_name = '/my_ur5e/endeffpos'
    bag_pth = dir_pth + 'tcp_bags/' + fname + '.bag'
    unity_log_pth = dir_pth + 'center_eye_data/' + fname + '.txt'


    # Read Headset Unity data
    (t_unity_GMT2,
     x_unity, y_unity, z_unity,
     qx_unity, qy_unity, qz_unity, qw_unity) = read_unity_log_data(unity_log_pth)

    # Read TCP rosbag data
    (t_tool,
     x_tool, y_tool, z_tool,
     qx_tool, qy_tool, qz_tool, qw_tool) = read_rosbag_topic(bag_pth, topic_name)


    # Save Unity and Tool timestamps as increments from 1st Unity entry. Needed for Hand-Eye calibration
    # Time shift calculated in compare_traj_peaks is not included here
    timestamp_unity_GMT2_normalized = [entry.timestamp() - t_unity_GMT2[0].timestamp() for entry in t_unity_GMT2]
    t_tool_unity_norm = [stamp - t_unity_GMT2[0].timestamp() for stamp in t_tool]

    # Visualization
    # Define axis to check peaks
    ax_tool = np.array(z_tool)    # Tool is Right-Hand System with Z-up
    ax_unity = y_unity            # Unity is Left-Hand with Y-up

    # Normalize chosen axis around starting coordinate
    ax_tool = [np.array(ax_t - ax_tool[0]) for ax_t in ax_tool]
    ax_unity = [np.array(ax_u - ax_unity[0]) for ax_u in ax_unity]

    TA = list([entry.timestamp() for entry in t_unity_GMT2])
    TA_shifted = list([(entry.timestamp() - t_offset) for entry in t_unity_GMT2])
    TB = list(t_tool)
    A = list(ax_unity)
    B = list(ax_tool)
    TA = [date2num(datetime.fromtimestamp(x)) for x in TA]
    TA_shifted = [date2num(datetime.fromtimestamp(x)) for x in TA_shifted]
    TB = [date2num(datetime.fromtimestamp(x)) for x in TB]

    # Create a 1x2 grid of subplots
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))  # Adjust figsize as needed

    # Plot in the first subplot
    axes[0].plot_date(TA, A, 'b', label='Unity')
    axes[0].plot_date(TB, B, 'g', label='Tool')
    axes[0].set_title('Pose matches without time shift (zoom peaks)')
    axes[0].xaxis.set_major_formatter(DateFormatter('%H:%M:%S'))
    axes[0].legend()

    # Plot in the second subplot
    axes[1].plot_date(TA_shifted, A, 'b', label='Unity')
    axes[1].plot_date(TB, B, 'g', label='Tool')
    axes[1].set_title('Pose matches with time shift compensated (zoom peaks)')
    axes[1].xaxis.set_major_formatter(DateFormatter('%H:%M:%S'))
    axes[1].legend()

    # Adjust layout and show the plots
    plt.tight_layout()
    plt.show()

    # save dir
    out_csv_pth = dir_pth + 'hand_eye_calib_input_data/'
    if not os.path.exists(out_csv_pth):
        os.makedirs(out_csv_pth)

    # Save TCP data to .csv
    name_TOOL = out_csv_pth + fname + '_tool.csv'
    column_name_TOOL_trans = ['t_tool', 'x_tool', 'y_tool', 'z_tool', 'qx_tool', 'qy_tool', 'qz_tool', 'qw_tool']
    data_TOOL = [t_tool_unity_norm, x_tool, y_tool, z_tool, qx_tool, qy_tool, qz_tool, qw_tool]
    save_poses_to_csv(name_TOOL, column_name_TOOL_trans, data_TOOL)

    # Save Unity data to .csv
    name_unity = out_csv_pth + fname + '_unity.csv'
    column_name_unity = ['t_unity', 'x_unity', 'y_unity', 'z_unity', 'qx_unity', 'qy_unity', 'qz_unity', 'qw_unity']
    data_unity = [timestamp_unity_GMT2_normalized, x_unity, y_unity, z_unity, qx_unity, qy_unity, qz_unity, qw_unity]
    save_poses_to_csv(name_unity, column_name_unity, data_unity)


if __name__ == "__main__":
    main()
