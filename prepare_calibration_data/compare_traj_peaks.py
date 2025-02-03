import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import numpy as np
from datetime import datetime
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


def main():
    fname = 'calib'
    dir_pth = 'data/experiments/quest2/'

    topic_name = '/my_ur5e/endeffpos'
    bag_pth = dir_pth + 'tcp_bags/'+ fname + '.bag'
    unity_log_pth = dir_pth + 'center_eye_data/' + fname + '.txt'

    # Read TCP rosbag data
    (t_tool_GMT2,
     x_tool, y_tool, z_tool,
     qx_tool, qy_tool, qz_tool, qw_tool) = read_rosbag_topic(bag_pth, topic_name)

    # Read Headset Unity data
    (t_unity_GMT2,
     x_unity, y_unity, z_unity,
     qx_unity, qy_unity, qz_unity, qw_unity) = read_unity_log_data(unity_log_pth)

    # Define axis to check peaks
    # Adjust those axes according to data
    ax_tool = -np.array(z_tool)    # Tool is Right-Hand System with Z-up
    ax_unity = -y_unity            # Unity is Left-Hand with Y-up

    # Normalize chosen axis around starting coordinate
    ax_tool = [ax_o - ax_tool[0] for ax_o in ax_tool]
    ax_unity = [ax_u - ax_unity[0] for ax_u in ax_unity]
    ax_tool = np.array(ax_tool)
    ax_unity = np.array(ax_unity)

    # h1, h2 - minimal heights of peaks
    # Adjust according to data
    h1 = 0.09
    h2 = 0.09
    # distance - minimal distance between following peaks
    # Adjust according to data
    peaks_tool, _ = find_peaks(ax_tool, height=h1, distance=200)
    peaks_unity, _ = find_peaks(ax_unity, height=h2, distance=500)

    for idx, (p_o, p_u) in enumerate(zip(peaks_tool, peaks_unity)):
        print(f'peak({idx}): unity id:', p_u, 'tool id:', p_o)

    # plot peaks -> check for correct matches
    plt.subplot(2, 1, 1)
    plt.plot(ax_tool)
    plt.plot(peaks_tool, ax_tool[peaks_tool], "x", c='r')
    plt.plot(np.zeros_like(ax_tool), "--", color="gray")
    plt.title('Traj peaks')
    plt.ylabel('Tool')
    plt.subplot(2, 1, 2)
    plt.plot(ax_unity)
    plt.plot(peaks_unity, ax_unity[peaks_unity], "x", c='r')
    plt.plot(np.zeros_like(ax_unity), "--", color="gray")
    plt.xlabel('samples')
    plt.ylabel('Unity')
    plt.show()

    assert len(peaks_unity) == len(peaks_tool), 'Unequal number of peaks. Adjust params'
    print('Equal number of peaks. Proceeding...')

    global_time_diff_vec = []
    for p_u, p_o in zip(peaks_unity, peaks_tool):
        global_time_diff_vec.append(t_unity_GMT2[p_u].timestamp() - t_tool_GMT2[p_o])
    print('global_time_diff_vec:', global_time_diff_vec)
    global_mean_time_diff = np.mean(global_time_diff_vec)
    print('global_mean_time_diff:', global_mean_time_diff)


if __name__ == "__main__":
    main()
