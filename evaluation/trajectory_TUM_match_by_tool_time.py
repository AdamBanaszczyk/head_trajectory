import matplotlib.pyplot as plt
from scipy.signal import find_peaks
import numpy as np
from datetime import datetime
from scipy.spatial.transform import Rotation
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
        date_stamp_vec.append(date_stamp.timestamp())

    # Position and orientation data
    x, y, z = data[:, 9:12].T
    qx, qy, qz, qw = data[:, 12:16].T

    return date_stamp_vec, x, y, z, qx, qy, qz, qw


def get_init_transformation(x, y, z, qx, qy, qz, qw):
    init_trans_matrix = np.eye(4)
    init_rotation = Rotation.from_quat([qx, qy, qz, qw])
    init_trans_matrix[:3, :3] = init_rotation.as_matrix()
    init_position = [x, y, z]
    init_trans_matrix[:3, 3] = init_position
    return init_trans_matrix


def normalize_poses(init_transform, x, y, z, qx, qy, qz, qw):
    x_norm, y_norm, z_norm, qx_norm, qy_norm, qz_norm, qw_norm = [], [], [], [], [], [], []
    for _x, _y, _z, _qx, _qy, _qz, _qw in zip(x, y, z, qx, qy, qz, qw):
        point = [_x, _y, _z, _qx, _qy, _qz, _qw]
        transformed_point = transform_pose(point, np.linalg.inv(init_transform))
        x_norm.append(transformed_point[0])
        y_norm.append(transformed_point[1])
        z_norm.append(transformed_point[2])
        qx_norm.append(transformed_point[3])
        qy_norm.append(transformed_point[4])
        qz_norm.append(transformed_point[5])
        qw_norm.append(transformed_point[6])
    return x_norm, y_norm, z_norm, qx_norm, qy_norm, qz_norm, qw_norm


def change_coord_system(transformation, x, y, z, qx, qy, qz, qw):
    x_out, y_out, z_out, qx_out, qy_out, qz_out, qw_out = [], [], [], [], [], [], []
    for _x, _y, _z, _qx, _qy, _qz, _qw in zip(x, y, z, qx, qy, qz, qw):
        # Converting Unity’s trajectory to the RHS by flipping the Y − axis,
        # which requires negating y, qy , qw components.
        point = [-_x, _y, _z, -_qx, _qy, _qz, -_qw]
        transformed_point = transform_pose_XPX_1(point, transformation)
        x_out.append(transformed_point[0])
        y_out.append(transformed_point[1])
        z_out.append(transformed_point[2])
        qx_out.append(transformed_point[3])
        qy_out.append(transformed_point[4])
        qz_out.append(transformed_point[5])
        qw_out.append(transformed_point[6])
    return x_out, y_out, z_out, qx_out, qy_out, qz_out, qw_out


def transform_pose_XPX_1(pose, transformation_matrix):
    # Extract translation and rotation from the pose
    translation = pose[:3]
    rotation = Rotation.from_quat(pose[3:])

    # Create a homogeneous transformation matrix for the pose
    homogeneous_pose = np.eye(4)
    homogeneous_pose[:3, :3] = rotation.as_matrix()
    homogeneous_pose[:3, 3] = translation

    # Apply the provided transformation matrix to the pose
    transformed_pose = np.dot(np.dot(transformation_matrix, homogeneous_pose), np.linalg.inv(transformation_matrix))

    # Extract the new translation/rotation
    new_translation = transformed_pose[:3, 3]
    new_rotation = Rotation.from_matrix(transformed_pose[:3, :3])

    # Combine the new translation and rotation into new pose
    new_pose = np.concatenate([new_translation, new_rotation.as_quat()])

    return new_pose


def transform_pose(pose, transformation_matrix):
    # Extract translation and rotation from the pose
    translation = pose[:3]
    rotation = Rotation.from_quat(pose[3:])

    # Create a homogeneous transformation matrix for the pose
    homogeneous_pose = np.eye(4)
    homogeneous_pose[:3, :3] = rotation.as_matrix()
    homogeneous_pose[:3, 3] = translation

    # Apply the provided transformation matrix to the pose
    transformed_pose = np.dot(transformation_matrix, homogeneous_pose)

    # Extract the new translation/rotation
    new_translation = transformed_pose[:3, 3]
    new_rotation = Rotation.from_matrix(transformed_pose[:3, :3])

    # Combine the new translation and rotation into new pose
    new_pose = np.concatenate([new_translation, new_rotation.as_quat()])

    return new_pose


def save_data_to_txt(pth, t, x, y, z, qx, qy, qz, qw):
    with open(pth, 'w') as f:
        for i in range(len(t)):
            f.write(str(t[i]) + ' ' + str(x[i]) + ' ' + str(y[i]) + ' ' + str(z[i]) + ' '+
                    str(qx[i]) + ' ' + str(qy[i]) + ' ' + str(qz[i]) + ' ' + str(qw[i]) + '\n')


def shift_timestamps(stamps, t_offset):
    t_shifted = []
    for i in range(len(stamps)):
        t_shifted.append(datetime.fromtimestamp(stamps[i]).timestamp() - datetime.fromtimestamp(t_offset).timestamp())
    return t_shifted


def find_global_time_offset(timestamps1, timestamps2, peak_axis1, peak_axis2):
    # Define axis to check peaks
    ax_t1 = np.array(peak_axis1)
    ax_t2 = np.array(peak_axis2)

    # Normalize chosen axis around starting coordinate
    ax_t1 = [ax_o - ax_t1[0] for ax_o in ax_t1]
    ax_t2 = [ax_u - ax_t2[0] for ax_u in ax_t2]
    ax_t1 = np.array(ax_t1)
    ax_t2 = np.array(ax_t2)

    # h1, h2 - minimal heights of peaks. Choose according to data
    h1 = 0.095
    h2 = 0.095
    # distance - minimal distance between following peaks. Choose according to data
    peaks_t1, _ = find_peaks(ax_t1, height=h1, distance=300)
    peaks_t2, _ = find_peaks(ax_t2, height=h2, distance=300)

    # Plot peaks -> check if correct matches
    plt.subplot(2, 1, 1)
    plt.plot(ax_t1)
    plt.plot(peaks_t1, ax_t1[peaks_t1], "x")
    plt.plot(np.zeros_like(ax_t1), "--", color="gray")
    plt.title('Finding trajectory peaks')
    plt.ylabel('Tool 1')
    plt.subplot(2, 1, 2)
    plt.plot(ax_t2)
    plt.plot(peaks_t2, ax_t2[peaks_t2], "x")
    plt.plot(np.zeros_like(ax_t2), "--", color="gray")
    plt.xlabel('samples')
    plt.ylabel('Tool 2')
    plt.show()

    print('Peaks in provided config:', peaks_t1, peaks_t2)
    if len(peaks_t1) == len(peaks_t2):
        print('equal number of peaks. Should be ok')
    else:
        print('!!! unequal number of peaks. Choose better params !!!')
        quit()

    print('Global time diffs:')
    global_time_diff_vec = []
    for idx, (p1, p2) in enumerate(zip(peaks_t1, peaks_t2)):
        t_diff = timestamps2[p2] - timestamps1[p1]
        print(f'peak({idx}): {t_diff}[s]', 'tool1 id:', p1, 'tool2 id:', p2)
        global_time_diff_vec.append(t_diff)

    print('global_time_diff_vec:', global_time_diff_vec)

    global_time_diff = np.mean(global_time_diff_vec)
    print('global_time_diff (between experiments 1 and 2):', global_time_diff)

    return global_time_diff


def extract_data(in_pth, fname, headset, t_diff, tcp2unity):
    dir_pth = in_pth + headset + '/'
    topic_name = '/my_ur5e/endeffpos'

    # Read TCP rosbag data
    (t_tool_GMT2,
     x_tool, y_tool, z_tool,
     qx_tool, qy_tool, qz_tool, qw_tool) = read_rosbag_topic(dir_pth + 'tcp_bags/' + fname + '.bag', topic_name)

    # Get initial TCP transformation for pose normalization
    init_tool_transform = get_init_transformation(x_tool[0], y_tool[0], z_tool[0],
                                                  qx_tool[0], qy_tool[0], qz_tool[0], qw_tool[0])

    # Normalize tool poses
    (x_tool_norm, y_tool_norm, z_tool_norm,
     qx_tool_norm, qy_tool_norm, qz_tool_norm, qw_tool_norm) = normalize_poses(init_tool_transform,
                                                                               x_tool, y_tool, z_tool,
                                                                               qx_tool, qy_tool, qz_tool, qw_tool)

    # Read Headset Unity data
    (t_unity_GMT2,
     x_unity, y_unity, z_unity,
     qx_unity, qy_unity, qz_unity, qw_unity) = read_unity_log_data(dir_pth + 'center_eye_data/' + fname + '.txt')

    # Apply time shift to Unity timestamps
    t_unity_shifted = shift_timestamps(t_unity_GMT2, t_diff)

    # Get initial unity transformation for pose normalization
    init_unity_transform = get_init_transformation(x_unity[0], y_unity[0], z_unity[0],
                                                   qx_unity[0], qy_unity[0], qz_unity[0], qw_unity[0])

    # Normalize unity poses
    (x_unity_norm, y_unity_norm, z_unity_norm,
     qx_unity_norm, qy_unity_norm, qz_unity_norm, qw_unity_norm) = normalize_poses(init_unity_transform,
                                                                                   x_unity,
                                                                                   y_unity,
                                                                                   z_unity,
                                                                                   qx_unity,
                                                                                   qy_unity,
                                                                                   qz_unity,
                                                                                   qw_unity)

    # Normalize unity poses
    (x_unity2tool, y_unity2tool, z_unity2tool,
     qx_unity2tool, qy_unity2tool, qz_unity2tool, qw_unity2tool) = change_coord_system(tcp2unity,
                                                                                       x_unity_norm,
                                                                                       y_unity_norm,
                                                                                       z_unity_norm,
                                                                                       qx_unity_norm,
                                                                                       qy_unity_norm,
                                                                                       qz_unity_norm,
                                                                                       qw_unity_norm)

    # Plot trajectories with time shift
    # Define axis to check peaks
    ax_tool = np.array(z_tool_norm)
    ax_unity = z_unity2tool

    # normalize chosen axis around starting coordinate
    ax_tool = [ax_t - ax_tool[0] for ax_t in ax_tool]
    ax_unity = [ax_u - ax_unity[0] for ax_u in ax_unity]
    ax_tool = np.array(ax_tool)
    ax_unity = np.array(ax_unity)

    TA = list(t_unity_shifted)
    TB = list(t_tool_GMT2)
    A = list(ax_unity)
    B = list(ax_tool)
    TA = [date2num(datetime.fromtimestamp(x)) for x in TA]
    TB = [date2num(datetime.fromtimestamp(x)) for x in TB]

    fig, ax = plt.subplots()
    ax.plot_date(TA, A, 'b', label='Unity 2 Tool')
    ax.plot_date(TB, B, 'g', label='Tool')
    plt.title(f'{fname}, {headset}, applied t_offset: {t_diff}')
    plt.legend()
    ax.xaxis.set_major_formatter(DateFormatter('%H:%M:%S'))
    plt.show()

    return (t_tool_GMT2, x_tool, y_tool, z_tool, qx_tool, qy_tool, qz_tool, qw_tool,
            t_unity_shifted, x_unity2tool, y_unity2tool, z_unity2tool, qx_unity2tool, qy_unity2tool, qz_unity2tool, qw_unity2tool)


def main():
    # Input data directory
    input_pth = 'data/experiments/'
    # Define save path
    out_path = input_pth + 'linked_tum/'

    # Define trajectory name
    fname = 'traj1'

    # Define first headset
    headset1 = 'quest2'  # quest2 / quest_pro
    t_diff_1 = -0.27
    # Hand-Eye Calibration result for quest 2
    tcp2unity_1 = np.array([[-0.999505, -0.0153348, 0.0274682, 0.00268286],
                            [0.0277369, -0.0175974, 0.99946, 0.0867053],
                            [-0.0148431, 0.999728, 0.018014, 0.276278],
                            [0, 0, 0, 1]])

    # Define second headset (experiments should be performed after first headset to keep correct timestamp operations)
    headset2 = 'quest_pro'  # quest2 / quest_pro
    t_diff_2 = 0.0
    # Hand-Eye Calibration result for quest pro
    tcp2unity_2 = np.array([[-0.999487, 0.0211027, 0.0240903, -0.00143335],
                            [0.0228977, -0.0550391, 0.998222, 0.099671],
                            [0.0223911, 0.998261, 0.0545277, 0.248939],
                            [0, 0, 0, 1]])

    # Read headset1 data
    (t_tool_1, x_tool_1, y_tool_1, z_tool_1, qx_tool_1, qy_tool_1, qz_tool_1, qw_tool_1,
     t_unity_shifted, x_ut2t, y_ut2t, z_ut2t, qx_ut2t, qy_ut2t, qz_ut2t, qw_ut2t) = extract_data(input_pth, fname, headset1, t_diff_1, tcp2unity_1)

    # Read headset2 data
    (t_tool_2, x_tool_2, y_tool_2, z_tool_2, qx_tool_2, qy_tool_2, qz_tool_2, qw_tool_2,
     t2_shifted, x2, y2, z2, qx2, qy2, qz2, qw2) = extract_data(input_pth, fname,headset2, t_diff_2, tcp2unity_2)

    # Save tool_1 data (GT)
    save_data_to_txt(out_path + fname + '_tool1_gt.txt',
                     t_tool_1, x_tool_1, y_tool_1, z_tool_1, qx_tool_1, qy_tool_1, qz_tool_1, qw_tool_1)

    # Save headset1 data transformed to robot (tool_1) system
    save_data_to_txt(out_path + fname + '_quest2_to_tool1.txt',
                     t_unity_shifted, x_ut2t, y_ut2t, z_ut2t, qx_ut2t, qy_ut2t, qz_ut2t, qw_ut2t)

    # Find global time offset between experiments on robot
    global_time_diff = find_global_time_offset(t_tool_1, t_tool_2, z_tool_1, z_tool_2)

    # Apply global time shift to tool_2, so it matches timestamps in tool_1
    t_tool_2_global_shifted = shift_timestamps(t_tool_2, global_time_diff)

    # Apply global time shift to unity_2, so it matches timestamps in tool_1 (and unity_1)
    t_unity_2_global_shifted = shift_timestamps(t2_shifted, global_time_diff)

    # Save tool_2 data (GT) with timestamps shifted to tool 1
    save_data_to_txt(out_path + fname + '_tool2_gt.txt',
                     t_tool_2_global_shifted, x_tool_2, y_tool_2, z_tool_2, qx_tool_2, qy_tool_2, qz_tool_2, qw_tool_2)

    # Save headset1 data transformed to robot (tool_1) system
    save_data_to_txt(out_path + fname + '_quest_pro_to_tool1.txt',
                     t_unity_2_global_shifted, x2, y2, z2, qx2, qy2, qz2, qw2)


if __name__ == "__main__":
    main()
