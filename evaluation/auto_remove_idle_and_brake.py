import matplotlib.pyplot as plt
import numpy as np


def read_txt_data(file_path):
    data = []  # Temporary list to hold rows of data
    with open(file_path, "r") as file:
        for line in file:
            # Split the line into individual components and convert to float
            values = list(map(float, line.split()))
            data.append(values)

    return np.array(data)


def filter_data_by_time(data_array, beg_time, end_time):
    # Apply the time range filter
    filtered_data = data_array[(data_array[:, 0] >= beg_time) & (data_array[:, 0] <= end_time)]
    return filtered_data


def plot_data_vs_filtered(base_data, filtered_data, choosen_axis):
    # Create a 1x2 grid of subplots
    fig, ax = plt.subplots(2, 1, figsize=(12, 6))  # Adjust figsize as needed

    # Extract all axis values
    values = base_data[:, choosen_axis]
    # Generate indices for the axis
    indices = range(len(values))
    ax[0].plot(indices, values, color='red')
    ax[0].set_title('Without filtering')

    # Extract all axis values
    values = filtered_data[:, choosen_axis]
    # Generate indices for the axis
    indices = range(len(values))
    ax[1].plot(indices, values, color='blue')
    ax[1].set_title('Idle and brake filtered')

    plt.tight_layout()
    plt.show()


def save_data_to_txt(pth, data_array):
    with open(pth, 'w') as f:
        for row in data_array:
            # Join the row values with space and write to the file
            f.write(' '.join(map(str, row)) + '\n')


def main():
    # Data directory
    txt_path = 'data/experiments/linked_tum/'
    # Define trajectory name
    fname = 'traj1'

    # Transformed data names
    gt1 = fname + '_tool1_gt'
    gt2 = fname + '_tool2_gt'
    h1 = fname + '_quest2_to_tool1'
    h2 = fname + '_quest_pro_to_tool1'

    # Read all txt data
    gt1_data = read_txt_data(txt_path + gt1 + '.txt')
    gt2_data = read_txt_data(txt_path + gt2 + '.txt')
    h1_data = read_txt_data(txt_path + h1 + '.txt')
    h2_data = read_txt_data(txt_path + h2 + '.txt')

    # Automatic idle detection
    examined_data = gt1_data
    init_position = np.array([examined_data[0][1], examined_data[0][2], examined_data[0][3]])
    for i in range(len(examined_data) - 1):
        dist_from_init_pose = np.linalg.norm(np.array([examined_data[i][1], examined_data[i][2], examined_data[i][3]]) - init_position)
        if dist_from_init_pose >= 0.005:
            # Add 1 second buffer to initial movement timestamp
            time_idle = examined_data[i][0] - 1.0
            break

    # Automatic brake detection
    end_position = np.array([examined_data[-1][1], examined_data[-1][2], examined_data[-1][3]])
    for i in reversed(range(len(examined_data))):
        dist_from_end_pose = np.linalg.norm(np.array([examined_data[i][1], examined_data[i][2], examined_data[i][3]]) - end_position)
        if dist_from_end_pose >= 0.005:
            # Add 1 second buffer to brake movement timestamp
            time_brake = examined_data[i][0] - 1.0
            break

    # Filter the data
    filtered_gt1_data = filter_data_by_time(gt1_data, time_idle, time_brake)
    filtered_gt2_data = filter_data_by_time(gt2_data, time_idle, time_brake)
    filtered_h1_data = filter_data_by_time(h1_data, time_idle, time_brake)
    filtered_h2_data = filter_data_by_time(h2_data, time_idle, time_brake)

    # Plot data to see the difference in removing idle and brake parts
    plot_data_vs_filtered(base_data=h1_data, filtered_data=filtered_h1_data, choosen_axis=3)

    # Save filtered data
    save_data_to_txt(txt_path + gt1 + '_cut.txt', filtered_gt1_data)
    save_data_to_txt(txt_path + gt2 + '_cut.txt', filtered_gt2_data)
    save_data_to_txt(txt_path + h1 + '_cut.txt', filtered_h1_data)
    save_data_to_txt(txt_path + h2 + '_cut.txt', filtered_h2_data)


if __name__ == "__main__":
    main()