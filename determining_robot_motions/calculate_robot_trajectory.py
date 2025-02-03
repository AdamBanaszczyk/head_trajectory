import numpy as np
import rosbag
from scipy.spatial.transform import Rotation
import geometry_msgs.msg
import copy


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


def pose2mat(position, quat_rot):
    quaternion_rotation = Rotation.from_quat(quat_rot)
    quaternion_rotation = quaternion_rotation.as_matrix()
    transform_matrix = np.identity(4)
    transform_matrix[:3, 3] = position
    transform_matrix[:3, :3] = quaternion_rotation
    return transform_matrix


def mat2pose(mat):
    position = mat[:3, 3]
    rotation_matrix = mat[:3, :3]
    quat_rotation = Rotation.from_matrix(rotation_matrix)
    quat_rotation = quat_rotation.as_quat()
    return position, quat_rotation


def main():
    head_traj_pth = 'data/real_world_head_trajectories/'
    # Bag recorded in Space Pirate Trainer DX, slightly shorter and less complicated, as it's only here as an example
    bag_name = 'real_world_head_movement_example_0.bag'
    bag_pth = head_traj_pth + bag_name
    topic_name = '/optitrack_node/rigid_body_0'

    # OptiTrack's frequency = 120Hz. Too much for smooth reproduction on the robot, hence divider
    dilute_param = 10   # Adjust to data

    # Transformation calculated with Hand-Eye Calibration
    # As described in paper A represents the transformation between
    # the TCP and OptiTrack’s Rigid Body corresponding to the VR headset
    # It can be calculated by a dot product of tcp2unity and unity2opti transformations
    A = np.array([
        [-0.999996, 0.000866265, -0.00261548, -0.0107378],
        [-0.00261606, -0.000663386, 0.999996, 0.134303],
        [0.000864527, 0.999999, 0.00066565, 0.342898],
        [0, 0, 0, 1]])


    # Move robot to initial pose - defined by [p, q] - [position, rotation]
    p = [0.4, 0.25, 0.55]  # Example of what worked for UR5e
    q = [0, 0, -0.3826834, 0.9238795]  # -45deg rotation in Z-axis. Example of what worked for UR5e

    # Calculate robot's pose for given init
    B = pose2mat(p, q)

    delta_H_0 = A @ np.identity(4) @ np.linalg.inv(A)
    B_0 = B @ delta_H_0

    position = B_0[:3, 3]
    rotation_matrix = B_0[:3, :3]
    quat_rotation = Rotation.from_matrix(rotation_matrix)
    quat_rotation = quat_rotation.as_quat()

    # Transform geometry_msgs for convenience
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = position[0]
    pose_goal.position.y = position[1]
    pose_goal.position.z = position[2]
    pose_goal.orientation.x = quat_rotation[0]
    pose_goal.orientation.y = quat_rotation[1]
    pose_goal.orientation.z = quat_rotation[2]
    pose_goal.orientation.w = quat_rotation[3]

    print('1) Move robot to initial pose:\n', pose_goal)

    # Read real-world head trajectory rosbag data
    (t_opti,
     x_opti, y_opti, z_opti,
     qx_opti, qy_opti, qz_opti, qw_opti) = read_rosbag_topic(bag_pth, topic_name)

    # Calculate robot's tool poses based on a loaded real-world head trajectory and A (tcp2opti) transformation
    final_matrix_positions, final_matrix_quat_rot = [], []
    for idx, (x, y, z, qx, qy, qz, qw) in enumerate(zip(x_opti, y_opti, z_opti, qx_opti, qy_opti, qz_opti, qw_opti)):
        if idx % dilute_param == 0:
            pose = [x, y, z, qx, qy, qz, qw]
            curr_position = pose[:3]
            curr_orientation = pose[3:]

            O_id = pose2mat(curr_position, curr_orientation)

            # Initial increment
            if idx == 0:
                O_0_inv = np.linalg.inv(O_id)

            delta_O_id = O_0_inv @ O_id
            delta_H_id = A @ delta_O_id @ np.linalg.inv(A)
            delta_B_id = B @ delta_H_id

            position, quat_rotation = mat2pose(delta_B_id)

            final_matrix_positions.append(position)
            final_matrix_quat_rot.append(quat_rotation)

    # Transform lists to geometry_msgs for convenience
    waypoints = []
    wpose = geometry_msgs.msg.Pose()
    for pos, q in zip(final_matrix_positions, final_matrix_quat_rot):
        wpose.position.x = pos[0]
        wpose.position.y = pos[1]
        wpose.position.z = pos[2]
        wpose.orientation.x = q[0]
        wpose.orientation.y = q[1]
        wpose.orientation.z = q[2]
        wpose.orientation.w = q[3]
        waypoints.append(copy.deepcopy(wpose))

    print('2) Follow waypoints in simulation and check if robot can complete such movement.\n  If so - save robot path')

    # TODO:
    #  Find the optimal initial_pose [p, q] and verify in simulation
    #  whether your robot can execute the entire calculated trajectory.

    """
    - Robot and Framework Used:
        We used a UR5e robot simulated in Gazebo, controlled via the MoveIt motion planning framework.
        Instructions for setting up this environment can be found here:
        https://github.com/ros-industrial/universal_robot
    
    - Process to Find Optimal [p, q]:
        Use Rviz and your MoveIt configuration to identify an initial pose [p, q].
        Verify that the entire trajectory (with waypoints) can be executed by the robot.
        
    - Path Execution Check:
        We used the "move_group.compute_cartesian_path()" function to ensure the calculated path is feasible.
        Example usage can be found in the following references:
        http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
        https://github.com/moveit/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
        
        
        In our case, the following code was used to obtain final robot path (as a set of poses) and save it to .pickle:
        
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01 * 3, 0.0  # waypoints to follow  # eef_step	# jump_threshold
        )  
        # Save robot trajectory if fraction == 1 - all poses are reachable
        if fraction == 1.0:
            with open(save_pth + bag_name + '_positions.pickle', 'wb') as fp:
                pickle.dump(plan, fp)
    """




if __name__ == "__main__":
    main()
