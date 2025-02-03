import rospy
from geometry_msgs.msg import PoseStamped
import rtde_receive
from scipy.spatial.transform import Rotation


"""
	Read TCP poses from the robot and publish them as a ROS topic
"""
def check_pose():
    # Connect to Your robot and specify topic
    rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.129")
    my_pub = rospy.Publisher('/my_ur5e/endeffpos', PoseStamped, queue_size=30)

    r = rospy.Rate(30)      # 30hz - adjusted to unity
    while not rospy.is_shutdown():
        if not rtde_r.isConnected():
            print('RTDE disconnected. Reconnecting...')
            rtde_r.reconnect()
        init_tool_pose = rtde_r.getActualTCPPose()

        # UR RTDE reads angles as the Rodrigues’ rotation vector (Angle-Axis representation)
        rot_rotvec = Rotation.from_rotvec(init_tool_pose[3:6])
        # Convert to quaternions
        rot_quat = rot_rotvec.as_quat()

        eff_pose = PoseStamped()
        eff_pose.header.stamp = rospy.Time.now()
        eff_pose.pose.position.x = init_tool_pose[0]
        eff_pose.pose.position.y = init_tool_pose[1]
        eff_pose.pose.position.z = init_tool_pose[2]
        eff_pose.pose.orientation.x = rot_quat[0]
        eff_pose.pose.orientation.y = rot_quat[1]
        eff_pose.pose.orientation.z = rot_quat[2]
        eff_pose.pose.orientation.w = rot_quat[3]

        my_pub.publish(eff_pose)
        r.sleep()


def main():
    rospy.init_node('eff_pose', anonymous=True)
    rospy.Time.now()
    try:
        check_pose()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()
