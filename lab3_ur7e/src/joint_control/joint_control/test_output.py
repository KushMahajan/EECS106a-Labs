import rclpy
from array import array

def compare_joint_trajectory(node, traj):
    logger = node.get_logger()

    if traj.joint_names != ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']:
        raise Exception("Joint names do not match expected joint names")
    elif traj.points[0].positions != array('d', [4.1768, -2.2087, -1.2924, -1.1133, 1.4865, -2.846]):
        logger.warn("Joint positions do not match test case given in lab document")
    else:
        logger.info("Joint Positions match test case")