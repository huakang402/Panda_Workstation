# coding: utf-8

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list


def create_pose(pos_x,pos_y,pos_z,ori_w,ori_x,ori_y,ori_z):
    pose = geometry_msgs.msg.Pose()

    pose.position.x = pos_x
    pose.position.y = pos_y
    pose.position.z = pos_z
    pose.orientation.w = ori_w
    pose.orientation.x = ori_x
    pose.orientation.y = ori_y
    pose.orientation.z = ori_z

    return pose

def all_close(goal, actual, tolerance):
    if isinstance(goal, list):
        for index, g in enumerate(goal):
          if abs(actual[index] - g) > tolerance:
            return False

    elif isinstance(goal, geometry_msgs.msg.PoseStamped):
        return all_close(goal.pose, actual.pose, tolerance)

    elif isinstance(goal, geometry_msgs.msg.Pose):
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class PandaMoveit:

    def __init__(self, use_panda_hand_link=False):
        try:
            rospy.get_param("/robot_description_semantic")
        except KeyError:
            rospy.loginfo(("Moveit server does not seem to be running."))
            raise Exception
        except (socket.error, socket.gaierror):
            print ("Failed to connect to the ROS parameter server!\n"
           "Please check to make sure your ROS networking is "
           "properly configured:\n")
            sys.exit()

        moveit_commander.roscpp_initialize(sys.argv)

        self._robot = moveit_commander.RobotCommander()

        self._scene = moveit_commander.PlanningSceneInterface()

        self._arm_group = moveit_commander.MoveGroupCommander("panda_arm")

        self._hand_group = moveit_commander.MoveGroupCommander("hand")

        if use_panda_hand_link and self._hand_group is not None:
            self._default_ee = 'panda_hand'
        else:
            self._default_ee = 'panda_link8'
        self._arm_group.set_end_effector_link(self._default_ee)

        rospy.logdebug("PandaMoveGroup: Setting default EE link to '{}' "
            "Use group.set_end_effector_link() method to change default EE.".format(self._default_ee))

        self._arm_group.set_max_velocity_scaling_factor(0.3)
        self._hand_group.set_max_velocity_scaling_factor(0.3)

    def move_joint(self, j1, j2, j3, j4, j5, j6, j7):
        positions = [j1,j2,j3,j4,j5,j6,j7]
        wait = True
        tolerance = 0.005

        rospy.loginfo("[move_joint] Starting positions: {}".format(self._arm_group.get_current_joint_values()))
        rospy.loginfo("[move_joint] Goal positions: {}".format(positions))
        
        self._arm_group.set_joint_value_target(positions)
        self._arm_group.go(wait = wait)
#       rospy.sleep(1)
        rospy.loginfo("[move_joint] Current positions: {}".format(self._arm_group.get_current_joint_values()))

        if wait:
            self._arm_group.stop()
            return all_close(positions[:7], self._arm_group.get_current_joint_values(), tolerance)

        return True

    def move_pose(self, pos_x, pos_y, pos_z, ori_w, ori_x, ori_y, ori_z):
        ee_link=""
        wait=True
        goal_pose = create_pose(pos_x, pos_y, pos_z, ori_w, ori_x, ori_y, ori_z)
	start_pose = self._arm_group.get_current_pose(ee_link).pose

        rospy.loginfo("[move_pose] Starting pose: {}".format(self._arm_group.get_current_pose(ee_link).pose))
        rospy.logdebug("[move_pose] Goal pose: {}".format(goal_pose))

        self._arm_group.clear_pose_targets()
        self._arm_group.set_pose_target(goal_pose, end_effector_link=ee_link)
        self._arm_group.go(wait=wait)
#        rospy.sleep(1)
        rospy.loginfo("[move_pose] Current pose: {}".format(self._arm_group.get_current_pose(ee_link).pose))

        if wait:
            self._arm_group.stop()
            return all_close(goal_pose, self._arm_group.get_current_pose().pose, 0.005)

        return True

    def move_finger(self, finger_l, finger_r):
        positions = [finger_l, finger_r]
        wait = True
        tolerance = 0.005

        rospy.loginfo("[move_finger] Starting positions: {}".format(self._hand_group.get_current_joint_values()))
        rospy.loginfo("[move_finger] Goal positions: {}".format(positions))
        
        self._hand_group.set_joint_value_target(positions)
        self._hand_group.go(wait = wait)
       # rospy.sleep(1)
        rospy.loginfo("[move_finger] Current positions: {}".format(self._hand_group.get_current_joint_values()))

        if wait:
            self._hand_group.stop()
            return all_close(positions[:2], self._hand_group.get_current_joint_values(), tolerance)

        return True

    def print_arm_joint_postion(self):
        rospy.loginfo("Print current arm joints: {}".format(self._arm_group.get_current_joint_values()))

    def print_end_effector_pose(self):
        rospy.loginfo("Print current end_effector pose: {}".format(self._arm_group.get_current_pose().pose))

    def print_hand_joint_postion(self):
        rospy.loginfo("Print current hand joints: {}".format(self._hand_group.get_current_joint_values()))

    def close_finger(self, wait = False):
        positions = [0, 0]
        rospy.loginfo("[close_finger] Starting positions: {}".format(self._hand_group.get_current_joint_values()))
        rospy.loginfo("[close_finger] Goal positions: {}".format(positions))
        self._hand_group.set_joint_value_target(positions)
        ret = self._hand_group.go(wait = wait)
        rospy.loginfo("[close_finger] Current positions: {}".format(self._hand_group.get_current_joint_values()))

        return ret

    def open_finger(self, wait = False):
        positions = [0.035, 0.035]
        rospy.loginfo("[open_finger] Starting positions: {}".format(self._hand_group.get_current_joint_values()))
        rospy.loginfo("[open_finger] Goal positions: {}".format(positions))
        self._hand_group.set_joint_value_target(positions)
        ret = self._hand_group.go(wait = wait)
        rospy.loginfo("[open_finger] Current positions: {}".format(self._hand_group.get_current_joint_values()))

        return ret

    def reset_joint(self, wait = True):
      #  positions = [0, 0, 0, 0, 0, 0, 0.785]
        positions = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        rospy.loginfo("[reset_joint] Starting positions: {}".format(self._arm_group.get_current_joint_values()))
        rospy.loginfo("[reset_joint] Goal positions: {}".format(positions))
        self._arm_group.set_joint_value_target(positions)
        ret = self._arm_group.go(wait = wait)
        rospy.loginfo("[reset_joint] Current positions: {}".format(self._arm_group.get_current_joint_values()))

        return ret


