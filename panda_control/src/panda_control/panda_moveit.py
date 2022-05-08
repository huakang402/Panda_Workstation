# coding: utf-8

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi
from tf.transformations import quaternion_from_euler
from moveit_commander.conversions import pose_to_list


def create_pose(pose_list):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = pose_list[0]
    pose.position.y = pose_list[1]
    pose.position.z = pose_list[2]
    if len(pose_list) == 6:
        q = quaternion_from_euler(pose_list[3], pose_list[4], pose_list[5])
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
    else:
        pose.orientation.x = pose_list[3]
        pose.orientation.y = pose_list[4]
        pose.orientation.z = pose_list[5]
        pose.orientation.w = pose_list[6]

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

        rospy.loginfo("PandaArmGroup: Setting default EE link to '{}' "
            "Use _arm_group.set_end_effector_link() method to change default EE.".format(self._default_ee))

        self._arm_group.set_max_velocity_scaling_factor(0.3)
        self._hand_group.set_max_velocity_scaling_factor(0.3)

    def move_joint(self, j1, j2, j3, j4, j5, j6, j7):
        positions = [j1,j2,j3,j4,j5,j6,j7]
        wait = True
        tolerance = 0.005

        rospy.loginfo("[move_joint] Starting joint positions: {}".format(self._arm_group.get_current_joint_values()))
        rospy.loginfo("[move_joint] Goal joint positions: {}".format(positions))
        
        self._arm_group.set_joint_value_target(positions)
        self._arm_group.go(wait = wait)
#       rospy.sleep(1)
        rospy.loginfo("[move_joint] Current joint positions: {}".format(self._arm_group.get_current_joint_values()))

        if wait:
            self._arm_group.stop()
            return all_close(positions[:7], self._arm_group.get_current_joint_values(), tolerance)

        return True

    def get_end_effector_pos(self, ee_link=""):
        pose = self._arm_group.get_current_pose(ee_link).pose
        position = [pose.position.x, pose.position.y, pose.position.z]

        return position

    def get_end_effector_ori(self, ee_link=""):
	pose = self._arm_group.get_current_pose(ee_link).pose
        ori = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        return ori

    def get_end_effector_rpy(self, ee_link=""):
        return self._arm_group.get_current_rpy(ee_link)

    def move_ee_pos(self, x, y, z):
        ee_link=""
        wait=True
        goal_position = [x, y, z]
        cur_ori = self.get_end_effector_ori(ee_link)
        goal_pose = create_pose(goal_position + cur_ori)
        cur_position = self.get_end_effector_pos(ee_link)

        rospy.loginfo("[move_ee_pos] Starting end_effector position: {}".format(cur_position))
        rospy.loginfo("[move_ee_pos] Goal end_effector position: {}".format(goal_position))

        self._arm_group.clear_pose_targets()
        self._arm_group.set_pose_target(goal_pose, end_effector_link=ee_link)
        self._arm_group.go(wait=wait)
#        rospy.sleep(1)
	cur_position = self.get_end_effector_pos()
        rospy.loginfo("[move_ee_pos] Current end_effector position: {}".format(cur_position))

        if wait:
            self._arm_group.stop()
            return all_close(goal_position, self.get_end_effector_pos(ee_link), 0.005)

        return True

    def move_ee_ori(self, x, y, z, w):
        ee_link=""
        wait=True
        goal_ori = [x, y, z, w]
        cur_ori = self.get_end_effector_ori(ee_link)
        cur_pos = self.get_end_effector_pos(ee_link)
        goal_pose = create_pose(cur_pos + goal_ori)

        rospy.loginfo("[move_ee_ori] Starting end_effector orientation: {}".format(cur_ori))
        rospy.loginfo("[move_ee_ori] Goal end_effector orientation: {}".format(goal_ori))

        self._arm_group.clear_pose_targets()
        self._arm_group.set_pose_target(goal_pose, end_effector_link=ee_link)
        self._arm_group.go(wait=wait)
#        rospy.sleep(1)
        cur_ori = self.get_end_effector_ori(ee_link)
        rospy.loginfo("[move_ee_ori] Current end_effector orientation: {}".format(cur_ori))

        if wait:
            self._arm_group.stop()
            return all_close(goal_ori, self.get_end_effector_ori(ee_link), 0.005)

        return True

    def move_ee_rpy(self, r, p, y):
        ee_link=""
        wait=True
        goal_rpy = [r, p, y]
	current_rpy = self.get_end_effector_rpy(ee_link)
        cur_pos = self.get_end_effector_pos(ee_link)
        goal_pose = create_pose(cur_pos + goal_rpy)

        rospy.loginfo("[move_ee_rpy] Starting end_effector RPY: {}".format(current_rpy))
        rospy.loginfo("[move_ee_rpy] Goal end_effector RPY: {}".format(goal_rpy))

        self._arm_group.clear_pose_targets()
        self._arm_group.set_pose_target(goal_pose, end_effector_link=ee_link)
        self._arm_group.go(wait=wait)
#        rospy.sleep(1)
	current_rpy = self.get_end_effector_rpy(ee_link)
        rospy.loginfo("[move_ee_rpy] Current end_effector RPY: {}".format(current_rpy))

        if wait:
            self._arm_group.stop()
            return all_close(goal_rpy, self.get_end_effector_rpy(ee_link), 0.005)

        return True

    def move_pose(self, pose_list):
        ee_link=""
        wait=True
        goal_pose = create_pose(pose_list)
	current_pose = self._arm_group.get_current_pose(ee_link).pose

        rospy.loginfo("[move_pose] Starting end_effector pose: {}".format(current_pose))
        rospy.loginfo("[move_pose] Goal end_effector pose: {}".format(goal_pose))

        self._arm_group.clear_pose_targets()
        self._arm_group.set_pose_target(goal_pose, end_effector_link=ee_link)
        self._arm_group.go(wait=wait)
#        rospy.sleep(1)
        current_pose = self._arm_group.get_current_pose(ee_link).pose
        rospy.loginfo("[move_pose] Current end_effector pose: {}".format(current_pose))

        if wait:
            self._arm_group.stop()
            return all_close(goal_pose, self._arm_group.get_current_pose(ee_link).pose, 0.005)

        return True

    def move_finger(self, finger_l, finger_r):
        wait = True
        tolerance = 0.005
        goal_positions = [finger_l, finger_r]
        current_positions = self._hand_group.get_current_joint_values()

        rospy.loginfo("[move_finger] Starting positions: {}".format(current_positions))
        rospy.loginfo("[move_finger] Goal positions: {}".format(goal_positions))
        
        self._hand_group.set_joint_value_target(goal_positions)
        self._hand_group.go(wait = wait)
       # rospy.sleep(1)
        current_positions = self._hand_group.get_current_joint_values()
        rospy.loginfo("[move_finger] Current positions: {}".format(current_positions))

        if wait:
            self._hand_group.stop()
            return all_close(goal_positions[:2], self._hand_group.get_current_joint_values(), tolerance)

        return True

    def print_arm_joint_postion(self):
        rospy.loginfo("Print current arm joints: {}".format(self._arm_group.get_current_joint_values()))

    def print_end_effector_pose(self):
        rospy.loginfo("Print current end_effector pose: {}".format(self._arm_group.get_current_pose().pose))

    def print_end_effector_rpy(self):
        rospy.loginfo("Print current end_effector RPY: {}".format(self._arm_group.get_current_rpy()))

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


