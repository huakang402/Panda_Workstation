#!/usr/bin/env python
# coding: utf-8

import rospy
from panda_control import PandaMoveit


def main():
  try:
    rospy.init_node("panda_script")
    panda = PandaMoveit(True)
    rospy.loginfo("-----------------------------------")
    rospy.loginfo("Start panda moveit control script.")
    rospy.loginfo("-----------------------------------")
    # Add user control command here.
  #  panda.move_joint(0.2,0.5,0.5,-0.7,0.5,0.2,0.4)
  #  panda.reset_joint()
    panda.move_finger(0.035, 0.035)
   # panda.print_end_effector_pose()
  #  panda.move_pose(0.5, 0, 0.59, 0, 1, 0, 0)
  #  panda.open_finger()
   # panda.close_finger()

    rospy.loginfo("------------------------------------")
    rospy.loginfo("Finish panda moveit control script.")
    rospy.loginfo("------------------------------------")

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
    main()

