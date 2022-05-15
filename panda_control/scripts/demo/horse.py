#!/usr/bin/env python
# coding: utf-8

import rospy
from panda_control import PandaMoveit
from panda_control import ModelMonitorThread


def main():
  try:
    rospy.init_node("panda_script")
    panda = PandaMoveit(True)
    thread = ModelMonitorThread("monitor_thread")
    thread.setDaemon(True)
    thread.start()
    rospy.loginfo("-----------------------------------")
    rospy.loginfo("Start panda moveit control script.")
    rospy.loginfo("-----------------------------------")

    # Add user control command here.
    panda.reset_joint()
    panda.open_finger()
    panda.move_pose([0.66, -0.355, 0.45, 3.14, 0, 0])
    panda.move_ee_pos(0.66, -0.355, 0.26)
    panda.move_finger(0.008, 0.008)
    panda.move_ee_pos(0.25, -0.355, 0.45)
    panda.move_ee_pos(0.25, 0.35, 0.45)
    panda.move_ee_pos(0.65, 0.3, 0.45)
    panda.move_ee_pos(0.65, 0.3, 0.28)
    panda.open_finger()
    panda.move_ee_pos(0.65, 0.3, 0.45)
    panda.reset_joint()

    rospy.loginfo("------------------------------------")
    rospy.loginfo("Finish panda moveit control script.")
    rospy.loginfo("------------------------------------")

  except rospy.ROSInterruptException:
    thread.exit()
    return
  except KeyboardInterrupt:
    thread.exit()
    return

if __name__ == '__main__':
    main()

