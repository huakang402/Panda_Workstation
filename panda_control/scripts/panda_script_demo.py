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

