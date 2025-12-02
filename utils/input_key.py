#!/usr/bin/env python3
import rospy
import sys, termios, tty
from std_msgs.msg import String

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

if __name__ == "__main__":
    rospy.init_node("key_input_node")
    pub = rospy.Publisher("/key_input", String, queue_size=1)

    rospy.loginfo("yaw   (u = +0.5°, j = -0.5°)")
    rospy.loginfo("roll  (r = +0.5°, f = -0.5°)")
    rospy.loginfo("pitch (p = +0.5°, l = -0.5°)")
    rospy.loginfo("Press Ctrl+C to exit")

    while not rospy.is_shutdown():
        c = getch()

        # ★ Ctrl+C (0x03) を検出して自分でシャットダウン
        if c == "\x03":
            rospy.loginfo("Ctrl+C detected, exiting key_input_node.")
            rospy.signal_shutdown("Ctrl+C pressed")
            break

        pub.publish(c)

