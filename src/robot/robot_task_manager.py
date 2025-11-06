#!/usr/bin/env python3
import rospy, threading, time
from std_msgs.msg import Empty
from std_srvs.srv import Trigger, TriggerResponse

class RobotTaskManager:
    def __init__(self):
        rospy.init_node("robot_task_manager")
        self.pub_done = rospy.Publisher("/task/done", Empty, queue_size=1)
        self.srv_stop = rospy.Service("/robot/stop", Trigger, self.on_stop)
        self.stop_flag = False
        threading.Thread(target=self.loop, daemon=True).start()
        rospy.loginfo("robot_task_manager ready")
        rospy.spin()

    def loop(self):
        while not rospy.is_shutdown():
            if self.stop_flag:
                rospy.logwarn("Task paused by stop request")
                time.sleep(1.0)
                continue
            rospy.loginfo("Task running...")
            time.sleep(5.0)
            self.pub_done.publish(Empty())
            rospy.loginfo("Task done")
            time.sleep(1.0)

    def on_stop(self, _):
        self.stop_flag = True
        return TriggerResponse(success=True, message="task paused")

if __name__ == "__main__":
    RobotTaskManager()
