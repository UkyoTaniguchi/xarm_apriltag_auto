#!/usr/bin/env python3
import rospy, subprocess, os
from std_msgs.msg import Float32

def launch_node(python_file):
    pkg=os.path.join(rospy.get_param("/ros_pkg_path",""),"src")
    path=os.path.join(pkg,python_file)
    return subprocess.Popen(["python3",path])

class MainSupervisor:
    def __init__(self):
        rospy.init_node("main_supervisor")
        self.procs=[]
        self.procs.append(subprocess.Popen(["python3",os.path.join(os.path.dirname(__file__),"../src/camera_monitor/imu_icp_monitor.py")]))
        self.procs.append(subprocess.Popen(["python3",os.path.join(os.path.dirname(__file__),"../src/camera_monitor/camera_supervisor.py")]))
        self.procs.append(subprocess.Popen(["python3",os.path.join(os.path.dirname(__file__),"../src/camera_monitor/roi_selector.py")]))
        self.procs.append(subprocess.Popen(["python3",os.path.join(os.path.dirname(__file__),"../src/robot/move_to_tag.py")]))
        self.procs.append(subprocess.Popen(["python3",os.path.join(os.path.dirname(__file__),"../src/robot/robot_task_manager.py")]))
        rospy.Subscriber("/monitor/tilt_deg",Float32,self.cb_tilt)
        rospy.Subscriber("/monitor/icp_trans_norm",Float32,self.cb_icp)
        self.tilt=self.icp=0.0
        rospy.Timer(rospy.Duration(5.0),self.log)
        rospy.loginfo("main_supervisor running")
        rospy.spin()

    def cb_tilt(self,m):self.tilt=m.data
    def cb_icp(self,m):self.icp=m.data
    def log(self,_):rospy.loginfo(f"[STATE] tilt={self.tilt:.2f} icp={self.icp:.3f}")

if __name__=="__main__":MainSupervisor()
