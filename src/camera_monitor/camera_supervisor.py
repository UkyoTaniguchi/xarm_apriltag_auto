#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

class CameraSupervisor:
    def __init__(self):
        rospy.init_node("camera_supervisor")
        self.tilt=0.0;self.icp=0.0
        self.tilt_thr=rospy.get_param("/thresholds/tilt_threshold_deg",1.0)
        self.icp_minor=rospy.get_param("/thresholds/icp_threshold",0.05)
        self.icp_major=rospy.get_param("/thresholds/icp_critical",0.15)
        rospy.Subscriber("/monitor/tilt_deg",Float32,self.cb_tilt)
        rospy.Subscriber("/monitor/icp_trans_norm",Float32,self.cb_icp)
        rospy.wait_for_service("/recalibration/run");self.recalib=rospy.ServiceProxy("/recalibration/run",Trigger)
        rospy.wait_for_service("/robot/stop");self.stop=rospy.ServiceProxy("/robot/stop",Trigger)
        rospy.Timer(rospy.Duration(0.5),self.timer)
        rospy.loginfo("camera_supervisor ready")
        rospy.spin()

    def cb_tilt(self,m):self.tilt=float(m.data)
    def cb_icp(self,m):self.icp=float(m.data)
    def timer(self,_):
        if self.icp>=self.icp_major:
            rospy.logerr(f"Critical ICP {self.icp:.3f}→STOP");self.stop();return
        if self.icp>=self.icp_minor or self.tilt>=self.tilt_thr:
            rospy.logwarn(f"Minor misalignment tilt={self.tilt:.2f} icp={self.icp:.3f}→Recalib")
            self.recalib()

if __name__=="__main__":CameraSupervisor()
