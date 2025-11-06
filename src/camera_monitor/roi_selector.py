#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty, Float32MultiArray

class RoiSelector:
    def __init__(self):
        rospy.init_node("roi_selector")
        self.roi=[rospy.get_param("/roi/xmin",0.25),
                  rospy.get_param("/roi/ymin",0.20),
                  rospy.get_param("/roi/xmax",0.40),
                  rospy.get_param("/roi/ymax",0.50)]
        self.pub=rospy.Publisher("/monitor/roi",Float32MultiArray,queue_size=1)
        rospy.Subscriber("/task/done",Empty,self.cb)
        rospy.loginfo("roi_selector ready")
        rospy.spin()

    def cb(self,_):
        x0,y0,x1,y1=self.roi
        cx,cy=(x0+x1)/2,(y0+y1)/2
        sx,sy=(x1-x0)*0.9,(y1-y0)*0.9
        self.roi=[cx-sx/2,cy-sy/2,cx+sx/2,cy+sy/2]
        self.pub.publish(Float32MultiArray(data=self.roi))
        rospy.loginfo(f"ROI updated {self.roi}")

if __name__=="__main__":RoiSelector()
