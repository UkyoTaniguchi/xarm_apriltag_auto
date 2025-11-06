#!/usr/bin/env python3
import math, time, threading, numpy as np, cv2, open3d as o3d
import rospy
from sensor_msgs.msg import Imu, Image, CameraInfo
from std_msgs.msg import Float32, Float32MultiArray
from cv_bridge import CvBridge

class ImuIcpMonitor:
    def __init__(self):
        rospy.init_node("imu_icp_monitor")
        self.bridge = CvBridge()
        self.tilt_thr = rospy.get_param("/thresholds/tilt_threshold_deg", 1.0)
        self.icp_rate = rospy.get_param("~icp_rate", 2.0)
        self.alpha = rospy.get_param("~alpha", 0.1)
        self.depth_topic = rospy.get_param("~depth_topic","/camera/depth/image_rect_raw")
        self.info_topic  = rospy.get_param("~camera_info_topic","/camera/depth/camera_info")
        self.imu_topic   = rospy.get_param("~imu_topic","/camera/imu")
        self.roi = [
            rospy.get_param("/roi/xmin",0.25),
            rospy.get_param("/roi/ymin",0.20),
            rospy.get_param("/roi/xmax",0.40),
            rospy.get_param("/roi/ymax",0.50)
        ]
        self.K=None; self.ref_g=None; self.filt_g=None; self.icp_ref=None
        self.pub_tilt=rospy.Publisher("/monitor/tilt_deg",Float32,queue_size=1)
        self.pub_icp=rospy.Publisher("/monitor/icp_trans_norm",Float32,queue_size=1)
        rospy.Subscriber(self.info_topic,CameraInfo,self.cb_info)
        rospy.Subscriber(self.imu_topic,Imu,self.cb_imu)
        rospy.Subscriber(self.depth_topic,Image,self.cb_depth)
        rospy.Subscriber("/monitor/roi",Float32MultiArray,self.cb_roi)
        rospy.loginfo("imu_icp_monitor ready")
        rospy.spin()

    def cb_info(self,msg):
        self.K=(msg.K[0],msg.K[4],msg.K[2],msg.K[5],msg.width,msg.height)

    def cb_roi(self,msg):
        if len(msg.data)==4:self.roi=list(msg.data)

    def cb_imu(self,msg):
        a=np.array([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        n=np.linalg.norm(a)
        if n<1e-6:return
        g=a/n
        if self.filt_g is None:self.filt_g=g.copy()
        if self.ref_g is None:self.ref_g=g.copy()
        self.filt_g=(1-self.alpha)*self.filt_g+self.alpha*g
        self.filt_g/=np.linalg.norm(self.filt_g)
        tilt=math.degrees(math.acos(np.clip(np.dot(self.filt_g,self.ref_g),-1,1)))
        self.pub_tilt.publish(Float32(data=tilt))

    def cb_depth(self,msg):
        if self.K is None:return
        depth=self.bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough")
        if depth.dtype==np.uint16:depth=depth.astype(np.float32)/1000.0
        fx,fy,cx,cy,W,H=self.K
        x0,y0,x1,y1=self.roi
        xmin,xmax=int(x0*W),int(x1*W);ymin,ymax=int(y0*H),int(y1*H)
        mask=np.zeros_like(depth,bool);mask[ymin:ymax,xmin:xmax]=True
        depth_roi=depth.copy();depth_roi[~mask]=0.0
        intr=o3d.camera.PinholeCameraIntrinsic(W,H,fx,fy,cx,cy)
        pcd=o3d.geometry.PointCloud.create_from_depth_image(o3d.geometry.Image(depth_roi),intr)
        if self.icp_ref is None:self.icp_ref=pcd;return
        reg=o3d.pipelines.registration.registration_icp(
            pcd,self.icp_ref,0.1,
            o3d.pipelines.registration.TransformationEstimationPointToPoint())
        dx,dy,dz=reg.transformation[0,3],reg.transformation[1,3],reg.transformation[2,3]
        self.pub_icp.publish(Float32(data=float(math.sqrt(dx*dx+dy*dy+dz*dz))))
