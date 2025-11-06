#!/usr/bin/env python3
import rospy, math
import moveit_commander
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler

class RecalibrationServer:
    def __init__(self):
        rospy.init_node("xarm_recalibration_server")
        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("xarm6")
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.arm.set_max_acceleration_scaling_factor(1.0)
        self.arm.set_planning_time(5.0)
        self.home = self.arm.get_current_pose().pose

        p = "/poses/recalib_pose"
        self.tx = rospy.get_param(p+"/x", 0.0)
        self.ty = rospy.get_param(p+"/y", -0.5)
        self.tz = rospy.get_param(p+"/z", 0.3)
        self.rx = math.radians(rospy.get_param(p+"/rx_deg", 90.0))
        self.ry = math.radians(rospy.get_param(p+"/ry_deg", 0.0))
        self.rz = math.radians(rospy.get_param(p+"/rz_deg", 0.0))

        self.srv = rospy.Service("/recalibration/run", Trigger, self.on_run)
        rospy.loginfo("Recalibration server ready")
        rospy.spin()

    def _move_to(self, x,y,z,rx,ry,rz):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x,y,z
        qx,qy,qz,qw = quaternion_from_euler(rx,ry,rz)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx,qy,qz,qw
        self.arm.stop(); self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose)
        return bool(self.arm.go(wait=True))

    def _go_home(self):
        self.arm.stop(); self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(self.home)
        self.arm.go(wait=True)

    def on_run(self, _):
        ok = self._move_to(self.tx,self.ty,self.tz,self.rx,self.ry,self.rz)
        if not ok: return TriggerResponse(success=False, message="move failed")
        rospy.sleep(2.0)
        self._go_home()
        return TriggerResponse(success=True, message="recalibration done")

if __name__ == "__main__":
    RecalibrationServer()
