#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import PoseStamped

from .transformation_matrix import *
from .utils import *

import rtde_control
import rtde_receive

from scipy.spatial.transform import Rotation as R
import copy
import numpy as np



class rtdeHelp(object):
    def __init__(self, rtde_frequency=125, node=None, robot_ip="10.0.0.1"):
        self.node = node
        self.tf_buffer = None
        self.tf_listener = None
        if node is not None:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.rtde_frequency = rtde_frequency
        self.robot_ip = robot_ip
        # Bound before the connections are attempted so a failure part way
        # through still leaves disconnect() something safe to look at.
        self.rtde_c = None
        self.rtde_r = None

        # Announced because this is the one call that can sit for minutes: the
        # UR runs a single control program at a time, and RTDEControlInterface
        # uploads its own. If ur_control.launch.py already holds that slot this
        # blocks with no output at all, which reads as the script doing nothing.
        print("Opening the RTDE control interface at %s ..." % robot_ip)
        try:
            self.rtde_c = rtde_control.RTDEControlInterface(robot_ip, rtde_frequency)
        except Exception as exc:
            raise RuntimeError(
                "Could not open the RTDE control interface at %s (%s). The UR "
                "accepts one control program at a time, so this usually means "
                "ur_control.launch.py is holding it. Stop that and retry."
                % (robot_ip, exc)
            )
        if not self.rtde_c.isConnected():
            raise RuntimeError(
                "RTDE control interface at %s reported not connected. Stop "
                "ur_control.launch.py, which contends for the same program "
                "slot, and retry." % robot_ip
            )
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip, rtde_frequency)
        print("RTDE connected.")

        self.checkDistThres = 1e-3
        self.checkQuatThres = 10e-3

    def disconnect(self):
        """Release the control program and close both interfaces.

        Nothing used to do this, so every run left its uploaded program running
        on the controller. The next run then inherited whatever state that left
        behind, which is why a sweep would sometimes do nothing until it was
        started a second time. Safe to call more than once, and safe on a
        half-built helper.
        """
        try:
            if self.rtde_c is not None and self.rtde_c.isConnected():
                self.rtde_c.stopScript()
                self.rtde_c.disconnect()
        except Exception as exc:
            print("Could not release the RTDE control program: %s" % exc)
        try:
            if self.rtde_r is not None and self.rtde_r.isConnected():
                self.rtde_r.disconnect()
        except Exception as exc:
            print("Could not close the RTDE receive interface: %s" % exc)
        self.rtde_c = None
        self.rtde_r = None

    def _append_ns(self, in_ns, suffix):
        """
        Append a sub-namespace (suffix) to the input namespace
        @param in_ns Input namespace
        @type in_ns str
        @return Suffix namespace
        @rtype str
        """
        ns = in_ns
        if ns[-1] != '/':
            ns += '/'
        ns += suffix
        return ns

    def getPoseObj(self, goalPosition, setOrientation):
        Pose = PoseStamped()  
        
        Pose.header.frame_id = "base_link"
        if self.node is not None:
            Pose.header.stamp = self.node.get_clock().now().to_msg()
        Pose.pose.orientation.x = setOrientation[0]
        Pose.pose.orientation.y = setOrientation[1]
        Pose.pose.orientation.z = setOrientation[2]
        Pose.pose.orientation.w = setOrientation[3]
        
        Pose.pose.position.x = goalPosition[0]
        Pose.pose.position.y = goalPosition[1]
        Pose.pose.position.z = goalPosition[2]
        
        return Pose

    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2,
                w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2)
    
    def getRotVector(self, goalPose):
        qx = goalPose.pose.orientation.x
        qy = goalPose.pose.orientation.y
        qz = goalPose.pose.orientation.z
        qw = goalPose.pose.orientation.w
        r = R.from_quat([qx, qy, qz, qw])
        Rx, Ry, Rz = r.as_rotvec()
        return Rx, Ry, Rz

    def getTCPPose(self, pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        Rx, Ry, Rz = self.getRotVector(pose)
        return [x, y, z, Rx, Ry, Rz]
    
    def speedl(self, goalPose,speed=0.5, acc=0.5, time=0.5, aRot='a'):

        if len(goalPose) != 6:
            raise ValueError("Target pose must have 6 elements: [x, y, z, Rx, Ry, Rz]")
        try:
        # Perform linear motion using moveL function
            self.rtde_c.speedL(goalPose, self.speed, self.acc, time, aRot)
        except Exception as e:
            print(f"Error occurred during linear motion: {e}")

    
    def setPayload(self, payload, CoG):
        # Assuming method is avalible within RTDEControlInterface
        self.rtde_c.set_payload(payload, CoG)

    def goToPositionOrientation(self, goalPosition, setOrientation, asynchronous = False):
        self.goToPose(getPoseObj(goalPosition, setOrientation, self.node))

    def goToPose(self, goalPose, speed = 0.1, acc = 0.1, asynchronous=False):
        targetPose = self.getTCPPose(goalPose)
        self.rtde_c.moveL(targetPose, speed, acc, asynchronous)

    def goToPoseAdaptive(self, goalPose, speed = 0.0, acc = 0.0,  time = 0.05, lookahead_time = 0.2, gain = 100.0):         # normal force measurement
        t_start = self.rtde_c.initPeriod()
        targetPose = self.getTCPPose(goalPose)
        self.rtde_c.servoL(targetPose, speed, acc, time, lookahead_time, gain)
        self.rtde_c.waitPeriod(t_start)

    def checkGoalPoseReached(self, goalPose, checkDistThres=np.nan, checkQuatThres = np.nan):
        if np.isnan(checkDistThres):
            checkDistThres=self.checkDistThres
        if np.isnan(checkQuatThres):
            checkQuatThres = self.checkQuatThres
        (trans1,rot) = self.readCurrPositionQuat()
        goalQuat = np.array([goalPose.pose.orientation.x,goalPose.pose.orientation.y, goalPose.pose.orientation.z, goalPose.pose.orientation.w])
        rot_array = np.array(rot)
        quatDiff = np.min([np.max(np.abs(goalQuat - rot_array)), np.max(np.abs(goalQuat + rot_array))])
        distDiff = np.linalg.norm(np.array([goalPose.pose.position.x,goalPose.pose.position.y, goalPose.pose.position.z])- np.array(trans1)) 
        # print(quatDiff, distDiff)
        print("quatdiff: %.4f" % quatDiff)
        print("distDiff: %.4f" % distDiff)
        return distDiff < checkDistThres and quatDiff < checkQuatThres

        
    def readCurrPositionQuat(self):
        if self.tf_buffer is None:
            pose = self.getCurrentPose()
            trans = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            rot = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
            return (trans, rot)

        transform = self.tf_buffer.lookup_transform("base_link", "tool0", Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        trans = [translation.x, translation.y, translation.z]
        rot = [rotation.x, rotation.y, rotation.z, rotation.w]
        return (trans, rot) #trans= position x,y,z, // quaternion: x,y,z,w

    def stopAtCurrPose(self,asynchronous = True):
        currPosition, orientation = self.readCurrPositionQuat()

        # always false
        # wait = False
        self.goToPositionOrientation(currPosition, orientation, asynchronous=asynchronous)
    
    def stopAtCurrPoseAdaptive(self):
        self.rtde_c.servoStop()
        # self.rtde_c.stopScript()

    # Get current pose from TF
    def getCurrentPoseTF(self):
        (Position, Orientation) = self.readCurrPositionQuat()
        return getPoseObj(Position, Orientation, self.node)

    def getCurrentPose(self):
        TCPPose = self.rtde_r.getActualTCPPose()
        Position = [TCPPose[0], TCPPose[1], TCPPose[2]]
        r = R.from_rotvec(np.array([TCPPose[3], TCPPose[4], TCPPose[5]]))
        return getPoseObj(Position, r.as_quat(), self.node)

    def getTCPoffset(self):
        return self.rtde_c.getTCPOffset()

    def setTCPoffset(self, offset):
        return self.rtde_c.setTcp(offset)

    def getMethodsName_r(self):
        object_methods = [method_name for method_name in dir(self.rtde_r) if callable(getattr(self.rtde_r, method_name))]
        print(object_methods)

    def getMethodsName_c(self):
        object_methods = [method_name for method_name in dir(self.rtde_c) if callable(getattr(self.rtde_c, method_name))]
        print(object_methods)