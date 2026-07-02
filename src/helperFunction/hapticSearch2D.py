#!/usr/bin/env python3

import numpy as np
import scipy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as Rot

from .utils import (
    create_transform_matrix,
    hat,
    quaternion_from_matrix,
    rotation_from_quaternion,
)


class hapticSearch2DHelp(object):
    def __init__(
        self,
        dP_threshold=10,
        dw=15,
        P_vac=-18000,
        d_lat=0.5e-3,
        d_z=1.5e-3,
        d_yaw=1.5,
        damping_factor=0.7,
        n_ch=4,
        p_reverse=False,
    ):
        self.dP_threshold = dP_threshold
        self.dw = dw * np.pi / 180.0

        self.P_vac = P_vac
        self.p_reverse = p_reverse
        self.d_lat = d_lat
        self.d_z_normal = d_z
        self.d_yaw = d_yaw

        self.x0 = 0
        self.BM_step = 0
        self.BM_x = 0
        self.BM_y = 0

        self.n = n_ch

        self.velocity = np.array([0.0, 0.0])
        self.d_lat_momentum = self.d_lat * 0.3
        self.damping_factor = damping_factor
        self.max_velocity_x = self.d_lat * 1.5

        self.cumulative_yaw_deg = 0.0
        self.max_cumulative_yaw_deg = 45.0
        self.max_velocity_y = self.d_lat * 1.5

    def reset_yaw_tracking(self):
        self.cumulative_yaw_deg = 0.0

    def get_yawRotation_from_T(self, T):
        R = T[0:3, 0:3]
        quat = quaternion_from_matrix(R)
        r = Rot.from_quat(quat)
        return r.as_euler("zyx")[0]

    def get_ObjectPoseStamped_from_T(self, T):
        thisPose = PoseStamped()
        thisPose.header.frame_id = "base_link"
        R = T[0:3, 0:3]
        quat = quaternion_from_matrix(R)
        position = T[0:3, 3]
        thisPose.pose.position.x = position[0]
        thisPose.pose.position.y = position[1]
        thisPose.pose.position.z = position[2]
        thisPose.pose.orientation.x = quat[0]
        thisPose.pose.orientation.y = quat[1]
        thisPose.pose.orientation.z = quat[2]
        thisPose.pose.orientation.w = quat[3]
        return thisPose

    def get_Tmat_from_Pose(self, PoseStamped):
        quat = [
            PoseStamped.pose.orientation.x,
            PoseStamped.pose.orientation.y,
            PoseStamped.pose.orientation.z,
            PoseStamped.pose.orientation.w,
        ]
        translate = [
            PoseStamped.pose.position.x,
            PoseStamped.pose.position.y,
            PoseStamped.pose.position.z,
        ]
        return self.get_Tmat_from_PositionQuat(translate, quat)

    def get_Tmat_from_PositionQuat(self, Position, Quat):
        rotationMat = rotation_from_quaternion(Quat)
        return create_transform_matrix(rotationMat, Position)

    def get_PoseStamped_from_T_initPose(self, T, initPoseStamped):
        T_now = self.get_Tmat_from_Pose(initPoseStamped)
        return self.get_ObjectPoseStamped_from_T(np.matmul(T_now, T))

    def get_Tmat_TranlateInBodyF(self, translate=(0.0, 0.0, 0.0)):
        return create_transform_matrix(np.eye(3), translate)

    def get_Tmat_TranlateInZ(self, direction=1):
        offset = [0.0, 0.0, np.sign(direction) * self.d_z_normal]
        return self.get_Tmat_TranlateInBodyF(translate=offset)

    def get_Tmat_TranlateInY(self, direction=1):
        offset = [0.0, np.sign(direction) * self.d_lat, 0.0]
        return self.get_Tmat_TranlateInBodyF(translate=offset)

    def get_Tmat_TranlateInX(self, direction=1):
        offset = [np.sign(direction) * self.d_lat, 0.0, 0.0]
        return self.get_Tmat_TranlateInBodyF(translate=offset)

    def calculate_unit_vectors(self, num_chambers):
        if num_chambers == 3:
            first_chamber_angle = -np.pi / 3
        elif num_chambers == 4:
            first_chamber_angle = -np.pi / 4
        elif num_chambers == 5:
            first_chamber_angle = -np.pi / 2
        elif num_chambers == 6:
            first_chamber_angle = -np.pi / 3
        else:
            raise ValueError("Number of chambers is not supported")
        return [
            np.array(
                [
                    np.cos(first_chamber_angle + 2 * np.pi * i / num_chambers),
                    np.sin(first_chamber_angle + 2 * np.pi * i / num_chambers),
                ]
            )
            for i in range(num_chambers)
        ]

    def calculate_direction_vector(self, unit_vectors, vacuum_pressures):
        direction_vector = np.sum(
            [vp * uv for vp, uv in zip(vacuum_pressures, unit_vectors)], axis=0
        )
        norm = np.linalg.norm(direction_vector)
        return direction_vector / norm if norm > 0 else np.array([0, 0])

    def get_lateral_direction_vector(self, P_array, thereshold=True):
        th = self.dP_threshold if thereshold else 0
        if not self.p_reverse:
            P_array = [-P for P in P_array]
        P_array = [P if P > th else 0 for P in P_array]
        unit_vectors = self.calculate_unit_vectors(self.n)
        return self.calculate_direction_vector(unit_vectors, P_array)

    def get_Tmat_lateralMove(self, P_array):
        v = self.get_lateral_direction_vector(P_array, True)
        v_step = v * self.d_lat
        return self.get_Tmat_TranlateInBodyF([-v_step[1], -v_step[0], 0.0])

    def get_Tmat_momentumMove(self, P_array):
        v = self.get_lateral_direction_vector(P_array, True)
        self.velocity = self.damping_factor * self.velocity + v * self.d_lat
        return self.get_Tmat_TranlateInBodyF([-self.velocity[1], -self.velocity[0], 0.0])

    def get_Tmat_yawRotation(self):
        d_yaw = self.d_yaw * np.pi / 180
        rot_axis = np.array([0, 0, -1])
        omega_hat = hat(rot_axis)
        Rw = scipy.linalg.expm(d_yaw * omega_hat)
        return create_transform_matrix(Rw, [0, 0, 0])

    def get_Tmats_from_controller(self, P_array, controller_str):
        if controller_str in ("normal", "greedy"):
            T_align = np.eye(4)
            T_later = self.get_Tmat_lateralMove(P_array)
            T_yaw = np.eye(4)
        elif controller_str == "yaw":
            T_align = np.eye(4)
            T_later = self.get_Tmat_lateralMove(P_array)
            T_yaw = self.get_Tmat_yawRotation()
        elif controller_str == "momentum":
            T_align = np.eye(4)
            T_later = self.get_Tmat_momentumMove(P_array)
            T_yaw = np.eye(4)
        elif controller_str in ("yaw_momentum", "momentum_yaw"):
            T_align = np.eye(4)
            T_later = self.get_Tmat_momentumMove(P_array)
            T_yaw = self.get_Tmat_yawRotation()
        else:
            print(f"Warning: Unknown controller '{controller_str}', using greedy controller")
            T_align = np.eye(4)
            T_later = self.get_Tmat_lateralMove(P_array)
            T_yaw = np.eye(4)

        return T_later, T_yaw, T_align
