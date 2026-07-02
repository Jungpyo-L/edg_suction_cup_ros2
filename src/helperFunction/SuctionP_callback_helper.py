#!/usr/bin/env python3

import threading
import time

import numpy as np
import rclpy
from scipy import signal

from suction_cup.msg import CmdPacket, SensorPacket


class P_CallbackHelp(object):
    def __init__(self, node):
        self.node = node
        self.subscription = node.create_subscription(
            SensorPacket,
            "SensorPacket",
            self.callback_P,
            10,
        )

        self.START_CMD = 2
        self.IDLE_CMD = 3
        self.RECORD_CMD = 10

        self.msg2Sensor = CmdPacket()
        self.P_vac = -10000.0

        self.sensorCMD_Pub = node.create_publisher(CmdPacket, "cmdPacket", 10)
        self.callback_Pub = node.create_publisher(SensorPacket, "SensorCallback", 10)
        self.callback_Pressure = SensorPacket()

        self.publish_enabled = True

        self.Psensor_Num = None
        self.BufferLen = 7

        self.PressureBuffer = None
        self.PressurePWMBuffer = None
        self.PressureOffsetBuffer = None
        self.P_idx = 0
        self.PWM_idx = 0
        self.offset_idx = 0

        self.startPresAvg = False
        self.startPresPWMAvg = False
        self.offsetMissing = True
        self.thisPres = None
        self.four_pressure = None
        self.four_pressurePWM = None
        self.PressureOffset = None
        self.power = 0.0

        self.samplingF = 166
        self.FFTbuffer_size = int(self.samplingF / 2)
        self.lock = threading.Lock()

    def initialize_arrays(self, num_ch):
        self.Psensor_Num = num_ch
        self.node.get_logger().info("Re-initializing arrays for %d chambers." % num_ch)

        self.PressureBuffer = [[0.0] * self.Psensor_Num for _ in range(self.BufferLen)]
        self.P_idx = 0
        self.startPresAvg = False

        self.PressurePWMBuffer = np.zeros((self.FFTbuffer_size, self.Psensor_Num))
        self.PressureOffsetBuffer = np.zeros((51, self.Psensor_Num))
        self.four_pressurePWM = np.zeros(self.Psensor_Num)
        self.PressureOffset = np.zeros(self.Psensor_Num)
        self.thisPres = np.zeros(self.Psensor_Num)
        self.PWM_idx = 0
        self.startPresPWMAvg = False

    def startSampling(self):
        self.msg2Sensor.cmd_input = self.START_CMD
        self.sensorCMD_Pub.publish(self.msg2Sensor)

    def stopSampling(self):
        self.msg2Sensor.cmd_input = self.IDLE_CMD
        self.sensorCMD_Pub.publish(self.msg2Sensor)

    def setNowAsOffset(self):
        if self.Psensor_Num is None:
            self.node.get_logger().warn(
                "Cannot set offset because number of chambers is unknown."
            )
            return

        self.PressureOffset *= 0
        time.sleep(0.5)

        with self.lock:
            buffer_copy = np.copy(self.PressureBuffer)

        self.PressureOffset = np.mean(buffer_copy, axis=0)

    def callback_P(self, data):
        if not self.publish_enabled or not rclpy_ok(self.node):
            return

        if self.Psensor_Num is None or self.Psensor_Num != data.ch:
            self.initialize_arrays(data.ch)

        self.thisPres = np.array(data.data, dtype=float)

        with self.lock:
            self.PressureBuffer[self.P_idx] = self.thisPres - self.PressureOffset
            self.P_idx += 1
            if self.P_idx == len(self.PressureBuffer):
                self.startPresAvg = True
                self.P_idx = 0

        self.PressurePWMBuffer[self.PWM_idx] = self.thisPres - self.PressureOffset
        self.PWM_idx += 1
        if self.PWM_idx == len(self.PressurePWMBuffer):
            self.startPresPWMAvg = True
            self.PWM_idx = 0

        if self.startPresAvg:
            buffer_np = np.array(self.PressureBuffer)
            averagePres_dummy = np.mean(buffer_np, axis=0)
            self.four_pressure = averagePres_dummy.tolist()

            if self.publish_enabled:
                self.callback_Pressure.ch = self.Psensor_Num
                self.callback_Pressure.data = self.four_pressure
                self.callback_Pub.publish(self.callback_Pressure)

        if self.startPresPWMAvg:
            averagePresPWM_dummy = np.zeros(self.Psensor_Num, dtype=float)
            fs = self.samplingF
            N = self.FFTbuffer_size
            fPWM = 30

            for i in range(self.Psensor_Num):
                _, _, Zxx = signal.stft(self.PressurePWMBuffer[:, i], fs, nperseg=N)
                if len(_) > 1:
                    delta_f = _[1] - _[0]
                    idx = int(fPWM / delta_f) if delta_f != 0 else 0
                    idx = min(idx, len(_) - 1)
                    power_spectrum = abs(Zxx[idx])
                    averagePresPWM_dummy[i] = np.mean(power_spectrum)

            self.four_pressurePWM = averagePresPWM_dummy

    def shutdown(self):
        self.publish_enabled = False


def rclpy_ok(node):
    return rclpy.ok()
