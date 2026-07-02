#!/usr/bin/env python3

"""ESP32-S3 pressure sensor reader and publisher."""

import argparse
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

import serial
from suction_cup.msg import CmdPacket, SensorPacket

NO_CMD = 0
START_CMD = 2
IDLE_CMD = 3


class ESP32PressureNode(Node):
    def __init__(self, expected_channels=None, auto_start=True):
        super().__init__("ESP32_Pressure")
        self.expected_channels = expected_channels
        self.num_channels = expected_channels
        self.streaming = False
        self.cmd_in = NO_CMD
        self.channels_detected = False

        self.pub = self.create_publisher(SensorPacket, "SensorPacket", 10)
        self.create_subscription(CmdPacket, "cmdPacket", self.callback, 10)

        self.msg = SensorPacket()
        self.msg.ch = 0
        self.msg.data = []

        self.declare_parameter("serial_port", "/dev/ttyPressure")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("auto_start", auto_start)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        auto_start = self.get_parameter("auto_start").get_parameter_value().bool_value

        self.ser = serial.Serial(port, baudrate=baudrate, timeout=1, write_timeout=1)
        self.ser.flushInput()

        if auto_start:
            self._start_streaming()

        self.timer = self.create_timer(0.01, self.timer_callback)

    def callback(self, data):
        self.cmd_in = data.cmd_input

    def _start_streaming(self):
        self.ser.write(struct.pack("<B", ord("i")))
        self.ser.write(struct.pack("<B", ord("s")))
        self.streaming = True
        self.cmd_in = NO_CMD

    def _stop_streaming(self):
        self.ser.write(struct.pack("<B", ord("i")))
        self.ser.flushInput()
        self.streaming = False
        self.cmd_in = NO_CMD

    def _parse_pressure_line(self, ser_bytes):
        split_data = [value for value in ser_bytes.split(" ") if value.strip()]
        if not split_data:
            return None

        try:
            values = [float(value) for value in split_data]
        except ValueError:
            self.get_logger().warning("Skipping non-numeric pressure serial line")
            return None

        detected_channels = len(values)
        if not self.channels_detected:
            self.num_channels = detected_channels
            self.channels_detected = True
        elif detected_channels != self.num_channels:
            self.get_logger().warning(
                "Received %d pressure values, expected %d"
                % (detected_channels, self.num_channels)
            )
            return None

        if self.expected_channels is not None and self.num_channels != self.expected_channels:
            self.get_logger().warning(
                "Detected %d chambers, but expected %d"
                % (self.num_channels, self.expected_channels)
            )

        return values

    def timer_callback(self):
        if not rclpy.ok():
            return

        try:
            if not self.streaming and self.cmd_in == START_CMD:
                self._start_streaming()
                return

            if self.streaming and self.cmd_in == IDLE_CMD:
                self._stop_streaming()
                return

            if not self.streaming:
                return

            ser_bytes = self.ser.readline().decode("utf-8", errors="ignore")
            if not ser_bytes.strip():
                return

            values = self._parse_pressure_line(ser_bytes)
            if values is None:
                return

            self.msg.ch = self.num_channels
            self.msg.data = values
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.msg)
        except Exception as exc:
            self.get_logger().error("SensorComError: %s" % exc)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ch",
        type=int,
        help="optional expected number of chambers for validation",
        default=None,
    )
    parser.add_argument(
        "--no-auto-start",
        action="store_true",
        help="wait for cmdPacket START_CMD instead of starting on launch",
    )
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = ESP32PressureNode(
        expected_channels=args.ch,
        auto_start=not args.no_auto_start,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
