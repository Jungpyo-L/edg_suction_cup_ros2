#!/usr/bin/env python3

"""ESP32 PWM bridge for suction cup experimentation."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

import serial


class ESP32PWMNode(Node):
    def __init__(self):
        super().__init__("ESP32_PWM")
        self.pwm_val = 0

        self.create_subscription(Int8, "pwm", self.callback, 10)

        self.declare_parameter("serial_port", "/dev/ttyPWM")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=1, write_timeout=1)
        self.ser.flushInput()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback(self, data):
        self.pwm_val = data.data

    def timer_callback(self):
        try:
            self.ser.readline().decode("utf-8", errors="ignore")
            payload = (str(self.pwm_val) + "\n").encode("utf-8")
            self.ser.write(payload)
        except serial.SerialTimeoutException:
            self.get_logger().warning("Serial write timeout")
        except serial.SerialException as exc:
            self.get_logger().error("Serial communication failed: %s" % exc)


def main():
    rclpy.init()
    node = ESP32PWMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
