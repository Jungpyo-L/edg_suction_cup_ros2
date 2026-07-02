#!/usr/bin/env python3

# Basic data logging example using the UR10e and ATI force/torque sensor.

import argparse
from datetime import datetime
import time

import numpy as np
import rclpy
from std_msgs.msg import Int8

from suction_cup.srv import Enable
from helperFunction.FT_callback_helper import FT_CallbackHelp
from helperFunction.fileSaveHelper import fileSaveHelp
from helperFunction.ros2_helpers import call_enable_service


def wait_for_data_logger(node, client):
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for the data_logging service...")


def main(args):
    sync_start = 1
    np.set_printoptions(precision=4)

    rclpy.init()
    node = rclpy.create_node("edg_experiment")
    try:
        ft_help = FT_CallbackHelp(node)
        time.sleep(0.5)
        file_help = fileSaveHelp()
        time.sleep(0.5)

        sync_pub = node.create_publisher(Int8, "sync", 1)
        data_logger_client = node.create_client(Enable, "data_logging")
        wait_for_data_logger(node, data_logger_client)

        call_enable_service(node, data_logger_client, False)
        time.sleep(1)
        file_help.clearTmpFolder()

        input("Press <Enter> to go to set bias")
        try:
            ft_help.setNowAsBias()
            time.sleep(0.1)
        except Exception:
            print("set now as offset failed, but it is okay")

        input("Press <Enter> to start to record data")
        for _ in range(5):
            sync_pub.publish(Int8(data=sync_start))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.1)

        logging_response = call_enable_service(node, data_logger_client, True)
        if not logging_response.output_file_name.strip():
            raise RuntimeError(
                "Data logger did not create any CSV files. Check that topics in "
                "config/TopicsList.txt are currently published before recording."
            )
        time.sleep(0.2)

        start_time = time.time()
        while (time.time() - start_time) < 5:
            rclpy.spin_once(node, timeout_sec=0.0)
            sync_pub.publish(Int8(data=sync_start))
            time.sleep(0.1)

        args.currentTime = datetime.now().strftime("%H%M%S")

        call_enable_service(node, data_logger_client, False)
        time.sleep(0.2)

        file_help.saveDataParams(
            args,
            appendTxt="Simple_data_log_" + "argument(int)_" + str(args.int) + "_argument(code)_" + str(args.currentTime),
        )
        file_help.clearTmpFolder()
        print("============ Python UR_Interface demo complete!")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--int", type=int, help="argument for int type", default=100)
    parser.add_argument("--str", type=str, help="argument for str type", default="string")
    parser.add_argument("--bool", type=bool, help="argument for bool type", default=True)
    main(parser.parse_args())
