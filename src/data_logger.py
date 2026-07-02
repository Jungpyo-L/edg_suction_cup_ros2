#!/usr/bin/env python3
import collections
import numbers
import os
import time
from datetime import datetime
from operator import attrgetter

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from suction_cup.srv import Enable

TOPIC_DISCOVERY_TIMEOUT_SEC = 5.0
TOPIC_DISCOVERY_POLL_SEC = 0.2


class DataLogger(Node):
    def __init__(self):
        super().__init__("edg_log_node")
        self.is_logging_enabled = False
        self.list_of_topics = []
        self.attributes_by_topic = {}
        self.subscribers = []
        self.output_file_name = {}
        self.output_file = {}
        self.all_output_file_names = ""
        self.create_service(Enable, "data_logging", self.set_logging_state)
        self.get_logger().info("DataLogger Started")

    def append_data_point(self, topic, msg):
        data_by_attribute = {}
        for attribute_name in list(self.attributes_by_topic[topic].keys()):
            data_by_attribute[attribute_name] = attrgetter(attribute_name)(msg)

        timestamp = self.get_clock().now().to_msg()
        if hasattr(msg, "header"):
            stamp = msg.header.stamp
            if stamp.sec > 0 or stamp.nanosec > 0:
                timestamp = stamp

        line = f"{timestamp.sec}.{timestamp.nanosec:09d},"
        for attribute_name in list(self.attributes_by_topic[topic].keys()):
            value = data_by_attribute[attribute_name]
            value_str = str(value)
            if self.attributes_by_topic[topic][attribute_name] > 1:
                value_str = value_str[1:-1]
            line += value_str + ","

        self.output_file[topic].write(line[:-1] + "\n")

    def update_attribute_list(self, topic, msg, attribute_name=""):
        if hasattr(msg, "__slots__"):
            for slot_name in msg.__slots__:
                if slot_name == "header":
                    continue
                slot_value = getattr(msg, slot_name)
                self.update_attribute_list(topic, slot_value, attribute_name + "." + slot_name)
        elif isinstance(msg, numbers.Number):
            self.attributes_by_topic.setdefault(topic, {})
            self.attributes_by_topic[topic][attribute_name[1:]] = 1
        elif isinstance(msg, (list, tuple)) and msg and isinstance(msg[0], numbers.Number):
            self.attributes_by_topic.setdefault(topic, {})
            self.attributes_by_topic[topic][attribute_name[1:]] = len(msg)

    def write_file_header(self, topic):
        header = "ROStimestamp,"
        for attribute_name in list(self.attributes_by_topic[topic].keys()):
            count = self.attributes_by_topic[topic][attribute_name]
            if count > 1:
                for index in range(count):
                    header += f"{topic}.{attribute_name}[{index}],"
            else:
                header += f"{topic}.{attribute_name},"
        self.output_file[topic].write(header[:-1] + "\n")

    def topic_callback(self, msg, topic):
        if not self.is_logging_enabled:
            return

        if topic not in self.output_file or self.output_file[topic].closed:
            self.get_logger().warning("File for topic '%s' is already closed or missing. Skipping data." % topic)
            return

        if topic not in self.attributes_by_topic:
            self.update_attribute_list(topic, msg)
            self.attributes_by_topic[topic] = collections.OrderedDict(
                sorted(self.attributes_by_topic[topic].items())
            )
            self.write_file_header(topic)

        self.append_data_point(topic, msg)

    def unsubscribe_all_topics(self):
        for subscription in self.subscribers:
            self.destroy_subscription(subscription)
        self.subscribers.clear()
        self.list_of_topics.clear()
        self.attributes_by_topic.clear()

    def close_output_files(self):
        for file_object in self.output_file.values():
            if not file_object.closed:
                file_object.close()
        self.output_file.clear()
        self.output_file_name.clear()
        self.all_output_file_names = ""

    def read_configured_topics(self, file_path):
        topics = []
        with open(file_path) as config_file:
            for line in config_file:
                topic = line.strip()
                if topic:
                    topics.append(topic)
        return topics

    def get_available_topic_types(self):
        topic_types = self.get_topic_names_and_types()
        return {topic: types[0] for topic, types in topic_types if types}

    def wait_for_configured_topic_types(self, configured_topics):
        deadline = time.monotonic() + TOPIC_DISCOVERY_TIMEOUT_SEC
        flattened_topic_types = {}
        while time.monotonic() < deadline:
            flattened_topic_types = self.get_available_topic_types()
            if all(topic in flattened_topic_types for topic in configured_topics):
                break
            time.sleep(TOPIC_DISCOVERY_POLL_SEC)
        return flattened_topic_types

    def load_config_file(self, file_path):
        self.unsubscribe_all_topics()

        configured_topics = self.read_configured_topics(file_path)
        flattened_topic_types = self.wait_for_configured_topic_types(configured_topics)
        self.get_logger().info("Discovered %d topics." % len(flattened_topic_types))

        for topic in configured_topics:
            msg_type = flattened_topic_types.get(topic)
            if msg_type is None:
                self.get_logger().warning(
                    "No matching type found for topic '%s'. Check if it is published or spelled correctly." % topic
                )
                continue

            try:
                msg_class = get_message(msg_type)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                self.get_logger().error("Could not load message class for %s: %s" % (msg_type, exc))
                continue

            subscription = self.create_subscription(
                msg_class,
                topic,
                lambda msg, topic=topic: self.topic_callback(msg, topic),
                10,
            )
            self.list_of_topics.append(topic)
            self.subscribers.append(subscription)
            self.get_logger().info("Subscribed to '%s' with type '%s'" % (topic, msg_type))

    def set_logging_state(self, request, response):
        desired_state = request.enable_data_logging

        if desired_state:
            if self.is_logging_enabled != desired_state:
                self.close_output_files()
                config_path = os.path.join(
                    get_package_share_directory("suction_cup"),
                    "config",
                    "TopicsList.txt",
                )
                self.load_config_file(config_path)
                self.get_logger().info("Listening for these topics: %s" % str(self.list_of_topics))

                if not self.list_of_topics:
                    self.get_logger().error(
                        "Data logging was not started because none of the configured topics are available."
                    )
                    self.is_logging_enabled = False
                    response.output_file_name = ""
                    return response

                current_time = datetime.now().strftime("%Y_%m%d_%H%M%S")
                self.all_output_file_names = ""
                for topic_name in self.list_of_topics:
                    safe_topic_name = topic_name.replace("/", "_")
                    self.output_file_name[topic_name] = f"/tmp/dataLog_{current_time}_{safe_topic_name}.csv"
                    self.output_file[topic_name] = open(self.output_file_name[topic_name], "w")
                    self.get_logger().info(
                        "Writing output for %s to: %s" % (topic_name, self.output_file_name[topic_name])
                    )
                    self.all_output_file_names += self.output_file_name[topic_name] + " "

                self.get_logger().info("Data logging is started.")
            else:
                self.get_logger().info("Data logging was already enabled.")
            self.is_logging_enabled = True
            response.output_file_name = self.all_output_file_names
            return response

        if self.is_logging_enabled != desired_state:
            self.close_output_files()
            self.unsubscribe_all_topics()
            self.get_logger().info("Data logging is stopped.")
        else:
            self.get_logger().info("Data logging was already disabled.")

        self.is_logging_enabled = False
        response.output_file_name = self.all_output_file_names
        return response


def main():
    rclpy.init()
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.unsubscribe_all_topics()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
