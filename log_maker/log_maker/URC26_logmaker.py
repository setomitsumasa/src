#!/usr/bin/env python3

import re
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String


class URC26StatusPublisher(Node):
    GPS_MOVING_STATUS = "Moving to a GPS coordinate"
    GPS_REACHED_STATUS = "Reached a GPS coordinate"
    TARGET_SEARCHING_STATUS = "Searching for a target"
    TARGET_DETECTED_STATUS = "Detected a target and approaching it"
    TARGET_REACHED_STATUS = "Reached a target"
    ABORT_STATUS = "Abort: Processing was aborted"
    MISSION_DONE_STATUS = "All tasks completed"

    PHASE_PATTERN = re.compile(r"\bphase=([A-Z0-9_]+)")
    ACTIVE_GOAL_PATTERN = re.compile(r'active_goal="([^"]*)"')
    GPS_COORD_PATTERN = re.compile(r"gps=\(([^,]+),([^)]+)\)")
    ARUCO_ID_PATTERN = re.compile(r"marker_id=(-?\d+)")
    YOLO_TARGET_PATTERN = re.compile(r"(?:target=|target |frame )'([^']+)'")

    def __init__(self):
        super().__init__("urc26_status_publisher")

        self.declare_parameter("mission_status_topic", "/mission/status")
        self.declare_parameter("status_topic", "URC26_statas")

        self.mission_status_topic = (
            self.get_parameter("mission_status_topic").get_parameter_value().string_value
        )
        self.status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        self.publish_period_sec = 1.0

        self.latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.status_publisher = self.create_publisher(
            String, self.status_topic, self.latched_qos
        )
        self.mission_status_subscription = self.create_subscription(
            String,
            self.mission_status_topic,
            self.mission_status_callback,
            self.latched_qos,
        )

        self.current_status = None
        self.last_published_status = None
        self.last_nav_mode = None
        self.current_gps_latitude = None
        self.current_gps_longitude = None
        self.current_target_type = None
        self.current_target_name = None
        self.last_aruco_id = None
        self.last_yolo_target_name = None
        self.skip_periodic_publish_for_status = None
        self.publish_timer = self.create_timer(
            self.publish_period_sec, self.publish_current_status
        )

        self.get_logger().info(
            f"Subscribing {self.mission_status_topic} and publishing summarized status to "
            f"{self.status_topic} at {self.publish_period_sec:.1f} Hz"
        )

    def mission_status_callback(self, msg):
        phase = self.extract_phase(msg.data)
        active_goal = self.extract_active_goal(msg.data)
        mission_message = self.extract_mission_message(msg.data)
        self.update_gps_context(msg.data)
        self.update_target_context(active_goal, mission_message)
        self.apply_phase_target_context(phase)

        if active_goal.startswith("Sending GPS goal"):
            self.last_nav_mode = "gps"
        elif active_goal.startswith("Sending spiral goal") or active_goal == "spiral spin scan":
            self.last_nav_mode = "spiral"
        elif active_goal.startswith("ArUco approach") or active_goal.startswith("YOLO approach"):
            self.last_nav_mode = "target"

        next_status = self.map_status(phase, mission_message)
        if next_status is None:
            return

        if next_status != self.current_status:
            self.get_logger().info(f"Updated URC26 status: {next_status}")
            self.current_status = next_status
            if self.skip_periodic_publish_for_status != next_status:
                self.skip_periodic_publish_for_status = None

        if phase in {"ARUCO_REACHED", "YOLO_REACHED", "NAV2_ABORTED", "NAV2_CANCELED"}:
            self.publish_status_immediately(next_status)

    def publish_current_status(self):
        if self.current_status is None:
            return

        status_text = self.current_status
        if status_text == self.skip_periodic_publish_for_status:
            return

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        status_msg = String()
        status_msg.data = f"{timestamp} {status_text}"
        self.status_publisher.publish(status_msg)
        if status_text != self.last_published_status:
            self.get_logger().info(f"Published URC26 status: {status_msg.data}")
            self.last_published_status = status_text

    def publish_status_immediately(self, status_text):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        status_msg = String()
        status_msg.data = f"{timestamp} {status_text}"
        self.status_publisher.publish(status_msg)
        self.skip_periodic_publish_for_status = status_text
        self.last_published_status = status_text
        self.get_logger().info(f"Published URC26 status immediately: {status_msg.data}")

    def extract_phase(self, mission_status_text):
        match = self.PHASE_PATTERN.search(mission_status_text)
        if match is None:
            return ""
        return match.group(1)

    def extract_active_goal(self, mission_status_text):
        match = self.ACTIVE_GOAL_PATTERN.search(mission_status_text)
        if match is None:
            return ""
        return match.group(1)

    def extract_mission_message(self, mission_status_text):
        _, separator, message = mission_status_text.partition(" | ")
        if not separator:
            return ""
        return message.strip()

    def update_target_context(self, active_goal, mission_message):
        search_text = f"{active_goal} {mission_message}"

        aruco_match = self.ARUCO_ID_PATTERN.search(search_text)
        if aruco_match is not None:
            aruco_id = aruco_match.group(1).strip()
            if aruco_id != "-1":
                self.current_target_type = "aruco"
                self.current_target_name = aruco_id
                self.last_aruco_id = aruco_id
            return

        yolo_match = self.YOLO_TARGET_PATTERN.search(search_text)
        if yolo_match is not None:
            yolo_target_name = yolo_match.group(1).strip()
            if yolo_target_name and yolo_target_name != "disable":
                self.current_target_type = "yolo"
                self.current_target_name = yolo_target_name
                self.last_yolo_target_name = yolo_target_name

    def apply_phase_target_context(self, phase):
        if "ARUCO" in phase and self.last_aruco_id is not None:
            self.current_target_type = "aruco"
            self.current_target_name = self.last_aruco_id
            return

        if "YOLO" in phase and self.last_yolo_target_name is not None:
            self.current_target_type = "yolo"
            self.current_target_name = self.last_yolo_target_name

    def update_gps_context(self, mission_status_text):
        gps_match = self.GPS_COORD_PATTERN.search(mission_status_text)
        if gps_match is None:
            return

        try:
            self.current_gps_latitude = float(gps_match.group(1).strip())
            self.current_gps_longitude = float(gps_match.group(2).strip())
        except ValueError:
            return

    def format_gps_coordinate(self):
        if self.current_gps_latitude is None or self.current_gps_longitude is None:
            return None

        return (
            f"({self.current_gps_latitude:.6f}, "
            f"{self.current_gps_longitude:.6f})"
        )

    def format_gps_moving_status(self):
        gps_coordinate = self.format_gps_coordinate()
        if gps_coordinate is None:
            return self.GPS_MOVING_STATUS
        return f"Moving to GPS coordinate {gps_coordinate}"

    def format_gps_reached_status(self):
        gps_coordinate = self.format_gps_coordinate()
        if gps_coordinate is None:
            return self.GPS_REACHED_STATUS
        return f"Reached GPS coordinate {gps_coordinate}"

    def format_target_search_status(self):
        if self.current_target_type == "aruco" and self.current_target_name:
            return f"Searching for ArUco ID {self.current_target_name}"
        if self.current_target_type == "yolo" and self.current_target_name:
            return f"Searching for YOLO target {self.current_target_name}"
        return self.TARGET_SEARCHING_STATUS

    def format_target_detected_status(self):
        if self.current_target_type == "aruco" and self.current_target_name:
            return f"Detected ArUco ID {self.current_target_name} and approaching it"
        if self.current_target_type == "yolo" and self.current_target_name:
            return f"Detected YOLO target {self.current_target_name} and approaching it"
        return self.TARGET_DETECTED_STATUS

    def format_target_reached_status(self):
        if self.current_target_type == "aruco" and self.current_target_name:
            return f"Reached ArUco ID {self.current_target_name}"
        if self.current_target_type == "yolo" and self.current_target_name:
            return f"Reached YOLO target {self.current_target_name}"
        return self.TARGET_REACHED_STATUS

    def format_abort_status(self, mission_message):
        if mission_message:
            return f"Abort: {mission_message}"
        return self.ABORT_STATUS

    def map_status(self, phase, mission_message):
        if phase == "GPS_GOAL":
            self.last_nav_mode = "gps"
            return self.format_gps_moving_status()

        if phase == "NAV2_ACCEPTED":
            if self.last_nav_mode == "gps":
                return self.format_gps_moving_status()
            if self.last_nav_mode == "spiral":
                return self.format_target_search_status()
            return None

        if phase == "NAV2_REACHED":
            if self.last_nav_mode == "gps":
                return self.format_gps_reached_status()
            if self.last_nav_mode == "spiral":
                return self.format_target_search_status()
            return None

        if phase in {
            "GPS_REACHED_START_SPIRAL",
            "SPIRAL_START",
            "SPIRAL_GOAL",
            "SPIRAL_STEP_REACHED",
            "SPIRAL_SPIN_SCAN_START",
            "SPIRAL_SPIN_SCAN_DONE",
            "TARGET_ACTIVE_ARUCO",
            "TARGET_ACTIVE_YOLO",
        }:
            self.last_nav_mode = "spiral"
            return self.format_target_search_status()

        if phase in {
            "TARGET_VISIBLE_ARUCO",
            "TARGET_VISIBLE_YOLO",
            "TARGET_FOUND_ARUCO",
            "TARGET_FOUND_YOLO",
        }:
            self.last_nav_mode = "target"
            return self.format_target_detected_status()

        if phase in {"WAIT_ARUCO_APPROACH", "WAIT_YOLO_APPROACH"}:
            self.last_nav_mode = "target"
            if "already visible" in mission_message or "Spiral search interrupted" in mission_message:
                return self.format_target_detected_status()
            if "Waiting for" in mission_message:
                return self.format_target_search_status()
            return self.format_target_detected_status()

        if phase in {"ARUCO_REACHED", "YOLO_REACHED"}:
            self.last_nav_mode = None
            return self.format_target_reached_status()

        if phase in {"NAV2_ABORTED", "NAV2_CANCELED"}:
            if self.last_nav_mode == "target":
                return self.format_target_detected_status()
            return self.format_abort_status(mission_message)

        if phase == "MISSION_DONE":
            self.last_nav_mode = None
            self.current_target_type = None
            self.current_target_name = None
            self.last_aruco_id = None
            self.last_yolo_target_name = None
            return self.MISSION_DONE_STATUS

        return None


def main(args=None):
    rclpy.init(args=args)
    node = URC26StatusPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
