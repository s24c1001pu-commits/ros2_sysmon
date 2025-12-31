#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Takuto
# SPDX-License-Identifier: BSD-3-Clause

"""ROS 2 system monitor node.

This node publishes CPU, memory, and disk usage information
as diagnostic messages.
"""

import shutil
import time

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def read_cpu_usage_percent(interval_s: float = 0.1) -> float:
    """Read CPU usage percentage from /proc/stat."""
    def read_stat():
        with open("/proc/stat", "r", encoding="utf-8") as f:
            line = f.readline().strip()
        parts = line.split()
        values = list(map(int, parts[1:]))
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        total = sum(values)
        return total, idle

    total1, idle1 = read_stat()
    time.sleep(interval_s)
    total2, idle2 = read_stat()

    delta_total = total2 - total1
    delta_idle = idle2 - idle1

    if delta_total <= 0:
        return 0.0

    usage = (delta_total - delta_idle) / delta_total * 100.0
    return max(0.0, min(100.0, usage))


def read_mem_usage_percent() -> float:
    """Read memory usage percentage from /proc/meminfo."""
    mem_total = None
    mem_available = None

    with open("/proc/meminfo", "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("MemTotal:"):
                mem_total = int(line.split()[1])
            elif line.startswith("MemAvailable:"):
                mem_available = int(line.split()[1])

    if mem_total is None or mem_available is None or mem_total == 0:
        return 0.0

    used = mem_total - mem_available
    return used / mem_total * 100.0


def read_disk_usage_percent(path: str = "/") -> float:
    """Read disk usage percentage."""
    usage = shutil.disk_usage(path)
    if usage.total == 0:
        return 0.0
    return usage.used / usage.total * 100.0


class SysMon(Node):
    """ROS 2 node for system monitoring."""

    def __init__(self):
        """Initialize the system monitor node."""
        super().__init__("sysmon")
        self.publisher = self.create_publisher(
            DiagnosticArray, "sysmon", 10
        )
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        """Publish system status as DiagnosticArray."""
        cpu = read_cpu_usage_percent()
        mem = read_mem_usage_percent()
        disk = read_disk_usage_percent("/")

        status = DiagnosticStatus()
        status.name = "ros2_sysmon"
        status.hardware_id = "local_pc"
        status.level = DiagnosticStatus.OK
        status.message = "OK"
        status.values = [
            KeyValue(key="cpu_usage_percent", value=f"{cpu:.1f}"),
            KeyValue(key="mem_usage_percent", value=f"{mem:.1f}"),
            KeyValue(key="disk_usage_percent", value=f"{disk:.1f}"),
        ]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]

        self.publisher.publish(msg)


def main():
    """Main entry point for ros2_sysmon."""
    rclpy.init()
    node = SysMon()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

