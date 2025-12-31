#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Takuto
# SPDX-License-Identifier: BSD-3-Clause

import shutil
import time

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def read_cpu_usage_percent(interval_s: float = 0.1) -> float:
    """
    CPU使用率(%)を /proc/stat から概算
    """
    def read_stat():
        with open("/proc/stat", "r", encoding="utf-8") as f:
            line = f.readline().strip()
        parts = line.split()
        vals = list(map(int, parts[1:]))  # user nice system idle iowait irq softirq steal ...
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        total = sum(vals)
        return total, idle

    total1, idle1 = read_stat()
    time.sleep(interval_s)
    total2, idle2 = read_stat()

    dt = total2 - total1
    didle = idle2 - idle1
    if dt <= 0:
        return 0.0
    usage = (dt - didle) / dt * 100.0
    return max(0.0, min(100.0, usage))


def read_mem_usage_percent() -> float:
    """
    メモリ使用率(%)を /proc/meminfo から概算
    """
    mem_total = None
    mem_avail = None
    with open("/proc/meminfo", "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("MemTotal:"):
                mem_total = int(line.split()[1])  # kB
            elif line.startswith("MemAvailable:"):
                mem_avail = int(line.split()[1])  # kB
    if mem_total is None or mem_avail is None or mem_total == 0:
        return 0.0
    used = mem_total - mem_avail
    return used / mem_total * 100.0


def read_disk_usage_percent(path: str = "/") -> float:
    usage = shutil.disk_usage(path)
    if usage.total == 0:
        return 0.0
    return usage.used / usage.total * 100.0


class SysMon(Node):
    def __init__(self):
        super().__init__("sysmon")
        self.pub = self.create_publisher(DiagnosticArray, "sysmon", 10)
        self.timer = self.create_timer(1.0, self.cb)

    def cb(self):
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

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SysMon()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

