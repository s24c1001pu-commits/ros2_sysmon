import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import psutil


class SysMon(Node):
    def __init__(self):
        super().__init__('sysmon')

        self.publisher_ = self.create_publisher(
            DiagnosticArray, 'sysmon', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        # CPU使用率 初期化（これが超重要）
        psutil.cpu_percent(interval=None)

    def timer_callback(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        stat = DiagnosticStatus()
        stat.name = 'ros2_sysmon'
        stat.hardware_id = 'local_pc'
        stat.message = 'OK'

        cpu = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory().percent
        disk = psutil.disk_usage('/').percent

        stat.values.append(KeyValue(
            key='cpu_usage_percent', value=f'{cpu:.1f}'))
        stat.values.append(KeyValue(
            key='mem_usage_percent', value=f'{mem:.1f}'))
        stat.values.append(KeyValue(
            key='disk_usage_percent', value=f'{disk:.1f}'))

        msg.status.append(stat)
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = SysMon()
    rclpy.spin(node)
    rclpy.shutdown()

