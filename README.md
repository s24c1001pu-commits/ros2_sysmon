# ros2_sysmon

CPU 使用率を定期的に取得し、ROS 2 トピックとして配信するシンプルなシステムモニタノードです。

## Usage

ビルド後、以下のコマンドでノードを起動します。

```bash
source install/setup.bash
ros2 run ros2_sysmon sysmon
```

状態情報は sysmon トピックに publish されます。

ros2 topic echo /sysmon

## Description

- CPU 使用率を取得し、定期的に publish します
- 軽量なシステムモニタとして動作します
- ROS 2 の学習用サンプルとして利用できます

## Topic

| Topic name | Type | Description |
|-----------|------|-------------|
| `sysmon` | `std_msgs/msg/Float32` | CPU usage (%) |

## Installation

ワークスペースにパッケージを配置し、ビルドします。

cd ~/ros2_ws/src
git clone https://github.com/s24c1001pu-commits/ros2_sysmon.git
cd ~/ros2_ws
colcon build



## Implementation

Python (rclpy) により実装

/proc/stat, /proc/meminfo を利用

CPU 使用率の計測間隔は interval = 0.1 秒

## License

This software is released under the BSD 3-Clause License.

© 2025 Takuto

## Acknowledgements

本パッケージは、講義  
**「ロボットシステム学 2025」**  
における ROS 2 の学習内容を参考にして作成しました。
README.mdREADME.md
