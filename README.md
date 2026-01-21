# ros2_sysmon

CPU 使用率を定期的に取得し、ROS 2 トピックとして配信するシンプルなシステムモニタノードです。


## Development process

本ノードは以下の手順で段階的に実装した。

1. rclpy.Node を継承した空のノードを作成し，起動確認を行った
2. Timer を用いて周期処理を実装した
3. CPU 使用率の取得機能を追加した
4. メモリおよびディスク使用率を追加した
5. DiagnosticArray を用いて情報を publish するように拡張した

## Continuous Integration

本リポジトリでは GitHub Actions を用いて，
push 時に自動でビルドおよびテストを実行しています。


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

## Topic

| Topic name | Type | Description |
|-----------|------|-------------|
| `sysmon` | `std_msgs/msg/Float32` | CPU usage (%) |

## Implementation

Python (rclpy) により実装

/proc/stat, /proc/meminfo を利用

CPU 使用率は，約 0.1 秒間隔で取得されます。

## License

This software is released under the BSD 3-Clause License.

© 2025 Takuto

## Acknowledgements

本パッケージは、講義  
**「ロボットシステム学 2025」**  
における ROS 2 の学習内容を参考にして作成しました。
