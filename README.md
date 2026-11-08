# rosbag-to-ros2db3

把 ROS 1 的 `.bag` 包转换成 ROS 2 的 rosbag2（sqlite3 / `.db3`）格式，基于纯 Python 的
[rosbags](https://pypi.org/project/rosbags/)，不需要安装 ROS 1 或 ROS 2。

## 安装

```bash
uv sync                 # 或 pip install -e .
```

需要 Python ≥ 3.10（`rosbags` 的要求）。

## 用法

```bash
bagcon drive.bag                                  # 输出到 ./drive_ros2
bagcon drive.bag -o /data/drive_ros2 -f           # 指定输出目录，已存在则覆盖
bagcon drive.bag -t /imu /camera/image/compressed # 只转指定话题
bagcon drive.bag --all-types                      # 转 typestore 已知的全部类型
```

也可以用 `python -m rosbag_to_ros2db3 ...`。

| 参数 | 说明 |
| --- | --- |
| `src` | ROS 1 bag 路径（必填） |
| `-o, --output` | 输出目录，默认 `<bag 所在目录>/<bag 名>_ros2` |
| `-f, --force` | 输出目录已存在时先删除；不删则直接报错退出 |
| `-t, --topics` | 只转换这些话题，可写多个 |
| `--all-types` | 转换 typestore 认识的所有类型，而不是默认的两个 |
| `--distro` | 用哪个 ROS 2 版本的消息定义，默认 `foxy` |
| `--register TYPE=PATH` | 为自定义类型注册 `.msg` 定义，可重复 |
| `--no-progress` | 关闭进度条，适合脚本和日志 |

默认只转换 `sensor_msgs/msg/CompressedImage` 和 `sensor_msgs/msg/Imu`，其余话题会被跳过并
打日志说明。

退出码：`0` 成功、`1` 转换失败、`2` 参数或路径有问题。

转换是逐条流式写入的，内存占用与包大小无关，几 GB 的包也能转。

## 自定义消息类型

`--distro` 选定的 typestore 里没有的类型默认会被跳过。要转换它们，用 `--register` 提供一个
ROS 2 语法的 `.msg` 文件（字段类型在前、字段名在后）：

```bash
bagcon livox.bag --all-types \
  --register livox_ros_driver/msg/CustomMsg=msg/CustomMsg.msg \
  --register livox_ros_driver/msg/CustomPoint=msg/CustomPoint.msg
```

依赖的标准消息类型（如 `std_msgs/Header`）由 typestore 提供，不需要重复注册。

## 已知限制

- **输出是 rosbag2 元数据版本 9**（Jazzy/Rolling 时代）。`rosbags` 只能写 v8 或 v9，
  老版本 rosbag2（Humble 及更早，最高 v7）会拒绝加载。如果目标机是 Humble，需要用
  `ros2 bag convert` 之类的工具再降级，或者改用目标版本原生工具链。
- ROS 1 的 `std_msgs/Header.seq` 在 ROS 2 里不存在，转换时会被丢弃，这是格式差异导致的。
- ROS 1 bag 里没有 QoS 信息，输出话题的 `offered_qos_profiles` 为空，播放时按默认 QoS 处理。
- 同一个话题上出现两种不同的消息类型属于异常数据，转换保留第一个类型并打警告。

## 测试

```bash
uv sync --group dev
uv run pytest
```

测试会自己构造 ROS 1 包、转换后再用 `AnyReader` 回读逐字段校验。
