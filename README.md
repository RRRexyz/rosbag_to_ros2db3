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
| `--keep-invalid-timestamps` | 保留时间戳早于 2000-01-01 的消息（默认丢弃） |
| `--sort-points` | 把每个 Livox CustomMsg 里的点按 `offset_time` 排序，保证帧内时间单调 |

默认只转换 `sensor_msgs/msg/CompressedImage`、`sensor_msgs/msg/Imu` 和两种 Livox
`CustomMsg`（`livox_ros_driver/msg/CustomMsg`、`livox_ros_driver2/msg/CustomMsg`），其余话题会被
跳过并打日志说明。

退出码：`0` 成功、`1` 转换失败、`2` 参数或路径有问题。

转换是逐条流式写入的，内存占用与包大小无关，几 GB 的包也能转。

## 时间戳校验

有些采集程序会在包开头写入几条时间戳为 0（1970 年）、内容为空的帧（常见于 Livox 雷达）。
这类消息一旦写进 rosbag2，整包的 `starting_time` 就被拉到 1970 年，`ros2 bag play` 会先空等
几十年才发布第一条数据。

转换默认把时间戳早于 **2000-01-01** 的消息整条丢弃，并按话题打印丢弃条数：

```
[WARNING] dropped 4 message(s) from /livox/lidar: timestamp before 2000-01-01
```

确实需要原样保留（比如要排查原始数据）时加 `--keep-invalid-timestamps`。

## Livox 激光雷达

ROS 2 的 Livox 驱动改名为 `livox_ros_driver2`，消息字段和 ROS 1 的 `livox_ros_driver` 完全一致，
只差包名。所以包里 `livox_ros_driver/msg/CustomMsg` 的话题会被自动写成
`livox_ros_driver2/msg/CustomMsg`，`timebase`、`point_num`、`lidar_id`、`rsvd` 以及每个点的
`offset_time`/`x`/`y`/`z`/`reflectivity`/`tag`/`line` 都逐字节保持不变，ROS 2 里用
`livox_ros_driver2` 的订阅者直接就能读：

```bash
bagcon hall01.bag -t /livox/lidar -o hall01_ros2
```

`livox_ros_driver2` 的 `.msg` 定义随包分发（`src/rosbag_to_ros2db3/msg/livox_ros_driver2/msg/`），
不需要另外装驱动包，也不需要 `--register`。取 ROS 1 bag 里内嵌的定义反序列化，再按 ROS 2 的定义
写 CDR，所以 `Header.seq` 会按格式差异丢掉，其余字段不变。

如果包里的类型**本来就是** `livox_ros_driver2/msg/CustomMsg`（例如从 ROS 2 转回 ROS 1 的包），
转换时类型名原样保留，不做任何改名。

### 帧内 `offset_time` 回跳

部分录制里，同一帧的点时间并不单调：点的 `offset_time` 在帧内会往回跳几百微秒。这不是转换
引入的——源 bag 里就是如此。典型成因是录制端 Livox 驱动给每个数据包打的时间戳抖动较大
（`livox_ros_driver2` 在 1.2.1 版修过 CustomMsg 的 offset time），转换只是原样搬运。

实际数据长这样：每帧由 96 个一包的点组成（每包内间隔恒定为几微秒），但包与包之间的起点
时间会前后错开，于是拼接后出现回跳。单帧 2 万个点里大约 130 处回跳，最大约 0.45 ms。

对建图的影响通常很小：每个点仍然带自己的 `offset_time`，FAST-LIO2、Point-LIO 这类按点
时间做去畸变的算法不要求点按时间排序。但如果下游代码假设"点按时间递增"（例如用相邻点
时间差计算速度、二分查找、按时间做 range image 分箱），负的时间差会出问题。

需要单调时间时加 `--sort-points`：对每帧的点按 `offset_time` 稳定排序，点的时刻、坐标、
属性都不改，只是顺序变了。默认不开，保证输出与源包逐点同序。

```bash
bagcon 大礼堂.bag -t /livox/lidar --sort-points
```

## 自定义消息类型

`--distro` 选定的 typestore 里没有的类型默认会被跳过。要转换它们，用 `--register` 提供一个
ROS 2 语法的 `.msg` 文件（字段类型在前、字段名在后）：

```bash
bagcon drive.bag --all-types \
  --register my_ros1_pkg/msg/Foo=msg/Foo.msg \
  --register my_ros1_pkg/msg/Bar=msg/Bar.msg
```

依赖的标准消息类型（如 `std_msgs/Header`）由 typestore 提供，不需要重复注册。类型名要和包里
连接记录的类型名一致；如果输出的类型名和 ROS 1 里的不同（像 Livox 那样改名），用 ROS 2 侧的名字
注册，转换时会按目标名字查定义。

## 已知限制

- **输出是 rosbag2 元数据版本 9**（Jazzy/Rolling 时代）。`rosbags` 只能写 v8 或 v9，
  老版本 rosbag2（Humble 及更早，最高 v7）会拒绝加载。如果目标机是 Humble，需要用
  `ros2 bag convert` 之类的工具再降级，或者改用目标版本原生工具链。
- ROS 1 的 `std_msgs/Header.seq` 在 ROS 2 里不存在，转换时会被丢弃，这是格式差异导致的。
- ROS 1 bag 里没有 QoS 信息，输出话题的 `offered_qos_profiles` 为空，播放时按默认 QoS 处理。
- 同一个话题上出现两种不同的消息类型属于异常数据，转换保留第一个类型并打警告。
