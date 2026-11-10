"""读取 ROS 1 bag 包并转换为 ROS 2 (rosbag2 / sqlite3) 格式。

转换流程分三步，且严格按此顺序，避免"先干完活再报错"：

1. 校验输入、输出路径（输出已存在且未加 ``--force`` 时立刻失败）；
2. 打开 ROS 1 bag，筛选话题/类型，确定最终要写出的连接列表；
3. 用 ``Writer`` 流式写入：读一条、反序列化一条、序列化一条、写一条。

第 3 步不缓存消息，因此内存占用与包大小无关。写入过程中任何异常都会删除
半成品输出目录，避免下一次运行被残留目录挡住，也避免留下"看起来正常、
实际只有一半消息"的 bag。

时间戳早于 2000-01-01 的消息默认丢弃：有些采集程序会在包开头写入时间戳为 0
的空帧，留着会把 rosbag2 的起始时间拉到 1970 年，播放时要空等几十年。
``--keep-invalid-timestamps`` 可以保留这些消息。

livox_ros_driver 只有 ROS 1 版本，对应的 ROS 2 驱动改名为 livox_ros_driver2，
消息定义字段一致但类型名不同。包里遇到 ``livox_ros_driver/msg/CustomMsg`` 时
会按 ``livox_ros_driver2/msg/CustomMsg`` 写出（定义随包分发，见 ``BUNDLED_TYPES``），
这样 ROS 2 侧的 livox_ros_driver2 和配套 rviz 插件可以直接读。

部分 Livox 录制里单帧内部的 ``offset_time`` 会往回跳：点按 96 个一包拼接，
包内时间严格递增，但包与包之间的时间戳抖动很大，后一包的起点可能早于前一包的
终点。这是录制端的时间戳问题（livox_ros_driver2 1.2.1 修过 CustomMsg 的
offset time），转换只能原样搬运。``--sort-points`` 会把每帧的点按 ``offset_time``
稳定排序，让帧内时间单调，点的时刻本身不变。
"""

# pyright: reportAttributeAccessIssue=false
# pyright: reportArgumentType=false
from __future__ import annotations

import argparse
import dataclasses
import logging
import shutil
import sys
from collections.abc import Collection, Iterable, Sequence
from importlib.resources import files
from pathlib import Path
from typing import Any, cast

from rosbags.highlevel import AnyReader
from rosbags.interfaces import Connection
from rosbags.rosbag2 import Writer
from rosbags.rosbag2.errors import WriterError
from rosbags.typesys import Stores, TypesysError, get_types_from_msg, get_typestore
from rosbags.typesys.store import Typestore
from tqdm import tqdm

LOGGER: logging.Logger = logging.getLogger("bagcon")

#: 消息类型名，形如 ``sensor_msgs/msg/Imu``。
MsgType = str

#: 一条消息的原始读数和元信息：(连接, 时间戳[ns], 序列化负载)。
RawMessage = tuple[Connection, int, bytes]

#: ROS 1 类型名 -> ROS 2 类型名。字段布局一致、只是包名不同的类型放这里。
TYPE_ALIASES: dict[MsgType, MsgType] = {
    # livox_ros_driver 没有 ROS 2 版本，ROS 2 驱动是 livox_ros_driver2
    "livox_ros_driver/msg/CustomMsg": "livox_ros_driver2/msg/CustomMsg",
}

#: 随包分发的消息定义：(类型名, 包内路径片段)。按依赖顺序注册。
BUNDLED_TYPES: tuple[tuple[MsgType, tuple[str, ...]], ...] = (
    (
        "livox_ros_driver2/msg/CustomPoint",
        ("msg", "livox_ros_driver2", "msg", "CustomPoint.msg"),
    ),
    (
        "livox_ros_driver2/msg/CustomMsg",
        ("msg", "livox_ros_driver2", "msg", "CustomMsg.msg"),
    ),
)

#: 默认转换的消息类型，用 ``--all-types`` 放开到 typestore 已知的全部类型。
#: livox 两个包名都收：ROS 1 原始数据是 ``livox_ros_driver``，从 ROS 2 转回来的
#: 包则可能已经是 ``livox_ros_driver2``，后者的类型名原样写出、不做改名。
DEFAULT_TYPES: frozenset[MsgType] = frozenset(
    {
        "sensor_msgs/msg/CompressedImage",
        "sensor_msgs/msg/Imu",
        "livox_ros_driver/msg/CustomMsg",
        "livox_ros_driver2/msg/CustomMsg",
    }
)

#: 读取 ROS 1 包时的兜底 typestore（包内未内嵌消息定义时使用）。
ROS1_TYPESTORE: Stores = Stores.ROS1_NOETIC

#: ``--distro`` 取值到 rosbags typestore 的映射。
DISTROS: dict[str, Stores] = {
    "foxy": Stores.ROS2_FOXY,
    "galactic": Stores.ROS2_GALACTIC,
    "humble": Stores.ROS2_HUMBLE,
    "iron": Stores.ROS2_IRON,
    "jazzy": Stores.ROS2_JAZZY,
    "kilted": Stores.ROS2_KILTED,
    "rolling": Stores.LATEST,
}

EXIT_OK = 0
EXIT_FAILED = 1
EXIT_USAGE = 2
EXIT_INTERRUPTED = 130

#: 时间戳早于此值的消息视为无效（2000-01-01T00:00:00Z）。
#: 有些采集程序会在包开头写入时间戳为 0（1970 年）、内容为空的帧，
#: 这类消息会把 rosbag2 的起始时间拉到 1970 年，播放时要空等几十年，
#: 所以默认丢弃；``--keep-invalid-timestamps`` 可以关掉这个过滤。
MIN_VALID_TIMESTAMP_NS: int = 946_684_800_000_000_000


def build_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="bagcon",
        description="Convert a ROS 1 bag into a ROS 2 (rosbag2/sqlite3) bag.",
        epilog=(
            "livox_ros_driver/msg/CustomMsg is written as "
            "livox_ros_driver2/msg/CustomMsg, because the ROS 2 Livox driver "
            "uses the livox_ros_driver2 package name; bags that already carry "
            "livox_ros_driver2/msg/CustomMsg keep that type name."
        ),
    )
    parser.add_argument("src", type=Path, help="path to the ROS 1 .bag file")
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="output rosbag2 directory (default: <src dir>/<src stem>_ros2)",
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="replace the output directory if it already exists",
    )
    parser.add_argument(
        "-t",
        "--topics",
        nargs="+",
        metavar="TOPIC",
        help="only convert these topics (default: all topics of the selected types)",
    )
    parser.add_argument(
        "--all-types",
        action="store_true",
        help="convert every message type known to the ROS 2 typestore",
    )
    parser.add_argument(
        "--distro",
        choices=sorted(DISTROS),
        default="foxy",
        help="ROS 2 distribution whose message definitions are used (default: foxy)",
    )
    parser.add_argument(
        "--register",
        action="append",
        default=[],
        metavar="TYPE=PATH",
        help=(
            "register a custom message type with a ROS 2 .msg file, "
            "e.g. --register livox_ros_driver/msg/CustomMsg=msg/CustomMsg.msg "
            "(repeatable)"
        ),
    )
    parser.add_argument(
        "--no-progress",
        action="store_true",
        help="disable the progress bar (useful for logs and automation)",
    )
    parser.add_argument(
        "--keep-invalid-timestamps",
        action="store_true",
        help=(
            "keep messages whose timestamp is before 2000-01-01 "
            "(dropped by default: they drag the bag start back to 1970 "
            "and make playback wait)"
        ),
    )
    parser.add_argument(
        "--sort-points",
        action="store_true",
        help=(
            "stable-sort the points of every Livox CustomMsg by offset_time, "
            "so the per-frame times are monotonic (some recordings have "
            "jittery packet timestamps that make offset_time jump backwards)"
        ),
    )
    return parser


def default_output(src: Path) -> Path:
    """未指定 ``--output`` 时的默认输出目录。"""
    return src.parent / f"{src.stem}_ros2"


def register_types(store: Typestore, specs: Iterable[str]) -> None:
    """把 ``--register TYPE=PATH`` 指定的自定义类型注册进 typestore。"""
    for spec in specs:
        typename: str
        file_arg: str
        typename, sep, file_arg = spec.partition("=")
        if not sep or not typename or not file_arg:
            raise ValueError(f"bad --register value {spec!r}, expected TYPE=PATH")
        path = Path(file_arg)
        if not path.is_file():
            raise ValueError(f"message definition not found: {path}")
        # .msg 文件按 ROS 2 语法书写（字段类型在前，字段名在后）
        store.register(get_types_from_msg(path.read_text(encoding="utf-8"), typename))
        LOGGER.info("registered %s from %s", typename, path)


def register_bundled_types(store: Typestore) -> None:
    """注册随包分发的消息定义（目前是 livox_ros_driver2 的 CustomMsg/CustomPoint）。"""
    package = __package__ or __name__
    for typename, parts in BUNDLED_TYPES:
        try:
            text = files(package).joinpath(*parts).read_text(encoding="utf-8")
        except OSError as exc:  # 定义文件缺失时不致命，后续按"类型未知"跳过
            LOGGER.warning("bundled definition for %s is missing: %s", typename, exc)
            continue
        store.register(get_types_from_msg(text, typename))


def sort_points_by_offset(msg: Any) -> Any:
    """把一帧内的点按 ``offset_time`` 稳定排序，返回新的消息对象。

    只处理带 ``points`` 列表、且点上有 ``offset_time`` 字段的消息（Livox
    CustomMsg 就是这种）；其它消息原样返回。排序不改变任何点的时刻，只改
    顺序，因此帧内 ``offset_time`` 变成单调递增。
    """
    points = getattr(msg, "points", None)
    if not isinstance(points, list) or len(points) < 2:
        return msg
    if not all(hasattr(point, "offset_time") for point in points):
        return msg
    if not dataclasses.is_dataclass(msg):
        return msg
    ordered = sorted(points, key=lambda point: point.offset_time)
    return dataclasses.replace(cast("Any", msg), points=ordered)


def select_connections(
    connections: Iterable[Connection],
    types: Collection[MsgType] | None,
    topics: Collection[str] | None,
) -> tuple[list[Connection], dict[str, MsgType]]:
    """按消息类型与话题筛选连接，返回 (选中的连接, 被跳过的话题 -> 类型)。"""
    selected: list[Connection] = []
    skipped: dict[str, MsgType] = {}
    for conn in connections:
        if topics is not None and conn.topic not in topics:
            continue
        if types is not None and conn.msgtype not in types:
            skipped[conn.topic] = conn.msgtype
            continue
        selected.append(conn)
    return selected, skipped


def resolve_connections(
    connections: Iterable[Connection], ros2_store: Typestore
) -> list[tuple[Connection, MsgType]]:
    """确定可写出的 (连接, ROS 2 类型)，丢掉不认识的类型和同名冲突话题。

    返回的目标类型经过 ``TYPE_ALIASES`` 改名，例如 livox_ros_driver 的
    ``CustomMsg`` 会以 ``livox_ros_driver2/msg/CustomMsg`` 写出。
    """
    resolvable: list[tuple[Connection, MsgType]] = []
    for conn in connections:
        out_type: MsgType = TYPE_ALIASES.get(conn.msgtype, conn.msgtype)
        try:
            ros2_store.generate_msgdef(out_type, ros_version=2)
        except TypesysError as exc:
            LOGGER.warning(
                "skipping %s: %s (use --register to add the definition)",
                conn.topic,
                exc,
            )
            continue
        resolvable.append((conn, out_type))

    kept: dict[str, tuple[Connection, MsgType]] = {}
    for conn, out_type in resolvable:
        previous = kept.get(conn.topic)
        if previous is not None:
            LOGGER.warning(
                "topic %s appears twice (%s, %s); keeping %s",
                conn.topic,
                previous[1],
                out_type,
                previous[1],
            )
            continue
        kept[conn.topic] = (conn, out_type)
    return list(kept.values())


def convert(
    src: Path,
    dest: Path,
    *,
    ros2_store: Typestore,
    types: Collection[MsgType] | None,
    topics: Collection[str] | None,
    progress: bool = True,
    drop_invalid: bool = True,
    sort_points: bool = False,
) -> int:
    """执行转换，返回进程退出码。

    ``drop_invalid`` 为真时丢弃时间戳早于 ``MIN_VALID_TIMESTAMP_NS`` 的消息，
    并逐个话题打印丢弃条数。``sort_points`` 为真时对每个 Livox CustomMsg
    按 ``offset_time`` 排序，保证帧内点时间单调。
    """
    try:
        with AnyReader(
            [src], default_typestore=get_typestore(ROS1_TYPESTORE)
        ) as reader:
            if topics is not None:
                known = {conn.topic for conn in reader.connections}
                for missing in sorted(set(topics) - known):
                    LOGGER.warning("topic not found in bag: %s", missing)

            connections, skipped = select_connections(reader.connections, types, topics)
            for topic, msgtype in sorted(skipped.items()):
                LOGGER.info("skipping %s (%s): type not selected", topic, msgtype)

            targets = resolve_connections(connections, ros2_store)
            if not targets:
                LOGGER.error("no convertible topics in %s", src)
                if skipped:
                    LOGGER.error(
                        "hint: %d topic(s) were skipped by message type, "
                        "use --all-types to include them",
                        len(skipped),
                    )
                return EXIT_FAILED

            total: int = sum(conn.msgcount for conn, _ in targets)
            out_types: dict[int, MsgType] = {
                conn.id: out_type for conn, out_type in targets
            }
            for conn, out_type in targets:
                LOGGER.info(
                    "converting %s (%s -> %s, %d messages)",
                    conn.topic,
                    conn.msgtype,
                    out_type,
                    conn.msgcount,
                )

            try:
                with Writer(dest, version=Writer.VERSION_LATEST) as writer:
                    out: dict[int, Connection] = {}
                    for conn, out_type in targets:
                        out[conn.id] = writer.add_connection(
                            conn.topic, out_type, typestore=ros2_store
                        )
                    dropped: dict[str, int] = {}
                    written: int = 0
                    with tqdm(
                        total=total,
                        desc="converting",
                        unit="msg",
                        disable=not progress,
                    ) as bar:
                        messages: Iterable[RawMessage] = reader.messages(connections)
                        for conn, timestamp, rawdata in messages:
                            if drop_invalid and timestamp < MIN_VALID_TIMESTAMP_NS:
                                dropped[conn.topic] = (
                                    dropped.get(conn.topic, 0) + 1
                                )
                                bar.update(1)
                                continue
                            msg: Any = reader.deserialize(rawdata, conn.msgtype)
                            if sort_points:
                                msg = sort_points_by_offset(msg)
                            writer.write(
                                out[conn.id],
                                timestamp,
                                ros2_store.serialize_cdr(msg, out_types[conn.id]),
                            )
                            written += 1
                            bar.update(1)

                for topic in sorted(dropped):
                    LOGGER.warning(
                        "dropped %d message(s) from %s: timestamp before 2000-01-01",
                        dropped[topic],
                        topic,
                    )
            except BaseException:
                # 失败时清掉半成品，避免下次运行撞上"目录已存在"
                shutil.rmtree(dest, ignore_errors=True)
                raise
    except WriterError as exc:
        LOGGER.error("%s", exc)
        return EXIT_FAILED

    LOGGER.info("wrote %s (%d messages)", dest, written)
    return EXIT_OK


def main(argv: Sequence[str] | None = None) -> int:
    """命令行入口，返回进程退出码。"""
    args: argparse.Namespace = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        stream=sys.stderr,
    )

    src: Path = args.src
    if not src.exists():
        LOGGER.error("input bag not found: %s", src)
        return EXIT_USAGE

    dest: Path = args.output if args.output is not None else default_output(src)
    if dest.exists():
        if not args.force:
            LOGGER.error("output already exists: %s (use --force to replace it)", dest)
            return EXIT_USAGE
        shutil.rmtree(dest)

    ros2_store: Typestore = get_typestore(DISTROS[args.distro])
    register_bundled_types(ros2_store)
    try:
        register_types(ros2_store, args.register)
    except (ValueError, OSError) as exc:
        LOGGER.error("%s", exc)
        return EXIT_USAGE

    try:
        return convert(
            src,
            dest,
            ros2_store=ros2_store,
            types=None if args.all_types else DEFAULT_TYPES,
            topics=set(args.topics) if args.topics else None,
            progress=not args.no_progress,
            drop_invalid=not args.keep_invalid_timestamps,
            sort_points=args.sort_points,
        )
    except KeyboardInterrupt:
        LOGGER.error("interrupted, removed %s", dest)
        return EXIT_INTERRUPTED
