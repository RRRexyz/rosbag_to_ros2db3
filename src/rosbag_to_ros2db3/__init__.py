"""读取 ROS 1 bag 包并转换为 ROS 2 (rosbag2 / sqlite3) 格式。

转换流程分三步，且严格按此顺序，避免"先干完活再报错"：

1. 校验输入、输出路径（输出已存在且未加 ``--force`` 时立刻失败）；
2. 打开 ROS 1 bag，筛选话题/类型，确定最终要写出的连接列表；
3. 用 ``Writer`` 流式写入：读一条、反序列化一条、序列化一条、写一条。

第 3 步不缓存消息，因此内存占用与包大小无关。写入过程中任何异常都会删除
半成品输出目录，避免下一次运行被残留目录挡住，也避免留下"看起来正常、
实际只有一半消息"的 bag。
"""

# pyright: reportAttributeAccessIssue=false
# pyright: reportArgumentType=false
import argparse
import logging
import shutil
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.rosbag2.errors import WriterError
from rosbags.typesys import Stores, TypesysError, get_types_from_msg, get_typestore
from tqdm import tqdm

LOGGER = logging.getLogger("bagcon")

#: 默认转换的消息类型，用 ``--all-types`` 放开到 typestore 已知的全部类型。
DEFAULT_TYPES = frozenset(
    {
        "sensor_msgs/msg/CompressedImage",
        "sensor_msgs/msg/Imu",
    }
)

#: 读取 ROS 1 包时的兜底 typestore（包内未内嵌消息定义时使用）。
ROS1_TYPESTORE = Stores.ROS1_NOETIC

#: ``--distro`` 取值到 rosbags typestore 的映射。
DISTROS = {
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


def build_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器。"""
    parser = argparse.ArgumentParser(
        prog="bagcon",
        description="Convert a ROS 1 bag into a ROS 2 (rosbag2/sqlite3) bag.",
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
    return parser


def default_output(src: Path) -> Path:
    """未指定 ``--output`` 时的默认输出目录。"""
    return src.parent / f"{src.stem}_ros2"


def register_types(store, specs: list[str]) -> None:
    """把 ``--register TYPE=PATH`` 指定的自定义类型注册进 typestore。"""
    for spec in specs:
        typename, sep, file_arg = spec.partition("=")
        if not sep or not typename or not file_arg:
            raise ValueError(f"bad --register value {spec!r}, expected TYPE=PATH")
        path = Path(file_arg)
        if not path.is_file():
            raise ValueError(f"message definition not found: {path}")
        # .msg 文件按 ROS 2 语法书写（字段类型在前，字段名在后）
        store.register(get_types_from_msg(path.read_text(encoding="utf-8"), typename))
        LOGGER.info("registered %s from %s", typename, path)


def select_connections(connections, types, topics):
    """按消息类型与话题筛选连接，返回 (选中的连接, 被跳过的话题->类型)。"""
    selected = []
    skipped: dict[str, str] = {}
    for conn in connections:
        if topics is not None and conn.topic not in topics:
            continue
        if types is not None and conn.msgtype not in types:
            skipped[conn.topic] = conn.msgtype
            continue
        selected.append(conn)
    return selected, skipped


def resolve_connections(connections, ros2_store):
    """丢掉 typestore 不认识的类型，以及同名但类型冲突的重复话题。"""
    resolvable = []
    for conn in connections:
        try:
            ros2_store.generate_msgdef(conn.msgtype, ros_version=2)
        except TypesysError as exc:
            LOGGER.warning(
                "skipping %s: %s (use --register to add the definition)",
                conn.topic,
                exc,
            )
            continue
        resolvable.append(conn)

    kept: dict[str, object] = {}
    for conn in resolvable:
        previous = kept.get(conn.topic)
        if previous is not None:
            LOGGER.warning(
                "topic %s appears twice (%s, %s); keeping %s",
                conn.topic,
                previous.msgtype,
                conn.msgtype,
                previous.msgtype,
            )
            continue
        kept[conn.topic] = conn
    return list(kept.values())


def convert(
    src: Path,
    dest: Path,
    *,
    ros2_store,
    types,
    topics,
    progress: bool = True,
) -> int:
    """执行转换，返回进程退出码。"""
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

            connections = resolve_connections(connections, ros2_store)
            if not connections:
                LOGGER.error("no convertible topics in %s", src)
                if skipped:
                    LOGGER.error(
                        "hint: %d topic(s) were skipped by message type, "
                        "use --all-types to include them",
                        len(skipped),
                    )
                return EXIT_FAILED

            total = sum(conn.msgcount for conn in connections)
            for conn in connections:
                LOGGER.info(
                    "converting %s (%s, %d messages)",
                    conn.topic,
                    conn.msgtype,
                    conn.msgcount,
                )

            try:
                with Writer(dest, version=Writer.VERSION_LATEST) as writer:
                    out = {}
                    for conn in connections:
                        out[conn.id] = writer.add_connection(
                            conn.topic, conn.msgtype, typestore=ros2_store
                        )
                    with tqdm(
                        total=total,
                        desc="converting",
                        unit="msg",
                        disable=not progress,
                    ) as bar:
                        for conn, timestamp, rawdata in reader.messages(connections):
                            msg = reader.deserialize(rawdata, conn.msgtype)
                            writer.write(
                                out[conn.id],
                                timestamp,
                                ros2_store.serialize_cdr(msg, conn.msgtype),
                            )
                            bar.update(1)
            except BaseException:
                # 失败时清掉半成品，避免下次运行撞上"目录已存在"
                shutil.rmtree(dest, ignore_errors=True)
                raise
    except WriterError as exc:
        LOGGER.error("%s", exc)
        return EXIT_FAILED

    LOGGER.info("wrote %s (%d messages)", dest, total)
    return EXIT_OK


def main(argv=None) -> int:
    """命令行入口，返回进程退出码。"""
    args = build_parser().parse_args(argv)
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

    ros2_store = get_typestore(DISTROS[args.distro])
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
        )
    except KeyboardInterrupt:
        LOGGER.error("interrupted, removed %s", dest)
        return EXIT_INTERRUPTED
