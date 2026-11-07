import argparse
import logging
from pathlib import Path
from rosbags.highlevel import AnyReader

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(filename)s:%(lineno)d - %(message)s'
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("src")
    args = parser.parse_args()
    logging.info(f"加载 {args.src} 成功。")

    with AnyReader([Path(args.src)]) as reader:
        for topic, info in reader.topics.items():
            logging.info(f"话题{topic} 消息类型{info.msgtype} 消息数量{info.msgcount}")
