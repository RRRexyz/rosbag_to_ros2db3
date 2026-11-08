"""支持 ``python -m rosbag_to_ros2db3`` 方式调用。"""

import sys

from . import main

if __name__ == "__main__":
    sys.exit(main())
