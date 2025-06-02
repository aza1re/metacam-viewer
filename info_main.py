from pathlib import Path
import sys
from typing import List

from reader import MetaCamEduReader, MetaCamEduSyncedMsgs


def main():
    if len(sys.argv) < 2:
        print("Usage: python info_main.py <path_to_data_folder>")
        return

    path = Path(sys.argv[1])

    with MetaCamEduReader(path) as reader:
        print("Topics:")
        print(reader.topic_table())

        msgs: MetaCamEduSyncedMsgs
        _, msgs = next(reader.synced_msg_with_framerate(1.0))

        print("Lidar fields:")
        print(msgs.lidar_field_table())


if __name__ == "__main__":
    main()
