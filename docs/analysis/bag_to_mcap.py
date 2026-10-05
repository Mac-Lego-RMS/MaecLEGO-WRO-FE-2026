"""Convert a recorded run (.db3) to MCAP for Foxglove, on any computer.

A rosbag2 .db3 file does not contain the message definitions. Foxglove then
falls back to its built-in definitions, and for visualization_msgs/Marker
those do not match ROS 2 Humble (Humble added texture and mesh fields). The
markers of the overlay (/viz/field, /viz/obstacles, /viz/wall_matches, ...)
then fail to decode, so walls and pillars are missing in the 3D panel.

MCAP stores the definition of every topic next to the data. This script
copies every message unchanged (the CDR bytes are not touched) into an MCAP
bag and embeds the ROS 2 Humble definitions plus our own robot_msgs, read
from src/robot_msgs/msg. No ROS installation is needed.

    python docs/analysis/bag_to_mcap.py ~/runs/cw_pos1_22
    python docs/analysis/bag_to_mcap.py ~/runs/cw_pos1_*      # several runs

The result, <bag>_mcap/, opens in Foxglove with File -> Open local file
(select the .mcap inside). Topics whose type is unknown are skipped and
listed.
"""
import argparse
import sys
from pathlib import Path

from rosbags.rosbag2 import Reader, StoragePlugin, Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

REPO = Path(__file__).resolve().parents[2]
OWN_MSGS = REPO / 'src' / 'robot_msgs' / 'msg'


def make_typestore():
    store = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for path in sorted(OWN_MSGS.glob('*.msg')):
        types.update(get_types_from_msg(path.read_text(), f'robot_msgs/msg/{path.stem}'))
    store.register(types)
    return store


def convert(src, dst, store):
    skipped = []
    with Reader(src) as reader, Writer(dst, version=5, storage_plugin=StoragePlugin.MCAP) as writer:
        out = {}
        for conn in reader.connections:
            if conn.msgtype not in store.types:
                skipped.append(f'{conn.topic} ({conn.msgtype})')
                continue
            out[conn.id] = writer.add_connection(
                conn.topic, conn.msgtype, typestore=store,
                serialization_format=conn.ext.serialization_format,
                offered_qos_profiles=conn.ext.offered_qos_profiles)
        n = 0
        for conn, stamp, data in reader.messages():
            if conn.id in out:
                writer.write(out[conn.id], stamp, data)
                n += 1
    return n, skipped


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+', help='bag directories (with metadata.yaml)')
    ap.add_argument('--out-dir', help='where to write; default: next to each bag')
    a = ap.parse_args(argv)
    store = make_typestore()
    for src in map(Path, a.bags):
        dst = (Path(a.out_dir) if a.out_dir else src.parent) / f'{src.name}_mcap'
        if dst.exists():
            print(f'{dst} exists, skipped')
            continue
        n, skipped = convert(src, dst, store)
        print(f'{src.name}: {n} messages -> {dst}')
        for s in skipped:
            print(f'  skipped, unknown type: {s}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
