#!/usr/bin/env python3
"""Publish AquaticVision exported grayscale image pairs as ROS image topics.

The AquaticVision paper reports ORB-SLAM2 results using the exported 30 Hz
grayscale images, not the 20 Hz rosbag image stream used for VINS-Stereo.  This
publisher keeps Exp.3 in ROS-online form while feeding the same image product to
ORB-SLAM3/UW-IDVO.
"""

import argparse
import math
import subprocess
import re
import sys
import time
from pathlib import Path

import cv2
import rosgraph
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


IMAGE_EXTS = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")


def numeric_tokens(text):
    return re.findall(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", text)


def token_to_sec(token):
    token = str(token).strip()
    if not token:
        return None
    nums = numeric_tokens(Path(token).stem)
    if not nums:
        return None
    value = float(nums[-1])
    if abs(value) > 1e12:
        return value * 1e-9
    if abs(value) > 1e6:
        return value * 1e-6
    return value


def image_map(img_dir):
    mapping = {}
    files = []
    for path in sorted(Path(img_dir).iterdir()):
        if not path.is_file() or path.suffix.lower() not in IMAGE_EXTS:
            continue
        files.append(path)
        mapping[path.name] = path
        mapping[path.stem] = path
        sec = token_to_sec(path.stem)
        if sec is not None:
            mapping[f"{sec:.9f}"] = path
            mapping[str(int(round(sec * 1e9)))] = path
    return mapping, files


def resolve_image(mapping, fallback, token, index):
    token = str(token).strip()
    if token in mapping:
        return mapping[token]
    stem = Path(token).stem
    if stem in mapping:
        return mapping[stem]
    sec = token_to_sec(token)
    if sec is not None:
        for key in (f"{sec:.9f}", str(int(round(sec * 1e9)))):
            if key in mapping:
                return mapping[key]
    for ext in IMAGE_EXTS:
        if token + ext in mapping:
            return mapping[token + ext]
    if index < len(fallback):
        return fallback[index]
    raise RuntimeError(f"cannot resolve image token '{token}' at pair index {index}")


def load_pairs(sequence_dir):
    sequence_dir = Path(sequence_dir)
    image_root = sequence_dir / "Stereo images"
    if not (image_root / "l1").is_dir():
        image_root = sequence_dir
    left_map, left_files = image_map(image_root / "l1")
    right_map, right_files = image_map(image_root / "r1")

    pair_file = None
    for name in ("timestamp_pairs.txt", "timestamp_pair.txt", "timestamps.txt", "times.txt"):
        candidate = image_root / name
        if candidate.is_file():
            pair_file = candidate
            break

    pairs = []
    if pair_file is not None:
        for line in pair_file.read_text(errors="ignore").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            toks = re.split(r"[\s,]+", line)
            if len(toks) >= 4:
                left_time, left_token, right_time, right_token = toks[:4]
                stamp = token_to_sec(left_time) or token_to_sec(left_token) or token_to_sec(right_time) or token_to_sec(right_token)
            elif len(toks) == 1:
                left_token = right_token = toks[0]
                stamp = token_to_sec(left_token)
            else:
                left_token, right_token = toks[:2]
                stamp = token_to_sec(left_token) or token_to_sec(right_token)
            index = len(pairs)
            if stamp is None:
                stamp = float(index)
            pairs.append((
                stamp,
                resolve_image(left_map, left_files, left_token, index),
                resolve_image(right_map, right_files, right_token, index),
            ))
    else:
        for index, (left, right) in enumerate(zip(left_files, right_files)):
            stamp = token_to_sec(left.stem) or token_to_sec(right.stem) or float(index)
            pairs.append((stamp, left, right))

    if not pairs:
        raise RuntimeError(f"no image pairs found under {sequence_dir}")
    pairs.sort(key=lambda item: item[0])
    return pairs


def image_msg(path, stamp, frame_id):
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"failed to read image: {path}")
    msg = Image()
    msg.header.stamp = rospy.Time.from_sec(stamp)
    msg.header.frame_id = frame_id
    msg.height, msg.width = image.shape[:2]
    msg.encoding = "mono8"
    msg.is_bigendian = 0
    msg.step = int(msg.width)
    msg.data = image.tobytes()
    return msg


def select_pairs(pairs, start, duration, max_frames):
    first = pairs[0][0]
    begin = first + max(0.0, start)
    end = math.inf if duration <= 0 else begin + duration
    selected = [p for p in pairs if begin <= p[0] <= end]
    if max_frames > 0:
        selected = selected[:max_frames]
    return selected


def ensure_ros_master(auto_start=True, timeout=10.0):
    if rosgraph.is_master_online():
        return None
    if not auto_start:
        raise RuntimeError("ROS master is not running")

    proc = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if rosgraph.is_master_online():
            print("[INFO] started temporary roscore", flush=True)
            return proc
        if proc.poll() is not None:
            raise RuntimeError("temporary roscore exited before becoming available")
        time.sleep(0.2)

    proc.terminate()
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        proc.kill()
    raise RuntimeError("timed out waiting for temporary roscore")


def stop_ros_master(proc):
    if proc is None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        proc.kill()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence-dir", required=True)
    parser.add_argument("--sensor", choices=("mono", "stereo"), required=True)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--warmup-count", type=int, default=15)
    parser.add_argument("--warmup-sleep", type=float, default=0.02)
    parser.add_argument("--left-topic", default="/davis_left/image_raw")
    parser.add_argument("--right-topic", default="/davis_right/image_raw")
    parser.add_argument("--left-frame-id", default="davis_left")
    parser.add_argument("--right-frame-id", default="davis_right")
    parser.add_argument("--stats-only", action="store_true")
    parser.add_argument("--no-auto-roscore", action="store_true")
    args = parser.parse_args()

    pairs = select_pairs(load_pairs(args.sequence_dir), args.start, args.duration, args.max_frames)
    if not pairs:
        raise RuntimeError("no frames selected after applying start/duration/max-frames")
    span = max(0.0, pairs[-1][0] - pairs[0][0]) if len(pairs) > 1 else 0.0
    if args.stats_only:
        print(f"selected_frames={len(pairs)}")
        print(f"first_stamp={pairs[0][0]:.9f}")
        print(f"last_stamp={pairs[-1][0]:.9f}")
        print(f"span_sec={span:.6f}")
        return

    roscore_proc = ensure_ros_master(auto_start=not args.no_auto_roscore)
    try:
        rospy.init_node("aquaticvision_image_pair_publisher", anonymous=True)
        left_pub = rospy.Publisher(args.left_topic, Image, queue_size=5)
        right_pub = rospy.Publisher(args.right_topic, Image, queue_size=5) if args.sensor == "stereo" else None
        clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)

        # Do not use rospy.sleep() here. Exp.3 runs with /use_sim_time=true and
        # this node is the /clock publisher, so simulated time cannot advance until
        # the first Clock message is published.
        time.sleep(0.5)
        first_stamp, first_left, first_right = pairs[0]
        first_ros_stamp = rospy.Time.from_sec(first_stamp)
        first_clock = Clock()
        first_clock.clock = first_ros_stamp
        first_left_msg = image_msg(first_left, first_stamp, args.left_frame_id)
        first_right_msg = image_msg(first_right, first_stamp, args.right_frame_id) if right_pub is not None else None
        for _ in range(max(0, args.warmup_count)):
            if rospy.is_shutdown():
                break
            clock_pub.publish(first_clock)
            left_pub.publish(first_left_msg)
            if right_pub is not None and first_right_msg is not None:
                right_pub.publish(first_right_msg)
            time.sleep(max(0.0, args.warmup_sleep))

        rate_scale = args.rate if args.rate > 0 else 1.0
        wall_prev = time.monotonic()
        stamp_prev = pairs[0][0]

        published = 0
        for stamp, left, right in pairs:
            if rospy.is_shutdown():
                break
            delay = max(0.0, (stamp - stamp_prev) / rate_scale)
            elapsed = time.monotonic() - wall_prev
            if delay > elapsed:
                time.sleep(delay - elapsed)
            ros_stamp = rospy.Time.from_sec(stamp)
            clock_msg = Clock()
            clock_msg.clock = ros_stamp
            clock_pub.publish(clock_msg)
            left_pub.publish(image_msg(left, stamp, args.left_frame_id))
            if right_pub is not None:
                right_pub.publish(image_msg(right, stamp, args.right_frame_id))
            published += 1
            stamp_prev = stamp
            wall_prev = time.monotonic()

        if pairs:
            clock_msg = Clock()
            clock_msg.clock = rospy.Time.from_sec(pairs[-1][0])
            clock_pub.publish(clock_msg)
        print(f"published_frames={published}")
        print(f"first_stamp={pairs[0][0]:.9f}")
        print(f"last_stamp={pairs[-1][0]:.9f}")
        print(f"span_sec={span:.6f}")
    finally:
        stop_ros_master(roscore_proc)


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(2)
