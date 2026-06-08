#!/usr/bin/env python3
import argparse
import csv
import os
from pathlib import Path

import cv2
import numpy as np
import rosbag


DATASET_TOPICS = {
    "euroc": {
        "left": "/cam0/image_raw",
        "right": "/cam1/image_raw",
        "imu": "/imu0",
        "left_compressed": False,
        "right_compressed": False,
    },
    "tank": {
        "left": "/camera/left/image_dehazed/compressed",
        "right": "/camera/right/image_dehazed/compressed",
        "imu": "/imu/data",
        "left_compressed": True,
        "right_compressed": True,
    },
    "harbor": {
        "left": "/camera/image_raw",
        "right": "",
        "imu": "/rtimulib_node/imu",
        "left_compressed": False,
        "right_compressed": False,
    },
}


def stamp_ns(msg):
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return None
    return int(stamp.to_nsec())


def image_msg_to_cv(msg, compressed):
    if compressed:
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        im = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
        if im is None:
            raise RuntimeError("failed to decode compressed image")
        return im

    h, w = int(msg.height), int(msg.width)
    enc = msg.encoding.lower()
    arr = np.frombuffer(msg.data, dtype=np.uint8)

    if enc in ("mono8", "8uc1"):
        return arr.reshape(h, w).copy()
    if enc in ("bgr8", "rgb8"):
        im = arr.reshape(h, w, 3).copy()
        if enc == "rgb8":
            im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        return im
    if enc in ("bgra8", "rgba8"):
        im = arr.reshape(h, w, 4).copy()
        if enc == "rgba8":
            im = cv2.cvtColor(im, cv2.COLOR_RGBA2BGRA)
        return im

    try:
        from cv_bridge import CvBridge

        return CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as exc:
        raise RuntimeError(f"unsupported image encoding {msg.encoding}: {exc}")


def write_image(out_dir, ns, image):
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"{ns}.png"
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"failed to write image: {path}")


def imu_row(msg):
    ns = stamp_ns(msg)
    if ns is None:
        return None
    return [
        ns,
        float(msg.angular_velocity.x),
        float(msg.angular_velocity.y),
        float(msg.angular_velocity.z),
        float(msg.linear_acceleration.x),
        float(msg.linear_acceleration.y),
        float(msg.linear_acceleration.z),
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True, choices=sorted(DATASET_TOPICS))
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--start", type=float, default=0.0, help="seconds from bag start")
    parser.add_argument("--duration", type=float, default=0.0, help="0 means full bag")
    parser.add_argument("--max-frames", type=int, default=0, help="0 means no limit")
    args = parser.parse_args()

    topics = DATASET_TOPICS[args.dataset]
    out = Path(args.out)
    cam0_dir = out / "mav0" / "cam0" / "data"
    cam1_dir = out / "mav0" / "cam1" / "data"
    imu_path = out / "mav0" / "imu0" / "data.csv"
    times_path = out / "timestamps.txt"

    out.mkdir(parents=True, exist_ok=True)
    (out / "mav0" / "imu0").mkdir(parents=True, exist_ok=True)

    image_topics = [topics["left"]]
    if topics["right"]:
        image_topics.append(topics["right"])
    read_topics = image_topics + [topics["imu"]]

    left_times = []
    paired_times = []
    imu_rows = []
    left_count = 0
    right_count = 0
    imu_count = 0
    latest_right = None
    pending_left = None
    sync_tolerance_ns = int(0.03 * 1e9)

    with rosbag.Bag(args.bag) as bag:
        bag_start = bag.get_start_time()
        start_time = bag_start + max(0.0, args.start)
        end_time = None if args.duration <= 0 else start_time + args.duration

        for topic, msg, t in bag.read_messages(topics=read_topics):
            ts = t.to_sec()
            if ts < start_time:
                continue
            if end_time is not None and ts > end_time:
                break

            if topic == topics["imu"]:
                row = imu_row(msg)
                if row:
                    imu_rows.append(row)
                    imu_count += 1
                continue

            ns = stamp_ns(msg)
            if ns is None:
                continue

            if topic == topics["left"]:
                if args.max_frames and left_count >= args.max_frames:
                    continue
                im = image_msg_to_cv(msg, topics["left_compressed"])
                write_image(cam0_dir, ns, im)
                left_times.append(ns)
                left_count += 1
                if topics["right"]:
                    pending_left = (ns, im)
                    if latest_right and abs(latest_right[0] - ns) <= sync_tolerance_ns:
                        write_image(cam1_dir, ns, latest_right[1])
                        paired_times.append(ns)
                        right_count += 1
                        pending_left = None
            elif topics["right"] and topic == topics["right"]:
                im = image_msg_to_cv(msg, topics["right_compressed"])
                latest_right = (ns, im)
                if pending_left and abs(pending_left[0] - ns) <= sync_tolerance_ns:
                    write_image(cam1_dir, pending_left[0], im)
                    paired_times.append(pending_left[0])
                    right_count += 1
                    pending_left = None

    left_times = sorted(set(paired_times if topics["right"] else left_times))
    with open(times_path, "w", newline="") as f:
        for ns in left_times:
            f.write(f"{ns}\n")

    imu_rows.sort(key=lambda r: r[0])
    with open(imu_path, "w", newline="") as f:
        f.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")
        writer = csv.writer(f)
        writer.writerows(imu_rows)

    print(f"dataset={args.dataset}")
    print(f"bag={args.bag}")
    print(f"out={out}")
    print(f"left_images={left_count}")
    print(f"right_images={right_count}")
    print(f"imu_rows={imu_count}")
    print(f"timestamps={times_path}")
    print(f"imu_csv={imu_path}")


if __name__ == "__main__":
    main()
