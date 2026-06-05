#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fix_rosbag_vio_sync.py

离线一体化修理工具（ROS1）：
1) IMU 时间偏移补偿：msg.header.stamp := stamp - SHIFT
2) 立体图像配对 + 等时戳：left/right 在容差内配对，两个帧的时间戳改成完全一致
3) 可选：CameraInfo 时间贴近对应图像
4) 干跑、统计、严格单调检查、临时文件写出保证

用法示例：
  ./fix_rosbag_vio_sync.py input.bag \
    --imu-topics /davis_left/imu \
    --imu-shift 0.0031802757031407 \
    --left-topic /davis_left/image_raw \
    --right-topic /davis_right/image_raw \
    --tolerance 0.03 --strategy average \
    --update-camera-info

注意：
- “t_imu = t_cam + shift” ==> 为了对齐到相机时间轴，应执行 stamp := stamp - shift
- 我们对图像不仅改 header.stamp，也将写入时间 t 改为新时间，使 rosbag play 调度严格等时
"""

from __future__ import print_function
import os, sys, argparse, tempfile, shutil, math
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional

try:
    import rosbag
    import rospy
    from rospy.rostime import Time
except Exception as e:
    print("导入 ROS 依赖失败：请在已 source 的 ROS1 环境中运行。", file=sys.stderr)
    raise

# 消息类型（可缺省）
try:
    from sensor_msgs.msg import Image, CompressedImage, CameraInfo, Imu
except Exception:
    Image = None
    CompressedImage = None
    CameraInfo = None
    Imu = None

@dataclass
class Sample:
    bag_t: Time
    header_t: Time
    idx: int

def t2f(t: Time) -> float:
    return float(t.secs) + float(t.nsecs) * 1e-9

def f2t(x: float) -> Time:
    secs = int(math.floor(x))
    nsecs = int(round((x - secs) * 1e9))
    if nsecs >= 1_000_000_000:
        secs += 1
        nsecs -= 1_000_000_000
    return Time(secs, nsecs)

def detect_stereo_topics(bag_path: str) -> Tuple[Optional[str], Optional[str]]:
    left, right = None, None
    with rosbag.Bag(bag_path, 'r') as bag:
        info = bag.get_type_and_topic_info()
        image_topics = []
        for t, meta in info.topics.items():
            if meta.msg_type in ('sensor_msgs/Image', 'sensor_msgs/CompressedImage'):
                image_topics.append(t)

        def pick(topics: List[str], key: str) -> Optional[str]:
            for tp in topics:
                low = tp.lower()
                if f"/{key}/" in low or low.endswith(f"/{key}") or f"_{key}" in low:
                    return tp
            return None

        left = pick(image_topics, "left")
        right = pick(image_topics, "right")

        if left is None:
            for tp in image_topics:
                if "left" in tp.lower():
                    left = tp; break
        if right is None:
            for tp in image_topics:
                if "right" in tp.lower():
                    right = tp; break

        if (left is None or right is None) and len(image_topics) == 2:
            s = sorted(image_topics)
            left, right = s[0], s[1]
    return left, right

def collect_samples(bag_path: str, topic: str) -> List[Sample]:
    res: List[Sample] = []
    with rosbag.Bag(bag_path, 'r') as bag:
        for i, (tp, msg, bag_t) in enumerate(bag.read_messages(topics=[topic])):
            header_t = getattr(getattr(msg, "header", None), "stamp", bag_t)
            res.append(Sample(bag_t=bag_t, header_t=header_t, idx=i))
    return res

def match_pairs(left: List[Sample], right: List[Sample], tol: float
               ) -> Tuple[List[Tuple[Sample, Sample]], List[Sample], List[Sample]]:
    i, j = 0, 0
    matched: List[Tuple[Sample, Sample]] = []
    unL: List[Sample] = []
    unR: List[Sample] = []
    while i < len(left) and j < len(right):
        lt = t2f(left[i].header_t); rt = t2f(right[j].header_t)
        dt = lt - rt
        if abs(dt) <= tol:
            matched.append((left[i], right[j]))
            i += 1; j += 1
        elif dt < 0:
            unL.append(left[i]); i += 1
        else:
            unR.append(right[j]); j += 1
    while i < len(left):  unL.append(left[i]);  i += 1
    while j < len(right): unR.append(right[j]); j += 1
    return matched, unL, unR

def build_retime_maps(matched: List[Tuple[Sample, Sample]], strategy: str
                     ) -> Tuple[Dict[int, Time], Dict[int, Time]]:
    left_map: Dict[int, Time] = {}
    right_map: Dict[int, Time] = {}
    for l, r in matched:
        lt = t2f(l.header_t); rt = t2f(r.header_t)
        if strategy == "left":
            sync = lt
        elif strategy == "right":
            sync = rt
        else:
            sync = 0.5 * (lt + rt)
        new_t = f2t(sync)
        left_map[int(l.bag_t.to_nsec())] = new_t
        right_map[int(r.bag_t.to_nsec())] = new_t
    return left_map, right_map

def nearest_time(target: Time, cand: List[Time]) -> Optional[Time]:
    if not cand: return None
    tf = t2f(target)
    best, bestd = None, 1e18
    for t in cand:
        d = abs(t2f(t) - tf)
        if d < bestd:
            bestd = d; best = t
    return best

def has_header(msg) -> bool:
    return hasattr(msg, "header") and hasattr(msg.header, "stamp")

def ensure_non_decreasing(prev: Optional[Time], curr: Time, eps_ns: int = 100) -> Time:
    # ROS 对写入时间 t 要求非递减；我们用极小纳秒级补偿避免倒序
    if prev is None: return curr
    if curr > prev: return curr
    # bump
    bumped = Time(prev.secs, prev.nsecs + eps_ns)
    if bumped.nsecs >= 1_000_000_000:
        return Time(prev.secs + 1, bumped.nsecs - 1_000_000_000)
    return bumped

def process_bag(in_bag: str, out_bag: str,
                imu_topics: List[str], imu_shift: float,
                left_topic: str, right_topic: str,
                tolerance: float, strategy: str,
                drop_unmatched: bool,
                update_camera_info: bool,
                left_info_topic: Optional[str], right_info_topic: Optional[str],
                info_tolerance: float,
                allow_backwards_imu: bool):
    # 采样/配对
    L = collect_samples(in_bag, left_topic)
    R = collect_samples(in_bag, right_topic)
    matched, unL, unR = match_pairs(L, R, tolerance)
    left_map, right_map = build_retime_maps(matched, strategy)

    # 统计
    print("=== 立体配对统计 ===")
    print(f"Left frames: {len(L)}  Right frames: {len(R)}")
    print(f"Matched: {len(matched)}  Unmatched(L/R): {len(unL)}/{len(unR)}")

    # 用于 CameraInfo 贴近
    left_synced_times = list(left_map.values())
    right_synced_times = list(right_map.values())

    dur = rospy.Duration.from_sec(imu_shift)
    prev_write_time: Optional[Time] = None
    prev_imu_stamp: Dict[str, Optional[Time]] = {tp: None for tp in imu_topics}

    # 临时文件安全写出
    out_dir = os.path.dirname(os.path.abspath(out_bag)) or "."
    tmp_fd, tmp_path = tempfile.mkstemp(prefix=".tmp_vio_sync_", dir=out_dir)
    os.close(tmp_fd)

    touched_imu = 0
    touched_left = 0
    touched_right = 0

    try:
        with rosbag.Bag(in_bag, 'r') as ib, rosbag.Bag(tmp_path, 'w') as ob:
            for topic, msg, bag_t in ib.read_messages():
                write_t = bag_t  # 默认保留原写入时间
                out_msg = msg

                # 立体图像：改 header.stamp 且将写入时间设为新时间（保证 rosbag play 等时调度）
                if topic == left_topic:
                    key = int(bag_t.to_nsec())
                    if key in left_map:
                        new_t = left_map[key]
                        if has_header(msg):
                            msg.header.stamp = new_t
                        write_t = new_t
                        touched_left += 1
                    else:
                        if drop_unmatched:
                            continue  # 丢弃
                    write_t = ensure_non_decreasing(prev_write_time, write_t)
                    prev_write_time = write_t
                    ob.write(topic, msg, t=write_t)
                    continue

                if topic == right_topic:
                    key = int(bag_t.to_nsec())
                    if key in right_map:
                        new_t = right_map[key]
                        if has_header(msg):
                            msg.header.stamp = new_t
                        write_t = new_t
                        touched_right += 1
                    else:
                        if drop_unmatched:
                            continue
                    write_t = ensure_non_decreasing(prev_write_time, write_t)
                    prev_write_time = write_t
                    ob.write(topic, msg, t=write_t)
                    continue

                # CameraInfo 贴近
                if update_camera_info and (topic == left_info_topic or topic == right_info_topic):
                    if has_header(msg):
                        cand = left_synced_times if topic == left_info_topic else right_synced_times
                        nearest = nearest_time(msg.header.stamp, cand)
                        if nearest is not None:
                            d = abs(t2f(nearest) - t2f(msg.header.stamp))
                            if d <= info_tolerance:
                                msg.header.stamp = nearest
                                write_t = nearest
                    write_t = ensure_non_decreasing(prev_write_time, write_t)
                    prev_write_time = write_t
                    ob.write(topic, msg, t=write_t)
                    continue

                # IMU 偏移补偿（只改 header.stamp，不改写入时间，避免全局节奏漂移）
                if topic in imu_topics and has_header(msg):
                    before = msg.header.stamp
                    after = before - dur  # 关键：IMU 时间前移 shift 秒
                    # 单调性检查（每个 IMU topic 自己检查）
                    prev = prev_imu_stamp.get(topic)
                    if prev is not None and after <= prev and not allow_backwards_imu:
                        print(f"[ERROR] 平移导致 IMU {topic} 时间不再单调：{t2f(before):.9f} -> {t2f(after):.9f} "
                              f"(上一个 {t2f(prev):.9f})。如确认安全可加 --allow-backwards-imu 放开。", file=sys.stderr)
                        raise RuntimeError("IMU 时间非单调")
                    msg.header.stamp = after
                    prev_imu_stamp[topic] = after
                    touched_imu += 1
                    # 写入时间仍用 bag_t（不改播放节奏）
                    write_t = ensure_non_decreasing(prev_write_time, write_t)
                    prev_write_time = write_t
                    ob.write(topic, msg, t=write_t)
                    continue

                # 其它消息：原样写
                write_t = ensure_non_decreasing(prev_write_time, write_t)
                prev_write_time = write_t
                ob.write(topic, msg, t=write_t)

        # 原子替换
        if os.path.exists(out_bag):
            os.remove(out_bag)
        shutil.move(tmp_path, out_bag)

        print("\n=== 写出完成 ===")
        print(f"输出文件：{out_bag}")
        print(f"IMU 修正条数：{touched_imu}")
        print(f"Left 等时条数：{touched_left}")
        print(f"Right 等时条数：{touched_right}")

    except Exception as ex:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        raise

def main():
    ap = argparse.ArgumentParser(description="离线一体化修理：IMU 时间偏移 + 立体等时戳（ROS1）。")
    ap.add_argument("in_bag", help="输入 bag 路径")
    ap.add_argument("out_bag", nargs="?", default=None, help="输出 bag 路径（默认 _fixed 后缀）")

    # IMU
    ap.add_argument("--imu-topics", nargs="+", required=True,
                    help="需要补偿时间的 IMU 话题列表，如 /davis_left/imu /imu")
    ap.add_argument("--imu-shift", type=float, required=True,
                    help="Kalibr timeshift（秒）。t_imu = t_cam + shift ==> 这里执行 stamp := stamp - shift")
    ap.add_argument("--allow-backwards-imu", action="store_true",
                    help="允许 IMU 平移后时间不严格单调（一般不推荐）")

    # Stereo
    ap.add_argument("--left-topic", default=None, help="左相机图像话题，可省略自动检测")
    ap.add_argument("--right-topic", default=None, help="右相机图像话题，可省略自动检测")
    ap.add_argument("--tolerance", type=float, default=0.03, help="左右配对容差（秒），默认 0.03")
    ap.add_argument("--strategy", choices=["average","left","right"], default="average",
                    help="等时策略：取平均 / 对齐左 / 对齐右")

    # CameraInfo
    ap.add_argument("--update-camera-info", action="store_true", help="CameraInfo 时间贴近对应图像")
    ap.add_argument("--left-info-topic", default=None, help="左 CameraInfo 话题（不填将尝试推测）")
    ap.add_argument("--right-info-topic", default=None, help="右 CameraInfo 话题（不填将尝试推测）")
    ap.add_argument("--info-tolerance", type=float, default=0.02, help="CameraInfo 贴近容差（秒）")

    args = ap.parse_args()

    in_bag = args.in_bag
    if not os.path.isfile(in_bag):
        print(f"[错误] 找不到输入：{in_bag}", file=sys.stderr); sys.exit(1)

    out_bag = args.out_bag or (os.path.splitext(in_bag)[0] + "_fixed.bag")

    left_topic, right_topic = args.left_topic, args.right_topic
    if left_topic is None or right_topic is None:
        autoL, autoR = detect_stereo_topics(in_bag)
        left_topic = left_topic or autoL
        right_topic = right_topic or autoR
    if not left_topic or not right_topic:
        print("[错误] 无法自动识别左右图像话题，请用 --left-topic / --right-topic 指定。", file=sys.stderr)
        sys.exit(2)

    # CameraInfo 话题自动推测（可选）
    left_info, right_info = args.left_info_topic, args.right_info_topic
    if args.update_camera_info and (left_info is None or right_info is None):
        def guess_info(tp: str) -> str:
            for src in ["/image_raw/compressed","/image_raw","/image_color","/image_mono","/image_rect","/image"]:
                if tp.endswith(src):
                    return tp[: -len(src)] + "/camera_info"
            return tp.rsplit("/",1)[0] + "/camera_info" if "/" in tp else tp + "/camera_info"
        if left_info is None:  left_info  = guess_info(left_topic)
        if right_info is None: right_info = guess_info(right_topic)

    print("=== 配置 ===")
    print("输入：", in_bag)
    print("输出：", out_bag)
    print("IMU 话题：", " ".join(args.imu_topics))
    print("IMU shift（秒）：", args.imu_shift, "(将执行 stamp := stamp - shift)")
    print("左图像：", left_topic)
    print("右图像：", right_topic)
    if args.update_camera_info:
        print("左 CameraInfo：", left_info)
        print("右 CameraInfo：", right_info)

    process_bag(
        in_bag=in_bag,
        out_bag=out_bag,
        imu_topics=args.imu_topics,
        imu_shift=args.imu_shift,
        left_topic=left_topic,
        right_topic=right_topic,
        tolerance=args.tolerance,
        strategy=args.strategy,
        drop_unmatched=False,  # 默认不丢帧，必要时可按需改为 True
        update_camera_info=args.update_camera_info,
        left_info_topic=left_info,
        right_info_topic=right_info,
        info_tolerance=args.info_tolerance,
        allow_backwards_imu=args.allow_backwards_imu
    )

if __name__ == "__main__":
    main()
