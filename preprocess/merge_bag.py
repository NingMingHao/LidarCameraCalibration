#!/usr/bin/env python3
"""
Merge+filter ROS1 bag files by time ranges.

- Reads all .bag files in BAG_DIR (non-recursive).
- Keeps ALL topics, but only messages whose timestamp falls within any time range in TIME_SLICES.
- Writes a single output bag at OUTPUT_BAG.

Tested with ROS1 (e.g., Melodic/Noetic). For ROS2 (rosbag2), ask for a rosbag2 version.
"""

import os
import glob
import rosbag
import rospy
from typing import List, Tuple

# ----------------------------
# User-configurable globals
# ----------------------------

SENSOR_ID = 1

# Folder containing input .bag files
BAG_DIR = f"/media/minghao/Data6TB1/OutdoorData/2025_08_12_sensorcalib/Node{SENSOR_ID}"

# Output bag file path (the folder will be created if needed)
OUTPUT_BAG = f"/home/minghao/Documents/UWaterloo/Projects/Outdoor/CalibTools/Bags/merged_node{SENSOR_ID}.bag"

# Time slices (inclusive): messages with timestamps in ANY of these ranges are kept.
# Provide as POSIX seconds (float/int). Example below shows two slices.
TIME_SLICES: List[Tuple[float, float]] = [
    # (start_epoch_sec, end_epoch_sec)
    # Example: (1723440000.0, 1723440300.0),  # Aug 12, 2024 00:00:00–00:05:00 UTC
    # Example: (1723440600.0, 1723440900.0),
    (1755022628.026973, 1755022639.528546),
    (1755023080.333644, 1755023099.429173)
]

# If your bag times are not wall-clock epoch (e.g., simulated), set these in raw ROS time seconds.
# You can also specify slices in ROS format directly by using rospy.Time.to_sec() values.
# ----------------------------


def _normalize_and_merge_ranges(ranges: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Sort and merge overlapping/adjacent [start, end] ranges in seconds."""
    if not ranges:
        return []
    cleaned = []
    for s, e in ranges:
        if e < s:
            s, e = e, s  # swap if given in reverse
        cleaned.append((float(s), float(e)))
    cleaned.sort(key=lambda x: x[0])

    merged = [cleaned[0]]
    for s, e in cleaned[1:]:
        last_s, last_e = merged[-1]
        if s <= last_e:  # overlap or touch
            merged[-1] = (last_s, max(last_e, e))
        else:
            merged.append((s, e))
    return merged


def _in_ranges(t_sec: float, merged_ranges: List[Tuple[float, float]]) -> bool:
    """Binary search could be used; linear is fine unless you have tons of ranges."""
    for s, e in merged_ranges:
        if s <= t_sec <= e:
            return True
    return False


def merge_and_filter_bags(bag_dir: str, output_bag: str, time_slices: List[Tuple[float, float]]) -> None:
    if not os.path.isdir(bag_dir):
        raise FileNotFoundError(f"Input bag folder not found: {bag_dir}")

    bag_paths = sorted(glob.glob(os.path.join(bag_dir, "*.bag")))
    if not bag_paths:
        raise FileNotFoundError(f"No .bag files found in: {bag_dir}")

    os.makedirs(os.path.dirname(output_bag), exist_ok=True)

    merged_ranges = _normalize_and_merge_ranges(time_slices)
    if not merged_ranges:
        raise ValueError("TIME_SLICES is empty. Provide at least one (start, end) range.")

    print(f"Found {len(bag_paths)} bag(s). Writing to: {output_bag}")
    print("Time slices (merged):")
    for s, e in merged_ranges:
        print(f"  [{s:.6f}, {e:.6f}] sec")

    total_in = 0
    total_out = 0

    with rosbag.Bag(output_bag, "w") as outbag:
        for bag_path in bag_paths:
            print(f"\nProcessing: {bag_path}")
            with rosbag.Bag(bag_path, "r") as inbag:
                for topic, msg, t in inbag.read_messages():  # t is rospy.Time
                    total_in += 1
                    t_sec = t.to_sec()
                    if _in_ranges(t_sec, merged_ranges):
                        # Preserve the original timestamp 't'
                        outbag.write(topic, msg, t)
                        total_out += 1

    print(f"\nDone. Messages read: {total_in}, messages written: {total_out}")
    print(f"Output saved to: {output_bag}")


if __name__ == "__main__":
    # Initialize ROS time for rospy.Time conversions (does not start a node).
    # Not strictly required for reading/writing bags, but safe to have rospy imported/ready.
    # if not rospy.core.is_initialized():
    #     rospy.init_node("bag_merge_filter", anonymous=True, disable_signals=True)

    merge_and_filter_bags(BAG_DIR, OUTPUT_BAG, TIME_SLICES)
