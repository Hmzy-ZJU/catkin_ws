#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
# WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements:
#   Python 3.x

"""
The Kinect provides color and depth images in an unsynchronized way. This means
that the set of time stamps from the color images does not intersect with those
of the depth images. Therefore, we need some way of associating color images to
depth images.

This script reads the time stamps from two files (e.g., rgb.txt and depth.txt)
and joins them by finding the best matches.
"""

from __future__ import annotations

import argparse
from typing import Dict, List, Tuple


def read_file_list(filename: str, remove_bounds: bool = False) -> Dict[float, List[str]]:
    """
    Read a timestamped file into a dict: {timestamp(float): [values as str...]}

    File format per line:
        stamp d1 d2 d3 ...
    Lines starting with '#' or empty lines are ignored.
    Commas and tabs are treated as whitespace.

    Parameters
    ----------
    filename : str
    remove_bounds : bool
        If True, drop the first and last 100 lines after parsing (to mimic
        older behavior used in some benchmarks).

    Returns
    -------
    dict[float, list[str]]
    """
    with open(filename, "r", encoding="utf-8") as f:
        data = f.read()

    lines = data.replace(",", " ").replace("\t", " ").split("\n")
    if remove_bounds:
        lines = lines[100:-100]

    rows = [
        [v.strip() for v in line.split(" ") if v.strip() != ""]
        for line in lines
        if len(line) > 0 and not line.startswith("#")
    ]

    # Keep payload as strings to preserve original formatting; timestamps as float.
    parsed = [(float(r[0]), r[1:]) for r in rows if len(r) > 1]
    return dict(parsed)


def associate(first_list: Dict[float, List[str]],
              second_list: Dict[float, List[str]],
              offset: float,
              max_difference: float) -> List[Tuple[float, float]]:
    """
    Associate two dictionaries {stamp: data} by nearest timestamps.

    We build all candidate pairs within `max_difference`, sort by absolute time
    difference, and greedily select non-conflicting matches.

    Parameters
    ----------
    first_list : dict
    second_list : dict
    offset : float
        Time offset added to timestamps of the second list when matching
        (i.e., compare a vs (b + offset)).
    max_difference : float
        Maximum allowed absolute time difference for a valid match.
        (Use the same unit as your timestamps, e.g., seconds.)

    Returns
    -------
    matches : list[tuple[float, float]]
        Sorted list of matched timestamp pairs (a_from_first, b_from_second).
    """
    first_keys = set(first_list.keys())
    second_keys = set(second_list.keys())

    potential_matches = [
        (abs(a - (b + offset)), a, b)
        for a in first_keys
        for b in second_keys
        if abs(a - (b + offset)) < max_difference
    ]
    potential_matches.sort(key=lambda x: x[0])

    matches: List[Tuple[float, float]] = []
    for _, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))

    matches.sort(key=lambda p: p[0])
    return matches


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Associate two timestamped files by nearest timestamps."
    )
    parser.add_argument("first_file", help="first text file (format: timestamp data)")
    parser.add_argument("second_file", help="second text file (format: timestamp data)")
    parser.add_argument("--first_only", action="store_true",
                        help="only output associated lines from the first file")
    parser.add_argument("--offset", type=float, default=0.0,
                        help="time offset added to the timestamps of the second file (default: 0.0)")
    parser.add_argument("--max_difference", type=float, default=0.02,
                        help="max allowed time difference for matching entries (default: 0.02)")
    parser.add_argument("--remove_bounds", action="store_true",
                        help="drop first/last 100 lines from both files before associating")

    args = parser.parse_args()

    first_list = read_file_list(args.first_file, remove_bounds=args.remove_bounds)
    second_list = read_file_list(args.second_file, remove_bounds=args.remove_bounds)

    matches = associate(first_list, second_list, args.offset, args.max_difference)

    if args.first_only:
        for a, _ in matches:
            print(f"{a:.6f} " + " ".join(first_list[a]))
    else:
        for a, b in matches:
            # print first stamp + payload, then matched second stamp (after removing offset) + payload
            print(
                f"{a:.6f} " + " ".join(first_list[a]) + " "
                f"{(b - args.offset):.6f} " + " ".join(second_list[b])
            )


if __name__ == "__main__":
    main()

