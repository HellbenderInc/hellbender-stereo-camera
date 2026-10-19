#!/usr/bin/env python3
"""
Phase 0: Max simultaneous detections over configurable time windows.

Subscribes to /hailo/pipeline_latency, extracts detection_count,
tracks the max per time window, and saves a histogram plot.

Usage:
    python3 detection_counter.py                          # defaults: 5 min windows, 60 min duration
    python3 detection_counter.py --window 10 --duration 120
    python3 detection_counter.py --window 5 --duration 60 --output /tmp/traffic.png
"""

import argparse
import json
import signal
import sys
import time
from collections import defaultdict
from datetime import datetime, timedelta

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.dates as mdates


class DetectionCounter(Node):
    def __init__(self, window_minutes: int, duration_minutes: int, output_path: str):
        super().__init__("detection_counter")

        self.window_minutes = window_minutes
        self.duration_minutes = duration_minutes
        self.output_path = output_path

        self.start_time = datetime.now()
        self.end_time = self.start_time + timedelta(minutes=duration_minutes)

        # {window_index: max_count_seen}
        self.window_max = defaultdict(int)
        self.total_frames = 0

        self.subscription = self.create_subscription(
            DiagnosticArray,
            "/hailo/pipeline_latency",
            self.on_message,
            10,
        )

        self.get_logger().info(
            f"Counting detections | window={window_minutes}min | "
            f"duration={duration_minutes}min | output={output_path}"
        )
        self.get_logger().info(
            f"Will finish at {self.end_time.strftime('%H:%M:%S')}. Ctrl+C to stop early and save."
        )

    # --- Subscription callback -------------------------------------------

    def on_message(self, msg: String):
        now = datetime.now()

        if now >= self.end_time:
            self.finish()
            return

        try:
            data = json.loads(msg.data)
            count = int(data.get("detection_count", 0))
        except (json.JSONDecodeError, ValueError, TypeError) as e:
            self.get_logger().warn(f"Bad message: {e}")
            return

        elapsed = (now - self.start_time).total_seconds()
        window_idx = int(elapsed // (self.window_minutes * 60))

        if count > self.window_max[window_idx]:
            self.window_max[window_idx] = count

        self.total_frames += 1

        # Periodic log every 100 frames
        if self.total_frames % 100 == 0:
            self.get_logger().info(
                f"Frames: {self.total_frames} | Current window {window_idx} max: {self.window_max[window_idx]}"
            )

    # --- Plot and save ---------------------------------------------------

    def finish(self):
        self.get_logger().info(f"Done. {self.total_frames} frames processed.")
        self.save_plot()
        rclpy.shutdown()

    def save_plot(self):
        num_windows = max(1, self.duration_minutes // self.window_minutes)

        window_starts = []
        counts = []

        for i in range(num_windows):
            t = self.start_time + timedelta(minutes=i * self.window_minutes)
            window_starts.append(t)
            counts.append(self.window_max.get(i, 0))

        fig, ax = plt.subplots(figsize=(12, 5))

        bar_width = (self.window_minutes * 60) / 86400  # width in days for date axis
        ax.bar(
            window_starts,
            counts,
            width=bar_width * 0.9,
            color="#4a90d9",
            edgecolor="#2c5f8a",
            alpha=0.85,
        )

        ax.set_xlabel("Time")
        ax.set_ylabel("Max Simultaneous Detections")
        ax.set_title(
            f"Peak Detections per {self.window_minutes}-Minute Window\n"
            f"{self.start_time.strftime('%Y-%m-%d %H:%M')} — "
            f"{datetime.now().strftime('%H:%M')}"
        )

        ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
        ax.xaxis.set_major_locator(mdates.AutoDateLocator())
        fig.autofmt_xdate()

        ax.set_ylim(bottom=0)
        ax.yaxis.get_major_locator().set_params(integer=True)
        ax.grid(axis="y", alpha=0.3)

        plt.tight_layout()
        plt.savefig(self.output_path, dpi=150)
        self.get_logger().info(f"Plot saved to {self.output_path}")


def main():
    parser = argparse.ArgumentParser(description="Phase 0: Max detection counter")
    parser.add_argument(
        "--window", type=int, default=5, help="Window size in minutes (default: 5)"
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=60,
        help="Total observation time in minutes (default: 60)",
    )
    parser.add_argument(
        "--output", type=str, default="traffic_histogram.png", help="Output image path"
    )
    args = parser.parse_args()

    rclpy.init()
    node = DetectionCounter(args.window, args.duration, args.output)

    # Save plot on Ctrl+C
    def handle_signal(sig, frame):
        node.get_logger().info("Interrupted — saving plot...")
        node.save_plot()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass


if __name__ == "__main__":
    main()
