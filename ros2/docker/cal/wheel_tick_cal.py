#!/usr/bin/env python3
"""Wheel tick stall cal — uses Go/session + brain cmd_vel (same path as driving)."""
from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt32

sys.path.insert(0, '/opt/fleet')
from rover_qos import CMD_VEL_QOS

BTN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
COLLECT_SEC = 20.0
SAMPLE_DT = 0.2


class WheelTickCal(Node):
    def __init__(self) -> None:
        super().__init__('wheel_tick_cal')
        self.btn_pub = self.create_publisher(Bool, '/rover/button', BTN_QOS)
        self._l: int | None = None
        self._r: int | None = None
        self._last_lin = 0.0
        self._last_ang = 0.0
        self._session = False
        self._pairs: list[tuple[float, float]] = []
        self._sample_combined: int | None = None
        self._sample_t: float | None = None
        self.create_subscription(UInt32, '/rover/wheel/left_ticks', self._on_l, 10)
        self.create_subscription(UInt32, '/rover/wheel/right_ticks', self._on_r, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, CMD_VEL_QOS)
        self.create_subscription(Bool, '/rover/session', self._on_session, BTN_QOS)
        self.create_timer(SAMPLE_DT, self._sample)

    def _on_l(self, msg: UInt32) -> None:
        self._l = int(msg.data)

    def _on_r(self, msg: UInt32) -> None:
        self._r = int(msg.data)

    def _on_cmd(self, msg: Twist) -> None:
        self._last_lin = float(msg.linear.x)
        self._last_ang = float(msg.angular.z)

    def _on_session(self, msg: Bool) -> None:
        self._session = bool(msg.data)

    def _combined(self) -> int | None:
        if self._l is None or self._r is None:
            return None
        return self._l + self._r

    def _sample(self) -> None:
        if not self._session:
            return
        cur = self._combined()
        if cur is None:
            return
        now = time.monotonic()
        if self._sample_combined is None or self._sample_t is None:
            self._sample_combined = cur
            self._sample_t = now
            return
        dt = now - self._sample_t
        if dt < SAMPLE_DT * 0.8:
            return
        delta = cur - self._sample_combined
        rate = delta / dt if dt > 0.0 else 0.0
        lin = self._last_lin
        ang = self._last_ang
        # Explore wanders — allow mild steer; reject reverse / spin-in-place.
        if lin > 0.05 and abs(ang) < 0.20 and delta > 0:
            self._pairs.append((lin, rate))
        self._sample_combined = cur
        self._sample_t = now

    def _tap_go(self) -> None:
        # One publish per tap — brain toggles session on each true Bool.
        btn = Bool()
        btn.data = True
        self.btn_pub.publish(btn)
        rclpy.spin_once(self, timeout_sec=0.15)

    def _wait_session(self, want: bool, timeout: float) -> bool:
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._session == want:
                return True
        return self._session == want


def main() -> int:
    rclpy.init()
    node = WheelTickCal()
    started_session = False
    try:
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._l is not None and node._r is not None:
                break

        if node._session:
            node.get_logger().info('Session already active — measuring (do not tap Go)')
        else:
            node.get_logger().info('Tap Go via /rover/button — brain must be running')
            for attempt in range(3):
                node._tap_go()
                if node._wait_session(True, 5.0):
                    break
                node.get_logger().warn(f'Session not started (attempt {attempt + 1}/3)')
            if not node._session:
                node.get_logger().error('Session did not start — is rover-brain running?')
                return 1
            started_session = True

        node.get_logger().info(f'Collecting {COLLECT_SEC:.0f}s straight cruise samples...')
        t_done = time.monotonic() + COLLECT_SEC
        while time.monotonic() < t_done:
            rclpy.spin_once(node, timeout_sec=0.05)

        if started_session:
            node._tap_go()
            node._wait_session(False, 3.0)

        if len(node._pairs) < 8:
            node.get_logger().error(
                f'Only {len(node._pairs)} samples — wheels must spin (floor or free stand)'
            )
            return 1

        ratios = [rate / lin for lin, rate in node._pairs if lin > 0.05]
        if len(ratios) < 8:
            node.get_logger().error('Not enough straight samples')
            return 1

        mean = sum(ratios) / len(ratios)
        var = sum((r - mean) ** 2 for r in ratios) / len(ratios)
        std = math.sqrt(var)

        out_path = '/tmp/wheel_stall.env'
        with open(out_path, 'w') as f:
            f.write('# Wheel tick stall cal — combined ticks/s per unit cmd magnitude\n')
            f.write(f'ROVER_WHEEL_TICKS_PER_CMD={mean:.3f}\n')
            f.write(f'ROVER_WHEEL_CAL_SAMPLES={len(ratios)}\n')
        node.get_logger().info(
            f'Cal: ticks/cmd={mean:.2f} ±{std:.2f} from {len(ratios)} samples → {out_path}'
        )
        print(f'ROVER_WHEEL_TICKS_PER_CMD={mean:.3f}')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
