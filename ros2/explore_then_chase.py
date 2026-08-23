#!/usr/bin/env python3
"""Wander forward on sonar; chase rover IR beacon when TSOP sees it."""
from __future__ import annotations

import math
import os
import socket
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Bool, Float32

from fleet_steering import IrSteerConfig, WanderConfig, WanderState, steer_ir_beacon


class ExploreThenChase(Node):
    TICK = 0.05

    def __init__(self) -> None:
        ns = os.environ.get('BOT_NS', 'bot1').strip('/')
        pan_center = float(os.environ.get('PAN_CENTER', '90'))
        super().__init__('explore_then_chase')
        self.pub = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
        self.create_subscription(Range, f'/{ns}/sonar/range', self._on_range, 10)
        self.create_subscription(Imu, f'/{ns}/imu', self._on_imu, 10)
        self.create_subscription(Bool, f'/{ns}/ir/detected', self._on_ir, 10)
        self.create_subscription(Float32, f'/{ns}/sonar/pan_deg', self._on_pan, 10)
        self.IR_ON_TICKS = int(os.environ.get('CHASE_IR_ON_TICKS', '24'))
        self.IR_OFF_TICKS = int(os.environ.get('CHASE_IR_OFF_TICKS', '15'))
        self.IR_OFF_TICKS_CHASE = int(os.environ.get('CHASE_IR_OFF_TICKS_ACTIVE', '4'))
        self.CHASE_MAX_SEC = float(os.environ.get('CHASE_MAX_SEC', '10.0'))
        self.CHASE_COOLDOWN_SEC = float(os.environ.get('CHASE_COOLDOWN_SEC', '6.0'))
        wander_cfg = WanderConfig.from_env(pan_center=pan_center)
        self._wander = WanderState.from_config(wander_cfg)
        self._wander._heading.gyro_sign = float(os.environ.get('WANDER_GYRO_SIGN', '-1'))
        self._wander._heading.cal_s = float(os.environ.get('IMU_CAL_SEC', '2.5'))
        self._wander._heading.cal_min_samples = int(os.environ.get('IMU_CAL_MIN_SAMPLES', '50'))
        self._ir_cfg = IrSteerConfig(
            cruise=float(os.environ.get('CHASE_CRUISE', '0.24')),
            pan_center=pan_center,
            spin_rate=float(os.environ.get('CHASE_SPIN_RATE', '0.45')),
            align_creep=float(os.environ.get('CHASE_ALIGN_CREEP', '0.14')),
            pan_align_deg=float(os.environ.get('CHASE_PAN_ALIGN_DEG', '28')),
        )
        self._ir = False
        self._ir_on_streak = 0
        self._ir_off_streak = 0
        self._pan_deg = pan_center
        self._notify_host = os.environ.get('ROVER_NOTIFY_HOST', 'lego-rover.local')
        self._notify_port = int(os.environ.get('ROVER_NOTIFY_PORT', '4242'))
        self._mode = 'wander'
        self._chase_started_at: float | None = None
        self._chase_cooldown_until = 0.0
        self._drive_log_at = 0.0
        self._drive_csv = os.environ.get('DRIVE_CSV', '').strip()
        self._csv_ready = False
        self._last_gz = 0.0
        self._imu_seen = False
        self._cal_log_at = 0.0
        self._cal_done_logged = False
        self.create_timer(self.TICK, self._tick)
        self.get_logger().info(f'explore_then_chase: /{ns} forward wander → chase on IR')

    def _on_range(self, msg: Range) -> None:
        self._wander.update_range(float(msg.range), self._pan_deg)

    def _on_imu(self, msg: Imu) -> None:
        self._imu_seen = True
        self._last_gz = float(msg.angular_velocity.z)
        self._wander.update_imu(self._last_gz)

    def _on_ir(self, msg: Bool) -> None:
        detected = bool(msg.data)
        if detected:
            self._ir_on_streak += 1
            self._ir_off_streak = 0
        else:
            self._ir_off_streak += 1
            self._ir_on_streak = 0

        prev = self._ir
        off_need = self.IR_OFF_TICKS_CHASE if self._ir else self.IR_OFF_TICKS
        if self._ir_on_streak >= self.IR_ON_TICKS:
            self._ir = True
        elif self._ir_off_streak >= off_need:
            self._ir = False

        if self._ir and not prev:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sock.sendto(
                        b'beacon_lock',
                        (self._notify_host, self._notify_port),
                    )
            except OSError as exc:
                self.get_logger().warn(f'beacon notification failed: {exc}')

    def _on_pan(self, msg: Float32) -> None:
        self._pan_deg = float(msg.data)
        self._wander._pan_deg = self._pan_deg

    def _tick(self) -> None:
        now = time.monotonic()
        if not self._imu_seen:
            self.pub.publish(Twist())
            return

        if not self._wander.imu_ready():
            if now >= self._cal_log_at:
                self._cal_log_at = now + 1.0
                pct = int(self._wander.imu_cal_progress() * 100.0)
                self.get_logger().info(
                    f'IMU calibrating — hold still ({pct}%, '
                    f'{self._wander._heading.cal_s:.0f}s)'
                )
            self.pub.publish(Twist())
            return

        if not self._cal_done_logged:
            self._cal_done_logged = True
            self.get_logger().info('IMU ready — exploring')

        chasing = self._ir and time.monotonic() >= self._chase_cooldown_until
        if chasing and self._chase_started_at is None:
            self._chase_started_at = time.monotonic()
        if chasing and self._chase_started_at is not None:
            if time.monotonic() - self._chase_started_at >= self.CHASE_MAX_SEC:
                chasing = False
                self._ir = False
                self._ir_on_streak = 0
                self._ir_off_streak = self.IR_OFF_TICKS
                self._chase_started_at = None
                self._chase_cooldown_until = time.monotonic() + self.CHASE_COOLDOWN_SEC
                self.get_logger().info(
                    f'chase timeout ({self.CHASE_MAX_SEC:.0f}s) — back to wander'
                )
        if not chasing:
            self._chase_started_at = None

        mode = 'chase' if chasing else 'wander'
        if mode != self._mode:
            self._mode = mode
            self.get_logger().info(f'mode → {mode}')
            if chasing:
                self._wander._heading.release()
            else:
                self._wander._heading.reset_target()

        if chasing:
            msg = Twist()
            steer_ir_beacon(
                msg,
                pan_deg=self._pan_deg,
                ir_now=self._ir_on_streak >= max(2, self.IR_ON_TICKS // 4),
                cfg=self._ir_cfg,
            )
            self._wander.apply_sonar_stop(msg)
            self._wander._last_driving = msg.linear.x > 0.05
        else:
            msg = Twist()
            self._wander.fill_twist(msg)
            if self._wander.consume_burst_started():
                self.get_logger().info(
                    f'sprint burst {self._wander.burst_speed:.0%} for '
                    f'{self._wander.burst_duration_s:.1f}s'
                )

        now = time.monotonic()
        if now >= self._drive_log_at:
            self._drive_log_at = now + 2.0
            rng = self._wander.debug_range_m()
            rng_s = 'nan' if math.isnan(rng) else f'{rng:.2f}'
            self.get_logger().info(
                f'drive lin={msg.linear.x:+.2f} ang={msg.angular.z:+.3f} '
                f'trim={self._wander._heading.trim_ang:+.3f} trap={self._wander.trap_level()} '
                f'mode={mode} rng={rng_s}m pan={self._pan_deg:.0f}'
            )

        if self._drive_csv:
            rng = self._wander.debug_range_m()
            rng_s = '' if math.isnan(rng) else f'{rng:.3f}'
            if not self._csv_ready:
                with open(self._drive_csv, 'w', encoding='utf-8') as f:
                    f.write('t,lin,ang,gz,range_m,mode,pan,trap\n')
                self._csv_ready = True
            with open(self._drive_csv, 'a', encoding='utf-8') as f:
                f.write(
                    f'{time.time():.3f},{msg.linear.x:.4f},{msg.angular.z:.4f},'
                    f'{self._last_gz:.5f},{rng_s},{mode},{self._pan_deg:.0f},'
                    f'{self._wander.trap_level()}\n'
                )

        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ExploreThenChase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        for _ in range(5):
            node.pub.publish(stop)
            time.sleep(0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
