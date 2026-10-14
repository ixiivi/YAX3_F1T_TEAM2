#!/usr/bin/env python3
"""vehicle_sim_node.py — Pacejka 자전거 동역학 SIL 시뮬레이터.

gym_bridge 와 동일한 토픽 인터페이스를 제공하면서
Pacejka 타이어 모델(race_script.yaml map: 섹션 파라미터)로 적분한다.

구독: /drive                  (AckermannDriveStamped)
     /map                    (OccupancyGrid — 충돌 감지용)
발행: /ego_racecar/odom       (Odometry, frame_id='map')
     /trajectory              (Path — RViz 궤적 오버레이)
     TF: map → ego_racecar/base_link

충돌 감지: 맵 OccupancyGrid 에서 차량 중심 + 앞/뒤 코너 4점을 확인.
           충돌 시 속도 0 으로 리셋 (튕기지 않고 정지).
"""

import math

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import TransformBroadcaster

from race_stack.vehicle_sim import VehicleParams, bicycle_ode


def _quat_from_yaw(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _rk4_step(state, delta, Fx_rear, p: VehicleParams, dt: float):
    def f(s):
        return bicycle_ode(0.0, s, delta, Fx_rear, p)
    k1 = np.array(f(state))
    k2 = np.array(f(state + dt / 2 * k1))
    k3 = np.array(f(state + dt / 2 * k2))
    k4 = np.array(f(state + dt * k3))
    return state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


# 차량 footprint 체크 포인트 (차체 좌표, 앞/뒤/좌/우 모서리)
# [x_offset, y_offset] — 차체 기준 (x: 전방, y: 좌측)
_FOOTPRINT = np.array([
    [ 0.22,  0.10],
    [ 0.22, -0.10],
    [-0.12,  0.10],
    [-0.12, -0.10],
    [ 0.05,  0.00],   # 중심 근처
])


class VehicleSimNode(Node):
    def __init__(self):
        super().__init__('vehicle_sim')

        # ── 파라미터 선언 ──────────────────────────────────────────────────
        self.declare_parameter('mu',       1.0)
        self.declare_parameter('mass',     3.354)
        self.declare_parameter('lf',       0.1757)
        self.declare_parameter('lr',       0.1441)
        self.declare_parameter('Iz',       0.050)
        self.declare_parameter('pac_b',    3.8)
        self.declare_parameter('pac_c',    1.3)
        self.declare_parameter('pac_d',    1.15)
        self.declare_parameter('pac_e',   -0.5)
        self.declare_parameter('sx',       0.0)
        self.declare_parameter('sy',       0.0)
        self.declare_parameter('stheta',   0.0)
        self.declare_parameter('tau_v',    0.25)
        self.declare_parameter('a_max',    9.51)
        self.declare_parameter('a_brake',  6.09)
        self.declare_parameter('s_max',    0.4189)
        self.declare_parameter('dt_inner', 0.005)
        self.declare_parameter('pub_hz',   50.0)

        def gp(name):
            return self.get_parameter(name).value

        # ── 차량 파라미터 ──────────────────────────────────────────────────
        self._p = VehicleParams(
            m=gp('mass'), lf=gp('lf'), lr=gp('lr'), Iz=gp('Iz'),
            mu=gp('mu'),
            pac_B=gp('pac_b'), pac_C=gp('pac_c'),
            pac_D=gp('pac_d'), pac_E=gp('pac_e'),
        )
        self._tau_v   = gp('tau_v')
        self._a_max   = gp('a_max')
        self._a_brake = gp('a_brake')
        self._s_max   = gp('s_max')
        self._dt      = gp('dt_inner')
        pub_hz        = gp('pub_hz')

        # ── 초기 상태: [vx, vy, r, x, y, psi] ──────────────────────────
        sx, sy, stheta = gp('sx'), gp('sy'), gp('stheta')
        self._spawn = np.array([0.0, 0.0, 0.0, sx, sy, stheta])
        self._state = self._spawn.copy()

        # ── 제어 명령 ──────────────────────────────────────────────────────
        self._v_cmd     = 0.0
        self._steer_cmd = 0.0

        # ── 맵 충돌 감지 ──────────────────────────────────────────────────
        self._grid: np.ndarray | None = None   # 2D uint8 점유 맵
        self._map_origin_x = 0.0
        self._map_origin_y = 0.0
        self._map_res      = 0.05
        self._collision_count = 0

        # ── ROS 인터페이스 ─────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/ego_racecar/odom', 10)
        self._traj_pub = self.create_publisher(Path, '/trajectory', 1)
        self.create_subscription(AckermannDriveStamped, '/drive', self._drive_cb, 10)
        # nav2 map_server는 TRANSIENT_LOCAL(latched)로 발행 → 동일 QoS로 구독해야 수신됨
        _map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, _map_qos)
        self._br = TransformBroadcaster(self)

        # ── 궤적 ──────────────────────────────────────────────────────────
        self._path = Path()
        self._path.header.frame_id = 'map'

        # ── 타이머 ────────────────────────────────────────────────────────
        self._steps_per_pub = max(1, round(1.0 / (pub_hz * self._dt)))
        period = self._steps_per_pub * self._dt
        self.create_timer(period, self._step)

        self.get_logger().info(
            f'VehicleSimNode 시작 — μ={gp("mu"):.3f}  '
            f'B={gp("pac_b")} C={gp("pac_c")} D={gp("pac_d")}  '
            f'스폰=({sx:.2f},{sy:.2f}) θ={math.degrees(stheta):.1f}°')

    # ── 맵 콜백: OccupancyGrid → numpy 배열 ──────────────────────────────
    def _map_cb(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        # 점유(≥50) 또는 미지(−1) → 벽으로 처리
        self._grid = (data >= 50) | (data < 0)
        self._map_origin_x = msg.info.origin.position.x
        self._map_origin_y = msg.info.origin.position.y
        self._map_res      = msg.info.resolution
        self.get_logger().info(
            f'맵 수신 {w}×{h} res={self._map_res:.3f}m/px', once=True)

    def _in_collision(self, x: float, y: float, psi: float) -> bool:
        """차량 footprint 상의 점이 점유 셀에 있으면 True."""
        if self._grid is None:
            return False
        c, s = math.cos(psi), math.sin(psi)
        res = self._map_res
        ox, oy = self._map_origin_x, self._map_origin_y
        rows, cols = self._grid.shape
        for lx, ly in _FOOTPRINT:
            wx = x + c * lx - s * ly
            wy = y + s * lx + c * ly
            col = int((wx - ox) / res)
            row = int((wy - oy) / res)
            if col < 0 or col >= cols or row < 0 or row >= rows:
                return True   # 맵 밖 = 충돌
            if self._grid[row, col]:
                return True
        return False

    # ── 드라이브 명령 콜백 ─────────────────────────────────────────────────
    def _drive_cb(self, msg: AckermannDriveStamped):
        self._v_cmd     = float(msg.drive.speed)
        self._steer_cmd = float(msg.drive.steering_angle)

    # ── 타이머: 적분 + 충돌 감지 + 발행 ──────────────────────────────────
    def _step(self):
        delta = max(-self._s_max, min(self._s_max, self._steer_cmd))
        v_cmd = self._v_cmd
        p     = self._p
        dt    = self._dt

        for _ in range(self._steps_per_pub):
            vx   = self._state[0]
            dv   = (v_cmd - vx) / self._tau_v
            Fx   = p.m * max(-self._a_brake, min(self._a_max, dv))
            next_state = _rk4_step(self._state, delta, Fx, p, dt)
            if v_cmd >= 0.0:
                next_state[0] = max(0.0, next_state[0])

            # 충돌 감지 — 다음 위치가 벽이면 속도 0, 위치 유지
            if self._in_collision(next_state[3], next_state[4], next_state[5]):
                self._collision_count += 1
                if self._collision_count == 1:
                    self.get_logger().warn(
                        f'충돌 감지 @ ({next_state[3]:.2f},{next_state[4]:.2f})')
                self._state[0] = 0.0   # vx = 0
                self._state[1] = 0.0   # vy = 0
                self._state[2] = 0.0   # r  = 0
                # 위치/방향은 충돌 직전 그대로 유지
            else:
                self._collision_count = 0
                self._state = next_state

        self._publish()

    # ── odom + TF + 궤적 발행 ─────────────────────────────────────────────
    def _publish(self):
        vx, vy, r, x, y, psi = self._state
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = _quat_from_yaw(psi)

        odom = Odometry()
        odom.header.stamp       = now
        odom.header.frame_id    = 'map'
        odom.child_frame_id     = 'ego_racecar/base_link'
        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = r
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp       = now
        tf.header.frame_id    = 'map'
        tf.child_frame_id     = 'ego_racecar/base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.x    = qx
        tf.transform.rotation.y    = qy
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self._br.sendTransform(tf)

        ps = PoseStamped()
        ps.header = odom.header
        ps.pose   = odom.pose.pose
        self._path.poses.append(ps)
        self._path.header.stamp = now
        self._traj_pub.publish(self._path)


def main():
    rclpy.init()
    node = VehicleSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
