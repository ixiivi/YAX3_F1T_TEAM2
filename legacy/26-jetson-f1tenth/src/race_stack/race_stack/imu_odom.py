#!/usr/bin/env python3
"""
imu_odom.py — odom 의 yaw 를 VESC IMU 자이로로 갈아끼운다.

왜 필요한가
───────────
`vesc_to_odom` 은 `use_servo_cmd_to_calc_angular_velocity: true` 라서 각속도를
**서보 명령값**으로 계산한다 (ω = v·tan(δ)/L). 명령은 목표값이라 실제 바퀴보다
앞서고, 기구학 자전거 모델은 타이어 슬립각을 0 으로 가정한다. 둘 다 yaw 를
과대 추정하는 방향이라, 트랙이 커지자 AMCL 이 누적 오차로 발산했다 (2026-08-02).

자이로의 yaw rate 는 서보 지연·슬립과 무관한 **실측**이다. 병진 속도는 휠
오도메트리가 정확하므로 그대로 두고, **회전만** 자이로로 바꾼다.

★ AMCL 은 `/odom` 토픽이 아니라 `odom→base_link` TF 를 본다.
  그래서 이 노드가 TF 발행을 맡고, `config/vesc.yaml` 에서
  `vesc_to_odom_node.publish_tf: false` 로 원래 발행자를 끈다.
  **이 노드가 안 뜨면 odom TF 자체가 없다** — AMCL·SLAM 이 전부 멈춘다.
  그래서 `imu_yaw: false` 여도 노드는 항상 뜨고, 그때는 들어온 odom 을
  그대로 중계(pass-through)해 기존 동작과 동일해진다.

바이어스
────────
자이로는 정지 상태에서도 0 이 아닌 값을 낸다(zero-rate output). 상수 오차라
적분하면 시간에 비례해 yaw 가 샌다 — 실측 0.11 deg/s 면 118초 랩에 13.3°.
그런데 이 값은 **부팅할 때마다 다르고 보드가 데워지면 또 변한다.** 그래서
상수로 박지 않고 **정지할 때마다 직접 잰다**:
  - 기동 직후 정지 구간에서 warmup_sec 동안 평균 → 초기 바이어스
  - 이후에도 정지가 감지되면 느린 지수평균(alpha)으로 계속 갱신

축 매핑 (2026-08-02 `imu_axis_check.py` 실측)
──────────────────────────────────────────────
    차량 전방 = IMU -y,  차량 좌측 = IMU +x,  차량 위 = IMU +z
    → 순수 z축 90° 회전. yaw 축은 z 그대로, 좌회전 = +z 로 부호까지 일치.
    회전축이 z 라 x/y 스왑은 yaw 에 영향이 없다. 그래서 기본값이 axis=z, sign=+1.
    (가속도까지 쓸 일이 생기면 그때 x/y 스왑이 필요하다)

발행
────
  /odom_imu (nav_msgs/Odometry)  — 보정된 오도메트리
  TF odom→base_link              — AMCL·SLAM 이 실제로 보는 것
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

AXIS_IDX = {'x': 0, 'y': 1, 'z': 2}


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class ImuOdom(Node):
    def __init__(self):
        super().__init__('imu_odom')

        self.declare_parameter('imu_yaw', True)          # False = 그대로 중계
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('out_topic', '/odom_imu')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        # 자이로 원본 단위 — VESC 는 deg/s 로 준다 (vesc_packet.cpp:627)
        self.declare_parameter('gyro_units', 'deg')      # deg | rad
        # ★ VESC 원시 IMU 는 g / deg-s 라 REP-145 가 아니다. robot_localization
        #   EKF 는 rad/s·m/s² 를 전제하므로 원시 토픽을 물리면 각속도를 57 배로
        #   읽는다. 여기서 단위만 바꿔 표준 토픽으로 다시 낸다 (자세·적분 없음).
        self.declare_parameter('imu_out_topic', '/imu/data')
        # 재발행 메시지의 frame. 원시 메시지는 frame_id 가 비어 있어
        # (vesc_driver.cpp 가 stamp 만 채운다) robot_localization 이 축을
        # 변환할 근거가 없다. imu_link 로 태그하고 static TF 로 회전을 준다.
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('accel_units', 'g')       # g | ms2
        self.declare_parameter('yaw_axis', 'z')
        self.declare_parameter('yaw_sign', 1.0)
        # 병진 속도 부호 (차량별). 상세는 odom_cb 주석. 기본 1.0 = 기존 동작 그대로.
        self.declare_parameter('speed_sign', 1.0)
        # 바이어스 추정
        self.declare_parameter('bias_warmup_sec', 3.0)
        self.declare_parameter('bias_alpha', 0.002)      # 정지 중 갱신 속도
        self.declare_parameter('stationary_speed', 0.05)     # m/s
        self.declare_parameter('stationary_rate_deg', 2.0)   # deg/s

        self.use_imu = self.p('imu_yaw')
        self.axis = AXIS_IDX.get(str(self.p('yaw_axis')).lower(), 2)
        self.sign = float(self.p('yaw_sign'))
        self.speed_sign = float(self.p('speed_sign'))
        self.to_rad = (math.pi / 180.0
                       if str(self.p('gyro_units')).lower().startswith('deg') else 1.0)

        # 적분 상태
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        self.last_t = None

        # 바이어스 상태
        self.bias = 0.0
        self.bias_ready = False
        self.warm = []
        self.warm_t0 = None
        self.last_odom = None
        self.last_report = None

        self._imu_frame = str(self.p('imu_frame'))
        self.to_ms2 = (9.80665
                       if str(self.p('accel_units')).lower() == 'g' else 1.0)

        self.pub = self.create_publisher(Odometry, self.p('out_topic'), 10)
        self.imu_pub = (self.create_publisher(Imu, self.p('imu_out_topic'), 20)
                        if self.p('imu_out_topic') else None)
        self.tfb = TransformBroadcaster(self) if self.p('publish_tf') else None
        self.create_subscription(Odometry, self.p('odom_topic'), self.odom_cb, 20)
        self.create_subscription(Imu, self.p('imu_topic'), self.imu_cb,
                                 qos_profile_sensor_data)

        mode = 'IMU yaw 사용' if self.use_imu else 'pass-through (기존 odom 그대로)'
        self.get_logger().info(
            f'ImuOdom 시작 — {mode}, 축={self.p("yaw_axis")}({self.sign:+.0f}), '
            f'속도부호={self.speed_sign:+.0f}, '
            f'단위={self.p("gyro_units")}')
        if not self.use_imu:
            self.get_logger().warn(
                'imu_yaw=false — TF 만 중계합니다. yaw 는 여전히 서보 명령 기반입니다.')

    def p(self, name):
        return self.get_parameter(name).value

    # ── 휠 오도메트리: 병진 속도만 가져온다 ────────────────────
    def odom_cb(self, msg: Odometry):
        # ★ speed_sign — /odom 의 병진 속도 부호를 이 차 기준으로 맞춘다.
        #   명령과 오도메트리의 부호 처리가 **비대칭**이라 필요하다:
        #       ackermann_to_vesc :  erpm = gain × v          (음수 없음)
        #       vesc_to_odom      :  v    = (−erpm) / gain    (음수 있음)
        #   2호차는 모터 결선이 반대라 gain 이 음수인데, 그러면 명령은 맞고
        #   오도메트리만 한 번 더 뒤집힌다. 조이스틱 전진은 정상인데 RViz 에서만
        #   차가 뒤로 가는 것으로 보이고, 그 위치가 그대로 TF 로 나가 SLAM 이
        #   "후진 중" 으로 믿는다 (2026-08-10 실차에서 관측).
        #   vesc_to_odom 은 C++ 이고 f1tenth_stack 소유라 1호차와 공유한다 —
        #   여기서 차량별로 맞추는 게 안전하다.
        #   ※ 값은 config/vehicle_car*.yaml 의 localization.speed_sign 이 원본.
        #     반드시 **실측으로** 정할 것: 차를 앞으로 밀며
        #       ros2 topic echo /odom_imu --field twist.twist.linear.x
        #     가 양수여야 한다.
        self.speed = msg.twist.twist.linear.x * self.speed_sign
        self.last_odom = msg
        if not self.use_imu:
            # 중계 모드: 들어온 그대로 내보낸다 (기존 동작과 동일)
            self.publish(msg.header.stamp,
                         msg.pose.pose.position.x,
                         msg.pose.pose.position.y,
                         yaw_from_quat(msg.pose.pose.orientation),
                         msg.twist.twist.angular.z)

    # ── 단위만 표준으로 바꾼 IMU 재발행 (EKF 입력용) ──────────
    # imu_yaw 설정과 무관하게 항상 낸다 — EKF 를 쓰든 안 쓰든 표준 토픽이
    # 있어야 rviz·rosbag·robot_localization 이 같은 값을 본다.
    def _republish_imu(self, msg: Imu):
        if self.imu_pub is None:
            return
        o = Imu()
        o.header.stamp = msg.header.stamp
        # ★ 원시 메시지는 frame_id 가 비어 있다 — 여기서 채워야 EKF 가
        #   base_link↔imu_link static TF 로 축(전방=-y, 좌측=+x)을 바로잡는다.
        #   축을 여기서 직접 돌리지 않는 이유: 그러면 레버암(센서~무게중심
        #   거리)이 무시된다. TF 에 맡기면 회전과 병진을 함께 처리한다.
        o.header.frame_id = self._imu_frame
        o.angular_velocity.x = msg.angular_velocity.x * self.to_rad
        o.angular_velocity.y = msg.angular_velocity.y * self.to_rad
        o.angular_velocity.z = msg.angular_velocity.z * self.to_rad
        o.linear_acceleration.x = msg.linear_acceleration.x * self.to_ms2
        o.linear_acceleration.y = msg.linear_acceleration.y * self.to_ms2
        o.linear_acceleration.z = msg.linear_acceleration.z * self.to_ms2
        # 자세(orientation)는 이 IMU 가 주지 않는다 — REP-145 규약대로
        # 공분산 [0] 을 -1 로 두어 "없음" 을 알린다. 이걸 빼먹으면 EKF 가
        # 단위 쿼터니언(0,0,0,1)을 "정북 고정" 자세로 믿고 yaw 를 묶어버린다.
        o.orientation_covariance[0] = -1.0
        o.angular_velocity_covariance[0] = 0.0
        o.linear_acceleration_covariance[0] = 0.0
        self.imu_pub.publish(o)

    # ── IMU: yaw 적분 + 위치 추측항법 ──────────────────────────
    def imu_cb(self, msg: Imu):
        self._republish_imu(msg)
        if not self.use_imu:
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t <= 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        w_raw = (msg.angular_velocity.x, msg.angular_velocity.y,
                 msg.angular_velocity.z)[self.axis] * self.sign * self.to_rad

        # 정지 판정 — 바퀴가 안 돌고 자이로도 잠잠할 때만 바이어스를 건드린다.
        # 속도만 보면 제자리에서 밀리는 경우를 놓치고, 자이로만 보면 등속
        # 직진 주행을 정지로 오인한다. 둘 다 만족해야 한다.
        rate_thr = math.radians(float(self.p('stationary_rate_deg')))
        still = (abs(self.speed) < float(self.p('stationary_speed'))
                 and abs(w_raw - self.bias) < rate_thr)

        if still:
            if not self.bias_ready:
                if self.warm_t0 is None:
                    self.warm_t0 = t
                self.warm.append(w_raw)
                if t - self.warm_t0 >= float(self.p('bias_warmup_sec')) and len(self.warm) > 20:
                    self.bias = float(np.mean(self.warm))
                    self.bias_ready = True
                    self.get_logger().info(
                        f'자이로 바이어스 확정: {math.degrees(self.bias):+.4f} deg/s '
                        f'({len(self.warm)}샘플) — 정지 시마다 갱신됩니다')
            else:
                a = float(self.p('bias_alpha'))
                self.bias += a * (w_raw - self.bias)
        elif not self.bias_ready:
            # 바이어스도 못 잰 채 움직이기 시작했다 — 적분은 하되 경고
            self.warm.clear()
            self.warm_t0 = None
            if self.last_report is None or t - self.last_report > 5.0:
                self.last_report = t
                self.get_logger().warn(
                    '정지 구간 없이 주행 시작 — 자이로 바이어스 미보정 상태입니다. '
                    '출발 전 몇 초 세워두면 자동으로 잡힙니다.')

        w = w_raw - self.bias

        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        if dt <= 0.0 or dt > 0.5:      # 시계 점프·끊김 방어
            return

        # 추측항법: 회전은 자이로, 병진은 휠 오도메트리
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt

        self.publish(msg.header.stamp, self.x, self.y, self.yaw, w)

    # ── 발행 ───────────────────────────────────────────────────
    def publish(self, stamp, x, y, yaw, w):
        qx, qy, qz, qw = quat_from_yaw(yaw)

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = self.p('odom_frame')
        od.child_frame_id = self.p('base_frame')
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.x = qx
        od.pose.pose.orientation.y = qy
        od.pose.pose.orientation.z = qz
        od.pose.pose.orientation.w = qw
        od.twist.twist.linear.x = self.speed
        od.twist.twist.angular.z = w
        self.pub.publish(od)

        if self.tfb is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.p('odom_frame')
            tf.child_frame_id = self.p('base_frame')
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tfb.sendTransform(tf)


def main():
    rclpy.init()
    node = ImuOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
