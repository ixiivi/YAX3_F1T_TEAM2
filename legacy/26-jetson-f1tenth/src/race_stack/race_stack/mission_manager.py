#!/usr/bin/env python3
"""
Mission Manager — 랩 카운트 + 제한시간 + 자동 정지.

/amcl_pose로 출발선(start_x, start_y, start_radius) 통과를 감지해
랩을 센다. 설정 랩수 완주 또는 제한시간 도달 시 /race/enabled=False를
발행해 컨트롤러를 멈춘다 (Fully Autonomous: 사람이 손 안 대고 자동 정지).

발행: /race/enabled (Bool, latched), /race/lap (Int32)
"""
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32, Int32


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('laps', 0)            # 0 = 무제한
        self.declare_parameter('time_limit_sec', 0)  # 0 = 무제한
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_radius', 1.5)
        # ── 출발 게이트 ────────────────────────────────────────────────
        # 대회에서는 출발선에 세워둔 채 스택을 미리 띄우고, 출발 신호에 맞춰
        # 바로 나가야 한다. 기동에 8초가 걸리므로 신호를 듣고 실행하면 늦는다.
        # 게이트를 켜면 /race/enabled=false 로 잡아두고 트리거를 기다린다.
        #   · 터미널 Enter (같은 터미널. /dev/tty 를 직접 열어 읽는다)
        #   · 조이스틱 버튼 (F710 기준 B=1)
        #   · 다른 터미널에서 ./race.sh go
        # 부수 효과로 **AMCL 이 수렴할 시간**이 생긴다 — 출발과 동시에 수렴을
        # 시작하던 예전 방식보다 안정적이다.
        self.declare_parameter('start_gate',   False)
        self.declare_parameter('start_button', 1)      # F710: A=0 B=1 X=2 Y=3
        self.declare_parameter('start_key',    True)   # 터미널 Enter 허용

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enabled_pub = self.create_publisher(Bool, '/race/enabled', latched)
        self.lap_pub = self.create_publisher(Int32, '/race/lap', latched)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.pose_cb, 10)

        self.lap = 0
        self.inside = True          # 시작 시 출발선 안에 있다고 가정
        self.armed = False          # 한 번 출발선을 벗어나야 카운트 시작
        self.start_time = self.get_clock().now()
        self.finished = False
        self.lap_start = self.start_time

        # ── 출발 게이트 상태 ───────────────────────────────────────────
        self.gate = bool(self.p('start_gate'))
        self.started = not self.gate      # 게이트가 꺼져 있으면 즉시 출발
        self._amcl_ok = False             # /amcl_pose 를 받아봤나
        self._amcl_last = None            # (x, y, 공분산 대각합)
        self._prev_btn = 0
        self._go_lock = threading.Lock()
        # ── 출발 준비 판정 ─────────────────────────────────────────────
        # "노드가 떴다" 와 "출발해도 된다" 는 다르다. 셋이 다 되어야 준비 완료:
        #   1) auto_localize 가 스캔을 맵에 맞춰 /initialpose 를 쐈나
        #   2) 그 결과로 AMCL 이 /amcl_pose 를 내고 있나
        #   3) 컨트롤러가 살아서 /drive 를 발행하고 있나
        # 하나라도 빠진 채 출발하면 초반 몇 미터를 틀린 위치로 달린다.
        self._fit = None                  # 정합 평균오차 (m)
        self._drive_ok = False            # 컨트롤러가 /drive 를 내고 있나
        self._ready = False               # 준비 완료 배너를 띄웠나
        self._force_t = None              # 미준비 상태에서 버튼 누른 시각

        # ── 수동 개입 ──────────────────────────────────────────────────
        # 조이스틱 LB 로 사람이 조종을 가져가면 컨트롤러를 멈춘다.
        # 나머지 노드(AMCL·LiDAR·맵)는 그대로 두므로 위치 추정이 끊기지 않고,
        # 다시 자율로 넘기면 그 자리에서 이어서 달린다.
        self.manual = False
        self.create_subscription(
            Bool, '/race/manual', self.manual_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.set_enabled(self.started)
        self.create_timer(0.5, self.check_time)
        self.create_timer(0.5, self._republish_enabled)

        if self.gate:
            latched_in = QoSProfile(depth=1,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Joy, '/joy', self.joy_cb, 10)
            self.create_subscription(Bool, '/race/go', self.go_cb, 10)
            self.create_subscription(Float32, '/race/localized',
                                     self.fit_cb, latched_in)
            self.create_subscription(AckermannDriveStamped, '/drive',
                                     self.drive_cb, 10)
            self.create_timer(2.0, self._wait_banner)
            if bool(self.p('start_key')):
                threading.Thread(target=self._key_thread, daemon=True).start()
            self.get_logger().info('초기화 중 — 위치 정렬을 기다립니다…')
        self.get_logger().info(
            f"MissionManager: laps={self.p('laps')}, "
            f"time_limit={self.p('time_limit_sec')}s, "
            f"start_gate={'ON' if self.gate else 'off'}")

    # ── 출발 게이트 ────────────────────────────────────────────────────────

    def _missing(self):
        """아직 안 된 준비 항목 목록."""
        m = []
        if self._fit is None:
            m.append('위치 정렬(auto_localize)')
        if not self._amcl_ok:
            m.append('AMCL pose')
        if not self._drive_ok:
            m.append('컨트롤러(/drive)')
        return m

    def _wait_banner(self):
        """준비될 때까지 남은 항목을 2초마다 보여주고, 다 되면 한 번만 크게 알린다."""
        if self.started:
            return
        missing = self._missing()
        if missing:
            self.get_logger().info(f'초기화 중 — 남은 항목: {", ".join(missing)}')
            return
        if not self._ready:
            self._ready = True
            x, y, cov = self._amcl_last
            btn = self.p('start_button')
            self.get_logger().info(
                '\n'
                '╔══════════════════════════════════════════════════════╗\n'
                '║              출  발  준  비  완  료                  ║\n'
                '╠══════════════════════════════════════════════════════╣\n'
                f'║  위치 정합  평균오차 {self._fit*100:5.1f} cm                      ║\n'
                f'║  AMCL       ({x:+.2f}, {y:+.2f})  공분산합 {cov:.3f}          ║\n'
                '║  컨트롤러   OK                                       ║\n'
                '╠══════════════════════════════════════════════════════╣\n'
                f'║  조이스틱 버튼{btn} 을 누르면 출발합니다              ║\n'
                '╚══════════════════════════════════════════════════════╝')
        else:
            x, y, cov = self._amcl_last
            self.get_logger().info(
                f'[출발 대기] 정합 {self._fit*100:.1f}cm | AMCL ({x:+.2f}, {y:+.2f}) '
                f'공분산합 {cov:.3f} — 버튼을 누르세요')

    def manual_cb(self, msg: Bool):
        if bool(msg.data) == self.manual:
            return
        self.manual = bool(msg.data)
        if self.manual:
            self.get_logger().warn('수동 개입 — 컨트롤러 정지 (/race/enabled=false)')
            self.set_enabled(False)
        elif self.finished:
            self.get_logger().info('자율 복귀 요청 — 이미 완주/정지 상태라 무시')
        elif self.started:
            self.get_logger().info('자율 복귀 — 현재 위치에서 주행을 이어갑니다')
            self.set_enabled(True)
        else:
            self.get_logger().info('자율 복귀 — 아직 출발 전이라 게이트에서 대기')

    def fit_cb(self, msg: Float32):
        self._fit = float(msg.data)
        self.get_logger().info(f'위치 정렬 완료 — 평균오차 {self._fit*100:.1f}cm')

    def drive_cb(self, msg: AckermannDriveStamped):
        self._drive_ok = True

    def _key_thread(self):
        """터미널 Enter 대기.

        ros2 launch 로 띄운 노드는 stdin 이 터미널이 아니라서 input() 이 안 된다.
        제어 터미널이 살아 있으면 /dev/tty 는 열리므로 그쪽을 직접 읽는다.
        (tmux·nohup 등 tty 가 없으면 조용히 포기하고 조이스틱/`race.sh go` 로 간다)
        """
        try:
            with open('/dev/tty') as tty:
                for _ in tty:
                    self.go('Enter 키')
                    return
        except Exception as e:
            self.get_logger().info(
                f'터미널 Enter 대기 불가 ({type(e).__name__}) — '
                '조이스틱 버튼이나 ./race.sh go 를 쓰세요')

    def joy_cb(self, msg: Joy):
        i = int(self.p('start_button'))
        if i < len(msg.buttons):
            btn = msg.buttons[i]
            if btn and not self._prev_btn:      # 눌린 순간에만 (엣지)
                self.go(f'조이스틱 버튼{i}')
            self._prev_btn = btn

    def go_cb(self, msg: Bool):
        if msg.data:
            self.go('./race.sh go')

    def go(self, src: str):
        # 준비가 안 됐으면 한 번은 막는다 — 무엇이 빠졌는지 알려주는 게 목적이다.
        # 다만 대회에서 못 나가는 것이 최악이므로, 3초 안에 다시 누르면 강행한다.
        missing = self._missing()
        if missing and not self.started:
            now = self.get_clock().now()
            recent = (self._force_t is not None
                      and (now - self._force_t).nanoseconds * 1e-9 < 3.0)
            if not recent:
                self._force_t = now
                self.get_logger().warn(
                    f'⚠ 아직 준비 안 됨: {", ".join(missing)} — '
                    '그래도 출발하려면 3초 안에 한 번 더 누르세요')
                return
            self.get_logger().warn(f'⚠ 강제 출발 ({", ".join(missing)} 미완료)')

        with self._go_lock:
            if self.started:
                return
            self.started = True
        # 랩타임·제한시간은 기동 시각이 아니라 **출발 시각**부터 센다
        self.start_time = self.get_clock().now()
        self.lap_start = self.start_time
        if not self._amcl_ok:
            self.get_logger().warn(
                '⚠ AMCL pose 미수신 상태로 출발합니다 — init_pose 가정으로 달립니다')
        self.get_logger().info(f'════════ 출발! ({src}) ════════')
        if self.manual:
            self.get_logger().warn(
                '수동 개입 중이라 컨트롤러는 아직 멈춰 있습니다 — '
                'LB 로 자율로 되돌리면 주행을 시작합니다')
            return
        self.set_enabled(True)

    def p(self, name):
        return self.get_parameter(name).value

    def set_enabled(self, val: bool):
        self._enabled_state = val
        self.enabled_pub.publish(Bool(data=val))

    def _republish_enabled(self):
        """현재 상태를 주기적으로 다시 쏜다 (0.5s).

        구독자는 래치로 받게 고쳤지만, 그래도 한 겹 더 둔다 — 컨트롤러가
        늦게 뜨거나 재시작해도 몇백 ms 안에 올바른 상태로 수렴한다.
        상태를 새로 만드는 게 아니라 이미 정해진 값을 반복할 뿐이라 부작용이 없다.
        """
        self.enabled_pub.publish(Bool(data=getattr(self, '_enabled_state', False)))

    def check_time(self):
        tl = self.p('time_limit_sec')
        if self.finished or not self.started or tl <= 0:
            return
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed >= tl:
            self.get_logger().info(f'제한시간 {tl}s 도달 → 정지')
            self.finish()

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        # 대기 중에도 AMCL 상태는 추적한다 (출발 판단 근거)
        c = msg.pose.covariance
        self._amcl_ok = True
        self._amcl_last = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                           float(c[0] + c[7] + c[35]))   # x, y, yaw 분산 합
        if self.finished or not self.started:
            return
        dx = msg.pose.pose.position.x - self.p('start_x')
        dy = msg.pose.pose.position.y - self.p('start_y')
        inside_now = math.hypot(dx, dy) < self.p('start_radius')

        if not inside_now:
            self.armed = True
        if inside_now and not self.inside and self.armed:
            # 출발선 재진입 = 1랩 완료
            self.lap += 1
            now = self.get_clock().now()
            lap_time = (now - self.lap_start).nanoseconds * 1e-9
            self.lap_start = now
            self.lap_pub.publish(Int32(data=self.lap))
            self.get_logger().info(f'LAP {self.lap} 완료! ({lap_time:.2f}s)')
            if 0 < self.p('laps') <= self.lap:
                self.get_logger().info('목표 랩수 완주 → 정지')
                self.finish()
        self.inside = inside_now

    def finish(self):
        self.finished = True
        self.set_enabled(False)


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl-C 로 컨텍스트가 이미 닫힌 뒤면 발행이 예외를 던진다.
        # 종료 경로에서만 나는 노이즈라 조건을 두고 조용히 넘긴다.
        if rclpy.ok():
            node.set_enabled(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
