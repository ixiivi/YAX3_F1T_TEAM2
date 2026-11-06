#!/usr/bin/env python3
"""F1TENTH control panel: start/stop the stack, save maps, edit waypoints, drive.

Run with ~/f1tenth_ws/panel.sh (sources ROS). Every program runs as a child process; its
output goes to a log tab. Manual driving and E-STOP publish on /teleop (mux priority 100),
so they override autonomous /drive commands at any time.
"""
import glob
import math
import os
import signal
import sys
import threading
import time

import rclpy
import rclpy.executors
from ackermann_msgs.msg import AckermannDriveStamped
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vesc_msgs.msg import VescStateStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import keyboard_teleop as kt  # noqa: E402  (speed steps / ramps shared with the terminal teleop)

WS = os.path.expanduser('~/f1tenth_ws')
MAPS = os.path.join(WS, 'maps')
WPS = os.path.join(WS, 'waypoints')
SETUP = f'source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash'
SLAM_PARAMS = f'{WS}/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml'
ERPM_GAIN = 4614.0   # speed_to_erpm_gain in vesc.yaml


class Proc(QtCore.QObject):
    """One child program in its own process group, output streamed to a log widget."""
    changed = QtCore.pyqtSignal()

    def __init__(self, name, log):
        super().__init__()
        self.name, self.log = name, log
        self.p = None

    def running(self):
        return self.p is not None and self.p.state() != QtCore.QProcess.NotRunning

    def start(self, cmd):
        if self.running():
            return
        self.p = QtCore.QProcess(self)
        self.p.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        env = QtCore.QProcessEnvironment.systemEnvironment()
        env.insert('PYTHONUNBUFFERED', '1')
        env.insert('RCUTILS_COLORIZED_OUTPUT', '0')
        self.p.setProcessEnvironment(env)
        self.p.readyReadStandardOutput.connect(self._read)
        self.p.finished.connect(self._finished)
        self.log.appendPlainText(f'$ {cmd}\n')
        # setsid: own process group, so stop() reaches every node a launch file started
        self.p.start('setsid', ['bash', '-c', f'{SETUP} && exec {cmd}'])
        self.changed.emit()

    def stop(self):
        if not self.running():
            return
        pid = int(self.p.processId())
        try:
            os.killpg(pid, signal.SIGINT)   # like Ctrl+C: launch shuts its nodes down cleanly
        except ProcessLookupError:
            return
        # escalate if it does not exit
        QtCore.QTimer.singleShot(8000, lambda: self._kill(pid, signal.SIGTERM))
        QtCore.QTimer.singleShot(12000, lambda: self._kill(pid, signal.SIGKILL))

    def _kill(self, pid, sig):
        if self.running() and int(self.p.processId()) == pid:
            try:
                os.killpg(pid, sig)
            except ProcessLookupError:
                pass

    def _read(self):
        text = bytes(self.p.readAllStandardOutput()).decode(errors='replace')
        self.log.moveCursor(QtGui.QTextCursor.End)
        self.log.insertPlainText(text)
        self.log.moveCursor(QtGui.QTextCursor.End)

    def _finished(self, code, _status):
        self.log.appendPlainText(f'\n[{self.name} exited, code {code}]\n')
        self.changed.emit()


class RosLink:
    """Status subscriptions plus the /teleop and pure pursuit command publishers."""

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('control_panel')
        self.core = None
        self.core_t = 0.0
        self.scan_times = []
        self.pp_status = ''
        self.pp_status_t = 0.0
        self.events = []
        n = self.node
        n.create_subscription(VescStateStamped, '/sensors/core', self._core, 10)
        n.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        n.create_subscription(String, '/pure_pursuit/status', self._pp, 10)
        n.create_subscription(String, '/pure_pursuit/event', lambda m: self.events.append(m.data), 10)
        self.teleop_pub = n.create_publisher(AckermannDriveStamped, '/teleop', 10)
        self.pp_cmd_pub = n.create_publisher(String, '/pure_pursuit/command', 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, n)
        # callbacks run in a background thread; they only store plain values the GUI reads
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(n)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _core(self, m):
        self.core, self.core_t = m.state, time.monotonic()

    def _scan(self, _m):
        now = time.monotonic()
        self.scan_times = [t for t in self.scan_times if now - t < 1.0] + [now]

    def _pp(self, m):
        self.pp_status, self.pp_status_t = m.data, time.monotonic()

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        age = (self.node.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
        if age > 1.0:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def teleop(self, speed, steer):
        m = AckermannDriveStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.drive.speed, m.drive.steering_angle = float(speed), float(steer)
        self.teleop_pub.publish(m)

    def pp(self, cmd):
        self.pp_cmd_pub.publish(String(data=cmd))


def nofocus(w):
    # buttons must not take keyboard focus, otherwise space/WASD would press them
    w.setFocusPolicy(QtCore.Qt.NoFocus)
    return w


class Panel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('F1TENTH 제어판')
        self.setFocusPolicy(QtCore.Qt.StrongFocus)   # receives WASD when nothing else has focus
        self.ros = RosLink()
        self.manual = False
        self.loc_map = None        # map the running localization was started with
        self.estop = False
        self.target = self.speed = self.steer = 0.0
        self.last_t = self.stopped_at = time.monotonic()

        self.tabs = nofocus(QtWidgets.QTabWidget())
        self.procs = {}
        for key, label in [('bringup', '드라이버'), ('slam', 'SLAM'), ('localize', '위치 추정'),
                           ('pp', '자율주행'), ('rviz', 'RViz'), ('editor', '웨이포인트 편집'),
                           ('tool', '맵 저장/위치 찾기')]:
            log = QtWidgets.QPlainTextEdit(readOnly=True)
            log.setMaximumBlockCount(3000)
            log.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
            nofocus(log)
            self.tabs.addTab(log, label)
            pr = Proc(label, log)
            pr.changed.connect(self.refresh)
            self.procs[key] = pr

        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(self._status_row())
        top = QtWidgets.QHBoxLayout()
        left = QtWidgets.QVBoxLayout()
        left.addWidget(self._run_box())
        left.addWidget(self._map_box())
        left.addWidget(self._auto_box())
        top.addLayout(left, 3)
        top.addWidget(self._manual_box(), 2)
        lay.addLayout(top)
        lay.addWidget(self.tabs, 1)

        QtWidgets.QApplication.instance().installEventFilter(self)
        self.timer = QtCore.QTimer(self, interval=50, timeout=self.tick)   # 20 Hz
        self.timer.start()
        self.slow = QtCore.QTimer(self, interval=300, timeout=self.refresh_status)
        self.slow.start()
        self.reload_lists()
        self.refresh()

    # ---------- layout ----------
    def _status_row(self):
        row = QtWidgets.QHBoxLayout()
        self.st = {}
        for k in ('battery', 'vesc', 'lidar', 'pose', 'speed'):
            lb = QtWidgets.QLabel('-')
            lb.setStyleSheet('padding:4px 8px; border-radius:4px; background:#444; color:white')
            row.addWidget(lb)
            self.st[k] = lb
        return row

    def _button(self, text, slot, color=None):
        b = nofocus(QtWidgets.QPushButton(text))
        b.clicked.connect(slot)
        if color:
            b.setStyleSheet(f'background:{color}; color:white; font-weight:bold; padding:6px')
        return b

    def _run_box(self):
        box = QtWidgets.QGroupBox('실행')
        g = QtWidgets.QGridLayout(box)
        self.run_btn = {}
        for r, (key, label) in enumerate([('bringup', '드라이버 (라이다+VESC)'),
                                          ('slam', 'SLAM (맵 만들기)'),
                                          ('localize', '위치 추정 (AMCL)'),
                                          ('rviz', 'RViz')]):
            g.addWidget(QtWidgets.QLabel(label), r, 0)
            b = self._button('켜기', lambda _=False, k=key: self.toggle(k))
            self.run_btn[key] = b
            g.addWidget(b, r, 1)
        return box

    def _map_box(self):
        box = QtWidgets.QGroupBox('맵')
        g = QtWidgets.QGridLayout(box)
        self.map_cb = nofocus(QtWidgets.QComboBox())
        self.map_cb.currentIndexChanged.connect(self.map_changed)
        g.addWidget(QtWidgets.QLabel('맵'), 0, 0)
        g.addWidget(self.map_cb, 0, 1, 1, 2)
        g.addWidget(self._button('새로고침', self.reload_lists), 0, 3)
        self.map_name = QtWidgets.QLineEdit(placeholderText='저장 이름 (비우면 map_월일_시분)')
        g.addWidget(self.map_name, 1, 0, 1, 2)
        self.save_btn = self._button('맵 저장', self.save_map)
        g.addWidget(self.save_btn, 1, 2)
        g.addWidget(self._button('웨이포인트 편집', self.edit_waypoints), 1, 3)
        return box

    def _auto_box(self):
        box = QtWidgets.QGroupBox('자율주행')
        g = QtWidgets.QGridLayout(box)
        self.wp_cb = nofocus(QtWidgets.QComboBox())
        g.addWidget(QtWidgets.QLabel('웨이포인트'), 0, 0)
        g.addWidget(self.wp_cb, 0, 1, 1, 3)
        g.addWidget(QtWidgets.QLabel('최고 속도 [m/s]'), 1, 0)
        self.vmax = QtWidgets.QDoubleSpinBox(minimum=0.5, maximum=3.0, singleStep=0.1, value=1.0)
        g.addWidget(self.vmax, 1, 1)
        self.reloc_btn = self._button('위치 다시 찾기', self.relocalize)
        g.addWidget(self.reloc_btn, 1, 2)
        self.pp_btn = self._button('자율주행 준비', lambda: self.toggle('pp'))
        g.addWidget(self.pp_btn, 1, 3)
        self.go_btn = self._button('출발', self.go, '#2e7d32')
        self.stop_btn = self._button('정지 (감속)', lambda: self.ros.pp('stop'), '#f9a825')
        g.addWidget(self.go_btn, 2, 0, 1, 2)
        g.addWidget(self.stop_btn, 2, 2, 1, 2)
        self.pp_label = QtWidgets.QLabel('-')
        g.addWidget(self.pp_label, 3, 0, 1, 4)
        self.avoid_cb = nofocus(QtWidgets.QCheckBox('장애물 우회', checked=True))
        self.reverse_cb = nofocus(QtWidgets.QCheckBox('위치 틀어지면 지나온 길로 후진', checked=True))
        g.addWidget(self.avoid_cb, 4, 0, 1, 2)
        g.addWidget(self.reverse_cb, 4, 2, 1, 2)
        self.event_label = QtWidgets.QLabel('')
        self.event_label.setWordWrap(True)
        self.event_label.setStyleSheet('color:#e65100; font-weight:bold')
        g.addWidget(self.event_label, 5, 0, 1, 4)
        return box

    def _manual_box(self):
        box = QtWidgets.QGroupBox('수동 주행 / 긴급 정지')
        v = QtWidgets.QVBoxLayout(box)
        self.manual_btn = self._button('키보드 조종 켜기', self.toggle_manual)
        v.addWidget(self.manual_btn)
        self.manual_label = QtWidgets.QLabel('꺼짐')
        self.manual_label.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        v.addWidget(self.manual_label)
        v.addWidget(QtWidgets.QLabel('W/S 속도 ±   A/D 조향   X 조향 중앙\n'
                                     'Space 감속 정지   B 즉시 정지\n'
                                     '(키 한 번 = 한 단계, 꾹 눌러도 한 번)'))
        v.addStretch(1)
        self.estop_btn = self._button('긴급 정지', self.toggle_estop, '#c62828')
        self.estop_btn.setMinimumHeight(70)
        f = self.estop_btn.font()
        f.setPointSize(f.pointSize() + 6)
        self.estop_btn.setFont(f)
        v.addWidget(self.estop_btn)
        return box

    # ---------- actions ----------
    def selected_map(self):
        return self.map_cb.currentData()

    def reload_lists(self):
        cur_map, cur_wp = self.map_cb.currentData(), self.wp_cb.currentData()
        self.map_cb.blockSignals(True)
        self.map_cb.clear()
        for y in sorted(glob.glob(f'{MAPS}/*.yaml'), key=os.path.getmtime, reverse=True):
            self.map_cb.addItem(os.path.basename(y)[:-5], y)
        if cur_map and self.map_cb.findData(cur_map) >= 0:
            self.map_cb.setCurrentIndex(self.map_cb.findData(cur_map))
        self.map_cb.blockSignals(False)
        self.wp_cb.clear()
        for c in sorted(glob.glob(f'{WPS}/*.csv'), key=os.path.getmtime, reverse=True):
            self.wp_cb.addItem(os.path.basename(c), c)
        if cur_wp and self.wp_cb.findData(cur_wp) >= 0:
            self.wp_cb.setCurrentIndex(self.wp_cb.findData(cur_wp))
        else:
            self.map_changed()

    def map_changed(self):
        # pick the waypoint file made for this map, if any
        m = self.map_cb.currentText()
        for i in range(self.wp_cb.count()):
            if self.wp_cb.itemText(i).startswith(m):
                self.wp_cb.setCurrentIndex(i)
                return

    def warn(self, text):
        QtWidgets.QMessageBox.warning(self, 'F1TENTH 제어판', text)

    def toggle(self, key):
        pr = self.procs[key]
        if pr.running():
            if key == 'pp':
                self.ros.pp('estop')
            pr.stop()
            return
        if key == 'slam' and self.procs['localize'].running():
            return self.warn('위치 추정(AMCL)을 먼저 끄세요. SLAM과 동시에 켜면 위치가 충돌해요.')
        if key == 'localize' and self.procs['slam'].running():
            return self.warn('SLAM을 먼저 끄세요. (맵은 저장했나요?)')
        if key in ('localize', 'pp') and not self.selected_map():
            return self.warn('맵이 없어요. SLAM으로 맵을 먼저 만들고 저장하세요.')
        if key == 'pp':
            if not self.procs['localize'].running():
                return self.warn('위치 추정을 먼저 켜세요.')
            if self.map_mismatch():
                return
            if not self.wp_cb.currentData():
                return self.warn('웨이포인트 파일이 없어요. "웨이포인트 편집"으로 만드세요.')
        cmds = {
            'bringup': 'ros2 launch f1tenth_stack bringup_launch.py',
            'slam': f'ros2 launch slam_toolbox online_async_launch.py slam_params_file:={SLAM_PARAMS}',
            'localize': f'ros2 launch waypoint_nav localize_launch.py map:={self.selected_map()}',
            'rviz': 'rviz2',
            'pp': ('ros2 run waypoint_nav pure_pursuit --ros-args '
                   f'-p waypoint_file:={self.wp_cb.currentData()} -p max_speed:={self.vmax.value():.2f} '
                   f'-p map_yaml:={self.selected_map()} '
                   f'-p avoid_enabled:={str(self.avoid_cb.isChecked()).lower()} '
                   f'-p reverse_enabled:={str(self.reverse_cb.isChecked()).lower()}'),
        }
        if key == 'localize':
            self.loc_map = self.selected_map()
        pr.start(cmds[key])
        self.tabs.setCurrentWidget(pr.log)

    def map_mismatch(self):
        # AMCL keeps the map it was started with; pure pursuit and the scan matcher must use
        # the same one, or every pose they compare against the scan is wrong
        if self.selected_map() == self.loc_map:
            return False
        self.warn(f'위치 추정은 {os.path.basename(self.loc_map)[:-5]} 맵으로 켜져 있어요.\n'
                  '맵을 바꾸려면 위치 추정을 끄고 다시 켜세요.')
        return True

    def save_map(self):
        if not self.procs['slam'].running():
            return self.warn('SLAM이 켜져 있을 때만 저장할 수 있어요.')
        name = self.map_name.text().strip() or time.strftime('map_%m%d_%H%M')
        if os.path.exists(f'{MAPS}/{name}.yaml') and QtWidgets.QMessageBox.question(
                self, '덮어쓰기', f'{name} 이(가) 이미 있어요. 덮어쓸까요?') != QtWidgets.QMessageBox.Yes:
            return
        self.procs['tool'].start(f'{WS}/save_map.sh {name}')
        self.tabs.setCurrentWidget(self.procs['tool'].log)
        QtCore.QTimer.singleShot(8000, self.reload_lists)

    def edit_waypoints(self):
        if not self.selected_map():
            return self.warn('맵이 없어요.')
        self.procs['editor'].start(f'ros2 run waypoint_nav waypoint_editor {self.selected_map()}')
        self.tabs.setCurrentWidget(self.procs['editor'].log)

    def relocalize(self):
        if not self.procs['localize'].running():
            return self.warn('위치 추정이 켜져 있어야 해요.')
        if self.map_mismatch():
            return
        self.procs['tool'].start(
            f'ros2 run waypoint_nav global_localize --ros-args -p map:={self.selected_map()}')
        self.tabs.setCurrentWidget(self.procs['tool'].log)

    def go(self):
        # manual mode publishes on /teleop (priority 100) even at speed 0, which would hold
        # the car still under the autonomous commands
        if self.manual:
            self.toggle_manual()
        self.ros.pp('go')

    def toggle_manual(self):
        self.manual = not self.manual
        self.target = self.speed = 0.0
        self.setFocus()
        self.refresh()

    def toggle_estop(self):
        self.estop = not self.estop
        if self.estop:
            self.ros.pp('estop')
            self.target = self.speed = 0.0
        self.refresh()

    # ---------- keyboard (manual mode) ----------
    def eventFilter(self, obj, ev):
        if ev.type() == QtCore.QEvent.KeyPress and self.manual and self.isActiveWindow():
            if isinstance(QtWidgets.QApplication.focusWidget(),
                          (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox)):
                return False   # typing in a field
            if ev.isAutoRepeat():
                return True
            k = ev.key()
            if k == QtCore.Qt.Key_W:
                self.target = kt.step_speed(self.target, 1)
            elif k == QtCore.Qt.Key_S:
                self.target = kt.step_speed(self.target, -1)
            elif k == QtCore.Qt.Key_A:
                self.steer = kt.clamp(self.steer + kt.STEER_STEP, kt.MAX_STEER)
            elif k == QtCore.Qt.Key_D:
                self.steer = kt.clamp(self.steer - kt.STEER_STEP, kt.MAX_STEER)
            elif k == QtCore.Qt.Key_X:
                self.steer = 0.0
            elif k == QtCore.Qt.Key_Space:
                self.target = 0.0
            elif k == QtCore.Qt.Key_B:
                self.target = self.speed = 0.0
            else:
                return False
            return True
        return False

    # ---------- loops ----------
    def tick(self):
        now = time.monotonic()
        dt, self.last_t = now - self.last_t, now
        if self.estop:
            self.ros.teleop(0.0, 0.0)   # priority 100: overrides everything, brakes
        elif self.manual:
            if self.speed != 0.0:
                self.stopped_at = now
            if not (self.speed == 0.0 and self.target != 0.0
                    and now - self.stopped_at < kt.REVERSE_HOLD):
                self.speed = kt.ramp(self.speed, self.target, dt)
            cmd = kt.floor_speed(self.speed, self.steer)
            self.ros.teleop(cmd, self.steer)
            self.manual_label.setText(
                f'속도 {cmd:+.2f} / 목표 {self.target:+.2f} m/s\n조향 {self.steer:+.2f} rad')

    def refresh_status(self):
        r, now = self.ros, time.monotonic()
        while r.events:
            text = r.events.pop(0)
            self.event_label.setText(time.strftime('%H:%M:%S ') + text)
            self.procs['pp'].log.appendPlainText(f'[event] {text}')
            QtCore.QProcess.startDetached('notify-send', ['-a', 'F1TENTH', 'F1TENTH 자율주행', text])

        def put(k, text, ok):
            color = {True: '#2e7d32', False: '#c62828', None: '#f9a825'}[ok]
            self.st[k].setText(text)
            self.st[k].setStyleSheet(f'padding:4px 8px; border-radius:4px; background:{color}; color:white')
        fresh = r.core is not None and now - r.core_t < 1.0
        if fresh:
            v = r.core.voltage_input
            put('battery', f'배터리 {v:.1f} V', True if v >= 14.8 else (None if v >= 14.0 else False))
            put('vesc', f'VESC fault {r.core.fault_code}', r.core.fault_code == 0)
            put('speed', f'속도 {r.core.speed / ERPM_GAIN:+.2f} m/s', True)
        else:
            put('battery', '배터리 -', False)
            put('vesc', 'VESC 연결 없음', False)
            put('speed', '속도 -', False)
        hz = len(r.scan_times)
        put('lidar', f'라이다 {hz} Hz', True if hz >= 30 else (None if hz > 0 else False))
        pose = r.pose()
        if pose:
            put('pose', f'위치 ({pose[0]:.1f}, {pose[1]:.1f}) {math.degrees(pose[2]):.0f}°', True)
        else:
            put('pose', '위치 없음 (SLAM/위치 추정)', None)
        if self.procs['pp'].running():
            text = r.pp_status if now - r.pp_status_t < 1.0 else '자율주행 노드 시작 중...'
            if self.manual or self.estop:
                text = ('⚠ 긴급 정지 중' if self.estop else '⚠ 키보드 조종이 켜져 있어 자율주행이 막혀 있어요') \
                    + '  |  ' + text
            self.pp_label.setText(text)
        else:
            self.pp_label.setText('-')

    def refresh(self):
        for key, b in self.run_btn.items():
            on = self.procs[key].running()
            b.setText('끄기' if on else '켜기')
            b.setStyleSheet('background:#2e7d32; color:white' if on else '')
        pp_on = self.procs['pp'].running()
        self.pp_btn.setText('자율주행 끄기' if pp_on else '자율주행 준비')
        self.go_btn.setEnabled(pp_on and not self.estop)
        self.stop_btn.setEnabled(pp_on)
        self.manual_btn.setText('키보드 조종 끄기' if self.manual else '키보드 조종 켜기')
        self.manual_btn.setStyleSheet('background:#1565c0; color:white' if self.manual else '')
        if not self.manual:
            self.manual_label.setText('꺼짐 (켜면 자율주행보다 우선)')
        self.estop_btn.setText('긴급 정지 해제' if self.estop else '긴급 정지')
        self.estop_btn.setStyleSheet(
            ('background:#555;' if self.estop else 'background:#c62828;')
            + ' color:white; font-weight:bold')

    def closeEvent(self, ev):
        running = [p.name for p in self.procs.values() if p.running()]
        if running and QtWidgets.QMessageBox.question(
                self, '종료', '실행 중인 것을 모두 끄고 종료할까요?\n' + ', '.join(running)
        ) != QtWidgets.QMessageBox.Yes:
            ev.ignore()
            return
        self.ros.pp('estop')
        for _ in range(5):
            self.ros.teleop(0.0, 0.0)
        for p in self.procs.values():
            p.stop()
        deadline = time.monotonic() + 10
        while any(p.running() for p in self.procs.values()) and time.monotonic() < deadline:
            QtWidgets.QApplication.processEvents()
            time.sleep(0.05)
        ev.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = Panel()
    # rclpy.init (inside Panel) installs its own SIGINT/SIGTERM handlers that only shut the
    # ROS context down and leave the Qt window running: make Ctrl+C / kill end the process
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    w.resize(1000, 800)
    w.show()
    code = app.exec_()
    # child programs are already stopped by closeEvent. Tearing down rclpy while its spin
    # thread runs aborts or hangs the interpreter, so leave right away instead.
    sys.stdout.flush()
    os._exit(code)


if __name__ == '__main__':
    main()
