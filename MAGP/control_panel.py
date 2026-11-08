#!/usr/bin/env python3
"""F1TENTH control panel: start/stop the stack, save maps, edit waypoints, drive.

Run with ~/MAGP/MAGP/panel.sh (sources ROS). Every program runs as a child process; its
output goes to a log tab and to logs/latest/<program>.log. Manual driving and E-STOP publish on /teleop (mux priority 100),
so they override autonomous /drive commands at any time.
"""
import glob
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
import rclpy.executors
from ackermann_msgs.msg import AckermannDriveStamped
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vesc_msgs.msg import VescStateStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import keyboard_teleop as kt  # noqa: E402  (speed steps / ramps shared with the terminal teleop)

WS = os.path.expanduser('~/MAGP/MAGP')
MAPS = os.path.join(WS, 'maps')
WPS = os.path.join(WS, 'waypoints')
LOGS = os.path.join(WS, 'logs')
LOG_DIR = os.path.join(LOGS, time.strftime('%Y%m%d_%H%M%S'))  # this panel's copies of the log tabs
LOG_FILE_MAX = 50_000_000   # [characters] per log file
SETUP =f'source /opt/ros/humble/setup.bash && source {WS}/install/setup.bash'
SLAM_PARAMS = f'{WS}/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml'
LOG_LINES = 9
FONT_SCALE = 0.63    # the desktop text scale is 2x, which is too large for a third of a screen
DRIVE_ROW_H = 48     # [px] height of the keyboard / gamepad / E-STOP row
DRIVE_ICON_W = 56    # [px] width of the keyboard and gamepad buttons
STATUS_CARD = 'padding:2px 6px; border-radius:4px; color:white;'
ERPM_GAIN = 4353.0   # speed_to_erpm_gain in vesc.yaml
REST_CURRENT = 1.0   # [A] above this the voltage sags, so the charge estimate is held
# resting LiPo voltage per cell -> state of charge [%]
LIPO_SOC = [(3.27, 0), (3.61, 5), (3.69, 10), (3.71, 15), (3.73, 20), (3.75, 25), (3.77, 30),
            (3.79, 35), (3.80, 40), (3.82, 45), (3.84, 50), (3.85, 55), (3.87, 60), (3.91, 65),
            (3.95, 70), (3.98, 75), (4.02, 80), (4.08, 85), (4.11, 90), (4.15, 95), (4.20, 100)]


def map_label(path):
    return os.path.basename(path)[:-5] if path else '없음'


def waypoint_problem(map_name, wp_path):
    """Why this waypoint file cannot be driven on this map, or None. Waypoint files are named
    <map>_<anything>.csv; a route from another map can still lie mostly inside this one, so
    pure_pursuit's own check on the map area does not catch it."""
    if not wp_path:
        return f'{map_name} 맵의 웨이포인트 파일이 없어요. "웨이포인트 편집"으로 만드세요.'
    name = os.path.basename(wp_path)
    if not name.startswith(f'{map_name}_'):
        return (f'{name} 은(는) {map_name} 맵의 웨이포인트가 아니에요.\n'
                f'{map_name}_wp.csv 를 고르거나 "웨이포인트 편집"으로 만드세요.')
    try:
        with open(wp_path) as f:
            points = sum(1 for line in f if line.strip()) - 1  # without the header
    except OSError as e:
        return f'{name} 을(를) 읽지 못했어요: {e}'
    if points < 2:
        return f'{name} 이(가) 비어 있어요. "웨이포인트 편집"으로 다시 만드세요.'
    return None


def battery_percent(volts, cells):
    """Estimated charge [%] from the resting pack voltage."""
    cell = volts / cells
    if cell <= LIPO_SOC[0][0]:
        return 0.0
    for (v0, p0), (v1, p1) in zip(LIPO_SOC, LIPO_SOC[1:]):
        if cell <= v1:
            return p0 + (p1 - p0) * (cell - v0) / (v1 - v0)
    return 100.0


def log_lines(pending, text):
    """Split program output into finished lines, as (lines, unfinished rest). Terminal status
    updates that overwrite themselves with \\r are dropped: the run log CSV records them."""
    parts = (pending + text).split('\n')
    lines = []
    for part in parts[:-1]:
        line = part.rsplit('\r', 1)[-1]
        # a status line without a newline may sit right in front of a ROS log message
        m = re.search(r'\[(DEBUG|INFO|WARN|ERROR|FATAL)\]', line)
        lines.append(line[m.start():] if m else line)
    return lines, parts[-1].rsplit('\r', 1)[-1]


class LogFile:
    """Copy of one log tab under logs/<panel start time>/<key>.log, one timestamped line at a
    time. A disk problem turns the copy off; it never gets in the way of driving."""

    def __init__(self, key):
        self.path = os.path.join(LOG_DIR, f'{key}.log')
        self.f = None
        self.size = 0
        self.pending = ''
        self.off = False

    def _open(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        latest = os.path.join(LOGS, 'latest')
        if os.path.islink(latest) or not os.path.exists(latest):
            if os.path.islink(latest):
                os.remove(latest)
            os.symlink(os.path.basename(LOG_DIR), latest)
        self.f = open(self.path, 'a', encoding='utf-8', errors='replace')

    def write(self, text):
        lines, self.pending = log_lines(self.pending, text)
        self._put(lines)

    def note(self, text):
        self._put([text])

    def _put(self, lines):
        if self.off or not lines:
            return
        try:
            if self.f is None:
                self._open()
            stamp = time.strftime('%H:%M:%S')
            data = ''.join(f'{stamp} {line}\n' for line in lines)
            if self.size + len(data) > LOG_FILE_MAX:
                data = f'{stamp} [log file full, stopped copying here]\n'
                self.off = True
            self.f.write(data)
            self.f.flush()
            self.size += len(data)
        except OSError:
            self.off = True


class Proc(QtCore.QObject):
    """One child program in its own process group, output streamed to a log widget."""
    changed = QtCore.pyqtSignal()

    def __init__(self, name, log, key):
        super().__init__()
        self.name, self.log = name, log
        self.file = LogFile(key)
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
        self.file.note(f'$ {cmd}')
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
        self.file.write(text)
        self.log.moveCursor(QtGui.QTextCursor.End)
        self.log.insertPlainText(text)
        self.log.moveCursor(QtGui.QTextCursor.End)

    def _finished(self, code, _status):
        self.log.appendPlainText(f'\n[{self.name} exited, code {code}]\n')
        self.file.note(f'[{self.name} exited, code {code}]')
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
        self.joy_t = 0.0
        self.events = []
        n = self.node
        n.create_subscription(VescStateStamped, '/sensors/core', self._core, 10)
        n.create_subscription(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
        n.create_subscription(Joy, '/joy', self._joy, 10)
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

    def _joy(self, _m):
        self.joy_t = time.monotonic()

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


def emoji_pixmap(emoji, size=128):
    """The emoji drawn large and cropped to its visible pixels, for a centred button icon."""
    pm = QtGui.QPixmap(size * 2, size * 2)
    pm.fill(QtCore.Qt.transparent)
    p = QtGui.QPainter(pm)
    f = QtGui.QFont('Noto Color Emoji')
    f.setPixelSize(size)
    p.setFont(f)
    p.drawText(pm.rect(), QtCore.Qt.AlignCenter, emoji)
    p.end()
    img = pm.toImage()
    xs, ys = [], []
    for y in range(0, img.height(), 2):
        for x in range(0, img.width(), 2):
            if img.pixelColor(x, y).alpha() > 20:
                xs.append(x)
                ys.append(y)
    if not xs:
        return pm
    return pm.copy(QtCore.QRect(QtCore.QPoint(min(xs), min(ys)), QtCore.QPoint(max(xs) + 1, max(ys) + 1)))


def fixed_font():
    f = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
    f.setPointSizeF(f.pointSizeF() * FONT_SCALE)
    return f


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
        self.pp_setup = (None, None)  # map and waypoints the running autonomy was started with
        self.estop = False
        self.target = self.speed = self.steer = 0.0
        self.last_t = self.stopped_at = time.monotonic()
        self.joy_live = False      # gamepad on and /joy arriving
        self.brake_until = 0.0     # keep braking this long after the gamepad is turned off
        self.rest_v = None         # smoothed pack voltage at low current, for the charge estimate

        self.tabs = nofocus(QtWidgets.QTabWidget())
        self.procs = {}
        for key, label in [('bringup', '드라이버'), ('slam', 'SLAM'), ('localize', '위치 추정'),
                           ('pp', '자율주행'), ('rviz', 'RViz'), ('editor', '웨이포인트 편집'),
                           ('map_editor', '맵 편집'), ('tool', '맵 저장/위치 찾기'),
                           ('joy', '조이스틱')]:
            log = QtWidgets.QPlainTextEdit(readOnly=True)
            log.setMaximumBlockCount(3000)
            log.setFont(fixed_font())
            log.setMinimumHeight(QtGui.QFontMetrics(fixed_font()).lineSpacing() * LOG_LINES + 8)
            nofocus(log)
            self.tabs.addTab(log, label)
            pr = Proc(label, log, key)
            pr.changed.connect(self.refresh)
            self.procs[key] = pr

        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(self._status_row())
        controls = QtWidgets.QWidget()
        rows = QtWidgets.QVBoxLayout(controls)
        rows.setContentsMargins(0, 0, 0, 0)
        top = QtWidgets.QHBoxLayout()
        left = QtWidgets.QVBoxLayout()
        left.addWidget(self._run_box())
        left.addWidget(self._map_box())
        left.addStretch(1)
        top.addLayout(left)
        top.addWidget(self._auto_box())
        rows.addLayout(top)
        for box in controls.findChildren(QtWidgets.QGroupBox):
            box.layout().setContentsMargins(6, 5, 6, 5)
            box.layout().setSpacing(3)
        # on a small screen the controls scroll; E-STOP stays outside so it is always visible
        self.scroll = nofocus(QtWidgets.QScrollArea(widgetResizable=True))
        self.scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.scroll.setWidget(controls)
        lay.addWidget(self.scroll)
        lay.addLayout(self._drive_row())
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
        # the name sits above each card and the card holds only the value, to keep the row narrow
        row = self.status = QtWidgets.QGridLayout()
        row.setHorizontalSpacing(4)
        row.setVerticalSpacing(1)
        self.st = {}
        for c, (k, name) in enumerate((('battery', '배터리'), ('vesc', 'VESC'), ('lidar', '라이다'),
                                       ('pose', '위치'), ('speed', '속도 (m/s)'))):
            if k != 'battery':   # its name shares a row with the cell count below
                row.addWidget(QtWidgets.QLabel(name), 0, c)
            lb = QtWidgets.QLabel('-', alignment=QtCore.Qt.AlignCenter)
            lb.setStyleSheet(f'{STATUS_CARD} background:#444')
            # always two lines tall, so the row does not jump when values with two lines appear
            lb.setText('0°\n약 0%')   # Hangul and symbols: their fallback fonts have taller lines
            lb.ensurePolished()
            lb.setMinimumHeight(lb.sizeHint().height())
            lb.setText('-')
            row.addWidget(lb, 1, c)
            row.setColumnStretch(c, 1)
            self.st[k] = lb
        self.st['pose'].setToolTip('SLAM이나 위치 추정을 켜면 나와요')
        self.st['battery'].setToolTip('잔량은 선택한 셀 수의 LiPo 기준 전압으로 계산한 추정치예요. '
                                      '주행 중에는 전압이 처져서 값을 갱신하지 않아요.')
        self.settings = QtCore.QSettings('MAGP', 'control_panel')
        self.cells_cb = nofocus(QtWidgets.QComboBox())
        self.cells_cb.setToolTip('배터리 셀 수')
        self.cells_cb.addItem('3S', 3)
        self.cells_cb.addItem('4S', 4)
        self.cells_cb.setCurrentIndex(self.cells_cb.findData(self.settings.value('cells', 3, type=int)))
        self.cells_cb.currentIndexChanged.connect(
            lambda: self.settings.setValue('cells', self.cells_cb.currentData()))
        title = QtWidgets.QHBoxLayout()
        title.addWidget(QtWidgets.QLabel('배터리'))
        # never narrower than its text: squeezed by the column, '3S' was cut off
        self.cells_cb.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        self.cells_cb.setMinimumWidth(self.cells_cb.sizeHint().width())
        title.addWidget(self.cells_cb)
        row.addLayout(title, 0, 0)
        return row

    def _button(self, text, slot, color=None):
        b = nofocus(QtWidgets.QPushButton(text))
        b.clicked.connect(slot)
        if color:
            b.setStyleSheet(f'background:{color}; color:white; font-weight:bold')
        return b

    def _combo(self):
        # size to a short name, so one long file name cannot widen the whole panel
        c = nofocus(QtWidgets.QComboBox())
        c.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToMinimumContentsLengthWithIcon)
        c.setMinimumContentsLength(5)
        return c

    def _run_box(self):
        box = QtWidgets.QGroupBox('실행')
        g = QtWidgets.QGridLayout(box)
        g.setVerticalSpacing(2)
        self.run_btn = {}
        for r, (key, label, tip) in enumerate([('bringup', '드라이버', '라이다 + VESC'),
                                               ('slam', 'SLAM', '맵 만들기'),
                                               ('localize', '위치 추정', 'AMCL'),
                                               ('rviz', 'RViz', '')]):
            name = QtWidgets.QLabel(label)
            name.setToolTip(tip)
            g.addWidget(name, r, 0)
            b = self._button('켜기', lambda _=False, k=key: self.toggle(k))
            b.setToolTip(tip)
            self.run_btn[key] = b
            g.addWidget(b, r, 1)
        g.setColumnStretch(0, 1)   # spare width goes to the labels, not the buttons
        return box

    def _map_box(self):
        box = QtWidgets.QGroupBox('맵')
        g = QtWidgets.QGridLayout(box)
        self.map_cb = self._combo()
        self.map_cb.currentIndexChanged.connect(self.map_changed)
        g.addWidget(self.map_cb, 0, 0)
        g.addWidget(self._button('새로고침', self.reload_lists), 0, 1)
        self.map_name = QtWidgets.QLineEdit(placeholderText='저장 이름')
        self.map_name.setMinimumWidth(10)
        g.addWidget(self.map_name, 1, 0)
        self.save_btn = self._button('맵 저장', self.save_map)
        g.addWidget(self.save_btn, 1, 1)
        g.addWidget(self._button('웨이포인트 편집', self.edit_waypoints), 2, 0)
        self.map_edit_btn = self._button('맵 편집', self.edit_map)
        self.map_edit_btn.setToolTip('벽 지우기/그리기. 처음 저장할 때 원본을 <맵>_orig.pgm으로 보관해요.\n'
                                     '저장한 맵은 위치 추정을 다시 켜야 적용돼요.')
        g.addWidget(self.map_edit_btn, 2, 1)
        g.setColumnStretch(0, 1)
        return box

    def _auto_box(self):
        box = QtWidgets.QGroupBox('자율주행')
        g = QtWidgets.QGridLayout(box)
        self.wp_cb = self._combo()
        g.addWidget(QtWidgets.QLabel('웨이포인트'), 0, 0)
        g.addWidget(self.wp_cb, 0, 1, 1, 3)
        g.addWidget(QtWidgets.QLabel('최고 속도'), 1, 0)
        self.vmax = QtWidgets.QDoubleSpinBox(minimum=0.5, maximum=3.0, singleStep=0.1, value=1.0,
                                             suffix=' m/s')
        g.addWidget(self.vmax, 1, 1, 1, 3)
        self.reloc_btn = self._button('위치 찾기', self.relocalize)
        self.reloc_btn.setToolTip('맵 전체에서 현재 위치를 다시 찾아요')
        g.addWidget(self.reloc_btn, 2, 0, 1, 2)
        self.pp_btn = self._button('자율주행 준비', lambda: self.toggle('pp'))
        g.addWidget(self.pp_btn, 2, 2, 1, 2)
        self.go_btn = self._button('출발', self.go, '#2e7d32')
        self.stop_btn = self._button('정지 (감속)', lambda: self.ros.pp('stop'), '#f9a825')
        g.addWidget(self.go_btn, 3, 0, 1, 2)
        g.addWidget(self.stop_btn, 3, 2, 1, 2)
        self.avoid_cb = nofocus(QtWidgets.QCheckBox('장애물 우회', checked=True))
        self.reverse_cb = nofocus(QtWidgets.QCheckBox('벽에 박으면 후진', checked=True))
        self.reverse_cb.setToolTip('앞바퀴나 범퍼가 벽에 닿으면 지나온 길로 후진해요.\n'
                                    '끄면 충돌 감지도 꺼져요. 위치가 틀어지면 후진 없이 제자리에서 다시 잡아요.')
        g.addWidget(self.avoid_cb, 4, 0, 1, 4)
        g.addWidget(self.reverse_cb, 5, 0, 1, 4)
        # the node's own status lines are in the 자율주행 log tab; here only the last event
        self.event_label = QtWidgets.QLabel('')
        self.event_label.setWordWrap(True)
        self.event_label.setStyleSheet('color:#e65100; font-weight:bold')
        g.addWidget(self.event_label, 6, 0, 1, 4)
        g.setRowStretch(7, 1)
        return box

    def _drive_row(self):
        """Keyboard and gamepad as small square buttons (blue while on), E-STOP beside them.

        Outside the scroll area, so all three are always on screen."""
        row = QtWidgets.QHBoxLayout()
        self.estop_btn = self._button('긴급 정지', self.toggle_estop, '#c62828')
        self.estop_btn.setMinimumHeight(DRIVE_ROW_H)
        f = self.estop_btn.font()
        f.setPointSize(f.pointSize() + 3)
        self.estop_btn.setFont(f)
        self.manual_btn = self._icon_button('\u2328\ufe0f', self.toggle_manual,
                                            '키보드 조종 (켜면 자율주행보다 우선)\n'
                                            'W/S 속도 ±   A/D 조향   X 조향 중앙\n'
                                            'Space 감속 정지   B 즉시 정지\n'
                                            '키 한 번 = 한 단계, 꾹 눌러도 한 번')
        self.joy_btn = self._icon_button('\U0001f579\ufe0f', self.toggle_joy,
                                         '조이스틱 조종\n'
                                         'RB 누른 채 RT 가속, LT 후진, 왼쪽 스틱 조향\n'
                                         'LB 자율주행 허용   둘 다 놓으면 정지\n'
                                         '주황색: 켜져 있지만 패드 입력이 없어요 (연결·전원 확인)')
        row.addWidget(self.manual_btn)
        row.addWidget(self.joy_btn)
        row.addWidget(self.estop_btn, 1)
        return row

    def _icon_button(self, emoji, slot, tip):
        b = self._button('', slot)
        b.setFixedSize(DRIVE_ICON_W, DRIVE_ROW_H)
        # as text the emoji sits off centre (its glyph has uneven side bearings); as an icon cut to
        # the drawn pixels Qt centres it exactly
        b.setIcon(QtGui.QIcon(emoji_pixmap(emoji)))
        b.setIconSize(QtCore.QSize(DRIVE_ROW_H * 6 // 10, DRIVE_ROW_H * 6 // 10))
        b.setToolTip(tip)
        return b

    def place_rviz(self, tries=20):
        """Move RViz beside this window. It ignores -geometry, so find its window once it opens."""
        out = subprocess.run(['xwininfo', '-root', '-tree'],
                             capture_output=True, text=True, check=False).stdout
        found = re.search(r'(0x[0-9a-f]+) "[^"]*RViz[^"]*": \("rviz2"', out)
        if found:
            # wait for RViz to finish applying its saved window size
            QtCore.QTimer.singleShot(2000, lambda: self.move_rviz(int(found.group(1), 16)))
        elif tries > 0:
            QtCore.QTimer.singleShot(500, lambda: self.place_rviz(tries - 1))

    def move_rviz(self, window_id):
        """Fill the screen to the right of this window: two thirds, unless it was resized."""
        area = QtWidgets.QApplication.primaryScreen().availableGeometry()
        left = self.frameGeometry().right() + 1
        QtGui.QWindow.fromWinId(window_id).setGeometry(
            left, area.y(), area.right() + 1 - left, self.height())

    def fit_to_screen(self):
        """Fill the left third of the screen, leaving the other two thirds for RViz."""
        area = QtWidgets.QApplication.primaryScreen().availableGeometry()
        line = self.fontMetrics().height()
        self.resize(area.width() // 3, area.height() - 3 * line)   # title bar and frame
        self.move(area.topLeft())

    def resizeEvent(self, ev):
        # wrapped labels make the controls taller as the window narrows: take that height from
        # the logs, and only scroll once the logs are down to their minimum
        super().resizeEvent(ev)
        lay = self.layout()
        m = lay.contentsMargins()
        fixed = (m.top() + m.bottom() + 3 * lay.spacing() + self.status.sizeHint().height()
                 + self.estop_btn.minimumSizeHint().height() + self.tabs.minimumSizeHint().height())
        controls = self.scroll.widget().layout()
        need = controls.totalHeightForWidth(self.width() - m.left() - m.right())
        if need < 0:   # before the first layout pass
            need = controls.sizeHint().height()
        self.scroll.setMinimumHeight(min(need, max(self.height() - fixed,
                                                   4 * self.fontMetrics().height())))

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
            problem = waypoint_problem(self.map_cb.currentText(), self.wp_cb.currentData())
            if problem:
                return self.warn(problem)
        if key == 'localize' and self.procs['pp'].running():
            return self.warn('자율주행을 먼저 끄세요. 켜져 있는 자율주행은 처음 켤 때의 맵을 계속 써요.')
        cmds = {
            # the gamepad has its own button: it would otherwise hold /teleop whenever connected
            'bringup': 'ros2 launch f1tenth_stack bringup_launch.py use_joy:=false',
            'slam': f'ros2 launch slam_toolbox online_async_launch.py slam_params_file:={SLAM_PARAMS}',
            'localize': f'ros2 launch waypoint_nav localize_launch.py map:={self.selected_map()}',
            # QT_FONT_DPI: the desktop text scale would crowd the RViz toolbar
            'rviz': f'env QT_FONT_DPI=60 rviz2 -d {WS}/f1tenth.rviz',
            'pp': ('ros2 run waypoint_nav pure_pursuit --ros-args '
                   f'-p waypoint_file:={self.wp_cb.currentData()} -p max_speed:={self.vmax.value():.2f} '
                   f'-p map_yaml:={self.selected_map()} '
                   f'-p avoid_enabled:={str(self.avoid_cb.isChecked()).lower()} '
                   f'-p reverse_enabled:={str(self.reverse_cb.isChecked()).lower()}'),
        }
        if key == 'localize':
            self.loc_map = self.selected_map()
        if key == 'pp':
            self.pp_setup = (self.selected_map(), self.wp_cb.currentData())
        pr.start(cmds[key])
        self.tabs.setCurrentWidget(pr.log)
        if key == 'rviz':
            self.place_rviz()

    def map_mismatch(self):
        # AMCL keeps the map it was started with; pure pursuit and the scan matcher must use
        # the same one, or every pose they compare against the scan is wrong
        if self.selected_map() == self.loc_map:
            return False
        self.warn(f'위치 추정은 {os.path.basename(self.loc_map)[:-5]} 맵으로 켜져 있어요.\n'
                  '맵을 바꾸려면 위치 추정을 끄고 다시 켜세요.')
        return True

    def drive_problem(self):
        """Why the running autonomy must not start: it keeps the map and route it was started
        with, whatever the panel shows now."""
        pp_map, pp_wp = self.pp_setup
        now = (self.selected_map(), self.wp_cb.currentData())
        if (pp_map, pp_wp) != now:
            return ('화면에서 고른 맵·웨이포인트와 켜져 있는 자율주행의 설정이 달라요.\n'
                    f'자율주행: {map_label(pp_map)} / {os.path.basename(pp_wp or "") or "없음"}\n'
                    f'화면: {map_label(now[0])} / {os.path.basename(now[1] or "") or "없음"}\n'
                    '자율주행을 끄고 다시 켜세요.')
        if pp_map != self.loc_map:
            return (f'자율주행은 {map_label(pp_map)}, 위치 추정은 {map_label(self.loc_map)} 맵이에요.\n'
                    '같은 맵으로 맞춘 뒤 자율주행을 다시 켜세요.')
        return waypoint_problem(self.map_cb.currentText(), pp_wp)

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

    def edit_map(self):
        if not self.selected_map():
            return self.warn('맵이 없어요.')
        pr = self.procs['map_editor']
        if pr.running():
            return self.warn('맵 편집기가 이미 열려 있어요. 닫은 뒤 다시 눌러 주세요.')
        cmd = f'ros2 run waypoint_nav map_editor {self.selected_map()}'
        # show the route only if it was made for this map
        if self.wp_cb.currentText().startswith(self.map_cb.currentText()):
            cmd += f' --waypoints {self.wp_cb.currentData()}'
        pr.start(cmd)
        self.tabs.setCurrentWidget(pr.log)

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
        problem = self.drive_problem() if self.procs['pp'].running() else None
        if problem:
            return self.warn(problem)
        self.ros.pp('go')

    def toggle_manual(self):
        self.manual = not self.manual
        self.target = self.speed = 0.0
        if self.manual:
            self.procs['joy'].stop()   # both drive /teleop: never run them together
        self.setFocus()
        self.refresh()

    def toggle_joy(self):
        pr = self.procs['joy']
        if pr.running():
            self.stop_joy()
            return
        if self.estop:
            return self.warn('긴급 정지를 먼저 해제하세요.')
        if not self.procs['bringup'].running():
            return self.warn('드라이버를 먼저 켜세요.')
        if self.manual:
            self.toggle_manual()
        pr.start('ros2 launch f1tenth_stack joy_launch.py')
        self.tabs.setCurrentWidget(pr.log)

    def stop_joy(self):
        if self.procs['joy'].running():
            self.procs['joy'].stop()
            self.brake_until = time.monotonic() + 0.5   # the last stick command must not linger

    def toggle_estop(self):
        self.estop = not self.estop
        if self.estop:
            self.ros.pp('estop')
            self.target = self.speed = 0.0
            self.stop_joy()   # LB would otherwise still reach /teleop alongside the zeros
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
        if self.estop or now < self.brake_until:
            self.ros.teleop(0.0, 0.0)   # priority 100: overrides everything, brakes
        elif self.manual:
            if self.speed != 0.0:
                self.stopped_at = now
            if not (self.speed == 0.0 and self.target != 0.0
                    and now - self.stopped_at < kt.REVERSE_HOLD):
                self.speed = kt.ramp(self.speed, self.target, dt)
            cmd = kt.floor_speed(self.speed, self.steer)
            self.ros.teleop(cmd, self.steer)

    def refresh_status(self):
        r, now = self.ros, time.monotonic()
        while r.events:
            text = r.events.pop(0)
            stamped = time.strftime('%H:%M:%S ') + text
            self.event_label.setText(stamped)
            self.procs['pp'].log.appendPlainText(f'[event] {stamped}')

        def put(k, text, ok):
            color = {True: '#2e7d32', False: '#c62828', None: '#f9a825'}[ok]
            self.st[k].setText(text)
            self.st[k].setStyleSheet(f'{STATUS_CARD} background:{color}')
        fresh = r.core is not None and now - r.core_t < 1.0
        if fresh:
            v = r.core.voltage_input
            if self.rest_v is None:
                self.rest_v = v
            elif max(abs(r.core.current_input), abs(r.core.current_motor)) < REST_CURRENT:
                self.rest_v += 0.03 * (v - self.rest_v)   # ~10 s time constant at this 0.3 s timer
            cells = self.cells_cb.currentData()
            soc = 5 * round(battery_percent(self.rest_v, cells) / 5)   # VESC reads in 0.1 V: about 5 % steps
            put('battery', f'{v:.1f} V\n약 {soc}%',
                True if v >= 3.7 * cells else (None if v >= 3.5 * cells else False))
            put('vesc', f'fault {r.core.fault_code}', r.core.fault_code == 0)
            put('speed', f'{r.core.speed / ERPM_GAIN:+.2f}', True)
        else:
            self.rest_v = None   # VESC off: the pack may be swapped before it returns
            put('battery', '-', False)
            put('vesc', '연결 없음', False)
            put('speed', '-', False)
        hz = len(r.scan_times)
        put('lidar', f'{hz} Hz', True if hz >= 30 else (None if hz > 0 else False))
        pose = r.pose()
        if pose:
            put('pose', f'({pose[0]:.1f}, {pose[1]:.1f})\n{math.degrees(pose[2]):.0f}°', True)
        else:
            put('pose', '없음', None)
        live = self.procs['joy'].running() and now - r.joy_t < 1.0
        if live != self.joy_live:
            self.joy_live = live
            self.refresh()

    def refresh(self):
        for key, b in self.run_btn.items():
            on = self.procs[key].running()
            b.setText('끄기' if on else '켜기')
            b.setStyleSheet('background:#2e7d32; color:white' if on else '')
        pp_on = self.procs['pp'].running()
        self.pp_btn.setText('자율주행 끄기' if pp_on else '자율주행 준비')
        self.go_btn.setEnabled(pp_on and not self.estop)
        self.stop_btn.setEnabled(pp_on)
        self.manual_btn.setStyleSheet('background:#1565c0' if self.manual else '')
        # blue while on; amber while on but the pad sends nothing
        joy_on = self.procs['joy'].running()
        self.joy_btn.setStyleSheet(('background:#1565c0' if self.joy_live else 'background:#f9a825')
                                   if joy_on else '')
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


def apply_style(app):
    font = app.font()
    font.setPointSizeF(font.pointSizeF() * FONT_SCALE)
    app.setFont(font)
    # the card border is drawn here so the layout margins measure from it: with the platform
    # style, tight margins let buttons and combo boxes stick out past the border
    app.setStyleSheet('QPushButton { padding: 1px 6px; }'
                      'QGroupBox { border: 1px solid palette(mid); border-radius: 5px;'
                      ' margin-top: 1.2em; }'
                      'QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 2px; }')


def main():
    app = QtWidgets.QApplication(sys.argv)
    apply_style(app)
    w = Panel()
    # rclpy.init (inside Panel) installs its own SIGINT/SIGTERM handlers that only shut the
    # ROS context down and leave the Qt window running: make Ctrl+C / kill end the process
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    w.fit_to_screen()
    w.show()
    code = app.exec_()
    # child programs are already stopped by closeEvent. Tearing down rclpy while its spin
    # thread runs aborts or hangs the interpreter, so leave right away instead.
    sys.stdout.flush()
    os._exit(code)


if __name__ == '__main__':
    main()
