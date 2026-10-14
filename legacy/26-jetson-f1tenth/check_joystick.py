#!/usr/bin/env python3
"""
조이스틱 입력값 직접 확인 스크립트
ROS2 불필요 — Python 표준 라이브러리만 사용

실행:  python3 check_joystick.py
종료:  Ctrl+C

★★ 주의: 이 스크립트의 트리거 부호는 ROS /joy 와 반대다 ★★
  이 스크립트(커널 /dev/input/js0): RT 안 누름 = -1,  끝까지 = +1
  ROS2 joy 노드의 /joy 토픽       : RT 안 누름 = +1,  끝까지 = -1

  race_script.yaml 의 trigger_invert 는 /joy 기준이므로 true 가 정상이다.
  여기 보이는 -1 을 보고 false 로 맞추면 RT를 안 눌러도 최고 속도가 나간다.
  실제 주행에 쓰이는 값을 보려면 ROS 쪽에서:
      ros2 topic echo /joy
"""
import struct
import sys
import os

DEVICE     = '/dev/input/js0'
EVENT_SIZE = 8          # time(4B) + value(2B) + type(1B) + number(1B)
TYPE_BTN   = 0x01
TYPE_AXIS  = 0x02
TYPE_INIT  = 0x80

def bar(val, width=20):
    """[-1, 1] 값을 막대그래프로 표시"""
    pos = int((val + 1) / 2 * width)
    b = [' '] * width
    b[width // 2] = '|'   # 중앙선
    if 0 <= pos < width:
        b[pos] = '█'
    return ''.join(b)

def render(axes, buttons):
    lines = []
    lines.append('━' * 52)
    lines.append('  조이스틱 입력 모니터  (Ctrl+C 종료)')
    lines.append('━' * 52)

    lines.append('  [ 축 (Axes) ]')
    for i, v in sorted(axes.items()):
        marker = ' ◀ 데드맨 RB' if i == 99 else ''  # placeholder
        lines.append(f'  축{i:2d}  [{bar(v)}]  {v:+.3f}')

    lines.append('')
    lines.append('  [ 버튼 (Buttons) ]')
    row = '  '
    for i, v in sorted(buttons.items()):
        state = f'[{i}:■]' if v else f'[{i}: ]'
        row += state + ' '
        if (i + 1) % 6 == 0:
            lines.append(row)
            row = '  '
    if row.strip():
        lines.append(row)

    pressed = [i for i, v in buttons.items() if v]
    lines.append('')
    if pressed:
        lines.append(f'  ★ 눌린 버튼: {pressed}')
    else:
        lines.append('  (버튼 없음)')
    lines.append('━' * 52)
    return lines

def main():
    if not os.path.exists(DEVICE):
        print(f'오류: 조이스틱 장치를 찾을 수 없음 → {DEVICE}')
        print('다른 번호라면: python3 check_joystick.py /dev/input/js1')
        sys.exit(1)

    device = sys.argv[1] if len(sys.argv) > 1 else DEVICE
    axes    = {}
    buttons = {}

    print(f'연결됨: {device}')
    print('스틱/버튼을 움직여보세요.\n')

    try:
        with open(device, 'rb') as js:
            while True:
                raw = js.read(EVENT_SIZE)
                if not raw:
                    break
                _time, value, etype, number = struct.unpack('IhBB', raw)
                etype &= ~TYPE_INIT   # init 플래그 제거

                if etype == TYPE_AXIS:
                    axes[number] = value / 32767.0
                elif etype == TYPE_BTN:
                    buttons[number] = value

                # 화면 갱신
                lines = render(axes, buttons)
                # 커서를 위로 올려서 덮어쓰기
                if axes or buttons:
                    sys.stdout.write(f'\033[{len(lines)}A')
                for line in lines:
                    sys.stdout.write('\r' + line + '\033[K\n')
                sys.stdout.flush()

    except KeyboardInterrupt:
        print('\n종료.')
    except PermissionError:
        print(f'권한 없음. 다음 명령 후 다시 실행:\n  sudo chmod a+r {device}')

if __name__ == '__main__':
    main()
