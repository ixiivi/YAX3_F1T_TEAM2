# `tune.sh` 사용법 — 주행 게인 튜닝

주행 → RViz 관찰 → 게인 수정 → 재주행 루프를 위한 도구.
`race_script.yaml` 을 에디터로 열지 않고 값만 바꾼다.

---

## 왜 이게 필요한가

`race.launch.py` 는 **매 실행마다 `race_script.yaml` 을 새로 읽어** ROS 파라미터로 넘긴다.
그래서 값만 바꾸면 다음 `./race.sh q2` 부터 바로 반영된다.

- **`colcon build` 불필요** — yaml 은 코드가 아니라 데이터다
- **스택 재시작만 하면 됨** — 주행을 멈추고 다시 `./race.sh` 하면 끝

에디터로 직접 고쳐도 되지만, `tune.sh` 는 오타로 실차를 벽에 보내는 값
(`gain: 1.5` 를 `15` 로 치는 등)을 막고 변경 이력을 남긴다.

---

## 속도만 빠르게 바꾸기 (SSH)

젯슨에 들어가서:

```bash
cd ~/yax
./tune.sh i          # 번호 고르고 값 입력 → 즉시 yaml 에 저장
./tune.sh -h         # q1/q2/race 가 각각 언제 먹는지 설명
./tune.sh cap=3      # hard_cap 만 (0 = 끔). 모든 모드 공통
./tune.sh q1+=0.1    # ./race.sh q1 전용. q2 / race 속도는 안 변함
./tune.sh race=0.6   # ./race.sh race 전용
```

`grip_util` 은 차량 파일(`config/vehicle_car*.yaml`)로 가고, `hard_cap` 은 `race_script.yaml` 로 간다. 대화형이 알아서 고른다. 다음 `./race.sh` 부터 적용.

---

## 튜닝 루프

```bash
# 1) 지금 상태 확인
./tune.sh status

# 2) 주행 — 노트북에서는 별도 터미널로 ./rviz.sh 띄워 관찰
./race.sh q2

# 3) 관찰 결과대로 게인 조정 (우선순위는 status 가 알려준다)
./tune.sh stanley.ff_gain+=0.3

# 4) 재주행
./race.sh q2
```

`docs/stanley_gain_tuning.md` 의 원칙대로 **한 번에 하나씩** 바꾸고 RViz로 확인할 것.

노트북에서 SSH 한 줄로 바꿔도 된다:

```bash
ssh jetson "cd ~/yax && ./tune.sh stanley.ff_gain+=0.3"
```

---

## 명령어

### 보기

| 명령 | 설명 |
|---|---|
| `./tune.sh i` | ★ 속도 대화형 (hard_cap / grip_util). 번호 고르고 값 입력, 즉시 저장 |
| `./tune.sh cap=3` | hard_cap 별칭. `q1=` `eta+=` 도 됨 |
| `./tune.sh status` | ★ 현재 튜닝 상태 (아래 참조) |
| `./tune.sh` | 설정값 전체 보기 (권장 범위 포함) |
| `./tune.sh stanley` | 한 섹션만 보기 |
| `./tune.sh stanley.gain` | 한 값만 출력 (스크립트에서 쓰기 좋음) |

### 바꾸기

| 명령 | 설명 |
|---|---|
| `./tune.sh stanley.gain=1.2` | 설정 |
| `./tune.sh stanley.gain=1.2 stanley.k=2.0` | 여러 개 한 번에 |
| `./tune.sh stanley.ff_gain+=0.3` | 상대 조정 (`+=` / `-=`) |
| `./tune.sh --undo` | 직전 `tune.sh` 실행 되돌리기 |
| `./tune.sh --force ...` | 안전 가드 무시하고 강제 적용 |

### 이력

| 명령 | 설명 |
|---|---|
| `./tune.sh --log` | 변경 이력 (시각 + 값 변화) |
| `./tune.sh --diff` | 마지막 커밋 대비 `git diff` |

---

## `./tune.sh status` 읽는 법

```
  튜닝 상태  07/21 16:19
  mode q2   controller stanley   map track_2nd_clean

  Stanley 조향 게인   (docs/stanley_gain_tuning.md)
   1 * ff_gain          2.8  (2.5 → 2.8)     코너에서 바깥으로 밀리면 ↑ (가장 먼저 조절)
   2   k                1.8                  밀림이 남으면 ↑ — 라인 복귀 속도
   3   gain             1.5                  1·2 이후에도 부족하면 소폭만 ↑
   4   ff_preview_sec   0.38                 너무 일찍/늦게 꺾는 타이밍 문제일 때만
   5   ks               1.5                  저속 떨림이 있을 때만
   5   steer_lpf        1.0                  고주파 떨림이 있을 때만 (1.0 = 필터 없음)

  그 밖의 변경
     * speed.q2_max               1.5 → 1.8

  * = 마지막 커밋 대비 변경됨 (2개). 되돌리려면  git checkout race_script.yaml
```

- **맨 왼쪽 숫자** = 조절 우선순위. 출처는 `docs/stanley_gain_tuning.md` 의 "조절 우선순위" 절.
  위에서부터 만지는 게 원칙이다.
- **`*`** = 마지막 커밋 이후 바뀐 값. 옆의 `(2.5 → 2.8)` 이 변화량.
  "내가 지금 뭘 건드려 놓은 상태인지" 를 한눈에 보기 위한 것.
- **그 밖의 변경** = Stanley 게인 6종 외에 바뀐 값.
  속도를 올려놓고 잊은 채 게인 탓을 하는 걸 막는다.
- **`controller` 가 `stanley` 가 아니면 경고가 뜬다.**
  `gap_follower` 상태에서 Stanley 게인을 아무리 만져도 주행에 반영되지 않는다.

---

## 안전 가드

실차가 걸린 파일이라 아래 경우 **거부하고 원본을 유지**한다.
의도한 값이면 `--force` 를 붙인다.

| 가드 | 이유 |
|---|---|
| `stanley.ff_gain=0` | 곡률 피드포워드가 꺼져 코너를 늦게 꺾는다. yaml 주석에도 "절대 0으로 두지 말 것" |
| 권장 범위 이탈 | `./tune.sh` 목록의 `[0.1~5.0]` 표기가 그 범위 |
| 한 번에 5배 이상 변경 | `1.5` 를 `15` 로 치는 오타 방지 |
| 타입 불일치 | 숫자 자리에 문자열 등 |

추가로 **쓰기 후 다시 파싱해서 검증**한다. 값이 의도대로 안 들어갔거나 YAML 이
깨졌으면 원본을 그대로 둔다.

```
$ ./tune.sh stanley.gain=15
[tune] stanley.gain: 권장 범위 0.1~5.0 밖의 값: 15
[tune] stanley.gain: 한 번에 10.0배 변경 (1.5 → 15) — 오타일 수 있다
[tune] 중단했다. 의도한 값이면 --force 를 붙일 것.
```

키 이름을 틀리면 비슷한 키를 제안한다.

```
$ ./tune.sh stanley.gian=1.0
[tune] 그런 키가 없다: stanley.gian
       혹시 이건가: stanley.gain, stanley.ff_gain, stanley.k, stanley.ks, stanley.interp_scale
```

---

## 주의사항

- **`--undo` 는 직전 1회만** 되돌린다. 그 이상은 `git checkout race_script.yaml` 로
  마지막 커밋 상태로 되돌릴 것.
- **좋은 값을 찾으면 커밋할 것.** `tune.sh` 는 파일을 고칠 뿐 커밋하지 않는다.
  ```bash
  git add race_script.yaml && git commit -m "tune: ff_gain 2.5→2.8 (코너 밀림 개선)"
  ```
- `.tune_history` / `.tune_backup.yaml` 은 로컬 전용이라 `.gitignore` 에 있다.
  튜닝 이력은 젯슨에만 남으므로, 팀과 공유할 결론은 커밋 메시지나 Notion에 적을 것.
- **주석은 보존된다.** `tune.sh` 는 대상 한 줄만 고치고 주석·정렬·빈줄은 손대지 않는다.
  그래서 `race_script.yaml` 을 팀 문서로 계속 써도 된다.

---

## 조절 대상 게인의 의미

각 게인이 무엇을 하는지, 어떤 증상에 무엇을 만져야 하는지는
**`docs/stanley_gain_tuning.md`** 에 정리돼 있다. `tune.sh status` 의 우선순위도 그 문서에서 온다.

두 문서는 같이 고쳐야 한다 — 우선순위를 바꾸면 `tune.py` 의 `TUNING_GUIDE` 도 수정할 것.
