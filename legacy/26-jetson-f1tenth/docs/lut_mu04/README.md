# ⚠ 이 폴더의 LUT 는 대체되었다 — `docs/lut_grip040/` 를 쓸 것

`lut_mu0.4.csv` 는 `mu = 0.40` 으로 생성했지만, **실효 최대 그립은 0.475g** 다.
파일 이름의 "0.4" 가 그립을 뜻하지 않는다.

## 왜 그런가

MAP 논문(2209.04346) III-B 의 파라미터화에서 횡력은

```
F_y = mu · F_z · D · sin(C·atan(B·α − E·(B·α − atan(B·α))))
```

이고, 논문이 명시하듯 **"the peak value D̃ is in fact μ·F_z,i · D_i"** 다.
즉 차가 실제로 낼 수 있는 최대 횡가속은 `mu` 가 아니라 **`mu × D`** 다.

```
mu 0.40 × D 1.2084 = 0.483        ← 이 파일. 실측 LUT 최대 0.475g
논문값 1.0489 × 1.2084 = 1.267    ← 논문 본문 "lateral accelerations of 1.25 g" 와 일치
```

우리가 실측한 0.40 은 **차가 미끄러질 때의 a_lat/g**, 즉 `mu × D` 쪽이다.
그래서 `mu` 에 그대로 넣으면 20% 낙관적인 모델이 된다.

## 대체본

`docs/lut_grip040/lut_grip0.40.csv` — `mu 0.348 × D 1.15 = 0.400g`.
실측 LUT 최대 0.398g 로 목표와 일치한다.

같이 고친 것: 컨트롤러(yaml `map:` 경유)와 LUT 생성기(`vehicle_sim.py`)가
서로 다른 타이어 곡선을 쓰고 있었다 — 컨트롤러 B=3.8/D=1.15, 생성기
B=3.629/D=1.2084. 이제 `race_script.yaml [map]` 을 단일 소스로 통일했다.

## 헷갈리지 않으려면

`mu` 라는 이름이 세 군데서 서로 다른 뜻으로 쓰인다:

| 위치 | 규약 | 값 |
|---|---|---|
| `race_script.yaml [map] mu` · `map_controller` · `vehicle_sim` | Pacejka — 실효 그립 = `mu × pac_d` | **0.348** |
| `gen_raceline --mu` | `a_lat_max = mu · g` | **0.40** |
| `race_script.yaml sim_vehicle.mu` (f1tenth_gym) | 위와 같은 규약 | **0.40** |

세 값이 달라 보이는 것이 정상이다. 바꿀 때는 **실효 그립 0.40g** 를 기준으로
환산할 것.
