# 시뮬 도구 (2026-11-15, 젯슨에서 사용)

실제 차 없이 젯슨 안에서 가짜 차(fake_car.py)로 위치 재설정과 AMCL을 시험하는 도구예요.
모두 `ROS_DOMAIN_ID=87`, `ROS_LOCALHOST_ONLY=1`로 격리해서 돌아가요. 이 값이 아니면 실행을 거부해요(실제 차에 명령이 가지 않게).
결과와 로그는 `~/.cache/magp_sim/`에 쌓여요.

- `fake_car.py <map.yaml> <x> <y> <yaw> <초>`: 자전거 모델 차, 맵에서 만든 노이즈 섞인 스캔, TF, /initialpose
- `reloc_sim.sh <cpu|cuda> <맵 이름> [경로 인덱스] [초]`: pure_pursuit을 띄우고 12초쯤 위치를 1.5 m / 20° 밀어서, 다시 찾는지 확인
- `amcl_sim.sh <맵 이름> <경로 인덱스> [초]`: 진짜 AMCL(light)과 pure_pursuit으로, 12초쯤 AMCL을 2.5 m 밀어서 확인
- `gpu_check.py <map.yaml> <wp.csv> [N]`: 위치 탐색의 CPU와 GPU 결과가 같은지, 얼마나 빠른지 비교

예: `bash tools/sim/reloc_sim.sh cuda left1043 190 40`
