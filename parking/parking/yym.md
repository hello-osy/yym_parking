# YYM 주차 노드 인수인계 문서

최종 갱신일: 2026-08-08
대상 파일: `parking_node_yym.py`
절대 경로: `/home/hailab/osy/260711/ai-autonomous-driving-competition-2026/parking/parking/parking_node_yym.py`

## 현재 기준점

이 문서는 Git 커밋 `df607c6`의 현재 파일을 기준으로 갱신했다. 대화 시작 당시 원본은 커밋 `5b91025`였으며, 현재는 최초 차량 인식 완화, 첫 후진 시작 시 원거리 pair 획득, 5초 정지 전 차량 기울기 보정, 주차 후 출차 활성화까지 반영되어 있다.

작성 시점 파일 SHA-256:

```text
d6e378925171b3e752405ef0c153589bcbcf503b4c6219bac6dcdce072eaa868
```

다른 세션에서 작업을 이어갈 때 먼저 실제 파일의 해시와 Git 변경사항을 확인해야 한다. 이후 수정이 있었다면 이 문서보다 실제 코드를 우선한다.

## 작업 범위 원칙

- 사용자는 주차 작업에서 별도 허가가 없으면 `parking_node_yym.py`만 수정하기를 원한다.
- 이 노드는 `steering_raw`나 Arduino 조향 토픽을 직접 조작하지 않는다.
- 모든 주행 요청은 `/motor_control` 토픽으로만 발행한다.
- `/motor_control` 메시지는 `std_msgs/msg/Int16MultiArray`이며 형식은 `[steer_deg, speed]`이다.
- 조향 명령은 `-45~+45`, 속도 명령은 코드에서 `-130~+130`으로 제한된다.
- 조향 부호는 `음수=좌`, `양수=우`다.
- 속도 부호는 `양수=전진`, `음수=후진`이다.

## ROS 입출력

| 방향 | 토픽 | 타입 | 용도 |
|---|---|---|---|
| 구독 | `/scan` | `sensor_msgs/msg/LaserScan` | 차량 감지, 중앙점/기울기 계산, 주차 종료 판단 |
| 발행 | `/motor_control` | `std_msgs/msg/Int16MultiArray` | `[조향각, 속도]` 명령 |

노드 이름은 `parking_node_yym`, 디버그 창 이름은 기본적으로 `parking_yym_debug`다.

## LiDAR 좌표계

장착 방향 기준 각도:

```text
REAR 0° | RIGHT +90° | FRONT ±180° | LEFT -90°
```

코드 내부 차량 좌표:

- `+x`: 차량 전방
- `-x`: 차량 후방
- `+y`: 차량 좌측
- `-y`: 차량 우측
- 초록 가로선: 후미 LiDAR를 지나는 `x=0` 선
- 초록 세로선: 차량 중심의 전후 기준선 `y=0`
- 빨간선: LiDAR 원점과 두 장애물 차량의 기준점을 잇는 선

`ParkingPair.lower`는 화면/차량 기준 우측 장애물이고, `ParkingPair.upper`는 좌측 장애물이다.

## 현재 전체 주차 시퀀스

### 1. 스캔 대기와 출발 대기

1. `WAIT_FOR_SCAN`에서 유효한 `/scan`을 기다린다.
2. 스캔이 오기 전에는 정지 상태의 바퀴가 움직이지 않도록 조향 명령도 보내지 않는다.
3. 첫 유효 스캔 이후 `START_DELAY`에서 기본 5초 동안 `steer/speed=0/0`으로 정지한다.

### 2. 첫 차량 인식

1. `APPROACH_FIRST_CAR`에서 `steer=0`, `speed=110`으로 직진한다.
2. 인식 모드에서는 후방 노이즈를 제거하기 위해 `x>=0.05m`인 전방 점만 클러스터링한다.
3. 첫 차량 후보는 다음 조건을 모두 만족해야 한다.
   - 우측 차량 후보
   - 클러스터 중심 `x>=0.10m`
   - 최근접 거리 2.0m 이하
   - 점 7개 이상
   - x/y 최대 크기 0.22m 이상
   - 2프레임 연속 검출
4. 이 필터는 첫 차량을 찾는 인식 모드에만 적용된다.

### 3. 시간 기반 최대 좌회전

1. 첫 차량이 확정되면 `SET_LEFT_STEER`에서 정지한다.
2. 최대 좌조향 `-45°`를 0.6초 동안 맞춘다.
3. `TURN_LEFT_TIMED`에서 `steer=-45`, `speed=110`으로 7초 주행한다.
4. `recognition_only=True`면 여기서 정지 후 종료한다.
5. 기본값은 `False`이므로 주차 모드로 넘어간다.

### 4. 두 차량과 주차 공간 획득

1. `SETTLE_AND_ACQUIRE_GAP`에서 정지한다.
2. 엄격한 간격 조건을 만족하는 두 차량 pair를 우선 선택한다.
3. 엄격한 pair가 없더라도 두 클러스터가 명확하면 fallback midpoint pair를 사용할 수 있다.
4. pair가 3프레임 연속 유지되고 조향 정렬 시간이 지나면 `REVERSE_CENTER`로 전환한다.
5. 4초 안에 pair를 획득하지 못하면 `PARKING_FAILED`가 된다.
6. 이 최초 pair 획득 상태에서만 차량 클러스터 최대 거리를 6.0m로 늘린다.
7. 후진이 시작되면 클러스터 최대 거리는 기존 4.0m로 즉시 돌아간다.

### 5. 1m 원 진입 전 후진

다음 동작을 반복한다.

1. 정지 상태에서 두 차량 기준점의 중간과 초록 세로선 사이 빨간선 각도를 계산한다.
2. 빨간선 각도에 `reverse_steer_multiplier=10`을 곱한다.
3. 양쪽 차량의 gap-facing 세로 경계 기울기를 구하고 `pre_final_alignment_steer_multiplier=5`를 곱한다.
4. 두 값을 더한 뒤 결과를 `±45°`로 제한한다.
5. 정지 상태에서 계산 조향각을 0.6초 맞춘다.
6. 같은 조향으로 1초 동안 `speed=-110` 후진한다.
7. 현재 조향을 유지한 상태로 0.4초 정지한다.
8. LiDAR pair와 조향각을 다시 계산한다.

사용자의 명시적 요청으로 이 구간에도 차량 기울기 `×5`가 추가되었다. 이후에는 이 결합 조향을 기준으로 보존하고, 별도 요청 없이 배율이나 시퀀스를 바꾸지 않는다.

### 6. 1m 원 진입과 5초 정지

- 두 장애물 차량 중 하나라도 LiDAR 점이 1.0m 원 안에 들어오면 조건을 latch한다.
- 즉시 `FINAL_STOP`으로 전환해 `steer/speed=0/0`으로 5초 정지한다.
- latch 이후에는 원래 선택했던 좌우 차량을 더 엄격한 거리로 추적해 뒤쪽 기둥이나 다른 유닛으로 바뀌는 것을 막는다.

### 7. 5초 정지 후 정밀 보정

최종 보정은 **0.5초씩 총 2회**다. 두 번 모두 독립적으로 정지·재측정한다.

각 회차의 판단 순서:

1. 0.4초 정지 상태에서 최신 pair를 측정한다.
2. 빨간선과 초록 세로선의 각도 오차를 계산한다.
3. 오차가 `±3°`를 벗어나면 빨간선 각도 보정을 최우선으로 적용한다.
4. 빨간선이 `±3°` 이내일 때만 양쪽 장애물 차량의 세로 경계 기울기를 사용한다.
5. 계산 조향을 정지 상태에서 0.6초 맞춘다.
6. 같은 조향으로 0.5초 후진한다.
7. 첫 회차가 끝나면 반드시 정지하고 다시 계산한다.

두 번째 0.5초 보정이 끝나면:

1. 반드시 정지한다.
2. `steer=0`을 명령한다.
3. 0.6초 동안 조향 중앙 정렬을 기다린다.
4. 그 뒤 `steer=0`, `speed=-110`으로 정지 조건까지 연속 후진한다.

rosbag 검증에서 연속 후진 구간은 실제로 `steer=0`이었고 조향 센서도 중앙값 부근으로 복귀했다. 주차가 삐뚤어진 경우는 직선 후진 중 새로 틀어진 것보다는 최종 보정 후 남은 차체 각도가 유지된 경우로 판단됐다.

### 8. 박스 세로 경계 기반 기울기 계산

장애물 차량을 박스로 대체했기 때문에 LiDAR에서는 박스가 L자 형태로 보일 수 있다. 클러스터 전체 PCA를 사용하면 짧은 가로면이 `80~90°`의 차량 방향으로 잘못 선택되는 문제가 있었다.

현재 구현은 `final_vehicle_gap_edge_angle()`에서 다음 방식으로 이를 방지한다.

1. 우측 차량은 주차 공간을 향한 위쪽 y 경계를 사용한다.
2. 좌측 차량은 주차 공간을 향한 아래쪽 y 경계를 사용한다.
3. 차량의 x 범위를 기본 7개 구간으로 나눈다.
4. 각 구간의 gap-facing 대표점을 추출한다.
5. 대표점에 직선을 적합해 세로 경계 기울기를 구한다.
6. x 방향 길이가 0.20m보다 짧은 가로면은 제외한다.
7. 절대 각도가 45°보다 큰 결과도 제외한다.
8. 양쪽에서 유효한 결과가 나오면 x 길이 가중 평균을 사용한다.

기울기 보정 배율은 `final_reverse_steer_multiplier=5`다.

### 9. 주차 완료와 출차

- 5초 정지 이후부터 처음 선택했던 좌우 차량을 계속 추적한다.
- 각 기준 차량에 대해 초록 가로선 아래, 즉 `x<0`인 점의 존재 여부를 확인한다.
- 좌측 또는 우측 기준 차량 중 **하나라도** 초록 가로선 아래에서 3프레임 연속 사라지면 `PARKED`가 된다.
- `PARKED`가 되면 `steer/speed=0/0`으로 2초 정지한다.
- 이후 출차 모드로 전환한다.
- `steer=0`, `speed=110`으로 3초 전진한다.
- 정지 상태에서 우조향 `+45°`를 0.6초 맞춘다.
- `steer=+45`, `speed=110`으로 10초 우회전 전진한다.
- 정지 상태에서 `steer=0`을 0.6초 맞춘다.
- `steer=0`, `speed=110`으로 10초 전진한다.
- 마지막에 `EXIT_COMPLETE`로 전환해 `steer/speed=0/0`을 계속 발행한다.

## LiDAR 클러스터와 pair 생성

기본 필터:

- 사용 각도 영역: 후방 0° 기준 `±125°`
- 최소 거리: 0.15m
- 최대 거리: 기본 4.0m
- 첫 후진 시작 전 `SETTLE_AND_ACQUIRE_GAP`에서만 최대 6.0m
- connected-component 점 간 거리: 0.20m
- 최소 점 개수: 7
- 최소 장애물 크기: 0.22m
- 우측 차량 중심 조건: `y<-0.12m`
- 1m latch 전 주차 차량 점 조건: `x<=0`

pair 처리:

- y가 작은 차량을 `lower/right`, 큰 차량을 `upper/left`로 정렬한다.
- 허용 gap 기본 범위는 0.48~1.40m다.
- 빨간선 기준점은 두 클러스터 median center의 평균이다.
- strict pair가 없으면 중심선에 가깝고 두 차량이 중심을 감싸는 fallback pair를 선택한다.
- 후진 중에는 이전 좌우 track center와의 이동량으로 원래 차량을 유지한다.

## 안전 및 실패 조건

- `/scan`이 0.5초 이상 끊기면 `EMERGENCY_STOP`.
- 유효 점 부족 스캔이 5프레임 누적되면 `EMERGENCY_STOP`.
- 첫 차량을 30초 안에 찾지 못하면 `PARKING_FAILED`.
- 주차 pair를 4초 안에 얻지 못하면 `PARKING_FAILED`.
- 1m latch 전 후방 `±11°`에서 0.18m 이하 물체가 감지되면 실패 처리한다.
- `PARKING_FAILED`, `EMERGENCY_STOP`, `EXIT_COMPLETE`는 `steer/speed=0/0`을 계속 발행하는 terminal hold 상태다.
- `PARKED`는 2초 정지 후 출차로 넘어가는 중간 상태다.

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `startup_delay_sec` | 5.0 | 스캔 수신 후 출발 대기 |
| `approach_speed` | 110 | 첫 차량까지 직진 속도 |
| `turn_speed` | 110 | 최대 좌회전 속도 |
| `reverse_speed` | -110 | 후진 속도 |
| `left_max_steer_deg` | -45 | 최대 좌조향 |
| `left_turn_duration_sec` | 7.0 | 시간 기반 좌회전 시간 |
| `first_car_max_distance_m` | 2.0 | 첫 차량 최대 감지 거리 |
| `first_car_confirm_frames` | 2 | 첫 차량 연속 확인 프레임 |
| `recognition_vehicle_min_x_m` | 0.05 | 인식용 전방 점 최소 x |
| `first_car_min_center_x_m` | 0.10 | 첫 차량 중심 최소 x |
| `first_car_min_points` | 7 | 첫 차량 최소 점 개수 |
| `first_car_min_extent_m` | 0.22 | 첫 차량 최소 크기 |
| `cluster_max_range_m` | 4.0 | 일반 차량 클러스터 최대 거리 |
| `initial_gap_cluster_max_range_m` | 6.0 | 첫 후진 시작 전 pair 획득 최대 거리 |
| `reverse_segment_duration_sec` | 1.0 | 1m 전 보정 후진 시간 |
| `reverse_measure_stop_sec` | 0.4 | 재계산 전 정지 시간 |
| `steer_settle_sec` | 0.6 | 정지 조향 정렬 시간 |
| `reverse_steer_multiplier` | 10.0 | 빨간선 보정 배율 |
| `pre_final_alignment_steer_multiplier` | 5.0 | 5초 정지 전 차량 세로 경계 보정 배율 |
| `straight_reverse_radius_m` | 1.0 | 최종 단계 진입 반경 |
| `straight_reverse_stop_sec` | 5.0 | 최종 단계 전 정지 시간 |
| `final_line_alignment_tolerance_deg` | 3.0 | 빨간선 우선 보정 허용 범위 |
| `final_reverse_steer_multiplier` | 5.0 | 차량 세로 경계 보정 배율 |
| `final_edge_min_x_span_m` | 0.20 | 유효 세로 경계 최소 길이 |
| `final_correction_duration_sec` | 0.5 | 최종 보정 1회 주행 시간 |
| `final_correction_segment_count` | 2 | 최종 보정 횟수 |
| `rear_half_empty_confirm_frames` | 3 | 주차 완료 확인 프레임 |
| `vehicle_width_m` | 0.38 | 차량 폭 |
| `minimum_side_clearance_m` | 0.05 | 최소 측면 여유 |
| `exit_wait_after_park_sec` | 2.0 | 주차 완료 후 출차 전 정지 |
| `exit_forward_duration_sec` | 3.0 | 출차 첫 직진 시간 |
| `exit_right_turn_duration_sec` | 10.0 | 최대 우조향 전진 시간 |
| `exit_final_forward_duration_sec` | 10.0 | 출차 마지막 직진 시간 |
| `exit_right_steer_deg` | 45 | 출차 우조향각 |
| `exit_speed` | 110 | 출차 전진 속도 |

## 디버그 화면

- 노란 상태 글자: 진행 중
- 초록 상태 글자: `PARKED` 또는 `EXIT_COMPLETE`
- 빨간 `FAILED`: 실패 또는 비상 정지
- 회색 점: LiDAR 점
- 색깔 점 묶음: 차량 클러스터
- `V1`, `V2` 각도: 원본 PCA 축 각도이며 최종 세로 경계 적합 각도와 다를 수 있음
- 빨간선: pair 기준점 방향
- 초록 세로선: 차량 중심선
- 상단 `cmd steer/speed`: 마지막 `/motor_control` 명령
- `phase`: 현재 세부 후진 단계
- `pair=STRICT/FALLBACK`: pair 선택 방식

## 하이카메라 관련 현재 상태

- 현재 실제 주차 제어는 LiDAR만 사용한다.
- 생성되는 ROS 구독은 `/scan` 하나뿐이며 하이카메라 구독은 생성하지 않는다.
- 따라서 하이카메라 콜백, 흰 실선 검출, 교차점 계산, 카메라 조향은 실행되지 않는다.
- `camera_debug_image`도 채워지지 않으므로 하이카메라 창은 뜨지 않는다.
- 다만 이전 실험에서 추가했던 `CvBridge`, `Image`, 카메라 파라미터, 실선 검출 함수와 디버그 표시 코드가 파일에 죽은 코드로 남아 있다.
- 이 잔여 코드는 주행 명령에는 영향을 주지 않지만 `cv_bridge` import/초기화 의존성은 만든다.
- 다음 세션에서 사용자가 정리를 요청하면 카메라 잔여 코드만 제거하되 LiDAR 주차 시퀀스는 변경하지 않는다.

## 실차 rosbag 분석 기록

분석했던 기존 bag:

```text
/home/hailab/osy/260711/ai-autonomous-driving-competition-2026/parking_test_02
/home/hailab/osy/260711/ai-autonomous-driving-competition-2026/parking_test_03
/home/hailab/osy/260711/ai-autonomous-driving-competition-2026/parking_test_04
/home/hailab/osy/260711/ai-autonomous-driving-competition-2026/parking_test_05
```

사용자 평가:

- 2번, 5번: 잘 된 케이스
- 3번, 4번: 삐뚤어진 케이스

확인 결과:

- `/motor_control`과 실제 `/arduino/steering_raw` 응답은 정상적으로 대응했다.
- 최종 직선 후진은 네 bag 모두 `steer=0, speed=-110`이었다.
- 박스 L자 클러스터의 전체 PCA가 짧은 가로면을 차량 방향으로 오인해 큰 최종 보정을 만드는 문제가 있었다.
- 이 분석을 바탕으로 현재 세로 경계 기반 최종 기울기 계산을 추가했다.

## 빌드와 실행

코드 또는 파라미터를 수정한 후:

```bash
cd /home/hailab/osy/260711/ai-autonomous-driving-competition-2026
source /opt/ros/humble/setup.bash
colcon build --packages-select parking --symlink-install
source install/setup.bash
```

센서 실행 터미널:

```bash
cd /home/hailab/osy/260711/ai-autonomous-driving-competition-2026
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch sensor_topic sensors.launch.py
```

주차 실행 터미널:

```bash
cd /home/hailab/osy/260711/ai-autonomous-driving-competition-2026
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch parking parking.launch.py
```

## rosbag 기록

```bash
cd /home/hailab/osy/260711/ai-autonomous-driving-competition-2026
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag record -o parking_test_06 \
  /scan \
  /motor_control \
  /arduino/steering_raw
```

테스트 종료 후 rosbag 터미널에서 `Ctrl+C`를 눌러야 `metadata.yaml`이 정상 저장된다.

## 다음 세션 시작 체크리스트

1. `parking/parking/yym.md`와 실제 `parking_node_yym.py`를 함께 읽는다.
2. `git status --short`로 기존 사용자 변경사항을 확인하고 보존한다.
3. 별도 허가가 없다면 `parking_node_yym.py` 외 파일을 수정하지 않는다.
4. 5초 정지 전 조향은 현재 `중점 각도×10 + gap-facing 경계 기울기×5`다. 별도 요청 없이 변경하지 않는다.
5. 5초 이후 수정 시 빨간선 보정이 차량 기울기보다 우선이라는 요구를 유지한다.
6. 최종 보정은 현재 0.5초씩 2회이며 각 회차 사이 반드시 정지·재측정한다.
7. 두 번째 보정 후 반드시 정지하고 조향 0 정렬 뒤 연속 후진한다.
8. 변경 후 `python3 -m py_compile parking/parking/parking_node_yym.py`를 실행한다.
9. `git diff --check -- parking/parking/parking_node_yym.py`로 형식을 검사한다.
10. `colcon build --packages-select parking --symlink-install`로 빌드한다.
11. 실차 테스트 시 사진과 `/scan`, `/motor_control`, `/arduino/steering_raw` rosbag을 같은 번호로 남긴다.
12. 사용자는 코드 수정 답변마다 빌드/실행 명령어도 함께 받기를 원한다.
13. 현재 주차 완료 후 출차가 자동 활성화되어 있으므로 실차 주변 안전 공간을 먼저 확인한다.
