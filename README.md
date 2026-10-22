# Tunnel clearance V1

기준: frenet_bbox_bestof_fix.zip. 터널에서 지도 경로보다 장애물과의 여유 거리를 우선하고, 검출 누락과 재계획의 횡이동 재시작을 줄이는 시험 패치.

## 터널 판정 및 적용 범위

변환 GPS 전용 /localization/gps(nav_msgs/Odometry)의 유효한 새 stamp 갱신이 1초간 없으면 GPS 음영 모드에 들어간다. 변환기가 raw 위경도 (0,0)을 무시하는 현재 구성 기준이다. EKF 예측 /localization/odometry는 GPS 판정에 사용하지 않는다. GPS가 1초간 연속으로 정상 갱신되면 외부 비용으로 복귀한다. GPS 음영은 지리적 터널의 대리 조건이며 실제 터널 판별은 아니다.

이전 GPS_TUNNEL_PROFILE의 1Hz 및 3/4/5초 후보 변경은 포함하지 않는다. 설치된 launch의 planning_rate_hz와 planning_times_s를 보존한다. 사용자의 최근 로그는 2Hz, [2,3,4]초였다. jerk=30, time=0.05는 bestof 기준이며 설치된 jerk 값은 설치기가 보존한다.

## 변경

1. 음영에서 OFFSET 비용을 100 -> 5, 평균 안전거리 비용을 4 -> 40으로 바꾼다. 최소 표면 여유 거리 1m 미만에는 `100 * max(0,(1-clearance)/1)^2` 비용을 추가한다. 1m는 선호값이며 강제 통과 제한이 아니다. 모든 후보에 기존 차량 크기/벽/장애물 OBB 충돌 검사를 적용한다. 큰 OFFSET도 벽과 겹치면 탈락한다. OFFSET 목록은 기존 34개(최대 ±6m).

2. GPS 음영에서 이전에 경로를 막았던 검출만 누락 후 최대 1.5초 유지한다. 현재 검출되는 벽과 장애물은 계속 모두 검사한다. 경로를 막지 않았던 벽 파편까지 전부 장기 보관하지 않는다. 빈 프레임은 유지 시간을 연장하지 않는다. 같은 ID 재검출은 최신 형상으로 교체한다. 다른 ID로 재검출되면 이전 ID와 짧게 중복되어 보수적인 형상이 남을 수 있다. 이미 지도 변환된 좌표를 보관하므로 위치 추정 오류 자체는 보정하지 못한다.

3. 경로 교체가 필요한 경우, 실제 차량의 횡속도와 기존 경로의 현재 지점 횡가속도를 초기값으로 사용한다. 기존 경로와 너무 멀리 떨어졌으면 가속도를 이어 쓰지 않는다. 기존 횡이동 완료 지점을 늦추는 시간의 제곱에 가중치 10의 비용을 줘서 매번 새 4초 경로를 시작하는 것을 줄인다. 완료 시점은 소프트 비용이고 안전 검사에 통과한 긴 경로를 배제하지 않는다. 반대 방향으로 전환할 때는 기존 완료 시점 비용을 적용하지 않는다. 위치가 급변해 경로 접속을 잃거나 경로 끝에 도달하면 이전 완료 지점을 초기화한다.

4. 모드 진입 시 새 비용으로 한 번 재검토한다. 이후 유효한 경로는 유지한다. 교체가 필요할 때는 같은 쪽 전체 OFFSET을 비교하고, 모두 실패하면 반대쪽도 검사한다. 경로 실패만으로 정지하는 로직은 추가하지 않는다. 모든 후보가 실패하면 bestof의 지도 경로 fallback이 남아 있으므로 충돌 회피를 보장하는 패치는 아니다.

## 설치: Ubuntu 호스트

기존 roslaunch 종료 후:

```bash
sudo docker --context default start morai-highway-gpu
sudo docker --context default cp /home/msclab/Downloads/frenet_tunnel_clearance_fix.zip morai-highway-gpu:/tmp/frenet_tunnel_clearance_fix.zip
sudo docker --context default exec -it morai-highway-gpu bash
```

## 설치 및 실행: 컨테이너

```bash
python3 -m zipfile -e /tmp/frenet_tunnel_clearance_fix.zip /tmp
bash /tmp/frenet_tunnel_clearance_fix/apply_fix.sh
source /opt/ros/noetic/setup.bash
source /opt/AutoVehicle/morai_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_IP=192.168.10.21
export ROS_MASTER_URI=http://192.168.10.21:11311
mkdir -p /opt/AutoVehicle/morai_ws/debug_logs
PYTHONUNBUFFERED=1 roslaunch --screen morai_bringup morai_udp_ekf_frenet_curvature_test.launch enable_control:=true morai_host_ip:=192.168.10.14 max_speed_kph:=20.0 2>&1 | tee /opt/AutoVehicle/morai_ws/debug_logs/frenet_tunnel_clearance.log
```

시작 버전: TUNNEL_CLEARANCE_V1
전환: Frenet tunnel clearance: active=True/False ...
검출 유지: Frenet tunnel held threats (id,age_s) ...
계획 로그에 tunnel_mode, held_obstacles, initial_lateral_acceleration, completion_budget_s 및 실제 비용 계수를 기록한다.

## 로그: 새 Ubuntu 호스트 터미널

```bash
sudo docker --context default exec morai-highway-gpu tail -n 100 -f /opt/AutoVehicle/morai_ws/debug_logs/frenet_tunnel_clearance.log
sudo docker --context default cp morai-highway-gpu:/opt/AutoVehicle/morai_ws/debug_logs/frenet_tunnel_clearance.log /home/msclab/Downloads/frenet_tunnel_clearance.log
```

설치기는 Python 3개와 launch의 OFFSET 항목만 변경한다. 네 파일을 frenet_fix_backups/tunnel_clearance_설치시각에 백업한다. 복구는 roslaunch 종료 후 `bash /tmp/frenet_tunnel_clearance_fix/restore_fix.sh 설치시출력된백업경로`. catkin_make 불필요.

## 검증

python3 test_tunnel_clearance.py: 실제 후보의 여유 거리 증가, 벽 충돌 거부, 검출 누락 유지/만료 및 형상 갱신, 횡가속도 연결, 완료 시점 비용, 기존 동역학/충돌 검사, GPS 모드/외부 비용 복귀와 주기 보존을 검사한다. 10개 테스트 통과. Python3.8 문법 및 설치/복구 검사 완료. 실제 MORAI 주행은 미검증.

## 기본 private params

- gps_health_topic=/localization/gps
- tunnel_gps_timeout_s=1.0, tunnel_gps_recovery_s=1.0
- tunnel_weight_offset=5.0, tunnel_weight_safety=40.0
- tunnel_weight_clearance=100.0, tunnel_preferred_clearance_m=1.0
- tunnel_weight_completion=10.0
- tunnel_obstacle_hold_s=1.5
