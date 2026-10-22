#!/usr/bin/env bash
set -euo pipefail
python3 - "${1:?설치 시 출력된 Backup 폴더 경로를 입력하세요}" <<'PY'
from pathlib import Path
import ast,shutil,sys
backup=Path(sys.argv[1]);ws=Path('/opt/AutoVehicle/morai_ws')
dest={
 'frenet_fsm_node.py':ws/'src/control/purepursuit_mgeo/scripts/frenet_fsm_node.py',
 'frenet_planner.py':ws/'src/control/purepursuit_mgeo/src/purepursuit_mgeo/frenet_planner.py',
 'lidar_tracked_obstacle_state_node.py':ws/'src/detection/lidar_perception/scripts/lidar_tracked_obstacle_state_node.py',
 'morai_udp_ekf_frenet_curvature_test.launch':ws/'src/bringup/morai_bringup/launch/morai_udp_ekf_frenet_curvature_test.launch'}
for name,path in dest.items():
 source=backup/name
 if not path.is_file() or not source.is_file():raise SystemExit('Missing backup or destination: '+name)
 if name.endswith('.py'):ast.parse(source.read_text(),filename=name)
for name,path in dest.items():shutil.copy2(backup/name,path)
print('Restored four files from:',backup)
print('Restart roslaunch.')
PY
