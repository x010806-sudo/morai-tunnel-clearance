#!/usr/bin/env bash
set -euo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
python3 - "$HERE" <<'PY'
import ast,re,shutil,sys
from pathlib import Path
from datetime import datetime

here=Path(sys.argv[1]); ws=Path('/opt/AutoVehicle/morai_ws')
launch_name='morai_udp_ekf_frenet_curvature_test.launch'
dest={
 'frenet_fsm_node.py':ws/'src/control/purepursuit_mgeo/scripts/frenet_fsm_node.py',
 'frenet_planner.py':ws/'src/control/purepursuit_mgeo/src/purepursuit_mgeo/frenet_planner.py',
 'lidar_tracked_obstacle_state_node.py':ws/'src/detection/lidar_perception/scripts/lidar_tracked_obstacle_state_node.py',
 launch_name:ws/'src/bringup/morai_bringup/launch'/launch_name,
}
for name,path in dest.items():
 source=here/'launch'/name if name.endswith('.launch') else here/name
 if not path.is_file() or not source.is_file(): raise SystemExit('Missing file: '+name)

jerk_source=dest['frenet_planner.py'].read_text()
m=re.search(r'(?m)^\s*self\.weight_jerk\s*=\s*([0-9]+(?:\.[0-9]+)?)[ \t]*$',jerk_source)
if m is None: raise SystemExit('Cannot determine installed weight_jerk; installation stopped')

texts={name:(here/'launch'/name if name.endswith('.launch') else here/name).read_text()
       for name in dest}
texts['frenet_planner.py'],count=re.subn(
 r'(?m)^(\s*self\.weight_jerk\s*=\s*)30\.0[ \t]*$',
 lambda v:v[1]+m[1],texts['frenet_planner.py'])
if count!=1: raise SystemExit('Unexpected planner weight source')
for name,text in texts.items():
 if name.endswith('.py'): ast.parse(text,filename=name)

# Preserve the installed launch file and replace only its OFFSET parameter.
launch_path=dest[launch_name]
installed_launch=launch_path.read_text()
template=re.search(r'<rosparam\s+param="avoidance_offsets_m">(.*?)</rosparam>',
                   texts[launch_name],re.S)
if template is None: raise SystemExit('Package launch has no avoidance_offsets_m')
installed_launch,new_count=re.subn(
 r'(<rosparam\s+param="avoidance_offsets_m">).*?(</rosparam>)',
 lambda v:v[1]+template.group(1)+v[2],installed_launch,count=1,flags=re.S)
if new_count!=1: raise SystemExit('Cannot find installed avoidance_offsets_m; installation stopped')
texts[launch_name]=installed_launch

backup=ws/'frenet_fix_backups'/('tunnel_clearance_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
backup.mkdir(parents=True)
for name,path in dest.items(): shutil.copy2(path,backup/name)
try:
 for name,path in dest.items(): path.write_text(texts[name])
except Exception:
 for name,path in dest.items(): shutil.copy2(backup/name,path)
 raise
print('Installed TUNNEL_CLEARANCE_V1; 34 avoidance offsets; installed jerk weight preserved:',m[1])
print('Backup:',backup)
print('Restart roslaunch; no catkin_make required for these Python replacements.')
PY
