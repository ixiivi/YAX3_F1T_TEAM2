#!/bin/bash
# isolated: fake car on a map, pure_pursuit with match_device=$1, a 1.5 m / 20 deg pose kick at ~12 s
S=$(cd "$(dirname "$0")" && pwd)   # this folder (fake_car.py)
D=$HOME/.cache/magp_sim; mkdir -p $D; cd $D   # results and logs go here
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash; source ~/MAGP/MAGP/install/setup.bash
[ "$ROS_DOMAIN_ID" = 87 ] && [ "$ROS_LOCALHOST_ONLY" = 1 ] || exit 1
DEV=$1 MAPN=$2 K=${3:-40} DUR=${4:-40}
M=$HOME/MAGP/MAGP/maps/$MAPN.yaml; W=$HOME/MAGP/MAGP/waypoints/${MAPN}_wp.csv; R=$D/rl_${MAPN}_${DEV}_$K; rm -rf $R
read SX SY SYAW < <(python3 -c "
import csv,math; p=[(float(r['x']),float(r['y'])) for r in csv.DictReader(open('$W'))]; k=$K
h=math.atan2(p[k+3][1]-p[k][1],p[k+3][0]-p[k][0]); print(p[k][0],p[k][1],h)")
(python3 "$S/fake_car.py" $M $SX $SY $SYAW $((DUR+3)) > car_rl.log 2>&1 &); sleep 2
(timeout -s INT $DUR ros2 run waypoint_nav pure_pursuit --ros-args -p waypoint_file:=$W -p map_yaml:=$M -p autostart:=true -p max_speed:=3.0 -p match_device:=$DEV -p run_log_dir:=$R ${@:5} > pp_rl_$DEV.log 2>&1 &)
sleep 12
python3 - <<PY
import math, rclpy, time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('kick'); b=Buffer(); TransformListener(b,n)
t0=time.time()
while time.time()-t0<2: rclpy.spin_once(n,timeout_sec=0.1)
tr=b.lookup_transform('map','base_link',rclpy.time.Time()).transform
yaw=math.atan2(2*tr.rotation.w*tr.rotation.z,1-2*tr.rotation.z**2)
x=tr.translation.x-${KICK:-3.0}*math.sin(yaw); y=tr.translation.y+${KICK:-3.0}*math.cos(yaw); yaw+=math.radians(20)
m=PoseWithCovarianceStamped(); m.header.frame_id='map'; m.pose.pose.position.x=x; m.pose.pose.position.y=y
m.pose.pose.orientation.z=math.sin(yaw/2); m.pose.pose.orientation.w=math.cos(yaw/2)
p=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
for _ in range(3): p.publish(m); rclpy.spin_once(n,timeout_sec=0.1)
PY
sleep $((DUR-11))
f=$(ls $R/*.csv | tail -1)
python3 - $f $DEV <<'PY'
import csv,sys
rows=list(csv.DictReader(open(sys.argv[1])))
for r in rows:
    if r['event']: print('  ',sys.argv[2],r['t'],r['event'][:95])
srch=[r for r in rows if r['mode'] in ('search','verify')]
print(f"[{sys.argv[2]}] search/verify {len(srch)*0.1:.1f} s; final: {rows[-1]['status'][:40]} off {rows[-1]['off_path']} fit {rows[-1]['fit']}")
PY
grep -E "pose search on|Traceback|Error" pp_rl_$DEV.log | head -3
