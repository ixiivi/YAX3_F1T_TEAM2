#!/bin/bash
# isolated: fake car (odom + scan only) + REAL AMCL (light) + pure_pursuit; kick AMCL 2.5 m off at 12 s
S=$(cd "$(dirname "$0")" && pwd)   # this folder (fake_car.py)
D=$HOME/.cache/magp_sim; mkdir -p $D; cd $D   # results and logs go here
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 NO_MAP_TF=1
source /opt/ros/humble/setup.bash; source ~/MAGP/MAGP/install/setup.bash
[ "$ROS_DOMAIN_ID" = 87 ] && [ "$ROS_LOCALHOST_ONLY" = 1 ] || exit 1
MAPN=$1 K=$2 DUR=${3:-45}; shift 3
M=$HOME/MAGP/MAGP/maps/$MAPN.yaml; W=$HOME/MAGP/MAGP/waypoints/${MAPN}_wp.csv; R=$D/am_${MAPN}_$K; rm -rf $R
read SX SY SYAW < <(python3 -c "
import csv,math; p=[(float(r['x']),float(r['y'])) for r in csv.DictReader(open('$W'))]; k=$K
print(p[k][0],p[k][1],math.atan2(p[k+3][1]-p[k][1],p[k+3][0]-p[k][0]))")
(timeout $((DUR+6)) ros2 run tf2_ros static_transform_publisher --x 0.267 --frame-id base_link --child-frame-id laser > st.log 2>&1 &)
(python3 "$S/fake_car.py" $M $SX $SY $SYAW $((DUR+6)) > car_am.log 2>&1 &)
(timeout -s INT $((DUR+5)) ros2 launch waypoint_nav localize_launch.py map:=$M amcl:=light auto:=false x:=$SX y:=$SY yaw:=$SYAW > loc_am.log 2>&1 &)
sleep 5
(timeout -s INT $DUR ros2 run waypoint_nav pure_pursuit --ros-args -p waypoint_file:=$W -p map_yaml:=$M -p autostart:=true -p max_speed:=3.0 -p run_log_dir:=$R "$@" > pp_am.log 2>&1 &)
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
x=tr.translation.x-2.5*math.sin(yaw); y=tr.translation.y+2.5*math.cos(yaw); yaw+=math.radians(25)
m=PoseWithCovarianceStamped(); m.header.frame_id='map'; m.header.stamp=n.get_clock().now().to_msg()
m.pose.pose.position.x=x; m.pose.pose.position.y=y; m.pose.pose.orientation.z=math.sin(yaw/2); m.pose.pose.orientation.w=math.cos(yaw/2)
m.pose.covariance[0]=m.pose.covariance[7]=0.01; m.pose.covariance[35]=0.005
p=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
for _ in range(3): p.publish(m); rclpy.spin_once(n,timeout_sec=0.1)
print('kicked AMCL 2.5 m / 25 deg off')
PY
sleep $((DUR-14))
f=$(ls $R/*.csv | tail -1)
python3 - $f <<'PY'
import csv,sys,collections
rows=list(csv.DictReader(open(sys.argv[1])))
for r in rows:
    if r['event']: print('  ',r['t'],r['event'][:90])
c=collections.Counter(r['mode'] for r in rows)
print(f"  modes(s) { {k:round(v*.1,1) for k,v in c.items()} }; final: {rows[-1]['status'][:40]} off {rows[-1]['off_path']} fit {rows[-1]['fit']}")
PY
grep -c "waiting for AMCL" pp_am.log; grep -E "Traceback|Error" pp_am.log | head -3
