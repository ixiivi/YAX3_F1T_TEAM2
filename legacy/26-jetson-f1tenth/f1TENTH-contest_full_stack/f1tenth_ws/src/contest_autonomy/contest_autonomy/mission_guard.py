#!/usr/bin/env python3
"""Count laps from the first reliable AMCL pose and stop on mission completion."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, Int32

class MissionGuard(Node):
    def __init__(self):
        super().__init__('mission_guard')
        for k,v in {'laps':3,'start_radius':0.9,'time_limit_sec':0}.items(): self.declare_parameter(k,v)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.enable=self.create_publisher(Bool,'/race/enabled',qos)
        self.lap_pub=self.create_publisher(Int32,'/race/lap',qos)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.pose_cb,10)
        self.start=None; self.inside=True; self.armed=False; self.lap=0; self.done=False
        self.begin=self.get_clock().now(); self.enable.publish(Bool(data=True))
        self.create_timer(0.2,self.timer)
    def p(self,k): return self.get_parameter(k).value
    def pose_cb(self,m):
        if self.done:return
        if max(m.pose.covariance[0],m.pose.covariance[7],m.pose.covariance[35])>0.8:return
        x=m.pose.pose.position.x; y=m.pose.pose.position.y
        if self.start is None:
            self.start=(x,y); self.get_logger().info(f'automatic start line: ({x:.2f}, {y:.2f})'); return
        inside=math.hypot(x-self.start[0],y-self.start[1])<self.p('start_radius')
        if not inside:self.armed=True
        if inside and not self.inside and self.armed:
            self.lap+=1; self.lap_pub.publish(Int32(data=self.lap)); self.get_logger().info(f'LAP {self.lap}')
            if 0<self.p('laps')<=self.lap:self.finish('target laps complete')
        self.inside=inside
    def timer(self):
        limit=self.p('time_limit_sec')
        if not self.done and limit>0 and (self.get_clock().now()-self.begin).nanoseconds*1e-9>=limit:
            self.finish('time limit')
    def finish(self,why):
        self.done=True; self.enable.publish(Bool(data=False)); self.get_logger().info(why)
def main():
    rclpy.init(); n=MissionGuard()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.enable.publish(Bool(data=False)); n.destroy_node(); rclpy.shutdown()
