#!/usr/bin/env python3
"""AMCL path follower with LiDAR disparity extension and localization fail-safe."""
import csv
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

class HybridDriver(Node):
    def __init__(self):
        super().__init__('hybrid_driver')
        defaults = {'waypoint_file':'', 'max_speed':3.0, 'min_speed':0.45,
                    'wheelbase':0.25, 'max_steer':0.34, 'lookahead_base':0.65,
                    'lookahead_gain':0.25, 'car_width':0.42, 'wall_margin':0.16,
                    'obstacle_distance':2.5, 'emergency_distance':0.32,
                    'localization_covariance_limit':0.8}
        for key, value in defaults.items(): self.declare_parameter(key, value)
        self.wp = self._load(self.p('waypoint_file'))
        self.scan = None; self.speed = 0.0; self.enabled = True
        self.drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/race/enabled', self.enable_cb, latched)
        self.get_logger().info(f'collision-aware path follower: {len(self.wp)} waypoints')

    def p(self, name): return self.get_parameter(name).value
    def _load(self, path):
        rows=[]
        with open(path, encoding='utf-8') as f:
            for row in csv.reader(f):
                try: rows.append(tuple(map(float, row[:3])))
                except (ValueError, IndexError): pass
        if len(rows) < 20: raise RuntimeError(f'invalid waypoint file: {path}')
        return np.asarray(rows)
    def scan_cb(self, msg): self.scan = msg
    def odom_cb(self, msg): self.speed = abs(msg.twist.twist.linear.x)
    def enable_cb(self, msg):
        self.enabled = msg.data
        if not self.enabled: self.publish(0.0, 0.0)

    def pose_cb(self, msg):
        if not self.enabled or self.scan is None: return
        cov = msg.pose.covariance
        if max(cov[0], cov[7], cov[35]) > self.p('localization_covariance_limit'):
            self.get_logger().warn('AMCL uncertainty too high: stopping', throttle_duration_sec=2.0)
            self.publish(0.0, 0.0); return
        x=msg.pose.pose.position.x; y=msg.pose.pose.position.y; heading=yaw(msg.pose.pose.orientation)
        d=np.hypot(self.wp[:,0]-x, self.wp[:,1]-y); nearest=int(np.argmin(d)); n=len(self.wp)
        L=self.p('lookahead_base')+self.p('lookahead_gain')*self.speed
        target=self.wp[nearest]
        distance=0.0; i=nearest
        while distance < L:
            j=(i+1)%n; distance += np.hypot(*(self.wp[j,:2]-self.wp[i,:2])); i=j
            target=self.wp[i]
        path_angle=wrap(math.atan2(target[1]-y,target[0]-x)-heading)
        local_angle, clearance, emergency = self._free_direction(path_angle)
        if emergency:
            self.get_logger().warn('obstacle inside emergency envelope: stopping', throttle_duration_sec=1.0)
            self.publish(0.0, 0.0); return
        # Follow the racing line when clear; progressively give LiDAR authority near objects.
        influence=np.clip((self.p('obstacle_distance')-clearance)/self.p('obstacle_distance'),0.0,0.85)
        command_angle=wrap((1.0-influence)*path_angle+influence*local_angle)
        ld=max(0.25,L); steer=math.atan2(2*self.p('wheelbase')*math.sin(command_angle),ld)
        steer=float(np.clip(steer,-self.p('max_steer'),self.p('max_steer')))
        path_speed=min(float(target[2]),self.p('max_speed'))
        clearance_scale=np.clip((clearance-self.p('emergency_distance'))/1.4,0.25,1.0)
        turn_scale=max(0.35,1.0-0.65*abs(steer)/self.p('max_steer'))
        speed=max(self.p('min_speed'),path_speed*clearance_scale*turn_scale)
        self.publish(steer,speed)

    def _free_direction(self, desired):
        s=self.scan; r=np.asarray(s.ranges,float)
        a=s.angle_min+np.arange(len(r))*s.angle_increment
        valid=np.isfinite(r)&(r>=s.range_min)&(r<=s.range_max)&(np.abs(a)<=math.radians(100))
        a=a[valid]; r=r[valid]
        if len(r)<20: return desired,0.0,True
        emergency=bool(np.any(r[np.abs(a)<math.radians(25)]<self.p('emergency_distance')))
        extended=r.copy(); radius=self.p('car_width')/2+self.p('wall_margin')
        for i in np.where(np.abs(np.diff(r))>0.35)[0]:
            close=min(r[i],r[i+1]); count=int(np.ceil(radius/max(close,0.05)/s.angle_increment))
            if r[i]<r[i+1]: extended[i+1:min(len(r),i+1+count)]=np.minimum(extended[i+1:min(len(r),i+1+count)],close)
            else: extended[max(0,i-count+1):i+1]=np.minimum(extended[max(0,i-count+1):i+1],close)
        score=np.minimum(extended,5.0)-1.6*np.abs(np.vectorize(wrap)(a-desired))
        best=int(np.argmax(score)); forward=extended[np.abs(a)<math.radians(15)]
        clearance=float(np.percentile(forward,25)) if len(forward) else float(extended[best])
        return float(a[best]),clearance,emergency

    def publish(self, steer, speed):
        m=AckermannDriveStamped(); m.header.stamp=self.get_clock().now().to_msg()
        m.drive.steering_angle=float(steer); m.drive.speed=float(speed); self.drive.publish(m)

def main():
    rclpy.init(); node=HybridDriver()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.publish(0.0,0.0); node.destroy_node(); rclpy.shutdown()
