#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import cv2
from cv_bridge import CvBridge

class SensorFusionDriver(Node):
    def __init__(self):
        super().__init__('sensor_fusion_driver')

        
        self.MAX_SPEED     = 4.0   # 최대 속도 
        self.MIN_SPEED     = 1.0   # 최소 속도
        self.OVERTAKE_SPEED= 5.0   # 추월 시 속도
        self.BUBBLE_RADIUS = 0.3   # 장애물 안전 버블 반지름
        self.MAX_LIDAR_DIST= 3.0   # LiDAR 최대 유효거리
        self.SCAN_ANGLE_DEG= 180.0 # 앞쪽 사용 각도
        self.DEPTH_DANGER  = 0.8   # 긴급 감속 거리
        self.DEPTH_WARN    = 1.5   # 경고 거리

        # 내부 상태
        self.bridge = CvBridge()
        self.depth_image = None
        self.emergency_brake = False
        self.opponent_detected = False
        self.opponent_direction = 'none'


        # LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # RealSense Depth
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback, 10)

        # RealSense Color
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback, 10)

        # 드라이브 명령
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.get_logger().info('자율주행 시작 LiDAR + RealSense 활성화')

   
    def depth_callback(self, msg):
      
        try:
            depth_array = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            depth_m = depth_array.astype(float) / 1000.0
            depth_m[depth_m == 0] = 5.0  

            # 이미지 중앙 1/3 영역 평균 거리
            h, w = depth_m.shape
            center = depth_m[h//3:2*h//3, w//3:2*w//3]
            avg_dist = np.mean(center[center > 0.1])

            if avg_dist < self.DEPTH_DANGER:
                self.emergency_brake = True
                self.get_logger().warn(f' 긴급! 앞 {avg_dist:.2f}m')
            else:
                self.emergency_brake = False

        except Exception as e:
            self.get_logger().error(f'Depth 오류: {e}')

    
    def color_callback(self, msg):
       
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            h, w = cv_img.shape[:2]

            # 주황색 마스크
            orange_mask = cv2.inRange(
                hsv,
                np.array([5, 100, 100]),
                np.array([25, 255, 255]))

            # 빨간색 마스크 
            red1 = cv2.inRange(hsv,
                np.array([0, 100, 100]),
                np.array([10, 255, 255]))
            red2 = cv2.inRange(hsv,
                np.array([160, 100, 100]),
                np.array([180, 255, 255]))

            mask = orange_mask | red1 | red2

            # 노이즈 제거
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            pixels = np.sum(mask > 0)

            if pixels > 500: 
                self.opponent_detected = True
                left_px  = np.sum(mask[:, :w//2] > 0)
                right_px = np.sum(mask[:, w//2:] > 0)

                if left_px > right_px * 1.5:
                    self.opponent_direction = 'left'
                elif right_px > left_px * 1.5:
                    self.opponent_direction = 'right'
                else:
                    self.opponent_direction = 'front'

                self.get_logger().info(
                    f'상대방 감지- 방향:{self.opponent_direction}')
            else:
                self.opponent_detected = False
                self.opponent_direction = 'none'

        except Exception as e:
            self.get_logger().error(f'Color 오류: {e}')

    
    def scan_callback(self, scan_msg):
        
        # LiDAR 처리
        ranges, _ = self.preprocess_lidar(scan_msg)
        ranges     = self.find_safety_bubble(ranges)
        s, e       = self.find_best_gap(ranges)
        steering   = self.calculate_steering(s, e, ranges)
        speed      = self.calculate_speed(steering, ranges)

      
        if self.emergency_brake:
            #긴급 감속
            speed = self.MIN_SPEED
            self.get_logger().warn('긴급 감속!')

        elif self.opponent_detected:
            # 상대방 대응
            if self.opponent_direction == 'front':
               
                speed = self.OVERTAKE_SPEED
                steering = float(np.clip(steering * 1.2, -0.4, 0.4))
                self.get_logger().info('추월 시도!')
            elif self.opponent_direction == 'left':
                
                steering = float(np.clip(steering - 0.05, -0.4, 0.4))
            elif self.opponent_direction == 'right':
                
                steering = float(np.clip(steering + 0.05, -0.4, 0.4))

        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

    def preprocess_lidar(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.MAX_LIDAR_DIST)
        ranges = np.clip(ranges, 0.0, self.MAX_LIDAR_DIST)
        total  = len(ranges)
        half   = int((self.SCAN_ANGLE_DEG/2.0) /
                     np.degrees(scan_msg.angle_increment))
        c = total // 2
        return ranges[max(0,c-half):min(total,c+half)], c

    def find_safety_bubble(self, ranges):
        ranges   = ranges.copy()
        min_idx  = np.argmin(ranges)
        min_dist = ranges[min_idx]
        bubble   = max(5, int(np.arctan2(
            self.BUBBLE_RADIUS, max(min_dist,0.01)) / 0.00436))
        ranges[max(0,min_idx-bubble):min(len(ranges),min_idx+bubble)] = 0.0
        return ranges

    def find_best_gap(self, ranges):
        mask = ranges > 0.1
        bs, be, bl = 0, 0, 0
        cs, cl, ig = 0, 0, False
        for i, v in enumerate(mask):
            if v:
                if not ig: cs, ig = i, True
                cl += 1
                if cl > bl: bl, bs, be = cl, cs, i
            else:
                ig, cl = False, 0
        return bs, be

    def calculate_steering(self, bs, be, ranges):
        if be <= bs: return 0.0
        best = bs + np.argmax(ranges[bs:be+1])
        return float(np.clip((best - len(ranges)//2) * 0.00436, -0.4, 0.4))

    def calculate_speed(self, steer, ranges):
        c  = len(ranges) // 2
        fd = np.mean(ranges[max(0,c-5):c+5])
        a  = abs(steer)
        if   a < 0.1: spd = self.MAX_SPEED
        elif a < 0.2: spd = self.MAX_SPEED * 0.8
        elif a < 0.3: spd = self.MAX_SPEED * 0.6
        else:         spd = self.MAX_SPEED * 0.4
        if fd < 0.5:  spd = self.MIN_SPEED
        elif fd < 1.0:spd = max(self.MIN_SPEED, spd * 0.7)
        return float(max(self.MIN_SPEED, spd))


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
