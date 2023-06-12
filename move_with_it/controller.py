#!/usr/bin/env python3

import cv2 as cv
import rclpy
from rclpy.node import Node
from move_with_it.brain import movement
from cv_bridge import CvBridge

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image

class Controller(Node):
    
    def __init__(self):
        super().__init__('controller')
        
        self.cap_ = cv.VideoCapture(0)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.controller_ = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.image_publisher_ = self.create_publisher(Image, "frames", 10)
        self.image_subscriber_ = self.create_subscription(Image, "frames", self.show_image, 10)
        
        self.velocity_ = TwistStamped()
        self.velocity_.header.frame_id = "panda_link0"
        self.bridge_ = CvBridge()
        
    def timer_callback(self):
        success, image = self.cap_.read()
        
        if success:      
            position = movement(image)
            x = position[0]
            y = position[1]
            
            if x != 0 and y != 0:
                if x > 0.65:
                    self.velocity_.twist.linear.y = -0.8
                elif x < 0.55:
                    self.velocity_.twist.linear.y = 0.8
                    
                if y > 0.65:
                    self.velocity_.twist.linear.x = -0.8
                elif y < 0.55:
                    self.velocity_.twist.linear.x = 0.8
                
            self.velocity_.header.stamp = self.get_clock().now().to_msg()
            self.controller_.publish(self.velocity_)
            image = cv.flip(image, 1)
            self.image_publisher_.publish(self.bridge_.cv2_to_imgmsg(image))
            
    def show_image(self, data):
        frame = self.bridge_.imgmsg_to_cv2(data)
        cv.imshow('Controller', frame)
        cv.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    