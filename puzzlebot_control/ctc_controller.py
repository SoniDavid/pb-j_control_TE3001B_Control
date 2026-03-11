#!/usr/bin/env python3
"""CTC Controller — tuned for Gazebo diff_drive."""
import rclpy, numpy as np
from puzzlebot_control.base_controller import BaseController, wrap_angle

class CTCController(BaseController):
    CONTROLLER_NAME = 'CTC'
    def __init__(self):
        super().__init__('ctc_controller')
        self.declare_parameter('kp_v', 0.50)
        self.declare_parameter('kd_v', 0.30)
        self.declare_parameter('kp_w', 2.00)
        self.declare_parameter('kd_w', 0.40)
        self.kp_v=self.get_parameter('kp_v').value
        self.kd_v=self.get_parameter('kd_v').value
        self.kp_w=self.get_parameter('kp_w').value
        self.kd_w=self.get_parameter('kd_w').value

    def compute_control(self, dist, angle_err, dx, dy):
        v_des=self.kp_v*dist*np.cos(angle_err)
        w_des=self.kp_w*angle_err
        v=v_des+self.kd_v*(v_des-self.v_odom)
        w=w_des+self.kd_w*(w_des-self.w_odom)
        V=0.5*dist**2+0.5*angle_err**2
        return v,w,V

def main(args=None):
    rclpy.init(args=args); n=CTCController(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
