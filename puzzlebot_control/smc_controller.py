#!/usr/bin/env python3
"""SMC Controller — tuned for Gazebo, returns sliding surface values."""
import rclpy, numpy as np
from puzzlebot_control.base_controller import BaseController, sat

class SMCController(BaseController):
    CONTROLLER_NAME = 'SMC'
    def __init__(self):
        super().__init__('smc_controller')
        self.declare_parameter('lambda_v', 0.60)
        self.declare_parameter('lambda_w', 2.00)
        self.declare_parameter('eta_v', 0.10)
        self.declare_parameter('eta_w', 0.50)
        self.declare_parameter('phi_v', 0.08)
        self.declare_parameter('phi_w', 0.15)
        self.lam_v=self.get_parameter('lambda_v').value
        self.lam_w=self.get_parameter('lambda_w').value
        self.eta_v=self.get_parameter('eta_v').value
        self.eta_w=self.get_parameter('eta_w').value
        self.phi_v=self.get_parameter('phi_v').value
        self.phi_w=self.get_parameter('phi_w').value

    def compute_control(self, dist, angle_err, dx, dy):
        s_v=dist; s_w=angle_err
        v=self.lam_v*s_v+self.eta_v*sat(s_v,self.phi_v)
        w=self.lam_w*s_w+self.eta_w*sat(s_w,self.phi_w)
        V=0.5*s_v**2+0.5*s_w**2
        return v,w,V,s_v,s_w

def main(args=None):
    rclpy.init(args=args); n=SMCController(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
