#!/usr/bin/env python3
"""ISMC Controller — tuned for Gazebo, returns integral sliding surfaces."""
import rclpy, numpy as np
from puzzlebot_control.base_controller import BaseController, sat

class ISMCController(BaseController):
    CONTROLLER_NAME = 'ISMC'
    def __init__(self):
        super().__init__('ismc_controller')
        self.declare_parameter('alpha_v', 0.20)
        self.declare_parameter('alpha_w', 0.50)
        self.declare_parameter('beta_v', 0.60)
        self.declare_parameter('beta_w', 2.00)
        self.declare_parameter('eta_v', 0.08)
        self.declare_parameter('eta_w', 0.40)
        self.declare_parameter('phi_v', 0.06)
        self.declare_parameter('phi_w', 0.12)
        self.alpha_v=self.get_parameter('alpha_v').value
        self.alpha_w=self.get_parameter('alpha_w').value
        self.beta_v=self.get_parameter('beta_v').value
        self.beta_w=self.get_parameter('beta_w').value
        self.eta_v=self.get_parameter('eta_v').value
        self.eta_w=self.get_parameter('eta_w').value
        self.phi_v=self.get_parameter('phi_v').value
        self.phi_w=self.get_parameter('phi_w').value
        self.iz=0.;self.ia=0.

    def on_new_goal(self): self.iz=self.ia=0.

    def compute_control(self, dist, angle_err, dx, dy):
        self.iz=np.clip(self.iz+dist*self.dt,-2,2)
        self.ia=np.clip(self.ia+angle_err*self.dt,-2,2)
        sv=dist+self.alpha_v*self.iz
        sw=angle_err+self.alpha_w*self.ia
        v=self.beta_v*sv+self.eta_v*sat(sv,self.phi_v)
        w=self.beta_w*sw+self.eta_w*sat(sw,self.phi_w)
        V=0.5*dist**2+0.5*angle_err**2
        return v,w,V,sv,sw

def main(args=None):
    rclpy.init(args=args); n=ISMCController(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
