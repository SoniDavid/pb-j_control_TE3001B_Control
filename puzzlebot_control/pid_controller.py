#!/usr/bin/env python3
"""PID Controller — tuned for Gazebo diff_drive plugin."""
import rclpy, numpy as np
from puzzlebot_control.base_controller import BaseController

class PIDController(BaseController):
    CONTROLLER_NAME = 'PID'
    def __init__(self):
        super().__init__('pid_controller')
        self.declare_parameter('linear_kp', 0.40)
        self.declare_parameter('linear_ki', 0.01)
        self.declare_parameter('linear_kd', 0.08)
        self.declare_parameter('angular_kp', 1.80)
        self.declare_parameter('angular_ki', 0.005)
        self.declare_parameter('angular_kd', 0.20)
        self.kp_v=self.get_parameter('linear_kp').value
        self.ki_v=self.get_parameter('linear_ki').value
        self.kd_v=self.get_parameter('linear_kd').value
        self.kp_w=self.get_parameter('angular_kp').value
        self.ki_w=self.get_parameter('angular_ki').value
        self.kd_w=self.get_parameter('angular_kd').value
        self.i_d=0.;self.i_a=0.;self.p_d=0.;self.p_a=0.

    def on_new_goal(self):
        self.i_d=self.i_a=self.p_d=self.p_a=0.

    def compute_control(self, dist, angle_err, dx, dy):
        dt=self.dt
        self.i_d=np.clip(self.i_d+dist*dt,-2,2)
        self.i_a=np.clip(self.i_a+angle_err*dt,-2,2)
        dd=(dist-self.p_d)/dt if dt>0 else 0.; self.p_d=dist
        da=(angle_err-self.p_a)/dt if dt>0 else 0.; self.p_a=angle_err
        v=self.kp_v*dist+self.ki_v*self.i_d+self.kd_v*dd
        w=self.kp_w*angle_err+self.ki_w*self.i_a+self.kd_w*da
        V=0.5*dist**2+0.5*angle_err**2
        return v,w,V

def main(args=None):
    rclpy.init(args=args); n=PIDController(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
