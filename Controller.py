import numpy as np
from math import sin, cos

class Controller3D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains, dt):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (PIDGains dataclass):           pid gain class
        """
        self.params = cfparams
        self.pid_gains = pid_gains


    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}
        """
        U = np.array([0.,0.,0.,0.])

        # translational

        e_pos_z = setpoint.z_pos - state.z_pos
        e_vel_z = setpoint.z_vel - state.z_vel

        a_z = self.pid_gains["kp_z"] * e_pos_z + self.pid_gains["kd_z"] * e_vel_z + self.params.g

        U1 = self.params.mass * a_z
        U[0] = U1

        # rotational

        e_p = setpoint.p - state.p
        e_q = setpoint.q - state.q
        e_r = setpoint.r - state.r

        e_phi = setpoint.phi - state.phi
        e_theta = setpoint.theta - state.theta
        e_psi = setpoint.psi - state.psi

        U2 = self.pid_gains["kd_p"] * e_p + self.pid_gains["kp_phi"] * e_phi
        U3 = self.pid_gains["kd_q"] * e_q + self.pid_gains["kp_theta"] * e_theta
        U4 = self.pid_gains["kd_r"] * e_r + self.pid_gains["kp_psi"] * e_psi

        U[1] = U2
        U[2] = U3
        U[3] = U4

        return U
