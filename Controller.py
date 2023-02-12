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
        - pid_gains (dict):                         pid gains

        N.B. pid_gains is a dictionary structure where the keys are 'kp_x', 'kd_z', etc.
        """

        self.params = cfparams
        self.pid_gains = pid_gains
        self.last_psi = 0
        
        self.se_phi = 0
        self.se_theta = 0
        self.se_psi = 0
        self.U = np.array([0.,0.,0.,0.])
        self.a_x = 0
        self.a_y = 0


    def compute_commands(self, setpoint, state, calc_z=True, calc_xy=True, calc_ptp=True):
        """
        Inputs:
        - setpoint (TrajPoint dataclass):   the desired control setpoint
        - state (State dataclass):          the current state of the system
        Returns:
        - U (np.array):     array of control inputs {u1-u4}

        N.B. TrajPoint is a new dataclass. Please check it out from the utils.py script
        """

        # translational

        # U1: z
        if calc_z:
            e_pos_z = setpoint.z_pos - state.z_pos
            e_vel_z = setpoint.z_vel - state.z_vel

            a_z = self.pid_gains["kp_z"] * e_pos_z + self.pid_gains["kd_z"] * e_vel_z

            U1 = self.params.mass * (a_z + self.params.g)
            self.U[0] = U1

        # x and y
        if calc_xy:
            e_pos_x = setpoint.x_pos - state.x_pos
            e_vel_x = setpoint.x_vel - state.x_vel

            e_pos_y = setpoint.y_pos - state.y_pos
            e_vel_y = setpoint.y_vel - state.y_vel

            self.a_x = self.pid_gains["kp_x"] * e_pos_x + self.pid_gains["kd_x"] * e_vel_x
            self.a_y = self.pid_gains["kp_y"] * e_pos_y + self.pid_gains["kd_y"] * e_vel_y

        # rotational
        if calc_ptp:

            e_p = setpoint.p - state.p
            e_q = setpoint.q - state.q
            e_r = setpoint.r - state.r

            phi_d = (1 / self.params.g) * \
                (self.a_x * sin(setpoint.psi) - self.a_y * cos(setpoint.psi))
            theta_d = (1 / self.params.g) * \
                (self.a_x * cos(setpoint.psi) + self.a_y * sin(setpoint.psi))
            psi_d = setpoint.psi

            e_phi = phi_d - state.phi
            e_theta = theta_d - state.theta
            e_psi = psi_d - state.psi
            
            dt = 0.01
            self.se_phi += e_phi *dt
            self.se_theta += e_theta *dt
            self.se_psi += e_psi *dt

            U2 = self.pid_gains["kd_p"] * e_p + self.pid_gains["kp_phi"] * e_phi + self.pid_gains["ki_phi"]*self.se_phi
            U3 = self.pid_gains["kd_q"] * e_q + self.pid_gains["kp_theta"] * e_theta + self.pid_gains["ki_theta"]*self.se_theta
            U4 = self.pid_gains["kd_r"] * e_r + self.pid_gains["kp_psi"] * e_psi + self.pid_gains["ki_psi"]*self.se_psi

            self.U[1] = U2
            self.U[2] = U3
            self.U[3] = U4
        

        self.last_psi = state.psi


        return self.U
    
    # def compute_rotational(self, setpoint, state):
        
    #     e_p = setpoint.p - state.p
    #     e_q = setpoint.q - state.q
    #     e_r = setpoint.r - state.r

    #     phi_d = (1 / self.params.g) * \
    #         (self.a_x * sin(setpoint.psi) - self.a_y * cos(setpoint.psi))
    #     theta_d = (1 / self.params.g) * \
    #         (self.a_x * cos(setpoint.psi) + self.a_y * sin(setpoint.psi))
    #     psi_d = setpoint.psi

    #     e_phi = phi_d - state.phi
    #     e_theta = theta_d - state.theta
    #     e_psi = psi_d - state.psi
        
    #     dt = 0.01
    #     self.se_phi += e_phi *dt
    #     self.se_theta += e_theta *dt
    #     self.se_psi += e_psi *dt

    #     U2 = self.pid_gains["kd_p"] * e_p + self.pid_gains["kp_phi"] * e_phi + self.pid_gains["ki_phi"]*self.se_phi
    #     U3 = self.pid_gains["kd_q"] * e_q + self.pid_gains["kp_theta"] * e_theta + self.pid_gains["ki_theta"]*self.se_theta
    #     U4 = self.pid_gains["kd_r"] * e_r + self.pid_gains["kp_psi"] * e_psi + self.pid_gains["ki_psi"]*self.se_psi

    #     self.U[1] = U2
    #     self.U[2] = U3
    #     self.U[3] = U4

    #     self.last_psi = state.psi
        
    #     return self.U
