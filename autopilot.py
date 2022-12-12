"""
autopilot block for mavsim_python
    - Beard & McLain, PUP, 2012
    - Last Update:
        2/6/2019 - RWB
"""
import sys
import numpy as np
sys.path.append('..')
sys.path.append('.')
import parameters.control_parameters as AP
from tools.transfer_function import transferFunction
from tools.wrap import wrap
from pi_control import PIControl
from pd_control_with_rate import PDControlWithRate
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta


class Autopilot:
    def __init__(self, ts_control):
        # instantiate lateral controllers
        self.roll_from_aileron = PDControlWithRate(
                        kp=AP.roll_kp,
                        kd=AP.roll_kd,
                        limit=np.radians(45))
        self.course_from_roll = PIControl(
                        kp=AP.course_kp,
                        ki=AP.course_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.yaw_damper = transferFunction(
                        num=np.array([[AP.yaw_damper_kr, 0]]),
                        den=np.array([[1, AP.yaw_damper_p_wo]]),
                        Ts=ts_control)

        # instantiate lateral controllers
        self.pitch_from_elevator = PDControlWithRate(
                        kp=AP.pitch_kp,
                        kd=AP.pitch_kd,
                        limit=np.radians(45))
        self.altitude_from_pitch = PIControl(
                        kp=AP.altitude_kp,
                        ki=AP.altitude_ki,
                        Ts=ts_control,
                        limit=np.radians(30))
        self.airspeed_from_throttle = PIControl(
                        kp=AP.airspeed_throttle_kp,
                        ki=AP.airspeed_throttle_ki,
                        Ts=ts_control,
                        limit=1.0)
        self.commanded_state = MsgState()

    def update(self, cmd, state):

        ################## lateral autopilot #########################

        #For lateral autopilot, initiate loop by setting the course angle
        chi_c =  wrap(cmd.course_command, state.chi)
        #Based on course angle, apply ki and kp controls on it to get commanded roll angle 
        phi_c = self.course_from_roll.update(chi_c, state.chi)   
        #Based on commanded roll angle, apply controls on it to get aileron setting 
        delta_a = self.roll_from_aileron.update(phi_c, state.phi, state.p) 
        #RUDDER NOT AVAILABLE
        delta_r = 0 #self.yaw_damper(state.r)

        ################ longitudinal autopilot #######################
        
        # saturate the altitude command
        h_c = self.saturate(cmd.altitude_command, state.altitude - AP.altitude_zone, state.altitude + AP.altitude_zone)
        #Based on altitude command, apply ki and kp controls on it to get commanded pitch angle 
        theta_c = self.altitude_from_pitch.update(h_c, state.altitude) 
        #Based on pitch, apply kd controls to get commmanded elevator
        delta_e = self.pitch_from_elevator.update(theta_c, state.theta, state.q) 
        #Also, obtain thrust from airspeed setting with ki, kp control
        delta_t = self.airspeed_from_throttle.update(cmd.airspeed_command, state.Va) 
        delta_t = self.saturate(delta_t, 0.0, 1.0)

        # construct output and commanded states
        delta = np.array([delta_e, #elevator
                          delta_a, #aileron
                          delta_r, #rudder
                          delta_t]) #throttle
        #delta = MsgDelta(elevator=delta_e,
                         #aileron=delta_a,
                         #rudder=delta_r,
                         #throttle=delta_t)
        self.commanded_state.altitude = cmd.altitude_command
        self.commanded_state.Va = cmd.airspeed_command
        self.commanded_state.phi = phi_c
        self.commanded_state.theta = theta_c
        self.commanded_state.chi = cmd.course_command
        return delta, self.commanded_state

    #def takeoff(self,cmd, state):

    def saturate(self, input, low_limit, up_limit):
        if input <= low_limit:
            output = low_limit
        elif input >= up_limit:
            output = up_limit
        else:
            output = input
        return output

    def wrap (chi_1 , chi_2):
        while chi_1 - chi_2 > np.pi:
            chi_1 = chi_1 - 2.0 * np.pi
        while chi_1 - chi_2 < - np.pi:
            chi_1 = chi_1 + 2.0 * np.pi
        return chi_1