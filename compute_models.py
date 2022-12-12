"""
compute_ss_model
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        2/4/2019 - RWB
"""
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import minimize
from UAVBook_references.tools.rotations import Euler2Quaternion, Quaternion2Euler
import UAVBook_references.parameters.aerosonde_parameters as MAV
from UAVBook_references.parameters.simulation_parameters import ts_simulation as Ts
from UAVBook_references.message_types.msg_delta import MsgDelta
import model_coef as coef
import control.matlab as mt
import UAVBook_references.parameters.control_parameters as AP
import sympy as sp
s = sp.symbols('s')

def compute_tf_model(mav, trim_state, trim_input):
    # trim values
    mav._state = trim_state
    mav._update_velocity_data()
    Va_trim = trim_state.item(3)
    Vg = np.sqrt(trim_state.item(3)**2 + trim_state.item(4)**2 + trim_state.item(5)**2)
    Va = Vg #no wind
    alpha_trim = mav._alpha
    phi, theta_trim, psi = Quaternion2Euler(trim_state[6:10])
    rho = MAV.rho
    S = MAV.S_wing
    b = MAV.b
    c = MAV.c
    delta_e_trim = trim_input.item(0)
    delta_a_trim = trim_input.item(1)
    delta_r_trim = trim_input.item(2)
    delta_t_trim = trim_input.item(3)

    # define transfer function constants coef.a_phi1
    a_phi1 = -.5* rho* Va_trim**2 * S * b * MAV.C_p_p * (b/(2*Va_trim))
    a_phi2 =  .5* rho* Va_trim**2 * S * b * MAV.C_p_delta_a 
    a_theta1 = -(rho* Va_trim**2 * c * S * MAV.C_m_q * c / (2*MAV.Jy*2*Va_trim))
    a_theta2 = rho* Va_trim**2 * c * S * MAV.C_m_alpha / (2*MAV.Jy)
    a_theta3 = rho* Va_trim**2 * c * S * MAV.C_m_delta_e / (2*MAV.Jy)

    # Compute transfer function coefficients using new propulsion model  coef.a_V1
    a_V1 = (rho*Va_trim*S/MAV.mass)*(MAV.C_D_0 + (MAV.C_D_alpha * alpha_trim) + (MAV.C_D_delta_e * delta_e_trim)) + (rho*MAV.S_prop*MAV.C_prop*Va_trim/MAV.mass)
    a_V2 = rho*MAV.S_prop*MAV.C_prop*MAV.k_motor**2*delta_t_trim/MAV.mass
    a_V3 = MAV.gravity*np.cos(theta_trim - alpha_trim)

    ###COMPUTE TRANSFER FUNCTIONS

    #Lateral transfer functions
    T_phi_delta_a = mt.tf([a_phi2],[1., a_phi1, 0.])
    T_p_delta_a = mt.tf([a_phi2],[1., a_phi1])
    T_phi_p = mt.tf([1],[1., 0])
    T_chi_phi = mt.tf([MAV.gravity/Vg],[1., 0.])

    #Longitudinal transfer functions
    T_theta_delta_e = mt.tf([a_theta3],[1., a_theta1, a_theta2])
    T_q_delta_e = mt.tf([a_theta3, 0],[1., a_theta1, a_theta2])
    T_theta_q = mt.tf([1],[1, 0])
    T_h_theta = mt.tf([Va],[1., 0])
    
    T_Va_delta_t = mt.tf([a_V2],[1., a_V1])
    T_Va_theta = mt.tf([-a_V3],[1., a_V1]) 

    s = mt.tf('s')

    #######Compute further loop transfer functions from these

    #Lateral loop transfer functions
    T_p_delta_a_cl = mt.feedback(T_p_delta_a, AP.roll_kd)
    T_phi_delta_a_cl = T_p_delta_a_cl * T_phi_p
    #c_delta_a_phi = mt.tf([AP.roll_kp, AP.roll_ki],[1., 0])  #for ki control
    c_delta_a_phi = AP.roll_kp 
    L_phi = c_delta_a_phi * T_phi_delta_a_cl 
    T_phi_phi_c = mt.feedback(L_phi, 1)
    c_phi_chi = mt.tf([AP.course_kp, AP.course_ki],[1., 0]) 
    L_course = c_phi_chi * T_phi_phi_c * T_chi_phi
    #Final desired course to course transfer function
    T_chi_chi_c = mt.feedback(L_course, 1)

    #Longitudinal loop transfer functionss
    T_q_delta_e_cl = mt.feedback(T_q_delta_e, AP.pitch_kd)
    L_theta = T_q_delta_e_cl * T_theta_q * AP.pitch_kp
    T_theta_theta_c = mt.feedback(L_theta, 1)
    c_h = mt.tf([AP.altitude_kp, AP.altitude_ki],[1., 0])
    L_h = c_h * T_theta_theta_c * T_h_theta
    #final transfer function for desired height to height
    T_h_h_c = mt.feedback(L_h, 1)

    #return Va_trim, alpha_trim, theta_trim, a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_V1, a_V2, a_V3, T_phi_delta_a, T_chi_phi, T_theta_delta_e,  T_q_delta_e,  T_theta_q, T_h_theta, T_Va_delta_t, T_Va_theta, T_phi_phi_c, T_chi_chi_c
    return T_chi_chi_c, T_h_h_c

