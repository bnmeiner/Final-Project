"""
mavsim_python
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/2/2019 - RWB
"""
import sys
sys.path.append('..')
sys.path.append('.')
import numpy as np

import parameters.simulation_parameters as SIM
from mav_viewer import MavViewer
#from data_viewer import DataViewer
from mav_dynamics import MavDynamics
# DONT INCLUDE WIND SIMULATINO from chap4.wind_simulation import WindSimulation
from trim import compute_trim
from compute_models import compute_tf_model
from tools.signals import Signals
import control.matlab as mt
import parameters.control_parameters as AP
import matplotlib.pyplot as plt


mav = MavDynamics(SIM.ts_simulation)

# use compute_trim function to compute trim state and trim input
Va = 50.
gamma = 0 #np.pi/4.
trim_state, trim_input = compute_trim(mav, Va, gamma)
mav._state = trim_state  # set the initial state of the mav to the trim state
delta = trim_input  # set input to constant constant trim input

# compute the transfer function models linearized about trim
T_chi_chi_c, T_h_h_c = compute_tf_model(mav, trim_state, trim_input)
print(T_h_h_c)
#simplified version
s = mt.tf('s')
T_h_h_c = (7370*s + 2948)/(s**4 + 125.1*s**3 + 1504*s**2 + 7370*s + 2948)
print(T_h_h_c)
print(T_chi_chi_c)
#To plot the step response, create time array
t = np.linspace(0, 10, 1000)

#Initate step response of the system 
chi, t = mt.step(T_chi_chi_c, t)
h, t = mt.step(T_h_h_c, t)
#Graph the output 
plt.close('all')

plt.figure()
plt.plot(t,chi, label = 'matlab step response solution')
plt.title('')
plt.legend()
plt.grid()
plt.xlabel('t')
plt.ylabel('chi (deg)')
plt.show()

plt.figure()
plt.plot(t,h, label = 'matlab step response solution')
plt.title('')
plt.legend()
plt.grid()
plt.xlabel('t')
plt.ylabel('chi (deg)')
plt.show()


