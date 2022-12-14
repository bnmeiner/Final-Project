"""
compute_trim 
    - Chapter 5 assignment for Beard & McLain, PUP, 2012
    - Update history:  
        12/29/2018 - RWB
"""
#COMPLETE

import sys
sys.path.append('..')
sys.path.append('.')
import numpy as np
from scipy.optimize import minimize
from tools.rotations import Euler2Quaternion
from message_types.msg_delta import MsgDelta  
from mav_dynamics import MavDynamics

def compute_trim(mav, Va, gamma):
    # define initial state and input
    e0 = Euler2Quaternion(0., gamma, 0.)
    state0 = np.array([[0],  # (0)
                       [0],   # (1)
                       [mav._state[2]],   # Down Position
                       [Va],   # (3)  Velocity in x direction
                       [0],    # (4)
                       [0],    # (5)
                       [e0.item(0)],    # (6) #Initial Orientation
                       [e0.item(1)],    # (7)
                       [e0.item(2)],    # (8)
                       [e0.item(3)],    # (9)
                       [0],    # (10)
                       [0],    # (11)
                       [0]     # (12)
                       ])
    delta0 = MsgDelta()
    x0 = np.concatenate((state0, delta0.to_array()), axis=0) #make one array with [state0,
    #                                                                   delta0]
    # define equality constraints
    bound = ((None, None),(None, None),(None, None),(None, None), (None, None),(None, None),(None, None),(None, None),(None, None),(None, None),(None, None),(None, None),(None, None),\
            (-1,1),(-1,1),(-1,1),(-1,1))
    cons = ({'type': 'eq',
             'fun': lambda x: np.array([
                                x[3]**2 + x[4]**2 + x[5]**2 - Va**2,  # magnitude of velocity vector is Va
                                x[4],  # v=0, force side velocity to be zero
                                x[6]**2 + x[7]**2 + x[8]**2 + x[9]**2 - 1.,  # force quaternion to be unit length
                                x[7],  # e1=0  - forcing e1=e3=0 ensures zero roll and zero yaw in trim
                                x[9],  # e3=0
                                x[10],  # p=0  - angular rates should all be zero
                                x[11],  # q=0
                                x[12],  # r=0
                                ]),
             'jac': lambda x: np.array([
                                [0., 0., 0., 2*x[3], 2*x[4], 2*x[5], 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 2*x[6], 2*x[7], 2*x[8], 2*x[9], 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
                                ])
             })
    # solve the minimization problem to find the trim states and inputs
    res = minimize(trim_objective_fun, x0, method='SLSQP', args=(mav, Va, gamma),
                   bounds = bound, constraints=cons, options={'ftol': 1e-10, 'disp': True})
    # extract trim state and input and return
    trim_state = np.array([res.x[0:13]]).T
    trim_input = np.array([res.x.item(13), #elevator
                          res.x.item(14), #aileron
                          res.x.item(15), #rudder
                          res.x.item(16)]) #throttle
    #trim_input.print()
    print('trim_state=', trim_state.T)
    return trim_state, trim_input


def trim_objective_fun(x, mav, Va, gamma):
    state = x[0:13]
    delta = x[13:17]
    #delta = MsgDelta(elevator=x.item(13),
                    #aileron=x.item(14),
                    #rudder=x.item(15),
                    #throttle=x.item(16))
    desired_trim_state_dot = np.array([[0],[0],[-Va*np.sin(gamma)],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]])
    mav._state = state
    mav._update_velocity_data()
    forces_moments = mav._forces_moments(delta)
    f = mav._derivatives(state, forces_moments)
    tmp = desired_trim_state_dot - f
    J = np.linalg.norm(tmp[2:13])**2.0 
    return J

#TEST TO MAKE SURE IT WORKS
"""
mav = dynamics.MavDynamics(10)
Va = 25.
gamma = 0.*np.pi/180.
trim_state, trim_input = compute_trim(mav, Va, gamma)

print(trim_state)
"""
