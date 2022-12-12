"""
mavsim_python
    - Chapter 6 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/5/2019 - RWB
        2/24/2020 - RWB
"""
import sys
sys.path.append('..')
sys.path.append('.')
import numpy as np
import parameters.simulation_parameters as SIM

from mav_viewer import MavViewer
#from chap3.data_viewer import DataViewer
from mav_dynamics import MavDynamics
#from UAVBook_references.chap4.wind_simulation import WindSimulation
from autopilot import Autopilot
#from chap6.autopilot_tecs import Autopilot
from tools.signals import Signals
import random
# initialize the visualization
VIDEO = False  # True==write video, False==don't write video
mav_view = MavViewer()  # initialize the mav viewer
#data_view = DataViewer()  # initialize view of data plots
if VIDEO is True:
    from video_writer import VideoWriter
    video = VideoWriter(video_name="chap6_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

# initialize elements of the architecture
#wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)

# autopilot commands
from message_types.msg_autopilot import MsgAutopilot
commands = MsgAutopilot()
"""
Va_command = Signals(dc_offset=25.0,
                     amplitude=3.0,
                     start_time=2.0,
                     frequency=0.01)
altitude_command = Signals(dc_offset=100.0,
                           amplitude=10.0,
                           start_time=0.0,
                           frequency=0.02)
course_command = Signals(dc_offset=np.radians(180),
                         amplitude=np.radians(45),
                         start_time=5.0,
                         frequency=0.015)
"""
# initialize the simulation time
sim_time = SIM.start_time

# main simulation loop
print("Press Command-Q to exit...")
while sim_time < SIM.end_time:
    # -------autopilot commands-------------
    commands.airspeed_command = 20 #Va_command.square(sim_time)
    commands.course_command = 0 #course_command.square(sim_time)
    commands.altitude_command = 10 #altitude_command.square(sim_time)
    if sim_time > 4: 
        commands.airspeed_command = 35 #Va_command.square(sim_time)
        commands.course_command = 0 #course_command.square(sim_time)
        commands.altitude_command = 20 #altitude_command.square(sim_time)
    if sim_time > 7:
        commands.airspeed_command = 50 #Va_command.square(sim_time)
        commands.course_command = np.pi/4 #course_command.square(sim_time)
        commands.altitude_command = 45 #altitude_command.square(sim_time)
    # -------autopilot-------------
    estimated_state = mav.true_state  # uses true states in the control
    delta, commanded_state = autopilot.update(commands, estimated_state)

    # -------physical system-------------
    if sim_time > 14:
        current_wind = np.array([[random.randint(0,9), 1, 1, 0, random.randint(0,9), 0]]).T  # get the new wind vector
    else:
        current_wind = np.array([[0, 0, 0, 0, 0, 0]]).T  # get the new wind vector

    mav.update(delta, current_wind)  # propagate the MAV dynamics

    # -------update viewer-------------
    mav_view.update(mav.true_state)  # plot body of MAV
    
    if VIDEO is True:
        video.update(sim_time)

    # -------increment time-------------
    sim_time += SIM.ts_simulation

if VIDEO is True:
    video.close()




