# import sys
# import os
# cwd = os.getcwd()
# sys.path.append(cwd)

######################################################################################
#   Sim Params
######################################################################################
ts_simulation = 0.01  # smallest time step for simulation
start_time = 0.  # start time for simulation
end_time = 1000.  # end time for simulation

ts_plotting = 0.1  # refresh rate for plots

ts_video = 0.1  # write rate for video

ts_control = ts_simulation  # sample rate for the controller