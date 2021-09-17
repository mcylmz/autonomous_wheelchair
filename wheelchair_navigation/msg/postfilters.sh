#!/bin/bash

# Decimation filter
rosrun dynamic_reconfigure dynparam set /camera/decimation filter_magnitude 6

# Spatial filter
rosrun dynamic_reconfigure dynparam set /camera/spatial filter_magnitude 5
rosrun dynamic_reconfigure dynparam set /camera/spatial filter_smooth_alpha 0.5
rosrun dynamic_reconfigure dynparam set /camera/spatial filter_smooth_delta 25

exit 0