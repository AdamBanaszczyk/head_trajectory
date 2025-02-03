# HandEyeCalibration
Code to calibrate Optitrack (or similar data) with other sensor solving the HandEye calibration problem

# Usage

``./trajectory_alignment opti.csv unity.csv --t_diff``

The resulting transformation is stored in `unity2opti.csv`

# Assumptions

For best performance:
- both unity and optitrack coordinate systems should have a Y axis facing upwards
- we take unity (left-handed cs) and negate the X axis to make it right-handed
- both trajectories should be synchronized using timestamps prior to running this code. Time offset to be specified as parser argument `--t_diff`

