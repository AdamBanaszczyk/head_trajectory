# How Accurate is the Positioning in VR? Using Motion Capture and Robotics to Compare Positioning Capabilities of Popular VR Headsets

### **Abstract**

In this paper, we introduce a new methodology for assessing the positioning accuracy of virtual reality (VR) headsets, utilizing a cooperative industrial robot to simulate user head trajectories in a reproducible manner. We conduct a comprehensive evaluation of two popular VR headsets, i.e., Meta Quest 2 and Meta Quest Pro. Using head movement trajectories captured from realistic VR game scenarios with motion capture, we compared the performance of these headsets in terms of precision and reliability. Our analysis revealed that both devices exhibit high positioning accuracy, with no significant differences between them. These findings may provide insights for developers and researchers seeking to optimize their VR experiences in particular contexts such as manufacturing.

<p align="center">
  <img src="https://github.com/user-attachments/assets/7f255694-8c91-4237-9aa7-1e2510a403c5" width="auto" height="500">
  <img src="https://github.com/user-attachments/assets/1d670980-e9c1-46fd-bb89-accf822ae446" width="auto" height="500">
</p>

- [Arxiv](https://arxiv.org/abs/2412.06116)
- [IEEE](https://ieeexplore.ieee.org/document/10765265)

### **Citation**
<pre>
@INPROCEEDINGS{10765265,
  author={Banaszczyk, Adam and Łysakowski, Mikołaj and Nowicki, Michał R. and Skrzypczyński, Piotr and Tadeja, Sławomir K.},
  booktitle={2024 IEEE International Symposium on Mixed and Augmented Reality Adjunct (ISMAR-Adjunct)}, 
  title={How Accurate is the Positioning in VR? Using Motion Capture and Robotics to Compare Positioning Capabilities of Popular VR Headsets}, 
  year={2024},
  volume={},
  number={},
  pages={79-86},
  keywords={Headphones;Accuracy;Service robots;Industrial robots;Motion capture;Trajectory;Calibration;Synchronization;Reliability;Visual odometry;Virtual reality;Positioning accuracy;Motion capture;Robotic trajectories},
  doi={10.1109/ISMAR-Adjunct64951.2024.00027}}
</pre>

### **Environment and Dependencies**
The Python packages required to run the provided code are listed in the `requirements.txt` file.  
The code was developed and tested using Python 3.8.  
To run the code, modify the PYTHONPATH environment variable to include the absolute path to the project root folder:
<pre>export PYTHONPATH=$PYTHONPATH:/.../.../head_trajectory</pre>

### **Walkthrough**
The purpose of this repository is to provide a comprehensive methodology, detailed step-by-step procedures, and developed software for conducting similar experiments. It's important to note that, due to the complexity of the process, which involves a specific set of equipment and environments, the shared code is tailored to the data and setup used in our experiments. This repository is intended to guide you through the entire process with examples, providing helpful hints and comments on how to adapt the functions to your specific case.
The following list represents the steps in the sequence required to achieve the final results. The scripts load and output the correct data, ensuring proper results when executed.

0) `determining_robot_motions/calculate_robot_trajectory.py`
      - determine tcp poses needed to reproduce real-world headset trajectories with proper transformations calculated.  
**NOTE:** To obtain the necessary transformations, a process similar to the one described below was required. We provide the adapted code solely for calculating the final results to ensure maximum clarity. Whenever transformations are used (_TCP-to-Unity, Unity-to-OptiTrack, A, etc._) assume that steps _1–3_ have already been completed on different data.
  
1) `prepare_calibration_data/compare_traj_peaks.py`
     - script loads tcp and Unity pose data and calculates time offset between those trajectories by detecting peaks in specified axes and comparing timestamps
  
2) `prepare_calibration_data/match_poses_by_time.py`
      - same data is loaded but now the script creates .csv files required for hand-eye calibration
  
3) `HandEyeCalibration`
      - using _.csv_ files created in _2._ and time_offset calculated in _1._ solve hand-eye calibration for provided systems.  
So for the provided example and data, we should run: `./trajectory_alignment ../data/experiments/quest2/hand_eye_calib_input_data/calib_tool.csv ../data/experiments/quest2/hand_eye_calib_input_data/calib_unity.csv --t_diff=-0.27`
  
5) `evaluation/trajectory_TUM_match_by_tool_time.py`
      - read the experimental data and convert it into a unified coordinate system with synchronized timestamps in TUM format
  
6) `evaluation/auto_remove_idle_and_brake.py`
      - remove TUM trajectory parts where the robot wasn't moving and where the final brake causing a sudden stop in head motion was detected, as it could alter the results
  
7) calculate results (inside `data/experiments/linked_tum` run:)
    - `evo_ape tum traj1_tool1_gt_cut.txt traj1_tool2_gt_cut.txt -va --plot --plot_mode xz --t_max_diff=0.11 --save_results traj1_cut/tool1_vs_tool2.zip`
    - `evo_ape tum traj1_tool1_gt_cut.txt traj1_quest2_to_tool1_cut.txt -va --plot --plot_mode xz --save_results traj1_cut/tool1_vs_quest_2.zip`
    - `evo_ape tum traj1_tool1_gt_cut.txt traj1_quest_pro_to_tool1_cut.txt -va --plot --plot_mode xz --save_results traj1_cut/tool1_vs_quest_pro.zip`  
      
    - `evo_res traj1_cut/*.zip -p`

### Experiments data naming clarification
In `data/experiments` there are subdirectories `quest2` and `quest_pro` matching studied headsets. For each system, i.e. unity and tcp, there are 3 trajectories:
  - `calib` - initial run after mounting headset on the robot. We used it to gather data needed for _time_offset_ calculation and hand-eye calibration
  - `traj1` - first run of real-world head trajectory reproduced by the robot
  - `traj2` - second run of real-world head trajectory reproduced by the robot  
Those trajectories were run one after another, without any interference in a system so we can assume that transformations and time offset calculated for _calib_ are also correct for the following trajectories. This is why in steps _1-3_ we use calib data and in _5-7_ switch to _traj1/traj2_ with the same parameters.
