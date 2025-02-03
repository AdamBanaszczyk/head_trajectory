Directory structure:

- real_world_head_trajectories
  - examples of real-world head trajectories recorded with OptiTrack during five different _Space Pirate Trainer DX_ gameplays
- experiments - data used in paper
  - linked_tum - final evaluation trajectories created with automatic `auto_remove_idle_and_brake.py`
  - linked_tum_paper_manual - final evaluation trajectories used in the paper. Brake and idle were removed manually (as the `auto_remove_idle_and_brake.py` script was developed later), so results can slightly differ from the ones in _linked_tum_
  - quest2
    - center_eye_data - Unity data recorded with our `unity_tracking_app` during the specified experiment
    - tcp_bags        - tcp data recorded with `utility/tool_pose_publisher.py` during the specified experiment
    - hand_eye_calib_input_data - data prepared for Hand-Eye calibration with `match_poses_by_time.py` based on calib trajectory
  - quest_pro
    - same structure as quest2
