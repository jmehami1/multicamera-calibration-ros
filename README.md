# Multicamera Calibration ROS

This project extrinsically calibrates a multi-camera system operating in ROS by solving the problem as a pose-graph optimization. This works for multi-camera systems that have the following features:

- intrinsically calibrated cameras
- partial overlapping views (each camera must have some overlapping FOV with ONE other camera)
- cameras must behaviour according to Pinhole model with lens distortion



## Instructions

1. Collect calibration data as Rosbags using double-sided ArUco board.

2. Copy **ALL** calibration Rosbags into the directory called [ros_bags](./ros_bags). *Note: Rosbags can be called any name*

3. Open terminal pointing to this folder. Build docker compose images using the following command:

   ```
   docker compose build 
   ```

4. Docker containers will playback Rosbag/s and estimate board poses which will be saved to as CSV file called "camera_board_poses_timestamped.csv". Run docker compose images:

   ```
   docker compose up
   ```

   **Note: CPU intensive, please do not run anything else**

5. All estimated poses per camera will be displayed at the end. Once you see the following message press ctrl-c to kill containers

![](/home/jmeh/Git/multicamera-calibration-ros/images/docker_containers_finished.png)

*NOTE: Check that all cameras have relatively similar scale of estimated poses. If not, you will need to recollect the data.

6. Then stop and remove containers:

   ```
   docker compose down
   ```

7. Run MATLAB calibration script `Main_Camera_Posegraph_Calibration.m` (Requires MATLAB 2019b or higher)

Extrinsic poses will be saved in a YAML file called `optimised_camera_poses.yaml` located in [calibration_files](./calibration_files).
