# robot_sl
An implememtation of [H. Luo and Y. Demiris, "Bi-Manual Robot Shoe Lacing," 2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)](https://ieeexplore.ieee.org/abstract/document/10341934/). The project is divided into 2 modules: vision and control. The vision module is responsible for detecting the aglet and the eyelets of the shoe. The control module is responsible for solving for the optimal pattern, planning the actions, and executing the primitives. The project is built on ROS Noetic and Ubuntu 20.04.

# Components
## Vision Module
The vision module is responsible for detecting the aglet and the eyelets of the shoe. It provides a ROS service `find_targets` that returns the poses of the detected objects in camera space. The service is defined in `sl_visn/srv/FindTargets.srv`. The request includes the target names and the camera name.

Detection relies on colour segmentation. The images are segmented in RGB space. The threshold values are defined in `sl_visn/config/param.xml`. The detected objects are filtered by size and shape. The detected objects are drawn on the image and published to the topic `/sl_visn/frame_processed`. Helper threshold sliders are provided in the `color_tuner.py` script.

## Control Module
The control module is responsible for solving for the optimal pattern, planning the actions, and executing the primitives. The main process control is implemented in `sl_ctrl/scripts/sl_ctrl_node.py`. The process first calls the vision module to detect the objects. The detected objects are managed by `sl_ctrl/scripts/sl_params.py`. Then the main process then calls the primitive sequence planner defined in `sl_ctrl/scripts/sl_planner.py` to find the execution sequence. The planner requires [`LAMA`](https://github.com/aibasel/downward.git) to be installed. Then the main process executes the primitives defined in `sl_ctrl/scripts/sl_ctrl.py` (the primitives are slightly modified on top of the original paper) in the sequence found by the planner. The main process is recorded by the `sl_ctrl/scripts/sl_logger.py`. Resulting logs and the detected objects are saved in the `sl_ctrl/results` directory.

# Usage
1. Start the YuMi MoveIt
```
roslaunch yumi_moveit_config moveit_planning_execution.launch
```
2. Start the vision module
```
roslaunch sl_visn sl_visn.launch
```
3. Start the control module
```
roslaunch sl_ctrl sl_ctrl.launch
```