# Final Project Phase 1
[![CICD Workflow status](https://github.com/dzinobile/ENPM700_Final_Group1/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)](https://github.com/dzinobile/ENPM700_TDD_group1/actions/workflows/run-unit-test-and-upload-codecov.yml)

[![codecov](https://codecov.io/gh/dzinobile/ENPM700_Final_Group1/graph/badge.svg?token=swo8nEary3)](https://codecov.io/gh/dzinobile/ENPM700_Final_Group1)

## Links
### AIP Tracker
https://docs.google.com/spreadsheets/d/12TAMyx9cW5lwyse37VmfeIvLcLHgc6YgxtjMCqo1hIw/edit?usp=sharing

## Objectives
Our objective for phase 1 was originally to create proof of concept simulations in Webots demonstrating multi-robot SLAM and the ability to encircle a target, using the turtlebot3 with its built in navigation stack. We would then create the general structure of our final package and beginning filling in the functionality. 

However, we experienced difficulties with launching a full navigation stack for each separate turtlebot. We have adjusted the scope of this phase to instead focus on creating a functional SLAM search with 1 turtlebot3 in a custom environment, with the hope that this will be applicable to a multi-robot simulation in phase 2. 

## Package contents


##


## Build and Run Instructions

```bash
sudo apt update
sudo apt install ros-humble-webots-ros2
git clone https://github.com/dzinobile/ENPM700_Final_Group1.git
cd ENPM700_Final_Group1
colcon build
source install/setup.bash
```
Confirm the Nav2 navigate_to_pose action server exists

Send one NavigateToPose goal

Verify that the goal returns a result (success or fail — but test expects success)