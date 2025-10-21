# Nav2 for Unitree H1 and G1

ROS 2 Nav2 for Unitree H1 humanoid robot.

## Prerequisites

- ROS 2 Humble 
- Unitree H1 ROS drivers
- Nav2 packages
- IsaacSim 5.0
- Isaac Lab

## Training a Unitree G1 for Locomotion on Flat Terrain

### Training the Policy

Train the Unitree G1 for locomotion on flat terrain in headless mode:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Flat-G1-v0 --headless
```

**Training Video:**

<!-- Add your training video here -->
[![Unitree G1 Training Video](https://img.shields.io/badge/Video-Training-blue)]()

[Screencast from 10-15-2025 12:20:42 AM.webm](https://github.com/user-attachments/assets/2f1aacb2-e9b6-4b0e-be36-a159f9727475)


### Watching Your Trained Policy

Play back and visualize the trained policy on the Unitree G1:

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Flat-G1-v0 --num_envs 16 --checkpoint src/humanoid_locomotion_policy_example/h1_fullbody_controller/policy/g1_policy.pt
```

**Note:** Update the checkpoint path to point to your trained model file.

**Policy Deployment Video:**

<!-- Add your policy deployment video here -->
[![Unitree G1 Policy Deployment](https://img.shields.io/badge/Video-Policy%20Deployment-green)]()


[Screencast from 10-20-2025 05:12:22 PM.webm](https://github.com/user-attachments/assets/0bd6f9a4-2806-4af2-b398-0909cbc51d14)


## Navigation Demo

**Unitree H1 Navigation Video:**

<!-- Add your H1 navigation video here -->
[![Unitree H1 Navigation Demo](https://img.shields.io/badge/Video-H1%20Navigation-red)]()

https://github.com/user-attachments/assets/df214221-9360-430f-b7ab-f83b28b00bc4

The navigation demo was done with Unitree H1 instead of the G1 as there were some issues with making the ros controller for the G1 and the ROS Action graph which I couldnt fix immediately, so I went ahead with the H1 policy and controller. 


The video showcase Static and Dynamic obstacles. I tried doing this in an office environment, however due to my laptop having some resources constraints and the simulator constantly, i went ahead with a bare minimum warehouse setup that does not have too many assets to load, but still showcases the fucntionality required.

## Installation

```bash
# Clone repository 
git clone https://github.com/anuragroy2001/Unitreeh1-g1-RL_Training_and_Navigation.git h1_nav2_ws

# Create workspace
cd h1_nav2_ws

# Install dependencies
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Build
colcon build
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run H1 joint command policy
ros2 launch h1_fullbody_controller h1_fullbody_controller.launch.py

#Open another terminal and run nav2 bringups
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_slam humanoid.launch.py
```

## Prerequisites to run the USD file in Isaac Sim

1. Go to the usd directory and update the config file
```bash
cd ~/h1_nav2_ws/src/my_slam/usd
nano config.yaml
```

2. Change directories in config file
   - command_file: /(path_to_current_usd_directory)/command.txt
   - output_dir: ~/ReplicatorResult
   - command_file: /(path_to_current_usd_directory)/robot_command.txt
   - asset_path: /(path_to_current_usd_directory)/h1_ros.usd

3. Open USD file in Isaac Sim & enable extensions
   - Window -> Extensions -> search for "agent" and "navigation" respectively
   - Enable "ACTOR SDG", "ACTOR SDG UI", "NAVIGATION BUNDLE (BETA)", "NAVIGATION CORE (BETA)", "NAVIGATION UI (BETA)"

4. Bake NavMesh
   - Locate and click on NavMesh prim
   - Click on edit NavMesh on property tab
   - Enable auto-bake
   - The floor should turn blue after enabling

5. Open Command Injection tab (If character does not move on play)
   - Tools -> Actions and Event Data Generation -> Command Injection
```bash
Character GoTo 5 -5 0.0 _
Character_01 GoTo 5 5 0.0 _
```
   - Press injection after play to get the character move

## Useful Links

### Isaac Lab Documentation
- [Reinforcement Learning with Existing Scripts](https://isaac-sim.github.io/IsaacLab/main/source/overview/reinforcement-learning/rl_existing_scripts.html)
- [Isaac Lab Environments Overview](https://isaac-sim.github.io/IsaacLab/main/source/overview/environments.html)

### Isaac Sim ROS 2 Tutorials
- [ROS 2 Navigation Tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_navigation.html)
- [ROS 2 RL Controller Tutorial](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/tutorial_ros2_rl_controller.html)


