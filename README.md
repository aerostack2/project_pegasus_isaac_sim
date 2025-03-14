# Project Pegasus Simulator in Isaac Sim

This project aims at installing and running a photorealistic simulation of a drone using [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator), [Nvidia Isaac Sim](https://developer.nvidia.com/isaac/sim) and [Aerostack 2](https://github.com/aerostack2/aerostack2). Here a short video of the resulting simulation:

https://github.com/user-attachments/assets/a465b64a-a23c-4e7f-9d47-6908acafc153

## Installation

This project has been tested in Ubuntu 22.04, Isaac Sim 4.2.0 and ROS 2 (Humble). It is recommended to have an Nvidia GPU with CUDA enabled capabilities and enough VRAM (tested on Nvidia RTX4090).

Note: Check Troubleshooting section if you have installation problems and remember to deactivate conda for all the building steps performed in the following sections (how to deactivate is also in Troubleshooting section)

### Clone this repository
```
mkdir ~/workspace
cd ~/workspace
git clone https://github.com/aerostack2/project_pegasus_isaac_sim
```

### Install Pegasus Simulator and Isaac Sim 4.2.0

Follow this instructions to install Isaac Sim 4.2.0 (https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html)

You should be able now to run:
```
# Run the bundled python interpreter and see if it prints on the terminal "Hello World."
ISAACSIM_PYTHON -c "print('Hello World.')"

# Run the python interpreter and check if we can run a script that starts the simulator and adds cubes to the world
ISAACSIM_PYTHON ${ISAACSIM_PATH}/standalone_examples/api/omni.isaac.core/add_cubes.py
```
Install Pegasus Simulator (https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html#installing-the-pegasus-simulator)

Install PX4 (release 1.14.3) as explained in the Pegasus Docs (https://pegasussimulator.github.io/PegasusSimulator/source/setup/installation.html#installing-px4-autopilot)

### Install ROS 2 (Humble)

Install full version of ROS 2 (Humble) as in https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Note: remember to upgrade the libraries explained in the ROS 2 docs if you don't want ROS 2 to mess with your ubuntu basic libraries.

### [Optional] Installation check
Download QGroundControl (https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html#ubuntu)

To check the installation is successful until this point:
```
source /opt/ros/humble/setup.bash
cd ~/workspace/project_pegasus_isaac_sim
ISAACSIM_PYTHON drone_camera_lidar_isaac_sim.py
```
Now the simulation with Pegasus Simulator and Isaac Sim will be running. Run now QGroundControl:
```
cd <qgroundcontrol_path>
./QGroundControl.AppImage
```
In QGroundControl you can now press "Take Off" and after, set navgation points for the drone to move. If this is working, you have everything until this point properly installed and working.

### Build Micro XRCE-DDS Agent & Client
Here it is being build from source but it can be installed in other ways (check: https://docs.px4.io/main/en/ros2/user_guide.html).

Note1: Remember to have conda deactivated properly.

Note2: If build error, fix CMakeList.txt as in https://github.com/PX4/PX4-Autopilot/issues/24477#issuecomment-2710838732
```
cd ~/workspace
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### Install Aerostack 2
Instal Aerostack 2 base:
```
sudo apt install ros-humble-aerostack2
```

Clone and build `ros-humble-as2-platform-pixhawk` (if you have conda, remove libtiff `conda uninstall libtiff` and deactivate environment previous to build):
```
mkdir -p ~/workspace/as2_platform_pixhawk_ws/src
cd ~/workspace/as2_platform_pixhawk_ws/src
git clone https://github.com/aerostack2/as2_platform_pixhawk
git clone https://github.com/PX4/px4_msgs
cd px4_msgs
git checkout remotes/origin/release/1.14
cd ../..
rosdep init
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Config [WIP]
Currently, the environment can be change in `drone_camera_lidar_isaac_sim.py`, in the line:
```
# Launch one of the worlds provided by NVIDIA
self.pg.load_environment(SIMULATION_ENVIRONMENTS["Warehouse with Shelves"])
```

There are a few possible environments (https://pegasussimulator.github.io/PegasusSimulator/source/features/environments.html). We are working currently in enabling custom environments.

## Execution

### Launch Simulation with Teleoperation

Open a terminal and launch Isaac Sim simulation:
```
source /opt/ros/humble/setup.bash
cd ~/workspace/project_pegasus_isaac_sim
ISAACSIM_PYTHON drone_camera_lidar_isaac_sim.py --px4_dir <root_to_px4>/PX4-Autopilot
```

Open another terminal and launch Micro XRCE-DDS Client:
```
cd ~/workspace/Micro-XRCE-DDS-Agent/build
./MicroXRCEAgent udp4 -p 8888
```

Open another terminal and launch Aerostack 2 (to detach from tmux you can do CTRL-B + D, or to move in the different windows CTRL-B + 0,1,2..):

```bash
./launch_as2.bash -n drone1
```

Now, you can teleoperate manually the drone in Isaac Sim, follow the instructions in the graphical interface that appeared. There are many topics being published, including RGB camera and LIDAR.

To finalize the execution, close Isaac Sim, exit Micro XRCE-DDS Client and close tmux sessions with the following command:

```bash
./stop_tmuxinator_sitl.bash
```

### Launch a mission [WIP]
There are several missions that can be executed:

- **AS2 keyboard teleoperation control**: You can use the keyboard teleoperation launched with the ground station, using the flag `-t`:
  ```bash
  ./launch_ground_station.bash -t
  ```
- **AS2 Python API single drone mission**: You can execute a mission that used AS2 Python API, launching the mission with:
  ```bash
  python3 mission.py -n drone0
  ```
- **AS2 Python API single drone mission using GPS**: You can execute a mission that used AS2 Python API with GPS, launching the mission with:
  ```bash
  python3 mission_gps.py -n drone0
  ```
- **AS2 Mission Interpreter single drone mission**: You can execute a mission that used AS2 Mission Interpreter, launching the mission with:
  ```bash
  python3 mission_interpreter.py -n drone0
  ```
- **AS2 Behavior Trees single drone mission**: You can execute a mission that used AS2 Behavior Trees, launching the mission with:
  ```bash
  python3 mission_behavior_tree.py -n drone0
  ```

You can force the end of all tmux sessions with the command:
```bash
tmux kill-server
```

If you are using gnome-terminal, you can end the execution by closing the terminal.

## Developers guide

All projects in aerostack2 are structured in the same way. The project is divided into the following directories:

- **tmuxinator**: Contains the tmuxinator launch file, which is used to launch all aerostack2 nodes.
  - **aerostack2.yaml**: Tmuxinator launch file for each drone. The list of nodes to be launched is defined here.
  - **ground_station.yaml**: Tmuxinator launch file for the ground station. The list of nodes to be launched is defined here.
- **config**: Contains the configuration files for the launchers of the nodes in the drones.
- **config_ground_station**: Contains the configuration files for the launchers of the nodes in the ground station.
- **launch_as2.bash**: Script to launch nodes defined in *tmuxinator/aerostack2.yaml*.
- **launch_ground_station.bash**: Script to launch nodes defined in *tmuxinator/ground_station.yaml*.
- **mission_\*.py**: Differents python mission files that can be executed.
- **stop_tmuxinator_as2.bash**: Script to stop all nodes launched by *launch_as2.bash*.
- **stop_tmuxinator_ground_station.bash**: Script to stop all nodes launched by *launch_ground_station.bash*.
- **stop_tmuxinator.bash**: Script to stop all nodes launched by *launch_as2.bash* and *launch_ground_station.bash*.
- **rosbag/record_rosbag.bash**: Script to record a rosbag. Can be modified to record only the topics that are needed.
- **trees\***: Contains the behavior trees that can be executed. They can be selected in the *aerostack2.yaml* file.
- **utils**: Contains utils scripts for launchers.

Both python and bash scripts have a help message that can be displayed by running the script with the `-h` option. For example, `./launch_as2.bash -h` will display the help message for the `launch_as2.bash` script.

**Note**: For knowing all parameters for each launch, you can execute the following command:

```bash
ros2 launch my_package my_launch.py -s
```

Also, you can see them in the default config file of the package, in the *config* folder. If you want to modify the default parameters, you can add the parameter to the config file.

## Troubleshooting
### Conda installation and compilation issues
Remove from `.bashrc` the conda init for the base environment. Source again `.bashrc` and run `sudo ldconfig`.
