1. Physical Integration
Connect the MultiSense S27 camera directly to your computer's Ethernet port using an Ethernet cable.
2. Network Configuration
You must configure a static IP address for the wired network interface. This ensures your computer is on the same subnet as the camera, which has a default IP of 10.66.171.21.
Use the nmcli command to set a static IP address on your ouster connection:
nmcli con modify ouster ipv4.method manual ipv4.addresses 10.66.171.200/24
Apply the changes and bring the connection up:
nmcli con up ouster
3. Software Setup
The following steps are required to get the driver code and its dependencies.
Prepare the Workspace
Ensure your driver repository is within a colcon workspace. Based on your system, this is the ~/colcon_ws directory. The repository should be placed inside a src folder.
cd ~/colcon_ws
Get the Driver Code
Clone the master branch of the multisense_ros2 repository. This branch contains the code for your ROS2 Humble installation.
git clone https://github.com/carnegierobotics/multisense_ros2.git src/multisense_ros2
Install Dependencies
The multisense_ros2 repository has a dependency on the LibMultiSense library, which is managed as a submodule. The git clone command does not automatically download it. You must also install system dependencies like xacro using rosdep.
git submodule init
git submodule update
rosdep install --from-paths src --ignore-src -r -y
Build the Packages
Build the core packages of the driver, which are multisense_msgs, multisense_lib, and multisense_ros.
colcon build --packages-select multisense_msgs multisense_lib multisense_ros
4. Driver Launch and Visualization
This process requires a minimum of two terminals running simultaneously. A third is recommended for visualization.
Terminal 1: Driver Launch
This terminal will run the driver and must remain active. Do not close it.
Navigate to your workspace and source the environment: cd ~/colcon_ws source install/setup.bash
Launch the driver using the correct launch file: ros2 launch multisense_ros multisense_launch.py
Terminal 2: Verification
This terminal is for verifying that the driver is running and publishing data.
Open a new terminal, navigate to your workspace, and source the environment.
Verify the topics are being published: ros2 topic list
Check the data from a specific topic: ros2 topic echo /multisense/left/image_mono/camera_info
Terminal 3: Visualization
This terminal will run the RViz2 visualization tool.
Open a new terminal, navigate to your workspace, and source the environment.
Run RViz2: ros2 run rviz2 rviz2
In the Displays panel, set the Fixed Frame to multisense/left_camera_frame.
Click Add and select the PointCloud2 display.
Set the Topic to /multisense/image_points2_color.

Common Troubleshooting
Error: Fatal: Remote branch humble not found: Use the master branch instead.
Error: 100% packet loss: The IP address of your computer is on a different subnet than the camera.
Error: package 'xacro' not found: Run rosdep install --from-paths src --ignore-src -r -y from your workspace root.
Error: file 'launch.py' not found: The launch file is named multisense_launch.py.
Error: Global Status Error, Frame [map] does not exist: Set the Fixed Frame in RViz2 to multisense/left_camera_frame.

