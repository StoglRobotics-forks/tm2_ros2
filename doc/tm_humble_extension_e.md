# __Related Projects and Tutorials Usage__
## &sect; ROS2 driver usage
> 
> :bulb: Do you prepare the __TM Robot__ ready ? Make sure that TM Robot's operating software (__TMflow__) network settings and __ROS Settings__ _(ROS Drive status: &rArr; __`Running`__)_ are ready and the __Listen node__ is running. Do you build TM relative ROS apps <sup>1</sup> on your (remote) computer?
> After the user has set up the ROS2 environment (example : [Debian packages for ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)) and built the TM relative ROS apps <sup>1</sup>  based on the specific workspace, please enter your workspace `<workspace>` by launching the terminal, and remember to make the workspace visible to ROS. 
>
>
> ```bash
> source /opt/ros/humble/setup.bash
> cd <workspace>
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> export ROS_DOMAIN_ID=<ROS_DOMAIN_ID>
> source ./install/setup.bash
> ```
>> **Note**: Domain ID is the key to ROS communication, and please make sure the ROS node works under the ROS environment setup with the same Domain ID as the robot.
>
> Now, the user can use the terminal to run the ROS node or command, but don't forget to export the correct <u>Domain ID</u> setup and source the correct setup shell files as starting a new terminal.
> Note: When you finish executing your developed scripts or motion commands through the TM ROS driver connection, press CTRL + C in all terminal windows to shut everything down.

## &sect; Usage with MoveIt2-humble (Binary)
>
> See [MoveIt2 tutorial](https://moveit.ros.org/install-moveit2/binary/) to install the MoveIt2 packages.<br/>
> ```bash
> sudo apt install ros-humble-moveit
> ```
> Then, use the following command to install these ROS2 Humble dependency packages
> ```bash
> sudo apt-get install ros-humble-controller-manager
> sudo apt install ros-humble-joint-trajectory-controller
> sudo apt install ros-humble-joint-state-broadcaster
> ```
>
>
> The `<tm2_ws>` means TM driver workspace, for example `tm2_ws` .<br/>
>
>
> Then to build the TM driver based on the <tm2_ws> workspace, please enter the specific workspace `tm2_ws` by launching the terminal, and remember to make the workspace visible to ROS.<br/>
>
>
> ```bash
> source /opt/ros/humble/setup.bash
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> cd ~/tm2_ws
> colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
> export ROS_DOMAIN_ID=<ROS_DOMAIN_ID>
> source ./install/setup.bash
> ```
>
> :bulb: If you have built the TM driver before, it is recommended that you delete the build, install and log folders by the command `rm -rf build install log`, and rebuild it. For example,<br/>
>
>
> ```bash
> source /opt/ros/humble/setup.bash
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> cd ~/tm2_ws
> rm -rf build install log
> colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
> export ROS_DOMAIN_ID=<ROS_DOMAIN_ID>
> source ./install/setup.bash
> ```
>
> The demo launches the RViz GUI and demonstrates planning and execution of a simple collision-free motion plan with TM Robot.<br/> 
> :bulb: Do you prepare the __TM Robot__ ready ? Make sure that TM Robot's operating software (__TMflow__) network settings are ready and the __Listen node__ is running.<br/>
>
> * To bring up the MoveIt2 - MoveGroup demo in simulation mode with the virtual TM Robot, by typing<br/>
>
>
> ```bash
> ros2 launch <tm_robot_type>_moveit_config <tm_robot_type>_run_move_group.launch.py
> ```
>
>> The prefix `<tm_robot_type>` means the TM Robot type, available for tm5s, tm7s, tm12s, tm14s, and tm25s models.
>
> Taking the TM12S robot as an example, use the commands introduced above, by typing
> ```bash
> ros2 launch tm12s_moveit_config tm12s_run_move_group.launch.py
> ```
>
> * The user can also manipulate the real TM Robot to run, by typing<br/>
>
> ```bash
> roslaunch <tm_robot_type>_moveit_config <tm_robot_type>_run_move_group.launch.py robot_ip:=<robot_ip_address>
> ```
> :warning:[CAUTION] This demo will let the real TM Robot move, please be careful. If the user are a beginner or unfamiliar with the arm movement path, it is recommended that the user place your hand on the big red emergency _Stick Stop Button_ at any time, and press the button appropriately in the event of any accident that may occur.<br/>
>
> Taking the TM12S robot as an example, use the commands introduced above, by typing<br/>
>
> ```bash
> ros2 launch tm12s_moveit_config tm12s_run_move_group.launch.py robot_ip:=<robot_ip_address>
> ```
>
>> The parameter `<robot_ip_address>` means the IP address of the TM Robot.<br/>
>
> Note: When you have finished, press CTRL + C in all terminal windows to shut everything down.<br/>
> :bookmark_tabs: Note1: There will be several built-in TM Robot nominal robot model settings, available for TM5S, TM7S, TM12S, TM14S, and TM25S models.<br/>
> :bookmark_tabs: Note2: TM Robot set the default to read the Xacro file, such as _TM5S_ model, to read the file _tm5s.urdf.xacro_ into robot_description or such as _TM12S_ model, to read the file _tm12s.urdf.xacro_ into robot_description. If the user wants to use the specific model parameters instead of the nominal model to control the robot, please go back to the section __6. Generate your TM Robot-Specific Kinematics Parameters Files__ to modify the Xacro file.<br/>
<div> </div>

