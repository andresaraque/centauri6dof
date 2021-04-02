# CENTAURI6DOF
ROS packages that can be use to plan and execute predefined trajectories or create new ones for simulation and real life through a user interface.
### [Video Demo Here!](https://youtu.be/Sm1RTgK0xwU)
## Simulation by the user interface with rqt
![centauri_rviz_gui.png](/centauri_rviz_gui.png)

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites
What things you need to install the software and how to install them
1. Make sure you have installed the open source operating system Ubuntu 16.04, if it has a different distribution, you may have to change some things that are not explained here.
2. Make sure you have ROS Kinetic installed correctly in Ubuntu with a workspace that works correctly. For more information about the installation and configuration of the workspace, visit: [ROSEnvironment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).
3. Install [moveit](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-moveit) for the control of manipulative robots.
4. Make sure you have the [Arduino IDE](https://www.arduino.cc/en/Main/Software) open source integrated development environment installed.
5. Download and Install the [AccelStepper](http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper-1.57.zip) library within our Arduino development environment, this library allows you to control all the stepper motors simultaneously.
6. Install the library ros_lib in Arduino IDE as shown in the [tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup).
   - If you already have ros_lib inside your libraries, you must delete ros_lib and regenerate it every time a new personalized message is introduced. If you do not see an error that says "ArmJointState.h does not exist" to solve it, you must execute the following lines in the terminal:
      ```
      rosrun rosserial_client make_libraries path_to_libraries
      rm -r ~/your_sketchbook/libraries/ros_lib
      rosrun rosserial_client make_libraries ~/your_sketchbook/libraries
      ```
7. Install [rqt](http://wiki.ros.org/rqt/UserGuide/Install/Groovy) to load the QT-based user interface.
8. Make sure you have [PIP](https://pypi.org/project/pip/) the system used to install and manage software packages written in Python.
   ```
   sudo apt install python-pip
   ```
9. Make sure you have Progressbar2
   ```
   pip install progressbar2
   ```
## How to Use:
1. Clone the repository in your workspace: _home/your_username/catkin_ws/src_
2. Make sure the GUIs in your rqt cache are updated (Don't worry if this outputs an error because the file could not be found, we just want to remove it if it is already there):
   ```
   sudo rm ~/.config/ros.org/rqt_gui.ini
   ```
3. Make _"catkin_make"_ and _"source devel/setup.bash"_ in your workspace.
4. Go to the following directory within your workspace: _"centauri6dof_moveit/centauri_moveit_arduino/arduino_centauri6dof"_. Run the file "arduino_centauri6dof.ino"
5. In the Arduino development environment select the tools tab at the top and confirm that the board is your Arduino. Check in the same tab the USB port in which the arduino is connected. For example: _dev/ttyACM#_ or also _dev/ttyUSB#_.
6. Load the code to the Arduino by pressing the upload button.
7. Go to the following directory within your workspace: _"centauri6dof_moveit_config/launch"_ and open the file "demo.launch". Modify within the node serial_node the value of the port parameter by which it had already been seen in step 5. This is done because the default port is: _dev/ttyACM0_.
8. To run the simulated robot in Rviz, open a terminal and run:
   ```
   roslaunch centauri6dof_moveit_config demo.launch 
   ```
9. To run the interface, open another terminal and run
   ```
   rosrun rqt_mypkg rqt_mypkg
   ```
## Graphical user interface
![gui.png](/GUI.png)
### Graphical user interface commands
- **Sliders**: allows the individual movement of each joint of the robot in an angular trajectory range of 180° starting from the initial position (0°) to a maximum position of 90° or - 90° in the opposite direction.
- **Gripper on/off**: The button allows the opening or closing of the clamp in the end effector of the robot.
- **Path 1-2**: the buttons path 1 and path 2 are pre-saved paths in the source code of the interface and serve as an example and demonstration of trajectories with the manipulator robot.
- **Home**: the home button allows sending the robot to its start configuration, where all its joints are in the initial position (0°).
- **Randomize**: the button allows to position each joint of the robot in a random position forming a random pose.
- **Preview**: The preview button executes the configuration of each articulation assigned by the user by means of the sliders in the Rviz visualizer for a pre-visualization.
- **Play**: the play button executes the configuration of each joint assigned to the real physical robot as well as the Rviz display if it has not been previously executed by the preview button.
- **Save pose**: the button allows you to temporarily save a pose of the robot at any time, you can save poses consecutively to create a defined trajectory to execute.
- **Remove previous pose**: remove or directly delete the last pose saved by the save pose button for the purpose of correcting a pose.
- **Execute path**: execute a pose or trajectory (several poses) that is saved or that has been imported into the interface.
- **Save path**: the saved of several poses generates a trajectory which can be saved by means of this button in the location shown below: _"home/your_username/trajectories_centauri6dof"_ and with a name that must be indicated in the text box on the left side of the "Name file" interface saves the file as a csv extension, a file of values separated by commas to generate a list of data that will contain the values of each joint.
- **Import path**: Import the saved paths by entering the name of the csv file.
## Schematic Diagram
![diagram.png](/diagram.png)
## Authors
- [Andres Araque](https://github.com/andresaraque)
- [Francisco Pedroza](https://github.com/franciscopedroza030595)
- [Victor Rommero](https://github.com/vromerocano)
