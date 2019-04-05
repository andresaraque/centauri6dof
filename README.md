# CENTAURI6DOF
ROS packages that can be used to plan and execute predefined trajectories or create new ones in simulation and real life through a user interface
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
