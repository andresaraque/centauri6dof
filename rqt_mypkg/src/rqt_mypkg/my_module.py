import os
import rospy
import rospkg
import math
import numpy as np
import moveit_commander.robot as MICRobot
import moveit_commander.planning_scene_interface as MICPlanningSceneInterface
import moveit_commander
import moveit_commander.move_group as MICMoveGroupCommander
import moveit_msgs.msg
import geometry_msgs.msg

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QSlider, QLabel
from python_qt_binding.QtGui import QIcon, QPixmap
from centauri6dof_moveit.msg import ArmJointState
from progressbar import ProgressBar
from moveit_commander.conversions import pose_to_list
from random import randint

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)

        #os.system('roslaunch centauri6dof_moveit_config demo.launch')

        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()

        robot = MICRobot.RobotCommander()
        scene = MICPlanningSceneInterface.PlanningSceneInterface()
        group_name = "centauri6dof_arm"
        group = MICMoveGroupCommander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.

        rospy.Subscriber("/move_group/fake_controller_joint_states", JointState, self.joint_states_callback)

        img = QPixmap('/home/araque/catkin_ws/src/rqt_mypkg/resource/logo_uao.png')

        self._widget.LabelImageUao.setPixmap(img)

        self._widget.Path1Button.setIcon(QIcon.fromTheme('media-playback-start'))
        self._widget.Path1Button.clicked[bool].connect(self.fcn_path_1)

        self._widget.Path2Button.setIcon(QIcon.fromTheme('media-playback-start'))
        self._widget.Path2Button.clicked[bool].connect(self.fcn_path_2)
        """
        self._widget.SavePoseButton.setIcon(QIcon.fromTheme('object-rotate-right'))
        self._widget.SavePoseButton.clicked[bool].connect(self.fcn_update_joints)
        """
        self._widget.PlayButton.setIcon(QIcon.fromTheme('user-available'))
        self._widget.PlayButton.clicked[bool].connect(self._Send_joints_teleoperation)

        self._widget.HomeButton.setIcon(QIcon.fromTheme('go-home'))
        self._widget.HomeButton.clicked[bool].connect(self._Center_joints_teleoperation)

        self._widget.RandomizeButton.setIcon(QIcon.fromTheme('software-update-available'))
        self._widget.RandomizeButton.clicked[bool].connect(self._Randomize_joints_teleoperation)

        self._widget.GripperButton.setIcon(QIcon.fromTheme('software-update-available'))
        self._widget.GripperButton.clicked[bool].connect(self._fcn_gripper)

        self._widget.SavePoseButton.setIcon(QIcon.fromTheme('software-update-available'))
        self._widget.SavePoseButton.clicked[bool].connect(self._save_pose)

        self._widget.DeletePoseButton.setIcon(QIcon.fromTheme('software-update-available'))
        self._widget.DeletePoseButton.clicked[bool].connect(self._delete_pose)

        self._widget.ExecutePathButton.setIcon(QIcon.fromTheme('software-update-available'))
        self._widget.ExecutePathButton.clicked[bool].connect(self._execute_path)

        self.goal = ArmJointState()

        self.arr_sl = [self._widget.SlJoint1,self._widget.SlJoint2,self._widget.SlJoint3,self._widget.SlJoint4,self._widget.SlJoint5,self._widget.SlJoint6]
        self.arr_ShowSl = [self._widget.ShowJoint1,self._widget.ShowJoint2,self._widget.ShowJoint3,self._widget.ShowJoint4,self._widget.ShowJoint5,self._widget.ShowJoint6]
        

        self.pub2 = rospy.Publisher('joint_steps', ArmJointState, queue_size=20)
        rate = rospy.Rate(20) # 20hz

        self.savePose = []
        self.trajectory = []
        self.count_save_pose = 0
        self.joint_visualizer = []

        for i in xrange(0,6):
            self.arr_sl[i].setEnabled(True)
            self.arr_sl[i].setMaximum(90)
            self.arr_sl[i].setMinimum(-90)
            self.arr_sl[i].setValue(0)
            self.arr_sl[i].valueChanged.connect(self.joints_changes)
            self.arr_ShowSl[i].setEnabled(True)
            self.arr_ShowSl[i].setText(str(self.arr_sl[i].value()))

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)    

        self.grip = 0
        self.count_save_pose = 0

    def fcn_path_1(self):
        self._widget.ShowText.setText("Path one")
        
        gripper = {'open': 0, 'banana': 70, 'box': 50}
        upright = [0, 0, 0, 0, 0, 0, 0]

        #predefined movements for pick and place of an box 
        box_pick = [0, 2400, 18000, 0, 1500, 0, gripper['box']]
        box_move = [0, 1200, 14000, 0, 0, 0, gripper['box']]
        box_place = [8000, 3300, 16000, 0, 1100, 0, gripper['open']]

        banana_pick = [0, -2243, -24410, 14, -400, 0, gripper['banana']]
        banana_move = [0, -1043, -17410, 14, -3300, 0, gripper['banana']]
        banana_place = [4600, -2400, -20410, -91, -400, 0, gripper['open']]

        object_trajectories = {"box": [upright, box_pick, box_move, box_place, upright],
                               "banana": [upright, banana_pick, banana_move, banana_place, upright]}

        pub = rospy.Publisher('joint_steps', ArmJointState, queue_size=20)
        rate = rospy.Rate(20) # 20hz
        pbar = ProgressBar()
        for i in pbar(object_trajectories["box"]):
            goal = ArmJointState()
            goal.position1 = i[0]
            goal.position2 = i[1]
            goal.position3 = i[2]
            goal.position4 = i[3]
            goal.position5 = i[4]
            goal.position6 = i[5]
            goal.position7 = i[6]
            pub.publish(goal)
            rospy.sleep(7)
        
    def fcn_path_2(self):
        self._widget.ShowText.setText("Path two")
        os.system('espeak "(Path two)"')

    def fcn_update_joints(self):
        self._widget.ShowText.setText(
            "Joint Angle1: " + str(round(self.pos.position[0]*(180/math.pi)))+
            "\nJoint Angle2: " + str(round(self.pos.position[1]*(180/math.pi)))+
            "\nJoint Angle3: " + str(round(self.pos.position[2]*(180/math.pi)))+
            "\nJoint Angle4: " + str(round(self.pos.position[3]*(180/math.pi)))+
            "\nJoint Angle5: " + str(round(self.pos.position[4]*(180/math.pi)))+
            "\nJoint Angle6: " + str(round(self.pos.position[5]*(180/math.pi)))
            )

    def joint_states_callback(self, joint_state):
        self.pos = joint_state

    def joints_changes(self):

        for i in xrange(0,6):
            self.arr_ShowSl[i].setText(str(self.arr_sl[i].value()))

    def _Send_joints_teleoperation(self):
        self._widget.ShowText.setText("Moving Joints with Sliders")
        os.system('espeak "(Moving Joints with Sliders)"')

        group = self.group
        
        self.goal.position1 = np.int16(((self.arr_sl[0].value()*np.pi)/180)*(32000/(2*np.pi)))
        self.goal.position2 = np.int16(((self.arr_sl[1].value()*np.pi)/180)*(16400/(2*np.pi)))
        self.goal.position3 = np.int16(((self.arr_sl[2].value()*np.pi)/180)*(72000/(2*np.pi)))
        self.goal.position4 = np.int16(((self.arr_sl[3].value()*np.pi)/180)*(3200/(2*np.pi)))
        self.goal.position5 = np.int16(((self.arr_sl[4].value()*np.pi)/180)*(14400/(2*np.pi)))
        self.goal.position6 = np.int16(((self.arr_sl[5].value()*np.pi)/180)*(3000/(2*np.pi)))

        self.pub2.publish(self.goal)

        joint_goal = group.get_current_joint_values()

        for i in xrange(0,6):
            joint_goal[i] = (self.arr_sl[i].value()*np.pi)/180

        group.go(joint_goal, wait=True)
        group.stop()

        current_joints = self.group.get_current_joint_values()
        
    def _Center_joints_teleoperation(self): 
        for i in xrange(0,6):      
            self.arr_sl[i].setValue(0)

    def _Randomize_joints_teleoperation(self):
        result = []
        for i in xrange(0,6):
            x = randint(0,90)
            y = randint(0,90)
            result.append(x-y)
            self.arr_sl[i].setValue(result[i])

    def _fcn_gripper(self):
        self.grip = self.grip + 1

        if self.grip == 1:
            self.goal.position7 = 80
            self.pub2.publish(self.goal)
        else:
            self.goal.position7 = 0
            self.pub2.publish(self.goal)
            self.grip = 0
        

    def _save_pose(self):
        self.GoalPosition = [self.goal.position1, self.goal.position2, self.goal.position3, self.goal.position4, self.goal.position5, self.goal.position6,self.goal.position7]
        self.count_save_pose = self.count_save_pose + 1
        if self.count_save_pose == 1:
            for i in xrange(0,7):
                self.savePose.append((self.GoalPosition[i]))
                self.joint_visualizer.append((self.GoalPosition[i]))
        else:
            self.savePose = []
            self.joint_visualizer = []
            for i in xrange(0,7):
                self.savePose.append((self.GoalPosition[i]))
                self.joint_visualizer.append((self.GoalPosition[i]))
        self.trajectory.append(self.savePose)

        self._widget.ShowText.setText(
            "Successfully saved position:"
            "\nJoint1:   "+ str((self.joint_visualizer[0]*360)/32000)+" degrees"+
            "\nJoint2:   "+ str((self.joint_visualizer[1]*360)/16400)+" degrees"+         
            "\nJoint3:   "+ str((self.joint_visualizer[2]*360)/72000)+" degrees"+ 
            "\nJoint4:   "+ str((self.joint_visualizer[3]*360)/3200)+" degrees"+ 
            "\nJoint5:   "+ str((self.joint_visualizer[4]*360)/14400)+" degrees"+ 
            "\nJoint6:   "+ str((self.joint_visualizer[5]*360)/3000)+" degrees"+ 
            "\nGripper:  "+ str(self.joint_visualizer[6])+" degrees"
            )

        print(self.trajectory)

    def _delete_pose(self, scale=1):

        self.trajectory.pop(len(self.trajectory) - 1)
        print(self.trajectory)
        self._widget.ShowText.setText("Previous pose delete successfully")


    def _execute_path(self):
        group = self.group
        joint_goal = group.get_current_joint_values()
        for num_array_pose in self.trajectory:
            goal = ArmJointState()
            goal.position1 = num_array_pose[0]
            joint_goal[0] = (num_array_pose[0]*(2*np.pi))/32000
            goal.position2 = num_array_pose[1]
            joint_goal[1] = (num_array_pose[1]*(2*np.pi))/16400
            goal.position3 = num_array_pose[2]
            joint_goal[2] = (num_array_pose[2]*(2*np.pi))/72000
            goal.position4 = num_array_pose[3]
            joint_goal[3] = (num_array_pose[3]*(2*np.pi))/3200
            goal.position5 = num_array_pose[4]
            joint_goal[4] = (num_array_pose[4]*(2*np.pi))/14400
            goal.position6 = num_array_pose[5]
            joint_goal[5] = (num_array_pose[5]*(2*np.pi))/3000
            goal.position7 = num_array_pose[6]
            self.pub2.publish(goal)
            group.go(joint_goal, wait=True)
            group.stop()
            rospy.sleep(5)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass