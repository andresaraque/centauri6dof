import os
import rospy
import rospkg
import math
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QSlider
from python_qt_binding.QtGui import QIcon
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

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "centauri6dof_arm"
        group = moveit_commander.MoveGroupCommander(group_name)

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


        self._widget.Path1Button.setIcon(QIcon.fromTheme('media-playback-start'))
        self._widget.Path1Button.clicked[bool].connect(self.fcn_path_1)

        self._widget.Path2Button.setIcon(QIcon.fromTheme('media-playback-start'))
        self._widget.Path2Button.clicked[bool].connect(self.fcn_path_2)

        self._widget.UpdateAngleButton.setIcon(QIcon.fromTheme('object-rotate-right'))
        self._widget.UpdateAngleButton.clicked[bool].connect(self.fcn_update_joints)

        self._widget.UpdateMoveButton.setIcon(QIcon.fromTheme('user-available'))
        self._widget.UpdateMoveButton.clicked[bool].connect(self._Send_joints_teleoperation)

        self._widget.CenterButton.setIcon(QIcon.fromTheme('go-home'))
        self._widget.CenterButton.clicked[bool].connect(self._Center_joints_teleoperation)

        self._widget.RandomizeButton.setIcon(QIcon.fromTheme('software-update-available'))
        self._widget.RandomizeButton.clicked[bool].connect(self._Randomize_joints_teleoperation)

        self.arr_sl = [self._widget.SlJoint1,self._widget.SlJoint2,self._widget.SlJoint3,self._widget.SlJoint4,self._widget.SlJoint5,self._widget.SlJoint6]
        self.arr_ShowSl = [self._widget.ShowJoint1,self._widget.ShowJoint2,self._widget.ShowJoint3,self._widget.ShowJoint4,self._widget.ShowJoint5,self._widget.ShowJoint6]
        
        for i in xrange(0,6):
            self.arr_sl[i].setEnabled(True)
            self.arr_sl[i].setMaximum(90)
            self.arr_sl[i].setMinimum(-90)
            self.arr_sl[i].setValue(0)
            self.arr_sl[i].valueChanged.connect(self.joints_changes)
            self.arr_ShowSl[i].setEnabled(True)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)        

    def fcn_path_1(self):
        self._widget.ShowText.setText("Path one MotherFucker")
        
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
        os.system('espeak "(Finish path one MotherFucker)"')
        
    def fcn_path_2(self):
        self._widget.ShowText.setText("Path two MotherFucker")
        os.system('espeak "(Path two MotherFucker)"')

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
        
        pub2 = rospy.Publisher('joint_steps', ArmJointState, queue_size=10)
        rate = rospy.Rate(10) # 20hz

        group = self.group

        goal = ArmJointState()
        goal.position1 = np.int16(((self.arr_sl[0].value()*np.pi)/180)*(32000/(2*np.pi)))
        goal.position2 = np.int16(((self.arr_sl[1].value()*np.pi)/180)*(16400/(2*np.pi)))
        goal.position3 = np.int16(((self.arr_sl[2].value()*np.pi)/180)*(72000/(2*np.pi)))
        goal.position4 = np.int16(((self.arr_sl[3].value()*np.pi)/180)*(3200/(2*np.pi)))
        goal.position5 = np.int16(((self.arr_sl[4].value()*np.pi)/180)*(14400/(2*np.pi)))
        goal.position6 = np.int16(((self.arr_sl[5].value()*np.pi)/180)*(3000/(2*np.pi)))
        goal.position7 = 0

        pub2.publish(goal)

        joint_goal = group.get_current_joint_values()

        for i in xrange(0,6):
            joint_goal[i] = (self.arr_sl[i].value()*np.pi)/180

        group.go(joint_goal, wait=True)
        group.stop()

        current_joints = self.group.get_current_joint_values()
        
        self._widget.ShowText.setText("Move joints with sliders Mother Fucker")
        os.system('espeak "(Joints with teleoperation Mother Fucker)"')

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