#!/usr/bin/python
# System imports
import sys
import time
import gi
gi.require_version('Gtk', '3.0')
gi.require_version('PangoCairo', '1.0')

from gi.repository import GLib, Gtk, cairo, Pango, PangoCairo, GObject, Gdk   # Interface design using Glade
import threading
import os
import shutil
import xmlrpclib
import socket
import subprocess
import math
import cairo

# Import Tkinter
import Tkinter
from Tkinter import *                                   ###here###
import tkFileDialog

# ROS related imports
import rospy
import geometry_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv
from geometry_msgs.msg import Quaternion, Point
import std_msgs.msg
import tf
from tf.transformations import *
import rospkg
import numpy
import tf2_ros
import numpy as np

# Import custom message from pointcloud processing
import spc_uav_visual.msg

# Possible flight modes
FREEFLIGHTMODE = 0.0
HORIZCIRCLEMODE = 1.0
VERTCIRCLEMODE = 2.0
CUSTOMTRAJMODE = 3.0
NEGHORIZCIRCLEMODE = 4.0
NEGHELIXMODE = 5.0
EXTOVRIDEMODE = 6.0
INSPOVRIDEMODE = 7.0

MODE_ON = 1.0
MODE_OFF = 2.0

#Parameters for Tkinter interface
canvas_width = 1000
canvas_height = 1000
button_width = 500
button_height = 100
#Arrays for tkinter interface
array_y = list()
array_z = list()
#data = []

previous_time = 0
timescale = 1
widthscale = 40
heightscale = 30
step = 1
yy = []
zz = []

class UAV:
    def __init__(self, uav_type, length, width):
        self.getCurrentPosition = False

        self.setpointX = -3.0
        self.setpointY = 0.0
        self.setpointZ = 0.0
        self.setpointYaw = 0.0 # Degrees
        self.setpointRoll = 0.0 # Degrees
        self.setpointPitch = 0.0 # Degrees

        self.circleTrajRadius = 1.5
        self.circleTrajFreq = 2.0
        self.climbRate = 0.0 #m/s
        self.offsetX = 0.0
        self.offsetY = 0.0
        self.offsetZ = 0.0
        self.offsetYaw = 0.0 # Degrees


        self.length = length
        self.width = width

        self.currentPose = geometry_msgs.msg.PoseStamped()
        self.currentYaw = 0.0 # Degrees
        self.currentRoll = 0.0 # Degrees
        self.currentPitch = 0.0 # Degrees



        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.tfbuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfbuffer)
        
        self.userNotified = False

        self.flightMode = FREEFLIGHTMODE
        self.mavrosState = mavros_msgs.msg.State()
        self.mavrosRCOut = mavros_msgs.msg.RCOut()

        self.pubSetpoint = rospy.Publisher("gui/setpoint", geometry_msgs.msg.TransformStamped, queue_size=1)
        self.pubCommander = rospy.Publisher("gui/commander", std_msgs.msg.Float32MultiArray, queue_size=1)
        self.pubOffboard = rospy.Publisher("offboardCommand", std_msgs.msg.Bool,queue_size=1)
        self.pubArmCommand = rospy.Publisher("armCommand", std_msgs.msg.Bool,queue_size=1)
        self.pubExtOvRide = rospy.Publisher("gui/extOvRide", std_msgs.msg.Bool,queue_size=1)
        self.pubInspOvRide = rospy.Publisher("gui/InspOvRide", std_msgs.msg.Bool,queue_size=1)

        self.subFlightMode = rospy.Subscriber("flightMode", std_msgs.msg.Float32, self.flightModeCB)
        self.subMavrosState = rospy.Subscriber("mavros/state", mavros_msgs.msg.State, self.mavrosStateCB)
        self.subMavrosRCOut = rospy.Subscriber("mavros/rc/out", mavros_msgs.msg.RCOut, self.mavrosRCOutCB)
        self.subPcProcSetP = rospy.Subscriber("helix/set_p", spc_uav_visual.msg.PcPos, self.pcProcSetPCB) 
        self.subInspProcSetP = rospy.Subscriber("/Insp_p", spc_uav_visual.msg.PcPos, self.InspProcSetPCB)  
 
        
        #self.ovRideSetpointMsg = geometry_msgs.msg.TransformStamped();
        self.ovRideSetpointMsg = spc_uav_visual.msg.PcPos();
        self.InspRideSetpointMsg = spc_uav_visual.msg.PcPos();

        self.track_end_effector = MODE_OFF
        self.enable_dist_observer = MODE_OFF
        self.reject_disturbances = MODE_OFF


        # Load the correct image
        global image_path
        if uav_type == 'hexacopter':
            self.image_surface = cairo.ImageSurface.create_from_png(image_path+'ramhex.png')
        elif uav_type == 'quadcopter':
            self.image_surface = cairo.ImageSurface.create_from_png(image_path+'ramquad.png')
        else:
            print "uav_type is not valid/supported"
            sys.exit()

    ## Callback for the state of mavros
    def mavrosStateCB(self, msg):
        self.mavrosState = msg
        # print msg

    ## Callback for the rc_out from mavros
    def mavrosRCOutCB(self, msg):
        self.mavrosRCOut = msg
        # print msg

    ## The flight mode can change externally. This function updates the GUI when such a change is advertised.
    def flightModeCB(self, msg):
        self.flightMode = msg.data
        Gdk.threads_enter()
        if self.flightMode == FREEFLIGHTMODE:
            builder.get_object("FreeFlightMode").set_active(True)
        if self.flightMode == HORIZCIRCLEMODE:
            builder.get_object("HorzCircleMode").set_active(True)
        if self.flightMode == NEGHORIZCIRCLEMODE:
            builder.get_object("NegHorzCircleMode").set_active(True)
        if self.flightMode == NEGHORIZCIRCLEMODE:
            builder.get_object("NegHelixMode").set_active(True)
        if self.flightMode == VERTCIRCLEMODE:
            builder.get_object("VertCircleMode").set_active(True)
        if self.flightMode == CUSTOMTRAJMODE:
            builder.get_object("CustomTrajMode").set_active(True)
        if self.flightMode == EXTOVRIDEMODE:
            builder.get_object("ExtOvRideMode").set_active(True)
        if self.flightMode == INSPOVRIDEMODE:
            builder.get_object("InspOvRideMode").set_active(True)
        Gdk.threads_leave()

    ## Publish the setpoint. The pose setpoint is broadcasted on the TF tree (and published on a topic but that is not used I think).
    ## The flight mode, contact force and approach pitch are published on a topic.
    def setSetpoint(self):
        setpointMsg = geometry_msgs.msg.TransformStamped()
        setpointMsg.header.stamp = rospy.Time.now()
        setpointMsg.header.frame_id = SetpointFrameId
        setpointMsg.child_frame_id = SetpointChildFrameId
        
        setpointMsg.transform.translation.x = self.setpointX
        setpointMsg.transform.translation.y = self.setpointY
        setpointMsg.transform.translation.z = self.setpointZ

        quaternion = tf.transformations.quaternion_from_euler((self.setpointRoll*2*math.pi)/360, (self.setpointPitch*2*math.pi)/360, (self.setpointYaw*2*math.pi)/360)
        setpointMsg.transform.rotation.x = quaternion[0]
        setpointMsg.transform.rotation.y = quaternion[1]
        setpointMsg.transform.rotation.z = quaternion[2]
        setpointMsg.transform.rotation.w = quaternion[3]

        self.broadcaster.sendTransform(setpointMsg)

        self.pubSetpoint.publish(setpointMsg)

        commanderMsg = std_msgs.msg.Float32MultiArray()
        commanderMsg.data = [self.flightMode, self.track_end_effector, self.enable_dist_observer, self.reject_disturbances]
        self.pubCommander.publish(commanderMsg)

        return True


    ## Look up the local position in the TF tree. If the transformation does not exist or times out, the user is notified.
    def getLocalPos(self):
        try:
            local_position = self.tfbuffer.lookup_transform(LocalPosFrameId, LocalPosChildFrameId, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        if 'local_position' in locals():
            if rospy.Time.now().to_sec() - local_position.header.stamp.to_sec() < 1.0:
                self.currentPose.header = local_position.header
                self.currentPose.pose.position.x = local_position.transform.translation.x
                self.currentPose.pose.position.y = local_position.transform.translation.y
                self.currentPose.pose.position.z = local_position.transform.translation.z
                self.currentPose.pose.orientation = local_position.transform.rotation
                euler = tf.transformations.euler_from_quaternion([self.currentPose.pose.orientation.x, self.currentPose.pose.orientation.y, self.currentPose.pose.orientation.z, self.currentPose.pose.orientation.w])
                self.currentYaw = euler[2]*360/(2*math.pi)
                self.currentRoll = euler[0]*360/(2*math.pi)
                self.currentPitch = euler[1]*360/(2*math.pi)

                ## Update the GUI. Enter and leave threads is necessary to ensure that this is the only function at the moment to change the GUI.
                Gdk.threads_enter()
                pos_draw.queue_draw()
                yaw_draw.queue_draw()
                roll_draw.queue_draw()
                pitch_draw.queue_draw()
                z_draw.queue_draw()
                Gdk.threads_leave()

                self.userNotified = False

            else:
                if not self.userNotified:
                    print "["+str(local_position.header.stamp.to_sec())+"]: Transform "+LocalPosFrameId+"->"+LocalPosChildFrameId+" timed out."
                    self.userNotified = True
        else:
            if not self.userNotified:
                print "["+str(rospy.Time.now().to_sec())+"]: Transform "+LocalPosFrameId+"->"+LocalPosChildFrameId+" does not exist."
                self.userNotified = True

        return True
    
    
    ## listen to override setpoint msg from pc_prc node (point cloud processign node)
    def pcProcSetPCB(self, msg):
        self.ovRideSetpointMsg = msg;

    def InspProcSetPCB(self, msg):
        self.InspRideSetpointMsg = msg;
        


## This function is executed when the close button of the window is pressed.
def btnClose(widget, event):
    # Quit and return to terminal.
    Gtk.main_quit(widget, event)


## This function is executed when the user clicks in the position window.
def btnPressPos(widget, event):
    global flightMap
    if flightMap == 'ramlab':
        if ((event.x > 50) and (event.x < 450) and (event.y > 240) and (event.y < 495)) or ((event.x > 175) and (event.x < 450) and (event.y > 95) and (event.y < 240)): # Within boundaries
            uav1.setpointX, uav1.setpointY = pixelToMeter(event.x, event.y) # Convert pixel location to metric setpoint
    else:
        if (event.x > 55) and (event.x < 645) and (event.y > 10) and (event.y < 690):
            uav1.setpointX, uav1.setpointY = pixelToMeter(event.x, event.y)
    pos_draw.queue_draw() # Redraw the position window

## This function is executed when the user clicks in the yaw window.
def btnPressYaw(widget, event):
    if (event.x > 50) and (event.x < 450) and (event.y > 30) and (event.y < 45): # Within boundaries
        uav1.setpointYaw = (250 - event.x)/400*360
        yaw_draw.queue_draw()

## This function is executed when the user clicks in the roll window.
def btnPressRoll(widget, event):
    if (event.x > 50) and (event.x < 450) and (event.y > 30) and (event.y < 45): # Within boundaries
        uav1.setpointRoll = (250 - event.x)/400*360
        roll_draw.queue_draw()

## This function is executed when the user clicks in the pitch window.
def btnPressPitch(widget, event):
    if (event.x > 50) and (event.x < 450) and (event.y > 30) and (event.y < 45): # Within boundaries
        uav1.setpointPitch = (250 - event.x)/400*360
        pitch_draw.queue_draw()

## This function is executed when the user clicks in the z window.
def btnPressZ(widget, event):
    if (event.x > 80) and (event.x < 95) and (event.y > 50) and (event.y < 450):
        uav1.setpointZ = (350 - event.y)/400*4
        z_draw.queue_draw()

## This function is executed when the Get current position button is clicked.
## Sets the current position as setpoint and updates the GUI
def btnGetCurrentPosition(button):
    uav1.setpointX = uav1.currentPose.pose.position.x
    uav1.setpointY = uav1.currentPose.pose.position.y
    uav1.setpointZ = uav1.currentPose.pose.position.z
    uav1.setpointYaw = uav1.currentYaw
    uav1.setpointRoll = uav1.currentRoll
    uav1.setpointPitch = uav1.currentPitch

    pos_draw.queue_draw()
    yaw_draw.queue_draw()
    z_draw.queue_draw()


## This function is executed when the Set setpoint button is clicked.
## Checks for valid input and sets the setpoint. Then redraws the GUI.
def btnSetSetpoint(button):
    if is_number(Xtextbox.get_text()) and is_number(Ytextbox.get_text()) and is_number(Ztextbox.get_text()) and is_number(Yawtextbox.get_text()):
        uav1.setpointX = float(Xtextbox.get_text())
        uav1.setpointY = float(Ytextbox.get_text())
        uav1.setpointZ = float(Ztextbox.get_text())
        uav1.setpointYaw = float(Yawtextbox.get_text())
        pos_draw.queue_draw()
        yaw_draw.queue_draw()
        z_draw.queue_draw()
    else:
        print "Not a valid input!"


## This function is executed when the Poulate with current position button is clicked.
## Puts the current position in the setpoint boxes
def btnPopulate(button):
    Xtextbox.set_text(str(round(uav1.currentPose.pose.position.x, 2)))
    Ytextbox.set_text(str(round(uav1.currentPose.pose.position.y, 2)))
    Ztextbox.set_text(str(round(uav1.currentPose.pose.position.z, 2)))
    Yawtextbox.set_text(str(round(uav1.currentYaw, 2)))


## This function is executed when one of the flight mode radio buttons is clicked.
## Checks if the flight mode transition is possible and switches flight mode
def modeButtonClicked(button):
    if button.get_active():
        if button.get_label() == "Free flight":
            uav1.flightMode = FREEFLIGHTMODE
        if button.get_label() == "Horizontal Circle":
            uav1.flightMode = HORIZCIRCLEMODE
            getCircleParameters()
        if button.get_label() == "Neg. Horiz. Circle rotating":
            uav1.flightMode = NEGHORIZCIRCLEMODE
            getCircleParameters()
        if button.get_label() == "Neg. Helix":
            uav1.flightMode = NEGHELIXMODE
            getHelixParameters()
        if button.get_label() == "Vertical Circle":
            uav1.flightMode = VERTCIRCLEMODE
            getCircleParameters()
        if button.get_label() == "Custom Trajectory":
            uav1.flightMode = CUSTOMTRAJMODE
            # Captures current setpoint as trajectory's starting point
            uav1.offsetY = uav1.setpointY
            uav1.offsetZ = uav1.setpointZ
            global step
            step = 0
        if button.get_label() == "External override":
            uav1.flightMode = EXTOVRIDEMODE
        if button.get_label() == "Insp override":
            uav1.flightMode = INSPOVRIDEMODE


## This function extracts the circluar trajectories radius, frequency and starting point
def getCircleParameters():
    # Initialize trajectory time
    global traj_begin_time
    traj_begin_time = rospy.Time.now()
    # Capture radius and frequency entries
    if is_number(circleRadius.get_text()):
        uav1.circleTrajRadius = float(circleRadius.get_text())
    else:
        print "Enter Valid Radius"
        uav1.circleTrajRadius = 0.0
    if is_number(circleFreq.get_text()):
        if uav1.flightMode == NEGHORIZCIRCLEMODE or NEGHELIXMODE:
            #taking abs to avoid misunderstandings if the frequency has to be passed negative or not
            uav1.circleTrajFreq = abs(float(circleFreq.get_text()))
        else:
            uav1.circleTrajFreq = float(circleFreq.get_text())
    else:
        print "Enter Valid Frequency"
        uav1.circleTrajFreq = 0.0
    # Captures current setpoint as trajectory's starting point
    uav1.offsetX = uav1.setpointX
    uav1.offsetY = uav1.setpointY
    uav1.offsetZ = uav1.setpointZ
    uav1.offsetYaw = uav1.setpointYaw
    
    
## This function extracts the climb rate in m/s for helix trajectories
def getClimbRate():
    if is_number(climbRate.get_text()):
        uav1.climbRate = float(climbRate.get_text())
    else:
        print "Enter Valid Climb Rate"
        uav1.climbRate = 0.0
        

## This function extracts the helix trajectories radius, frequency, starting point and climb rate       
def getHelixParameters():
    getCircleParameters()
    getClimbRate()
       


## This function is executed on the callback of the click on the offboard button in the interface.
## Checks the status of the offboard button and sends the offboard command to mavros.
def offboardToggled(button):
    if button.get_active():
        true = std_msgs.msg.Bool()
        true.data = True
        uav1.pubOffboard.publish(true)
        uav1.pubArmCommand.publish(true)
    else:
        false = std_msgs.msg.Bool()
        false.data = False
        uav1.pubOffboard.publish(false)
        uav1.pubArmCommand.publish(false)



## This function is executed when Ramlab or SmartXP is clicked.
## Loads the correct flight map, changes the location of the origin and resizes the window.
def flightmapBtnClicked(button):
    global flightMap, map_image_surface, origin_x, origin_y
    if button.get_active():
        if button.get_label() == "Ramlab":
            flightMap = 'ramlab'
            map_image_surface = cairo.ImageSurface.create_from_png(image_path+'ramlab.png')
            origin_x = 265.0
            origin_y = 299.0
            pos_draw.set_size_request(500, 500) #1 pixel is 1 cm
        else:
            flightMap = 'smartXP'
            map_image_surface = cairo.ImageSurface.create_from_png(image_path+'smartXP.png')
            origin_x = 315.0
            origin_y = 370.0
            pos_draw.set_size_request(700, 700) #1 pixel is 1 cm
        pos_draw.queue_draw()


## This function is executed when a key is pressed.
## Processes the key and changes the setpoint accordingly
def window_key(widget,event):
    if event.keyval == ord('w'):
        uav1.setpointY = uav1.setpointY + 0.01
        pos_draw.queue_draw()
    elif event.keyval == ord('s'):
        uav1.setpointY = uav1.setpointY - 0.01
        pos_draw.queue_draw()
    elif event.keyval == ord('d'):
        uav1.setpointX = uav1.setpointX + 0.01
        pos_draw.queue_draw()
    elif event.keyval == ord('a'):
        uav1.setpointX = uav1.setpointX - 0.01
        pos_draw.queue_draw()
    elif event.keyval == ord('q'):
        uav1.setpointYaw = uav1.setpointYaw + 1.0
        if uav1.setpointYaw < -180.0:
            uav1.setpointYaw = uav1.setpointYaw + 360
        elif uav1.setpointYaw > 180.0:
            uav1.setpointYaw = uav1.setpointYaw - 360
        yaw_draw.queue_draw()
    elif event.keyval == ord('e'):
        uav1.setpointYaw = uav1.setpointYaw - 1.0
        if uav1.setpointYaw < -180.0:
            uav1.setpointYaw = uav1.setpointYaw + 360
        elif uav1.setpointYaw > 180.0:
            uav1.setpointYaw = uav1.setpointYaw - 360
        yaw_draw.queue_draw()
    elif event.keyval == ord('u'):
        uav1.setpointZ = uav1.setpointZ + 0.01
        if uav1.setpointZ > 3.0:
            uav1.setpointZ = 3.0
        z_draw.queue_draw()
    elif event.keyval == ord('j'):
        uav1.setpointZ = uav1.setpointZ - 0.01
        z_draw.queue_draw()
    elif event.keyval == ord('m'):
        uav1.setpointRoll = uav1.setpointRoll + 0.25
        if uav1.setpointRoll < -180.0:
            uav1.setpointRoll = uav1.setpointRoll + 360
        elif uav1.setpointRoll > 180.0:
            uav1.setpointRoll = uav1.setpointRoll - 360
        roll_draw.queue_draw()
    elif event.keyval == ord('b'):
        uav1.setpointRoll = uav1.setpointRoll - 0.25
        if uav1.setpointRoll < -180.0:
            uav1.setpointRoll = uav1.setpointRoll + 360
        elif uav1.setpointRoll > 180.0:
            uav1.setpointRoll = uav1.setpointRoll - 360
        roll_draw.queue_draw()
    elif event.keyval == ord('c'):
        uav1.setpointPitch = uav1.setpointPitch + 0.25
        if uav1.setpointPitch < -180.0:
            uav1.setpointPitch = uav1.setpointPitch + 360
        elif uav1.setpointPitch > 180.0:
            uav1.setpointPitch = uav1.setpointPitch - 360
        pitch_draw.queue_draw()
    elif event.keyval == ord('z'):
        uav1.setpointPitch = uav1.setpointPitch - 0.25
        if uav1.setpointPitch < -180.0:
            uav1.setpointPitch = uav1.setpointPitch + 360
        elif uav1.setpointPitch > 180.0:
            uav1.setpointPitch = uav1.setpointPitch - 360
        pitch_draw.queue_draw()

## Timer Callback for trajectories setpoint
def update_trajectory_setpoint(event):
    if uav1.flightMode == HORIZCIRCLEMODE:
        update_horizontal_circle(event)
    if uav1.flightMode == NEGHORIZCIRCLEMODE:
        update_neg_horizontal_circle(event)
    if uav1.flightMode == NEGHELIXMODE:
        update_neg_helix(event)
    if uav1.flightMode == VERTCIRCLEMODE:
        update_vertical_circle(event)
    if uav1.flightMode == CUSTOMTRAJMODE:
        update_custom_trajectory(event)
    if uav1.flightMode == EXTOVRIDEMODE:
        update_ext_override(event)
        uav1.pubExtOvRide.publish(True)
    else:
        uav1.pubExtOvRide.publish(False)
    if uav1.flightMode == INSPOVRIDEMODE:
        update_Insp_override(event)
        uav1.pubInspOvRide.publish(True)
    else:
        uav1.pubInspOvRide.publish(False)

## Function to update GUI with MAVROS messages
def update_uav_mavros_state(event):
    # Extract MAVROS Data
    pwmArray = uav1.mavrosRCOut.channels
    stateArray = uav1.mavrosState

    Gdk.threads_enter()
    setPX4States(stateArray.connected,stateArray.armed,stateArray.guided,stateArray.mode)
    if len(pwmArray) > 0:
        setPWMValues(pwmArray)
    else:
        setPWMValues([0.0,0.0,0.0,0.0,0.0,0.0])
    Gdk.threads_leave()

## Function to update GUI Motor Values
def setPWMValues(pwmArray):
    M1_value.set_text(str(pwmArray[0]))
    M2_value.set_text(str(pwmArray[1]))
    M3_value.set_text(str(pwmArray[2]))
    M4_value.set_text(str(pwmArray[3]))
    M5_value.set_text(str(pwmArray[4]))
    M6_value.set_text(str(pwmArray[5]))

# Function to update GUI - PX4 States
def setPX4States(connected,armed,guided,mode):
    if connected:
        Connected_Value.set_markup("<span foreground='green'><b>ON</b></span>")
    else:
        Connected_Value.set_markup("<span foreground='red'><b>OFF</b></span>")

    if armed:
        Armed_Value.set_markup("<span foreground='green'><b>ON</b></span>")
    else:
        Armed_Value.set_markup("<span foreground='red'><b>OFF</b></span>")

    if guided:
        Guided_Value.set_markup("<span foreground='green'><b>ON</b></span>")
    else:
        Guided_Value.set_markup("<span foreground='red'><b>OFF</b></span>")

    Mode_Value.set_text(mode)

## This function update UAV Setpoint by a loaded custom trajectory (By: Jornt Lagaveen)
def update_custom_trajectory(event):
    global previous_time
    global timescale
    global custom_traj_y
    global custom_traj_z
    global step

    if (previous_time == 0):
	previous_time = time.time()

    if ((time.time() - previous_time) > timescale):
        previous_time = time.time()
        if (step<len(custom_traj_y)):
            uav1.setpointY= custom_traj_y[step]+uav1.offsetY
            uav1.setpointZ= custom_traj_z[step]+uav1.offsetZ
            step = step+1
            #print uav1.setpointY, "  ", uav1.setpointZ # DEBUG


## This function implements horizontal circle in x-y plane
def update_horizontal_circle(event):
    t_i = computeTimeElapsed()
    uav1.setpointX = uav1.circleTrajRadius*math.cos(uav1.circleTrajFreq*t_i) + uav1.offsetX - uav1.circleTrajRadius
    uav1.setpointY = uav1.circleTrajRadius*math.sin(uav1.circleTrajFreq*t_i) + uav1.offsetY
    # Uncomment to see setpoint changes in GUI --> Gives an error because update is too fast
    #pos_draw.queue_draw()
    #z_draw.queue_draw()

## This function implements a clockwise/negative horizontal circle in x-y plane and lets the UAV rotate with respect to the angular velocity when flying the circle
def update_neg_horizontal_circle(event):
    t_i = computeTimeElapsed()
    uav1.setpointX = -(uav1.circleTrajRadius*math.cos(uav1.circleTrajFreq*t_i)) + uav1.offsetX + uav1.circleTrajRadius
    uav1.setpointY = uav1.circleTrajRadius*math.sin(uav1.circleTrajFreq*t_i) + uav1.offsetY
    uav1.setpointYaw = -(uav1.circleTrajFreq*t_i*360/(2*math.pi)) + uav1.offsetYaw
    # Uncomment to see setpoint changes in GUI --> Gives an error because update is too fast
    #pos_draw.queue_draw()
    #z_draw.queue_draw()
    
## This function implements a clockwise/negative helix around the z axis with fixed climb rate
## and lets the UAV rotate with respect to the angular velocity of the circular motion in x and y
def update_neg_helix(event):
    t_i = computeTimeElapsed()
    uav1.setpointX = -(uav1.circleTrajRadius*math.cos(uav1.circleTrajFreq*t_i)) + uav1.offsetX + uav1.circleTrajRadius
    uav1.setpointY = uav1.circleTrajRadius*math.sin(uav1.circleTrajFreq*t_i) + uav1.offsetY
    uav1.setpointYaw = -(uav1.circleTrajFreq*t_i*360/(2*math.pi)) + uav1.offsetYaw
    uav1.setpointZ = uav1.climbRate * t_i + uav1.offsetZ;
    # Uncomment to see setpoint changes in GUI --> Gives an error because update is too fast
    #pos_draw.queue_draw()
    #z_draw.queue_draw()

## This function implements vertical circle in y-z plane
def update_vertical_circle(event):
    t_i = computeTimeElapsed()
    uav1.setpointY = uav1.circleTrajRadius*math.cos(uav1.circleTrajFreq*t_i) + uav1.offsetY - uav1.circleTrajRadius
    uav1.setpointZ = uav1.circleTrajRadius*math.sin(uav1.circleTrajFreq*t_i) + uav1.offsetZ
    # Uncomment to see setpoint changes in GUI --> Gives an error because update is too fast
    #pos_draw.queue_draw()
    #z_draw.queue_draw()
    
## This function updates the setpoint as recieved by the point cloud processing node
def update_ext_override(event):
    uav1.setpointX = uav1.ovRideSetpointMsg.x
    uav1.setpointY = uav1.ovRideSetpointMsg.y
    uav1.setpointZ = uav1.ovRideSetpointMsg.z
    uav1.setpointRoll = uav1.ovRideSetpointMsg.roll
    uav1.setpointPitch = uav1.ovRideSetpointMsg.pitch
    uav1.setpointYaw = uav1.ovRideSetpointMsg.yaw

def update_Insp_override(event):
    uav1.setpointX = uav1.InspRideSetpointMsg.x
    uav1.setpointY = uav1.InspRideSetpointMsg.y
    uav1.setpointZ = uav1.InspRideSetpointMsg.z
    uav1.setpointRoll = uav1.InspRideSetpointMsg.roll
    uav1.setpointPitch = uav1.InspRideSetpointMsg.pitch
    uav1.setpointYaw = uav1.InspRideSetpointMsg.yaw
    

## This function computes time elapsed since start of trajectory
def computeTimeElapsed():
    global traj_begin_time
    diff_time = rospy.Time.now() - traj_begin_time
    return diff_time.to_sec()

## Function executed when the Load Trajectory Button is pressed
def btnLoadTrajectory(button):
    root = Tkinter.Tk()
    root.withdraw()
    global destinationTraj
    global custom_traj_y
    global custom_traj_z

    # Load Files
    # TODO: Choose 1 file and then automatically load the second
    file_path_Y = tkFileDialog.askopenfilename(initialdir=destinationTraj,title='Select Y File')
    file_path_Z = file_path_Y[:-5] + 'z.txt'
    print file_path_Y
    print file_path_Z

    fileY = open(file_path_Y,"r")
    fileZ = open(file_path_Z,"r")

    # Load Pixel Arrays
    yy = fileY.read().split(',')
    zz = fileZ.read().split(',')
    custom_traj_y = []
    custom_traj_z = []
    for j in range(1, len(yy)-1):
        custom_traj_y.append(float(yy[j]))
        custom_traj_z.append(float(zz[j]))

    print "Length Y Array: ", len(custom_traj_y), "     Length Z Array: ", len(custom_traj_z)

## Function executed when the Apply Settings Button is pressed
def btnApplySettings(button):
    global widthscale, heightscale,timescale,custom_traj_y,custom_traj_z

    # Get User Entries
    if (Widthscale.get_text() == ""):
        widthscale = -0.4
    else:
        widthscale = float(Widthscale.get_text())/100.0*-1.0
    if (Heightscale.get_text() == ""):
        heightscale = -0.4
    else:
        heightscale = float(Heightscale.get_text())/100.0*-1.0
    if (Timescale.get_text() == ""):
        timescale = 1.0
    else:
        timescale = float(Timescale.get_text())
    print "Width scale: " , widthscale, "    Height scale is: ", heightscale, "   Time scale is", timescale

    # Convert Pixel Array to a width of $widthscale x $heightscale [m]x[m]
    minY = min(custom_traj_y)
    maxY = max(custom_traj_y)
    minZ = min(custom_traj_z)
    maxZ = max(custom_traj_z)
    #print minY, "  ",maxY

    # Set initial point in Trajectory to 0,0
    custom_traj_y[0] = (custom_traj_y[0] - minY)/(maxY-minY)*widthscale
    custom_traj_z[0] = (custom_traj_z[0] - minZ)/(maxZ-minZ)*heightscale
    traj_offset_y = custom_traj_y[0]
    traj_offset_z = custom_traj_z[0]
    custom_traj_y[0] -=traj_offset_y
    custom_traj_z[0] -=traj_offset_z
    # Convert rest of points
    for j in range(1, len(custom_traj_y)):
        custom_traj_y[j] = (custom_traj_y[j] - minY)/(maxY-minY)*widthscale - traj_offset_y
        custom_traj_z[j] = (custom_traj_z[j] - minZ)/(maxZ-minZ)*heightscale - traj_offset_z

############################## Create Traj ##################################
# Jornt Lagaveen's Work

##This function should run a POP-UP for the user to creating a trajectory
def btnCreateTrajectory(button):
    _init_screens()

def paint( event ):
   python_green = "#476042"
   x1, y1 = ( event.x - 5 ), ( event.y - 5 )
   x2, y2 = ( event.x + 5 ), ( event.y + 5 )
   w.create_oval( x1, y1, x2, y2, fill = python_green )
   global  array_y
   global  array_z
   #global  data

   array_y.extend([event.x])
   array_z.extend([event.y])
   #data.extend([array_y,array_z])

def save():
    input = e1.get()      #Inspiration: https://www.youtube.com/watch?v=FueIPFqRyyY
    print input
    print os.getcwd()
    fileName= input + "-trajectory-y.txt"
    global destinationTraj

    file = open(fileName,"w+")
    shutil.move(os.getcwd()+"/"+fileName,destinationTraj+fileName)
    file.write(str(array_y))   #Inspiration:  https://stackoverflow.com/questions/24439988/typeerror-expected-a-character-buffer-object/24440021
    file.close()

    fileName= input + "-trajectory-z.txt"
    file2 = open(fileName,"w+")
    shutil.move(os.getcwd()+"/"+fileName,destinationTraj+fileName)
    file2.write(str(array_z))
    file2.close()

def refresh():
        master.destroy()
        master2.destroy()
        _init_screens()

def stop():
    master.destroy()
    master2.destroy()


def _init_screens():
    global master2
    global master
    global w
    global e1
    print "Opening Popup"
    master = Tk()
    master.title( "Create your trajectory here" )

    w = Canvas(master,
           width=canvas_width,
           height=canvas_height)
    w.pack(expand = YES, fill = BOTH)
    w.bind( "<B1-Motion>", paint ) #For more info on binding, see:  http://effbot.org/tkinterbook/tkinter-events-and-bindings.htm

    message = Label( master, text = "Press and Drag the mouse to draw the desired trajectory" )
    message.pack( side = BOTTOM )

    ## Creating the new interface was done by means of this example: https://www.python-course.eu/tkinter_entry_widgets.php
    master2= Tk()
    master2.title( "Options Window" )
    Label(master2, text="File Name").grid(row=0)

    e1 = Entry(master2)
    e1.grid(row=0, column=1)

    Button(master2, text='Refresh', command=refresh).grid(row=3, column=0, sticky=W, pady=4)
    Button(master2, text='Save', command=lambda: save()).grid(row=3, column=1, sticky=W, pady=4)
    Button(master2, text='Quit', command=stop).grid(row=3, column=2, sticky=W, pady=4)
    print "Acquired Custom Trajectory"
    mainloop()
############################## End of Create Traj ##############################

## Function to Toggle End Effector Tracking Module
def btnTrackEndEffector(button):
    if button.get_active():
        TrackEE_Value.set_markup("<span foreground='green'><b>ON</b></span>")
        uav1.track_end_effector = MODE_ON
    else:
        TrackEE_Value.set_markup("<span foreground='red'><b>OFF</b></span>")
        uav1.track_end_effector = MODE_OFF

## Function to Toggle Disturbance Observer Module
def btnEnableDistObs(button):
    if button.get_active():
        DObs_Value.set_markup("<span foreground='green'><b>ON</b></span>")
        uav1.enable_dist_observer = MODE_ON
    else:
        DObs_Value.set_markup("<span foreground='red'><b>OFF</b></span>")
        uav1.enable_dist_observer = MODE_OFF

## Function to Toggle Disturbance Rejection Module
def btnRejectDisturbances(button):
    if button.get_active():
        DRej_Value.set_markup("<span foreground='green'><b>ON</b></span>")
        uav1.reject_disturbances = MODE_ON
    else:
        DRej_Value.set_markup("<span foreground='red'><b>OFF</b></span>")
        uav1.reject_disturbances = MODE_OFF

## Draws the position map.
def pos_draw_handler(widget, context):
    # Draw map
    global map_image_surface
    context.set_source_surface(map_image_surface)
    context.paint()

    # Draw UAV
    pos_x_px, pos_y_px = meterToPixel(uav1.currentPose.pose.position.x, uav1.currentPose.pose.position.y)
    draw_image(context, uav1.image_surface, pos_x_px, pos_y_px, uav1.length, uav1.width, uav1.currentYaw/360*2*math.pi)

    # Draw setpoint
    setpointX_px, setpointY_px = meterToPixel(uav1.setpointX, uav1.setpointY)
    draw_setpoint(context, setpointX_px, setpointY_px)

    # Draw the current xy and setpoint xy in the corner
    currentXYText = "({: .2f},{: .2f})".format(uav1.currentPose.pose.position.x, uav1.currentPose.pose.position.y)
    setpointXYText = "({: .2f},{: .2f})".format(uav1.setpointX, uav1.setpointY)
    context.set_line_width(2)
    context.select_font_face("Courier", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
    context.set_font_size(14)
    context.set_source_rgba(0, 0, 0, 1)
    context.move_to(55,20)
    context.show_text(currentXYText)
    context.set_source_rgba(1.0, 0.0, 0.0, 1.0)
    context.move_to(55,35)
    context.show_text(setpointXYText)

## Draws the yaw bar.
def yaw_draw_handler(widget, context):
    # Settings
    context.set_line_width(2)
    context.select_font_face("Courier", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
    context.set_font_size(14)
    context.set_source_rgba(0, 0, 0, 1)

    # Draw the rectangle box
    context.rectangle(50,30,400,15)
    context.stroke

    # Draw a line for the origin
    context.move_to(250,30)
    context.line_to(250,25)
    context.stroke()

    # Draw the text "Yaw control"
    (x, y, width, height, dx, dy) = context.text_extents("Yaw")
    context.move_to(5,35)
    context.show_text("Yaw")
    #context.move_to(0,30)
    #context.show_text("control")

    # Draw the current yaw value with a black line at the correct position
    currentYawText = "{:.1f}".format(uav1.currentYaw)
    (x, y, width, height, dx, dy) = context.text_extents(currentYawText)

    context.move_to(250 - uav1.currentYaw/360.0*400-width/2,height)
    context.show_text(currentYawText)
    context.move_to(250 - uav1.currentYaw/360.0*400,30)
    context.line_to(250 - uav1.currentYaw/360.0*400,45)
    context.stroke()

    # Draw the setpoint yaw value with a red line at the correct position
    context.set_source_rgba(1.0, 0.0, 0.0, 1.0)

    setpointYawText = "{:.1f}".format(uav1.setpointYaw)
    (x, y, width, height, dx, dy) = context.text_extents(setpointYawText)
    context.move_to(250 - uav1.setpointYaw/360*400-width/2,15 + height)
    context.show_text(setpointYawText)
    context.move_to(250 - uav1.setpointYaw/360*400,30)
    context.line_to(250 - uav1.setpointYaw/360*400,45)
    context.stroke()

## Draws the roll bar.
def roll_draw_handler(widget, context):
    # Settings
    context.set_line_width(2)
    context.select_font_face("Courier", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
    context.set_font_size(14)
    context.set_source_rgba(0, 0, 0, 1)

    # Draw the rectangle box
    context.rectangle(50,30,400,15)
    context.stroke

    # Draw a line for the origin
    context.move_to(250,30)
    context.line_to(250,25)
    context.stroke()

    # Draw the text "Roll control"
    (x, y, width, height, dx, dy) = context.text_extents("Roll")
    context.move_to(5,35)
    context.show_text("Roll")
    #context.move_to(0,30)
    #context.show_text("control")

    # Draw the current roll value with a black line at the correct position
    currentRollText = "{:.1f}".format(uav1.currentRoll)
    (x, y, width, height, dx, dy) = context.text_extents(currentRollText)

    context.move_to(250 - uav1.currentRoll/360.0*400-width/2,height)
    context.show_text(currentRollText)
    context.move_to(250 - uav1.currentRoll/360.0*400,30)
    context.line_to(250 - uav1.currentRoll/360.0*400,45)
    context.stroke()

    # Draw the setpoint roll value with a red line at the correct position
    context.set_source_rgba(1.0, 0.0, 0.0, 1.0)

    setpointRollText = "{:.1f}".format(uav1.setpointRoll)
    (x, y, width, height, dx, dy) = context.text_extents(setpointRollText)
    context.move_to(250 - uav1.setpointRoll/360*400-width/2,15 + height)
    context.show_text(setpointRollText)
    context.move_to(250 - uav1.setpointRoll/360*400,30)
    context.line_to(250 - uav1.setpointRoll/360*400,45)
    context.stroke()

## Draws the pitch bar.
def pitch_draw_handler(widget, context):
    # Settings
    context.set_line_width(2)
    context.select_font_face("Courier", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
    context.set_font_size(14)
    context.set_source_rgba(0, 0, 0, 1)

    # Draw the rectangle box
    context.rectangle(50,30,400,15)
    context.stroke

    # Draw a line for the origin
    context.move_to(250,30)
    context.line_to(250,25)
    context.stroke()

    # Draw the text "Pitch control"
    (x, y, width, height, dx, dy) = context.text_extents("Pitch")
    context.move_to(5,35)
    context.show_text("Pitch")
    #context.move_to(0,30)
    #context.show_text("control")

    # Draw the current pitch value with a black line at the correct position
    currentPitchText = "{:.1f}".format(uav1.currentPitch)
    (x, y, width, height, dx, dy) = context.text_extents(currentPitchText)

    context.move_to(250 - uav1.currentPitch/360.0*400-width/2,height)
    context.show_text(currentPitchText)
    context.move_to(250 - uav1.currentPitch/360.0*400,30)
    context.line_to(250 - uav1.currentPitch/360.0*400,45)
    context.stroke()

    # Draw the setpoint pitch value with a red line at the correct position
    context.set_source_rgba(1.0, 0.0, 0.0, 1.0)

    setpointPitchText = "{:.1f}".format(uav1.setpointPitch)
    (x, y, width, height, dx, dy) = context.text_extents(setpointPitchText)
    context.move_to(250 - uav1.setpointPitch/360*400-width/2,15 + height)
    context.show_text(setpointPitchText)
    context.move_to(250 - uav1.setpointPitch/360*400,30)
    context.line_to(250 - uav1.setpointPitch/360*400,45)
    context.stroke()


## Draws the z bar
def z_draw_handler(widget, context):
    # Settings
    context.set_line_width(2)
    context.select_font_face("Courier", cairo.FONT_SLANT_NORMAL, cairo.FONT_WEIGHT_BOLD)
    context.set_font_size(14)
    context.set_source_rgba(0, 0, 0, 1)

    # Draw the rectangle box
    context.rectangle(80,50,15,400)
    context.stroke()

    # Draw a line for the origin
    context.move_to(80,350)
    context.line_to(75,350)
    context.stroke()

    # Draw the text "Z control"
    (x, y, width, height, dx, dy) = context.text_extents("Z control")
    context.move_to(50 - width/2,25 + height/2)
    context.show_text("Z control")

    # Draw the current z value with a black line at the correct position
    currentZText = "{:.2f}".format(uav1.currentPose.pose.position.z)
    (x, y, width, height, dx, dy) = context.text_extents(currentZText)

    context.move_to(0, 350 - uav1.currentPose.pose.position.z/4*400 + height/2)
    context.show_text(currentZText)
    context.move_to(80, 350 - uav1.currentPose.pose.position.z/4*400)
    context.line_to(95, 350 - uav1.currentPose.pose.position.z/4*400)
    context.stroke()

    # Draw the setpoint z value with a red line at the correct position
    setpointZText = "{:.2f}".format(uav1.setpointZ)
    (x, y, width, height, dx, dy) = context.text_extents(setpointZText)
    context.set_source_rgba(1, 0, 0, 1)

    context.move_to(40, 350 - uav1.setpointZ/4*400 + height/2)
    context.show_text(setpointZText)
    context.move_to(80, 350 - uav1.setpointZ/4*400)
    context.line_to(95, 350 - uav1.setpointZ/4*400)
    context.stroke()

## Draws the setpoint as a red dot
def draw_setpoint(context, centerX, centerY):
    context.save()
    context.set_source_rgba(1.0, 0, 0, 1)
    context.arc(centerX, centerY, 4, 0, 2*math.pi)
    context.fill()
    context.restore()

## Helper function to translate a cairo context
def pre_translate(ctx, tx, ty):
    """Translate a cairo context without taking into account its
    scale and rotation"""
    mat = ctx.get_matrix()
    ctx.set_matrix(cairo.Matrix(mat[0],mat[1],
                                mat[2],mat[3],
                                mat[4]+tx,mat[5]+ty))

## Helper function to draw an image on a context at a given place and angle.
def draw_image(ctx, image_surface, centerX, centerY, height, width, angle=0):
    """Draw a scaled image on a given context."""
    # image_surface = uav1.image_surface #cairo.ImageSurface.create_from_png(image)

    # calculate proportional scaling
    img_height,img_width = (image_surface.get_height(),
                            image_surface.get_height())
    scale_xy = min(1.0*width/img_width,1.0*height / img_height)

    # scale, translate, and rotate the image around its center.
    ctx.save()
    ctx.rotate(-angle + math.pi/2)
    ctx.translate(-img_width/2*scale_xy,-img_height/2*scale_xy)

    ctx.scale(scale_xy, scale_xy)
    pre_translate(ctx, centerX, centerY)
    ctx.set_source_surface(image_surface)

    ctx.paint()
    ctx.restore()

## Convert a metric position to a pixel location.
def meterToPixel(x_meter, y_meter):
    global origin_x, origin_y
    x_px = origin_x + x_meter*100
    y_px = origin_y - y_meter*100
    return x_px, y_px

## Convert a pixel location to a metric position.
def pixelToMeter(x_px, y_px):
    global origin_x, origin_y
    x_meter = (x_px - origin_x)/100
    y_meter = -(y_px - origin_y)/100
    return x_meter, y_meter


## Check if the input is a number.
def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


# Main function
if __name__ == "__main__":

    # Check for the presence of a roscore by checking the ROS_MASTER_URI
    m = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    try:
        code, msg, val = m.getSystemState('controller_instance')
    except socket.error:
        print "Please start roscore first."
        sys.exit()

    # Initialize node
    rospy.init_node('setpoint_gui')


    # Load UAV type size from parameters
    uav_type = rospy.get_param('~uav_type', 'hexacopter')
    uav_size = rospy.get_param('~uav_size', 90)


    # Load frame names for setpoint and visualization
    SetpointChildFrameId = rospy.get_param('~SetpointChildFrameId','setpoint')
    SetpointFrameId = rospy.get_param('~SetpointFrameId','world')
    LocalPosFrameId = rospy.get_param('~LocalPosFrameId', 'world')
    LocalPosChildFrameId = rospy.get_param('~LocalPosChildFrameId', 'state')

    # Threaded application, we want to be able to update the interface from this application directly
    GObject.threads_init()
    Gdk.threads_init()

    # Get glade file for the interface
    rospack = rospkg.RosPack()
    package_location = rospack.get_path("spc_uav_comm")
    gladefile = package_location+"/scripts/simulation_omni_gui.glade"
    builder = Gtk.Builder()
    builder.add_from_file(gladefile)

    # Initialize map as ramlab map
    global image_path, map_image_surface, origin_x, origin_y, flightMap
    flightMap = 'ramlab'
    image_path = package_location+"/images/"

    map_image_surface = cairo.ImageSurface.create_from_png(image_path+flightMap+'.png')
    origin_x = 265.0
    origin_y = 299.0

    # Create an instance of the UAV class
    uav1 = UAV(uav_type, uav_size, uav_size)

    # Call this function every x ms. Functions have to return True to keep on going.
    GObject.timeout_add(20,uav1.setSetpoint)
    GObject.timeout_add(100,uav1.getLocalPos)

    # SPECTORS UPDATE: Create a timer for trajectory updates
    rospy.Timer(rospy.Duration(0.05), update_trajectory_setpoint)
    rospy.Timer(rospy.Duration(0.1), update_uav_mavros_state)
    global destinationTraj
    destinationTraj = package_location + "/scripts/custom_trajectories/"


    # Connect all buttons from the Glade interface
    handlers = {
        "quit": btnClose,
        "btnPressPos": btnPressPos,
        "btnPressYaw": btnPressYaw,
        "btnPressRoll": btnPressRoll,
        "btnPressPitch": btnPressPitch,
        "btnPressZ": btnPressZ,
        "window_key": window_key,
        "btnGetCurrentPosition": btnGetCurrentPosition,
        "pos_draw_handler": pos_draw_handler,
        "yaw_draw_handler": yaw_draw_handler,
        "roll_draw_handler": roll_draw_handler,
        "pitch_draw_handler": pitch_draw_handler,
        "z_draw_handler": z_draw_handler,
        "btnSetSetpoint": btnSetSetpoint,
	    "btnApplySettings": btnApplySettings,
        "btnPopulate": btnPopulate,
        "btnCreateTrajectory": btnCreateTrajectory,
        "btnLoadTrajectory": btnLoadTrajectory,
        "modeButtonClicked": modeButtonClicked,
        "flightmapBtnClicked": flightmapBtnClicked,
        "offboardToggled" : offboardToggled,
        "btnTrackEndEffector" : btnTrackEndEffector,
        "btnEnableDistObs" : btnEnableDistObs,
        "btnRejectDisturbances" : btnRejectDisturbances
    }
    builder.connect_signals(handlers)

    # Get window and set title, show window
    window = builder.get_object("window1")
    window.set_title("ARW Control Interface")
    window.set_resizable(True)


    pos_draw = builder.get_object("pos_draw")
    yaw_draw = builder.get_object("yaw_draw")
    roll_draw = builder.get_object("roll_draw")
    pitch_draw = builder.get_object("pitch_draw")
    z_draw = builder.get_object("z_draw")

    circleRadius = builder.get_object("circleRadius")
    circleFreq = builder.get_object("circleFreq")
    climbRate = builder.get_object("climbRate")

    Widthscale = builder.get_object("Widthscale")
    Heightscale = builder.get_object("Heightscale")
    Timescale = builder.get_object("Timescale")

    M1_value = builder.get_object("M1_value")
    M2_value = builder.get_object("M2_value")
    M3_value = builder.get_object("M3_value")
    M4_value = builder.get_object("M4_value")
    M5_value = builder.get_object("M5_value")
    M6_value = builder.get_object("M6_value")

    Connected_Value = builder.get_object("Connected_Value")
    Armed_Value = builder.get_object("Armed_Value")
    Guided_Value = builder.get_object("Guided_Value")
    Mode_Value = builder.get_object("Mode_Value")

    TrackEE_Value = builder.get_object("TrackEE_Value")
    DRej_Value = builder.get_object("DRej_Value")
    DObs_Value = builder.get_object("DObs_Value")


    Xtextbox = builder.get_object("Xtextbox")
    Ytextbox = builder.get_object("Ytextbox")
    Ztextbox = builder.get_object("Ztextbox")
    Yawtextbox = builder.get_object("Yawtextbox")

    # Enable pressing buttons in drawing area. Not sure if this does anything.
    pos_draw.set_events(Gdk.EventMask.BUTTON_PRESS_MASK)
    yaw_draw.set_events(Gdk.EventMask.BUTTON_PRESS_MASK)
    roll_draw.set_events(Gdk.EventMask.BUTTON_PRESS_MASK)
    pitch_draw.set_events(Gdk.EventMask.BUTTON_PRESS_MASK)
    z_draw.set_events(Gdk.EventMask.BUTTON_PRESS_MASK)

    window.show_all()

    # These lines make the process stop when CTRL+C is pressed
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Run main loop
    Gdk.threads_enter()
    Gtk.main()
    Gdk.threads_leave()
