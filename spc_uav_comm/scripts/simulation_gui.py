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
import xmlrpclib
import socket
import subprocess
import math
import cairo

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
#import mavros_msgs.msg

# Possible flight modes
FREEFLIGHTMODE = 0.0
APPROACHMODE = 1.0
CONTACTMODE = 2.0


class UAV:
    def __init__(self, uav_type, length, width):
        self.getCurrentPosition = False

        self.setpointX = -3.0
        self.setpointY = 0.0
        self.setpointZ = 0.0
        self.setpointYaw = 0.0 # Degrees

        self.length = length
        self.width = width

        self.currentPose = geometry_msgs.msg.PoseStamped()
        self.currentYaw = 0.0 # Degrees

        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.tfbuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfbuffer)

        self.userNotified = False

        self.flightMode = FREEFLIGHTMODE
        self.mavrosState = mavros_msgs.msg.State()
        
        self.pubSetpoint = rospy.Publisher("setpoint", geometry_msgs.msg.TransformStamped, queue_size=1)
        self.pubImpactSetpoint = rospy.Publisher("impactSetpoint", std_msgs.msg.Float32MultiArray, queue_size=1)      
        self.pubOffboard = rospy.Publisher("offboardCommand", std_msgs.msg.Bool,queue_size=1)
        self.pubArmCommand = rospy.Publisher("armCommand", std_msgs.msg.Bool,queue_size=1)
        
        self.subFlightMode = rospy.Subscriber("flightMode", std_msgs.msg.Float32, self.flightModeCB)
        self.subMavrosState = rospy.Subscriber("mavros/state", mavros_msgs.msg.State, self.mavrosStateCB)

        


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
        print msg
        
      

    ## The flight mode can change externally. This function updates the GUI when such a change is advertised.
    def flightModeCB(self, msg):
        self.flightMode = msg.data
        Gdk.threads_enter()   
        if self.flightMode == FREEFLIGHTMODE:
            builder.get_object("FreeFlightMode").set_active(True)
        if self.flightMode == APPROACHMODE:
            builder.get_object("ApproachMode").set_active(True)
        if self.flightMode == CONTACTMODE:
            builder.get_object("ContactMode").set_active(True)
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

        quaternion = tf.transformations.quaternion_from_euler(0, 0, (self.setpointYaw*2*math.pi)/360)
        setpointMsg.transform.rotation.x = quaternion[0]
        setpointMsg.transform.rotation.y = quaternion[1]
        setpointMsg.transform.rotation.z = quaternion[2]
        setpointMsg.transform.rotation.w = quaternion[3]

        self.broadcaster.sendTransform(setpointMsg)

        self.pubSetpoint.publish(setpointMsg)

        impactSetpointMsg = std_msgs.msg.Float32MultiArray()
        impactSetpointMsg.data = [self.flightMode, builder.get_object("ContactForceValue").get_value(), builder.get_object("PitchValue").get_value()]
        self.pubImpactSetpoint.publish(impactSetpointMsg)

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

                ## Update the GUI. Enter and leave threads is necessary to ensure that this is the only function at the moment to change the GUI. 
                Gdk.threads_enter()    
                pos_draw.queue_draw()
                yaw_draw.queue_draw()
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
        if button.get_label() == "Approach":
            uav1.flightMode = APPROACHMODE
        if button.get_label() == "Contact":
            uav1.flightMode = CONTACTMODE    

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
    context.move_to(0,15)
    context.show_text("Yaw")
    context.move_to(0,30)
    context.show_text("control")

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

    # Draw the setpoint yaw value with a red line at the correct position
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
    gladefile = package_location+"/scripts/simulation_gui.glade"
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

    # Connect all buttons from the Glade interface
    handlers = {
        "quit": btnClose,
        "btnPressPos": btnPressPos,
        "btnPressYaw": btnPressYaw,
        "btnPressZ": btnPressZ,
        "window_key": window_key,
        "btnGetCurrentPosition": btnGetCurrentPosition,
        "pos_draw_handler": pos_draw_handler,
        "yaw_draw_handler": yaw_draw_handler,
        "z_draw_handler": z_draw_handler,
        "btnSetSetpoint": btnSetSetpoint,
        "btnPopulate": btnPopulate,
        "modeButtonClicked": modeButtonClicked,
        "flightmapBtnClicked": flightmapBtnClicked,
        "offboardToggled" : offboardToggled
    }
    builder.connect_signals(handlers)

    # Get window and set title, show window
    window = builder.get_object("window1")
    window.set_title("ARW Control Interface")
    window.set_resizable(True)
    

    pos_draw = builder.get_object("pos_draw")
    yaw_draw = builder.get_object("yaw_draw")
    z_draw = builder.get_object("z_draw")

    Xtextbox = builder.get_object("Xtextbox")
    Ytextbox = builder.get_object("Ytextbox")
    Ztextbox = builder.get_object("Ztextbox")
    Yawtextbox = builder.get_object("Yawtextbox")

    # Enable pressing buttons in drawing area. Not sure if this does anything.
    pos_draw.set_events(Gdk.EventMask.BUTTON_PRESS_MASK)
    yaw_draw.set_events(Gdk.EventMask.BUTTON_PRESS_MASK)
    z_draw.set_events(Gdk.EventMask.BUTTON_PRESS_MASK)
 
    window.show_all()

    # These lines make the process stop when CTRL+C is pressed
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Run main loop
    Gdk.threads_enter()
    Gtk.main()
    Gdk.threads_leave()
