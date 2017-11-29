#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
import math
from twist_controller import Controller
from styx_msgs.msg import Lane

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.
You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.
One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.
We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.
We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.
Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.
'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')
        
	class Paramater_State():
		vehicle_mass = None
        	fuel_capacity = None
        	brake_deadband = None
        	decel_limit = None
        	accel_limit = None
        	wheel_radius = None
        	wheel_base = None
        	steer_ratio = None
        	max_lat_accel = None
        	max_steer_angle = None

        #Paramater States:
        S = Paramater_State()

        S.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        S.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        S.brake_deadband = rospy.get_param('~brake_deadband', .1)
        S.decel_limit = rospy.get_param('~decel_limit', -5)
        S.accel_limit = rospy.get_param('~accel_limit', 1.)
        S.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        S.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        S.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        S.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        S.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `TwistController` object
        self.dbw_enabled = True
        self.initial_time = rospy.get_time()
        self.pose = None
        self.twist_cmd = None
        self.reset = True
        self.waypoints = None
        self.current_velocity = None
        self.controller = Controller(S)
        
            
        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_callback, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_callback, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_callback, queue_size=1)
        self.loop()    
        

    def loop(self):
        #rate = rospy.Rate(50) # 50Hz
        rate = rospy.Rate(150) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            time = rospy.get_time()
            delta_t = time - self.initial_time
            
            # Check time stamps.
            if time == self.initial_time:
                continue

            # Chck for divide by zero.
            #FIXME: Commenting for now, check why this is erroing delta_t = (time.nsec - self.initial_time.nsec) * 1e-6
            #delta_t = (time - self.initial_time) * 1e-9  # Isn't nsec 9 zeros
            delta_t = (time - self.initial_time) * 1e-2  # Isn't nsec 9 zeros

            # Make sure there are enough waypoints.
            waypoints_in = self.waypoints is not None
            if not waypoints_in:
                continue
                
            #continue driving if less than 3 waypoints.
            #FIXME: -- commented the next 2 lines for now.
            ###if(len(self.waypoints) < 3):
            ###    continue
                
            
            #Enable manual override and manual stop.
            #Reset first
            if self.dbw_enabled and self.twist_cmd is not None and self.current_velocity is not None:
                if self.reset:
                    self.controller.reset()
                    self.reset = False

                #Commands from twist controlller
                throttle, brake, steering = self.controller.control(self.twist_cmd, self.current_velocity, delta_t, self.pose, self.waypoints)
                
                rospy.loginfo("dbw_node: throttle: %f brake: %f steering: %f", throttle, brake, steering) 
            else:
                self.reset = True
                throttle = 0.0
                brake = 0.0
                steering = 0.0
                rospy.loginfo("dbw_node: reset") 
            
            self.publish(throttle, brake, steering)
            
            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)
        
    def  twist_cmd_callback(self, msg):
        #self.twist_cmd = msg.twist
        self.twist_cmd = msg

    #Current_velocity callback comes from twist.
    def current_velocity_callback(self, msg):
        #self.current_velocity = msg.velocity
        self.current_velocity = msg
            
    #Pose callback comes from messages in pose.
    def pose_callback(self, msg):
	#self.pose = msg.pose
	self.pose = msg
            
    #DBW info comes in from data.
    def dbw_enabled_callback(self, msg):
        #self.dbw_enabled = msg.data
        self.dbw_enabled = msg
    
    #Populate waypoints
    def waypoints_callback(self, msg):
        #self.waypoints = msg.waypoints
        self.waypoints = msg


if __name__ == '__main__':
    DBWNode()


# In[ ]:



