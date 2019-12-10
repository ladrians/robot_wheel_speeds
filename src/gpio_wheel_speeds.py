#!/usr/bin/env python

"""
   gpio_wheel_speeds.py - reads GPIO pins connected to shaft encoders of left and right weels.
"""

import rospy
import yaml
from std_msgs.msg import Int32
import Jetson.GPIO as GPIO

class WheelEncoders:

    def __init__(self):
        rospy.init_node("wheel_encoders")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)

        #### parameters #######
        self.leftPinA = rospy.get_param('leftPinA',6)
        self.leftPinB = rospy.get_param('leftPinB',5)
        self.rightPinA = rospy.get_param('rightPinA',13)
        self.rightPinB = rospy.get_param('rightPinB',19)

        self.ticksPerRevolution = float(rospy.get_param('ticksPerRevolution', 3000))
        self.wheelDiameterMm = float(rospy.get_param('wheelDiameterMm', 120))
        self.wheelAxisMm = rospy.get_param('wheelAxisMm',210)
        self.rate = rospy.get_param('rate', 15)
        
        self.wheel_speeds_topic = rospy.get_param('wheel_speeds_topic', 'wheel_speeds')
        self.lwheel_topic = rospy.get_param('lwheel_topic', 'lwheel')
        self.rwheel_topic = rospy.get_param('rwheel_topic', 'rwheel')
        
        self.publish_velocities = rospy.get_param('publish_velocities', False)
        self.publish_rwheel = rospy.get_param('publish_rwheel', True)
        self.publish_lwheel = rospy.get_param('publish_lwheel', True)

        self.velocitiy_frame = rospy.get_param('velocitiy_frame', 'odom')
        self.velocityUpdateIntervalNs = rospy.get_param('velocityUpdateIntervalNs', 20000000)

        self.lticks = 0
        self.rticks = 0
        self.ldirection = 0
        self.rdirection = 0
        self.lelapsed_sticks = 0
        self.relapsed_sticks = 0

        self.then = rospy.Time.now()
		
        # Shaft encoders from wheels
        self.prev_lpinA = GPIO.LOW
        self.prev_lpinB = GPIO.LOW
        self.prev_rpinA = GPIO.LOW
        self.prev_rpinB = GPIO.LOW

        self.lValueA = None
        self.lValueB = None
        self.rValueA = None
        self.rValueB = None

        # display parameters data
        self.display_params()
        
        # Subscriptions


        # Publications
        self.lwheelPub = rospy.Publisher(self.lwheel_topic, Int32, queue_size=10)
        self.rwheelPub = rospy.Publisher(self.rwheel_topic, Int32, queue_size=10)


    def display_params(self):
        rospy.loginfo(" leftPinA: %s" % self.leftPinA)
        rospy.loginfo(" leftPinB: %s" % self.leftPinB)
        rospy.loginfo(" rightPinA: %s" % self.rightPinA)
        rospy.loginfo(" rightPinB: %s" % self.rightPinB)
        rospy.loginfo(" ticksPerRevolution: %s" % self.ticksPerRevolution)
        rospy.loginfo(" wheelDiameterMm: %s" % self.wheelDiameterMm)
        rospy.loginfo(" wheelAxisMm: %s" % self.wheelAxisMm)
        rospy.loginfo(" rate: %s" % self.rate)
        rospy.loginfo(" wheel_speeds_topic: %s" % self.wheel_speeds_topic)        
        rospy.loginfo(" rwheel_topic: %s" % self.rwheel_topic)
        rospy.loginfo(" lwheel_topic: %s" % self.lwheel_topic)
        rospy.loginfo(" publish_velocities: %s" % self.publish_velocities)
        rospy.loginfo(" publish_rwheel: %s" % self.publish_rwheel)
        rospy.loginfo(" publish_lwheel: %s" % self.publish_lwheel)
        rospy.loginfo(" velocitiy_frame: %s" % self.velocitiy_frame)
        rospy.loginfo(" velocityUpdateIntervalNs: %s" % self.velocityUpdateIntervalNs)
        
    def spin(self):

        # Set the GPIO modes
        GPIO.setmode(GPIO.BCM) # BOARD 
        GPIO.setwarnings(False)

        # Set the GPIO Pin mode to be Input
        GPIO.setup(self.leftPinA, GPIO.IN)
        GPIO.setup(self.leftPinB, GPIO.IN)
        GPIO.setup(self.rightPinA, GPIO.IN)
        GPIO.setup(self.rightPinB, GPIO.IN)                                  

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.lticks = 0
            self.rticks = 0            
            self.update()
            r.sleep()
       
    def update(self):

        now = rospy.Time.now()
        if True:#(now > self.t_next):
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            try:
				self.lValueA = GPIO.input(self.leftPinA) 
				self.lValueB = GPIO.input(self.leftPinB)
				if (self.lValueA == GPIO.HIGH):
					if (self.lValueB == GPIO.LOW):
                        self.lticks += 1
                    else:
                        self.lticks -= 1
                else:
                    if (self.lValueB == GPIO.LOW):
                        self.lticks -= 1
                    else:
                        self.lticks += 1

				self.rValueA = GPIO.input(self.rightPinA) 
				self.rValueB = GPIO.input(self.rightPinB) 
				if (self.rValueA == GPIO.HIGH):
					if (self.rValueB == GPIO.LOW):
                        self.rticks += 1
                    else:
                        self.rticks -= 1
                else:
                    if (self.rValueB == GPIO.LOW):
                        self.rticks -= 1
                    else:
                        self.rticks += 1
            except:
                GPIO.cleanup()                       

            print(self.lticks,self.rticks,self.lelapsed_sticks,self.relapsed_sticks)
            self.lwheelPub.publish(self.lsticks)
            self.rwheelPub.publish(self.rsticks)
           

if __name__ == '__main__':
    """ main """
    try:
        wheelEncoders = WheelEncoders()
        wheelEncoders.spin()
    except rospy.ROSInterruptException:
        GPIO.cleanup()