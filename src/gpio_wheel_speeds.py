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

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
    def update(self):

        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            # Set the GPIO modes
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            
            # Set the GPIO Pin mode to be Input
            GPIO.setup(self.leftPinA, GPIO.IN)
            GPIO.setup(self.leftPinB, GPIO.IN)
            GPIO.setup(self.rightPinA, GPIO.IN)
            GPIO.setup(self.rightPinB, GPIO.IN)

            # read shaft encoders from wheels
            prev_lpinA = GPIO.LOW
            prev_lpinB = GPIO.LOW
            prev_rpinA = GPIO.LOW
            prev_rpinB = GPIO.LOW                                    
            try:
                While True:
                    lValueA = GPIO.input(self.leftPinA) 
                    lValueB = GPIO.input(self.leftPinB) 
                    if prev_lpinA == GPIO.LOW and lValueA == GPIO.HIGH // rising edge
                        self.lticks += 1
                        if lValueB == GPIO.HIGH:
                            self.ldirection = 1
                        else:
                            self.ldirection = -1
                        self.lelapsed_sticks += int(self.lticks) * self.ldirection
                        prev_lpinA = lValueA

                    rValueA = GPIO.input(self.rightPinA) 
                    rValueB = GPIO.input(self.rigthPinB) 
                    if prev_rpinA == GPIO.LOW and rValueA == GPIO.HIGH // rising edge
                        self.rticks += 1
                        if rValueB == GPIO.HIGH:
                            self.rdirection = 1
                        else:
                            self.rdirection = -1
                        self.relapsed_sticks += int(self.rticks) * self.rdirection    
                        prev_rpinA = rValueA
            finally:
                GPIO.cleanup()                       

            self.lwheelPub.publish(self.lelapsed_sticks)
            self.rwheelPub.publish(self.relapsed_sticks)
            self.lticks = 0
            self.rticks = 0
            # self.lelapsed_sticks = 0
            # self.relapsed_sticks = 0
            

if __name__ == '__main__':
    """ main """
    try:
        wheelEncoders = WheelEncoders()
        wheelEncoders.spin()
    except rospy.ROSInterruptException:
        GPIO.cleanup()