#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <tf/transform_broadcaster.h>

#include "robot_wheel_speeds/DifferentialDriveEncoders.h"
#include "robot_wheel_speeds/SystemGpio.h"


int velocityUpdateIntervalNs, leftPinA, leftPinB, rightPinA, rightPinB, ticksPerRevolution, wheelDiameterMm, wheelAxisMm, rate;
std::string nodeBaseName = "ros_wheel_speeds", wheel_speeds_topic = "wheel_speeds", lwheel_topic = "lwheel", rwheel_topic = "rwheel", velocitiy_frame = "odom";
bool publish_velocities = true, publish_rwheel = true, publish_lwheel = true;

void getParams()
{
    ros::param::get("/wheel_speeds/velocityUpdateIntervalNs", velocityUpdateIntervalNs);
    ros::param::get("/wheel_speeds/leftPinA", leftPinA);
    ros::param::get("/wheel_speeds/leftPinB", leftPinB);
    ros::param::get("/wheel_speeds/rightPinA", rightPinA);
    ros::param::get("/wheel_speeds/rightPinB", rightPinB);
    ros::param::get("/wheel_speeds/ticksPerRevolution", ticksPerRevolution);
    ros::param::get("/wheel_speeds/wheelDiameterMm", wheelDiameterMm);
    ros::param::get("/wheel_speeds/wheelAxisMm", wheelAxisMm);
    ros::param::get("/wheel_speeds/rate", rate);
    ros::param::get("/wheel_speeds/wheel_speeds_topic", wheel_speeds_topic);
    ros::param::get("/wheel_speeds/lwheel_topic", lwheel_topic);
    ros::param::get("/wheel_speeds/rwheel_topic", rwheel_topic);
    ros::param::get("/wheel_speeds/publish_velocities", publish_velocities);
    ros::param::get("/wheel_speeds/publish_rwheel", publish_rwheel);
    ros::param::get("/wheel_speeds/publish_lwheel", publish_lwheel);
    ros::param::get("/wheel_speeds/velocitiy_frame", velocitiy_frame);

    ROS_INFO("Read Parameters");
    ROS_INFO(" velocityUpdateIntervalNs: %.0f", velocityUpdateIntervalNs*1.0f);
    ROS_INFO(" leftPinA: %.0f", leftPinA*1.0f);
    ROS_INFO(" leftPinB: %.0f", leftPinB*1.0f);
    ROS_INFO(" rightPinA: %.0f", rightPinA*1.0f);
    ROS_INFO(" rightPinB: %.0f", rightPinB*1.0f);
    ROS_INFO(" ticksPerRevolution: %.0f", ticksPerRevolution*1.0f);
    ROS_INFO(" wheelDiameterMm: %.0f", wheelDiameterMm*1.0f);
    ROS_INFO(" wheelAxisMm: %.0f", wheelAxisMm*1.0f);
    ROS_INFO(" rate: %.0f", rate*1.0f);
    ROS_INFO(" wheel_speeds_topic: %s", wheel_speeds_topic.c_str());
    ROS_INFO(" lwheel_topic: %s", lwheel_topic.c_str());
    ROS_INFO(" rwheel_topic: %s", rwheel_topic.c_str());
    ROS_INFO(" publish_velocities: %s", publish_velocities? ("true") : ("false"));
    ROS_INFO(" publish_rwheel: %s", publish_rwheel? ("true") : ("false"));
    ROS_INFO(" publish_lwheel: %s", publish_lwheel? ("true") : ("false"));
    ROS_INFO(" velocitiy_frame: %s", velocitiy_frame.c_str());
}

int main(int argc, char *argv[])
{
    std_msgs::Int16 lwheel_ticks;
    std_msgs::Int16 rwheel_ticks;

    ros::init(argc, argv, "robot_wheel_speeds");

    // Get parameters from parameter server
    getParams();
    ros::Duration velocityUpdateInterval(0, velocityUpdateIntervalNs);

    SystemGPIO gpio({leftPinA, leftPinB, rightPinA, rightPinB});

    ros::NodeHandle nodeHandle;

    auto publisher = nodeHandle.advertise<robot_wheel_speeds::WheelVelocities>(wheel_speeds_topic, 10);
    ros::Publisher lwheel_pub = nodeHandle.advertise<std_msgs::Int16>(lwheel_topic, 10);
    ros::Publisher rwheel_pub = nodeHandle.advertise<std_msgs::Int16>(rwheel_topic, 10);

    ros::Rate loop_rate(rate);

    ROS_INFO("Starting wheel_speeds");
    DifferentialDriveEncoders encoders(leftPinA, leftPinB, rightPinA, rightPinB,
                                       ticksPerRevolution, wheelDiameterMm, wheelAxisMm,
                                       velocityUpdateInterval);

    encoders.Start();
    ROS_INFO("Encoders started");
    int seq = 1;


    while(ros::ok())
    {
        auto msg = encoders.GetVelocities();
        msg.header.frame_id = velocitiy_frame;
        msg.header.seq = seq++;

        // publish topics
        if (publish_velocities) {
          publisher.publish(msg);
        }

        if (publish_lwheel) {
          lwheel_ticks.data = msg.left_ticks;
          lwheel_pub.publish(lwheel_ticks);
        }
        if (publish_rwheel) {
          rwheel_ticks.data = msg.right_ticks;
          rwheel_pub.publish(rwheel_ticks);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
