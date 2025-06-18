#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>

#define SAFETY_DISTANCE 0.3
#define SLOW_DISTANCE 0.5

geometry_msgs::Twist modified_cmd;
ros::Publisher safe_cmd_vel("safe_cmd_vel", &modified_cmd);
float current_distance = 999.0;

void sonarCallback(const sensor_msgs::Range& range_msg) {
    current_distance = range_msg.range;
}

void safetyCheck(const geometry_msgs::Twist& original_cmd) {
    modified_cmd = original_cmd;

    if (current_distance < SAFETY_DISTANCE && original_cmd.linear.x > 0) {
        modified_cmd.linear.x = 0;
        modified_cmd.angular.z = original_cmd.angular.z;
    }
    else if (current_distance < SLOW_DISTANCE && original_cmd.linear.x > 0) {
        modified_cmd.linear.x = original_cmd.linear.x * 
            ((current_distance - SAFETY_DISTANCE) / (SLOW_DISTANCE - SAFETY_DISTANCE));
    }

    safe_cmd_vel.publish(&modified_cmd);
}

ros::Subscriber<sensor_msgs::Range> sonar_sub("sonar_range", &sonarCallback);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &safetyCheck);

void setup_obstacle_avoidance(ros::NodeHandle &nh) {
    nh.advertise(safe_cmd_vel);
    nh.subscribe(sonar_sub);
    nh.subscribe(cmd_vel_sub);
}

#endif
