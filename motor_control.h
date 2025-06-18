#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ros.h>
#include <geometry_msgs/Twist.h>

#define ENA 5
#define IN1 7
#define IN2 8

#define ENB 6
#define IN3 9
#define IN4 11

const float wheel_base = 0.18;

void drive(int left_speed, int right_speed) {
    if (left_speed > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }
    analogWrite(ENB, abs(left_speed));

    if (right_speed > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    analogWrite(ENA, abs(right_speed));
}

void cmdVelCallback(const geometry_msgs::Twist& twist_msg) {
  float linear_x = twist_msg.linear.x;
  float angular_z = twist_msg.angular.z;

  float right_vel = linear_x + (angular_z * wheel_base / 2.0);
  float left_vel = linear_x - (angular_z * wheel_base / 2.0);

  int right_pwm = map(right_vel * 100, -100, 100, -255, 255);
  int left_pwm = map(left_vel * 100, -100, 100, -255, 255);
  
  drive(left_pwm, right_pwm);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("safe_cmd_vel", &cmdVelCallback);

void setup_motors(ros::NodeHandle &nh) {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  drive(0, 0);
  nh.subscribe(cmd_vel_sub);
  nh.loginfo("Motor controller subscribed to /cmd_vel");
}

void loop_motors() {
}

#endif
