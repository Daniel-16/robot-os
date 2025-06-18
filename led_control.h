#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <ros.h>
#include <std_msgs/Bool.h>

const int ledPin = 13;

void ledCallback(const std_msgs::Bool& led_msg) {
    if (led_msg.data) {
        digitalWrite(ledPin, HIGH);
    } else {
        digitalWrite(ledPin, LOW);
    }
}

ros::Subscriber<std_msgs::Bool> led_sub("set_led_state", &ledCallback);

void setup_leds(ros::NodeHandle &nh) {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    nh.subscribe(led_sub);
    nh.loginfo("LED controller subscribed to /set_led_state");
}

void loop_leds() {
}

#endif
