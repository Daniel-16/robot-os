#ifndef BUTTON_PUBLISHER_H
#define BUTTON_PUBLISHER_H

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#define BUTTON_PIN 4
#define LONG_PRESS_DURATION 1000

std_msgs::Bool button_msg;
std_msgs::Int32 press_type_msg;

ros::Publisher button_pub("button_event", &button_msg);
ros::Publisher press_type_pub("press_type", &press_type_msg);

int last_button_state = HIGH;
long last_debounce_time = 0;
long press_start_time = 0;
long debounce_delay = 50;
bool is_button_pressed = false;

enum PressType {
    SHORT_PRESS = 1,
    LONG_PRESS = 2,
    DOUBLE_PRESS = 3
};

long last_press_time = 0;
const long double_press_interval = 300;
int press_count = 0;

void setup_button(ros::NodeHandle &nh) {
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    nh.advertise(button_pub);
    nh.advertise(press_type_pub);
    nh.loginfo("Button publisher advertising on /button_event and /press_type");
}

void publish_press_type(PressType type) {
    press_type_msg.data = type;
    press_type_pub.publish(&press_type_msg);
}

void loop_button() {
    int reading = digitalRead(BUTTON_PIN);

    if (reading != last_button_state) {
        last_debounce_time = millis();
    }

    if ((millis() - last_debounce_time) > debounce_delay) {
        if (reading != last_button_state) {
            if (reading == LOW) {
                press_start_time = millis();
                is_button_pressed = true;
                
                if (millis() - last_press_time < double_press_interval) {
                    publish_press_type(DOUBLE_PRESS);
                    press_count = 0;
                }
                last_press_time = millis();
                
            } else {
                if (is_button_pressed) {
                    long press_duration = millis() - press_start_time;
                    
                    if (press_duration >= LONG_PRESS_DURATION) {
                        publish_press_type(LONG_PRESS);
                    } else {
                        publish_press_type(SHORT_PRESS);
                    }
                    
                    button_msg.data = true;
                    button_pub.publish(&button_msg);
                    is_button_pressed = false;
                }
            }
        }
    }
    
    last_button_state = reading;
}

#endif
