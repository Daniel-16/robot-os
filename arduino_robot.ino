#include <ros.h>
#include <Arduino.h>

#include "led_control.h"
#include "motor_control.h"
#include "sonar_publisher.h"
#include "button_publisher.h"
#include "obstacle_avoidance.h"

ros::NodeHandle nh;

void setup() {
    nh.initNode();
    
    while(!nh.connected()) {
        nh.spinOnce();
    }

    setup_leds(nh);
    setup_motors(nh);
    setup_sonar(nh);
    setup_button(nh);
    setup_obstacle_avoidance(nh);

    nh.loginfo("Arduino Robot node is now set up.");
}

void loop() {
    loop_leds();
    loop_motors();
    loop_sonar();
    loop_button();
    
    nh.spinOnce();
    
    delay(10);
}
