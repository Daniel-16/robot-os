#ifndef SONAR_PUBLISHER_H
#define SONAR_PUBLISHER_H

#include <ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>

#define SONAR_TRIG_PIN 3
#define SONAR_ECHO_PIN 2
#define TEMPERATURE_PIN A0
#define NUM_SAMPLES 5

sensor_msgs::Range range_msg;
std_msgs::Float32 temp_msg;
ros::Publisher sonar_pub("sonar_range", &range_msg);
ros::Publisher temp_pub("sonar_temp", &temp_msg);

long last_sonar_publish_time = 0;
const int sonar_publish_interval = 200;
float distance_buffer[NUM_SAMPLES];
int buffer_index = 0;
bool buffer_filled = false;
float speed_of_sound = 343.0;

float calculate_median(float arr[], int size) {
    float temp;
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
    return arr[size / 2];
}

float update_speed_of_sound() {
    int raw_temp = analogRead(TEMPERATURE_PIN);
    float temperature = (raw_temp * 5.0 / 1024.0 - 0.5) * 100.0;
    temp_msg.data = temperature;
    temp_pub.publish(&temp_msg);
    return 331.3 + (0.606 * temperature);
}

void setup_sonar(ros::NodeHandle &nh) {
    pinMode(SONAR_TRIG_PIN, OUTPUT);
    pinMode(SONAR_ECHO_PIN, INPUT);
    pinMode(TEMPERATURE_PIN, INPUT);

    nh.advertise(sonar_pub);
    nh.advertise(temp_pub);

    range_msg.header.frame_id = "sonar_link";
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.field_of_view = 0.1;
    range_msg.min_range = 0.02;
    range_msg.max_range = 2.0;

    for(int i = 0; i < NUM_SAMPLES; i++) {
        distance_buffer[i] = 0.0;
    }
}

void loop_sonar() {
    if (millis() - last_sonar_publish_time > sonar_publish_interval) {
        last_sonar_publish_time = millis();
        speed_of_sound = update_speed_of_sound();

        digitalWrite(SONAR_TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(SONAR_TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(SONAR_TRIG_PIN, LOW);

        long duration = pulseIn(SONAR_ECHO_PIN, HIGH, 30000);
        
        if(duration > 0) {
            float distance_m = (duration / 2.0) * (speed_of_sound * 0.000001);
            distance_buffer[buffer_index] = distance_m;
            buffer_index = (buffer_index + 1) % NUM_SAMPLES;
            
            if(buffer_index == 0) {
                buffer_filled = true;
            }

            if(buffer_filled) {
                float filtered_distance = calculate_median(distance_buffer, NUM_SAMPLES);
                if (filtered_distance > range_msg.min_range && filtered_distance < range_msg.max_range) {
                    range_msg.range = filtered_distance;
                    range_msg.header.stamp = nh.now();
                    sonar_pub.publish(&range_msg);
                }
            }
        }
    }
}

#endif