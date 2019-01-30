#include <ros.h>
#include "auton_drone/UltrasonicArray.h"

#include "sensor_pins.h"

#define PUB_DELAY 100  // max speed of 15 * 4 = 60 microseconds, 100 to be safe

ros::NodeHandle nh;

auton_drone::UltrasonicArray sonars;
ros::Publisher pub_sonars( "/sensors/sonars", &sonars); 
 
void init_sonar_msg() {
  sonars.sonar0.pose.position.x = 0.0;
  sonars.sonar0.pose.position.y = 0.0;
  sonars.sonar0.pose.position.z = 0.0;
  sonars.sonar0.pose.orientation.x = 0.0;
  sonars.sonar0.pose.orientation.y = 0.0;
  sonars.sonar0.pose.orientation.z = 0.0;
  sonars.sonar0.pose.orientation.w = 0.0;
  
  sonars.sonar90.pose.position.x = 0.0;
  sonars.sonar90.pose.position.y = 0.0;
  sonars.sonar90.pose.position.z = 0.0;
  sonars.sonar90.pose.orientation.x = 0.0;
  sonars.sonar90.pose.orientation.y = 0.0;
  sonars.sonar90.pose.orientation.z = 0.0;
  sonars.sonar90.pose.orientation.w = 0.0;

    sonars.sonar180.pose.position.x = 0.0;
  sonars.sonar180.pose.position.y = 0.0;
  sonars.sonar180.pose.position.z = 0.0;
  sonars.sonar180.pose.orientation.x = 0.0;
  sonars.sonar180.pose.orientation.y = 0.0;
  sonars.sonar180.pose.orientation.z = 0.0;
  sonars.sonar180.pose.orientation.w = 0.0;

    sonars.sonar270.pose.position.x = 0.0;
  sonars.sonar270.pose.position.y = 0.0;
  sonars.sonar270.pose.position.z = 0.0;
  sonars.sonar270.pose.orientation.x = 0.0;
  sonars.sonar270.pose.orientation.y = 0.0;
  sonars.sonar270.pose.orientation.z = 0.0;
  sonars.sonar270.pose.orientation.w = 0.0;
}


void setup() {
  pinMode(sonar1Trig, OUTPUT);
  pinMode(sonar1Echo, INPUT);

  pinMode(sonar2Trig, OUTPUT);
  pinMode(sonar2Echo, INPUT);

  pinMode(sonar2Trig, OUTPUT);
  pinMode(sonar2Echo, INPUT);

  pinMode(sonar3Trig, OUTPUT);
  pinMode(sonar3Echo, INPUT);

  pinMode(sonar4Trig, OUTPUT);
  pinMode(sonar4Echo, INPUT);
  init_sonar_msg();
  nh.initNode();
  nh.advertise(pub_sonars);
}

float read_sensor(int trigPin, int echoPin) {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  float duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  float distance= duration*0.034/2;
  
  return distance;
}

void loop() {
  // Prints the distance on the Serial Monitor
  sonars.sonar0.dist = read_sensor(sonar1Trig, sonar1Echo);
  sonars.sonar90.dist = read_sensor(sonar2Trig, sonar2Echo);
  sonars.sonar180.dist = read_sensor(sonar3Trig, sonar3Echo);
  sonars.sonar270.dist = read_sensor(sonar4Trig, sonar4Echo);

  pub_sonars.publish(&sonars);
  nh.spinOnce();
  delay(1000);
}
