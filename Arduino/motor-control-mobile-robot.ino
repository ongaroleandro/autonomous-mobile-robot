#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>    //contains the topic cmd_vel
#define PWM_1 3
#define DIR_1 4
#define PWM_2 6
#define DIR_2 7

/*
Truth table motor driver:
  PWM | DIR | M1_A | M1_B 
  
  low | X   | Low  | Low   standstill
  high| low | high | low   counter-clockwise
  high| high| low  | high  clockwise
 */

//Angular velocity of left motor w_l, right motor w_r 
double w_r=0, w_l=0, dw_r=0, dw_l=0, abs_dw_l=0, abs_dw_r=0;

//wheel_rad is the wheel radius ,wheel_sep is distance between wheels
double wheel_rad = 1, wheel_sep = 1;

//rosserial, setup node. Allows program to subscribe/publish topics
ros::NodeHandle nh;

double speed_ang=0, speed_lin=0;

//Calculate angular velocities of wheels based on received Twist msg
//twist msg info:
/*geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
*/

void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  dw_r = w_r*255;    //dw_r is w_l transformed to motor driver value
  dw_l = w_l*255;
}

//rosserial, subscibe to topic cmd_vel (this is the Twist msg i.e. vector3 linear and vector3 angular) and execute messageCb when a message from the topic is received
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

//Arduino initialisation(void setup()) and program it should run(void loop())
void setup(){
 Motors_init();
 nh.initNode();
 nh.subscribe(sub);
}
void loop(){
 MotorL(dw_l);
 MotorR(dw_r);
 nh.spinOnce();
}

//Motor initialisation (i.e. state of motor on startup)
void Motors_init(){
 pinMode(PWM_1, OUTPUT);
 pinMode(DIR_1, OUTPUT);
 pinMode(PWM_2, OUTPUT);
 pinMode(DIR_2, OUTPUT);
 digitalWrite(PWM_1, LOW);
 digitalWrite(PWM_2, LOW);
 digitalWrite(DIR_1, LOW);
 digitalWrite(DIR_2, LOW);
}
//Motor states
void MotorL(int Pulse_Width1){
  if (Pulse_Width1 >= 255){
    analogWrite(PWM_1, 255);
    digitalWrite(DIR_1, LOW);
  }
  if (Pulse_Width1 > 0 && Pulse_Width1 < 255){
    analogWrite(PWM_1, dw_l);
    digitalWrite(DIR_1, LOW);
     
  }
  if (Pulse_Width1 < 0){
    abs_dw_l = abs(dw_l);
    analogWrite(PWM_1, abs_dw_l);
    digitalWrite(DIR_1, HIGH);
  }
  if (Pulse_Width1 == 0){
    analogWrite(PWM_1, 0);
    digitalWrite(DIR_1, HIGH);
  }
}

void MotorR(int Pulse_Width2){
  if (Pulse_Width2 >= 255){
    analogWrite(PWM_2, 255);
    digitalWrite(DIR_2, HIGH);
  }
  if (Pulse_Width2 >0 && Pulse_Width2 < 255){
    analogWrite(PWM_2, dw_r);
    digitalWrite(DIR_2, HIGH);
     
  }
  if (Pulse_Width2 < 0){
    abs_dw_r = abs(dw_r);
    analogWrite(PWM_2, abs_dw_r);
    digitalWrite(DIR_2, LOW);
  }
  if (Pulse_Width2 == 0){
    analogWrite(PWM_2, 0);
    digitalWrite(DIR_2, HIGH);
  }
}
