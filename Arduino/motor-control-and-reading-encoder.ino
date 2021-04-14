#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>    //message type cmd_vel sends
#include <std_msgs/Float32MultiArray.h>
#include <ros/time.h>
#include <TimerOne.h>
#include <Encoder.h>

//define pins for motor driver
#define PWM_1 6
#define DIR_1 7
#define PWM_2 11
#define DIR_2 12

/*
Truth table motor driver:
  PWM | DIR | M_A  | M_B 
  ----|-----|------|-----
  low | X   | Low  | Low   standstill
  high| low | high | low   counter-clockwise
  high| high| low  | high  clockwise
 */

const long deltaT = 50000;

double w_r=0, w_l=0, dw_r=0, dw_l=0, abs_dw_l=0, abs_dw_r=0;  //Angular velocity of left motor w_l, right motor w_r 

double wheel_rad = 0.025, wheel_sep = 0.210;  //wheel_rad is the wheel radius in meter,wheel_sep is distance between wheels in meter

double speed_ang=0, speed_lin=0;

void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

ros::NodeHandle nh; //rosserial, setup node. Allows program to subscribe/publish topics

/*
Calculate angular velocities of wheels based on received Twist msg from cmd_vel
twist msg info:
geometry_msgs/Vector3 linear
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
  w_r = ((speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad)))*30;
  w_l = ((speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad)))*30;
  dw_r = 255*w_r/1602;    //dw_r is w_r transformed into motor driver value
  dw_l = 255*w_l/1602;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb ); //rosserial, subscibe to topic cmd_vel (this is the Twist msg i.e. vector3 linear and vector3 angular) and execute messageCb when a message from the topic is received

std_msgs::Float32MultiArray array_msg;  //type of message we want to publish
ros::Publisher pub("arduino_data", &array_msg); //setup publisher node named arduino_data

Encoder motor_encoderL(2, 4, deltaT, 2280);  //interrupt counts rising and falling edge of a pulse, so for 1120 pulses/rev there are 2240 changes
Encoder motor_encoderR(3, 5, deltaT, 2280);

int Lspeed; //speed read from left motor encoder
int Rspeed; //speed read from right motor encoder

void setup(){
 Motors_init();
 
 Timer1.initialize(deltaT); //set timer to deltaT ms
 Timer1.attachInterrupt(readSpeed); //call getSpeed() from encoder.h every deltaT ms.
 attachInterrupt(0, read_motor_encoderL, CHANGE);
 attachInterrupt(1, read_motor_encoderR, CHANGE);
 
 nh.initNode();
 nh.subscribe(sub); //start subscriber node
 
 //defining multiarray size
 array_msg.data = (float*)malloc(sizeof(float) *3);
 array_msg.data_length=3;
 
 nh.advertise(pub); //start publisher node
}

void loop(){
 MotorL(dw_l);
 MotorR(dw_r);
 int sec = nh.now().sec % 10000;  //this works
 double nsec = nh.now().nsec / 100000;
 double nsec2 = nsec / 10000;
 
 if(Lspeed || Rspeed) {
  array_msg.data[0] = Lspeed;
  array_msg.data[1] = Rspeed;
  array_msg.data[2] = sec + nsec2;
  pub.publish(&array_msg); //publish message
 }
 
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

//Motor states, DIR_2 is opposite of DIR_1
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

void read_motor_encoderL(){
  motor_encoderL.updateCount();
}

void read_motor_encoderR(){
  motor_encoderR.updateCount();
}

void readSpeed(){
  Lspeed = motor_encoderL.getSpeed();
  Rspeed = motor_encoderR.getSpeed();
}
