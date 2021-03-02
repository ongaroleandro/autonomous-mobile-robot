#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>    //message type cmd_vel sends
#include <std_msgs/Float32MultiArray.h>
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

//wheel_rad is the wheel radius in meter,wheel_sep is distance between wheels in meter
double wheel_rad = 0.025, wheel_sep = 0.210;

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
  w_r = ((speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad)))*30;
  w_l = ((speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad)))*30;
  dw_r = 255*w_r/1602;    //dw_r is w_r transformed into motor driver value
  dw_l = 255*w_l/1602;
}

//rosserial, subscibe to topic cmd_vel (this is the Twist msg i.e. vector3 linear and vector3 angular) and execute messageCb when a message from the topic is received
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

//setup publisher node
std_msgs::Float32MultiArray array_msg;
ros::Publisher pub("motor_controller_data", &array_msg);

void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

//Arduino initialisation(void setup()) and program it should run(void loop())
void setup(){
 Motors_init();
 nh.initNode();
 nh.subscribe(sub);
 //defining multiarray layout, not really necessary except for last 3 lines
 array_msg.layout.dim = (std_msgs::MultiArrayDimension *)
 malloc(sizeof(std_msgs::MultiArrayDimension)*2);
 array_msg.layout.dim[0].label = "[dw_l, dw_r]";
 array_msg.layout.dim[0].size = 2;
 array_msg.data = (float*)malloc(sizeof(float) *2);
 array_msg.data_length=2;
 nh.advertise(pub);
}
void loop(){
 MotorL(dw_l);
 MotorR(dw_r);

 array_msg.data[0] = dw_l;
 array_msg.data[1] = dw_r;
 pub.publish(&array_msg);
 nh.spinOnce();
 delay(100);
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
