/*
 * rosserial Servo Control Example
 *
 * This sketch dmonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */



#include <Servo.h> 
#include <ros.h>
#include <arm/Pwm.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle  nh;

 int pwm1pin = 13, pwm2pin = 12, dir1,dir2, base_pwm = 7,elbow_pwm = 6,yaw_pwm = 5,pitch_pwm = 4,roll_pwm = 3;
  int dirpin1=45, dirpin2=34,base_dir = 24,elbow_dir = 26,yaw_dir = 28,pitch_dir = 30,roll_dir = 32;
  int xpotpin = A0, ypotpin = A1; 

std_msgs::String str_msg;



void servo_cb( const arm::Pwm& cmd_msg){
  digitalWrite(13, HIGH-digitalRead(13));
   
    //toggle led  
  char hello21[12] = "start";
  str_msg.data = "start";

  int val1 = cmd_msg.base;
  if(val1 == 0){
    analogWrite(base_pwm,0);
    char hello[12] = "base val =0";
    str_msg.data = hello;
  }else if(val1>0){
    analogWrite(base_pwm,val1);
    digitalWrite(base_dir,1);
    char hello1[12] = "base val >0";
    str_msg.data = hello1;
  }else{
    val1=-1*val1;
    analogWrite(base_pwm,val1);
    digitalWrite(base_dir,0);
    char hello2[12] = "base val <0";
    str_msg.data = hello2;
  }
  

  int val2 = cmd_msg.gripper;
  if(val2 == 0){
    analogWrite(elbow_pwm,0);
    char hello3[15] = "gripper val =0";
    str_msg.data = hello3;
  }else if(val2>0){
    analogWrite(elbow_pwm,val2);
    digitalWrite(elbow_dir,1);
    char hello4[15] = "gripper val >0";
    str_msg.data = hello4;
  }else{
    val2=-1*val2;
    analogWrite(elbow_pwm,val2);
    digitalWrite(elbow_dir,0);
    char hello5[15] = "gripper val <0";
    str_msg.data = hello5;
  }


  int val3 = cmd_msg.yaw;
  if(val3 == 0){
    analogWrite(yaw_pwm,0);
    char hello6[11] = "yaw val =0";
    str_msg.data = hello6;
  }else if(val3>0){
    analogWrite(yaw_pwm,val3);
    digitalWrite(yaw_dir,1);
    char hello7[11] = "yaw val >0";
    str_msg.data = hello7;
  }else{
    val3=-1*val3;
    analogWrite(yaw_pwm,val3);
    digitalWrite(yaw_dir,0);
    char hello8[11] = "yaw val <0";
    str_msg.data = hello8;
  }
  
  int val = cmd_msg.pitch;
  if(val == 0){
    analogWrite(pitch_pwm,0);
    char hello9[13] = "pitch val =0";
    str_msg.data = hello9;
  }else if(val>0){
    analogWrite(pitch_pwm,val);
    digitalWrite(pitch_dir,1);
    char hello10[13] = "pitch val >0";
    str_msg.data = hello10;
  }else{
    val=-1*val;
    analogWrite(pitch_pwm,val);
    digitalWrite(pitch_dir,0);
    char hello11[13] = "pitch val <0";
    str_msg.data = hello11;
  }


  int val4 = cmd_msg.roll;
  if(val4 == 0){
    analogWrite(roll_pwm,0);
    char hello12[12] = "roll val =0";
    str_msg.data = hello12;
  }else if(val4>0){
    analogWrite(roll_pwm,val4);
    digitalWrite(roll_dir,1);
    char hello13[12] = "roll val >0";
    str_msg.data = hello13;
  }else{
    val4=-1*val4;
    analogWrite(roll_pwm,val4);
    digitalWrite(roll_dir,0);
    char hello14[12] = "roll val <0";
    str_msg.data = hello14;
  }

  int val5 = cmd_msg.shoulder_angle;
  if(val5 == 0){
    analogWrite(pwm1pin,0);
    char hello15[16] = "shoulder val =0";
    str_msg.data = hello15;
  }else if(val5>0){
    analogWrite(pwm1pin,val5);
    digitalWrite(dirpin1,0);
    char hello16[16] = "shoulder val >0";
    str_msg.data = hello16;
  }else{
    val5=-1*val5;
    analogWrite(pwm1pin,val5);
    digitalWrite(dirpin1,1                  );
    char hello17[16] = "shoulder val <0";
    str_msg.data = hello17;
  }
  int val6 = cmd_msg.elbow_angle;
  if(val6 == 0){
    analogWrite(pwm2pin,0);
    char hello18[13] = "elbow val =0";
    str_msg.data = hello18;
  }else if(val6>0){
    analogWrite(pwm2pin,val6);
    digitalWrite(dirpin2,1);
    char hello19[13] = "elbow val >0";
    str_msg.data = hello19;
  }else{
    val6=-1*val6;
    analogWrite(pwm2pin,val6);
    digitalWrite(dirpin2,0);
    char hello20[13] = "elbow val <0";
    str_msg.data = hello20;
  }

//  set_length(cmd_msg);  
}

ros::Subscriber<arm::Pwm> sub("set", servo_cb );
ros::Publisher chatter("chatter", &str_msg);

void setup(){
  pinMode(13, OUTPUT);
  pinMode(pwm1pin, OUTPUT);
  pinMode(dirpin1, OUTPUT);
  pinMode(pwm2pin, OUTPUT);
  pinMode(dirpin2, OUTPUT);
  pinMode(base_pwm, OUTPUT);
  pinMode(elbow_pwm, OUTPUT);
  pinMode(yaw_pwm, OUTPUT);
  pinMode(pitch_pwm, OUTPUT);
  pinMode(roll_pwm, OUTPUT);
  pinMode(base_dir, OUTPUT);
  pinMode(elbow_dir, OUTPUT);
  pinMode(yaw_dir, OUTPUT);
  pinMode(pitch_dir, OUTPUT);
  pinMode(roll_dir, OUTPUT);
  pinMode(xpotpin, INPUT);
  pinMode(ypotpin, INPUT);
  analogWrite(pwm1pin, 0);
  analogWrite(pwm2pin, 0);
  analogWrite(base_pwm, 0);
  analogWrite(elbow_pwm, 0);
  analogWrite(yaw_pwm, 0);
  analogWrite(pitch_pwm, 0);
  analogWrite(roll_pwm, 0);
  nh.initNode();
  
  nh.subscribe(sub);
  nh.advertise(chatter);
}

void loop(){
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);

}
