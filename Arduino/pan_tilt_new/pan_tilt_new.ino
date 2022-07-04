

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

Servo servox;
Servo servoy;

void servo_cb1( const std_msgs::UInt16& cmd_msg){
  servox.write(cmd_msg.data);  
}
void servo_cb2( const std_msgs::UInt16& cmd_msg){
  servoy.write(cmd_msg.data);  
}


ros::Subscriber<std_msgs::UInt16> sub1("pan", servo_cb1);
ros::Subscriber<std_msgs::UInt16> sub2("tilt", servo_cb2);

void setup(){
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  servox.attach(9);
  servoy.attach(10);//a1
 
}

void loop(){
  nh.spinOnce();
  delay(1);
}
