#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <gripper_driver/force.h>
#define pumpSensor 2
#define pumpOut 7

//Force Sensor
float force1=0;
float force2=0;
float rest_m= 2700; //in ohm
float v_supply=5;
float rest1= 10000;
float rest2= 1000;

const float p1l= 1217.7;
const float p2l= -2941.6;
const float p3l= 2928.9;
const float p4l= 157.2 ;
const float p5l= 2.4420;

const float p1h= 0.8532;
const float p2h= -14.6476;
const float p3h= 405.0821;
const float p4h= 914.0176;
const float p5h= -1.3282;

float rest_fsr1=0;
float cond_fsr1=0;
float rest_fsr2=0;
float cond_fsr2=0;

//Pump Input
bool pump_input=false;


/******************* ROS configuration  *******************/
ros::NodeHandle                     nh;

std_msgs::Bool                              boolean_msg;
gripper_driver::force                       force_msg;

ros::Publisher force_publisher("gripper/force", &force_msg);
ros::Publisher pumpsensor_publisher("gripper/pump_state", &boolean_msg);

void set_pump_input(const std_msgs::Bool &set_pump_input){
  
  pump_input  = set_pump_input.data;
 }

ros::Subscriber<std_msgs::Bool> pump_in_subscriber("gripper/pump_input", &set_pump_input);

void publishForce(){
  char str[]= "inside publishforce()";
  nh.loginfo(str);
  
  float v_in1=(float)analogRead(A1)/1023*5;
  float v_in2=(float)analogRead(A2)/1023*5;

  //for fsr1
  rest_fsr1= (v_supply*rest_m/v_in1)-rest_m;  // >>>>>>>>>>>>>>>>>>>>>> rest_far undefined <<<<<<<<<<<<<<<<<<<<
  rest_fsr1=rest_fsr1/1000;
  cond_fsr1=1/rest_fsr1; //in k ohm           // >>>>>>>>>>>>>>>>>>>>>> cond_far undefined <<<<<<<<<<<<<<<<<<<<

  //low-high borderline C=0.8(1/kohm) or R=1.25 kohm
  if(rest_fsr1<1250){
    force1=p1l*pow(cond_fsr1,4) + p2l*pow(cond_fsr1,3) + p3l*pow(cond_fsr1,2) + p4l*cond_fsr1;
  }
  else{
    force1=p1h*pow(cond_fsr1,4) + p2h*pow(cond_fsr1,3) + p3h*pow(cond_fsr1,2) + p4h*cond_fsr1;
  }

  //for fsr2
  rest_fsr2= (v_supply*rest_m/v_in2)-rest_m;  // >>>>>>>>>>>>>>>>>>>>>> rest_far undefined <<<<<<<<<<<<<<<<<<<<
  rest_fsr2=rest_fsr2/1000;
  cond_fsr2=1/rest_fsr2; //in k ohm           // >>>>>>>>>>>>>>>>>>>>>> cond_far undefined <<<<<<<<<<<<<<<<<<<<

  //low-high borderline C=0.8(1/kohm) or R=1.25 kohm
  if(rest_fsr2<1250){
    force2=p1l*pow(cond_fsr2,4) + p2l*pow(cond_fsr2,3) + p3l*pow(cond_fsr2,2) + p4l*cond_fsr2;
  }
  else{
    force2=p1h*pow(cond_fsr2,4) + p2h*pow(cond_fsr2,3) + p3h*pow(cond_fsr2,2) + p4h*cond_fsr2;
  }
  
  force_msg.force1=force1;
  force_msg.force2=force2;
  force_publisher.publish(&force_msg);

  char str1[]= "publishforce() finish";
  nh.loginfo(str1);
  return;
}

void publishPumpState(){
  bool pump_state = digitalRead(pumpSensor);
  boolean_msg.data= pump_state;
  pumpsensor_publisher.publish(&boolean_msg);
  return;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  
  pinMode(pumpSensor, INPUT);
  pinMode(pumpOut, OUTPUT);
  
  nh.initNode();

  nh.getHardware()->setBaud(57600);
  nh.advertise(force_publisher);
  nh.advertise(pumpsensor_publisher);

  nh.subscribe(pump_in_subscriber);

  char str[]= "setup finish";
  nh.loginfo(str);
}

void loop() {
  // put your main code here, to run repeatedly:
  char str[]= "in loop";
  nh.loginfo(str);
  
  publishForce();
  publishPumpState();

  if(pump_input){
    digitalWrite(pumpOut, HIGH);
  }
  else{
    digitalWrite(pumpOut, LOW);
  }

  delay(100);

  char str1[]= "end loop";
  nh.loginfo(str1);
  
  nh.spinOnce();
}
