#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <gripper_driver/force.h>
#define pumpSensor 1
#define pumpOut 7

//Pump Imput
bool pump_input = false;

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

/******************* ROS configuration  *******************/
ros::NodeHandle                     nh;

gripper_driver::force               force_msg;

ros::Publisher force_publisher("gripper/force", &force_msg); 

void publishForce(){
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
  return;
}

void set_pump_input(const std_msgs::Bool &set_pump_input){
  pump_input    = set_pump_input.data;
 }

ros::Subscriber<std_msgs::Bool> pump_in_subscriber("gripper/pump_input", &set_pump_input);

void setup() {
  // put your setup code here, to run once:
  pinMode(pumpOut, OUTPUT);
  
  nh.initNode();

  nh.subscribe(pump_in_subscriber);

  nh.advertise(force_publisher);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  /* >>>>>> Do we really need a publihPumpState function in the loop?
   * When the user sends a command to set the pump_high, we can just publish  
   * the pumps status to the topic. If the user wants the pump off, we can publish  
   * a pump off status and then turn the pump off. We dont need to constantly run the 
   * publishPumpState() in the loop since this process takes processing time. <<<<<<<<<
   */
   publishForce();
   
  if(pump_input){
    digitalWrite(pumpOut, HIGH);
  }
  else{
    digitalWrite(pumpOut, LOW);
  }
  
  // >>>>> Maybe put a short delay here <<<<<
  nh.spinOnce();
  delay(10);
}
