#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <gripper_driver/force.h>
#define pumpSensor 1
#define pumpOut 7

float force1=0;
float force2=0;

bool pump_input=false;


/******************* ROS configuration  *******************/
ros::NodeHandle                     nh;

std_msgs::Bool                              boolean_msg;
gripper_driver::force                       force_msg;

ros::Publisher force_publisher("gripper/force", &force_msg);      // >>>> Would be nice if you can make the topic specific to the gripper, such as: gripper/force <<<<<
ros::Publisher pumpsensor_publisher("gripper/pump_state", &boolean_msg);

void set_pump_input(const std_msgs::Bool &set_pump_input){
  pump_input  = set_pump_input.data;
 }

ros::Subscriber<std_msgs::Bool> pump_in_subscriber("gripper/pump_input", &set_pump_input);

void publishForce(){
  force1=1;
  force2=2;
  force_msg.force1=force1;
  force_msg.force2=force2;
  force_publisher.publish(&force_msg);
  return;
}


void setup() {
  // put your setup code here, to run once:
  pinMode(pumpSensor, INPUT);
  pinMode(pumpOut, OUTPUT);
  
  nh.initNode();

  nh.advertise(force_publisher);

  nh.subscribe(pump_in_subscriber);
}

void loop() {
  // put your main code here, to run repeatedly:
  publishForce();

  /* >>>>>> Do we really need a publihPumpState function in the loop?
   * When the user sends a command to set the pump_high, we can just publish  
   * the pumps status to the topic. If the user wants the pump off, we can publish  
   * a pump off status and then turn the pump off. We dont need to constantly run the 
   * publishPumpState() in the loop since this process takes processing time. <<<<<<<<<
   */
  if(pump_input){
    digitalWrite(pumpOut, HIGH);
  }
  else{
    digitalWrite(pumpOut, LOW);
  }

  delay(100);
  // >>>>> Maybe put a short delay here <<<<<
  nh.spinOnce();
}
