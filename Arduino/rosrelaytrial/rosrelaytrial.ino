#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#define pumpSensor 1
#define pumpOut 7


bool pump_input = false;



/******************* ROS configuration  *******************/
ros::NodeHandle                     nh;

std_msgs::Float32                   force_msg;

void set_pump_input(const std_msgs::Bool &set_pump_input){
  pump_input    = set_pump_input.data;
 }

ros::Subscriber<std_msgs::Bool> pump_in_subscriber("gripper/pump_input", &set_pump_input);

void setup() {
  // put your setup code here, to run once:
  pinMode(pumpOut, OUTPUT);
  
  nh.initNode();

  nh.subscribe(pump_in_subscriber);
}

void loop() {
  // put your main code here, to run repeatedly:

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
  
  // >>>>> Maybe put a short delay here <<<<<
  nh.spinOnce();
  delay(1);
}
