#define USE_USBCON
#include <ros.h>
#define pumpSensor 1
#define pumpOut 7


bool pump_input = true;



/******************* ROS configuration  *******************/
ros::NodeHandle                     nh;

std_msgs::Float32                   force_msg;

ros::Subscriber<std_msgs::Bool> pump_in_subscriber("gripper/pump_input", &set_pump_input);

void publishForce(){
  float v_in=(float)analogRead(A1)/1023*5;

  rest_fsr= (v_supply*rest_m/v_in)-rest_m;  // >>>>>>>>>>>>>>>>>>>>>> rest_far undefined <<<<<<<<<<<<<<<<<<<<
  rest_fsr=rest_fsr/1000;
  cond_fsr=1/rest_fsr; //in k ohm           // >>>>>>>>>>>>>>>>>>>>>> cond_far undefined <<<<<<<<<<<<<<<<<<<<

  //low-high borderline C=0.8(1/kohm) or R=1.25 kohm
  if(rest_fsr<1250){
    force=p1l*pow(cond_fsr,4) + p2l*pow(cond_fsr,3) + p3l*pow(cond_fsr,2) + p4l*cond_fsr;
  }
  else{
    force=p1h*pow(cond_fsr,4) + p2h*pow(cond_fsr,3) + p3h*pow(cond_fsr,2) + p4h*cond_fsr;
  }
  
  force_msg.data= force;
  force_publisher.publish(&force_msg);
  return;
}

void publishPumpState(){
  pumpstate = digitalRead(pumpState);
  boolean_msg.data= pump_state;
  pumpsensor_publisher.publish(&boolean_msg);
  return;
}

void set_pump_input(const std_msgs::Bool &set_pump_input){
  pump_input    = set_pump_input.data;
  }

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

  delay(1)
  // >>>>> Maybe put a short delay here <<<<<
  nh.spinOnce();
}
