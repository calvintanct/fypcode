#include <ros.h>
#define pumpSensor 1
#define pumpOut 2

float force=0;
float rest_m= 3; //in k ohm
float v_supply=5;

const float p1l= 5.706;
const float p2l= -19.45;
const float p3l= 42.94;
const float p4l= 360.4 ;
const float p5l= 350.5;

const float p1h= 3.075;
const float p2h= -19.49;
const float p3h= 636.3;
const float p4h= 3388;
const float p5h= 3608;

bool pump_state = false;
bool pump_input = true;

float rest_fsr=0;
float cond_fsr=0;


/******************* ROS configuration  *******************/
ros::NodeHandle                     nh;

std_msgs::Bool                      boolean_msg;
std_msgs::Float32                   force_msg;

ros::Publisher force_publisher("gripper/force", &force_msg);      // >>>> Would be nice if you can make the topic specific to the gripper, such as: gripper/force <<<<<
ros::Publisher pumpsensor_publisher("gripper/pump_state", &boolean_msg);

ros::Subscriber<std_msgs::Bool> pump_in_subscriber("gripper/pump_input", &set_pump_input);

void publishForce(){

  rest_fsr= (v_supply*rest_m/v_in)-rest_m;  // >>>>>>>>>>>>>>>>>>>>>> rest_far undefined <<<<<<<<<<<<<<<<<<<<
  cond_fsr=1/rest_fsr; //in k ohm           // >>>>>>>>>>>>>>>>>>>>>> cond_far undefined <<<<<<<<<<<<<<<<<<<<

  //low-high borderline C=0.8(1/kohm) or R=1.25 kohm
  if(rest_fsr<1250){
    force=p1l*pow(cond_fsr,4) + p2l*pow(cond_fsr,3) + p3l*pow(cond_fsr,2) + p4l*cond_fsr + p5l;
  }
  else{
    force=p1h*pow(cond_fsr,4) + p2h*pow(cond_fsr,3) + p3h*pow(cond_fsr,2) + p4h*cond_fsr + p5h;
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
  pinMode(pumpState, INPUT);
  pinMode(pumpOut, OUTPUT);
  
  nh.initNode();

  nh.advertise(force_publisher);
  nh.advertise(pumpsensor_publisher);

  nh.subscribe(pump_in_subscriber);
}

void loop() {
  // put your main code here, to run repeatedly:
  publishForce();
  publishPumpState();

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
