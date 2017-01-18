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

float time1=0;
float time2=0;
float timeinms=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedl1y:
  time1=millis();
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
  time2=millis();
  timeinms=time2-time1;
  Serial.println(timeinms);
  Serial.print("force 1 : ");
  Serial.println(force1);
  Serial.print("force 2 : ");
  Serial.println(force2);
  
}
