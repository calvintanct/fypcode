float force=0;
float rest_m= 2700; //in k ohm
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

float rest_fsr=0;
float cond_fsr=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedl1y:
  float v_inpump=(float)analogRead(A1)/1023*5;
  Serial.println(v_inpump);
  float v_in=(float)analogRead(A1)/1023*5;
  //Serial.println(v_in);
  //v_in=v_in/(rest1/rest2);
  
  rest_fsr= (v_supply*rest_m/v_in)-rest_m;  // >>>>>>>>>>>>>>>>>>>>>> rest_far undefined <<<<<<<<<<<<<<<<<<<<
  Serial.print("resistance : ");
  Serial.println(rest_fsr);
  rest_fsr=rest_fsr/1000;
  cond_fsr=1/rest_fsr; //in k ohm           // >>>>>>>>>>>>>>>>>>>>>> cond_far undefined <<<<<<<<<<<<<<<<<<<<

  //low-high borderline C=0.8(1/kohm) or R=1.25 kohm
  if(rest_fsr>1.25){
    force=p1l*pow(cond_fsr,4) + p2l*pow(cond_fsr,3) + p3l*pow(cond_fsr,2) + p4l*cond_fsr;
  }
  else{
    force=p1h*pow(cond_fsr,4) + p2h*pow(cond_fsr,3) + p3h*pow(cond_fsr,2) + p4h*cond_fsr;
  }
  Serial.print("force : ");
  Serial.println(force);
}
