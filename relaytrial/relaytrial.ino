#define pumpOut 7

void setup() {
  // put your setup code here, to run once:
  pinMode(pumpOut, OUTPUT);
  Serial.begin(9600);
  digitalWrite(pumpOut, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()==0);
  
  if(Serial.read()=='1'){
    digitalWrite(pumpOut, HIGH);
    Serial.println("ON");
  }
  else{
    digitalWrite(pumpOut, LOW);
    Serial.println("OFF");
  }
}
