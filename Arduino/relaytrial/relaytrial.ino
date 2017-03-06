#define pumpAtmValve 8

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pumpAtmValve, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()==0){
  }
    char a = Serial.read();
    if(a=='y'){
      digitalWrite(pumpAtmValve, LOW);
      Serial.println("yes");
      Serial.println(a);
    }
    else{
      digitalWrite(pumpAtmValve, HIGH);
      Serial.println("no");
      Serial.println(a);
    }
}
