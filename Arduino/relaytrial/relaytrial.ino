#define pumpAtmValve 8
#define pumpOut 7

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(pumpAtmValve, OUTPUT);
  pinMode(pumpOut, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()==0){
    Serial.print("waiting");
    delay(1);
  }
  
    char a = Serial.read();
    if(a=='n'){
      digitalWrite(pumpAtmValve, LOW);
      digitalWrite(pumpOut, LOW);
      Serial.println("yes");
      Serial.println(a);
    }
    else if(a=='y'){
      digitalWrite(pumpAtmValve, HIGH);
      digitalWrite(pumpOut, HIGH);
      Serial.println("no");
      Serial.println(a);
    }
}
