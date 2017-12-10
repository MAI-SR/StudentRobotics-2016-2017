#include <Arduino.h>
#include <PinChangeInt.h>
#include <Servo.h>

// We communicate with the power board at 115200 baud.
#define SERIAL_BAUD 115200

#define FW_VER 0

#define USPWM 5
#define USTrigger 4
#define interruptRightOne 6
#define interruptRightTwo 7
#define interruptLeftOne 2
#define interruptLeftTwo 3
volatile int counterRight = 0;
volatile int counterLeft = 0;

Servo servoA;//instance of Servo
Servo servoB;

void setup() {
  servoA.attach(9, 500, 2500);//attaches the servo(pin, min, max)
  servoB.attach(10, 500, 2500);
  setArm(0);
  Serial.begin(SERIAL_BAUD);
  pinMode(USTrigger, OUTPUT);
  pinMode(USPWM, INPUT);
  digitalWrite(USTrigger, HIGH);
  //pinMode(interruptRightOne, INPUT); this makes everything stop working
  //PCintPort::attachInterrupt(digitalPinToInterrupt(interruptRightOne), interruptRightA, CHANGE); this makes everything stop working
  //pinMode(interruptRightTwo, INPUT); this makes everything stop working
  //PCintPort::attachInterrupt(digitalPinToInterrupt(interruptRightTwo), interruptRightB, CHANGE); this makes everything stop working
  PCintPort::attachInterrupt(interruptRightOne, interruptRightA, CHANGE);
  PCintPort::attachInterrupt(interruptRightTwo, interruptRightB, CHANGE);
  PCintPort::attachInterrupt(interruptLeftOne, interruptLeftA, CHANGE);
  PCintPort::attachInterrupt(interruptLeftTwo, interruptLeftB, CHANGE);
}

int read_pin() {
  while (!Serial.available());
  int pin = Serial.read();
  return (int)(pin - 'a');
}

void command_read() {
  int pin = read_pin();
  // Read from the expected pin.
  int level = digitalRead(pin);
  // Send back the result indicator.
  if (level == HIGH) {
    Serial.write('h');
  } else {
    Serial.write('l');
  }
}

void command_analogue_read() {
  int pin = read_pin();
  int value = analogRead(pin);
  Serial.print(value);
}

void command_write(int level) {
  int pin = read_pin();
  digitalWrite(pin, level);
}

void command_mode(int mode) {
  int pin = read_pin();
  pinMode(pin, mode);
}

void interruptRightA(){
  int a = digitalRead(interruptRightOne);
  int b = digitalRead(interruptRightTwo);
  if(a == HIGH && b == LOW || a == LOW && b == HIGH){
    counterRight++;
  } else{//if(a == HIGH && b == HIGH || a == LOW && b == LOW)
    counterRight--;
  }
}

void interruptRightB(){
  int a = digitalRead(interruptRightOne);
  int b = digitalRead(interruptRightTwo);
  if(a == HIGH && b == HIGH || a == LOW && b == LOW){
    counterRight++;
  } else {//if(a == HIGH && b == LOW || a == LOW && b == HIGH)
    counterRight--;
  }
}

void interruptLeftA(){
  int a = digitalRead(interruptLeftOne);
  int b = digitalRead(interruptLeftTwo);
  if(a == HIGH && b == LOW || a == LOW && b == HIGH){
    counterLeft++;
  } else{//if(a == HIGH && b == HIGH || a == LOW && b == LOW)
    counterLeft--;
  }
}

void interruptLeftB(){
  int a = digitalRead(interruptLeftOne);
  int b = digitalRead(interruptLeftTwo);
  if(a == HIGH && b == HIGH || a == LOW && b == LOW){
    counterLeft++;
  } else {//if(a == HIGH && b == LOW || a == LOW && b == HIGH)
    counterLeft--;
  }
}

void readUS(){                              // a low pull on pin COMP/TRIG  triggering a sensor reading
  digitalWrite(USTrigger, LOW);
  digitalWrite(USTrigger, HIGH);               // reading Pin PWM will output pulses
    
  unsigned long DistanceMeasured=pulseIn(USPWM ,LOW);
  unsigned int Distance = 0;
  if(DistanceMeasured>=10200)
  {              // the reading is invalid.
    Serial.print("300"); //used to be: Serial.println("Invalid");
  }
  else
  {
    Distance=DistanceMeasured/50;           // every 50us low level stands for 1cm
    Serial.print(Distance);
  }  
}

void motorStatusRight(){
  Serial.print(counterRight);
  counterRight = 0;
}

void motorStatusLeft(){
  Serial.print(counterLeft);
  counterLeft = 0;
}

void setPosition(char servo,int angle) {
  if(servo == 'A')
  {
    servoA.write(angle);
  }
  else
  {
    servoB.write(angle);
  }
}

void setArm(int angle) {
  servoA.write(angle);
  servoB.write(180-angle);
}

void setArmUp() {
  for(int i = 180; i > -1; i-=2)
  {
    setArm(i);
    delay(20);
  }
}

void setArmDown() {
  for(int i = 0; i < 181; i+=2)
  {
    setArm(i);
    delay(20);
  }
}

void loop() {
  // Fetch all commands that are in the buffer
  while (Serial.available()) {
    int selected_command = Serial.read();
    // Do something different based on what we got:
    switch (selected_command) {
      case 'a':
        command_analogue_read();
        Serial.print("\n");
        break;
      case 'b':
        motorStatusRight();
        Serial.print("\n");
        break;
      case 'c':
        counterRight = 0;
        counterLeft = 0;
        Serial.print("\n");
        break;
      case 'd':
        readUS();
        Serial.print("\n");
        break;
      case 'e':
        motorStatusLeft();
        Serial.print("\n");
        break;
      case 'f':
        Serial.print("\n");
        setArmUp();
        break;
      case 'g':
        Serial.print("\n");
        setArmDown();
        break;
      case 'r':
        command_read();
        Serial.print("\n");
        break;
      case 'l':
        command_write(LOW);
        Serial.print("\n");
        break;
      case 'h':
        command_write(HIGH);
        Serial.print("\n");
        break;
      case 'i':
        command_mode(INPUT);
        Serial.print("\n");
        break;
      case 'o':
        command_mode(OUTPUT);
        Serial.print("\n");
        break;
      case 'p':
        command_mode(INPUT_PULLUP);
        Serial.print("\n");
        break;
      case 'v':
        Serial.print("SRcustom:");
        Serial.print(FW_VER);
        Serial.print("\n");
        break;
      default:
        Serial.print("\n");
        // A problem here: we do not know how to handle the command!
        // Just ignore this for now.
        break;
    }
  }
}
