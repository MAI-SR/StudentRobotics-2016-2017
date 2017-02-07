#include <Arduino.h>

// We communicate with the power board at 115200 baud.
#define SERIAL_BAUD 115200

#define FW_VER 0

const int rightStepPin = 2;
const int rightDirPin = 3;
const int leftStepPin = 4;
const int leftDirPin = 5;

int trig = 7;//COMP/TRIG auf US
int pwm = 8;//PWM auf US

unsigned int ticks = 0;

uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};    // distance measure command[copy-paste]

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(rightStepPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  pinMode(leftStepPin, OUTPUT);
  pinMode(leftDirPin, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(pwm, INPUT);
  digitalWrite(trig, HIGH);
  for(int i=0;i<4;i++)
  {
      Serial.write(EnPwmCmd[i]);
  } 
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

void rightTurn()
{
  digitalWrite(rightDirPin, HIGH);
  digitalWrite(leftDirPin, HIGH);
  tick();
}

void leftTurn()
{
  digitalWrite(rightDirPin, LOW);
  digitalWrite(leftDirPin, LOW);
  tick();
}

void forwardDrive()
{
  digitalWrite(rightDirPin, HIGH);
  digitalWrite(leftDirPin, LOW);
  tick();
}

void backwardDrive()
{
  digitalWrite(rightDirPin, LOW);
  digitalWrite(leftDirPin, HIGH);
  tick();
}

void tick()
{
  int x = 0;
  while(x < ticks)
  {
    digitalWrite(rightStepPin, HIGH);
    digitalWrite(leftStepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(rightStepPin, LOW);
    digitalWrite(leftStepPin, LOW);
    delayMicroseconds(1000);
    x = x+1;
  }
  ticks = 0;
}

void readUS(){                              // a low pull on pin COMP/TRIG  triggering a sensor reading
  digitalWrite(trig, LOW);
  digitalWrite(trig, HIGH);               // reading Pin PWM will output pulses
    
  unsigned long DistanceMeasured=pulseIn(pwm,LOW);
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
      case 'b':
        readUS();
        Serial.print("\n");
        break;
      case 'c':
        Serial.print("\n");
        forwardDrive();
        break;
      case 'd':
        Serial.print("\n");
        backwardDrive();
        break;
      case 'e':
        Serial.print("\n");
        rightTurn();
        break;
      case 'f':
        Serial.print("\n");
        leftTurn();
        break;
      case 'g':
        Serial.print("\n");
        ticks = ticks+5000;
        break;
      case 'j':
        Serial.print("\n");
        ticks = ticks+2000;
        break;
      case 'k':
        Serial.print("\n");
        ticks = ticks+1000;
        break;
      case 'm':
        Serial.print("\n");
        ticks = ticks+500;
        break;
      case 'n':
        Serial.print("\n");
        ticks = ticks+200;
        break;
      case 'q':
        Serial.print("\n");
        ticks = ticks+100;
        break;
      case 's':
        Serial.print("\n");
        ticks = ticks+50;
        break;
      case 't':
        Serial.print("\n");
        ticks = ticks+20;
        break;
      case 'u':
        Serial.print("\n");
        ticks = ticks+10;
        break;
      case 'w':
        Serial.print("\n");
        ticks = ticks+5;
        break;
      case 'x':
        Serial.print("\n");
        ticks = ticks+2;
        break;
      case 'y':
        Serial.print("\n");
        ticks = ticks+1;
        break;
      default:
        Serial.print("\n");
        // A problem here: we do not know how to handle the command!
        // Just ignore this for now.
        break;
    }
  }
}
