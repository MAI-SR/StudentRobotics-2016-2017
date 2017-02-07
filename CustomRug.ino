#include <PinChangeInt.h>
#include <Arduino.h>

// We communicate with the power board at 115200 baud.
#define SERIAL_BAUD 115200

#define FW_VER 0

int const trigUST = A1;  //token - Analogue Pin

int const echoUST = 10;

void setup() {
  Serial.begin(SERIAL_BAUD);
  resetBEDPins(); //Set step, direction, microstep and enable pins to default states
  pinMode(trigUST, OUTPUT);
  digitalWrite(trigUST, HIGH);
  pinMode(echoUST, INPUT);
  
  for(int i=0;i<4;i++) {
      Serial.write(EnPwmCmd[i]);
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

void readUS(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);                // a low pull on pin COMP/TRIG  triggering a sensor reading
  digitalWrite(trigPin, HIGH);               // reading Pin PWM will output pulses
    
  unsigned long DistanceMeasured=pulseIn(echoPin,LOW);
    
  if(DistanceMeasured>=25000) {              // the reading is invalid.
    Serial.println(10000);    
  } else {
    unsigned int DistanceReal = DistanceMeasured/50;           // every 50us low level stands for 1cm
    Serial.print(DistanceReal);
  }  
}

void loop() {
  // Fetch all commands that are in the buffer
  while (Serial.available()) {
    digitalWrite(EN, LOW); //Pull enable pin low to set FETs active and allow motor control
    int selected_command = Serial.read();
    // Do something different based on what we got:
    switch (selected_command) {
      case 'a':
        command_analogue_read();
        break;
      case 'r':
        command_read();
        break;
      case 'l':
        command_write(LOW);
        break;
      case 'h':
        command_write(HIGH);
        break;
      case 'i':
        command_mode(INPUT);
        break;
      case 'o':
        command_mode(OUTPUT);
        break;
      case 'p':
        command_mode(INPUT_PULLUP);
        break;
      case 'v':
        Serial.print("SRcustom:");
        Serial.print(FW_VER);
        break;
      case 'b':
        readUS(trigUST, echoUST);
        break;
      default:
        // A problem here: we do not know how to handle the command!
        // Just ignore this for now.
        break;
    }
    Serial.print("\n");
  }
}
}
