/*
 * Pinout!!!
 * 
 * -------
 * Analog
 * -------
 * DONE >>> Moisture Sensor -> A1 -> LMB Hold (Val ~ <200)     <<< DONE
 * 
 * 
 * 
 * -------
 * Digital
 * -------
 * DONE >>> GyroScope -> 12 (SCL) 11 (SDA) -> Mouse Movement  <<< DONE
 * DONE >>> Buttons - > D7 D6 D4 D5 -> W A S D                <<< DONE
 * DONE >>> Shock Sensor -> 2 -> RMB click                    <<< DONE
 * DONE >>> Microphone -> 1 -> Space                          <<< DONE
 * Rotary Encoder Turn -> 3 0 -> Mouse Wheel up/down (Use interrupts?)
 * Rotary Encoder Click -> 8 -> E (Use interrupts)
 * DONE >>> Potentiometer -> A0 -> Shift/Ctrl (Use Interrupts) <<< DONE
 * 
 * 
 */



#include "JSAL_Buttons.h"
#include <Keyboard.h>
#include <Mouse.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>




//Objects
Button buttonArray[5] = {
  Button(6),
  Button(7),
  Button(4),
  Button(5),
  Button(9)
};
MPU6050 mpu;

//Pin assignments
uint8_t micPin = 1;
uint8_t potPin = A0;
uint8_t moisturePin = A1;
uint8_t shockPin = 2;
uint8_t encoderPinA = 3;
uint8_t encoderPinB = 8;

char buttonCharacters[5] = {'w', 'a', 's', 'd', 'e'};

unsigned long timeStamp = 0;

uint8_t numberOfButtons = 5;

void setup(){
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Serial Begun!!");

  Mouse.begin();
  Keyboard.begin();
  
  //Gyro inits for mouse movement
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()){
    while (1);
  }
   
  //Pin setups
  pinMode(micPin, INPUT);
  pinMode(potPin, INPUT);
  pinMode(moisturePin, INPUT);
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  
} 

void loop(){
  
  sampleButtons();
  sampleMic();
  samplePot();
  sampleMoisture();
  sampleShock();
  sampleRotary();
  if ((timeStamp + 20) < millis()){
    sampleGyro();
    timeStamp = millis();
  }
}

/*WASD*/
void sampleButtons(){
  uint8_t buttonActioner[5] = {}; 
  for (int i = 0; i < numberOfButtons; i++){
    //Store to do action to array
    buttonActioner[i] = buttonArray[i].debounce();
    //Reverse button logic because i'm lazy
    if (i == 4){
      if (buttonActioner[i] == 1){
        buttonActioner[i] = 2;
      }
      else if (buttonActioner[i] == 2){
        buttonActioner[i] = 1;
      }
    }
    //Perform appropriate action
    switch (buttonActioner[i]){
      //Key pressed
      case 1:
        Keyboard.press(buttonCharacters[i]);
        Serial.print("Pressing: ");
        Serial.println(buttonCharacters[i]);
      break;
      //Key released
      case 2:
        Keyboard.release(buttonCharacters[i]);
        Serial.print("Releasing: ");
        Serial.println(buttonCharacters[i]);
      break;
    }
  }
}

/*Mouse X/Y*/
void sampleGyro(){
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  int vx = (gx + 300) / 200;
  int vy = -(gz - 100) / 200;

  Mouse.move(vx, vy);
}

/*Space*/
void sampleMic(){
  static uint8_t prevMicState;  
  static unsigned long lastMicTs = 0;
  static uint16_t micDelay = 120;
  
  bool micState = digitalRead(micPin);
  prevMicState = (prevMicState << 1);
  prevMicState |= micState;
  if ((millis() > (lastMicTs + micDelay)) && prevMicState == 0b01111111){
    Serial.println("Microphone debounced to HIGH state");
    lastMicTs = millis();
    //Write Space
   // Keyboard.write(0x20);
  }
}

/*RMB*/
/*!!! ADD MOUSE PRESS !!!*/
void sampleShock(){
  static uint8_t shockBuffer = 0;
  static uint8_t shockState = 0;
  uint8_t shockSample = digitalRead(shockPin);
  shockBuffer = shockBuffer << 1;
  shockBuffer |= shockSample;

  if (shockState == 0 && shockBuffer == 0b01111111){
    shockState = 1;
    Serial.println("RMB Released");
  }
  else if (shockState == 1 && shockBuffer == 0b10000000){
    shockState = 0;
    Serial.println("RMB Pressed");
  }
  
}

/*LMB*/
/*!!!! ADD MOUSE PRESS !!!!*/
void sampleMoisture(){
  //Variables
  static uint8_t moistureState = 0;
  uint16_t moistureSample = analogRead(moisturePin);
  const uint16_t highSample = 250;
  const uint16_t lowSample = 200; 
  
  if (moistureState == 0 && moistureSample > highSample){
    moistureState = 1;
    Serial.println("LMB Pressed");
  }
  else if(moistureState == 1 && moistureSample < lowSample){
    moistureState = 0;
    Serial.println("LMB Released");
  }
}

/*Mouse Wheel*/
/*!!!! ADD MOUSE WHEEL MOVES !!!!*/
void sampleRotary(){
  static uint8_t lastPinASample = 0;
  uint8_t pinASample = digitalRead(encoderPinA);

  //Sample Encoder
  if((lastPinASample == 1) && (pinASample == 0)){
    if (digitalRead(encoderPinB) == 0){
      Serial.println("Mouse wheel scrolldown");
    }
    else{
      Serial.println("Mouse wheel scrollup");
    }
  }
  lastPinASample = pinASample;
}

/*L Shift*/
void samplePot(){
  //Variables
  static uint8_t potState = 0;
  uint16_t potSample = analogRead(potPin);
  const uint16_t highSample = 800;
  const uint16_t lowSample = 400; 
  
  if(potState == 0 && potSample > highSample){
    potState = 1;
    Serial.println("Shift pressed");
    Keyboard.press(0x2A);
  }
  else if (potState == 1 && potSample < lowSample){
    potState = 0;
    Serial.println("Shift Released");
    Keyboard.release(0x2A);
  }
}

