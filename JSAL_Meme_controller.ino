/*
 * 
 * TO DO:
 * ---
 * Fix RMB
 * Tweak Gyro to reduce drifte
 * ---
 * 
 * Pinout!!!
 * -------
 * Analog
 * -------
 * DONE >>> Moisture Sensor -> A1 -> LMB Hold (Val ~ <200)      <<< DONE
 * DONE >>> Potentiometer -> A0 -> Shift/Ctrl                   <<< DONE
 * DONE >>> Metal Touch Sensor (Prev Shock  -> A2 -> RMB click  <<< DONE
 * -------
 * Digital
 * -------
 * DONE >>> GyroScope -> 12 (SCL) 11 (SDA) -> Mouse Movement    <<< DONE
 * DONE >>> Buttons - > D7 D6 D4 D5 -> W A S D                  <<< DONE
 * DONE >>> Microphone -> 1 -> Space                            <<< DONE
 * DONE >>> Rotary Encoder Turn -> 3 0 -> Mouse Wheel up/down   <<< DONE
 * DONE >>> Rotary Encoder Click -> 8 -> E                      <<< DONE
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
uint8_t shockPin = A2;
uint8_t encoderPinA = 3;
uint8_t encoderPinB = 8;

char buttonCharacters[5] = {'w', 'a', 's', 'd', 'e'};

unsigned long timeStamp = 0;

uint8_t numberOfButtons = 5;

volatile bool shockState = 0;
volatile bool shockFlag = 0;

void setup(){
  Serial.begin(115200);
  //while (!Serial);
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
  pinMode(shockPin, INPUT);
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
  int vx = (gx + 200) / 200;
  int vy = -(gz + 50) / 200;
  
  Mouse.move(vx, vy);
}

/*Space*/
void sampleMic(){
  static uint8_t prevMicState;  
  static unsigned long lastMicTs = 0;
  static uint16_t micDelay = 300;
  
  bool micState = digitalRead(micPin);
  prevMicState = (prevMicState << 1);
  prevMicState |= micState;
  if ((millis() > (lastMicTs + micDelay)) && micState == 1){
    Serial.println("Microphone debounced to HIGH state");
    lastMicTs = millis();
    //Write Space
    Keyboard.press(0x20);
  }
  else if (prevMicState == 0b10000000){
    Keyboard.release(0x20);
  }
}

/*RMB*/
/*!!! ADD MOUSE PRESS !!!*/
void sampleShock(){
  uint16_t shockSample = analogRead(shockPin);
  //Serial.println(shockSample);
  static unsigned long shockDebounceTs = millis();
  const uint16_t shockDebounceDelay = 120;
  static bool shockState = 0;

  switch (shockState){
    case 0:
      shockDebounceTs = millis();
      //Button has been pressed
      if (shockSample < 800){
        shockState = 1;
        Serial.println("RMB Pressed");
        Mouse.press(MOUSE_RIGHT);
      }
    break;
    case 1:
      //Falling debounce has occurred
      if (millis() > (shockDebounceTs + shockDebounceDelay)){
        if (shockSample >= 1023){
          shockState = 0;
          Serial.println("RMB Released");
          Mouse.release(MOUSE_RIGHT);
        }
      }
    break;
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
    Mouse.press(MOUSE_LEFT);
  }
  else if(moistureState == 1 && moistureSample < lowSample){
    moistureState = 0;
    Serial.println("LMB Released");
    Mouse.release(MOUSE_LEFT);
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
      Mouse.move(0,0,-1);
    }
    else{
      Serial.println("Mouse wheel scrollup");
      Mouse.move(0,0,1);
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
