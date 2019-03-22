#include "JSAL_Buttons.h"
#include <Arduino.h>

Button::Button(uint8_t pin){
	//Set initial vars
	_pin = pin;
	_state = 0;
	_lastState = 0;
	_lastDebounce = millis();
	_debounceDelay = 50;
	
	//Init button mode
	pinMode(_pin, INPUT);
}


uint8_t Button::poll(void){
	uint8_t buttonState = digitalRead(_pin);
	return buttonState;
}

uint8_t Button::debounce(void){
	uint8_t currentState = digitalRead(_pin);
	uint8_t returnVar = 0;
	
	//Contact is still bouncing
	if (currentState != _lastState){
		_lastDebounce = millis();
	}
	
	//If passed button is in steady state (not necessarily latched)
	if ((millis() - _lastDebounce) > _debounceDelay){

		//State has changed
		if (currentState != _state){
			//Update State;
			_state = currentState;
			
			//Button has been pressed
			if (_state == 1){
				returnVar = 1;
			}
			//Button has been release
			else{
				returnVar = 2;
			}			
		}	
	}
	
	//Update state for next itteration
	_lastState = currentState;
	
	return returnVar;
}
