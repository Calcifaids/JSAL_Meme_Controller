#ifndef JSAL_Buttons_h
#define JSAL_Buttons_h

#include <stdint.h>


#endif


class Button {
	protected: 
		uint8_t _pin;					// HW Pin number	
		uint8_t _state;					// Current pin State
		uint8_t _lastState;
		unsigned long _lastDebounce;				// ongoing ts to compare for steady state
		unsigned long _debounceDelay;				// Debounce Delay
		
	public: 
		
	
		Button(uint8_t pin);			// Button Constructor
		uint8_t poll(void);				// Poll Button
		uint8_t debounce(void);
};
