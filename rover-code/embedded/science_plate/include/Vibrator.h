#ifndef VIBRATOR_H
#define VIBRATOR_H

#include "Pinout.h"
#include <Arduino.h>

class Vibrator {
	private:
		VibratorPins pin;
		bool active;
	public:
		Vibrator(VibratorPins pin);
		bool isActive();
		bool toggle();
		bool set(bool status);
		void writePin();
};

#endif