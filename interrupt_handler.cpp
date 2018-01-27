#include <Arduino.h>
#include "interrupt_handler.h"

//#define DEBUG_ON
#if defined(DEBUG_ON)
#define DEBUG_OUT(cap, val) { Serial.print((cap)); Serial.println((val));}
#else
#define DEBUG_OUT(cap, val)
#endif

namespace interrupt
{

struct InterruptCallback
{
	int mode;
	interrupt_handler handler;
	void *user_data;
};

static InterruptCallback callbacks[24] = {};
static InterruptCallback hw_callbacks[2] = {};


volatile static int pcint_last[3] = {};
volatile uint8_t *port_to_pcmask[] =
{
	&PCMSK0,
	&PCMSK1,
	&PCMSK2
};

void internal_hw_int_handler0()
{
	hw_callbacks[0].handler(hw_callbacks[0].user_data);
}

void internal_hw_int_handler1()
{
	hw_callbacks[1].handler(hw_callbacks[1].user_data);
}

void internal_handle_interrupt(uint8_t port)
{
	uint8_t curr = *portInputRegister(port + 2);
	uint8_t mask = curr ^ pcint_last[port];
	pcint_last[port] = curr;
	if ((mask &= *port_to_pcmask[port]) == 0)
		return;

	for (uint8_t i = 0; i < 8; ++i) {
		uint8_t bit = 1 << i;
		if (bit & mask) {
			uint8_t pin = port * 8 + i;
			InterruptCallback &cb = callbacks[pin];
			if ((cb.mode == CHANGE) ||
				((cb.mode == FALLING) && !(curr & bit)) ||
				((cb.mode == RISING) && (curr & bit)) &&
				(cb.handler != nullptr)) {
				cb.handler(cb.user_data);
			}
			
		}
	}

}


ISR(PCINT0_vect)
{
	internal_handle_interrupt(0);
}

ISR(PCINT1_vect)
{
	internal_handle_interrupt(1);
}

ISR(PCINT2_vect)
{
	internal_handle_interrupt(2);
}



void attach_interrupt(int pin, interrupt_handler handler, int mode, void *user_data)
{
	int hw_interrupt = digitalPinToInterrupt(pin);

	if (hw_interrupt == NOT_AN_INTERRUPT) {
		uint8_t port = digitalPinToPort(pin) - 2;
		uint8_t mask = digitalPinToBitMask(pin);
		volatile uint8_t *pcmask = port_to_pcmask[port];
		int slot = (port == 1) ? (port * 8 + pin - 14) : (port * 8 + (pin % 8));

		callbacks[slot].handler = handler;
		callbacks[slot].mode = mode;
		callbacks[slot].user_data = user_data;

		*pcmask |= mask;
		PCICR |= (1<<port);
	}
	else {
		hw_callbacks[hw_interrupt].handler = handler;
		hw_callbacks[hw_interrupt].user_data = user_data;
		attachInterrupt(hw_interrupt, (hw_interrupt == 0) ? internal_hw_int_handler0 : internal_hw_int_handler1, mode);
	}
}

void detach_interrupt(int pin)
{
	int hw_interrupt = digitalPinToInterrupt(pin);
	if (hw_interrupt == NOT_AN_INTERRUPT) {
		uint8_t port = digitalPinToPort(pin) - 2;
		uint8_t mask = digitalPinToBitMask(pin);
		volatile uint8_t *pcmask = port_to_pcmask[port];
		*pcmask &= ~(mask);
		// disable interrupt on last pin clear
		if (*pcmask == 0) {
			PCICR &= ~(1 << port);
		}
	}
	else {
		detachInterrupt(hw_interrupt);
	}
}

}