#pragma once

namespace interrupt
{

typedef void(*interrupt_handler)(void *user_data);

void attach_interrupt(int pin, interrupt_handler handler, int mode, void *user_data);
void detach_interrupt(int pin);


}