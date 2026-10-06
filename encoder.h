#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

void encoder_init(int pin);
long encoder_get_count(void);
void encoder_reset(void);

#endif