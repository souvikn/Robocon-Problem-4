#include "encoder.h"

// volatile because it changes inside the interrupt
static volatile long count = 0;

// Runs automatically on every pulse from the sensor
void IRAM_ATTR encoder_isr(void)
{
    count++;
}

void encoder_init(int pin)
{
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), encoder_isr, RISING);
}

long encoder_get_count(void)
{
    return count;
}

void encoder_reset(void)
{
    count = 0;
}