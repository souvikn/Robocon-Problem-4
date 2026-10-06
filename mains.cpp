#include "encoder.h"

#define SENSOR_PIN 4
#define SLOTS_ON_DISC 20   // change to the number of slots on your disc

void setup()
{
    Serial.begin(115200);
    encoder_init(SENSOR_PIN);
}

void loop()
{
    long before = encoder_get_count();
    delay(1000);                          // measure for 1 second
    long after = encoder_get_count();

    long pulses = after - before;
    long rpm = (pulses * 60) / SLOTS_ON_DISC;

    Serial.print("Total pulses: ");
    Serial.print(after);
    Serial.print("   Pulses/sec: ");
    Serial.print(pulses);
    Serial.print("   RPM: ");
    Serial.println(rpm);
}