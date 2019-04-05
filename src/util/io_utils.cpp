#include <Arduino.h>
#include "io_utils.h"

//beep beeps a pizo buzzer with a given frequency in Hz for duration in ms (not us)
void beep(int pin, float hz, float dur) {
  int halfT = 500000 / hz;
  for (int i = 0; i < (500 * dur / halfT) ; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(halfT);
    digitalWrite(pin, LOW);
    delayMicroseconds(halfT);
  }
}

// blink is intended for blinking a LED
// TODO: Possibly switch this to use interupts so it doesn't block?
void blink(int pin, int times, int duration) {
  for(int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(duration);
    digitalWrite(pin, LOW);
    delayMicroseconds(duration);
  }
}
