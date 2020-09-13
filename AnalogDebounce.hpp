#include <Bounce2.h>

#ifndef ANALOGDEBOUNCE_HPP
#define ANALOGDEBOUNCE_HPP

/*
 * Debounces for Analog Inputs on Arduino.
 * 
 */

class AnalogDebouncer : public Debouncer
{
  public:
    AnalogDebouncer(int analogPin, int limit = 512, bool activeHigh = true): Debouncer(), mPin(analogPin), mLimit(limit), mActiveHigh(activeHigh)  {}

  protected:
    virtual bool readCurrentState() {
      return mActiveHigh ? (analogRead(mPin) > mLimit) : (analogRead(mPin) < mLimit);
    }
  private:
    const uint8_t mPin;
    const bool mActiveHigh;
    const int mLimit;
};

#endif // ANALOGDEBOUNCE_HPP
