#ifndef OUTPUT_HPP
#define OUTPUT_HPP

/**
 * Wrapper for digital output pin.
 * Allows more fluent code when dealing with digital outputs,
 * and enables read back of the current state. 
 */

class Output {
  public:
    Output(int pin, bool lowActive=false): mState(LOW), mPin(pin), mLowActive(lowActive) {}
    int getState(){return mState;}
    void enable()  { digitalWrite(mPin, HIGH); mState = mLowActive ? LOW : HIGH;}
    void disable() { digitalWrite(mPin, LOW);  mState = mLowActive ? HIGH : LOW;}
    void toggle()  { digitalWrite(mPin, mState == LOW ? HIGH:LOW);}
    void off() { disable(); }
    void on() { enable(); }
  private:
   bool mState;
   const bool mLowActive;
   const int mPin;
};

#endif // OUTPUT_HPP
