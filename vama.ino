#define USE_CONTROLLINNO


#ifdef USE_CONTROLLINNO
  #include <Controllino.h>  
#else
  #include <Arduino.h>
#endif

#ifdef USE_CONTROLLINNO
  #define BAR_VENT               CONTROLLINO_D0
  #define AIR_VENT               CONTROLLINO_D1
  #define WELD                   CONTROLLINO_D2
  #define PUMP                   CONTROLLINO_D3
  #define LIGHT_POWER            CONTROLLINO_D4
  #define LIGHT_WELD             CONTROLLINO_D5
  #define INPUT_POTI_PUMP        CONTROLLINO_A0
  #define INPUT_POTI_WELD        CONTROLLINO_A1
  #define INPUT_LID_SWITCH       CONTROLLINO_A2
  #define INPUT_EMERGENCY_SWITCH CONTROLLINO_A3
#else
  #define BAR_VENT               0
  #define AIR_VENT               1
  #define WELD                   2
  #define PUMP                   3
  #define LIGHT_POWER            4
  #define LIGHT_WELD             5
  #define INPUT_POTI_PUMP        A0
  #define INPUT_POTI_WELD        A1
  #define INPUT_LID_SWITCH       A2
  #define INPUT_EMERGENCY_SWITCH A3
#endif

#define MAX_PUMP_DURATION_SECONDS (20)

// used libs
#include <Fsm.h>     // https://github.com/jonblack/arduino-fsm
#include <Bounce2.h> // https://github.com/thomasfredericks/Bounce2

#include "AnalogDebounce.hpp"
#include "Output.hpp"
#include "PotiMapping.hpp"

#define DEBUG 1
/** 
 *  Pin Mapping:
 *  
 *  Outputs:
 *    DO - Balken Ventil
 *    D1 - Belüften
 *    D2 - Schweißen
 *    D3 - Pumpe
 *    D4 - Leuchtmittel Betrieb / nicht aktiv
 *    D5 - Leuchtmittel Schweißen / nicht aktiv
 *    
 *  Inputs:
 *    A0 - Poti Vakuum ( 15 sec) 0-800 inc 
 *    A1 - Poti Schweißen (ca 0.5 sec,) 0-800 inc
 *    A2 - Deckel Schalter
 *    A3 - Reset / Schnellstop
 */


// forward decls for state actions;
void on_idle_enter();
void on_idle_run();
void on_idle_exit();
void on_pumping_enter();
void on_pumping_run();
void on_pumping_exit();
void on_welding_enter();
void on_welding_run();
void on_welding_exit();
void on_ventilate_enter();
void on_ventilate_run();
void on_ventilate_exit();

void desc(); // prints current state and values on Serial, used for debugging.  

// event flags
const uint8_t EVENT_LID_CLOSE = 1<<0;
const uint8_t EVENT_LID_OPEN  = 1<<1;
const uint8_t EVENT_RESET     = 1<<2;
const uint8_t EVENT_PUMP_END  = 1<<3;
const uint8_t EVENT_WELD_END  = 1<<4;

// mapping table for non-linear potentiometer, values measured manually.
const Mapping weldMapping{
        // measured value on input, poti value *100
        {1, 1000},
        {24, 900},
        {41, 800},
        {51, 700},
        {67, 600},
        {91, 500},
        {116, 400},
        {154, 300},
        {238, 200},
        {502, 100},
        {802, 1}
};

// mapping table for non-linear potentiometer, values measured manually.
const Mapping pumpMapping{
        // measured value on input, poti value *100
        {1, 1000},
        {54, 900},
        {70, 800},
        {84, 700},
        {100, 600},
        {150, 500},
        {175, 400},
        {270, 300},
        {502, 200},
        {775, 100},
        {793, 2}
};




// globals
uint32_t pumpStartTicks;
uint32_t weldStartTicks;
uint32_t ventilateStartTicks;
bool pumpTimerEnabled = false;
bool weldTimerEnabled = false;
bool inStartup = true; // used to indicate that powercycle has just begun. 

int configuredPumpDuration_ms = 0; // mapped poti value in millis duration
int configuredWeldDuration_ms = 0; // mapped poti value in millis duration

// debouncing analog pins used as inputs
AnalogDebouncer lidSwitch(INPUT_LID_SWITCH, 512);
AnalogDebouncer emergencySwitch(INPUT_EMERGENCY_SWITCH, 512);

// Mappers to linearize non-linear potis
PotiMapping pumpPotiMapper(&pumpMapping, 11);
PotiMapping weldPotiMapper(&weldMapping, 11);

// Wrapper for Output Pins 
Output barVent(BAR_VENT);
Output airVent(AIR_VENT);
Output welder(WELD);
Output pump(PUMP);


// state defs and callbacks
State state_idle(&on_idle_enter, &on_idle_run, &on_idle_exit);
State state_pumping(&on_pumping_enter, &on_pumping_run, &on_pumping_exit);
State state_welding(&on_welding_enter, &on_welding_run, &on_welding_exit);
State state_ventilate(&on_ventilate_enter, &on_ventilate_run, &on_ventilate_exit);

// Statemachine instance
Fsm fsm(&state_idle);


void setup() {
    Serial.begin(19200);
    Serial.print("VamaControl v0.1\n----------\n");
#if DEBUG
    Serial.print(" event injection: 1 -> EVENT_LID_CLOSE, 2-> EVENT_LID_OPEN, 3->EVENT_RESET \n\n");
#endif    

    inStartup = true;
    
    // initialize all used digital output pins as outputs
    pinMode(BAR_VENT, OUTPUT);
    pinMode(AIR_VENT, OUTPUT);  
    pinMode(WELD, OUTPUT);
    pinMode(PUMP, OUTPUT);  
    pinMode(LIGHT_POWER, OUTPUT);
    pinMode(LIGHT_WELD, OUTPUT); 

    lidSwitch.interval(5);
    emergencySwitch.interval(5);
    

    // wire up transitions.
    fsm.add_transition(&state_idle, &state_pumping, EVENT_LID_CLOSE, NULL);

    fsm.add_transition(&state_pumping, &state_idle, EVENT_LID_OPEN, NULL);
    fsm.add_transition(&state_pumping, &state_ventilate, EVENT_RESET, NULL);
    fsm.add_transition(&state_pumping, &state_welding, EVENT_PUMP_END, NULL);

    fsm.add_transition(&state_welding, &state_idle, EVENT_LID_OPEN, NULL);
    fsm.add_transition(&state_welding, &state_ventilate, EVENT_RESET, NULL);
    fsm.add_transition(&state_welding, &state_ventilate, EVENT_WELD_END, NULL);

    fsm.add_transition(&state_ventilate, &state_idle, EVENT_LID_OPEN, NULL);
    fsm.add_transition(&state_ventilate, &state_idle, EVENT_RESET, NULL);
    fsm.add_timed_transition(&state_ventilate, &state_idle, 5000, NULL);

    //lightPower.enable();    
}

void desc(){
  char buffer [120]; // must be large enough for your whole string!
  auto a0 = analogRead(INPUT_POTI_PUMP);
  auto a1 = analogRead(INPUT_POTI_WELD);
  auto a2 = analogRead(INPUT_LID_SWITCH);
  auto a3 = analogRead(INPUT_EMERGENCY_SWITCH);
  
  if(DEBUG) {
     sprintf (buffer, "bar: %u air: %u, weld: %u,pump: %u poti_pumo=%03u,poti_weld=%03u, i_lid a2=%03u, i_emgecy%03u pDur=%5u wDur=%5u\n", 
                      barVent.getState(),
                      airVent.getState(),
                      welder.getState(),
                      pump.getState(), 
                      pumpPotiMapper.getMappedValue(a0),
                      weldPotiMapper.getMappedValue(a1), 
                      a2, 
                      a3,
                      configuredPumpDuration_ms,
                      configuredWeldDuration_ms);
  } else {
    buffer[0] = '.';
    buffer[1] = 0x0;    
  }
  Serial.print (buffer);
}


// checks inputs and duration and fires events to the FSM
void generateEvents(uint32_t pumpTargetDuration_ms, uint32_t weldTargetDuration_ms) {
  
    if(lidSwitch.rose()) {
      Serial.println("EVENT_LID_CLOSE");
      if(!inStartup) {
        fsm.trigger(EVENT_LID_CLOSE);
      }
    }
    if(lidSwitch.fell()) {
      Serial.println("EVENT_LID_OPEN");
      fsm.trigger(EVENT_LID_OPEN);
    }
    if(emergencySwitch.rose()) {
      Serial.println("EVENT_RESET");
      fsm.trigger(EVENT_RESET);
    }
    if(pumpTimerEnabled) {
      int32_t pumpDuration = millis() - pumpStartTicks;
      if(pumpDuration > (pumpTargetDuration_ms)){
        Serial.println("EVENT_PUMP_END");
        fsm.trigger(EVENT_PUMP_END);
      };
    }
    if(weldTimerEnabled) {
      int32_t weldDuration = millis() - weldStartTicks;
      if(weldDuration > (weldTargetDuration_ms)){
        Serial.println("EVENT_WELD_END");
        fsm.trigger(EVENT_WELD_END);
      };
    }

#if DEBUG    
    char input;
    if (Serial.available()) {
      input = Serial.read();
    }
    switch(input) {
      case '1':
            fsm.trigger(EVENT_LID_CLOSE);
            break;
      case '2':
            fsm.trigger(EVENT_LID_OPEN);
            break;
      case '3':
            fsm.trigger(EVENT_RESET);
            break;
    }    
#endif
}


// the loop function runs over and over again forever
void loop() {
    // delay reactions on lid close for 100ms
    if(inStartup && (millis() > 100)) {
      inStartup = false;
    }
    
    lidSwitch.update();
    emergencySwitch.update();
    
    delay(10); 
    // every 1000 ms
    if(millis()%100 == 0){
      desc();
    }

     // mapped value is between 0 and 1000, 
    configuredPumpDuration_ms = pumpPotiMapper.getMappedValue(analogRead(INPUT_POTI_PUMP)) * MAX_PUMP_DURATION_SECONDS;
    configuredWeldDuration_ms = weldPotiMapper.getMappedValue(analogRead(INPUT_POTI_WELD));
    
    generateEvents(configuredPumpDuration_ms, configuredWeldDuration_ms);
    
    fsm.run_machine();    
}


void on_idle_enter(){
  Serial.print("on_idle_enter \n");
  pump.off();
  airVent.off();
  welder.off();
}

void on_idle_run(){
}


void on_idle_exit(){
}

void on_pumping_enter(){
  Serial.print("on_pumping_enter \n");
  Serial.print("configured duration ms: ");
  Serial.println(configuredPumpDuration_ms);
  pump.on();
  airVent.off();
  welder.off();
  barVent.on();
  pumpStartTicks = millis();
  pumpTimerEnabled = true;
  Serial.flush();
}

void on_pumping_run(){
}

void on_pumping_exit(){
  Serial.print("on_pumping_exit \n");
  auto actualDuration = millis()-pumpStartTicks;
  Serial.print("pump duration ms: ");
  Serial.println(actualDuration);
  barVent.off();
  pumpTimerEnabled = false;
  Serial.flush();
}


void on_welding_enter(){
  Serial.print("on_welding_enter \n");
  Serial.print("configured duration ms: ");
  Serial.println(configuredWeldDuration_ms);
  weldStartTicks = millis();
  weldTimerEnabled = true;
  welder.enable();
  //lightWeld.enable();
  pump.off();

  Serial.flush();
}

void on_welding_run(){
  
}

void on_welding_exit(){
  Serial.print("on_welding_exit \n");
  auto actualDuration = millis()-weldStartTicks;
  Serial.print("weld duration ms: ");
  Serial.println(actualDuration);
  weldTimerEnabled = false;
  welder.disable();
  Serial.flush();
}

void on_ventilate_enter(){
  Serial.print("on_ventilate_enter \n");
  airVent.on();
  ventilateStartTicks = millis();
  
  desc();
}

void on_ventilate_run(){
  auto duration = millis()-ventilateStartTicks;

}

void on_ventilate_exit(){
  Serial.print("on_ventilate_exit \n");
  airVent.off();
}
