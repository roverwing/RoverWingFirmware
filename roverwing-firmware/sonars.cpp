#include "sonars.h"
uint8_t activeSonar =0;
volatile uint32_t pulseStart =0; //timestamp, in us
volatile uint32_t pulseEnd =0;   //timestamp, in us
uint32_t pingSent=0;
uint16_t numTimeouts[NUM_SONARS]; //number of timeouts in a row
volatile uint8_t status =0 ; /* 0: ping completed 1: ping initiated
                                2: return pulse started 3: returned pulse ends */
uint8_t PINS_SONAR_ECHO[]={PIN_SONAR1_ECHO,PIN_SONAR2_ECHO, PIN_SONAR3_ECHO};
uint8_t PINS_SONAR_TRIG[]={PIN_SONAR1_TRIG,PIN_SONAR2_TRIG, PIN_SONAR3_TRIG};

extern uint16_t * sonarRaw; //array of distances reported by sonars, in mm
extern uint16_t * sonarAvg; //10*(distance in mm), after low pass filter
extern uint16_t * sonarTimeout; //timeout in us
extern volatile byte * sonarBitmask;


void setupSonarPins(){
  for (int i=0; i<NUM_SONARS; i++){
    pinMode(PINS_SONAR_TRIG[i], OUTPUT);
    pinMode(PINS_SONAR_ECHO[i], INPUT);
    sonarRaw[i]=0;
    sonarAvg[i]=0;
    numTimeouts[i]=0;
  }
  *sonarTimeout = 20000; //20 000 us is about 3m 43 cm max distance
  attachInterrupt(digitalPinToInterrupt(PIN_SONAR1_ECHO), ISR_sonar1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SONAR2_ECHO), ISR_sonar2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_SONAR3_ECHO), ISR_sonar3, CHANGE);

}

/*
 * Checks for timeout or completed pings; if needed, updates
 * distance and average distance for the currently active sonar
 * and moves to the next sonar, sending a new ping
 */
void updateSonars(){
  byte bitmask = (*sonarBitmask);
  if (status==3) {
    //echo received, let us update distances
    numTimeouts[activeSonar]=0;//clear  timeouts counter
    sonarRaw[activeSonar]= MICROS_TO_MM*(pulseEnd-pulseStart-SONAR_DELAY);
    //low pass filter
    sonarAvg[activeSonar]=0.8*sonarAvg[activeSonar]+2*sonarRaw[activeSonar];
    status=0;
  } else if (status && ((micros()-pingSent)>*sonarTimeout)) {
    status=0;
    sonarRaw[activeSonar]=NO_ECHO;
    numTimeouts[activeSonar]++;
    if (numTimeouts[activeSonar]>3) {
      sonarAvg[activeSonar]=10*NO_ECHO;
    }
    status=0;
  }
  if ((status==0) && bitmask) {
    //determine next active sonar
    do {
      activeSonar++;
      if (activeSonar==NUM_SONARS) activeSonar=0;
    } while (! ( bitmask &(B00000001<<activeSonar) ) ) ;

    //now, do a new ping
    digitalWrite(PINS_SONAR_TRIG[activeSonar], LOW);
    delayMicroseconds(4);
    digitalWrite(PINS_SONAR_TRIG[activeSonar], HIGH);
    delayMicroseconds(10);
    digitalWrite(PINS_SONAR_TRIG[activeSonar], LOW);
    status=1;
    pingSent=micros();
  }

}


void ISR_sonar1(){
    if (status && (activeSonar==0) ) {
      if (REG_PORT_IN0 & PORT_PA28) {
        //pin is high - pulse started!
        if (status==1) {
          pulseStart = micros();
          status =2;
        }
      } else {
        //pin is low - pulse ends!
        if (status == 2) {
          pulseEnd = micros();
          status=3;
        }
      }
    }
}
void ISR_sonar2(){
    if (status && (activeSonar==1) ) {
      if (REG_PORT_IN1 & PORT_PB02) {
        //pin is high - pulse started!
        if (status==1) {
          pulseStart = micros();
          status =2;
        }
      } else {
        //pin is low - pulse ends!
        if (status == 2) {
          pulseEnd = micros();
          status=3;
        }
      }
    }
}
void ISR_sonar3(){
    if (status && (activeSonar==2) ) {
      if (REG_PORT_IN1 & PORT_PB03) {
        //pin is high - pulse started!
        if (status==1) {
          pulseStart = micros();
          status =2;
        }
      } else {
        //pin is low - pulse ends!
        if (status == 2) {
          pulseEnd = micros();
          status=3;
        }
      }
    }
}
