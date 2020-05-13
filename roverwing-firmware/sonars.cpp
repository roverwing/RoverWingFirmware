#include "sonars.h"
#include "regmap.h"
uint8_t activeSonar =0;
volatile uint32_t pulseStart =0; //timestamp, in us
volatile uint32_t pulseEnd =0;   //timestamp, in us
uint32_t pingSent=0;
uint16_t numTimeouts[NUM_SONARS]; //number of timeouts in a row
volatile uint8_t status =0 ; /* 0: ping completed 1: ping initiated
                                2: return pulse started 3: returned pulse ends */
uint8_t PINS_SONAR_ECHO[]={PIN_SONAR1_ECHO,PIN_SONAR2_ECHO, PIN_SONAR3_ECHO};
uint8_t PINS_SONAR_TRIG[]={PIN_SONAR1_TRIG,PIN_SONAR2_TRIG, PIN_SONAR3_TRIG};

//#define SONAR_DEBUG


void setupSonarPins(){
  for (int i=0; i<NUM_SONARS; i++){
    pinMode(PINS_SONAR_TRIG[i], OUTPUT);
    pinMode(PINS_SONAR_ECHO[i], INPUT);
    sonarRaw[i]=0;
    sonarAvg[i]=0;
    numTimeouts[i]=0;
  }
  *sonarTimeout = 20000; //20 000 us is about 3m 43 cm max distance
                         //this is default; it can be changed by user writing this register
  attachInterrupt(PIN_SONAR1_ECHO, ISR_sonar1, CHANGE);
  attachInterrupt(PIN_SONAR2_ECHO, ISR_sonar2, CHANGE);
  attachInterrupt(PIN_SONAR3_ECHO, ISR_sonar3, CHANGE);

}

/*
 * Checks for timeout or completed pings; if needed, updates
 * distance and average distance for the currently active sonar
 * and moves to the next sonar, sending a new ping
 */
void updateSonars(){
  byte bitmask = (*sonarBitmask);
  #ifdef SONAR_DEBUG
      Serial.print("Active sonar: "); Serial.print(activeSonar);
      Serial.print(" status: "); Serial.println(status);
  #endif
  if (status==3) {
    //echo received, let us update distances
    #ifdef SONAR_DEBUG
        Serial.println("Echo received");
    #endif
    numTimeouts[activeSonar]=0;//clear  timeouts counter
    sonarRaw[activeSonar]= MICROS_TO_MM*(pulseEnd-pulseStart-SONAR_DELAY);
    //low pass filter
    sonarAvg[activeSonar]=0.7*sonarAvg[activeSonar]+3*sonarRaw[activeSonar];
    status=0;
  } else if (status && ((micros()-pingSent)>*sonarTimeout)) {
      #ifdef SONAR_DEBUG
          Serial.println("Timeout");
      #endif
    /*
    pinMode(PINS_SONAR_ECHO[activeSonar], OUTPUT);
    digitalWrite(PINS_SONAR_ECHO[activeSonar], LOW);
    pinMode(PINS_SONAR_ECHO[activeSonar], INPUT);
    */ 
    status=0;
    sonarRaw[activeSonar]=(*sonarTimeout)*MICROS_TO_MM; //set the raw value to max distance,
                                                        //but do not change the average value yet.
    numTimeouts[activeSonar]++;
    if (numTimeouts[activeSonar]>=3) {
      //after 3 timeouts in a row, change sonarAvg to  max distance value
      sonarAvg[activeSonar]=10*sonarRaw[activeSonar];
    }
    status=0;
  } else {
    #ifdef SONAR_DEBUG
        Serial.println("Still waiting");
    #endif
  }
  if ((status==0) && bitmask) {
      #ifdef SONAR_DEBUG
          Serial.println("Moving to new sonar");
      #endif

    //determine next active sonar
    do {
      activeSonar++;
      if (activeSonar==NUM_SONARS) activeSonar=0;
    } while (! ( bitmask &(B00000001<<activeSonar) ) ) ;
    #ifdef SONAR_DEBUG
        Serial.print("Pinging  sonar "); Serial.println(activeSonar);
    #endif

    //now, do a new ping
    digitalWrite(PINS_SONAR_TRIG[activeSonar], LOW);
    delayMicroseconds(4);
    digitalWrite(PINS_SONAR_TRIG[activeSonar], HIGH);
    delayMicroseconds(10);
    digitalWrite(PINS_SONAR_TRIG[activeSonar], LOW);
    status=1;
    pingSent=micros();
    #ifdef SONAR_DEBUG
        Serial.println("Ping sent");
    #endif
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
