/////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEBUG_NAME HR
#include <Debug.h>
#include <PID_v1.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <limits.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// --- The motor driver ---
#define IGNITION 4 // Pin D4. ON = "pull low". OFF = "INPUT-PULLUP". (Never "pull high"!)
#define DRIVE 5 // Pin D5 is the PWM ('analog') output to the motor controller - 980Hz
#define DRIVE_MIN 51  // The analogWrite() value on DRIVE which demands zero throttle (1V nomimal)
#define DRIVE_MAX 225 //  "       "          "     "     "     "        max     "     (4V    "   )

// --- The current sensor ---
#define BATTERY B0101 // Pin A5 senses the battery current. NB motor current can be higher than battery current
#define IZERO 512 // Value on A5 when current is actually zero. (Determine by experiment. Smaller is safer.)
#define ISCALE 305 // Milliamps per unit deviation from IZERO. (Determine by calculation.)
#define OCD 3 // Pin D3: sensor pulls this low to signal an error - generally meaning >125A has been  sensed
#define MOVINGAVERAGECOUNT 1000 // How many samples of the actual current to average over. Typically 1000's.
// NOTE: ISCALE needs a fudge-factor (about X1.1) to get true values, with the way we've threaded the sensor.
// But it seems to work as-is so we're not messing with it in this stable version.

// --- The thumb-lever controlled by the rider ---
#define THUMB B0011 // Pin A3 reads the thumb controller
#define THUMB_MIN 190  // The maximum analogRead() value expected at zero throttle (1.2V nomimal)
#define THUMB_MAX 1024 //  "  minimum     "          "     "     "  max     "      (4.2V  "   )
#define IMAXDEMAND 36000 // The current (theoretically) demanded at max throttle. This WILL be exceeded.
// NOTE: THUMB_MAX should be about 860. Which means that IMAXDEMAND would be about 30000-ish to be equivalent.
// Yes, this is a bug. But the software works reliably and gives a powerful drive without blowing fuses. So
// mess with it at your peril.

// --- The fuse ---
#define FUSE_NOMINAL 33000//30000 // Rating (mA). You may prefer to use the 14,400second rating, rather than nominal, here
#define FUSE_90      37000//33000 // Max avg current (mA) during 90 seconds
#define FUSE_3       40000//37000 // Max avg current (mA) during 3 seconds
#define FUSE_0       45000 // Max instantanous current
// NOTE: We never hit even the FUSE_90 limit. Probably because THUMB_MAX is wrong (see above). So most of the limit-checking
// is redundant.

/////////////////////////////////////////////////////////////////////////////////////////////////////////
class CountDown {
  public:
    CountDown(unsigned long duration_in) : duration(duration_in), triggerTime(-duration_in) {} // Duration is in milliseconds
    void trigger() { triggerTime = millis(); }
    bool isActive() { return ((millis() - triggerTime) < duration); }
  private:
    unsigned long triggerTime;
    const unsigned long duration;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
class RollingAverage {
  public:
    RollingAverage(int count_in) : count(count_in), rollingTotal(0), rollingAverage(0) {}
    void newValue(long value) {
      rollingTotal -= rollingAverage;
      rollingTotal += value;
      rollingAverage = rollingTotal / count;
    }
    long getValue() {return rollingAverage;}
  private:
    const int count;
    long rollingTotal;
    long rollingAverage;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile RollingAverage batteryAverage(MOVINGAVERAGECOUNT); // This should be treated as effectively a private member of SenseI
int volatile ADC_thumb; // This should be treated as effectively a private member of ThumbLever
long volatile counterB, counterT; // Purely for diagnostics // These are truly global
ISR(ADC_vect) // SenseI sets this up so it will be entered 125,000 times per second.
{
    static byte toggler(0);
    int ADCvalue = ADCL; ADCvalue += (ADCH <<8); // 0 to 1023. Reading ADCH causes update... so read ADCL first
    switch(toggler) {
      case 0:  // Set up ready for THUMB. Don't read this time round.
        ADMUX &= 0xF0; ADMUX |= THUMB;
      break;
      case 1: // Don't read again - allow time to settle (at least one ADC clock cycle must elapse before reading)
      break;
      case 2: // Read the thumb lever
        ADC_thumb = ADCvalue;
        counterT++; // How many times have we read the throttle?
      break;
      case 3:  // Set up ready for BATTERY. Don't read this time round.
        ADMUX &= 0xF0; ADMUX |= BATTERY;
      break;
      case 4: // Don't read again - allow time to settle (at least one ADC clock cycle must elapse before reading)
      break;
      default: // For all values 5..255 (i.e. most of the time) we read the battery current to establish a good average over the waveform
        batteryAverage.newValue(ADCvalue);
        counterB++; /// How many times have we read the current?
    }
    toggler++;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
class SenseI {
  public:
    SenseI() : range(125000), iZeroC(IZERO) {}
    void begin() { pinMode(OCD, INPUT_PULLUP); setupAtoD(); }
    inline bool alarmed() { return !digitalRead(OCD); } // If is pulled low then there's an error
    inline long milliAmps() { return map(batteryAverage.getValue(), IZERO, 1023, 0, range); }
    
    void setupAtoD() {
      ADMUX = ADCSRA = ADCSRB = 0;
      bitSet(ADMUX, REFS0);   // Uses Vcc as reference voltage
      ADMUX |= THUMB;         // Set up for the thumb input, initially
      bitClear(ADMUX, ADLAR); // Clear means use 10-bit resolution
      bitSet(ADCSRA, ADPS2); bitSet(ADCSRA, ADPS1); bitSet(ADCSRA, ADPS0);  // Prescale by 128 (=125,000 samples per second)
      bitSet(ADCSRA, ADATE);  // Free running mode
      bitSet(ADCSRA, ADEN);   // Enable the ADC
      bitSet(ADCSRA, ADIE);   // ADC interrupt-driven
      bitSet(ADCSRA, ADSC);   // Start
      sei();
    }
  private:
    const long range; // The maximum value (milliamps) that the sensor can read. Depends on model and how it's threaded.
    long iZeroC;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
class ThumbLever {
  public:
    void begin() { ADC_thumb = 0; }
    inline long demand() { return constrain(map(ADC_thumb, THUMB_MIN, THUMB_MAX, 0, IMAXDEMAND), 0, IMAXDEMAND); }
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
class Motor {
  public:
    Motor() : idleTimer(5000) {}
    void begin() { drive(0); turnOff(); }
    inline void turnOff() { pinMode(IGNITION, INPUT_PULLUP); }
    inline void turnOn() { pinMode(IGNITION, OUTPUT); digitalWrite(IGNITION, 0); }
    inline void drive(int perMille) {
       analogWrite(DRIVE, map(perMille, 0 , 1000, DRIVE_MIN, DRIVE_MAX));
       if(perMille) idleTimer.trigger(); // Keep the timer alive
       if(idleTimer.isActive()) turnOn(); else turnOff();
    }

  private:
    CountDown idleTimer;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
class Fuse {
  public:
    Fuse() : nominal(FUSE_NOMINAL), limit90s(FUSE_90), limit3s(FUSE_3), limit0s(FUSE_0), actual90s(900), actual3s(30), cooling90s(15000), cooling3s(5000), cooling0s(1000) {}
    void begin() {now = millis();}
    void updateAverages(long current) {
      if(millis() - now > 100) { // Update the rolling averages every 0.1sec
        now = millis();
        actual90s.newValue(abs(current));
        actual3s.newValue(abs(current));
        actual0s = abs(current);
      }
    }
    
    int check90s() { return map(actual90s.getValue(), 0, limit90s, 0, 100); }
    int check3s() { return map(actual3s.getValue(), 0, limit3s, 0, 100); }
    int check0s() { return map(actual0s, 0, limit0s, 0, 100); }
    
    long getSafe() { // What's the maximum current it's now safe to pass, based on recent history?
      if(cooling90s.isActive()) return nominal;
      if(actual90s.getValue() > limit90s) { cooling90s.trigger(); return nominal; }
      if(cooling3s.isActive()) return limit90s;
      if(actual3s.getValue() > limit3s) { cooling3s.trigger(); return limit90s; }
      if(cooling0s.isActive()) return limit3s;
      if(actual0s > limit0s) { cooling0s.trigger(); return limit3s; }
      return limit0s;
    }
    
  private:
    const long nominal; // The rated value of the fuse in milliamps (or the 14400second current if preferred)
    const long limit90s, limit3s, limit0s; // The current it can stand during 90secs, 3secs, and instantaneously respectively
    long now;
    RollingAverage actual90s; // The actual (well, rolling mean) current passed in the last 90 seconds
    RollingAverage actual3s;  // The actual current passed in the last 3 seconds
    long actual0s;            // The actual instantaneous current being passed right now
    CountDown cooling90s; // Timer to manage cooling-off until limit90s will be permitted again
    CountDown cooling3s;  // Timer to manage cooling-off until limit3s will be permitted again
    CountDown cooling0s;  // Timer to manage cooling-off until limit0s will be permitted again
    // NOTE: Most of this seems redundant in this version (see notes above).
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////
SenseI sensor;
Motor motor;
ThumbLever thumb;
Fuse fuse;
//unsigned long now;

double setpoint, input, output;
double kp=0.1, ki=0.3, kd=0.001; // These are complete guesses and are surely far from optimal
PID pid(&input, &output, &setpoint, kp, ki, kd, P_ON_E, DIRECT);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  DEBUG_INIT();
  DEBUG("Setup begins\n");
  sensor.begin();
  motor.begin();
  thumb.begin();
  fuse.begin();
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(20);
  pid.SetOutputLimits(0,1000); // Motor takes an input in this range
  delay(1000); // Allow the AtoD to settle
  DEBUG("Setup is complete\n");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() { 
  static unsigned long now(millis());
  setpoint = thumb.demand();     // Demand in milliamps
  input = sensor.milliAmps();    // Actual in milliamps

  fuse.updateAverages(input);
  long fuseLimit(fuse.getSafe()); 
  if(fuseLimit < setpoint) setpoint = fuseLimit;

  pid.Compute();

  if(millis() - now > 250){
    now = millis();DEBUG("ALARM");DEBUG(sensor.alarmed());DEBUG("DEMAND");DEBUG(setpoint);DEBUG("  ACTUAL");DEBUG(input);DEBUG("OUTPUT");DEBUG((int)output);
    //DEBUG(counterB);DEBUG(counterT);counterB=counterT=0;
    DEBUG(fuse.check90s());DEBUG(fuse.check3s());DEBUG(fuse.check0s());DEBUG(fuseLimit);
    DBNL
  }

  if(setpoint < 0.1) motor.turnOff(); else {motor.turnOn(); motor.drive(output);}
}

// PRECOMPILE CHECK that we're on a 16MHz Nano
#if !defined ARDUINO_AVR_NANO
#error "Oops: this code works only on a Nano"
#endif
#if clockCyclesPerMicrosecond() != 16
#error "Oops: this code works only on a 16MHz CPU"
#endif
