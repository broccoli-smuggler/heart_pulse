#include <Arduino.h>

/*  PulseSensor Starter Project and Signal Tester
 *  The Best Way to Get Started  With, or See the Raw Signal of, your PulseSensor.com™ & Arduino.
 *
 *  Here is a link to the tutorial
 *  https://pulsesensor.com/pages/code-and-guide
 *
 *  WATCH ME (Tutorial Video):
 *  https://www.youtube.com/watch?v=RbB8NSRa5X4
 *
 *
-------------------------------------------------------------
1) This shows a live human Heartbeat Pulse.
2) Live visualization in Arduino's Cool "Serial Plotter".
3) Blink an LED on each Heartbeat.
4) This is the direct Pulse Sensor's Signal.
5) A great first-step in troubleshooting your circuit and connections.
6) "Human-readable" code that is newbie friendly."

*/


//  Variables
int pulse_in_pin = 34;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
int heart_pin = 27;

int pulse_reading;                // holds the incoming raw data. Signal value can range from 0-1024
int threshold = 2000;            // Determine which Signal to "count as a beat", and which to ingore.
int upper = 3200;

// heartbeat range in ms: 40bpm - 200bpm or 1500ms to 
int min_time = 300;
int max_time = 1200;
uint16_t last_on;
bool latch = false;

// The SetUp Function:
void setup() {
  pinMode(heart_pin, OUTPUT_OPEN_DRAIN);

  Serial.begin(9600);
  // Print log
  Serial.println("setup");
  last_on = millis();
}

// The Main Loop Function
void loop() {

  pulse_reading = analogRead(pulse_in_pin);  // Read the PulseSensor's value.

   if(pulse_reading > threshold && pulse_reading < upper && !latch){
      uint16_t duration = millis() - last_on;
      last_on = millis();
      latch = true;
      
      // only do stuff if we really think it's a valid heartbeat
      if (duration > min_time && duration < max_time)
      {
        Serial.print(60.0 / (float(duration)/1000.0));
        Serial.print(" bpm. \n");
        digitalWrite(heart_pin, LOW);
      }

   } else {
      latch = false;
      digitalWrite(heart_pin, HIGH);                  //  Else, the sigal must be below "550", so "turn-off" this LED.
   }


  delay(10);
}