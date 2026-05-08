//define a list of pin numbers, flash them in sequence
//each pin should be connected to an LED and a resistor in series to ground
#define ALL_PINS {4,5,6,7,12,20,21,22,23,24,25,26}
#define PIN_LIST {20,21,22,23,24,25,26}
#define NUM_PINS 7

// loop through the pins in sequence, turning each on for this long
#define FLASH_TIME 1000
// wait this long between pins
#define PAUSE_TIME 1000

// set up each pin as an output
void setup() {
  int allPins[] = ALL_PINS;
  //for all pins, set as output to false as default
  for (int i = 0; i < sizeof(allPins)/sizeof(allPins[0]); i++) {
    pinMode(allPins[i], OUTPUT);
    digitalWrite(allPins[i], LOW); // set all pins low initially
  }
} 

// loop forever
void loop() {
  int pinList[] = PIN_LIST;
  // intitialize a value to store time


  for (int i = 0; i <= NUM_PINS; i++) {
    for (int j = 0; j <= i; j++) { 
      digitalWrite(pinList[i], HIGH); // turn on the pin
      delay(FLASH_TIME/(i+1));              // wait
      digitalWrite(pinList[i], LOW);  // turn off the pin
      delay(PAUSE_TIME/(i+1));              // wait before turning on the next pin
 
    }          // wait before turning on the next pin
  }
}