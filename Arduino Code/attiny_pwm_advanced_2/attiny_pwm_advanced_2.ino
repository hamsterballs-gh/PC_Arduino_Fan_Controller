
// PWM functions heavily based on:
// https://matt16060936.blogspot.com/2012/04/attiny-pwm.html


const int inputs[] = {5, 3, 2};  // Physical pins 1, 2, 7 (1 is also reset)

const long samples = 1000;
const int pwm_top = 255; // Going with 32KHz, this can stay at 255
long samples_high[3] = {};
int pwm_vals[3] = {};

void setup() {
  // Set all PWM pins to output
  DDRB = 1<<DDB4 | 1<<DDB1 | 1<<DDB0;

  // Sets PWM modes for timer 0 output A
  TCCR0A = 2<<COM0A0 | 2<<COM0B0 | 3<<WGM00;

  // Set PWM modes for timer 0 output B, and disable prescaler
  TCCR0B = 0<<WGM02 | 1<<CS00;

  // Disable timer 1 output A, disable prescaler
  TCCR1 = 0<<PWM1A | 0<<COM1A0 | 1<<CS10;

  // Set PWM modes timer 1 output B
  GTCCR = 1<<PWM1B | 2<<COM1B0;

  for (int i = 0; i < 3; i++) {
    pinMode(inputs[i], INPUT);
  }
  
}


void loop() {
  // Take samples
  for (int i = 0; i < samples; i++) {
    // Sample each pin
    for (int j = 0; j < 3; j++) {
      int read_val = digitalRead(inputs[j]);
      // If pin was High, add to respective count
      if (read_val == HIGH) {
        samples_high[j] += 1;
      }
    }
    

    delayMicroseconds(10);
  }

  for (int i = 0; i < 3; i++) {
    pwm_vals[i] = (samples_high[i] * pwm_top) / samples;
    samples_high[i] = 0;
  }
  
  
  // OCR1A = pwm_val;  // This timer 1 output is disabled so timer 0 can work
  OCR1B = pwm_vals[0]; // Physical pin 3
  OCR0A = pwm_vals[1]; // Physical pin 5
  OCR0B = pwm_vals[2]; // Physical pin 6

}
