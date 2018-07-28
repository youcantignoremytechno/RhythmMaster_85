//----------------------------------------------------------------------
// https://github.com/youcantignoremytechno/RhythmMaster_85
// https://www.youtube.com/channel/UCU91YcyX4E5mC6s4t2yFgUw
// https://www.instagram.com/youcantignoremytechno/
// 
// 1. When you will turn on this device, you will hear white
// noise with periodic scratches. If you do not hear the scratches,
// just wail until ATTiny85 will warm up or cold down.
// 
// 2. Turn the encoder knob, it will change beats. 
// 
// 3. Push and hold the encoder knob, turn it, it will change hi-hat/snare decay.
// 
// 4. Turn on swith, it will enable compression.
// 
// 5. Do not forget to check my Instagram and YouTube channel!
//----------------------------------------------------------------------

#include "wavetables.h"

#define PIN_A         2       //Encoder pin A
#define PIN_B         0       //Encoder pin B
#define PIN_BUTTON    1       //Encoder button / Bass drum
#define PIN_PWM       4       //PWM out
#define PIN_SNARE     3       //HI-Hat / Snare
#define PIN_CLOCK     A0      //Clock input
#define INCREMENT     8192    //Envelope step size

volatile unsigned char PWM, envelopePWM, _envelopePWM, beatIndex, _beatIndex, amplitude = 255, amp, wave = 0, HS;
volatile signed char beat = 16;
volatile unsigned long envelopeACC, ADC_Time = 400;
volatile unsigned int envelopeINDEX, _envelopeINDEX, pulseTIME, tempoACC, tempo = 512;
volatile int lastEncoded = 0;

volatile byte PLAY = LOW, bassdrum, clock = LOW, _clock = LOW;

void setup() {
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  digitalWrite (PIN_A, HIGH);
  digitalWrite (PIN_B, HIGH);

  pinMode(PIN_BUTTON, INPUT); 
  digitalWrite (PIN_BUTTON, HIGH);

  pinMode(PIN_CLOCK, INPUT);
  pinMode(PIN_SNARE, OUTPUT);
  pinMode(PIN_PWM, OUTPUT); 

  GIMSK = 0b00100000;           // Enable pin change interrupts
  PCMSK = 0b00000101;           // Enable pin change interrupt for PB0 and PB2
  sei();                        // Turn on interrupts

  // Code for fast PWM: http://www.technoblogy.com/show?11TQ
  // Enable 64 MHz PLL and use as source for Timer1
  PLLCSR = 1<<PCKE | 1<<PLLE;     

  // Set up Timer/Counter1 for PWM output
  TIMSK = 0;                     // Timer interrupts OFF
  TCCR1 = 1<<CS10;               // 1:1 prescale
  GTCCR = 1<<PWM1B | 2<<COM1B0;  // PWM B, clear on match

  OCR1B = 128;
  DDRB = 1<<DDB4;                // Enable PWM output on pin 4

  // Set up Timer/Counter0 for 40kHz interrupt to output samples.
  TCCR0A = 4<<WGM00;             // Fast PWM
  TCCR0B = 1<<WGM02 | 2<<CS00;   // 1/8 prescale
  OCR0A = 99;                    // Divide by 100
  TIMSK = 1<<OCIE0A;             // Enable compare match, disable overflow
}

void loop() {

  //External clock
  _clock = clock;
  clock = LOW;
  if (analogRead(PIN_CLOCK) > 900) clock = HIGH;
  if (clock == HIGH && _clock == LOW) 
  {
    //Bass drum STOP
    if (bassdrum == HIGH) {pinMode(PIN_BUTTON, INPUT); digitalWrite(PIN_BUTTON, HIGH); bassdrum = LOW;}
  
    amp = pgm_read_byte(&beats[beat >> 2][beatIndex]);
    if (amp > 0) {amplitude = amp; digitalWrite(PIN_SNARE, HIGH); HS = 0; EG_start();}
    if (amp == 255) {digitalWrite(PIN_SNARE, LOW); HS = 1;}
    if (pgm_read_byte(&bass[beat >> 2][beatIndex]) > 0 && digitalRead(PIN_BUTTON) == HIGH) // Do not burn P1 !!!!!
    {
      //Bass drum START
      bassdrum = HIGH;
      pinMode(PIN_BUTTON, OUTPUT); 
      digitalWrite (PIN_BUTTON, LOW);
    }
       
    //Vinyl noise 
    //if (beat == 0 && analogRead(A0+15) & 0x01 && amp == 0) OCR1B = pgm_read_byte(&wavetable[1][beatIndex << 3]);
    if (analogRead(A0+15) & 0x01 && amp == 0) OCR1B = pgm_read_byte(&wavetable[1][beatIndex << 3]);
     
    beatIndex++;
    if (beatIndex == 32) beatIndex = 0;
  }
}

void EG_start() {
  envelopeINDEX = 0;
  _envelopeINDEX = 0;
  envelopeACC = 0;
  PLAY = HIGH;
}

void EG_stop() {
  envelopeINDEX = 0;
  envelopeACC = 0;
  PLAY = LOW;
}

ISR(TIMER0_COMPA_vect) {
  if (PLAY == HIGH)
  {
    if (HS == 0) {envelopeACC += (ADC_Time * INCREMENT) >> 1;} //Hi-Hat twice longer
    else {envelopeACC += ADC_Time * INCREMENT;}
    envelopeINDEX = envelopeACC >> 23;
    
    if (envelopeINDEX >= _envelopeINDEX){envelopePWM = pgm_read_byte(&adsr[HS][envelopeINDEX]);} 
    else {EG_stop();}

    _envelopeINDEX = envelopeINDEX;
  }

  //Vinyl noise
  if (beat == 0) {envelopePWM = 64; amplitude = 255; digitalWrite(PIN_SNARE, LOW);}
  
  PWM = (int(envelopePWM) * int(amplitude)) >> 8;
  OCR1B = PWM;
}

ISR(PCINT0_vect)
{
  // Code for encoder: https://thewanderingengineer.com/2014/08/11/rotary-encoder-on-the-attiny85-part-2/
  
  int MSB = digitalRead(PIN_A); //MSB = most significant bit
  int LSB = digitalRead(PIN_B); //LSB = least significant bit
 
  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value
 
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
  {
    if(digitalRead(PIN_BUTTON) == HIGH) beat ++;
    if(digitalRead(PIN_BUTTON) == LOW && bassdrum == LOW) {ADC_Time ++;}
  }

  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
  {
    if(digitalRead(PIN_BUTTON) == HIGH) beat --;
    if(digitalRead(PIN_BUTTON) == LOW && bassdrum == LOW) {ADC_Time --;}
  }

  lastEncoded = encoded;
  if (beat > 31) beat = 0;
  if (beat < 0) beat = 31;
}

