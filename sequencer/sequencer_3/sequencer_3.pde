/*

This version of the sequencer has two separate sine waves: A and
B. Each have their own recording button and sequence.  They are both
played at the same tempo.

This version of the sequencer performs the playback on those sines via
an interrupt. The frequency output of channel A and channel B, and the
tempo stepping through the sequence are all controlled via
counters. The main program loop only performs observations on the
digital input ports and the three analog lines.

This sequencer does not directly output a square waveform. Instead, it
produces an 8bit sequence serially which must be decoded by an IC and
in turn passed to a DAC.

WIRING:
  D4, D5 : left and right rockers on a three-position (On-Off-On)
           switch (center pole to Ground). For tempo direction.
  D6 : momentary button, normally open (other end to Ground). For
       recording to channel A.
  D7 : momentary button, normally open (other end to Ground). For
       recording to channel B.
  D8 : Serial data for DAC
  D9 : Serial data for LED
  D10 : Clock to DAC and LED serial-to-parallel
  D11 : Latch to DAC and LED serial-to-parallel
  A0 : center pole on potentiometer for tempo selection
  A1 : center pole on potentiometer for scale selection
  A2 : center pole on potentiometer for note selection

  Looking down at the potentiometers, with the contacts to the North:
  Left contact is +5V, center contact is the analog pin, Right contact
  is Ground.


author: Christopher O'Brien  <obriencj@gmail.com>

*/



#include <avr/interrupt.h>



// the analog pin that the note knob is on
#define NOTE_KNOB 2

// the analog pin that the scale knob is on
#define SCALE_KNOB 1

// the analog pin that the tempo knob is on
#define TEMPO_KNOB 0

// the scale divider ranges
#define SCALE_LOW  220
#define SCALE_HIGH 20


// how often we update the DAC, in hertz
#define SAMPLE_RATE 10000


// The tempo range, in hertz
#define TEMPO_SLOW 1
#define TEMPO_FAST 10



#define SERIAL_DEBUG 0
#define SERIAL_BAUD 115200

#if SERIAL_DEBUG
#define Debug_begin() Serial.begin(SERIAL_BAUD);
#define Debug_print(a...) Serial.print(a)
#define Debug_println(a...) Serial.println(a)
#define Debug_delay(a) delay(a)
#else
#define Debug_begin(...)
#define Debug_print(...)
#define Debug_println(...)
#define Debug_delay(...)
#endif



/* a 256 sampled 8 bit sine wave. Why so many samples? Because we're
   going to be doing some weird jumping around in this table at most
   frequencies, and the greater the resolution inside the table, the
   less like crap our output looks */
static const byte sine_table[] = {
  127, 130, 133, 136, 139, 142, 145, 148,
  151, 154, 157, 160, 163, 166, 169, 172,
  175, 178, 181, 184, 186, 189, 192, 194,
  197, 200, 202, 205, 207, 209, 212, 214,
  216, 218, 221, 223, 225, 227, 229, 230,
  232, 234, 235, 237, 239, 240, 241, 243,
  244, 245, 246, 247, 248, 249, 250, 250,
  251, 252, 252, 253, 253, 253, 253, 253,
  254, 253, 253, 253, 253, 253, 252, 252,
  251, 250, 250, 249, 248, 247, 246, 245,
  244, 243, 241, 240, 239, 237, 235, 234,
  232, 230, 229, 227, 225, 223, 221, 218,
  216, 214, 212, 209, 207, 205, 202, 200,
  197, 194, 192, 189, 186, 184, 181, 178,
  175, 172, 169, 166, 163, 160, 157, 154,
  151, 148, 145, 142, 139, 136, 133, 130,
  127, 123, 120, 117, 114, 111, 108, 105,
  102, 99, 96, 93, 90, 87, 84, 81,
  78, 75, 72, 69, 67, 64, 61, 59,
  56, 53, 51, 48, 46, 44, 41, 39,
  37, 35, 32, 30, 28, 26, 24, 23,
  21, 19, 18, 16, 14, 13, 12, 10,
  9, 8, 7, 6, 5, 4, 3, 3,
  2, 1, 1, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 1, 1,
  2, 3, 3, 4, 5, 6, 7, 8,
  9, 10, 12, 13, 14, 16, 18, 19,
  21, 23, 24, 26, 28, 30, 32, 35,
  37, 39, 41, 44, 46, 48, 51, 53,
  56, 59, 61, 64, 67, 69, 72, 75,
  78, 81, 84, 87, 90, 93, 96, 99,
  102, 105, 108, 111, 114, 117, 120, 123 };


// note frequency values thanks to Paul Badger
// http://www.arduino.cc/playground/Main/Freqout
static const float Rest  = 0,
  A     = 14080,
  AS    = 14917.2,
  B     = 15804.3,
  C     = 16744,
  CS    = 17739.7,
  D     = 18794.5,
  DS    = 19912.1,
  E     = 21096.2,
  F     = 22350.6,
  FS    = 23679.6,
  G     = 25087.7,
  GS    = 26579.5,
  A2    = 28160,
  A2S   = 29834.5,
  B2    = 31608.5,
  C2    = 33488.1,
  C2S   = 35479.4,
  D2    = 37589.1,
  D2S   = 39824.3,
  E2    = 42192.3,
  F2    = 44701.2,
  F2S   = 47359.3,
  G2    = 50175.4,
  G2S   = 53159,
  A3    = 56320;


// the notes available via the NOTE_KNOB
static const float note_set[] = {
  Rest, A,  B,  CS,  D,  E,  FS,  GS,
  A2,   B2,  C2S,  D2,  E2,  F2S,  G2S,  A3};


// this is where we store the actual pattern. Each entry in the pattern is
// a counter that the interrupt will use to cycle itself.
static unsigned int pattern_i = 0;
static unsigned int pattern[64][2];


// the direction that the pattern is played. Either 1, -1, or 0
static int tempo_step = 1;


static unsigned int div_a = 0;
static unsigned int mult_a = 0;

static unsigned int counter_a = 0;
static unsigned int position_a = 192;


// same as with the ticker above, the tempo_counter decrements until
// zero, then resets to the tempo
static unsigned int tempo = TEMPO_FAST;
static unsigned int tempo_counter = TEMPO_FAST;



ISR(TIMER1_COMPA_vect) {
  boolean changed = false;

  /* The sound channels are serviced by first checking that they are
     not a Rest note (tickr_a == 0). Rest notes will have the speaker
     line set low for the duration. Non-rest notes will have their
     8bit value advanced along the sine table every tick interrupts.
     For example, counter_a will be set to ticker_a, which could be
     56. After 56 triggers of this interrupt, the value of that wave
     at that time will be fetched from the next step in the
     sine_table, and counter_a will be reset to the value in
     ticker_a. */

  if(div_a) {
    if(! counter_a--) {
      position_a += mult_a;
      position_a %= 256;
      counter_a = div_a;
      changed = true;
    }
  }

  if(changed) {
    /* serialize out the average of the two sine_table values and the
       current pattern index. */

    byte avg = sine_table[position_a];
    byte pti = pattern_i;

    for(int i = 8; i--; ) {

      // low latch, low clock, and set the MSB of pti and avg
      PORTB = (PORTB & 0xf0) | ((!!(pti & 0x80)) << 1) | (!!(avg & 0x80));

      // now we have a new MSB to write
      avg <<= 1;
      pti <<= 1;

      // clock on rising edge
      PORTB |= 0x4;
    }

    // latch on rising edge
    PORTB |= 0x8;
  }
}



ISR(TIMER2_COMPA_vect) {
 
  /* The tempo advances the sequences. Every time tempo_counter
     reaches zero, it is reset to the tempo again, and the pattern
     index is advanced in the direction of the tempo_step. At this
     time, the ticker_a and ticker_b values are set to the contents of
     the pattern at that index, causing both channels to begin
     alternating their line after that many interrupts. */
  

  if(! tempo_counter--) {
    
    div_a = pattern[pattern_i][0];
    mult_a = pattern[pattern_i][1];
    
    tempo_counter = tempo;

    pattern_i += tempo_step;
    pattern_i %= 64;
    
    Debug_print("div_a =");
    Debug_print(div_a, DEC);
    Debug_print(", mult_a = ");
    Debug_println(mult_a, DEC);
  } 
}



static void note_conversion(unsigned int note_index, unsigned int scale,
                            unsigned int *stepper, unsigned int *divider) {
  float freq = note_set[note_index] / scale;
  float step = (SAMPLE_RATE / 256) / freq;
  *stepper = ((unsigned int) step) || 1;
  *divider = (unsigned int) (1.0 / step);
}



void setup() {
  
  // our initial patterns, which is just some beep boop looping.
  const int foo_a[] = {3, 0, 1, 0, 6, 0, 1, 0};
  
  Debug_begin();
  Debug_delay(1000);
  Debug_println("in setup");
  
  cli();

  for(int index = 64; index--; ) {
    note_conversion(foo_a[index % 8], 32, &pattern[index][0], &pattern[index][1]);
  }
  
  Debug_println("finished note conversion");
  
  div_a = pattern[0][0];
  mult_a = pattern[0][1];
  counter_a = div_a;
  
    Debug_print("converted to: div_a = ");
    Debug_print(div_a, DEC);
    Debug_print(", mult_a = ");
    Debug_println(mult_a, DEC);
    
  Debug_println("setting up Timer1");

  // this madness sets up our Timer 1 interrupt frequency.
  // thanks to Michael Smith
  // http://www.arduino.cc/playground/Code/PCMAudio
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  OCR1A = F_CPU / SAMPLE_RATE;
  TIMSK1 |= _BV(OCIE1A);
  
  Debug_println("setting up Timer2");
  
  TCCR2A = TCCR2A & ~(_BV(WGM21) | _BV(WGM20));
  TIMSK2 |= _BV(OCIE2A);

  Debug_println("timers ready");

  // set the bottom of PORTB as output
  DDRB = 0x0f;
  
  // set all of PORTD as input with the bottom four as pull-up
  DDRD = 0x00;
  PORTD = 0xf0;

  Debug_println("re-enabling interrupts");

  // re-enable interrupts
  sei();
  
  Debug_println("finished setup");
}



void loop() {
  int recording_a = 0, recording_b = 0;
  int tt;
  byte data = PIND;
  
  tt = map(analogRead(TEMPO_KNOB), 0, 1023, TEMPO_SLOW, TEMPO_FAST);
  tempo = 500 / tt;
  tempo_step = !(data & 16)? -1: !(data & 32)? 1: 0;
  
  recording_a = !(data & 64);
  //recording_b = !(data & 128);
  
  if(recording_a) {
    
    /* we only bother doing the analog read if we're actually going to
       record the value into our pattern. */
        
    int scale, notei, i = pattern_i;
    
    // the scale divider
    scale = map(analogRead(SCALE_KNOB), 0, 1023, SCALE_LOW, SCALE_HIGH);
    scale &= 0xfffe;
    
    // an index into the note_set array
    notei = map(analogRead(NOTE_KNOB), 0, 1023, 0, 15);

    Debug_print("recording: notei = ");
    Debug_print(notei, DEC);
    Debug_print(", scale = ");
    Debug_println(scale, DEC);

    note_conversion(notei, scale, &pattern[i][0], &pattern[i][1]);
    div_a = pattern[i][0];
    mult_a = pattern[i][1];
    
    Debug_print("converted to: div_a = ");
    Debug_print(div_a, DEC);
    Debug_print(", mult_a = ");
    Debug_println(mult_a, DEC);
    
    Debug_delay(2000);
  }
}



/* The end. */
