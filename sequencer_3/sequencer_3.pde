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
  D2, D3 : left and right rockers on a three-position (On-Off-On)
           switch (center pole to Ground). For tempo direction.
  D4 : momentary button, normally open (other end to Ground). For
       recording to channel A.
  D5 : momentary button, normally open (other end to Ground). For
       recording to channel B.
  D8 : Serial data for DAC
  D9 : Serial data for LED
  D10 : Clock to DAC and LED serial-to-parallel
  D11 : Latch for DAC and LED serial-to-parallel
  A0 : center pole on potentiometer for tempo selection
  A1 : center pole on potentiometer for scale selection
  A2 : center pole on potentiometer for note selection

  Looking down at the potentiometers, with the contacts to the North:
  Left contact is +5V, center contact is the analog pin, Right contact
  is Ground.


author: Christopher O'Brien  <obriencj@gmail.com>

*/


/* BIG NOTE:

   This software hasn't been tested yet, because I haven't build the
   R/2R DAC, and haven't wired anything together.
 */


#include <avr/interrupt.h>


// the analog pin that the note knob is on
#define NOTE_KNOB 2

// the analog pin that the scale knob is on
#define SCALE_KNOB 1

// the analog pin that the tempo knob is on
#define TEMPO_KNOB 0

// the scale divider ranges
#define SCALE_LOW  (1 << 8)
#define SCALE_HIGH (1 << 4)

/* greater than our highest frequency tone, times the number of steps
   in our sine_table */
#define SAMPLE_RATE (4000 * 32)

// The tempo range, as a counter triggered at SAMPLE_RATE
#define TEMPO_SLOW (SAMPLE_RATE / 2)
#define TEMPO_FAST (SAMPLE_RATE / 50)

static const float timer_freq = F_CPU / SAMPLE_RATE;


static const byte sine_table[] = {
  0, 3, 10, 22, 38, 57, 79, 103,
  127, 152, 176, 198, 217, 233, 245, 252,
  254, 252, 245, 233, 217, 198, 176, 152, 
  128, 103, 79, 57, 38, 22, 10, 3,
};


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


// ticker is what the counter resets itself to upon reaching zero.
// a ticker of zero is treated specially as a rest
static unsigned int ticker_a = 0, ticker_b = 0;
static unsigned int counter_a = 0, counter_b = 0;


// same as with the ticker above, the tempo_counter decrements until
// zero, then resets to the tempo
static unsigned int tempo = 1000;
static unsigned int tempo_counter = 1000;


// the current index into the sine_table
static unsigned int spkr1 = 0, spkr2 = 0;



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

  // service channel A
  if(ticker_a) {
    if(! counter_a--) {
      changed = true;
      spkr1++;
      counter_a = ticker_a;
    }
  } else {
    spkr1 = 0;
  }
  
  // service channel B
  if(ticker_b) {
    if(! counter_b--) {
      changed = true;
      spkr2++;
      counter_b = ticker_b;
    }
  } else {
    spkr2 = 0;
  }

  /* The tempo advances the sequences. Every time tempo_counter
     reaches zero, it is reset to the tempo again, and the pattern
     index is advanced in the direction of the tempo_step. At this
     time, the ticker_a and ticker_b values are set to the contents of
     the pattern at that index, causing both channels to begin
     alternating their line after that many interrupts. */
  
  // service the tempo
  if(! tempo_counter--) {

    /* we don't set changed to true here, because we won't consider
       the next step in the tempo pattern ready to be displayed until
       we've actually got the new tone playing. */
    
    ticker_a = pattern[pattern_i][0];
    ticker_b = pattern[pattern_i][1];
    
    tempo_counter = tempo;

    pattern_i += tempo_step;
    pattern_i %= 64;
  }

  if(changed) {

    /* serialize out the average of the two sine_table values and the
       current pattern index. LSB first, because it's easier. */

    byte avg = (sine_table[spkr1] + sine_table[spkr2]) / 2;
    byte pti = pattern_i;

    for(int i = 8; i--; ) {

      // low latch, low clock, and set the LSB of pti and avg
      PORTB = (PORTB & 0xf0) | ((pti & 1) << 1) | avg & 1;

      // now we have a new LSB to write
      avg >>= 1;
      pti >>= 1;

      // clock on rising edge
      PORTB |= 0x4;
    }

    // latch on rising edge
    PORTB |= 0x8;
  }
}



/* turn the frequency of the tone (note / scale) into a number of
   ticks for our interrupt to count before stepping through the sine
   table. */
#define note_to_ticks(notei, scale) \
  (notei? ((SAMPLE_RATE / 32) / (note_set[notei] / scale)): 0)



void setup() {
  
  // our initial patterns, which is just some beep boop looping.
  const int foo_a[] = {3, 0, 1, 0, 6, 0, 1, 0};
  const int foo_b[] = {1, 0, 0, 0, 1, 0, 0, 1};
  
  cli();

  for(int index = 64; index--; ) {
    pattern[index][0] = note_to_ticks(foo_a[index % 8], 64);
    pattern[index][1] = note_to_ticks(foo_b[index % 8], 64);
  }

  // this madness sets up our Timer 1 interrupt frequency.
  // thanks to Michael Smith
  // http://www.arduino.cc/playground/Code/PCMAudio
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  OCR1A = F_CPU / SAMPLE_RATE;
  TIMSK1 |= _BV(OCIE1A);
  
  // set the bottom of PORTC as output
  DDRB = 0x0f;
  
  // set all of PORTB as input with the bottom four as pull-up
  DDRD = 0x00;
  PORTD = 0x0f;

  // re-enable interrupts
  sei();
}



void loop() {
  int recording_a, recording_b;
  byte data = PINB;
  
  tempo = map(analogRead(TEMPO_KNOB), 0, 1023, TEMPO_SLOW, TEMPO_FAST);
  tempo_step = !(data & 1)? -1: !(data & 2)? 1: 0;
  
  recording_a = !(data & 4);
  recording_b = !(data & 8);
  
  if(recording_a || recording_b) {
    
    /* we only bother doing the analog read if we're actually going to
       record the value into our pattern. */
    
    int scale, notei, i = pattern_i;
    unsigned int ticks;
    
    // the scale divider
    scale = map(analogRead(SCALE_KNOB), 0, 1023, SCALE_LOW, SCALE_HIGH);
    scale &= 0xfffe;
    
    // an index into the note_set array
    notei = map(analogRead(NOTE_KNOB), 0, 1023, 0, 15);
    
    /* converts the above into the number of ticks the interrupt fires
       per sample of the tone. */
    ticks = note_to_ticks(notei, scale);
    
    if(recording_a) {
      pattern[i][0] = ticks;
    }
    if(recording_b) {
      pattern[i][1] = ticks;
    }
  }
}



/* The end. */
