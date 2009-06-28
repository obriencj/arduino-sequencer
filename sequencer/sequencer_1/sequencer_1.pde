

// the digital pin that pulses a square wave to a speaker
#define SPEAKER 3

// the digital pin that the LED is flashed on
//#define INDICATOR 8

// the analog pin that the note knob is on
#define NOTE_KNOB 2

// the analog pin that the scale knob is on
#define SCALE_KNOB 1

// the analog pin that the tempo knob is on
#define TEMPO_KNOB 0

// the digital in pin that the Poke button is on
#define POKE_BUTTON 0

// the digital in pins connect to the three-position switch
#define STEP_LEFT 7
#define STEP_RIGHT 6

// the scale divider ranges
#define SCALE_LOW 500
#define SCALE_HIGH 10

// The tempo range, as length of note in ms
#define TEMPO_SLOW 500
#define TEMPO_FAST 50

// how many notes we loop over
#define PATTERN_LEN  64

// debug some data out over serial if 1
#define SERIAL_DEBUG 0
#define SERIAL_BAUD  115200


// note values for two octave scale
// divide them by powers of two to generate other octaves
static const float
  Rest  = 0,
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
static const float majScale[] = {
  Rest, A,  B,  CS,  D,  E,  FS,  GS,
  A2,   B2,  C2S,  D2,  E2,  F2S,  G2S,  A3};


// our pattern
static int pattern[PATTERN_LEN];       // indexes into majScale
static int pattern_scale[PATTERN_LEN]; // the scale int itself


void setup() {

  int foo[] = {0, 3, 0, 1, 0, 6, 0, 1};

  // write our initial pattern
  for(int index = PATTERN_LEN; index--; ) {
    pattern_scale[index] = 64;
    pattern[index] = foo[index % 8];
  }
  
  // set some pin modes.
  pinMode(POKE_BUTTON, INPUT);

  #if INDICATOR
  pinMode(INDICATOR, OUTPUT);
  #endif

  // create a pair of pull-up inputs for our three-way toggle.
  pinMode(STEP_LEFT, INPUT);
  pinMode(STEP_RIGHT, INPUT);
  digitalWrite(STEP_LEFT, HIGH);
  digitalWrite(STEP_RIGHT, HIGH);
    
  #if SERIAL_DEBUG
  Serial.begin(SERIAL_BAUD);
  #endif
} 


#define readStep() \
  ((digitalRead(STEP_LEFT) == LOW)? -1: \
   ((digitalRead(STEP_RIGHT) == LOW)? 1: 0))


void loop(){ 
  unsigned int poke, tempo, scale;
  unsigned int index = 0, indi = 0;
  
  while(true) {

    #if INDICATOR
    // blink the indicator for alternating notes.
    digitalWrite(INDICATOR, indi? HIGH: LOW);
    indi = !indi;
    #endif
    
    // get the tempo every cycle
    tempo = map(analogRead(TEMPO_KNOB), 0, 1023, TEMPO_SLOW, TEMPO_FAST);

    if(! digitalRead(POKE_BUTTON)) {
      // the poke button is pressed, so we'll read the note and scale knobs
      poke = map(analogRead(NOTE_KNOB), 0, 1023, 0, 15);
      scale = map(analogRead(SCALE_KNOB), 0, 1023, SCALE_LOW, SCALE_HIGH);
      scale &= 0xfffe;

      // save the note and scale into the pattern
      pattern_scale[index] = scale;
      pattern[index] = poke;
      
      #if SERIAL_DEBUG 
      Serial.print("mod ");
      Serial.print(index, DEC);
      Serial.print(" to ");
      Serial.print(scale, DEC);
      Serial.print(":");
      Serial.println(poke, DEC);
      #endif
      
    } else {
      // load the note and scale from the existing pattern
      poke = pattern[index];
      scale = pattern_scale[index];
    }
    
    #if SERIAL_DEBUG
    Serial.print("play ");
    Serial.print(index, DEC);
    Serial.print(" as ");
    Serial.println(poke);
    #endif

    // play the note in its scale for the duration of the tempo
    freqout((int)(majScale[poke] / scale), tempo);
    
    // move the index forward or backwards depending on the step pins
    index += readStep();
    index %= PATTERN_LEN;
  }
}


/* frequency and time in ms */
static void freqout(int freq, int t) {
  if(! freq) {
    // just a Rest
    delay(t);
      
  } else {   
    long cycles, period;

    // subtract 7 microseconds to make up for digitalWrite overhead 
    period = (500000 / freq) - 7;
    cycles = ((long)freq * ((long)t)) / 1000;

    // turn the speaker pin on only when we're going to use it
    pinMode(SPEAKER, OUTPUT);
    freqcycle(SPEAKER, cycles, period);
    pinMode(SPEAKER, INPUT);
  }
}


static void freqcycle(int pin, long cycles, int hperiod) {

  #if SERIAL_DEBUG
  Serial.print("freqcycle ");
  Serial.print(cycles, DEC);
  Serial.print(" ");
  Serial.println(hperiod, DEC);
  #endif

  while(cycles--) {
      digitalWrite(pin, HIGH);
      delayMicroseconds(hperiod);
      digitalWrite(pin, LOW);
      delayMicroseconds(hperiod);
   }
}
