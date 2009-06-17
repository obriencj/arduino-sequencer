

// the digital pin that pulses a square wave to a speaker
#define SPEAKER 3

// the digital pin that the LED is flashed on
#define INDICATOR 8

// the analog pin that the note knob is on
#define NOTE_KNOB 0

// the analog pin that the scale knob is on
#define SCALE_KNOB 3

// the analog pin that the tempo knob is on
#define TEMPO_KNOB 5

// the digital in pin that the Poke button is on
#define POKE_BUTTON 0

// The tempo range, as length of note in ms
#define TEMPO_SLOW 1000
#define TEMPO_FAST 20

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
  Rest, A,  B,  CS,  D,  E,  FS,  GS,  A2,   B2,  C2S,  D2,  E2,  F2S,  G2S,  A3};


// our pattern
static int pattern[PATTERN_LEN];       // indexes into majScale
static int pattern_scale[PATTERN_LEN]; // the scale int itself


void setup() {

  // write our initial pattern
  for(int i = PATTERN_LEN; i--; ) {
    pattern_scale[i] = 64;
    switch(i % 4) {
      case 0: pattern[i] = 0; break;
      case 1: pattern[i] = 5; break;
      case 2: pattern[i] = 0; break;
      case 3: pattern[i] = 1; break;
    }
  }
  
  // set some pin modes.
  pinMode(POKE_BUTTON, INPUT);
  pinMode(INDICATOR, OUTPUT);
    
  #if SERIAL_DEBUG
  Serial.begin(SERIAL_BAUD);
  #endif
} 


void loop(){ 
  float note;
  unsigned int i, poke, tempo, scale;
  
  for(i = PATTERN_LEN; i--; ) {
    
    // blink the indicator for alternating notes.
    digitalWrite(INDICATOR, i & 1? HIGH: LOW);
    
    // get the tempo every cycle
    tempo = map(analogRead(TEMPO_KNOB), 0, 1023, TEMPO_SLOW, TEMPO_FAST);

    if(! digitalRead(POKE_BUTTON)) {
      poke = map(analogRead(NOTE_KNOB), 0, 1023, 15, 0);
      pattern[i] = poke;

      scale = map(analogRead(SCALE_KNOB), 0, 1023, 10, 500);
      pattern_scale[i] = scale;
      
      #if SERIAL_DEBUG 
      Serial.print("mod ");
      Serial.print(i, DEC);
      Serial.print(" to ");
      Serial.print(scale, DEC);
      Serial.print(":");
      Serial.println(poke, DEC);
      #endif
      
    } else {
      poke = pattern[i];
      scale = pattern_scale[i];
    }
    
    #if SERIAL_DEBUG
    Serial.print("play ");
    Serial.print(i, DEC);
    Serial.print(" as ");
    Serial.println(poke);
    #endif
    
    note = majScale[poke] / scale;
    freqout((int) note, tempo);
    
    // the stopped tempo is magical, it never progresses
    if(tempo == TEMPO_SLOW) i++;
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
