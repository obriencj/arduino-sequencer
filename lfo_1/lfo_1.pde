/*


WIRING:
  D8 : Serial data for DAC A
  D9 : Serial data for DAC B
  D10 : Clock to DAC and LED serial-to-parallel
  D11 : Latch to DAC and LED serial-to-parallel
  A0 : center pole on potentiometer for super frequency divider
  A1 : center pole on potentiometer for frequency A
  A2 : center pole on potentiometer for frequency B

  Looking down at the potentiometers, with the contacts to the North:
  Left contact is +5V, center contact is the analog pin, Right contact
  is Ground.

author: Christopher O'Brien  <obriencj@gmail.com>

*/


#include <avr/interrupt.h>


// how often we update the DAC, in hertz
#define SAMPLE_RATE 10000



#define SERIAL_DEBUG 0
#define SERIAL_BAUD 115200

#if SERIAL_DEBUG
#define Debug_begin() Serial.begin(SERIAL_BAUD);
#define Debug_print(a...) Serial.print(a)
#define Debug_println(a...) Serial.println(a)
#define Debug_delay(a) delay(a)
#define Debug(a,b...) (a(b))
#else
#define Debug_begin(...)
#define Debug_print(...)
#define Debug_println(...)
#define Debug_delay(...)
#define Debug(...)
#endif



/* a 256 sampled 8 bit sine wave. Why so many samples? Because we're
   going to be doing some weird jumping around in this table at most
   frequencies, and the greater the resolution inside the table, the
   less like crap our output looks */
static const byte sine_table[] = {
  0, 0, 0, 0, 0, 0, 1, 1,
  2, 3, 3, 4, 5, 6, 7, 8,
  9, 10, 12, 13, 14, 16, 18, 19,
  21, 23, 24, 26, 28, 30, 32, 35,
  37, 39, 41, 44, 46, 48, 51, 53,
  56, 59, 61, 64, 67, 69, 72, 75,
  78, 81, 84, 87, 90, 93, 96, 99,
  102, 105, 108, 111, 114, 117, 120, 123,
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
  2, 1, 1, 0, 0, 0, 0, 0 };



static const byte saw_table[] = {
  0, 1, 2, 3, 4, 5, 6, 7,
  8, 9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27, 28, 29, 30, 31,
  32, 33, 34, 35, 36, 37, 38, 39,
  40, 41, 42, 43, 44, 45, 46, 47,
  48, 49, 50, 51, 52, 53, 54, 55,
  56, 57, 58, 59, 60, 61, 62, 63,
  64, 65, 66, 67, 68, 69, 70, 71,
  72, 73, 74, 75, 76, 77, 78, 79,
  80, 81, 82, 83, 84, 85, 86, 87,
  88, 89, 90, 91, 92, 93, 94, 95,
  96, 97, 98, 99, 100, 101, 102, 103,
  104, 105, 106, 107, 108, 109, 110, 111,
  112, 113, 114, 115, 116, 117, 118, 119,
  120, 121, 122, 123, 124, 125, 126, 127,
  128, 129, 130, 131, 132, 133, 134, 135,
  136, 137, 138, 139, 140, 141, 142, 143,
  144, 145, 146, 147, 148, 149, 150, 151,
  152, 153, 154, 155, 156, 157, 158, 159,
  160, 161, 162, 163, 164, 165, 166, 167,
  168, 169, 170, 171, 172, 173, 174, 175,
  176, 177, 178, 179, 180, 181, 182, 183,
  184, 185, 186, 187, 188, 189, 190, 191,
  192, 193, 194, 195, 196, 197, 198, 199,
  200, 201, 202, 203, 204, 205, 206, 207,
  208, 209, 210, 211, 212, 213, 214, 215,
  216, 217, 218, 219, 220, 221, 222, 223,
  224, 225, 226, 227, 228, 229, 230, 231,
  232, 233, 234, 235, 236, 237, 238, 239,
  240, 241, 242, 243, 244, 245, 246, 247,
  248, 249, 250, 251, 252, 253, 254, 255 };



static const byte triangle_table[] = {
  0, 2, 4, 6, 8, 10, 12, 14,
  16, 18, 20, 22, 24, 26, 28, 30,
  32, 34, 36, 38, 40, 42, 44, 46,
  48, 50, 52, 54, 56, 58, 60, 62,
  64, 66, 68, 70, 72, 74, 76, 78,
  80, 82, 84, 86, 88, 90, 92, 94,
  96, 98, 100, 102, 104, 106, 108, 110,
  112, 114, 116, 118, 120, 122, 124, 126,
  128, 130, 132, 134, 136, 138, 140, 142,
  144, 146, 148, 150, 152, 154, 156, 158,
  160, 162, 164, 166, 168, 170, 172, 174,
  176, 178, 180, 182, 184, 186, 188, 190,
  192, 194, 196, 198, 200, 202, 204, 206,
  208, 210, 212, 214, 216, 218, 220, 222,
  224, 226, 228, 230, 232, 234, 236, 238,
  240, 242, 244, 246, 248, 250, 252, 254,
  255, 253, 251, 249, 247, 245, 243, 241,
  239, 237, 235, 233, 231, 229, 227, 225,
  223, 221, 219, 217, 215, 213, 211, 209,
  207, 205, 203, 201, 199, 197, 195, 193,
  191, 189, 187, 185, 183, 181, 179, 177,
  175, 173, 171, 169, 167, 165, 163, 161,
  159, 157, 155, 153, 151, 149, 147, 145,
  143, 141, 139, 137, 135, 133, 131, 129,
  127, 125, 123, 121, 119, 117, 115, 113,
  111, 109, 107, 105, 103, 101, 99, 97,
  95, 93, 91, 89, 87, 85, 83, 81,
  79, 77, 75, 73, 71, 69, 67, 65,
  63, 61, 59, 57, 55, 53, 51, 49,
  47, 45, 43, 41, 39, 37, 35, 33,
  31, 29, 27, 25, 23, 21, 19, 17,
  15, 13, 11, 9, 7, 5, 3, 1 };



static const byte square_table[] = {
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  255, 255, 255, 255, 255, 255, 255, 255,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0 };



/* controlling the A wave */
static unsigned int div_a = 0;
static unsigned int mult_a = 0;
static unsigned int counter_a = 0;
static unsigned int position_a = 0;
static const byte *table_a = sine_table;


/* controlling the B wave */
static unsigned int div_b = 0;
static unsigned int mult_b = 0;
static unsigned int counter_b = 0;
static unsigned int position_b = 0;
static const byte *table_b = sine_table;


#define BIT(num, bit)  (((num) >> bit) & 1)


/* The playback interrupt */
ISR(TIMER1_COMPA_vect) {

  boolean changed = false;

  if(! counter_a--) {
    position_a += mult_a;
    position_a %= 256;
    counter_a = div_a;
    changed = true;
  }

  if(! counter_b--) {
    position_b += mult_b;
    position_b %= 256;
    counter_b = div_b;
    changed = true;
  }

  if(changed) {

    byte A = table_a[position_a];
    byte B = table_b[position_b];

    for(int i = 8; i--; ) {
      // clock and latch low, bis of A and B channels
      PORTB = (PORTB & 0xf0) | (BIT(B, i) << 1) | BIT(A, i);

      // clock on rising edge
      PORTB |= 0x4;
    }

    // latch on rising edge
    PORTB |= 0x8;
  }
}



static void freq_conversion(float frequency, unsigned int *multiplier, unsigned int *divider) {
  unsigned int m, d;
  float s = (SAMPLE_RATE) / (frequency * 256);
  
  m = (1.0 / s);
  d = s;
  
  Debug_print("freq ");
  Debug_print(frequency, DEC);
  Debug_print(" = div:");
  Debug_print(d, DEC);
  Debug_print(", mult:");
  Debug_println(m, DEC);
  Debug_delay(500);
  
  *multiplier = m || 1;
  *divider = d;
}



void setup() {
  
  Debug_begin();
  Debug_delay(1000);
  Debug_println("in setup");
  
  cli();
  
  freq_conversion(1.2, &mult_a, &div_a);
  counter_a = div_a;
  table_a = saw_table;

  freq_conversion(5.0, &mult_b, &div_b);
  counter_b = div_b;
  table_b = triangle_table;
  
  Debug_println("setting up Timer1");
  
  // this madness sets up our Timer 1 interrupt frequency.
  // thanks to Michael Smith
  // http://www.arduino.cc/playground/Code/PCMAudio
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  OCR1A = F_CPU / SAMPLE_RATE;
  TIMSK1 |= _BV(OCIE1A);
  
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
  unsigned int div_t = 0, mult_t = 0;
  int super_tempo;
  float freq = 0.0;
  
  Debug_println("entering loop");
  
  /* normally we'd want to divide the freq input by 100, which is full right on this knob. */
  //super_tempo = map(analogRead(0), 0, 1023, 400, 100);
  super_tempo = 100;
  
  /* between 0.10 Hz, and 10 Hz */
  freq = map(analogRead(1), 0, 1023, 10, 1000);
  freq_conversion(freq / super_tempo, &mult_a, &div_a);

  /* todo: set table_a */

  /* between 0.10 Hz, and 10 Hz */
  freq = map(analogRead(2), 0, 1023, 10, 1000);
  freq_conversion(freq / super_tempo, &mult_b, &div_b);
  
  /* todo: set table_b */
  
  Debug_println("finished loop");
  Debug_delay(500);
}



/* The end. */
