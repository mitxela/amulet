#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "i2cmaster.h"

#define VEML6075_ADDR 0x20

// PA0: PA3 - Anodes
// PA4, PA5 - bar graph cathodes
// PA6      - mode indicator cathode
// PA7      - power to sensor

// PB2      - button

#define MATLEN 6

// 15 seconds
#define POWEROFF_TIMEOUT 150

unsigned char volatile matrix[MATLEN]={0};
unsigned char mode = 0;
unsigned char button = 0;
unsigned char timeout = 0;

ISR(TIM0_OVF_vect) {
  static unsigned char idx = 0;

  PORTA = matrix[idx];
  if (++idx == MATLEN) idx = 0;
}

ISR(EXT_INT0_vect) {
  GIMSK &= ~(1<<INT0);
  mode = 0;
  button = 255;
  timeout=0;
}

void set_bar_graph(unsigned char v) {
  unsigned char a = 0b11100000, b = 0b11010000;
  if (v>=16+8) a|=1;
  if (v>=48+8) a|=2;
  if (v>=80+8) a|=4;
  if (v>=112+8) a|=8;

  if (v>=144+8) b|=1;
  if (v>=176+8) b|=2;
  if (v>=208+8) b|=4;
  if (v>=240+8) b|=8;
  matrix[0]=a;
  matrix[1]=b;

  a = 0b11100000; b = 0b11010000;
  if (v>=16-8) a|=1;
  if (v>=48-8) a|=2;
  if (v>=80-8) a|=4;
  if (v>=112-8) a|=8;

  if (v>=144-8) b|=1;
  if (v>=176-8) b|=2;
  if (v>=208-8) b|=4;
  if (v>=240-8) b|=8;
  matrix[3]=a;
  matrix[4]=b;
}

float read_reg(unsigned char reg) {

  i2c_start(VEML6075_ADDR+I2C_WRITE);
  i2c_write(reg);
  i2c_rep_start(VEML6075_ADDR+I2C_READ);
  unsigned char lsb = i2c_readAck();
  unsigned char msb = i2c_readNak();
  i2c_stop();

  return (float)(lsb + (msb<<8));
}

int main(void) {

  PRR = 0b00001011;
  ACSR = 1<<ACD; // Analog comparator disable
  ADCSRA &= ~(1<<ADEN);
  DDRA = 0xFF;
  PORTA = 0xF0;
  DDRB = 0;
  //PORTB = 1<<6;

  TCCR0B = 2; // prescaler /8 - overflow 3906Hz
  TIMSK0 = (1<<TOIE0);

  sei();


  while (1) {
    if (mode == 0) {
      #define animDelay 30

      for (unsigned char j=MATLEN;j--;) matrix[j]=0x80;

      matrix[0]=0b11100001;
      _delay_ms(animDelay);
      matrix[0]=0b11100010;
      _delay_ms(animDelay);
      matrix[0]=0b11100100;
      _delay_ms(animDelay);
      matrix[0]=0b11101000;
      _delay_ms(animDelay);
      matrix[0]=0x80;
      matrix[1]=0b11010001;
      _delay_ms(animDelay);
      matrix[1]=0b11010010;
      _delay_ms(animDelay);
      matrix[1]=0b11010100;
      _delay_ms(animDelay);
      matrix[1]=0b11011000;
      _delay_ms(animDelay*2);

      i2c_init();
      if ( i2c_start(VEML6075_ADDR+I2C_WRITE) ) {
        i2c_stop();
        matrix[0]=0b11101010;
        matrix[1]=0b11011010;
        while (1);
      }
      i2c_write(0);
      i2c_write(0x10); // power on, int time 100ms
      i2c_stop();

      matrix[1]=0b11010100;
      _delay_ms(animDelay);
      matrix[1]=0b11010010;
      _delay_ms(animDelay);
      matrix[1]=0b11010001;
      _delay_ms(animDelay);
      matrix[1]=0x80;
      matrix[0]=0b11101000;
      _delay_ms(animDelay);
      matrix[0]=0b11100100;
      _delay_ms(animDelay);
      matrix[0]=0b11100010;
      _delay_ms(animDelay);
      matrix[0]=0b11100001;
      _delay_ms(animDelay);
      matrix[0]=0x80;

      mode=1;
    } else _delay_ms(100);

    float uva, uvb, uvcomp1, uvcomp2, uva_calc=0, uvb_calc=0;

    #define c_a 2.22
    #define c_b 1.33
    #define c_c 2.95
    #define c_d 1.74
    #define c_resp_uva 0.001461
    #define c_resp_uvb 0.002591

    #define UVI_256 23.3

    while ((PINB & (1<<2)) ==0) { //button held
      // freeze display
      _delay_ms(10);
      timeout = 0;
      button++; if (button==0) button--;
    }
    if (button >0) { // button just released
      if (button <50) { // was held for less than 500ms
        mode++;
        if (mode>3) mode=0;
      }
      button =0;
    }

    timeout++;
    if (timeout >= POWEROFF_TIMEOUT) {
      mode = 0;
      timeout=254;
    }

    uva = read_reg(0x07);
    uvb = read_reg(0x09);
    uvcomp1 = read_reg(0x0A);
    uvcomp2 = read_reg(0x0B);

    uva_calc = uva - c_a*uvcomp1 - c_b*uvcomp2;
    uvb_calc = uvb - c_c*uvcomp1 - c_d*uvcomp2;

    if (uva_calc<0.0) uva_calc = 0;
    if (uvb_calc<0.0) uvb_calc = 0;


    if (mode == 1) {
      matrix[2]=0b10110001;
      float uvi = ( uva_calc*c_resp_uva + uvb_calc*c_resp_uvb )*0.5*UVI_256;
      if (uvi >254.0) uvi=254.0;
      set_bar_graph( (unsigned char)( uvi ) );

    } else if (mode == 2) {
      matrix[2]=0b10110010;
      float out = uva_calc * c_resp_uva * UVI_256;
      if (out>254.0) out=254.0;
      set_bar_graph( (unsigned char)( out ) );

    } else if (mode == 3) {
      matrix[2]=0b10110100;
      float out = uvb_calc * c_resp_uvb * UVI_256;
      if (out>254.0) out=254.0;
      set_bar_graph( (unsigned char)( out ) );

    } else {
      matrix[0]=0b11101111;
      matrix[1]=0b11011111;
      matrix[2]=0x80;
      _delay_ms(200);

      PRR |= (1<< PRTIM0); // timer off
      for (int j=0;j<6;j++) matrix[j]=0;
      PORTA =0;
      DDRA = 0;
      DDRB = 0;
      PORTB = 0;

      // MCUCR 0 default for low-level triggered
      GIMSK |= (1<<INT0);
      MCUCR |= (1<<SE) | (1<<SM1);
      asm("sleep");

      timeout=0;
      DDRA = 0xFF;
      PRR &= ~(1<< PRTIM0);
    }

  }
}