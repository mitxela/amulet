#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "i2cmaster.h"

#define VEML6075_ADDR 0x20

// PA0: PA3 - Anodes
// PA4, PA5 - bar graph cathodes
// PA6      - mode indicator cathode

#define MATLEN 24

unsigned char matrix[MATLEN]={0};

// TIM0_OVF for attiny84
ISR(TIMER0_OVF_vect) {
  static unsigned char idx = 0;

  PORTA = matrix[idx];
  if (++idx == MATLEN) idx = 0;
}

void set_bar_graph(unsigned char v) {
  unsigned char a = 0b11100000, b = 0b11010000;
  if (v>=16) a|=1;
  if (v>=48) a|=2;
  if (v>=80) a|=4;
  if (v>=112) a|=8;

  if (v>=144) b|=1;
  if (v>=176) b|=2;
  if (v>=208) b|=4;
  if (v>=240) b|=8;
  matrix[0]=a;
  matrix[1]=b;
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

  DDRA = 0xFF;
  PORTA = 0xF0;

  TCCR0B = 2; // prescaler /8 - overflow 3906Hz
  TIMSK = (1<<TOIE0);

  set_bar_graph(0);
  sei();



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

  _delay_ms(101);

  while (1) {
    //unsigned short uva = read_reg(0x07);
    //unsigned short uvb = read_reg(0x09);
    //unsigned short uvcomp1 = read_reg(0x0A);
    //unsigned short uvcomp2 = read_reg(0x0B);

    #define c_a 2.22
    #define c_b 1.33
    #define c_c 2.95
    #define c_d 1.74
    #define c_resp_uva 0.001461
    #define c_resp_uvb 0.002591

    //float uva_calc = (float)uva - c_a*(float)uvcomp1 - c_b*(float)uvcomp2;
    //float uvb_calc = (float)uvb - c_c*(float)uvcomp1 - c_d*(float)uvcomp2;


    float uva = read_reg(0x07);
    float uvb = read_reg(0x09);
    float uvcomp1 = read_reg(0x0A);
    float uvcomp2 = read_reg(0x0B);

    float uva_calc = uva - c_a*uvcomp1 - c_b*uvcomp2;
    float uvb_calc = uvb - c_c*uvcomp1 - c_d*uvcomp2;

    if (uva_calc<0.0) uva_calc = 0;
    if (uvb_calc<0.0) uvb_calc = 0;

    float uvi = ( uva_calc*c_resp_uva + uvb_calc*c_resp_uvb )*0.5;

    set_bar_graph( (char)( uvi*25 ) );
    _delay_ms(100);
  }


}