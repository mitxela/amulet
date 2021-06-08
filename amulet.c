#include <avr/io.h>
#include <util/delay.h>
#include "i2cmaster.h"

#define VEML6075_ADDR 0x20

unsigned short read_reg(unsigned char reg) {

  i2c_start(VEML6075_ADDR+I2C_WRITE);
  i2c_write(reg);
  i2c_rep_start(VEML6075_ADDR+I2C_READ);
  unsigned char lsb = i2c_readAck();
  unsigned char msb = i2c_readNak();
  i2c_stop();

  return lsb + (msb<<8);
}

int main(void) {

  DDRA = 0xFF;
  PORTA = 0xFF;

  i2c_init();

  if ( i2c_start(VEML6075_ADDR+I2C_WRITE) ) {
    i2c_stop();
    PORTA = 0x55;
    while (1);
  }


  PORTA = 0;

  i2c_write(0);
  i2c_write(0x10); // power on, int time 100ms
  i2c_stop();

  _delay_ms(101);

  while (1) {
    PORTA = (char)read_reg(0x07);
    _delay_ms(100);
  }


}