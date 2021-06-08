.PHONY: clean flash disasm fuses

file = amulet
device = attiny861
F_CPU=8000000UL


%.o : %.c
	avr-gcc -c -DF_CPU=$(F_CPU) -mmcu=$(device) -Wall -std=gnu99 -O2 $< -o $(@F)

%.o : %.S
	avr-gcc -c -DF_CPU=$(F_CPU) -mmcu=$(device) -x assembler-with-cpp $< -o $(@F)

$(file).elf: $(file).o i2cmaster.o
	avr-gcc -mmcu=$(device) $^ -o $(@F)

$(file).hex: makefile $(file).elf
	avr-objcopy -j .text -j .data -O ihex $(file).elf $(file).hex

clean:
	rm -f $(file).hex $(file).elf *.o

flash: $(file).hex
	avrdude -c usbasp -p $(device) -U flash:w:$(file).hex:i

disasm:	$(file).elf
	avr-objdump -d $(file).elf

fuses:
	avrdude -c usbasp -p $(device) -U hfuse:w:0xdf:m -U lfuse:w:0xe2:m -B10