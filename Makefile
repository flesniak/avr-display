BIN = main
SRC = main.c usiTwiSlave.c
HDR = types.h usiTwiSlave.h
# BIN = isrtest
# SRC = isrtest.c

MCU = attiny261a
CFLAGS = -g -Os -Wall -Wextra -std=gnu99 -DF_CPU=$(F_CPU) -mmcu=$(MCU) -fshort-enums -fdata-sections -ffunction-sections -mtiny-stack -Wl,--gc-sections
AVRDUDE_MCU = t261
AVRDUDE_PROG = usbasp

F_CPU = 8000000
HFUSE = 0xdf
LFUSE = 0xe2

HEX = $(BIN:%=%.hex)
OBJ = $(SRC:%.c=%.o)

.PHONY: all flash fuses clean

all: $(HEX)

$(BIN): $(OBJ)
	avr-gcc $(CFLAGS) -o $(BIN) $^

%.o: %.c $(HDR)
	avr-gcc $(CFLAGS) -c -o $@ $<

$(HEX): $(BIN)
	avr-objcopy -O ihex $(BIN) $(HEX)
	avr-size $(BIN)

flash: $(HEX)
	avrdude -p $(AVRDUDE_MCU) -c $(AVRDUDE_PROG) -eU flash:w:$(HEX)

fuses:
	avrdude -p $(AVRDUDE_MCU) -c $(AVRDUDE_PROG) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m

clean:
	rm -f $(BIN) $(HEX) $(OBJ)
