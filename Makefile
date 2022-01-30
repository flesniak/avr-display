BIN = main
SRC = main.c

MCU = attiny261
CFLAGS = -s -Os -Wall -Wextra -std=gnu99 -DF_CPU=$(F_CPU) -mmcu=$(MCU)
AVRDUDE_MCU = t261
AVRDUDE_PROG = usbasp

F_CPU = 8000000
HFUSE = 0xdf
LFUSE = 0xe2

HEX = $(BIN).hex
OBJ = $(SRC:%.c=%.o)

.PHONY: all flash fuses clean

all: $(HEX)

$(BIN): $(OBJ)
	avr-gcc $(CFLAGS) -o $(BIN) $^

%.o: %.c
	avr-gcc $(CFLAGS) -c -o $@ $^

$(HEX): $(BIN)
	avr-objcopy -O ihex $(BIN) $(HEX)

flash: $(HEX)
	avrdude -p $(AVRDUDE_MCU) -c $(AVRDUDE_PROG) -eU flash:w:$(HEX)

fuses:
	avrdude -p $(AVRDUDE_MCU) -c $(AVRDUDE_PROG) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m

clean:
	rm -f $(BIN) $(HEX) $(OBJ)
