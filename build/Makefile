CC = avr-gcc

CPU = atmega328p

CFLAGS = -mmcu=$(CPU)
CSRCFLAGS = -Os -DF_CPU=16000000UL

SRC_DIR = ../src
HDR_DIR = ../include
OBJ_DIR = objects
SOURCES = $(wildcard $(SRC_DIR)/*.c)
OBJECTS = $(patsubst $(SRC_DIR)/%.c, $(OBJ_DIR)/%.o, $(SOURCES))

EXEC = main
HEX = $(EXEC).hex

build: $(OBJECTS) $(HEADERS)
	echo $(OBJECTS)
	$(CC) $(CFLAGS) -o $(EXEC) $(OBJECTS)
	avr-objcopy -O ihex -R .eeprom $(EXEC) $(HEX)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	mkdir -p $(@D)
	$(CC) -I $(HDR_DIR) $(CSRCFLAGS) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR)
	rm $(EXEC) $(HEX)

upload: build
	avrdude -F -V -c arduino -p $(CPU) -P /dev/ttyACM0 -b 115200 -U flash:w:$(HEX)
