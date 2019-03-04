INCLUDE_DIR=../include
CC=g++
CFLAGS=-I$(INCLUDE_DIR) -lm

ODIR=.
LDIR=../lib

_DEPS=gpioDevice.h motors.h sensors.h robot.h PID.h 
DEPS=$(patsubst %,$(INCLUDE_DIR)/%,$(_DEPS))

_OBJ=main.o gpioDevice.o motors.o sensors.o PID.o robot.o
OBJ=$(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.cc $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCLUDE_DIR)/*~
	rm main -f
