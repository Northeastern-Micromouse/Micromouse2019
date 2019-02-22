INCLUDE_DIR=../include
CC=g++
CFLAGS=-I$(INCLUDE_DIR)

ODIR=obj
LDIR=../lib

_DEPS=i2cDevice.h gpioDevice.h rgbLedDevice.h motors.h ledDriver.h robot.h PID.h reflSensor.h imu.h Direction.h AlgoRobot.h Cell.h Maze.h
DEPS=$(patsubst %,$(INCLUDE_DIR)/%,$(_DEPS))

_OBJ=i2cDevice.o main.o gpioDevice.o rgbLedDevice.o motors.o ledDriver.o PID.o robot.o reflSensor.o imu.o Maze.o Cell.o AlgoRobot.o
OBJ=$(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.cc $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

main: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCLUDE_DIR)/*~
	rm main -f
