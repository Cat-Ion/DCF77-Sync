MAKEFILE      = Makefile

MCU = STM32F103C8
FAMILY = STM32F1

####### Compiler, tools and options

CC            = arm-none-eabi-gcc
CXX           = arm-none-eabi-g++
DEFINES       = -D$(MCU) -D$(FAMILY)
COMMONFLAGS  += -pipe -mcpu=cortex-m3 -mthumb -msoft-float -g -Wall -W -D_REENTRANT $(DEFINES)
CFLAGS       += $(COMMONFLAGS) -std=c99  
CXXFLAGS     += $(COMMONFLAGS) -std=c++17 -fno-exceptions -fno-rtti
INCPATH       = -I. -Ilibopencm3/include -IFixedPoint/include
LINK          = arm-none-eabi-g++
LFLAGS        = --static -nostartfiles -Tgenerated.$(MCU).ld -mcpu=cortex-m3 -mthumb -msoft-float -Wl,-Map=main.map -Wl,--gc-sections -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group --specs=nosys.specs -fno-rtti
LIBS          = $(SUBLIBS) -Llibopencm3/lib/ -lopencm3_stm32f1
####### Output directory

OBJECTS_DIR   = ./

####### Files

SOURCES       = main.cpp usb.c ringbuffer.c
OBJECTS       = main.o usb.o ringbuffer.o
DESTDIR       = 
TARGET        = DCF77-Firmware


all: Makefile $(TARGET)
####### Build rules

$(TARGET): Makefile $(OBJECTS) generated.$(MCU).ld 
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

clean: 
	rm -f $(OBJECTS) generated.$(MCU).ld

distclean: clean 
	rm -f $(TARGET) 

####### Compile

main.o: Makefile main.cpp Smoother.hpp PID.hpp SigmaDelta.hpp ringbuffer.h usb.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o main.o main.cpp

usb.o: Makefile usb.c usb.h ringbuffer.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o usb.o usb.c

ringbuffer.o: Makefile ringbuffer.c ringbuffer.h
	$(CC) -c $(CFLAGS) $(INCPATH) -o ringbuffer.o ringbuffer.c

generated.$(MCU).ld: Makefile
	$(CXX) -E $(CFLAGS) -D_ROM=64K -D_RAM=20K -D_ROM_OFF=0x08000000 -D_RAM_OFF=0x20000000  -P -E libopencm3//ld/linker.ld.S > $@
