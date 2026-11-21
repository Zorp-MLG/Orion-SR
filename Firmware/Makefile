PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
CXX     = $(PREFIX)g++
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

CPU      = -mcpu=cortex-m0plus -mthumb
COMMON   = $(CPU) -Os -Wall -ffunction-sections -fdata-sections -ffreestanding
CFLAGS   = $(COMMON) -std=gnu11
CXXFLAGS = $(COMMON) -std=gnu++14 -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit
LDFLAGS  = $(CPU) -T link.ld -nostartfiles -nostdlib -Wl,--gc-sections -Wl,-Map=orion.map

# add your own .cpp files here
OBJS = startup.o orion_core.o orion_usb.o sketch.o

all: orion.bin
	$(SIZE) orion.elf

orion.elf: $(OBJS) link.ld
	$(CXX) $(LDFLAGS) $(OBJS) -lgcc -o $@

orion.bin: orion.elf
	$(OBJCOPY) -O binary $< $@

%.o: %.c orion.h
	$(CC) $(CFLAGS) -c $< -o $@
%.o: %.cpp orion.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Flash over USB: put the board in the ROM bootloader (hold BOOT, tap NRST) or send 1200 baud, then:
dfu: orion.bin
	dfu-util -a 0 -s 0x08000000:leave -D orion.bin

# Or flash over SWD with an ST-Link:
flash: orion.elf
	openocd -f interface/stlink.cfg -f target/stm32g0x.cfg -c "program orion.elf verify reset exit"

clean:
	rm -f *.o orion.elf orion.bin orion.map

.PHONY: all dfu flash clean
