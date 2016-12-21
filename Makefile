NAME := LoRaMac-node

CC       = arm-none-eabi-gcc
CXX      = arm-none-eabi-g++
PLATFORM = EFM32GG_STK3700
DEVICE   = EFM32GG990F1024
MCU      = efm32gg
BUILD    = Debug

C_SRCS := \
src/system/adc.c \
src/system/delay.c \
src/system/eeprom.c \
src/system/fifo.c \
src/system/gpio.c \
src/system/gps.c \
src/system/i2c.c \
src/system/timer.c \
src/system/uart.c \
src/system/crypto/aes.c \
src/system/crypto/cmac.c \
src/radio/sx1276/sx1276.c \
src/mac/LoRaMac.c \
src/mac/LoRaMacCrypto.c \
src/boards/EFM32GG_STK3700/adc-board.c \
src/boards/EFM32GG_STK3700/board.c \
src/boards/EFM32GG_STK3700/eeprom-board.c \
src/boards/EFM32GG_STK3700/gpio-board.c \
src/boards/EFM32GG_STK3700/gps-board.c \
src/boards/EFM32GG_STK3700/i2c-board.c \
src/boards/EFM32GG_STK3700/rtc-board.c \
src/boards/EFM32GG_STK3700/spi-board.c \
src/boards/EFM32GG_STK3700/sx1276-board.c \
src/boards/EFM32GG_STK3700/uart-board.c \
src/boards/EFM32GG_STK3700/uart-usb-board.c \
src/boards/mcu/efm32gg/utilities.c \
src/boards/mcu/efm32gg/CMSIS/system_efm32gg.c \
src/boards/mcu/efm32gg/emlib/src/em_assert.c \
src/boards/mcu/efm32gg/emlib/src/em_cmu.c \
src/boards/mcu/efm32gg/emlib/src/em_emu.c \
src/boards/mcu/efm32gg/emlib/src/em_gpio.c \
src/boards/mcu/efm32gg/emlib/src/em_lcd.c \
src/boards/mcu/efm32gg/emlib/src/em_rtc.c \
src/boards/mcu/efm32gg/emlib/src/em_system.c \
src/boards/mcu/efm32gg/emlib/src/em_timer.c \
src/boards/mcu/efm32gg/emlib/src/em_usart.c \
src/apps/LoRaMac/classA/EFM32GG_STK3700/main.c

INCLUDE_DIRS := \
src \
src/system \
src/system/crypto \
src/mac \
src/radio \
src/boards/$(PLATFORM) \
src/boards/mcu/$(MCU) \
src/boards/mcu/$(MCU)/emlib/inc \
src/boards/mcu/$(MCU)/Device \
src/boards/mcu/$(MCU)/CMSIS/include

C_OBJS := ${C_SRCS:.c=.o}
OBJS := $(C_OBJS) startup_efm32gg.o

CFLAGS += $(foreach includedir,$(INCLUDE_DIRS),-I$(includedir))
CFLAGS += -D$(DEVICE)=1
CFLAGS += -DUSE_BAND_868=1

ifeq ($(BUILD),Debug)
CFLAGS += -DDEBUG=1
endif

.PHONY: all clean distclean

all: $(NAME)

%.o: %.c
	$(CC) $(CFLAGS) -c -g -gdwarf-2 -mcpu=cortex-m3 -mthumb -O0 -Wall -fmessage-length=0 -mno-sched-prolog -fno-builtin -ffunction-sections -fdata-sections -std=c99 -MMD -MP -MF"${@:.o=.d}" -MT"$@" -o "$@" "$<"

startup_efm32gg.o: src/boards/mcu/efm32gg/CMSIS/startup_gcc_efm32gg.s
	$(CC) -g -gdwarf-2 -mcpu=cortex-m3 -mthumb -x assembler-with-cpp -c -o "$@" "$<"

$(NAME): $(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C++ Linker'
	$(CXX) -g -gdwarf-2 -mcpu=cortex-m3 -mthumb -T "$(NAME).ld" --specs=nosys.specs -Xlinker --gc-sections -Xlinker -Map="$(NAME).map" --specs=nano.specs -o $(NAME).axf $(OBJS) -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: $(NAME).hex'
	arm-none-eabi-objcopy -O ihex "$(NAME).axf" "$(NAME).hex"
	@echo ' '

	@echo 'Building bin file: $(NAME).bin'
	arm-none-eabi-objcopy -O binary "$(NAME).axf" "$(NAME).bin"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "$(NAME).axf"
	@echo ' '

clean:
	@- $(RM) $(NAME).axf
	@- $(RM) $(OBJS) ${OBJS:.o=.d}

distclean: clean
