# mecanum_stm32 — STM32F405 네이티브(CMSIS 레지스터) 펌웨어
#   make          → build/mecanum_stm32.{elf,bin,hex} + 크기
#   make test     → PC에서 core·cfgstore 시험(호스트 C 컴파일러 HOSTCC, 기본 cc)
#                    Windows: make test HOSTCC="python -m ziglang cc"   (pip install ziglang)
#   make clean
PREFIX ?= arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size
HOSTCC ?= cc

TARGET = build/mecanum_stm32
SRCS   = src/core.c src/cfgstore.c src/hw.c
ASMS   = vendor/cmsis/startup_stm32f405xx.s

ARCH   = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS = $(ARCH) -std=c11 -O2 -g3 -Wall -Wextra -Wshadow -Wdouble-promotion -Wformat=2 -Werror \
         -fno-common -ffunction-sections -fdata-sections -fsingle-precision-constant \
         -DSTM32F405xx -Isrc -Ivendor/cmsis
LDFLAGS = $(ARCH) -T stm32f405rg.ld -nostartfiles --specs=nano.specs --specs=nosys.specs \
          -Wl,--gc-sections -Wl,-Map=$(TARGET).map -Wl,--print-memory-usage

all: $(TARGET).bin $(TARGET).hex

build:
	mkdir -p build

$(TARGET).elf: $(SRCS) $(ASMS) src/*.h stm32f405rg.ld | build
	$(CC) $(CFLAGS) $(SRCS) $(ASMS) $(LDFLAGS) -lm -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

ifeq ($(OS),Windows_NT)
EXE = .exe
endif
test: | build
	$(HOSTCC) -std=c11 -O1 -g -Wall -Wextra -Werror -Isrc -o build/test_core$(EXE) test/test_core.c test/fake_hw.c src/core.c src/cfgstore.c -lm
	./build/test_core$(EXE)

clean:
	rm -rf build

.PHONY: all test clean
