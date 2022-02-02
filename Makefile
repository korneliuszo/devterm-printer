BINARY     := mtp02
KERNEL      ?= /lib/modules/$(shell uname -r)/build
ARCH        ?= x86
C_FLAGS     ?= -Wall
KMOD_DIR    ?= $(shell pwd)
#TARGET_PATH = /lib/modules/$(shell uname -r)/kernel/drivers/char

OBJECTS := main.o

ccflags-y += $(C_FLAGS)

obj-m += $(BINARY).o

dtbo-y +=  mtp02-a06-devterm.dtbo

targets += $(dtbo-y)
always-y  := $(dtbo-y)

$(BINARY)-y := $(OBJECTS)

all:
	make -C $(KERNEL) M=$(KMOD_DIR) modules

clean:
	make -C $(KERNEL) M=$(KMOD_DIR) clean
