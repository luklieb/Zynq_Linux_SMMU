
# Compile direclty on Ultra96
# Kernel sources must be installed first

obj-m += mod.o

all: unload module load


module: mod.c
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean

unload:
	rmmod mod

load:
	insmod mod.ko
