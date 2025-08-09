obj-m += cpufreq_laputil.o

KVERSION = $(shell uname -r)

KDIR = /lib/modules/$(KVERSION)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
