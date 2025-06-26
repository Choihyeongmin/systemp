obj-m := speed_tx_drv.o speed_rx_drv.o
KDIR  ?= /lib/modules/$(shell uname -r)/build
all:
\t$(MAKE) -C $(KDIR) M=$(PWD) modules
clean:
\t$(MAKE) -C $(KDIR) M=$(PWD) clean
