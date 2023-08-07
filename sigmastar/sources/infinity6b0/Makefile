EXTRA_CFLAGS := -I $(PWD)/include -I $(KSRC)/drivers/sstar/include -D SENSOR_MODULE_VERSION=$(SENSOR_VERSION)
obj-m := $(patsubst $(PWD)/%.c, %.o, $(wildcard $(PWD)/sensor/*.c))

modules:
	$(MAKE) -C $(KSRC) M=$(PWD) modules

clean:
	$(MAKE) -C $(KSRC) M=$(PWD) clean
