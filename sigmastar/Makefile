ifeq ($(SENSOR_VERSION),infinity6c)
	EXTRA_CFLAGS += -D SIGMASTAR_MARUKO
endif

EXTRA_CFLAGS += -I $(PWD)/include -I $(KSRC)/drivers/sstar/include -D SENSOR_MODULE_VERSION=$(SENSOR_VERSION)
obj-m := sensor_config.o $(patsubst $(PWD)/%.c,%.o,$(wildcard $(PWD)/*$(OPENIPC_SNS_MODEL)*.c))

modules:
	$(MAKE) -C $(KSRC) M=$(PWD) modules
