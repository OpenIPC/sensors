ifeq ($(SENSOR_VERSION),infinity6c)
	EXTRA_CFLAGS += -D SIGMASTAR_MARUKO
endif

EXTRA_CFLAGS += -I $(PWD)/include -I $(KSRC)/drivers/sstar/include -D SENSOR_MODULE_VERSION=$(SENSOR_VERSION)
obj-m := $(if $(OPENIPC_SNS_MODEL), sensor_config.o sensor_$(OPENIPC_SNS_MODEL)_mipi.o, \
	$(patsubst $(PWD)/%.c, %.o, $(wildcard $(PWD)/*.c)))

modules:
	$(MAKE) -C $(KSRC) M=$(PWD) modules
