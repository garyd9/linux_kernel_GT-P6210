# arch/arm/mach-s5pv310/Makefile
#
# Copyright (c) 2010 Samsung Electronics Co., Ltd.
#		http://www.samsung.com/
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

# Core support for S5PV310 system

obj-$(CONFIG_CPU_S5PV310)	+= cpu.o init.o clock.o irq-combiner.o gpiolib.o irq-eint.o
obj-$(CONFIG_CPU_S5PV310)	+= setup-i2c0.o dma.o dmc.o
obj-$(CONFIG_SAMSUNG_IRQ_GPIO)  += irq-gpio.o

obj-y				+= mct.o

obj-$(CONFIG_S5PV310_PM)	+= pm.o sleep.o

obj-$(CONFIG_SMP)		+= platsmp.o headsmp.o
obj-$(CONFIG_HOTPLUG_CPU)	+= hotplug.o
obj-$(CONFIG_CPU_FREQ)		+= cpufreq.o cpu_ppmu.o
obj-$(CONFIG_CPU_IDLE)		+= cpuidle.o
obj-$(CONFIG_S5P_MEM_BOOTMEM)	+= bootmem-smdkv310.o
obj-$(CONFIG_S5PV310_PM_HOTPLUG)	+= pm-hotplug.o
obj-$(CONFIG_SAMSUNG_PHONE_SVNET)	+= c1-modem.o

# machine support

obj-$(CONFIG_MACH_SMDKC210)	+= mach-smdkc210.o button-smdkc210.o
obj-$(CONFIG_MACH_SMDKV310)	+= mach-smdkv310.o button-smdkv310.o
obj-$(CONFIG_MACH_UNIVERSAL_C210)	+= mach-universal_c210.o
obj-$(CONFIG_MACH_C1)		+= c1-panel.o
obj-$(CONFIG_MACH_C1)		+= c1-regulator-consumer.o

obj-$(CONFIG_MACH_C1_REV02)	+= mach-c1.o

obj-$(CONFIG_MACH_P4W_REV00)	+= mach-px.o
obj-$(CONFIG_MACH_P4W_REV00)	+= px-gps.o px-wlan.o px-rfkill.o px-switch.o
obj-$(CONFIG_MACH_P4W_REV00)	+= p4w-gpio.o

obj-$(CONFIG_MACH_P4W_REV01)	+= mach-px.o
obj-$(CONFIG_MACH_P4W_REV01)	+= px-gps.o px-wlan.o px-rfkill.o px-switch.o
obj-$(CONFIG_MACH_P4W_REV01)	+= p4w-gpio.o

obj-$(CONFIG_MACH_P2_REV00)	+= mach-px.o
obj-$(CONFIG_MACH_P2_REV00)	+= px-gps.o px-wlan.o px-rfkill.o px-switch.o
obj-$(CONFIG_MACH_P2_REV00)	+= p2-gpio.o

obj-$(CONFIG_MACH_P2_REV01) += mach-px.o
obj-$(CONFIG_MACH_P2_REV01) += px-gps.o px-wlan.o px-rfkill.o px-switch.o
obj-$(CONFIG_MACH_P2_REV01) += p2-gpio.o

obj-$(CONFIG_MACH_P2_REV02) += mach-px.o
obj-$(CONFIG_MACH_P2_REV02) += px-gps.o px-wlan.o px-rfkill.o px-switch.o
obj-$(CONFIG_MACH_P2_REV02) += p2-gpio.o

obj-$(CONFIG_MACH_P8_REV00)	+= mach-px.o
obj-$(CONFIG_MACH_P8_REV00)	+= px-gps.o px-wlan.o px-rfkill.o px-switch.o
obj-$(CONFIG_MACH_P8_REV00)	+= p8-gpio.o

obj-$(CONFIG_MACH_P8_REV01)	+= mach-px.o
obj-$(CONFIG_MACH_P8_REV01)	+= px-gps.o px-wlan.o px-rfkill.o px-switch.o
obj-$(CONFIG_MACH_P8_REV01)	+= p8-gpio.o

obj-$(CONFIG_MACH_P8LTE_REV00)	+= mach-px.o
obj-$(CONFIG_MACH_P8LTE_REV00)	+= px-gps.o px-wlan.o px-rfkill.o px-switch.o
obj-$(CONFIG_MACH_P8LTE_REV00)	+= p8lte-gpio.o

# device support
obj-$(CONFIG_S5PV310_SETUP_FB_S3C)		+= setup-fb.o
obj-$(CONFIG_S5PV310_SETUP_FIMC0)	+= setup-fimc0.o
obj-$(CONFIG_S5PV310_SETUP_FIMC1)	+= setup-fimc1.o
obj-$(CONFIG_S5PV310_SETUP_FIMC2)	+= setup-fimc2.o
obj-$(CONFIG_S5PV310_SETUP_FIMC3)	+= setup-fimc3.o
obj-$(CONFIG_S5PV310_SETUP_CSIS)	+= setup-csis.o
obj-$(CONFIG_S5PV310_SETUP_I2C1)	+= setup-i2c1.o
obj-$(CONFIG_S5PV310_SETUP_I2C2)	+= setup-i2c2.o
obj-$(CONFIG_S5PV310_SETUP_I2C3)	+= setup-i2c3.o
obj-$(CONFIG_S5PV310_SETUP_I2C4)	+= setup-i2c4.o
obj-$(CONFIG_S5PV310_SETUP_I2C5)	+= setup-i2c5.o
obj-$(CONFIG_S5PV310_SETUP_I2C6)	+= setup-i2c6.o
obj-$(CONFIG_S5PV310_SETUP_I2C7)	+= setup-i2c7.o
obj-$(CONFIG_S5PV310_SETUP_SDHCI)	+= setup-sdhci.o
obj-$(CONFIG_S5PV310_SETUP_SDHCI_GPIO)	+= setup-sdhci-gpio.o
obj-$(CONFIG_S5PV310_SETUP_MSHCI)	+= setup-mshci.o
obj-$(CONFIG_S5PV310_SETUP_MSHCI_GPIO)	+= setup-mshci-gpio.o
obj-$(CONFIG_S5PV310_SETUP_TVOUT)	+= setup-tvout.o
obj-$(CONFIG_S5PV310_SETUP_USB_PHY)	+= setup-usb.o
ifneq ($(CONFIG_SATA_AHCI_PLATFORM),)
obj-y					+= dev-ahci.o
endif
obj-$(CONFIG_SND_SOC)			+= dev-audio.o
obj-$(CONFIG_S3C64XX_DEV_SPI)		+= dev-spi.o
ifeq ($(CONFIG_S3C_DEV_ADC1),y)
obj-$(CONFIG_SAMSUNG_DEV_TS)		+= dev-ts1.o
endif
obj-$(CONFIG_S5PV310_DEV_PD)		+= dev-pd.o
obj-$(CONFIG_DEV_THERMAL)		+= tmu.o
obj-$(CONFIG_SEC_DEBUG)			+= sec_debug.o sec_getlog.o sec_gaf.o
obj-$(CONFIG_S5PV310_WATCHDOG_RESET)	+= c1-watchdog.o
obj-$(CONFIG_SEC_THERMISTOR)		+= sec_thermistor.o
obj-$(CONFIG_SAMSUNG_LTE)		+= lte-modem.o
obj-$(CONFIG_CDMA_LINK_DPRAM)           += board-c1-modem_ota.o setup-dpram.o
obj-$(CONFIG_UMTS_LINK_HSIC)            += board-p4note-modem.o
