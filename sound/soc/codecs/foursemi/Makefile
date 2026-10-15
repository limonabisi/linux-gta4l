# SPDX-License-Identifier: GPL-2.0
snd-soc-fs18xx-y := fsm_core.o fs1801.o fsm_i2c.o
obj-$(CONFIG_SND_SOC_FS18XX) += snd-soc-fs18xx.o
ccflags-y += -DCONFIG_FSM_CODEC -DCONFIG_FSM_FIRMWARE -DCONFIG_FSM_I2C \
	     -DCONFIG_FSM_STEREO -DCONFIG_FSM_FS1801
ccflags-y += -Wno-missing-prototypes -Wno-missing-declarations -Wno-old-style-declaration -Wno-unused-function
