#
# Makefile for the Omnivision TCM touchscreen driver.
#

# Each configuration option enables a list of files.

obj-y += omnivision_tcm_driver.o
omnivision_tcm_driver-objs += omnivision_tcm_core.o omnivision_tcm_spi.o omnivision_tcm_zeroflash.o omnivision_tcm_touch.o omnivision_tcm_device.o omnivision_tcm_testing.o omnivision_tcm_reflash.o omnivision_tcm_diagnostics.o

EXTRA_CFLAGS += -Iarch/arm/mach-hisi
EXTRA_CFLAGS += -Idrivers/hisi/tzdriver
EXTRA_CFLAGS += -Idrivers/tzdriver
EXTRA_CFLAGS += -Idrivers/devkit/lcdkit/core/common
