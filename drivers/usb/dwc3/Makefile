ccflags-$(CONFIG_USB_DWC3_DEBUG)	:= -DDEBUG
ccflags-$(CONFIG_USB_DWC3_VERBOSE)	+= -DVERBOSE_DEBUG

obj-$(CONFIG_USB_DWC3)			+= dwc3.o

dwc3-y					:= core.o

ifneq ($(CONFIG_USB_DWC3_GADGET),)
	dwc3-y				+= gadget.o ep0.o
endif

ifneq ($(CONFIG_ARCH_OMAP5),)
	obj-$(CONFIG_USB_DWC3)		+= dwc3-omap.o
endif

ifneq ($(CONFIG_PCI),)
	obj-$(CONFIG_USB_DWC3)		+= dwc3-pci.o
endif

ifneq ($(CONFIG_DEBUG_FS),)
	dwc3-y				+= debugfs.o
endif
