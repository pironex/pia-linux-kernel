ccflags-$(CONFIG_USB_DWC3_DEBUG)	:= -DDEBUG

obj-$(CONFIG_USB_DWC3)			+= dwc3.o
obj-$(CONFIG_PCI)			+= dwc3-haps.o

dwc3-y					:= core.o

ifneq ($(CONFIG_USB_DWC3_GADGET),)
	dwc3-y				+= gadget.o ep0.o
endif
