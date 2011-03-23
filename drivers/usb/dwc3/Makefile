ccflags-$(CONFIG_USB_DWC3_DEBUG)	:= -DDEBUG

obj-$(CONFIG_USB_DWC3)			+= dwc3.o
obj-$(CONFIG_USB_DWC3_OMAP)		+= dwc3-omap.o
obj-$(CONFIG_USB_DWC3_HAPS)		+= dwc3-haps.o

dwc3-y					:= core.o
ifneq ($(CONFIG_USB_DWC3_GADGET),)
dwc3-y					+= gadget.o ep0.o
endif
