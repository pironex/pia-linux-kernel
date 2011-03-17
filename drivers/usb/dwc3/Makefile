ccflags-$(CONFIG_USB_DWC3_DEBUG)	:= -DDEBUG

obj-$(CONFIG_USB_DWC3)			+= dwc3.o
obj-$(CONFIG_USB_DWC3_OMAP)		+= dwc3-omap.o
obj-$(CONFIG_USB_DWC3_HAPS)		+= dwc3-haps.o

dwc3-y					:= core.o
dwc3-$(CONFIG_USB_GADGET_DWC3)		+= gadget.o ep0.o
