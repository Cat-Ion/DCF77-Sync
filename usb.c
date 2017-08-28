#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "ringbuffer.h"

/*
 * USB Vendor:Interface control requests.
 */
#define GZ_REQ_PRODUCE		2
#define GZ_REQ_SETPAT		3

/* USB configurations */
#define GZ_CFG_SOURCESINK	1
#define BULK_EP_MAXPACKET	64

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_VENDOR,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = BULK_EP_MAXPACKET,
    .idVendor = 0xcafe,
    .idProduct = 0xcafe,
    .bcdDevice = 0x0001,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};
static const struct usb_endpoint_descriptor endp_bulk[] = {
{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x81,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = BULK_EP_MAXPACKET,
    .bInterval = 5,
},
{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 20,
    .bInterval = 1,
},
};
static const struct usb_interface_descriptor iface_source[] = {
{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_VENDOR,
    .iInterface = 0,
    .endpoint = endp_bulk,
}
};
static const struct usb_interface ifaces_source[] = {
{
    .num_altsetting = 1,
    .altsetting = iface_source,
}
};
static const struct usb_config_descriptor config[] = {
{
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 1,
    .bConfigurationValue = GZ_CFG_SOURCESINK,
    .iConfiguration = 4, /* string index */
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,
    .interface = ifaces_source,
},
};

static char serial[] = "1";
static const char *usb_strings[] = {
    "imaginaerraum",
    "DCF77-Sync",
    serial,
    "PID and phase values",
};

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[5*BULK_EP_MAXPACKET];
static usbd_device *our_dev;

static void usb_bulk_in_cb(usbd_device *usbd_dev, uint8_t ep)
{
    uint8_t buf[BULK_EP_MAXPACKET];
    uint8_t *src;

    src = buf;

    unsigned overflow = ringbuffer_overflow();
    src[0] = overflow;
    src[1] = overflow >> 8;
    src[2] = overflow >> 16;
    src[3] = overflow >> 24;
    unsigned len = ringbuffer_get(src+4, BULK_EP_MAXPACKET-8);

    usbd_ep_write_packet(usbd_dev, ep, src, len+4);
}

static int usb_control_request(usbd_device *usbd_dev,
                               struct usb_setup_data *req,
                               uint8_t **buf,
                               uint16_t *len,
                               usbd_control_complete_callback *complete)
{
    (void) usbd_dev;
    (void) complete;
    (void) buf;
    (void) len;

    /* TODO - what do the return values mean again? */
    switch (req->bRequest) {
    case GZ_REQ_PRODUCE:
        if (req->wValue > sizeof(usbd_control_buffer)) {
            return USBD_REQ_NOTSUPP;
        }
        /* Don't produce more than asked for! */
        if (req->wValue > req->wLength) {
            *len = req->wLength;
        } else {
            *len = req->wValue;
        }
        return USBD_REQ_HANDLED;
    case GZ_REQ_SETPAT:
        return USBD_REQ_HANDLED;
    }
    return USBD_REQ_NEXT_CALLBACK;
}

static void usb_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    switch (wValue) {
    case GZ_CFG_SOURCESINK:
        usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
                      usb_bulk_in_cb);
        usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_INTERRUPT, BULK_EP_MAXPACKET,
                      0);
        usbd_register_control_callback(
                    usbd_dev,
                    USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE,
                    USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                    usb_control_request);
        /* Prime source for IN data. */
        usb_bulk_in_cb(usbd_dev, 0x81);
        break;
    default:
        break;
    }
}

usbd_device *usb_init(const usbd_driver *driver, const char *userserial)
{
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
    gpio_clear(GPIOA, GPIO12);
	for (unsigned int i = 0; i < 16000; i++) __asm__("nop");
    rcc_periph_clock_enable(RCC_OTGFS);
    if (userserial) {
        usb_strings[2] = userserial;
    }
    our_dev = usbd_init(driver, &dev, config,
                        usb_strings, 5,
                        usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(our_dev, usb_set_config);

    return our_dev;
}

void usb_run(usbd_device *usbd_dev)
{
    usbd_poll(usbd_dev);
}
