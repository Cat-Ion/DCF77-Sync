#ifndef USB_H
#define USB_H
#ifdef __cplusplus
extern "C" {
#endif
#include <libopencm3/usb/usbd.h>
usbd_device *usb_init(const usbd_driver *driver, const char *userserial);
void usb_run(usbd_device *usbd_dev);
#ifdef __cplusplus
}
#endif
#endif
