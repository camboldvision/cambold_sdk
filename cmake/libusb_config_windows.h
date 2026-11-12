/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal libusb configuration used by the in-tree static MinGW build. */
#ifndef CB_LIBUSB_CONFIG_H
#define CB_LIBUSB_CONFIG_H

#define DEFAULT_VISIBILITY
#define ENABLE_LOGGING 0
#define PLATFORM_WINDOWS 1
#define PRINTF_FORMAT(a, b)

/* MinGW provides the POSIX-ish time declarations used by libusb's common
 * headers, while the backend itself uses WinUSB. */
#define HAVE_SYS_TIME_H 1

#endif
