/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal libusb configuration used by the in-tree static build. */
#ifndef CB_LIBUSB_CONFIG_H
#define CB_LIBUSB_CONFIG_H

#define DEFAULT_VISIBILITY __attribute__ ((visibility ("default")))
#define ENABLE_LOGGING 0
#define HAVE_SYS_TIME_H 1
#define HAVE_NFDS_T 1
#define PLATFORM_POSIX 1
#define PRINTF_FORMAT(a, b) __attribute__ ((__format__ (__printf__, a, b)))
#define _GNU_SOURCE 1

#if defined(__linux__)
# define HAVE_CLOCK_GETTIME 1
# define HAVE_EVENTFD 1
# define HAVE_TIMERFD 1
# define HAVE_PIPE2 1
# define HAVE_PTHREAD_CONDATTR_SETCLOCK 1
#endif

#if defined(__APPLE__)
# define HAVE_PTHREAD_THREADID_NP 1
# define HAVE_IOKIT_USB_IOUSBHOSTFAMILYDEFINITIONS_H 1
#endif

#endif
