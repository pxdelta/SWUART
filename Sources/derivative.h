#ifndef DERIVATIVE_H
#define DERIVATIVE_H

#include <intrinsics.h>
#include <iostm8s105c6.h>

/* IWDG_KR */
#define KEY_ENABLE  0xCC
#define KEY_REFRESH 0xAA
#define KEY_ACCESS  0x55

#define __RESET_WATCHDOG()  IWDG_KR=KEY_REFRESH

#endif
