/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Compatibility shim for Zephyr input API differences between versions.
 * Include this after <zephyr/input/input.h> in files that need it.
 */

#ifndef ZEPHYR_INCLUDE_INPUT_COMPAT_H_
#define ZEPHYR_INCLUDE_INPUT_COMPAT_H_

/* Renamed INPUT_LISTENER_CB_DEFINE → INPUT_CALLBACK_DEFINE after Zephyr 3.4 */
#ifndef INPUT_CALLBACK_DEFINE
#define INPUT_CALLBACK_DEFINE INPUT_LISTENER_CB_DEFINE
#endif

/* Input event codes added after Zephyr 3.4 */
#ifndef INPUT_REL_WHEEL
#define INPUT_REL_WHEEL  0x08
#endif
#ifndef INPUT_ABS_RUDDER
#define INPUT_ABS_RUDDER 0x07
#endif
#ifndef INPUT_ABS_GAS
#define INPUT_ABS_GAS    0x09
#endif
#ifndef INPUT_ABS_BRAKE
#define INPUT_ABS_BRAKE  0x0a
#endif

#endif /* ZEPHYR_INCLUDE_INPUT_COMPAT_H_ */
