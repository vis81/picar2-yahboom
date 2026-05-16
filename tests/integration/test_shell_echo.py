#!/usr/bin/env python3
"""
Shell UART echo integrity test.

Sends a 60-character string to the Zephyr shell on USART3 one character at a
time (15 ms inter-character delay), then presses Enter and verifies that the
full string appears verbatim in the "command not found" error response.

This confirms that all 60 characters were correctly received by the MCU,
stored in cmd_buff, and echoed to the terminal.  The 15 ms per-character
delay exceeds the 10 ms DMA idle-flush timeout so each byte is independently
flushed and processed by the shell before the next arrives — matching the
timing of interactive terminal input.

Sending a burst (all bytes at once) causes a non-deterministic RX drop in
the DMA path at 921600 baud.  Verifying via the error-response (not via the
raw echo stream) avoids interference from motor-debug log output which the
shell interleaves with the echo on the TX side.

Usage:
    python3 tests/integration/test_shell_echo.py
    python3 tests/integration/test_shell_echo.py --port /dev/ttyYahboom1 --count 20
"""

import argparse
import sys
import time

import serial

PORT   = "/dev/ttyYahboom1"
BAUD   = 921600
PROMPT = b"uart:~$ "
CTRL_C = b"\x03"

# 60 printable chars — no control chars, no shell-special chars.
TEST_STR = b"abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ01234567"

# Per-character delay — must exceed the MCU DMA idle-flush timeout (10 ms).
CHAR_DELAY_S = 0.015

PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"


def wait_for_prompt(ser: serial.Serial, timeout: float = 3.0) -> bool:
    """Read bytes until PROMPT appears anywhere in the buffer."""
    deadline = time.monotonic() + timeout
    buf = b""
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if PROMPT in buf:
                return True
    return False


def echo_round(
    ser: serial.Serial, test_str: bytes, read_timeout: float = 5.0
) -> tuple[bool, str]:
    """
    Send test_str one character at a time, press Enter, then verify that the
    complete string appears in the shell's error response.

    The shell tries to execute the string as a command, fails with
    "<string>: command not found", and that response contains the exact string
    that cmd_buff held — proving all characters were correctly received.

    Returns (ok, detail_string).
    """
    ser.reset_input_buffer()

    for ch in test_str:
        ser.write(bytes([ch]))
        time.sleep(CHAR_DELAY_S)

    # Execute.  The shell will report "<test_str>: command not found".
    ser.write(b"\r")

    # Collect output until the next prompt appears.
    deadline = time.monotonic() + read_timeout
    buf = b""
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            if PROMPT in buf:
                break

    if PROMPT not in buf:
        return False, "timeout — shell did not return to prompt"

    if test_str in buf:
        return True, ""

    # Try to show what the shell actually executed for diagnostics.
    marker = b"not found"
    if marker in buf:
        # error line is between the last \n before "not found" and "not found"
        idx = buf.index(marker)
        line_start = buf.rfind(b"\n", 0, idx) + 1
        snippet = buf[line_start:idx + len(marker)]
        return False, f"string mismatch — shell saw: {snippet!r}"

    return False, f"unexpected response (no 'not found'): {buf!r}"


def main() -> None:
    parser = argparse.ArgumentParser(description="Shell UART echo integrity test")
    parser.add_argument("--port",  default=PORT,
                        help=f"Shell serial port (default: {PORT})")
    parser.add_argument("--baud",  type=int, default=BAUD)
    parser.add_argument("--count", type=int, default=10,
                        help="Number of echo rounds (default: 10)")
    args = parser.parse_args()

    total_time_s = args.count * len(TEST_STR) * CHAR_DELAY_S
    print(f"\nShell echo test  —  {args.port} @ {args.baud} baud  ×{args.count}")
    print(f"  test string ({len(TEST_STR)} chars): {TEST_STR.decode()}")
    print(f"  inter-char delay: {int(CHAR_DELAY_S * 1000)} ms  "
          f"(est. {total_time_s:.0f} s total)")
    print("=" * 62)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except serial.SerialException as e:
        print(f"Cannot open {args.port}: {e}")
        sys.exit(1)

    # Obtain a clean prompt before starting rounds.
    ser.write(CTRL_C)
    if not wait_for_prompt(ser, timeout=3.0):
        print("FATAL: timed out waiting for shell prompt — is the board running?")
        ser.close()
        sys.exit(1)

    passed = 0
    for i in range(1, args.count + 1):
        ok, detail = echo_round(ser, TEST_STR)
        tag = PASS if ok else FAIL
        suffix = f"  ({detail})" if detail else ""
        print(f"  [{tag}] round {i:2d}{suffix}")
        if ok:
            passed += 1

    ser.close()

    print(f"\n{'=' * 62}")
    colour = "\033[32m" if passed == args.count else "\033[31m"
    print(f"  {colour}{passed}/{args.count} rounds passed\033[0m")
    print(f"{'=' * 62}\n")
    sys.exit(0 if passed == args.count else 1)


if __name__ == "__main__":
    main()
