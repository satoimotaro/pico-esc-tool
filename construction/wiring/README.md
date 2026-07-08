# Wiring — bring-up (A0)

Minimal single-ESC bench setup for the LittleBee Spring 30A + a Pico.

| ESC lead | Connects to | Notes |
|----------|-------------|-------|
| Signal   | Pico GPIO `SIGNAL_PIN` (default GP10) | bidirectional DShot; keep leads short |
| Ground   | Pico GND **and** power-supply GND | common ground is mandatory |
| Battery + / − | ESC power pads → bench PSU / battery | **do not** power the motor from the Pico |
| Motor A/B/C | 3 motor phases | swap any two to reverse (or set direction in `esc_setup`) |

Safety:
- Bench PSU with current limit for first spin; props/impellers off.
- Pico is powered over USB from the PC (separate from motor power); grounds common.
- ESC BEC (if present): do **not** back-feed the Pico unless you know the rail is safe.

TODO: add a diagram; document telemetry-UART pad if used (KISS telem, §3).
