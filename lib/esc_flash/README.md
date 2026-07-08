# esc_flash  [core, hard]

BLHeli-S **1-wire bootloader** flashing over the signal wire — load firmware `.hex`
(stock BLHeli-S/BlueJay, later our BlueGill) without external programmer.

Sequence: enter bootloader → identify device (SiLabs EFM8 BB2x) → erase pages →
write pages → verify → run. See `.ai/architecture/interfaces.md` §4.

Reference to port: **esc-configurator** (JS) bootloader/flash modules; BLHeliSuite.

Plan (Phase A0 spike → A1): connect + device-ID first, then full erase/program/verify.

Status: stub. This + `esc_setup` are the highest-risk items — do the spike early.
