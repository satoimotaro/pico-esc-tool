# pc_iface

Host ↔ controller command interface. Phase 1 transport: **USB-C CDC** (`Serial`).

Command set (draft, see `.ai/architecture/interfaces.md` §6):
`SCAN`, `ARM`/`DISARM`, `SET <ch> <throttle>`, `TELEM <ch>`,
`SETUP READ/WRITE <ch>`, `FLASH <ch> <hex>`. Binary framing for bulk (flash) transfers.

Later transports (BLE/WiFi/CAN) implement the same command set behind this module.

Status: stub. A0 uses a minimal inline parser (`T/C/E/A/D/?`) in `src/main.cpp`;
promote to a real command table here at A1.
