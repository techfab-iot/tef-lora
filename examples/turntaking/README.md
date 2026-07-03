# SX1262 Turn-Taking Example

This example targets `tef::boards::meshlink_gateway::v0_5_0` and the SX1262
pinout used by the Meshlink Gateway v0.5.0 hardware.

The firmware has a single role:

1. Send a packet.
2. Switch to receiver mode.
3. Listen for a fixed window.
4. Wait for a randomized backoff before sending again.

That cycle makes it possible to flash the same firmware to multiple devices and
have them exchange packets with fewer TX collisions.

The payload includes the device MAC address, the persisted boot count stored in
NVS, and a per-boot message counter.
