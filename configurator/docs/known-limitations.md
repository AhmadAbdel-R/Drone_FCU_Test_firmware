# Known Limitations

- USB configurator firmware is available only through the bench target `esp32-s3-mini-configurator`.
- BLE firmware/GATT, GPIO0 triple-click toggle, BLE LED override, and Windows Web Bluetooth transport are implemented, but still need hardware validation on the target FCU and Windows adapter.
- USB device selection uses a child-process serial bridge by default so COM-port refresh does not load native serial code inside NW.js.
- Deadman motor testing is implemented for the USB configurator path, but stop latency and rejection behavior still need hardware measurement.
- USB boot logs can appear before the binary protocol starts. Runtime log-router USB output is disabled in the configurator env.
- Portable NW.js packaging is implemented. Inno Setup installer and code signing scripts are present, but require local Inno Setup and a signing certificate.
- Firmware update command IDs and UI validation are wired, but the FCU deliberately returns `NOT_SUPPORTED` until a signed writer path is enabled.
- Performance metrics have not been captured on hardware for configurator USB load.
