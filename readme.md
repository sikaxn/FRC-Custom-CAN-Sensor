## IronMaple ESP32‐FRC‐devkit sponsored by JLCPCB

If you are looking for IronMaple ESP32‐FRC‐devkit resources, please switch branch to dev-board.

Code, CAD models: https://github.com/sikaxn/FRC-Custom-CAN-Sensor/tree/dev-board/pcb

Manual: https://github.com/sikaxn/FRC-Custom-CAN-Sensor/wiki/800.-IronMaple-ESP32%E2%80%90FRC%E2%80%90devkit-Manual

Design file: https://oshwlab.com/sikaxn/seal-v2-dev-board

Factory test code: https://studenttechsupport.com/customcanespfw/factory/

## Firmware tbd fix

### Addressable LED

1. Allow change number of pixel over CAN
2. offline detection on roborio driver

### Battery Tracking

1. Indicator Task that use IM dev board RGB LED
2. Better CAN protocol - Allow writing energy form roboRIO by user
3. Better CAN protocol - Allow user to trigger tag rescan over CAN
4. Better CAN protocol - More status feedback from ESP32 (tag write error)
5. Better CAN protocol - Empty message upon startup 
6. Allow change CAN Device Number using web tool
7. APP improvment - init new serial number follow BEST protocol (or disable this in setting)
8. APP improvment - App should not be set as default NFC app on Android

### IM ESP Devkit demo 

1. Bugfix - Change CAN Device Number using web tool currently not working.
2. Maybe merge Addressable LED with it.

### Battery Tracking + addressable LED combined firmware

1. New firmware that combine those two together.