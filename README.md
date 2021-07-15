# NRF52 firmware for uPlant

Requires https://github.com/ultimaterobotics/urf_lib for compiling

Firmware of the uPlant device which reads soil moisture data, lighting conditions, temperature and transmits everything as BLE advertising events. Data intended to be received by ESP32 based central node which provides Telegram interface - although it's possible to write a phone app for that purpose as well
