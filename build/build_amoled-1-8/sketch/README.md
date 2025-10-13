#line 1 "E:\\DataJPL\\arduino\\arduino_maker\\companion\\README.md"
# Project Title

companion_camera.ino,  October 2025.  JP Lavoie
Square Line Studio UI 1.4.3 using LVGL 8.4.0

## Features
Display a jpg image via HTTPS GET request (latest image, new, back)
Forwarded port on pfsense.  Synology to do a reverse proxy to a Flask server on Dockerpi
to avoid exposing Node-Red to the internet.  Only use defined end-points on the Flask server, encryption
with HTTPS and use of a security token to make the image requests for added security.
Connects to local wifi or iPhone hotspot and report battery voltage on MQTT.
Node-Red will send a latest image taken MQTT message to display a new image on the module.

Display the power (kW) and energy (kWh) of the house from Hilo on screen 1.

Deep sleep consumption: 4 mA.  Wake up from touch screen
Running awake: 27 mA (80 MHz clock)
Shutdown with AXP2101: 1 mA.  Wake up from power module side button
Go into shutdown also if boards becomes stationary

##  HARDWARE
- MCU: ESP32-S3R8 Dual-core LX7
- ESP32 on board 8 MB Flash et 8 MB PSRAM and external 16MB Flash
- CST816 Touch chip driver library (FT3168 self-Capacitance Touch Controller)
communicating through I2C
- AMOLED Display Driver: SH8601, Résolution 368×448, 16.7M colors
communicating through QSPI
- High-definition display: FT3168
- IMU: QMI8658
- RTC: PCF85063
- Low Power Mono Audio CODEC ES8311
- PCF85063 RTC chip connected to the battery via the AXP2101 for uninterrupted power supply, and a 
backup battery pad is reserved to ensure that the RTC function continues to work during main battery replacement
3.7V MX1.25 lithium battery recharge/discharge header
- TF card slot
https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8


## ARDUINO IDE SETUP
- Board: Waveshare ESP32-S3-Touch-AMOLED-1.8
- Boards manager: ESP32 Espressif 3.0.7, 3.1.3  
3.2.0 & 3.3.0: not working, i2c: CONFLICT! driver_ng is not allowed to be used with this old driver
Board keeps resetting.
- PSRAM: OPI (8MB)
- Partion Scheme: 16M Flash (3MB app, 9.9MB FATFS)
- USB CDC on Boot: enabled
- CPU frequency: 80 MHz wifi to reduce power consumption


## Libraries Used

| Library | Version | Path |
|---------|---------|------|
| Arduino_DriveBus | 1.0.1 | C:\Users\photo\Documents\Arduino\libraries\Arduino_DriveBus |
| ESP32_IO_Expander | 0.0.3 | C:\Users\photo\AppData\Local\Arduino15\internal\ESP32_IO_Expander_0.0.3_486fe45ba20089a0\ESP32_IO_Expander |
| GFX Library for Arduino | 1.4.9 | C:\Users\photo\Documents\Arduino\libraries\GFX_Library_for_Arduino |
| PubSubClient | 2.8 | C:\Users\photo\AppData\Local\Arduino15\internal\PubSubClient_2.8_48867b22d3bf7501\PubSubClient |
| SensorLib | 0.3.1 | C:\Users\photo\AppData\Local\Arduino15\internal\SensorLib_0.3.1_f1d18563defb1310\SensorLib |
| TJpg_Decoder | 1.1.0 | C:\Users\photo\AppData\Local\Arduino15\internal\TJpg_Decoder_1.1.0_3837ce1b1aac2443\TJpg_Decoder |
| XPowersLib | 0.2.6 | C:\Users\photo\AppData\Local\Arduino15\internal\XPowersLib_0.2.6_fb7691a308ca517a\XPowersLib |
| lvgl | 8.4.0 | C:\Users\photo\AppData\Local\Arduino15\internal\lvgl_8.4.0_18f8734bf9323e2e\lvgl |
| Wire | 3.1.3 | C:\Users\photo\AppData\Local\Arduino15\internal\esp32_esp32_3.1.3_e149c3cd368ed269\libraries\Wire |
| SPI | 3.1.3 | C:\Users\photo\AppData\Local\Arduino15\internal\esp32_esp32_3.1.3_e149c3cd368ed269\libraries\SPI |
| WiFi | 3.1.3 | C:\Users\photo\AppData\Local\Arduino15\internal\esp32_esp32_3.1.3_e149c3cd368ed269\libraries\WiFi |
| Networking | 3.1.3 | C:\Users\photo\AppData\Local\Arduino15\internal\esp32_esp32_3.1.3_e149c3cd368ed269\libraries\Network |
| NetworkClientSecure | 3.1.3 | C:\Users\photo\AppData\Local\Arduino15\internal\esp32_esp32_3.1.3_e149c3cd368ed269\libraries\NetworkClientSecure |
| HTTPClient | 3.1.3 | C:\Users\photo\AppData\Local\Arduino15\internal\esp32_esp32_3.1.3_e149c3cd368ed269\libraries\HTTPClient |
| FS | 3.1.3 | C:\Users\photo\AppData\Local\Arduino15\internal\esp32_esp32_3.1.3_e149c3cd368ed269\libraries\FS |
| LittleFS | 3.1.3 | C:\Users\photo\AppData\Local\Arduino15\internal\esp32_esp32_3.1.3_e149c3cd368ed269\libraries\LittleFS |
| SPIFFS | 3.1.3 | C:\Users\photo\AppData\Local\Arduino15\internal\esp32_esp32_3.1.3_e149c3cd368ed269\libraries\SPIFFS |
| SD | 3.1.3 | C:\Users\photo\AppData\Local\Arduino15\internal\esp32_esp32_3.1.3_e149c3cd368ed269\libraries\SD |