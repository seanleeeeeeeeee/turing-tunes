
### Parts
<p>• PCB, manufacturing files [here](pcb) <img align="center" height="110" alt="screencap_kicad" src="https://github.com/user-attachments/assets/40edcba1-787c-4db7-a4d3-bc827588cc30"> <p/>
- ST7735 - LCD display (SPI 128x160) <img align="center" height="100" alt="Subject" src="https://github.com/user-attachments/assets/f29ef42a-52c1-469d-a7eb-4b10f357b1b5"><p/>
- ESP32-S3-WROOM - signal processing, lcd driver
- WM8960 - audio codec (i2s)
- Limit switch - footswitch
- 6.35mm audio jacks x2 - amplifier send and receive
- Analog [joystick](code/stick.h)(2D)
- Caps on power rails (opt.)

### Hardware Setup
A board costs about $52 in materials as of 2026.
Pin assignments are in main.cpp, lcd.cpp, and stick.cpp.
