# turing-tunes
Digital guitar FX processor. Generates trippy audio-reactive organic visuals ([Turing patterns!](https://visualpde.com/visual-stories/turing-morphogenesis.html) by running a reaction-diffusion simulation. Also fits on 2 credit cards.

### Parts
<p>- PCB, manufacturing files [here](pcb) <img align="center" height="90" alt="screencap_kicad" src="https://github.com/user-attachments/assets/40edcba1-787c-4db7-a4d3-bc827588cc30"> <p/>
<p>- ST7735 - LCD display (SPI 128x160) <img align="center" height="100" alt="Subject" src="https://github.com/user-attachments/assets/f29ef42a-52c1-469d-a7eb-4b10f357b1b5"><p/>
<p>- ESP32-S3-WROOM - signal processing, lcd driver <img align="center" height="80" alt="IMG_2622" src="https://github.com/user-attachments/assets/6e0118fd-2739-4b50-9670-155551c593d6" />
<p/>

- WM8960 - audio codec (i2s)
- Limit switch - footswitch
- 6.35mm audio jacks x2 - amplifier send and receive
- Analog [joystick](code/stick.h)(2D)
- Caps on power rails (opt.)

### Hardware Setup
[Schematic diagram](schematics)
A board costs about $CAD in materials as of 2026.
| ------------- | ------------- |
| IC modules  | CAD  |
| Passives  | CAD  |
| JLCPCB  | CAD  |

### Software Setup
The code should be built and flashed with ESP-IDF, resulting in a binary ~333kb in size.
```
$ git clone https://github.com/seanleeeeeeeeee/turing-tunes/
cd code
source ~/esp/esp-idf/export.sh          //or equivalent
idf.py set-target esp32s3
idf.py build flash monitor
```

### Known Issues
- WM8960 registers inconsistently send the i2c acknowledge bit. the program must stall and retry several times when initializing these, but only at startup. in addition, master volume changes take delayed effect.
- Signal-to-noise ratio is mediocre (shielding?)
