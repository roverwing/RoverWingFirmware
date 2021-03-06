# RoverWing Firmware

This repository contains the firmware (both source and pre-built binaries) and
necessary support files for the RoverWing board. For more info about RoverWing,
please visit [RoverWing User Guide](https://roverwing-board.readthedocs.io/)

Since RoverWing comes with the firmware pre-installed, most users do not need
any of the files in this repository. You only need to use it if you want to
understand the inner workings of the firmware, or to create your own modified
version of the firmware. Note: the authors of the firmware provide no technical
support for modifying the firmware; do it at your own risk.


## Updating the firmware using pre-built binaries

Pre-built firmware binaries are contained in folder `binaries`. To install a new version of the firmware, please follow these steps:
1. Download the prebuilt firmware binary from `binaries` folder in this
repository to your computer. The file must should have extension .uf2,
indicating that it can be used with UF2 bootloader (which is what we use on
RoverWing). You can find more info abut the UF2 bootloader in [this guide from
Adafruit](https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/uf2-bootloader-details).
2. Connect your RoverWing board to a computer using microUSB cable. You can use
Windows (Windows 10 recommended), Mac, or Linux computer. If you are using
WIndows 7 or earlier, you might need to install additional drivers; see the
Adafruit guide referenced above for details.
3. Power your RoverWing board by connecting a 7-14V power source. (RoverWing can
not be powered via USB cable.)
4. Double-click reset button on the RoverWing. This should put  RoverWing board
in bootloader mode, and it will appear as a new removable drive on your
computer, with the name **ROVERWING**. If you open this drive, you will see that
it contains three files, CURRENT.UF2, INDEX.HTM, and INFO_UF2.TXT. The file
CURRENT.UF2 is the current firmware.

5. Drag the firmware .uf2 file you had downloaded into the ROVERWING folder. You
might get a warning from the OS that the file is copied without its properties;
just click "YES".

After the file is copied, the RoverWing should restart automatically, the
ROVERWING drive disappears from your computer screen, and your RoverWing is
ready for use!


## Building the firmware from source
This information is for advanced users only. Use at your own risk!!

It is assumed that you have some experience with Arduino, so the instructions
are brief. This is intentional, to discourage inexperienced users.

To build the firmware from the source, you need the following software installed
on your computer:
1.  Arduino IDE (version 1.8 or later).

2. Board definition files -- see instructions below

3. Required libraries:

   1. NeoGPS
   2. Adafruit_NeoPixel
   3. Adafruit_Zero_DMA_Library
   4. Adafruit_DMA_neopixel_library
   5. FlashStorage

   All of these libraries can be installed using library manager built into Arduino IDE

4. After installing these libraries, you need to modify the the file Adafruit_NeoPixel_ZeroDMA.cpp in Adafruit_DMA_neopixel_library, adding the following lines:
```C
#elif defined(ADAFRUIT_CRICKIT_M0)
  &sercom0, SERCOM0, SERCOM0_DMAC_ID_TX,   33,   10,  34, SPI_PAD_2_SCK_3, SERCOM_RX_PAD_1, PIO_SERCOM,
```
immediately before the line
```C
#elif defined(__SAMD51__) // Metro M4
```

5. After completing the steps above,  restart the Arduino IDE and select `Adafruti Crickit M0` in *Tools->Board* menu.

You are now ready to build and upload new firmware from source. Download the
`src` folder from this repository as a zip file, unpack, rename the folder to
`roverwing-firmware`, and move it to Arduino sketchbook folder. Now find in that
folder file  `roverwing-firmware.ino` and open it in Arduino IDE. Edit is as you
like and upload to the board in the usual way.

### Board definition files
For step 2 above, you need to install the board definition files for the
RoverWing board.  The easiest way to do it is to reuse the board definitions
provided by Adafruit for their Crickit board, just changing some files, as
follows:

1. Install Adafruit's board support package for SAMD-based boards, as described
[here](https://learn.adafruit.com/adafruit-feather-m0-basic-proto/setup). Please
use version 1.5.13, **even if later versions are available** (most likely, using
later versions would also be OK, but it was not tested.)

2. Find the installed package files.  To do this, first find the folder with
Arduino configuration data;  depending on your OS and version of Arduino IDE, it
can be either `<username>\AppData\Local\Arduino15` (Windows),
`<username>\Documents\ArduinoData\` (Windows 10, using Arduino IDE installed
from Windows store), or `/home/<user>/.arduino15/` (Linux).

    Once you found the Arduino configuration data folder, navigate to `\packages\adafruit\hardware\samd\1.5.13\variants\crickit_m0`.

3. Download two files `variant.cpp` and `variant.h` from `boardDefinitions`
folder this repository and use them to replace the corresponding files in
`crickit_m0` folder.

## Related Documents

[Roverwing Github page](https://github.com/roverwing/)

[RoverWing User guide](https://roverwing-board.readthedocs.io/)  

[RoverWing Library](https://roverwing-library.readthedocs.io/)


## License
RoverWing firmware is distributed under the terms of GNU General Public
License version 3.0. Full text of the license is given in the LICENSE file in
this repository.
