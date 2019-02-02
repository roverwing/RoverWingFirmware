# RoverWing Firmware

This repository contains the firmware (both source and pre-built binaries) and necessary support files for the RoverWing board. 
For more info about RoverWing, please visit Roverwing main website: ????

Since RoverWing comes with the firmware pre-installed, most users do not need any of the files in this repository. 
You only need to use it if you want to understand the inner workings of the firmware, or to create your own modified 
version of the firmware. Note: the authors of the firmware provide no technical support for modifying the firmware; 
do it at your own risk. 


## Updating the firmware using pre-built binaries

Pre-built firmware binaries are contained in folder `binaries`. To install a new version of the firmware, please follow these steps:
1. Download the prebuilt firmware binary to your computer. The file must should have extension .uf2, indicating that it can be used with UF2 bootloader (which is what we use on RoverWing). You can find more info abut the UF2 bootloader in [this guide from Adafruit](https://learn.adafruit.com/adafruit-feather-m0-express-designed-for-circuit-python-circuitpython/uf2-bootloader-details). 
2. Connect your RoverWing board to a computer using microUSB cable. You can use Windows (Windows 10 recommended), Mac, or Linux computer. If you are using WIndows 7 or earlier, you might need to install additional drivers; see the Adafruit guide referenced above for details. 
3. Power your RoverWing board by connecting a 7-14V power source. (RoverWing can not be powered via USB cable.)
4. Double-click reset button on the RoverWing. This should put  RoverWing board in bootloader mode, and it will appear as a new removable drive on your computer, with the name **HALLOWBOOT**. If you open this drive, you will see that it contains 3 files, CURRENT.UF2, INDEX.HTM, and INFO_UF2.TXT. The file CURRENT.UF2 is the current firmware. 

**FIXME**: we really need to modify the bootloader, so that it doesn't use HALLOWBOOT but instead displays ROVERBOOT or something similar. 

5. Drag the firmware .uf2 file you had downloaded into the HALLOWBOOT folder. You might get a warning form the OS that the file is copied without its properties; just click "YES".

After the file is copied, the RoverWing should restart automatically, the HALLOWBOOT drive disappears from your computer screen, and your RoverWing is ready for use!


## Building the firmware from source
This information is for advanced users only. Use at your own risk!!

To build the firmware from the source, you need to have Arduino IDE installed on your computer (version 1.8 or later). You also need to install the the board definition files. The easiest way to do it is to reuse the board definitions provided by Adafruit for their Crickit board, just changing some files, as follows:

1. Install Adafruit's board support package for SAMD-based boards, as described [here](https://learn.adafruit.com/adafruit-feather-m0-basic-proto/setup). Please use version 1.2.9, **even if later versions are available** (most likely, using later versions would also be OK, but it was not tested.)

2. Find the installed package files.  To do this, first find the folder with Arduino configuration data;  depending on your OS and version of Arduino IDE, it can be either `<username>\AppData\Local\Arduino15` (Windows), `<username>\Documents\ArduinoData\` (Windows 10, using Arduino IDE installed from Windows store), or `/home/<user>/.arduino15/` (Linux). 

    Once you found the Arduino configuration data folder, navigate to `\packages\adafruit\hardware\samd\1.2.9\variants\crickit_m0`.

3. Download two files `variant.cpp` and `variant.h` from this repository and use them to replace the corresponding files in `crickit_m0` folder. 

4. Restart the Arduino IDE and select `Adafruti Crickit M0` in *Tools->Board* menu. 

You are now ready to build and upload new firmware from source. Download the `src` folder from this repository as a zip file, unpack, rename the folder to `roverwing-firmware`, and move it to Arduino sketchbook folder. Now find in that folder file  `roverwing-firmware.ino` and open it in Arduino IDE. Edit is as you like and upload to the board in the usual way. 




