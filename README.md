PPM decoder for the JLX Overdrive RC car
=============

The [JLX Overdrive](http://jlxoverdrive.com) RC cars decoding circuit (U4 on the board) can be replaced with a [PIC12LF1822](http://ww1.microchip.com/downloads/en/DeviceDoc/41413C.pdf).
This firmware will let you drive the car with a standard [PPM](http://www.endurance-rc.com/ppmtut.php) transmitter. (Any 27Mhz AM transmitter should work.)
This will enable you to proportionally drive the wheels.

The code can be built with the [Microchip XC8 lite compiler](http://www.microchip.com/pagehandler/en-us/devtools/mplabxc/home.html).