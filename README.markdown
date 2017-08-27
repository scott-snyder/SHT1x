SHT1x Temperature / Humidity Sensor Library for Arduino
=======================================================
Copyright 2017 Vincent Pang wingshun.pang@gmail.com  
Copyright 2009 Jonathan Oxer jon@oxer.com.au / http://www.practicalarduino.com  
Copyright 2008 Maurice Ribble ribblem@yahoo.com / http://www.glacialwanderer.com

Provides a simple interface to the SHT1x series (SHT10, SHT11, SHT15)
and SHT7x series (SHT71, SHT75) temperature / humidity sensors from
Sensirion, http://www.sensirion.com. These sensors use a "2-wire"
communications buss that is similar to I2C and can co-exist on the same
physical wire as I2C devices.

Installation
------------
Download the directory "SHT1x" and move it into the "libraries"
directory inside your sketchbook directory, then restart the Arduino
IDE. You will then see it listed under File->Examples->SHT1x.

Usage
-----
The library is instantiated as an object with methods provided to read
relative humidity and temperature. Include it in your sketch and then
create an object, specifying the pins to use for communication with the
sensor:

    #include <SHT1x.h>
    
    #define dataPin 10
    #define clockPin 13
    
    // default to 5.0v boards, e.g. Arduino UNO
    SHT1x sht1x(dataPin, clockPin);
    
    // if 3.3v board is used
    //SHT1x sht1x(dataPin, clockPin, SHT1x::Voltage::DC_3_3v);

You can then call methods on that object within your program. In this
example we created an object called "sht1x", but it could have been
called whatever you like. A complete example program is included with
the library and can be accessed from the File->Examples->SHT1x menu.

### readTemperatureC() ###

Returns a float within the valid range of the sensor of -40 to +123.8C.

Example:

    float tempC = sht1x.readTemperatureC();

### readTemperatureF() ###

Returns a float within the valid range of the sensor of -40 to +254.9F.

Example:

    float tempF = sht1x.readTemperatureF();

### readHumidity() ###

Returns a float within the valid range of the sensor of 0 to 100%.

Example:

    float humidity = sht1x.readHumidity();

More Info
-----
### Coefficients ###
This library is updated with the latest coefficients according to the datasheet v5.  
https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Sensirion_Humidity_Sensors_SHT1x_Datasheet_V5.pdf

The coefficients are different slightly between each stated voltages. Only the values for 5v, 4v, 3.5v, 3v and 2.5v are stated in the datasheet. The values for the common 3.3v are missing. So the values used for 3.3v in this library are calculated by linear interpolation.

### 11ms start up time ###
From the datasheet, "After power-up the sensor needs 11ms to get to Sleep State. No commands must be sent before that time."

### Maximum one measurement per second ###
From the datasheet, "To keep self heating below 0.1°C, SHT1x should not be active for more than 10% of the time – e.g. maximum one measurement per second at 12bit accuracy shall be made."

### Measurement blocking time, around 320ms ###
From the datasheet, "This takes a maximum of 20/80/320 ms for a 8/12/14bit measurement. The time varies with the speed of the internal oscillator and can be lower by up to 30%."

### Settings used in this library ###
In case you are interested in the settings used by this library, here you are.
- 14 bit resolution (max) is used for temperature measurement.
- 12 bit resolution (max) is used for humidity measurement.
- No crc checking is done after each reading from sensor.
- Datatype "double" is used in the internal calculation in order to preserve precision. And "float" is used in final result.
