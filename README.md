## MMC5883
Driver for the MMC5883 magnetometer sensor

## Information
This code is a simple driver intended to be used in a flight computer. The code was implemented using references:
* MEMSIC MMC5883MA Rev.C datasheet

## Example usage:
```
>>import time
>>import MMC5883

>>sensor = mmc5883(0x30)
>>sensor.calibrate()
>>while True:
>>      print(sensor.get_mag())
>>      time.sleep(0.01)
[-1.35, -7.275, -32.425000000000004]
[-1.4000000000000001, -7.2250000000000005, -32.300000000000004]
[-1.35, -7.2, -32.300000000000004]
[-1.5, -7.275, -32.325]
[-1.35, -7.2250000000000005, -32.325]
[-1.35, -7.2, -32.475]
[-1.4000000000000001, -7.325, -32.425000000000004]
[-1.375, -7.25, -32.300000000000004]...
```
