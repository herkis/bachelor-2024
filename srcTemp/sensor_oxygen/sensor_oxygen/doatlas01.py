try:
    import smbus
except:
    print('Try sudo apt-get install python3-smbus')
    
import time

from AtlasI2C import (
	 AtlasI2C
)


# Valid units
UNITS_mg_L = 1

# Wait time
delay_time = 1.5


def get_devices():
    device = AtlasI2C()
    device_address_list = device.list_i2c_devices()
    device_list = []
    
    for i in device_address_list:
        device.set_i2c_address(i)
        response = device.query("I")
        moduletype = response.split(",")[1] 
        response = device.query("name,?").split(",")[1]
        device_list.append(AtlasI2C(address = i, moduletype = moduletype, name = response))
    return device_list 

device_list = get_devices()

  
class DOATLAS(object):
    
    def __init__(self, bus=1):
        # mg/L
        self._oxygen = 0
        self._k = []
        
        try:
            self._bus = smbus.SMBus(bus)
        except:
            print("Bus %d is not available."%bus)
            print("Available busses are listed as /dev/i2c*")
            self._bus = None
          
    # def init(self):
    #     if self._bus is None:
    #         "No bus!"
    #         return False
        
    #     self._bus.write_byte(self._DOATLAS01_ADDR, self._DOATLAS01_RESET)
        
    #     # Wait for reset to complete
    #     sleep(0.1)
        
    #     self._k = []

    #     # Read calibration values
    #     # Read one 16 bit byte word at a time
    #     for prom in range(0xAA, 0xA2-2, -2):
    #         k = self._bus.read_word_data(self._TSYS01_ADDR, prom)
    #         k =  ((k & 0xFF) << 8) | (k >> 8) # SMBus is little-endian for word transfers, we need to swap MSB and LSB
    #         self._k.append(k)
            
    #     return True
        
    def read(self):
        if self._bus is None:
            print("No bus!")
            return False
        
        # Request conversion
        # self._bus.write_byte(self._TSYS01_ADDR, self._TSYS01_CONVERT)
        
        for dev in device_list:
            dev.write("R")
        # Wait time for the sensor to reach a value : at least 1.5s
            time.sleep(1.5)
            for dev in device_list:
                print(dev.read())
                
        # adc = self._bus.read_i2c_block_data(self._TSYS01_ADDR, self._TSYS01_READ, 3)
        # adc = adc[0] << 16 | adc[1] << 8 | adc[2]
        self._oxygen(dev.read())
        return True

    # Temperature in requested units
    # default degrees C
    def oxygen(self, conversion=UNITS_mg_L):                                                            # Change if other units wanted
        if conversion == 2:
            return (9/5) * self._oxygen + 32
        elif conversion == 3:
            return self._oxygen - 273
        return self._oxygen

    # # Cribbed from datasheet
    # def _calculate(self, adc):
    #     adc16 = adc/256
    #     self._temperature = -2 * self._k[4] * 10**-21 * adc16**4 + \
    #         4  * self._k[3] * 10**-16 * adc16**3 +                \
    #         -2 * self._k[2] * 10**-11 * adc16**2 +                \
    #         1  * self._k[1] * 10**-6  * adc16   +                 \
    #         -1.5 * self._k[0] * 10**-2