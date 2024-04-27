import time
from mx6675class import MAX6675, MAX6675Error
cs_pin = 25
clock_pin = 24
data_pin = 22
units = "c"
degree_sign = u'\xb0'

def temperature(): 
    try: # Main program loop
        while True:
            thermocouple = MAX6675(cs_pin, clock_pin, data_pin, units)
            time.sleep(2)
            print("Temperature\t{} {}C".format(thermocouple.get(), degree_sign))
    except KeyboardInterrupt:
        print("Script end!")
        thermocouple.cleanup()