import time
from max6675class import MAX6675, MAX6675Error
cs_pin = 24
clock_pin = 23
data_pin = 22
units = "c"
degree_sign = u'\xb0'
print("[press ctrl+c to end the script]")  
try: # Main program loop
    while True:
        thermocouple = MAX6675(cs_pin, clock_pin, data_pin, units)
        time.sleep(2)
        print("Temperature\t{} {}C".format(thermocouple.get(), degree_sign))
except KeyboardInterrupt:
    print("Script end!")
    thermocouple.cleanup()