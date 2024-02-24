import time
from mx6675class import MAX6675, MAX6675Error
cs_pin = 24
clock_pin = 23
data_pin = 22
units = "c"
degree_sign = u'\xb0'
import RPi.GPIO as GPIO
import signal
import sys

#temperatuur variable
cs_pin = 24
clock_pin = 23
data_pin = 22
units = "c"

#PWM variable
PWM_FREQ = 25           # [Hz] PWM frequency
WAIT_TIME = 1           # [s] Time to wait between each refresh
OFF_TEMP = 40           # [°C] temperature below which to stop the fan
MIN_TEMP = 45           # [°C] temperature above which to start the fan
MAX_TEMP = 70           # [°C] temperature at which to operate at max fan speed
FAN_LOW = 1
FAN_HIGH = 100
FAN_OFF = 0
FAN_MAX = 100
FAN_GAIN = float(FAN_HIGH - FAN_LOW) / float(MAX_TEMP - MIN_TEMP)
t = time.time()
rpm = 0

# Pin configuration
TACH = 24       # Fan's tachometer output pin
PULSE = 2       # Noctua fans puts out two pluses per revolution
WAIT_TIME = 1   # [s] Time to wait between each refresh
FAN_PIN = 18    # BCM pin used to drive PWM fan

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TACH, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull up to 3.3V



def temperature(): 
    try: # Main program loop
        while True:
            thermocouple = MAX6675(cs_pin, clock_pin, data_pin, units)
            time.sleep(2)
            print("Temperature\t{} {}C".format(thermocouple.get(), degree_sign))
    except KeyboardInterrupt:
        print("Script end!")
        thermocouple.cleanup()

def handleFanSpeed(fan, temperature):
    if temperature > MIN_TEMP:
        delta = min(temperature, MAX_TEMP) - MIN_TEMP
        fan.start(FAN_LOW + delta * FAN_GAIN)

    elif temperature < OFF_TEMP:
        fan.start(FAN_OFF)


try:
    signal.signal(signal.SIGTERM, lambda *args: sys.exit(0))
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.LOW)
    fan = GPIO.PWM(FAN_PIN, PWM_FREQ)
    while True:
        handleFanSpeed(fan, getTemperature())
        time.sleep(WAIT_TIME)

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()

# Caculate pulse frequency and RPM
def fell(n):
    global t
    global rpm

    dt = time.time() - t
    if dt < 0.005:
        return  # Reject spuriously short pulses

    freq = 1 / dt
    rpm = (freq / PULSE) * 60
    t = time.time()


# Add event to detect
GPIO.add_event_detect(TACH, GPIO.FALLING, fell)

try:
    while True:
        print("%.f RPM" % rpm)
        rpm = 0
        time.sleep(1)   # Detect every second

except KeyboardInterrupt:   # trap a CTRL+C keyboard interrupt
    GPIO.cleanup()          # resets all GPIO ports used by this function