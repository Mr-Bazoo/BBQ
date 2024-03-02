import RPi.GPIO as GPIO
from time import sleep, time
from mx6675class import MAX6675, MAX6675Error
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106

# Constants
PWM_FREQ = 25
FAN_PIN = 18
WAIT_TIME = 1
OFF_TEMP = 40
MIN_TEMP = 45
MAX_TEMP = 70
FAN_LOW = 1
FAN_HIGH = 100
FAN_OFF = 0
FAN_MAX = 100
FAN_GAIN = float(FAN_HIGH - FAN_LOW) / float(MAX_TEMP - MIN_TEMP)

# Rotary Encoder Constants
SW_PIN = 5
DT_PIN = 6
CLK_PIN = 13

# OLED Display Constants
serial = i2c(port=1, address=0x3C)
oled = sh1106(serial, rotate=0)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize Fan PWM
GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.LOW)
fan = GPIO.PWM(FAN_PIN, PWM_FREQ)

# Initialize Rotary Encoder
GPIO.setup(CLK_PIN, GPIO.IN)
GPIO.setup(DT_PIN, GPIO.IN)

# Initialize MAX6675
CS_PIN = 24
CLOCK_PIN = 23
DATA_PIN = 22
units = "c"
thermocouple = MAX6675(CS_PIN, CLOCK_PIN, DATA_PIN, units)

# Global Variables
position = 0
a = 0
b = 0
a_last = 1
b_last = 1
taster = False 
last_taster = False
press_start_time = 0
press_duration = 0

# Function to get temperature
def get_temperature():
    try:
        return thermocouple.get()
    except MAX6675Error as e:
        print(f"Error reading temperature: {e.value}")
        return None

# Function to handle fan speed
def handle_fan_speed(temperature):
    if temperature is not None:
        if temperature > MIN_TEMP:
            delta = min(temperature, MAX_TEMP) - MIN_TEMP
            fan.start(FAN_LOW + delta * FAN_GAIN)
        elif temperature < OFF_TEMP:
            fan.start(FAN_OFF)

# Function to display on OLED
def display_on_oled(temperature, fan_speed):
    oled.clear()
    oled.text("Temp: {:.2f}C".format(temperature), 0, 0)
    oled.text("Fan Speed: {}".format(fan_speed), 0, 20)
    oled.show()

# Function to handle long press
def handle_long_press():
    global press_duration
    press_duration = time() - press_start_time
    if press_duration >= 5:  # If pressed for 5 seconds, initiate shutdown
        print("Shutting down...")
        oled.clear()
        oled.text("Shutting down...", 0, 0)
        oled.show()
        sleep(2)
        GPIO.cleanup()
        oled.clear()
        oled.show()
        # You can add additional shutdown commands if needed
        # e.g., os.system("sudo shutdown -h now")

# Main program loop
try:
    while True:
        a, b = GPIO.input(CLK_PIN), GPIO.input(DT_PIN)
        taster = not GPIO.input(SW_PIN)

        if taster != last_taster:
            if taster:
                press_start_time = time()
            else:
                handle_long_press()

            position = position % 101  # Ensure position stays within 0-100
            temp_setpoint = round((position / 100.0) * (MAX_TEMP - MIN_TEMP) + MIN_TEMP, 2)
            print(f'Setpoint Temperature: {temp_setpoint}°C')
            sleep(0.01)

        last_taster = taster

        # Read temperature and adjust fan speed
        temperature = get_temperature()
        handle_fan_speed(temperature)

        # Update the fan duty cycle variable
        current_duty_cycle = FAN_LOW + delta * FAN_GAIN if temperature > MIN_TEMP else FAN_OFF

        # Display on OLED
        fan_speed = int(current_duty_cycle / FAN_MAX * 100)
        display_on_oled(temperature, fan_speed)

        sleep(WAIT_TIME)

except KeyboardInterrupt:
    print('\nScript end!')

finally:
    fan.stop()
    GPIO.cleanup()
    oled.clear()
    oled.show()
