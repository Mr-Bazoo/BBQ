import RPi.GPIO as GPIO
from time import sleep
from mx6675class import MAX6675, MAX6675Error
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from PIL import Image, ImageDraw
from rotary import Rotary
import time

# Constants
PWM_FREQ = 24
FAN_PIN = 18
WAIT_TIME = 1
OFF_TEMP = 10
MIN_TEMP = 30
MAX_TEMP = 500
FAN_LOW = 50
FAN_HIGH = 100
FAN_OFF = 0
FAN_MAX = 100
SETPOINT_TEMP = 100  # Startwaarde instellen
FAN_GAIN = float(FAN_HIGH - FAN_LOW) / float(MAX_TEMP - MIN_TEMP)  # Fan speed gain based on temperature difference

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
GPIO.setup(DT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(CLK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize Fan PWM
GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.LOW)
fan = GPIO.PWM(FAN_PIN, PWM_FREQ)

# Initialize MAX6675
CS_PIN = 25
CLOCK_PIN = 23
DATA_PIN = 22
units = "c"
thermocouple = MAX6675(CS_PIN, CLOCK_PIN, DATA_PIN, units)

# Constants for fan speed calculation
MIN_PWM_DUTY_CYCLE = 0  # Minimum PWM duty cycle
MIN_FAN_RPM = 0  # Minimum fan RPM (corresponding to minimum PWM duty cycle)
MAX_PWM_DUTY_CYCLE = 100  # Maximum PWM duty cycle
MAX_FAN_RPM = 1120  # Maximum fan RPM (corresponding to maximum PWM duty cycle)

# Function to get temperature
def get_temperature():
    try:
        return thermocouple.get()
    except MAX6675Error as e:
        print(f"Error reading temperature: {e.value}")
        return None

# Calculate fan speed in RPM based on PWM duty cycle
def calculate_fan_speed(fan_speed_percent):
    duty_cycle_range = MAX_PWM_DUTY_CYCLE - MIN_PWM_DUTY_CYCLE
    rpm_range = MAX_FAN_RPM - MIN_FAN_RPM
    pwm_duty_cycle = MIN_PWM_DUTY_CYCLE + (fan_speed_percent / 100) * duty_cycle_range
    fan_speed_rpm = MIN_FAN_RPM + (pwm_duty_cycle - MIN_PWM_DUTY_CYCLE) / duty_cycle_range * rpm_range
    return fan_speed_rpm

# Function to handle fan speed
def handle_fan_speed(temperature):
    if temperature is not None:
        if temperature > MIN_TEMP:
            delta = min(temperature, MAX_TEMP) - MIN_TEMP
            fan_speed_percent = FAN_LOW + delta * FAN_GAIN
            fan_speed_percent = min(FAN_MAX, fan_speed_percent)  # Limit fan speed to FAN_MAX
        elif temperature < (SETPOINT_TEMP - 10):
            fan_speed_percent = FAN_LOW  # Start fan at minimum speed if temperature is too low
        else:
            fan_speed_percent = FAN_OFF
    else:
        fan_speed_percent = FAN_OFF

    fan_speed_rpm = calculate_fan_speed(fan_speed_percent)
    fan.start(fan_speed_percent)
    return fan_speed_percent, fan_speed_rpm


# Function to display temperature, fan speed, and setpoint temperature on OLED
def display_on_oled(temperature, fan_speed_rpm, setpoint_temp):
    # Create an Image object
    image = Image.new("1", oled.size)

    # Create a drawing object
    draw = ImageDraw.Draw(image)

    # Clear the area where the numbers will be displayed
    draw.rectangle((0, 0, oled.width, oled.height), fill="black")

    # Display actual temperature
    if temperature is not None:
        draw.text((0, 0), "Temp: {:.2f}C".format(temperature), fill="white")

    # Display setpoint temperature
    draw.text((0, 20), "Setpoint: {:.2f}C".format(setpoint_temp), fill="white")

    # Display fan speed in RPM
    draw.text((0, 40), "Fan Speed: {:.2f} RPM".format(fan_speed_rpm), fill="white")

    # Paste the updated area onto the OLED display
    oled.display(image)

# Function to handle long button press
def handle_long_press():
    press_duration = time.time() - press_start_time
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

# Callback function for rotary encoder
def rotary_changed(change):
    global SETPOINT_TEMP
    if change == Rotary.ROT_CW:
        SETPOINT_TEMP += 10  # Increase temperature setpoint when turned right
    elif change == Rotary.ROT_CCW:
        SETPOINT_TEMP -= 10  # Decrease temperature setpoint when turned left
    elif change == Rotary.SW_PRESS:
        print('PRESS')
    elif change == Rotary.SW_RELEASE:
        print('RELEASE')

# Initialize rotary encoder
rotary = Rotary(CLK_PIN, DT_PIN, SW_PIN)
rotary.add_handler(rotary_changed)

# Main program loop
try:
    while True:
        # Read temperature
        temperature = get_temperature()

        # Adjust fan speed
        fan_speed_percent, fan_speed_rpm = handle_fan_speed(temperature)

        # Display on OLED
        display_on_oled(temperature, fan_speed_rpm, SETPOINT_TEMP)

        sleep(WAIT_TIME)

except KeyboardInterrupt:
    print('\nScript eindigt!')

finally:
    fan.stop()
    GPIO.cleanup()
    oled.clear()