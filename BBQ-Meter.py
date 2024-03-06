from pyky040 import pyky040
import RPi.GPIO as GPIO
from time import sleep, time
from mx6675class import MAX6675, MAX6675Error
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.render import canvas
from PIL import Image, ImageDraw

# Constants
PWM_FREQ = 25
FAN_PIN = 18
WAIT_TIME = 1
OFF_TEMP = 20
MIN_TEMP = 50
MAX_TEMP = 500
FAN_LOW = 1
FAN_HIGH = 100
FAN_OFF = 0
FAN_MAX = 100
FAN_GAIN = float(FAN_HIGH - FAN_LOW) / float(MAX_TEMP - MIN_TEMP)
SETPOINT_TEMP = 100  # Startwaarde instellen

# Rotary Encoder
SW_PIN = 5
DT_PIN = 6
CLK_PIN = 13

# OLED Display Constants
serial = i2c(port=1, address=0x3C)
oled = sh1106(serial, rotate=0)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialize Fan PWM
GPIO.setup(FAN_PIN, GPIO.OUT, initial=GPIO.LOW)
fan = GPIO.PWM(FAN_PIN, PWM_FREQ)

# Initialize MAX6675
CS_PIN = 25
CLOCK_PIN = 23
DATA_PIN = 22
units = "c"
thermocouple = MAX6675(CS_PIN, CLOCK_PIN, DATA_PIN, units)

# Initialize Telegram bot
TELEGRAM_BOT_TOKEN = "7170337296:AAEy930irkZ_829_KGCmMd2jOn9WGgrgOMQ"
updater = Updater(token=TELEGRAM_BOT_TOKEN, use_context=True)
dispatcher = updater.dispatcher

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

# Function to update setpoint temperature based on rotary encoder
def update_setpoint(scale_position):
    global SETPOINT_TEMP
    SETPOINT_TEMP = min(max(scale_position, MIN_TEMP), MAX_TEMP)

# Function to display on OLED
# Function to display on OLED
def display_on_oled(temperature, fan_speed, setpoint_temp):
    oled.clear()

    # Create an Image object
    image = Image.new("1", oled.size)

    # Create a drawing object
    draw = ImageDraw.Draw(image)

    # Display actual temperature
    if temperature is not None:
        draw.text((0, 0), "Temp: {:.2f}C".format(temperature), fill="white")
    else:
        draw.text((0, 0), "Error reading temperature", fill="white")

    # Display setpoint temperature
    draw.text((0, 20), "Setpoint: {:.2f}C".format(setpoint_temp), fill="white")

    # Display fan speed
    draw.text((0, 40), "Fan Speed: {}%".format(fan_speed), fill="white")

    # Paste the image onto the OLED display
    oled.display(image)

    # Print debugging message
    print("Display updated on OLED")

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

# Callback function for pyky040
def my_callback(scale_position):
    update_setpoint(scale_position)

# Init the encoder pins with pyky040
my_encoder = pyky040.Encoder(CLK=CLK_PIN, DT=DT_PIN, SW=SW_PIN)
my_encoder.setup(scale_min=MIN_TEMP, scale_max=MAX_TEMP, step=1, chg_callback=my_callback)
my_encoder.watch()

# Function to display on OLED
def display_on_oled(temperature, fan_speed, setpoint_temp):
    oled.clear()

    # Create an Image object
    image = Image.new("1", oled.size)

    # Create a drawing object
    draw = ImageDraw.Draw(image)

    # Display actual temperature
    if temperature is not None:
        draw.text((0, 0), "Temp: {:.2f}C".format(temperature), fill="white")
    else:
        draw.text((0, 0), "Error reading temperature", fill="white")

    # Display setpoint temperature
    draw.text((0, 20), "Setpoint: {:.2f}C".format(setpoint_temp), fill="white")

    # Display fan speed
    draw.text((0, 40), "Fan Speed: {}%".format(fan_speed), fill="white")

    # Paste the image onto the OLED display
    oled.display(image)

# Main program loop
try:
    while True:
        taster = not GPIO.input(SW_PIN)

        if taster != last_taster:
            if taster:
                press_start_time = time()
            else:
                handle_long_press()

        # Read temperature and adjust fan speed
        temperature = get_temperature()
        handle_fan_speed(temperature)

        # Update fan speed duty cycle variable
        current_duty_cycle = (
            FAN_LOW + (temperature - MIN_TEMP) * FAN_GAIN
            if temperature is not None and temperature > MIN_TEMP
            else FAN_OFF
        )

        # Display on OLED
        fan_speed = int(current_duty_cycle / FAN_MAX * 100)
        display_on_oled(temperature, fan_speed, SETPOINT_TEMP)

        sleep(WAIT_TIME)

        last_taster = taster

except KeyboardInterrupt:
    print('\nScript eindigt!')

finally:
    fan.stop()
    GPIO.cleanup()
    oled.clear()
    oled.show()
