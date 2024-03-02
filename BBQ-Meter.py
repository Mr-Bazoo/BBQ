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
OFF_TEMP = 40
MIN_TEMP = 50
MAX_TEMP = 500
FAN_LOW = 1
FAN_HIGH = 100
FAN_OFF = 0
FAN_MAX = 100
FAN_GAIN = float(FAN_HIGH - FAN_LOW) / float(MAX_TEMP - MIN_TEMP)
SETPOINT_TEMP = 100  # Je kunt hier een startwaarde instellen

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
CS_PIN = 25
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

    # Create an Image object
    image = Image.new("1", oled.size)

    # Create a drawing object
    draw = ImageDraw.Draw(image)

    if temperature is not None:
        # Display temperature and fan speed
        draw.text((0, 0), "Temp: {:.2f}C".format(temperature), fill="white")
    else:
        # Display a message when temperature is None
        draw.text((0, 0), "Error reading temperature", fill="white")

    # Display setpoint temperature
    draw.text((0, 20), "Setpoint: {:.2f}C".format(SETPOINT_TEMP), fill="white")

    draw.text((0, 40), "Fan Speed: {}".format(fan_speed), fill="white")

    # Paste the image onto the OLED display
    oled.display(image)

def handle_long_press():
    global press_duration, SETPOINT_TEMP
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
        # Je kunt extra shutdown-opdrachten toevoegen indien nodig
        # b.v., os.system("sudo shutdown -h now")
    else:
        # Hier kun je de temperatuur instellen met de rotary encoder
        # Pas het volgende deel aan om de temperatuur te wijzigen op basis van de rotary encoder
        SETPOINT_TEMP = round((position / 100.0) * (MAX_TEMP - MIN_TEMP) + MIN_TEMP, 2)
        print(f"Setpoint Temperature: {SETPOINT_TEMP}C")

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

        # Bij elke klik van de rotary encoder
        if a != a_last:
            if a == 0:
                # Controleer de draairichting
                direction = 1 if a == b else -1

                # Update de temperatuur op basis van de draairichting
                SETPOINT_TEMP += direction
                SETPOINT_TEMP = min(max(SETPOINT_TEMP, MIN_TEMP), MAX_TEMP)
                print(f"Setpoint Temperature: {SETPOINT_TEMP}C")

        # Lees de temperatuur en pas de ventilatorsnelheid aan
        temperature = get_temperature()
        if temperature is not None:
            if temperature > MIN_TEMP:
                delta = min(temperature, MAX_TEMP) - MIN_TEMP
                fan.start(FAN_LOW + delta * FAN_GAIN)
            elif temperature < OFF_TEMP:
                fan.start(FAN_OFF)
            else:
                delta = 0  # Of stel een standaardwaarde in of behandel None op een gepaste manier

        # Werk de ventilator duty cycle variabele bij
        current_duty_cycle = FAN_LOW + delta * FAN_GAIN if temperature is not None and temperature > MIN_TEMP else FAN_OFF

        # Weergave op OLED
        fan_speed = int(current_duty_cycle / FAN_MAX * 100)
        display_on_oled(temperature, fan_speed)

        sleep(WAIT_TIME)

        last_taster = taster
        a_last = a

except KeyboardInterrupt:
    print('\nScript eindigt!')

finally:
    fan.stop()
    GPIO.cleanup()
    oled.clear()
    oled.show()

