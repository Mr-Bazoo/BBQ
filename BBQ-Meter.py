import RPi.GPIO as GPIO
from time import sleep
from mx6675class import MAX6675, MAX6675Error
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from PIL import Image, ImageDraw
from rotary import Rotary
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import time
import os

# InfluxDB configuration
bucket = "BBQ_Meter"
org = "BBQ_Meter"
token = os.environ.get("INFLUXDB_TOKEN")  # Make sure to set this environment variable
url = "http://192.168.1.160:8086"  # Update this with your InfluxDB URL

# Create InfluxDB client
client = InfluxDBClient(url=url, token=token, org=org)
write_api = client.write_api(write_options=SYNCHRONOUS)

# Constants
PWM_FREQ = 24
FAN_PIN = 18
WAIT_TIME = 1
OFF_TEMP = 20
MIN_TEMP = 30
MAX_TEMP = 500
FAN_LOW = 1
FAN_HIGH = 100
FAN_OFF = 0
FAN_MAX = 100
SETPOINT_TEMP = 100
TARGET_FAN_RPM = 500
TEMP_DIFF_THRESHOLD = 10
FAN_MAX_SPEED_PERCENT = 100

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
MIN_PWM_DUTY_CYCLE = 0
MIN_FAN_RPM = 0
MAX_PWM_DUTY_CYCLE = 100
MAX_FAN_RPM = 1120

# Function to read temperature
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
        if temperature > SETPOINT_TEMP:
            fan_speed_percent = FAN_OFF
        else:
            temp_diff = SETPOINT_TEMP - temperature
            if temp_diff > TEMP_DIFF_THRESHOLD:
                fan_speed_percent = FAN_MAX_SPEED_PERCENT
            else:
                fan_speed_percent = (TARGET_FAN_RPM / MAX_FAN_RPM) * 100
    else:
        fan_speed_percent = FAN_OFF

    fan_speed_rpm = calculate_fan_speed(fan_speed_percent)
    fan.start(fan_speed_percent)
    return fan_speed_percent, fan_speed_rpm

# Function to display on OLED
def display_on_oled(temperature, fan_speed_rpm, setpoint_temp):
    image = Image.new("1", oled.size)
    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, oled.width, oled.height), fill="black")
    
    if temperature is not None:
        draw.text((0, 0), "Temp: {:.2f}C".format(temperature), fill="white")
    draw.text((0, 20), "Setpoint: {:.2f}C".format(setpoint_temp), fill="white")
    draw.text((0, 40), "Fan Speed: {:.2f} RPM".format(fan_speed_rpm), fill="white")
    
    oled.display(image)

# Function to handle long button press
def handle_long_press(press_start_time):
    press_duration = time.time() - press_start_time
    if press_duration >= 5:
        print("Shutting down...")
        oled.clear()
        oled.text("Shutting down...", 0, 0)
        oled.show()
        sleep(2)
        
        GPIO.cleanup()
        oled.clear()
        oled.show()
        
        os.system("sudo shutdown -h now")

# Callback function for rotary encoder
def rotary_changed(change):
    global SETPOINT_TEMP
    if change == Rotary.ROT_CW:
        SETPOINT_TEMP += 10
    elif change == Rotary.ROT_CCW:
        SETPOINT_TEMP -= 10
    elif change == Rotary.SW_PRESS:
        print('PRESS')
    elif change == Rotary.SW_RELEASE:
        print('RELEASE')

# Function to send data to InfluxDB
def write_to_influxdb(temperature, fan_speed_rpm):
    point = Point("temperature_and_fan_speed") \
        .tag("device", "BBQ_meter") \
        .field("temperature", temperature) \
        .field("fan_speed_rpm", fan_speed_rpm)
    
    write_api.write(bucket=bucket, org=org, record=point)

# Initialize rotary encoder
rotary = Rotary(CLK_PIN, DT_PIN, SW_PIN)
rotary.add_handler(rotary_changed)

# Main program loop
try:
    while True:
        temperature = get_temperature()
        fan_speed_percent, fan_speed_rpm = handle_fan_speed(temperature)
        display_on_oled(temperature, fan_speed_rpm, SETPOINT_TEMP)
        write_to_influxdb(temperature, fan_speed_rpm)
        sleep(WAIT_TIME)
except KeyboardInterrupt:
    print('\nScript ended!')
finally:
    fan.stop()
    GPIO.cleanup()
    oled.clear()
