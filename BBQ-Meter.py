import RPi.GPIO as GPIO
from time import sleep
from mx6675class import MAX6675, MAX6675Error
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106
from luma.core.render import canvas
from PIL import Image, ImageDraw
from telegram.ext import Updater, CommandHandler, MessageHandler, filters

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
SETPOINT_TEMP = 100
DEBOUNCE_TIME = 0.05

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

# Function to display on OLED
def display_on_oled(temperature, fan_speed):
    oled.clear()
    image = Image.new("1", oled.size)
    draw = ImageDraw.Draw(image)

    if temperature is not None:
        draw.text((0, 0), "Temp: {:.2f}C".format(temperature), fill="white")
    else:
        draw.text((0, 0), "Error reading temperature", fill="white")

    draw.text((0, 20), "Setpoint: {:.2f}C".format(SETPOINT_TEMP), fill="white")
    draw.text((0, 40), "Fan Speed: {}".format(fan_speed), fill="white")

    oled.display(image)

# Function to get fan speed from duty cycle
def fan_speed_from_duty_cycle():
    current_duty_cycle = fan.dutyCycle
    fan_speed = int(current_duty_cycle / FAN_MAX * 100)
    return fan_speed

# Command handlers for Telegram
def start(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Bot started. Use /setpoint to change setpoint_temp.")

def setpoint(update, context):
    if context.args:
        try:
            new_setpoint = float(context.args[0])
            global SETPOINT_TEMP
            SETPOINT_TEMP = new_setpoint
            context.bot.send_message(chat_id=update.effective_chat.id, text=f"Setpoint temperature changed to {SETPOINT_TEMP}C.")
        except ValueError:
            context.bot.send_message(chat_id=update.effective_chat.id, text="Invalid input. Please provide a valid temperature.")
    else:
        context.bot.send_message(chat_id=update.effective_chat.id, text="Please provide a temperature to set as the new setpoint.")

def get_data(update, context):
    temperature = get_temperature()
    fan_speed = fan_speed_from_duty_cycle()
    context.bot.send_message(chat_id=update.effective_chat.id, text=f"Temperature: {temperature}C\nFan Speed: {fan_speed}%\nSetpoint Temperature: {SETPOINT_TEMP}C")

# Command handlers registration
start_handler = CommandHandler('start', start)
dispatcher.add_handler(start_handler)

setpoint_handler = CommandHandler('setpoint', setpoint, pass_args=True)
dispatcher.add_handler(setpoint_handler)

get_data_handler = CommandHandler('geefmedata', get_data)
dispatcher.add_handler(get_data_handler)

# Start polling for Telegram messages
updater.start_polling()

# Main program loop
try:
    while True:
        temperature = get_temperature()

        if temperature is not None:
            handle_fan_speed(temperature)
            fan_speed = fan_speed_from_duty_cycle()
            display_on_oled(temperature, fan_speed)

        sleep(WAIT_TIME)

except KeyboardInterrupt:
    print('\nScript ends!')

finally:
    fan.stop()
    GPIO.cleanup()
    oled.clear()
    oled.show()
    updater.stop()
