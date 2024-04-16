import RPi.GPIO as GPIO
import time

class Rotary:

    ROT_CW = 1
    ROT_CCW = 2
    SW_PRESS = 4
    SW_RELEASE = 8

    def __init__(self, dt, clk, sw):
        self.dt_pin = dt
        self.clk_pin = clk
        self.sw_pin = sw
        self.last_status = (GPIO.input(self.dt_pin) << 1) | GPIO.input(self.clk_pin)
        GPIO.setup(self.dt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.clk_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.sw_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.dt_pin, GPIO.BOTH, callback=self.rotary_change)
        GPIO.add_event_detect(self.clk_pin, GPIO.BOTH, callback=self.rotary_change)
        GPIO.add_event_detect(self.sw_pin, GPIO.BOTH, callback=self.switch_detect)
        self.handlers = []
        self.last_button_status = GPIO.input(self.sw_pin)

    def rotary_change(self, pin):
        new_status = (GPIO.input(self.dt_pin) << 1) | GPIO.input(self.clk_pin)
        if new_status == self.last_status:
            return
        transition = (self.last_status << 2) | new_status
        if transition == 0b1110:
            self.call_handlers(Rotary.ROT_CW)
        elif transition == 0b1101:
            self.call_handlers(Rotary.ROT_CCW)
        self.last_status = new_status

    def switch_detect(self, pin):
        if GPIO.input(self.sw_pin):
            self.call_handlers(Rotary.SW_RELEASE)
        else:
            self.call_handlers(Rotary.SW_PRESS)

    def add_handler(self, handler):
        self.handlers.append(handler)

    def call_handlers(self, event_type):
        for handler in self.handlers:
            handler(event_type)


# Usage example
if __name__ == "__main__":
    # Define GPIO pins
    DT_PIN = 6  # Example GPIO pin for DT (data) of rotary encoder
    CLK_PIN = 13  # Example GPIO pin for CLK (clock) of rotary encoder
    SW_PIN = 5  # Example GPIO pin for switch of rotary encoder

    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)

    # Initialize Rotary object
    rotary = Rotary(DT_PIN, CLK_PIN, SW_PIN)

    # Define a handler function to respond to rotary encoder events
    def rotary_handler(event_type):
        if event_type == Rotary.ROT_CW:
            print("Rotary encoder rotated clockwise")
        elif event_type == Rotary.ROT_CCW:
            print("Rotary encoder rotated counterclockwise")
        elif event_type == Rotary.SW_PRESS:
            print("Rotary encoder switch pressed")
        elif event_type == Rotary.SW_RELEASE:
            print("Rotary encoder switch released")

    # Add the handler function to the Rotary object
    rotary.add_handler(rotary_handler)

    try:
        # Keep the program running to handle events
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        # Clean up GPIO before exiting
        GPIO.cleanup()