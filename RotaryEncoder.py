import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

SW_PIN = 5
DT_PIN = 6
CLK_PIN = 13

GPIO.setup(CLK_PIN, GPIO.IN)
GPIO.setup(DT_PIN, GPIO.IN)
GPIO.setup(SW_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

position = 0
a = 0
b = 0
a_last = 1
b_last = 1
taster = False 
last_taster = False  # Corrected the typo here

print('[Press CTRL + C to end the script!]')

try:
    # Main program loop
    while True:
        a, b = GPIO.input(CLK_PIN), GPIO.input(DT_PIN)
        taster = not GPIO.input(SW_PIN)

        if taster != last_taster:
            print('{}|{}'.format(position, taster))
            sleep(0.01)
        
        last_taster = taster

        if a != a_last or b != b_last:
            if a == 0 and b == 1:
                while not (a == 1 and b == 1):
                    a = GPIO.input(CLK_PIN)
                    b = GPIO.input(DT_PIN)
                    position += 1
                    print('{}|{}'.format(position, taster))

            if a == 1 and b == 0:
                while not (a == 1 and b == 1):
                    a = GPIO.input(CLK_PIN)
                    b = GPIO.input(DT_PIN)
                    position -= 1
                    print('{}|{}'.format(position, taster))

        a_last = a
        b_last = b

except KeyboardInterrupt:
    print('\nScript end!')
finally:
    GPIO.cleanup()
