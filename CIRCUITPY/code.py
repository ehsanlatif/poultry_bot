"""
Board Pins

code.py output:
board.A0 board.D12 board.SPEAKER
board.A1 board.D6 board.SCK
board.A2 board.D9 board.MISO
board.A3 board.D10 board.MOSI
board.A4 board.D3 board.SCL
board.A5 board.D2 board.SDA
board.A6 board.D0 board.RX
board.A7 board.D1 board.TX
board.A8 board.LIGHT
board.A9 board.TEMPERATURE
board.ACCELEROMETER_INTERRUPT
board.ACCELEROMETER_SCL
board.ACCELEROMETER_SDA
board.BUTTON_A board.D4
board.BUTTON_B board.D5
board.D13
board.D7 board.SLIDE_SWITCH
board.D8 board.NEOPIXEL
board.IR_PROXIMITY
board.IR_RX board.REMOTEIN
board.IR_TX board.REMOTEOUT
board.MICROPHONE_CLOCK
board.MICROPHONE_DATA
board.SPEAKER_ENABLE
"""
import time
#from adafruit_circuitplayground import cp
from adafruit_circuitplayground.express import cpx
import touchio
import audiobusio
import array
import simpleio
import board
import math


cpx.pixels.auto_write = False
#cp.pixels.brightness = 0.3
cpx.pixels.brightness = 0.2
touch = touchio.TouchIn(board.A1)

DRY_VALUE = 1500  # calibrate this by hand!
WET_VALUE = 2100  # calibrate this by hand!

NUM_SAMPLES = 160


def normalized_rms(values):
    minbuf = int(mean(values))
    samples_sum = sum(
        float(sample - minbuf) * (sample - minbuf)
        for sample in values
    )

    return math.sqrt(samples_sum / len(values))


def mean(values):
    return sum(values) / len(values)
mic = audiobusio.PDMIn(board.MICROPHONE_CLOCK, board.MICROPHONE_DATA,
                       sample_rate=16000, bit_depth=16)
samples = array.array('H', [0] * NUM_SAMPLES)
mic.record(samples, len(samples))
# Set lowest level to expect, plus a little.
input_floor = normalized_rms(samples) + 10


def scale_range(value):
    """Scale a value from 0-320 (light range) to 0-9 (NeoPixel range).
    Allows remapping light value to pixel position."""
    return round(value / 320 * 9)


while True:
    peak = scale_range(cpx.light)
    print("light:",cpx.light)
    print("temperature:",cpx.temperature * 1.8 + 32)
    X = 0
    Y = 0
    Z = 0
    for count in range(10):
        x,y,z = cpx.acceleration
        X = X + x
        Y = Y + y
        Z = Z + z
        time.sleep(0.01)
    X = X / 10
    Y = Y / 10
    Z = Z / 10
    print("acceleration:",x, y, z)
    XYangle = (math.atan2(-X,Y))*57.3
    YZangle = (math.atan2(-Y,Z))*57.3
    ZXangle = (math.atan2(-Z,X))*57.3
    print("angle:",XYangle,YZangle,ZXangle)
    value_A1 = touch.raw_value
    # fill the pixels from red to green based on soil moisture
    humidity = int(simpleio.map_range(value_A1, DRY_VALUE, WET_VALUE, 0, 100))
    print("humidity:", humidity)
    mic.record(samples, len(samples))
    magnitude = normalized_rms(samples)
    print("Sound level:", magnitude)

    for i in range(10):
        if i <= peak:
            cpx.pixels[i] = (50,50 , 50)
        else:
            cpx.pixels[i] = (0, 0, 0)
    cpx.pixels.show()
    time.sleep(0.4)