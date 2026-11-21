# OpenMV camera script: find the ball and send its center to the ESP32-S3 over UART.
# Protocol (what ROBOFUT_FINAL.ino parses): "(x,y)\n" at 115200 baud, QVGA 320x240.
# "(0,0)\n" means the ball is not in view.
import sensor
import time
from pyb import UART

BALL_LAB = [(30, 80, 15, 70, 20, 70)]  # orange ball in LAB color space; tune on the field

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240, matches CENTER_X/CENTER_Y in the firmware
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)            # fixed gain and white balance keep color tracking stable
sensor.set_auto_whitebal(False)

uart = UART(3, 115200)
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    blobs = img.find_blobs(BALL_LAB, pixels_threshold=40, area_threshold=40, merge=True)
    if blobs:
        ball = max(blobs, key=lambda b: b.pixels())
        img.draw_rectangle(ball.rect())
        img.draw_cross(ball.cx(), ball.cy())
        uart.write("(%d,%d)\n" % (ball.cx(), ball.cy()))
    else:
        uart.write("(0,0)\n")
