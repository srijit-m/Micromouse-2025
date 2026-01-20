"""
This file is provided as a sample of basic initialisation and working for
"plug-and-play" of the drivers, but is expected to be altered to implement
system control algorithms.
"""
from micromouse import Micromouse
from machine import Pin, Timer
import time

mm = Micromouse()


if __name__ == "__main__":
    mm.led_red_set(1)
    mm.led_green_set(0)

    while True:
        # mm.move_forward(180)
        # time.sleep_ms(200)
        mm.turn(90)
        time.sleep_ms(250)

    # mm.move_forward_encoders(1000)

    # # time.sleep(0.5)

    # # mm.move_forward(-138)
    # mm.turn(360)

    # mm.move_forward_encoders(100)

    # #### TEST ENCODERS
    # mm.reset_encoders()
    # time.sleep(1)
    # mm.led_red_set(1)

    # time.sleep(15)

    # e1 = mm.encoder_1_counts()
    # e2 = mm.encoder_2_counts()
    # d1 = mm.encoder_1_distance()
    # d2 = mm.encoder_2_distance()
    # print(f"{e1=}, {e2=}")
    # print(f"{d1=}, {d2=}")
    # ####

    mm.led_green_set(1)
    mm.led_red_set(0)
