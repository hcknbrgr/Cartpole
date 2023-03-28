from pyfirmata import Arduino, util
import time


board = Arduino('COM7')
print("communication successfully started")
it = util.Iterator(board)
it.start()

''' 
    Ultrasonic sensor pins - 
    VCC +5VDC
    Trig: Trigger - Input Pin 11
    Echo: Echo Output - Output pin 12
    Gnd: Gnd
'''
trigPin = 11
echoPin = 12

while True:
    board.digital[trigPin].write(0)
    time.sleep(0.000005)   # sleep for 5 microseconds to before sending ultrasonic burst
    board.digital[trigPin].write(1)  # send the burst
    time.sleep(0.000010)  # sleep for 10 microseconds, then turn off to determine rebound of sound
    board.digital[trigPin].write(0)   # turn off the burst

    duration = board.digital[echoPin].read()   # read the value it took for the response
    cm = (duration/2.0) / 29.1  # calculate the distance in cm

    