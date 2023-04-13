import serial
import time
import struct  # for sending data as bytes
import matplotlib.pyplot as plt

# TODO: INITIALIZE CART DISTANCE
# TODO: INITIALIZE VERTICAL ON SENSOR
# TODO: MARK START OF EPISODE -- TURN ON THE BOARD LIGHT
# TODO: MARK END OF EPISODE
# TODO: IMPLEMENT REINFORCEMENT ALGORITHM

# Cart Directions -
# 0 = move towards sensor
# 1 = stationary
# 2 = move away from sensor

# Test the cart distance for initialization.
arduino = serial.Serial('COM7', 115200, write_timeout=0.5, timeout=.5)
time.sleep(2)
cartCenteredCounts = 20  # TODO: CHANGE THIS TO 0 .. TESTING MPU
poleVerticalCounts = 0
initializeDistanceData = []
initializeVerticalData = []
while cartCenteredCounts <= 10:  # INITIALIZE CART POSITION
    line = arduino.readline()
    if line:
        string = line.decode()
        num = int(string)
        cm = (num / 2.0) / 29.1
        print(cm)
        if cm < 7.0 or cm > 1200:  # TOO CLOSE, MOVE AWAY FROM WALL
            arduino.write(struct.pack('>B', 2))
            cartCenteredCounts = 0
        elif cm > 9.0:  # TOO FAR, MOVE CLOSER TO THE WALL
            arduino.write(struct.pack('>B', 0))
            cartCenteredCounts = 0
        else:  # CART IS CENTERED, LET'S MAKE SURE IT'S STAYING STABLE
            cartCenteredCounts += 1
            if cartCenteredCounts >= 100:   # IF IT'S BEEN CENTERED LONG ENOUGH THEN THE SIGNAL THAT IT'S GOOD
                arduino.write(struct.pack('>B', 3))
            else:   #  MAKE SURE THE CART IS STAYING CENTERED
                arduino.write(struct.pack('>B', 1))
        initializeDistanceData.append(cm)
#  END CART INITIALIZED POSITION on board LED should turn off
#  INITIALIZE THE ACCELEROMETER POSITION
while poleVerticalCounts < 20:
    line = arduino.readline()
    if line:
        string = line.decode()
        num = int(string)
        print(num)
        initializeVerticalData.append(num)
        if num < -86:  # if it's vertical then get ready to go
            poleVerticalCounts += 1
            arduino.write(struct.pack('>B', 0))
arduino.write(struct.pack('>B', 1))  # cart has been vertical for a total of 3 read cycles

#  CART IS COMPLETELY INITIALIZED, DO AN EPISODE!  The on-board LED should turn on.



arduino.close()

plt.plot(initializeDistanceData)
plt.xlabel('Time')
plt.ylabel('Distance')
plt.title('Time vs Distance')
plt.show()

plt.plot(initializeVerticalData)
plt.xlabel('Time')
plt.ylabel('Angle')
plt.title('Time vs Angle')
plt.show()
