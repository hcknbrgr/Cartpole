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
cartCenteredCounts = 0
poleVerticalCounts = 0
initializeDistanceData = []
initializeVerticalData = []
currentState = []
while cartCenteredCounts != 10:  # INITIALIZE CART POSITION
    line = arduino.readline()
    if line:
        string = line.decode()
        num = int(string)
        cm = (num / 2.0) / 29.1
        print(cm)
        if 7.0 < cm < 9.0:  # CART IS CENTERED, MAKE SURE IT STAYS THERE A FEW SECONDS
            cartCenteredCounts += 1
            if cartCenteredCounts == 10:   # IF IT'S BEEN CENTERED LONG ENOUGH THEN THE SIGNAL THAT IT'S GOOD
                arduino.write(struct.pack('>B', 3))
            else:
                arduino.write(struct.pack('>B', 1))
        else:
            arduino.write(struct.pack('B', 1))  # if it's not, send that's it's not...
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
runningEpisode = 0
while runningEpisode == 0:
    line = arduino.readline().strip()
    if line:
        string = line.decode().split('/')
        for a in string:
            currentState.append(float(a))

        runningEpisode=1



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
