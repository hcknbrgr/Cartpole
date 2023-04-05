import serial
import time
import struct  # for sending data as bytes
import matplotlib.pyplot as plt

# TODO: INITIALIZE CART DISTANCE
# TODO: INITIALIZE VERTICAL ON SENSOR
# TODO: MARK START OF EPISODE -- TURN ON THE BOARD LIGHT
# TODO: MARK END OF EPISODE
# TODO: IMPLEMENT REINFORCEMENT ALGORITHM

# Test the cart distance for initialization.
arduino = serial.Serial('COM7', 115200, write_timeout=0.5, timeout=.5)
time.sleep(2)
cartCenteredCounts = 0
data = []
while cartCenteredCounts >= 100:
    line = arduino.readline()
    if line:
        string = line.decode()
        num = int(string)
        cm = (num / 2.0) / 29.1
        if num < 8.0:
            arduino.write(struct.pack('>B', 2))
            cartCenteredCounts = 0
        elif num > 9.0:
            arduino.write(struct.pack('>B', 0))
            cartCenteredCounts = 0
        else:
            cartCenteredCounts += 1
            if cartCenteredCounts >= 100:
                arduino.write(struct.pack('>B', 3))
            else:
                arduino.write(struct.pack('>B', 1))
        data.append(num)

arduino.close()

plt.plot(data)
plt.xlabel('Time')
plt.ylabel('Distance')
plt.title('Time vs Distance')
plt.show()
